#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <TinyGPS++.h>
#include <Preferences.h>
#include <ESP_I2S.h>
#include <math.h>

// --- Testing Override ---
// Set to 'false' when you go outside so the GPS fallback to LoRa actually works.
// Leave as 'true' while testing in your house so ESP-NOW audio stays active.
bool indoorTestingMode = true; 

// --- Hardware Definitions ---
Adafruit_SH1106G display(128, 64, &Wire, -1);
TinyGPSPlus gps;
Preferences preferences;
I2SClass i2s;

// LoRa Pins (SPI bus)
const int loraCsPin = 5;      
const int loraResetPin = 14;  
const int loraDio0Pin = 2;    

// Safe Button Pins 
const int DOT_PIN = 4;  
const int DASH_PIN = 13; 

// I2S Amplifier Pins (MAX98357A) & Mic Pin (MAX9814)
const int I2S_DIN_PIN  = 25;
const int I2S_BCLK_PIN = 26;
const int I2S_LRC_PIN  = 27;
const int micPin       = 34;        

int micCenter = 1482; // Auto-calibrated in setup()

// --- Human Voice Band-Pass Filter State (300Hz - 3400Hz @ 8kHz) ---
float prevRawCentered = 0.0f;
float prevHpOut       = 0.0f;
float prevLpOut       = 0.0f;
const float ALPHA_HP = 0.8093f;
const float ALPHA_LP = 0.7276f;
const float VOICE_GATE_THRESHOLD = 45.0f;

// --- Acoustic Loop & Echo Suppressor ---
volatile unsigned long lastRxAudioTime = 0;
const unsigned long ECHO_HOLD_MS = 180;     // Suppresses room echo tail after speaker stops
const float LOOP_DUCK_MULTIPLIER = 4.5f;    // Raises mic gate 4.5x while speaker is active

// --- Protocol & GPS Variables ---
bool useEspNow = false;
double currentLat = 0.0; 
double currentLon = 0.0; 
double lastValidLat = 0.0; 
double lastValidLon = 0.0; 
bool hasCachedLocation = false;

double peerLat = 0.0;
double peerLon = 0.0;
float distanceMeters = 9999.0;
unsigned long lastGpsBroadcast = 0;
const unsigned long GPS_INTERVAL = 10000;

unsigned long lastFlashSave = 0;
const unsigned long FLASH_SAVE_INTERVAL = 10000;

uint8_t peerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// Strict byte-packing (76 bytes total)
#pragma pack(push, 1)
typedef struct struct_message {
    char magic[3]; 
    char type;     
    uint8_t payload[64]; 
    float lat;
    float lon;
} struct_message;
#pragma pack(pop)

struct_message outgoingData;
struct_message incomingData;

// --- Voice RX Queue (Safe bridge between Wi-Fi callback and loop) ---
#define VOICE_QUEUE_SLOTS 8
uint8_t voiceQueue[VOICE_QUEUE_SLOTS][64];
volatile uint8_t vqHead = 0;
volatile uint8_t vqTail = 0;

// --- User Morse Code Variables ---
String rxMessage = "None";
String currentMorse = "";         
String assembledWord = "";        
unsigned long lastPressTime = 0;  
const unsigned long CHAR_TIMEOUT = 1000; 
const unsigned long WORD_TIMEOUT = 2200; 
bool waitingForNextWord = false;

// Hold Timers & Flags
unsigned long dotPressTime = 0;
bool dotActionFired = false;
unsigned long dashPressTime = 0;
bool dashActionFired = false;
unsigned long bothPressTime = 0;
bool bothActionFired = false;

const unsigned long BACKSPACE_HOLD = 2000; 
const unsigned long CLEAR_ALL_HOLD = 5000; 

struct MorseTable {
    char letter;
    const char* code;
};

MorseTable morseTable[] = {
    {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."}, {'E', "."},
    {'F', "..-."}, {'G', "--."}, {'H', "...."}, {'I', ".."}, {'J', ".---"},
    {'K', "-.-"}, {'L', ".-.."}, {'M', "--"}, {'N', "-."}, {'O', "---"},
    {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
    {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"}, {'Y', "-.--"},
    {'Z', "--.."}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
    {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},
    {'0', "-----"}, {' ', " "}
};

// --- Function Prototypes ---
void setupI2S();
void playI2STone(int freq, int durationMs);
void playI2SVoicePacket(const uint8_t* samples, int len);
float filterHumanVoice(float rawCentered);
void playBootSequence();
void parseGPS();
float calculateDistance(double lat1, double lon1, double lat2, double lon2);
void transmitData(char c);
void triggerSOS(double lat, double lon);
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len);
void handleIncomingData(char type, String payload);
char decodeMorse(String code);
void updateDisplay();
void showFeedbackOLED(String msg);

// --- 300Hz - 3400Hz Band-Pass Filter for Human Speech ---
float filterHumanVoice(float rawCentered) {
    float hpOut = ALPHA_HP * (prevHpOut + rawCentered - prevRawCentered);
    prevRawCentered = rawCentered;
    prevHpOut = hpOut;

    float lpOut = prevLpOut + ALPHA_LP * (hpOut - prevLpOut);
    prevLpOut = lpOut;

    return lpOut;
}

// --- Core v3.x Stereo I2S Audio Functions ---
void setupI2S() {
    i2s.setPins(I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DIN_PIN);
    if (!i2s.begin(I2S_MODE_STD, 8000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
        Serial.println("I2S Init Failed!");
    }
}

void playI2STone(int freq, int durationMs) {
    lastRxAudioTime = millis(); // Prevent local tone from triggering mic loop
    const int sampleRate = 8000;
    int totalSamples = (sampleRate * durationMs) / 1000;
    int16_t stereoBuf[128];
    
    for (int i = 0; i < totalSamples; i += 64) {
        int chunk = min(64, totalSamples - i);
        for (int j = 0; j < chunk; j++) {
            float t = (float)(i + j) / sampleRate;
            int16_t val = (int16_t)(sin(2.0 * PI * freq * t) * 8000);
            stereoBuf[j * 2]     = val;
            stereoBuf[j * 2 + 1] = val;
        }
        i2s.write((uint8_t *)stereoBuf, chunk * 2 * sizeof(int16_t));
    }
    lastRxAudioTime = millis();
}

void playI2SVoicePacket(const uint8_t* samples, int len) {
    lastRxAudioTime = millis(); // Engage loop filter while speaker plays RX voice
    int16_t stereoBuf[128];
    int count = min(len, 64);
    for (int i = 0; i < count; i++) {
        int16_t val = ((int16_t)samples[i] - 128) << 8;
        stereoBuf[i * 2]     = val;
        stereoBuf[i * 2 + 1] = val;
    }
    i2s.write((uint8_t *)stereoBuf, count * 2 * sizeof(int16_t));
}

void playBootSequence() {
  float cube3d[8][3] = {{-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1}, {-1,-1,1}, {1,-1,1}, {1,1,1}, {-1,1,1}};
  int edges[12][2] = {{0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7}};
  float angleX = 0, angleY = 0, angleZ = 0;
  float scale = 4.0; 
  
  for (int frame = 0; frame < 35; frame++) {
    display.clearDisplay();
    scale += 1.2; 
    angleX += 0.18; angleY += 0.25; angleZ += 0.12;
    
    int proj2d[8][2];
    for (int i = 0; i < 8; i++) {
      float x = cube3d[i][0], y = cube3d[i][1], z = cube3d[i][2];
      float x1 = x * cos(angleY) - z * sin(angleY);
      float z1 = x * sin(angleY) + z * cos(angleY);
      float y2 = y * cos(angleX) - z1 * sin(angleX);
      float z2 = y * sin(angleX) + z1 * cos(angleX);
      float x3 = x1 * cos(angleZ) - y2 * sin(angleZ);
      float y3 = x1 * sin(angleZ) + y2 * cos(angleZ);
      
      proj2d[i][0] = (int)(x3 * scale) + 64;
      proj2d[i][1] = (int)(y3 * scale) + 32;
    }
    
    for (int i = 0; i < 12; i++) {
      display.drawLine(proj2d[edges[i][0]][0], proj2d[edges[i][0]][1], proj2d[edges[i][1]][0], proj2d[edges[i][1]][1], SH110X_WHITE);
    }
    display.display();
    delay(25); 
  }

  String target = "E C H O : Z E R O";
  display.setTextSize(1);
  
  for (int locked = 0; locked <= target.length(); locked++) {
    for (int spin = 0; spin < 2; spin++) {
      display.clearDisplay();
      display.setCursor(14, 24);
      for (int i = 0; i < target.length(); i++) {
        if (i < locked) display.print(target[i]); 
        else display.print((char)random(33, 90)); 
      }
      display.drawRect(10, 52, 108, 6, SH110X_WHITE);
      int fill = map(locked, 0, target.length(), 0, 104);
      if(fill > 0) display.fillRect(12, 54, fill, 2, SH110X_WHITE);
      display.display();
      delay(30);
    }
  }
  
  playI2STone(523, 80);
  playI2STone(784, 120);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(6, 24);
  display.print("ECHO: ZERO");
  display.display();
  delay(1000); 

  for (int y = 24; y >= 0; y -= 3) {
    display.clearDisplay();
    if (y > 10) {
      display.setTextSize(2);
      display.setCursor(6, y);
      display.print("ECHO: ZERO");
    } else {
      display.setTextSize(1);
      int shiftX = map(y, 10, 0, 16, 34); 
      display.setCursor(shiftX, y);
      display.print("Echo: Zero"); 
    }
    display.display();
    delay(35); 
  }
}

void setup() {
    Serial.begin(115200);

    pinMode(DOT_PIN, INPUT_PULLUP);
    pinMode(DASH_PIN, INPUT_PULLUP);
    pinMode(micPin, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    setupI2S();

    // Auto-calibrate resting DC offset of MAX9814 mic (~1482)
    long sum = 0;
    for (int i = 0; i < 200; i++) {
        sum += analogRead(micPin);
        delay(2);
    }
    micCenter = sum / 200;
    Serial.print("Calibrated Mic Center: ");
    Serial.println(micCenter);

    preferences.begin("echozero", false); 
    lastValidLat = preferences.getDouble("lat", 0.0);
    lastValidLon = preferences.getDouble("lon", 0.0);
    
    if (lastValidLat != 0.0 && lastValidLon != 0.0) {
        hasCachedLocation = true;
    }

    strcpy(outgoingData.magic, "EZ");

    Wire.begin(21, 22);
    if(!display.begin(0x3C, true)) { 
        Serial.println("OLED Failed!");
        while (1); 
    }
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    
    playBootSequence();

    // --- WI-FI LONG RANGE & POWER BOOST ---
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); 
    WiFi.setTxPower(WIFI_POWER_19_5dBm); 
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerAddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    SPI.begin(18, 19, 23, 5);
    LoRa.setPins(loraCsPin, loraResetPin, loraDio0Pin);
    LoRa.setSPIFrequency(4E6);
    
    pinMode(loraResetPin, OUTPUT);
    digitalWrite(loraResetPin, LOW);
    delay(20);
    digitalWrite(loraResetPin, HIGH);
    delay(50);

    if (!LoRa.begin(433E6)) { 
        display.clearDisplay();
        display.setCursor(0, 24);
        display.print("LoRa Radio Error!");
        display.display();
        while (1);
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.enableCrc();

    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    updateDisplay();
}

void loop() {
    // 1. Drain any incoming ESP-NOW voice packets to MAX98357A speaker
    while (vqTail != vqHead) {
        playI2SVoicePacket(voiceQueue[vqTail], 64);
        vqTail = (vqTail + 1) % VOICE_QUEUE_SLOTS;
    }

    // 2. GPS & Protocol Switching
    parseGPS();
    double activeLat = (currentLat != 0.0) ? currentLat : (hasCachedLocation ? lastValidLat : 0.0);
    double activeLon = (currentLon != 0.0) ? currentLon : (hasCachedLocation ? lastValidLon : 0.0);

    distanceMeters = calculateDistance(activeLat, activeLon, peerLat, peerLon);
    
    if (indoorTestingMode) {
        useEspNow = true; 
    } else {
        useEspNow = (distanceMeters < 150.0 && peerLat != 0.0); 
    }

    if (millis() - lastGpsBroadcast > GPS_INTERVAL) {
        if (activeLat != 0.0 && activeLon != 0.0) {
            if (useEspNow) {
                outgoingData.type = 'G';
                outgoingData.lat = activeLat;
                outgoingData.lon = activeLon;
                esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
            } else {
                LoRa.beginPacket();
                LoRa.print("EZ:G:"); 
                LoRa.print(activeLat, 6);
                LoRa.print(",");
                LoRa.print(activeLon, 6);
                LoRa.endPacket();
            }
        }
        lastGpsBroadcast = millis();
        updateDisplay();
    }

    if (!useEspNow) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            String rxIncoming = "";
            while (LoRa.available()) rxIncoming += (char)LoRa.read();
            
            if (rxIncoming.startsWith("EZ:")) {
                char pktType = rxIncoming.charAt(3);
                String payload = rxIncoming.substring(5);
                
                if (pktType == 'G') {
                    int commaIndex = payload.indexOf(',');
                    if (commaIndex > 0) {
                        peerLat = payload.substring(0, commaIndex).toDouble();
                        peerLon = payload.substring(commaIndex + 1).toDouble();
                    }
                } else if (pktType == 'S') {
                    handleIncomingData('S', payload);
                } else if (pktType == 'M') {
                    handleIncomingData('M', payload);
                }
            }
        }
    }

    // 3. Human Voice Band-Pass + Acoustic Loop Suppressor (300Hz - 3400Hz @ 8kHz)
    if (useEspNow) {
        outgoingData.type = 'A';
        float packetEnergySum = 0.0f;

        // Check if local speaker played recently (within ECHO_HOLD_MS)
        bool isSpeakerActive = (millis() - lastRxAudioTime) < ECHO_HOLD_MS;
        float activeGateThreshold = isSpeakerActive 
                                    ? (VOICE_GATE_THRESHOLD * LOOP_DUCK_MULTIPLIER) 
                                    : VOICE_GATE_THRESHOLD;
        float sampleFloorGate     = isSpeakerActive ? 95.0f : 25.0f;
        
        for (int i = 0; i < 64; i++) {
            unsigned long startMicros = micros();
            float rawCentered = (float)(analogRead(micPin) - micCenter);
            
            // Apply 300Hz - 3400Hz band-pass filter
            float voiceSample = filterHumanVoice(rawCentered);
            packetEnergySum += fabsf(voiceSample);
            
            if (fabsf(voiceSample) < sampleFloorGate) {
                outgoingData.payload[i] = 128; // Dead silence
            } else {
                int clamped = constrain((int)voiceSample, -850, 850);
                outgoingData.payload[i] = map(clamped, -850, 850, 0, 255);
            }
            
            while (micros() - startMicros < 125); // 8000Hz lock
        }
        
        // Transmit only if voice energy clears the loop-adjusted gate
        float avgVoiceEnergy = packetEnergySum / 64.0f;
        if (avgVoiceEnergy > activeGateThreshold) {
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
        }
    }

    // 4. Morse Button Handling
    bool dotPressed = (digitalRead(DOT_PIN) == LOW);
    bool dashPressed = (digitalRead(DASH_PIN) == LOW);

    if (dotPressed && dashPressed) {
        dotPressTime = 0;
        dashPressTime = 0;
        dotActionFired = false;
        dashActionFired = false;

        if (bothPressTime == 0) {
            bothPressTime = millis();
            bothActionFired = false;
        } else if (!bothActionFired && (millis() - bothPressTime >= CLEAR_ALL_HOLD)) {
            assembledWord = "";
            currentMorse = "";
            playI2STone(300, 250);
            showFeedbackOLED("All text cleared!");
            bothActionFired = true;
        }
        return; 
    } else {
        bothPressTime = 0;
        bothActionFired = false;
    }

    if (dotPressed) {
        if (dotPressTime == 0) {
            dotPressTime = millis();
            dotActionFired = false;
        } else if (!dotActionFired && (millis() - dotPressTime >= BACKSPACE_HOLD)) {
            if (assembledWord.length() > 0) {
                assembledWord.remove(assembledWord.length() - 1);
                playI2STone(400, 150);
                showFeedbackOLED("Backspace!");
            }
            dotActionFired = true; 
        }
    } else {
        if (dotPressTime > 0) {
            unsigned long holdDuration = millis() - dotPressTime;
            if (!dotActionFired && holdDuration < BACKSPACE_HOLD) {
                currentMorse += ".";
                lastPressTime = millis();
                waitingForNextWord = false;
                playI2STone(650, 50); // Dot Sidetone
                updateDisplay();
            }
            dotPressTime = 0;
            dotActionFired = false;
        }
    }

    if (dashPressed) {
        if (dashPressTime == 0) {
            dashPressTime = millis();
            dashActionFired = false;
        } else if (!dashActionFired && (millis() - dashPressTime >= BACKSPACE_HOLD)) {
            if (assembledWord.length() > 0) {
                assembledWord.remove(assembledWord.length() - 1);
                playI2STone(400, 150);
                showFeedbackOLED("Backspace!");
            }
            dashActionFired = true; 
        }
    } else {
        if (dashPressTime > 0) {
            unsigned long holdDuration = millis() - dashPressTime;
            if (!dashActionFired && holdDuration < BACKSPACE_HOLD) {
                currentMorse += "-";
                lastPressTime = millis();
                waitingForNextWord = false;
                playI2STone(650, 140); // Dash Sidetone
                updateDisplay();
            }
            dashPressTime = 0;
            dashActionFired = false;
        }
    }

    if (currentMorse.length() > 0 && (millis() - lastPressTime > CHAR_TIMEOUT)) {
        if (currentMorse.equals("...---...")) {
            triggerSOS(activeLat, activeLon);
        } else {
            char translatedChar = decodeMorse(currentMorse);
            if (assembledWord.length() < 38) assembledWord += translatedChar;
            transmitData(translatedChar);
        }

        currentMorse = ""; 
        lastPressTime = millis(); 
        waitingForNextWord = true;
        updateDisplay();
    }

    if (waitingForNextWord && (millis() - lastPressTime > WORD_TIMEOUT)) {
        if (assembledWord.length() < 38 && !assembledWord.endsWith(" ")) {
            if (assembledWord.length() != 17) assembledWord += " "; 
        }
        waitingForNextWord = false; 
        updateDisplay();
    }
}

void transmitData(char c) {
    if (useEspNow) {
        outgoingData.type = 'M';
        sprintf((char*)outgoingData.payload, "%c", c);
        esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
    } else {
        LoRa.beginPacket();
        LoRa.print("EZ:M:"); 
        LoRa.print(c);
        LoRa.endPacket();
    }
}

void triggerSOS(double lat, double lon) {
    display.clearDisplay();
    display.setCursor(20, 20);
    display.setTextSize(2);
    display.print("SOS ALERT!");
    display.setCursor(0, 45);
    display.setTextSize(1);
    display.print("Broadcasting SOS!");
    display.display();

    if (useEspNow) {
        outgoingData.type = 'S';
        sprintf((char*)outgoingData.payload, "EMERGENCY SOS");
        outgoingData.lat = lat;
        outgoingData.lon = lon;
        esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
    } else {
        LoRa.beginPacket();
        LoRa.print("EZ:S:"); 
        LoRa.print(lat, 4);
        LoRa.print(",");
        LoRa.print(lon, 4);
        LoRa.endPacket();
    }

    playI2STone(2000, 1000); 
    delay(1000);
    updateDisplay();
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len) {
    if (len < sizeof(struct_message)) return;
    memcpy(&incomingData, incoming, sizeof(incomingData));
    if (incomingData.magic[0] != 'E' || incomingData.magic[1] != 'Z') return; 
    
    if (incomingData.type == 'A') {
        lastRxAudioTime = millis(); // Mark RX active immediately upon packet arrival
        uint8_t nextHead = (vqHead + 1) % VOICE_QUEUE_SLOTS;
        if (nextHead != vqTail) {
            memcpy(voiceQueue[vqHead], incomingData.payload, 64);
            vqHead = nextHead;
        }
    } else if (incomingData.type == 'G') {
        peerLat = incomingData.lat;
        peerLon = incomingData.lon;
    } else if (incomingData.type == 'S') {
        handleIncomingData('S', String((char*)incomingData.payload));
    } else if (incomingData.type == 'M') {
        incomingData.payload[63] = '\0';
        handleIncomingData('M', String((char*)incomingData.payload));
    }
}

void handleIncomingData(char type, String payload) {
    if (type == 'S') {
        rxMessage = "!! SOS ALERT !!";
        playI2STone(2500, 1000); 
        updateDisplay();
    } else if (type == 'M') {
        if (rxMessage == "None" || rxMessage == "!! SOS ALERT !!") rxMessage = "";
        for (int i = 0; i < payload.length(); i++) {
            if (rxMessage.length() == 17 && payload[i] == ' ') continue; 
            if (rxMessage.length() < 38) rxMessage += payload[i];
        }
        playI2STone(1000, 150); 
        updateDisplay();
    }
}

char decodeMorse(String code) {
    int tableSize = sizeof(morseTable) / sizeof(morseTable[0]);
    for (int i = 0; i < tableSize; i++) {
        if (code.equals(morseTable[i].code)) return morseTable[i].letter;
    }
    return '?'; 
}

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(34, 0); 
    display.print("Echo: Zero");

    display.setCursor(0, 8);
    display.printf("%s | Dist:%dm", useEspNow ? "ESP-NOW" : "LoRa", (int)distanceMeters);
    
    display.setCursor(0, 16);
    if (currentLat != 0.0) {
        display.printf("Lat:%.2f Lon:%.2f", currentLat, currentLon);
    } else if (hasCachedLocation) {
        display.printf("Lat*:%.2f Lon*:%.2f", lastValidLat, lastValidLon); 
    } else {
        display.print("Loc: Searching sat..");
    }

    display.setCursor(0, 24);
    display.print("RX: ");
    display.print(rxMessage);
    
    display.setCursor(0, 40);
    display.print("TX: ");
    display.print(assembledWord);
    
    display.setCursor(0, 56);
    display.print("Code: ");
    display.print(currentMorse);
    
    if (currentMorse.length() > 0) {
        char preview = decodeMorse(currentMorse);
        if (preview != '?') {
            display.setCursor(100, 56); 
            display.print("[");
            display.print(preview);
            display.print("]");
        }
    }
    display.display();
}

void showFeedbackOLED(String msg) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(34, 0); 
    display.print("Echo: Zero");
    display.setCursor(0, 8);
    display.printf("%s | Dist:%dm", useEspNow ? "ESP-NOW" : "LoRa", (int)distanceMeters);
    display.setCursor(0, 16);
    
    if (currentLat != 0.0) {
        display.printf("Lat:%.2f Lon:%.2f", currentLat, currentLon);
    } else if (hasCachedLocation) {
        display.printf("Lat*:%.2f Lon*:%.2f", lastValidLat, lastValidLon);
    } else {
        display.print("Loc: Searching sat..");
    }

    display.setCursor(0, 24);
    display.print("RX: ");
    display.print(rxMessage);
    
    display.setCursor(0, 40);
    display.print("TX: ");
    display.print(msg); 
    
    display.setCursor(0, 56);
    display.print("Code: ");
    display.print(currentMorse);
    display.display();
    
    delay(800);      
    updateDisplay(); 
}

void parseGPS() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }
    if (gps.location.isValid() && gps.location.isUpdated()) {
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();
        lastValidLat = currentLat;
        lastValidLon = currentLon;
        hasCachedLocation = true;
        
        if (millis() - lastFlashSave > FLASH_SAVE_INTERVAL) {
            preferences.putDouble("lat", currentLat);
            preferences.putDouble("lon", currentLon);
            lastFlashSave = millis();
        }
        updateDisplay();
    }
}

float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    if (lat2 == 0.0 || lon2 == 0.0 || lat1 == 0.0) return 9999.0;
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000.0 * c;
}
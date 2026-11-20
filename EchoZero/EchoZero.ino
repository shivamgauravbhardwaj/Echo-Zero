// ============================================================================
// ECHO: ZERO — Hybrid Off-Grid & Cloud Emergency Communicator Firmware
// Hardware: ESP32 + SX1278 LoRa (433MHz) + NEO-6M GPS + SH1106 1.3" OLED
//           + MAX9814 Mic + MAX98357A I2S Amp + 2 Tactile Morse Buttons
// ============================================================================

// --- BLYNK CLOUD CREDENTIALS (MUST STAY AT VERY TOP BEFORE LIBRARIES) ---
#define BLYNK_TEMPLATE_ID   "TMPL35aY_w7Hw"
#define BLYNK_TEMPLATE_NAME "Echo Zero"
#define BLYNK_AUTH_TOKEN    "_WcgUhLCo2WIcupkvp-mmAqlnAr-KyNj"

// --- 2.4 GHz WI-FI HOTSPOT CREDENTIALS ---
// Note: ESP32 Wi-Fi radio only supports 2.4 GHz networks (not 5 GHz).
const char* WIFI_SSID = "doughnut";
const char* WIFI_PASS = "kshitijpratap";

// --- DUAL-UNIT CLOUD ROLE SWITCH ---
// Set 'true' on Unit #1 (connects to Blynk Cloud) and 'false' on Unit #2 (handheld peer).
// Both units still connect to the hotspot so their Wi-Fi radios lock onto the exact same channel.
bool enableBlynkCloud = false;

// --- REQUIRED LIBRARIES ---
#include <SPI.h>               // SPI bus communication for SX1278 LoRa module
#include <LoRa.h>              // 433 MHz long-range LoRa packet radio driver
#include <Wire.h>              // I2C bus communication for 1.3" OLED screen
#include <Adafruit_GFX.h>      // Core graphics primitives (text, lines, rectangles)
#include <Adafruit_SH110X.h>   // Driver for SH1106 128x64 monochrome OLED display
#include <WiFi.h>              // ESP32 Wi-Fi station mode driver
#include <esp_wifi.h>          // Low-level ESP-IDF Wi-Fi radio controls
#include <esp_now.h>           // Connectionless peer-to-peer 2.4 GHz protocol (for live voice & fast Morse)
#include <BlynkSimpleEsp32.h>  // Blynk IoT cloud telemetry & remote dashboard sync
#include <TinyGPS++.h>         // NMEA sentence parser for NEO-6M GPS module
#include <Preferences.h>       // Non-Volatile Storage (NVS) flash memory to cache last GPS fix
#include <ESP_I2S.h>           // ESP32 Arduino Core v3.x digital I2S audio driver
#include <math.h>              // Trigonometry functions (sin, cos, atan2, sqrt) for 3D boot cube & Haversine GPS distance

// --- TESTING OVERRIDE ---
// true  = Forces ESP-NOW indoors (where GPS satellites can't lock or units are <150m apart).
// false = Automatically switches to 433 MHz LoRa when peer distance exceeds 150 meters.
bool indoorTestingMode = true; 

// --- HARDWARE OBJECT INSTANCES ---
Adafruit_SH1106G display(128, 64, &Wire, -1); // 128x64 OLED on I2C, -1 = no hardware reset pin
TinyGPSPlus gps;                              // GPS parser object
Preferences preferences;                      // Flash storage object for persistent GPS coordinates
I2SClass i2s;                                 // Digital audio output object for MAX98357A amp

// --- SX1278 LORA MODULE PINS (VSPI BUS) ---
const int loraCsPin    = 5;   // Chip Select (NSS)
const int loraResetPin = 14;  // Hardware Reset
const int loraDio0Pin  = 2;   // Interrupt pin (triggers when a LoRa packet arrives)

// --- MORSE INPUT BUTTON PINS ---
// Configured as INPUT_PULLUP: unpressed = HIGH (3.3V), pressed to GND = LOW (0V).
const int DOT_PIN  = 4;       // Short tap = '.' | Hold 2s = Backspace on TX
const int DASH_PIN = 13;      // Short tap = '-' | Hold 2s = Clear RX screen

// --- AUDIO HARDWARE PINS ---
const int I2S_DIN_PIN  = 25;  // I2S Serial Data Out -> MAX98357A DIN
const int I2S_BCLK_PIN = 26;  // I2S Bit Clock       -> MAX98357A BCLK
const int I2S_LRC_PIN  = 27;  // I2S Left/Right Word Clock -> MAX98357A LRC
const int micPin       = 34;  // Analog ADC1 input   <- MAX9814 Microphone OUT (ADC1 works alongside Wi-Fi)

int micCenter = 1482;         // DC bias midpoint (~1.2V on 12-bit 0-4095 ADC), auto-calibrated in setup()

// --- DIGITAL SIGNAL PROCESSING (DSP): VOICE BAND-PASS & NOISE FILTER ---
// Isolates human speech (350 Hz - 2500 Hz) at 8 kHz sampling rate and blocks screech/RF noise.
float prevRawCentered = 0.0f; // Previous spike-cleaned sample for derivative calculation
float prevHpOut       = 0.0f; // High-pass filter state memory
float prevLpOut1      = 0.0f; // 1st-stage low-pass filter state memory
float prevLpOut2      = 0.0f; // 2nd-stage low-pass filter state memory
const float ALPHA_HP  = 0.7844f;   // 350 Hz IIR high-pass coefficient (removes 50Hz AC hum & breath rumble)
const float ALPHA_LP  = 0.6626f;   // 2500 Hz 2-stage IIR low-pass (12dB/oct roll-off kills high-pitch feedback screech)
const float MAX_SLEW_STEP = 550.0f; // Slew-rate limiter: clips sudden single-sample Wi-Fi RF power spikes
const float VOICE_GATE_THRESHOLD = 85.0f; // Noise gate: ignores background room hiss from MAX9814's 60dB AGC

// --- HALF-DUPLEX ACOUSTIC ECHO LOCKOUT ---
// Prevents audio played on the speaker from feeding back into the mic and causing an infinite squeal.
volatile unsigned long lastRxAudioTime = 0; // Timestamp of last incoming voice/tone sample played
unsigned long lastTxAudioTime = 0;          // Timestamp of last outgoing voice packet sent
unsigned long lastMorseTxTime = 0;          // Timestamp of last Morse packet (pauses mic 60ms so Morse never collides)
const unsigned long ECHO_HOLD_MS = 280;     // Keeps mic muted for 280ms after speaker finishes playing

// --- NETWORK PROTOCOL & GPS STATE VARIABLES ---
bool useEspNow = false;           // true = ESP-NOW (short-range high-speed), false = LoRa (long-range)
double currentLat = 0.0;          // Live GPS latitude
double currentLon = 0.0;          // Live GPS longitude
double lastValidLat = 0.0;        // Last saved GPS latitude from NVS flash
double lastValidLon = 0.0;        // Last saved GPS longitude from NVS flash
bool hasCachedLocation = false;   // true if valid coordinates exist in flash memory

double peerLat = 0.0;             // Remote unit's latitude received over radio
double peerLon = 0.0;             // Remote unit's longitude received over radio
float distanceMeters = 9999.0;    // Calculated Haversine distance between the two units
unsigned long lastGpsBroadcast = 0;
const unsigned long GPS_INTERVAL = 10000;      // Broadcast GPS coordinates every 10 seconds

unsigned long lastFlashSave = 0;
const unsigned long FLASH_SAVE_INTERVAL = 10000; // Save GPS to flash max once every 10s (protects flash lifespan)

// Broadcast MAC address (FF:FF:FF:FF:FF:FF) sends ESP-NOW packets to any listening Echo: Zero peer
uint8_t peerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// --- STRICT 77-BYTE BINARY PACKET STRUCTURE ---
// '#pragma pack(1)' removes compiler memory padding so both ESP32s read exact byte offsets.
#pragma pack(push, 1)
typedef struct struct_message {
    char magic[3];       // Signature "EZ\0" to ignore random 2.4 GHz traffic
    char type;           // Packet type: 'A'=Audio, 'M'=Morse char, 'T'=Cloud Text, 'G'=GPS, 'S'=SOS
    uint8_t seq;         // Sequence ID (1-255) used to drop duplicate packets from 3x burst sends
    uint8_t payload[64]; // 64 bytes of 8-bit PCM voice samples OR ASCII text/Morse character
    float lat;           // Sender's latitude
    float lon;           // Sender's longitude
} struct_message;
#pragma pack(pop)

struct_message outgoingData;
struct_message incomingData;

uint8_t txSeqCounter = 1;         // Increments on every outgoing Morse/Text/SOS message
volatile uint8_t lastRxSeq = 0;   // Stores last received sequence ID to prevent duplicate letters

// --- LOCK-FREE RING BUFFERS (SAFE BRIDGE BETWEEN WI-FI INTERRUPT & MAIN LOOP) ---
// Wi-Fi receive callbacks run in a high-priority system task; queues let loop() handle audio/OLED safely.
#define VOICE_QUEUE_SLOTS 8
uint8_t voiceQueue[VOICE_QUEUE_SLOTS][64]; // Circular buffer holding up to 8 incoming 64-byte voice packets
volatile uint8_t vqHead = 0;               // Write index (updated by Wi-Fi callback)
volatile uint8_t vqTail = 0;               // Read index (updated by loop())

struct CtrlPacket {
    char type;
    char payload[64];
};
#define CTRL_QUEUE_SLOTS 8
CtrlPacket ctrlQueue[CTRL_QUEUE_SLOTS];    // Circular buffer holding up to 8 incoming Morse/Text/SOS commands
volatile uint8_t cqHead = 0;
volatile uint8_t cqTail = 0;

// --- MORSE DECODER & AUTO-SPACING VARIABLES ---
String rxMessage = "None";             // Text displayed on the "RX:" line of the OLED
String pendingCloudText = "";          // Queues Blynk V3 text so loop() broadcasts it cleanly to Unit #2
unsigned long lastRxMorseTime = 0;     // Timestamp when last Morse character arrived on RX
bool waitingForRxWord = false;         // Flag indicating RX is waiting to see if a word space is needed

String currentMorse = "";              // Raw dots/dashes currently being tapped (e.g., ".-")
String assembledWord = "";             // Decoded characters displayed on the "TX:" line
unsigned long lastPressTime = 0;       // Timestamp of last button tap on TX
const unsigned long CHAR_TIMEOUT    = 1000; // 1.0s pause after tapping finalizes the letter & transmits it
const unsigned long WORD_TIMEOUT    = 2200; // 2.2s pause on TX inserts a space between words
const unsigned long RX_WORD_TIMEOUT = 4200; // 4.2s pause on RX inserts a space (accounts for sender's tap time + 1s CHAR_TIMEOUT)
bool waitingForNextWord = false;       // Flag indicating TX is waiting to insert a word space

// --- BUTTON HOLD TIMERS & DEBOUNCE FLAGS ---
unsigned long dotPressTime = 0;
bool dotActionFired = false;           // Prevents long-hold Backspace from also registering as a Dot tap
unsigned long dashPressTime = 0;
bool dashActionFired = false;          // Prevents long-hold RX Clear from also registering as a Dash tap
unsigned long bothPressTime = 0;
bool bothActionFired = false;

const unsigned long DEBOUNCE_MIN_MS = 25;   // Ignores mechanical switch bounce shorter than 25ms
const unsigned long BACKSPACE_HOLD  = 2000; // Hold Dot (2s) = Backspace TX | Hold Dash (2s) = Clear RX
const unsigned long CLEAR_ALL_HOLD  = 5000; // Hold Both (5s) = Clear both TX & RX

// --- INTERNATIONAL MORSE LOOKUP TABLE (A-Z, 0-9) ---
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

// --- FUNCTION PROTOTYPES ---
bool isCloudReady();
uint8_t nextSeqId();
void pushCloudMapAndStatus(double lat, double lon);
void setupI2S();
void playI2STone(int freq, int durationMs);
void playI2SVoicePacket(const uint8_t* samples, int len);
float filterHumanVoice(float rawCentered);
void playBootSequence();
void parseGPS();
float calculateDistance(double lat1, double lon1, double lat2, double lon2);
void transmitData(char c);
void transmitTextString(String text);
void triggerSOS(double lat, double lon);
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len);
void handleIncomingData(char type, String payload);
char decodeMorse(String code);
void updateDisplay();
void showFeedbackOLED(String msg);

// Returns true only when this unit is the cloud gateway AND both Wi-Fi and Blynk are active
bool isCloudReady() {
    return (enableBlynkCloud && WiFi.status() == WL_CONNECTED && Blynk.connected());
}

// Generates a non-zero rolling sequence ID (1..255) for packet deduplication
uint8_t nextSeqId() {
    txSeqCounter++;
    if (txSeqCounter == 0) txSeqCounter = 1;
    return txSeqCounter;
}

// ============================================================================
// BLYNK IOT CLOUD CALLBACKS (V0 = SOS, V1 = Log, V2 = Maps Link, V3 = Text, V4 = Status)
// ============================================================================

// Runs automatically once the ESP32 authenticates with Blynk Cloud servers
BLYNK_CONNECTED() {
    if (!enableBlynkCloud) return;
    Serial.println("Blynk Cloud Connected!");
    Blynk.virtualWrite(V0, 0);                        // Reset SOS switch to OFF on connect
    Blynk.virtualWrite(V1, "Echo: Zero Connected");   // Print connection status to V1 terminal/label
    double lat = (currentLat != 0.0) ? currentLat : (hasCachedLocation ? lastValidLat : 0.0);
    double lon = (currentLon != 0.0) ? currentLon : (hasCachedLocation ? lastValidLon : 0.0);
    pushCloudMapAndStatus(lat, lon);                  // Push initial GPS link & radio status
    updateDisplay();                                  // Refresh OLED to show '[B]' badge
}

// V0: Two-way SOS Alarm Switch triggered from the laptop/phone Blynk dashboard
BLYNK_WRITE(V0) {
    if (!isCloudReady()) return;
    int sosCmd = param.asInt();
    if (sosCmd == 1) {
        handleIncomingData('S', "CLOUD SOS");         // Trigger local OLED alert + 2500 Hz alarm tone
        Blynk.virtualWrite(V1, "!! SOS TRIGGERED FROM CLOUD !!");

        // Forward the cloud SOS alert to the second handheld unit over ESP-NOW (3x burst for reliability)
        double lat = (currentLat != 0.0) ? currentLat : lastValidLat;
        double lon = (currentLon != 0.0) ? currentLon : lastValidLon;
        outgoingData.type = 'S';
        outgoingData.seq = nextSeqId();
        sprintf((char*)outgoingData.payload, "EMERGENCY SOS");
        outgoingData.lat = lat;
        outgoingData.lon = lon;
        for (int r = 0; r < 3; r++) {
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
            delay(15);
        }
    } else {
        // Turning SOS switch OFF clears the alarm banner on OLED
        rxMessage = "None";
        waitingForRxWord = false;
        updateDisplay();
        Blynk.virtualWrite(V1, "SOS Cleared");
    }
}

// V3: Text Input widget on Blynk dashboard -> displays locally and queues relay to Peer Unit
BLYNK_WRITE(V3) {
    if (!isCloudReady()) return;
    String rawText = param.asStr();
    rawText.trim();
    if (rawText.length() == 0) return;

    // Format text to fit the 2-line (38-char) RX area on the 128x64 OLED
    rxMessage = "";
    waitingForRxWord = false;
    for (int i = 0; i < rawText.length(); i++) {
        if (rxMessage.length() == 17 && rawText[i] == ' ') continue; // Skip leading space at line wrap
        if (rxMessage.length() < 38) rxMessage += rawText[i];
    }
    playI2STone(1000, 150);                           // Notification chime
    updateDisplay();

    Blynk.virtualWrite(V1, "CLOUD -> RX: " + rxMessage);
    pendingCloudText = rawText;                       // Hand off to loop() for clean radio broadcast
}

// Formats live/cached GPS into a clickable Google Maps URL (V2) and radio link summary (V4)
void pushCloudMapAndStatus(double lat, double lon) {
    if (!isCloudReady()) return;

    if (lat != 0.0 || lon != 0.0) {
        char mapUrl[64];
        snprintf(mapUrl, sizeof(mapUrl), "https://maps.google.com/?q=%.5f,%.5f", lat, lon);
        Blynk.virtualWrite(V2, mapUrl);
    } else {
        Blynk.virtualWrite(V2, "Searching for GPS lock...");
    }

    char statusBuf[64];
    const char* gpsTag = (currentLat != 0.0) ? "Live GPS" : (hasCachedLocation ? "Cached GPS" : "No GPS Fix");
    snprintf(statusBuf, sizeof(statusBuf), "%s | %s | Dist:%dm",
             gpsTag, useEspNow ? "ESP-NOW" : "LoRa", (int)distanceMeters);
    Blynk.virtualWrite(V4, statusBuf);
}

// ============================================================================
// DSP VOICE FILTER & I2S DIGITAL AUDIO FUNCTIONS
// ============================================================================

// 3-Stage Digital Filter: Slew Spike Limiter -> 350Hz High-Pass -> 2nd-Order 2500Hz Low-Pass
float filterHumanVoice(float rawCentered) {
    // Stage 1: Slew-rate limiter clamps sudden single-sample Wi-Fi power-rail spikes
    float delta = rawCentered - prevRawCentered;
    if (delta > MAX_SLEW_STEP) delta = MAX_SLEW_STEP;
    else if (delta < -MAX_SLEW_STEP) delta = -MAX_SLEW_STEP;
    float cleanSample = prevRawCentered + delta;

    // Stage 2: 1st-order IIR High-Pass filter (350 Hz cutoff) removes DC drift & low rumble
    float hpOut = ALPHA_HP * (prevHpOut + cleanSample - prevRawCentered);
    prevRawCentered = cleanSample;
    prevHpOut = hpOut;

    // Stage 3: Two cascaded IIR Low-Pass stages (2500 Hz cutoff, 12 dB/octave) kill acoustic screech
    float lpOut1 = prevLpOut1 + ALPHA_LP * (hpOut - prevLpOut1);
    prevLpOut1 = lpOut1;

    float lpOut2 = prevLpOut2 + ALPHA_LP * (lpOut1 - prevLpOut2);
    prevLpOut2 = lpOut2;

    return lpOut2;
}

// Initializes I2S peripheral at 8000 Hz sample rate, 16-bit stereo PCM for MAX98357A amplifier
void setupI2S() {
    i2s.setPins(I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DIN_PIN);
    if (!i2s.begin(I2S_MODE_STD, 8000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
        Serial.println("I2S Init Failed!");
    }
}

// Synthesizes a pure sine-wave tone (used for Morse dot/dash sidetones, RX chimes, and SOS alarm)
void playI2STone(int freq, int durationMs) {
    lastRxAudioTime = millis(); // Engage half-duplex mic lockout while speaker plays
    const int sampleRate = 8000;
    int totalSamples = (sampleRate * durationMs) / 1000;
    int16_t stereoBuf[128];     // 64 stereo frames (Left + Right interleaved)
    
    for (int i = 0; i < totalSamples; i += 64) {
        int chunk = min(64, totalSamples - i);
        for (int j = 0; j < chunk; j++) {
            float t = (float)(i + j) / sampleRate;
            int16_t val = (int16_t)(sin(2.0 * PI * freq * t) * 8000); // Sine wave amplitude = 8000
            stereoBuf[j * 2]     = val; // Left channel
            stereoBuf[j * 2 + 1] = val; // Right channel
        }
        i2s.write((uint8_t *)stereoBuf, chunk * 2 * sizeof(int16_t));
    }
    lastRxAudioTime = millis(); // Extend lockout window after tone completes
}

// Converts incoming 8-bit unsigned voice samples (0-255) back to 16-bit signed PCM and plays via I2S
void playI2SVoicePacket(const uint8_t* samples, int len) {
    lastRxAudioTime = millis(); // Keep mic muted while incoming voice is playing
    int16_t stereoBuf[128];
    int count = min(len, 64);
    for (int i = 0; i < count; i++) {
        // Re-center around 0 (-128..+127) and scale by 2^7 (<< 7) for clean room volume
        int16_t val = ((int16_t)samples[i] - 128) << 7;
        stereoBuf[i * 2]     = val;
        stereoBuf[i * 2 + 1] = val;
    }
    i2s.write((uint8_t *)stereoBuf, count * 2 * sizeof(int16_t));
}

// ============================================================================
// 3D ROTATING CUBE & MATRIX TEXT BOOT ANIMATION
// ============================================================================
void playBootSequence() {
  // 8 vertices (x,y,z) and 12 edges of a 3D wireframe cube
  float cube3d[8][3] = {{-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1}, {-1,-1,1}, {1,-1,1}, {1,1,1}, {-1,1,1}};
  int edges[12][2] = {{0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7}};
  float angleX = 0, angleY = 0, angleZ = 0;
  float scale = 4.0; 
  
  // Phase 1: 35-frame 3D rotation matrix projection onto 2D OLED screen
  for (int frame = 0; frame < 35; frame++) {
    display.clearDisplay();
    scale += 1.2; 
    angleX += 0.18; angleY += 0.25; angleZ += 0.12;
    
    int proj2d[8][2];
    for (int i = 0; i < 8; i++) {
      float x = cube3d[i][0], y = cube3d[i][1], z = cube3d[i][2];
      // Rotate around Y, then X, then Z axis
      float x1 = x * cos(angleY) - z * sin(angleY);
      float z1 = x * sin(angleY) + z * cos(angleY);
      float y2 = y * cos(angleX) - z1 * sin(angleX);
      float z2 = y * sin(angleX) + z1 * cos(angleX);
      float x3 = x1 * cos(angleZ) - y2 * sin(angleZ);
      float y3 = x1 * sin(angleZ) + y2 * cos(angleZ);
      
      proj2d[i][0] = (int)(x3 * scale) + 64; // Center X at pixel 64
      proj2d[i][1] = (int)(y3 * scale) + 32; // Center Y at pixel 32
    }
    
    for (int i = 0; i < 12; i++) {
      display.drawLine(proj2d[edges[i][0]][0], proj2d[edges[i][0]][1], proj2d[edges[i][1]][0], proj2d[edges[i][1]][1], SH110X_WHITE);
    }
    display.display();
    delay(25); 
  }

  // Phase 2: Character-by-character decryption reveal with progress bar
  String target = "E C H O : Z E R O";
  display.setTextSize(1);
  
  for (int locked = 0; locked <= target.length(); locked++) {
    for (int spin = 0; spin < 2; spin++) {
      display.clearDisplay();
      display.setCursor(14, 24);
      for (int i = 0; i < target.length(); i++) {
        if (i < locked) display.print(target[i]); 
        else display.print((char)random(33, 90)); // Random ASCII scramble before locking
      }
      display.drawRect(10, 52, 108, 6, SH110X_WHITE);
      int fill = map(locked, 0, target.length(), 0, 104);
      if(fill > 0) display.fillRect(12, 54, fill, 2, SH110X_WHITE);
      display.display();
      delay(30);
    }
  }
  
  // Dual-tone startup chime (C5 -> G5)
  playI2STone(523, 80);
  playI2STone(784, 120);

  // Phase 3: Title banner slides up into the top status bar
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

// ============================================================================
// SETUP: RUNS ONCE AT BOOT TO INITIALIZE ALL PERIPHERALS & RADIOS
// ============================================================================
void setup() {
    Serial.begin(115200);

    // 1. Configure Morse buttons and 12-bit ADC (0-3.3V full range via 11dB attenuation)
    pinMode(DOT_PIN, INPUT_PULLUP);
    pinMode(DASH_PIN, INPUT_PULLUP);
    pinMode(micPin, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 2. Start I2S audio and auto-calibrate microphone silence DC offset (200 samples)
    setupI2S();

    long sum = 0;
    for (int i = 0; i < 200; i++) {
        sum += analogRead(micPin);
        delay(2);
    }
    micCenter = sum / 200;

    // 3. Load last saved GPS coordinates from ESP32 non-volatile flash memory
    preferences.begin("echozero", false); 
    lastValidLat = preferences.getDouble("lat", 0.0);
    lastValidLon = preferences.getDouble("lon", 0.0);
    
    if (lastValidLat != 0.0 && lastValidLon != 0.0) {
        hasCachedLocation = true;
    }

    strcpy(outgoingData.magic, "EZ"); // Set packet header signature

    // 4. Initialize I2C OLED display (address 0x3C) and run boot animation
    Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
    if(!display.begin(0x3C, true)) { 
        Serial.println("OLED Failed!");
        while (1); 
    }
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    
    playBootSequence();

    // 5. Connect to 2.4 GHz Hotspot (if available) or lock onto Channel 1 for Off-Grid ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);             // Disable Wi-Fi power-saving sleep for low-latency audio
    WiFi.setAutoReconnect(true);
    WiFi.setTxPower(WIFI_POWER_17dBm); 

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 8000)) {
        delay(100);
    }

    if (enableBlynkCloud) {
        Blynk.config(BLYNK_AUTH_TOKEN);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi Connected! IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(" | Channel: ");
        Serial.println(WiFi.channel());
        if (enableBlynkCloud) {
            Blynk.connect(3000);
        }
    } else {
        // Crucial Off-Grid Fallback: stops background channel scanning so ESP-NOW stays locked on Ch 1
        WiFi.setAutoReconnect(false);
        WiFi.disconnect(false);
        Serial.println("No WiFi -> Pure Off-Grid Mode Locked (Ch 1)");
    }
    
    // 6. Initialize ESP-NOW peer-to-peer protocol and register broadcast peer
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerAddress, 6);
        peerInfo.channel = 0;  // 0 = use current Wi-Fi channel
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    // 7. Initialize SX1278 LoRa radio on 433 MHz (SF7, 125 kHz bandwidth, CRC enabled)
    SPI.begin(18, 19, 23, 5); // SCK=18, MISO=19, MOSI=23, CS=5
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

    // 8. Start UART2 at 9600 baud for NEO-6M GPS module (RX2=GPIO 16, TX2=GPIO 17)
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    updateDisplay();
}

// ============================================================================
// MAIN LOOP: RUNS CONTINUOUSLY (AUDIO -> QUEUES -> BLYNK -> GPS/LORA -> MIC -> MORSE)
// ============================================================================
void loop() {
    // --- STAGE 1: PLAY QUEUED INCOMING VOICE PACKETS ---
    // Drains circular voiceQueue into MAX98357A speaker first to prevent audio stutter
    while (vqTail != vqHead) {
        playI2SVoicePacket(voiceQueue[vqTail], 64);
        vqTail = (vqTail + 1) % VOICE_QUEUE_SLOTS;
    }

    // --- STAGE 2: PROCESS QUEUED INCOMING MORSE / CLOUD TEXT / SOS ---
    // Safely handles control packets outside the Wi-Fi interrupt callback
    while (cqTail != cqHead) {
        char pktType = ctrlQueue[cqTail].type;
        String payload = String(ctrlQueue[cqTail].payload);
        cqTail = (cqTail + 1) % CTRL_QUEUE_SLOTS;
        handleIncomingData(pktType, payload);
    }

    // --- STAGE 3: MAINTAIN BLYNK CLOUD CONNECTION & RELAY CLOUD TEXT ---
    // Runs only when Wi-Fi is up and voice audio has been idle for >250ms (prevents audio clicks)
    bool audioIdle = ((millis() - lastRxAudioTime) > 250) && ((millis() - lastTxAudioTime) > 250);
    if (enableBlynkCloud && WiFi.status() == WL_CONNECTED && audioIdle) {
        Blynk.run();
    }

    // Transmit any queued Blynk V3 text message out to Unit #2 in clean loop context
    if (pendingCloudText.length() > 0) {
        transmitTextString(pendingCloudText);
        pendingCloudText = "";
    }

    // --- STAGE 4: GPS PARSING, HYBRID PROTOCOL SWITCHING & LORA LISTENING ---
    parseGPS();
    double activeLat = (currentLat != 0.0) ? currentLat : (hasCachedLocation ? lastValidLat : 0.0);
    double activeLon = (currentLon != 0.0) ? currentLon : (hasCachedLocation ? lastValidLon : 0.0);

    // Compute distance to peer and decide between ESP-NOW (<150m) and 433 MHz LoRa (>=150m)
    distanceMeters = calculateDistance(activeLat, activeLon, peerLat, peerLon);
    
    if (indoorTestingMode) {
        useEspNow = true; 
    } else {
        useEspNow = (distanceMeters < 150.0 && peerLat != 0.0); 
    }

    // Broadcast own GPS coordinates and update Blynk map every 10 seconds
    if (millis() - lastGpsBroadcast > GPS_INTERVAL) {
        if (activeLat != 0.0 && activeLon != 0.0) {
            if (useEspNow) {
                outgoingData.type = 'G';
                outgoingData.seq = 0;
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
        pushCloudMapAndStatus(activeLat, activeLon);

        lastGpsBroadcast = millis();
        updateDisplay();
    }

    // Poll SX1278 LoRa radio for long-range packets ("EZ:<TYPE>:<PAYLOAD>")
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
            } else if (pktType == 'S' || pktType == 'M' || pktType == 'T') {
                handleIncomingData(pktType, payload);
            }
        }
    }

    // --- STAGE 5: HALF-DUPLEX 8kHz VOICE CAPTURE & TRANSMISSION ---
    // Muted while speaker is playing (+280ms echo tail) and for 60ms after Morse TX
    bool isSpeakerActive = (millis() - lastRxAudioTime) < ECHO_HOLD_MS;
    if (useEspNow && !isSpeakerActive && (millis() - lastMorseTxTime > 60)) {
        outgoingData.type = 'A';
        outgoingData.seq = 0;
        float packetEnergySum = 0.0f;
        
        // Sample 64 audio frames at exact 8000 Hz (1 sample every 125 microseconds)
        for (int i = 0; i < 64; i++) {
            unsigned long startMicros = micros();
            float rawCentered = (float)(analogRead(micPin) - micCenter);
            
            float voiceSample = filterHumanVoice(rawCentered); // Apply 3-stage DSP filter
            packetEnergySum += fabsf(voiceSample);
            
            if (fabsf(voiceSample) < 40.0f) {
                outgoingData.payload[i] = 128; // Encode quiet floor as exact 8-bit silence (128)
            } else {
                int clamped = constrain((int)voiceSample, -1100, 1100);
                outgoingData.payload[i] = map(clamped, -1100, 1100, 0, 255); // Compress to 8-bit PCM
            }
            
            while (micros() - startMicros < 125); // Busy-wait to maintain exact 8 kHz sample clock
        }
        
        // Voice Activity Detection (VAD): only transmit packet if speech energy exceeds noise gate
        float avgVoiceEnergy = packetEnergySum / 64.0f;
        if (avgVoiceEnergy > VOICE_GATE_THRESHOLD) {
            lastTxAudioTime = millis();
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
        }
    }

    // --- STAGE 6: MORSE BUTTON INPUTS, SHORTCUTS & AUTO-WHITESPACE TIMERS ---
    bool dotPressed = (digitalRead(DOT_PIN) == LOW);
    bool dashPressed = (digitalRead(DASH_PIN) == LOW);

    // Shortcut A: Hold BOTH buttons for 5 seconds -> Clear TX, RX, and Blynk SOS state
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
            rxMessage = "None";
            waitingForRxWord = false;
            playI2STone(300, 250);
            if (isCloudReady()) {
                Blynk.virtualWrite(V1, "(Cleared)");
                Blynk.virtualWrite(V0, 0);
            }
            showFeedbackOLED("All text cleared!");
            bothActionFired = true;
        }
        return; 
    } else {
        bothPressTime = 0;
        bothActionFired = false;
    }

    // Shortcut B: Hold DOT button for 2 seconds -> Backspace last character on TX
    if (dotPressed) {
        if (dotPressTime == 0) {
            dotPressTime = millis();
            dotActionFired = false;
        } else if (!dotActionFired && (millis() - dotPressTime >= BACKSPACE_HOLD)) {
            if (assembledWord.length() > 0) {
                assembledWord.remove(assembledWord.length() - 1);
                playI2STone(400, 150);
                if (isCloudReady()) Blynk.virtualWrite(V1, "TX: " + assembledWord);
                showFeedbackOLED("Backspace!");
            }
            dotActionFired = true; 
        }
    } else {
        // Short release of DOT button (<2s, >=25ms debounce) -> Append '.' to current Morse letter
        if (dotPressTime > 0) {
            unsigned long holdDuration = millis() - dotPressTime;
            if (!dotActionFired && holdDuration >= DEBOUNCE_MIN_MS && holdDuration < BACKSPACE_HOLD) {
                currentMorse += ".";
                lastPressTime = millis();
                waitingForNextWord = false;
                playI2STone(650, 50); // Short 50ms sidetone beep
                updateDisplay();
            }
            dotPressTime = 0;
            dotActionFired = false;
        }
    }

    // Shortcut C: Hold DASH button for 2 seconds -> Clear RX message line back to "None"
    if (dashPressed) {
        if (dashPressTime == 0) {
            dashPressTime = millis();
            dashActionFired = false;
        } else if (!dashActionFired && (millis() - dashPressTime >= BACKSPACE_HOLD)) {
            rxMessage = "None";
            waitingForRxWord = false;
            playI2STone(400, 150);
            if (isCloudReady()) Blynk.virtualWrite(V1, "RX: (Cleared)");
            showFeedbackOLED("RX cleared!");
            dashActionFired = true; 
        }
    } else {
        // Short release of DASH button (<2s, >=25ms debounce) -> Append '-' to current Morse letter
        if (dashPressTime > 0) {
            unsigned long holdDuration = millis() - dashPressTime;
            if (!dashActionFired && holdDuration >= DEBOUNCE_MIN_MS && holdDuration < BACKSPACE_HOLD) {
                currentMorse += "-";
                lastPressTime = millis();
                waitingForNextWord = false;
                playI2STone(650, 140); // Longer 140ms sidetone beep
                updateDisplay();
            }
            dashPressTime = 0;
            dashActionFired = false;
        }
    }

    // Letter Commit Timer: 1.0s after last tap, decode dots/dashes into an ASCII character and transmit
    if (currentMorse.length() > 0 && (millis() - lastPressTime > CHAR_TIMEOUT)) {
        if (currentMorse.equals("...---...")) {
            triggerSOS(activeLat, activeLon); // Special distress code SOS triggers emergency broadcast
        } else {
            char translatedChar = decodeMorse(currentMorse);
            if (assembledWord.length() < 38) assembledWord += translatedChar;
            transmitData(translatedChar);
            if (isCloudReady()) {
                Blynk.virtualWrite(V1, "TX: " + assembledWord);
            }
        }

        currentMorse = ""; 
        lastPressTime = millis(); 
        waitingForNextWord = true;
        updateDisplay();
    }

    // TX Auto-Whitespace Timer: adds a space after 2.2s of inactivity on local transmitter
    if (waitingForNextWord && (millis() - lastPressTime > WORD_TIMEOUT)) {
        if (assembledWord.length() < 38 && !assembledWord.endsWith(" ")) {
            if (assembledWord.length() != 17) assembledWord += " "; // Avoids blank space at OLED line wrap
        }
        waitingForNextWord = false; 
        updateDisplay();
    }

    // RX Auto-Whitespace Timer: adds a space after 4.2s of no incoming Morse character
    if (waitingForRxWord && (millis() - lastRxMorseTime > RX_WORD_TIMEOUT)) {
        if (rxMessage != "None" && rxMessage != "!! SOS ALERT !!" &&
            rxMessage.length() < 38 && !rxMessage.endsWith(" ")) {
            if (rxMessage.length() != 17) rxMessage += " ";
        }
        waitingForRxWord = false;
        updateDisplay();
    }
}

// ============================================================================
// TRANSMISSION, RECEPTION & DISPLAY HELPER FUNCTIONS
// ============================================================================

// Sends a single decoded Morse character via ESP-NOW (3x burst) or LoRa
void transmitData(char c) {
    lastMorseTxTime = millis();
    if (useEspNow) {
        outgoingData.type = 'M';
        outgoingData.seq = nextSeqId();
        memset(outgoingData.payload, 0, sizeof(outgoingData.payload));
        outgoingData.payload[0] = (uint8_t)c;
        for (int i = 0; i < 3; i++) {
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
            delay(4);
        }
    } else {
        LoRa.beginPacket();
        LoRa.print("EZ:M:"); 
        LoRa.print(c);
        LoRa.endPacket();
    }
}

// Sends a full text string (typed from Blynk V3) via ESP-NOW (3x burst with 15ms spacing) or LoRa
void transmitTextString(String text) {
    lastMorseTxTime = millis();
    String clipped = text.substring(0, 38);
    if (useEspNow) {
        outgoingData.type = 'T';
        outgoingData.seq = nextSeqId();
        memset(outgoingData.payload, 0, sizeof(outgoingData.payload));
        clipped.toCharArray((char*)outgoingData.payload, 63);
        for (int i = 0; i < 3; i++) {
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
            delay(15);
        }
    } else {
        LoRa.beginPacket();
        LoRa.print("EZ:T:");
        LoRa.print(clipped);
        LoRa.endPacket();
    }
}

// Broadcasts an SOS distress packet with GPS coordinates over active radio + Blynk Cloud
void triggerSOS(double lat, double lon) {
    lastMorseTxTime = millis();
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
        outgoingData.seq = nextSeqId();
        sprintf((char*)outgoingData.payload, "EMERGENCY SOS");
        outgoingData.lat = lat;
        outgoingData.lon = lon;
        for (int i = 0; i < 3; i++) {
            esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
            delay(15);
        }
    } else {
        LoRa.beginPacket();
        LoRa.print("EZ:S:"); 
        LoRa.print(lat, 4);
        LoRa.print(",");
        LoRa.print(lon, 4);
        LoRa.endPacket();
    }

    if (isCloudReady()) {
        Blynk.virtualWrite(V0, 1);
        Blynk.virtualWrite(V1, "!! EMERGENCY SOS ALERT !!");
        pushCloudMapAndStatus(lat, lon);
    }

    playI2STone(2000, 1000); // High-priority 2000 Hz alert tone
    delay(1000);
    updateDisplay();
}

// ESP-NOW Hardware Interrupt Callback: copies incoming packets into lock-free ring buffers
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len) {
    if (len < sizeof(struct_message)) return;
    memcpy(&incomingData, incoming, sizeof(incomingData));
    if (incomingData.magic[0] != 'E' || incomingData.magic[1] != 'Z') return; // Verify "EZ" header
    
    // Voice packet -> push to voiceQueue for loop() playback
    if (incomingData.type == 'A') {
        lastRxAudioTime = millis();
        uint8_t nextHead = (vqHead + 1) % VOICE_QUEUE_SLOTS;
        if (nextHead != vqTail) {
            memcpy(voiceQueue[vqHead], incomingData.payload, 64);
            vqHead = nextHead;
        }
        return;
    }

    // GPS packet -> update peer coordinates immediately
    if (incomingData.type == 'G') {
        peerLat = incomingData.lat;
        peerLon = incomingData.lon;
        return;
    }

    // Morse ('M'), Cloud Text ('T'), or SOS ('S') -> deduplicate via seq ID and push to ctrlQueue
    if (incomingData.type == 'S' || incomingData.type == 'M' || incomingData.type == 'T') {
        if (incomingData.seq != 0 && incomingData.seq == lastRxSeq) return;
        lastRxSeq = incomingData.seq;
        uint8_t nextHead = (cqHead + 1) % CTRL_QUEUE_SLOTS;
        if (nextHead != cqTail) {
            ctrlQueue[cqHead].type = incomingData.type;
            memcpy(ctrlQueue[cqHead].payload, incomingData.payload, 64);
            ctrlQueue[cqHead].payload[63] = '\0';
            cqHead = nextHead;
        }
    }
}

// Updates OLED, plays notification tone, and logs to Blynk when Morse/Text/SOS arrives
void handleIncomingData(char type, String payload) {
    if (type == 'S') {
        rxMessage = "!! SOS ALERT !!";
        waitingForRxWord = false;
        if (isCloudReady()) {
            Blynk.virtualWrite(V0, 1);
            Blynk.virtualWrite(V1, "RX: !! SOS ALERT !!");
        }
        playI2STone(2500, 1000); 
        updateDisplay();
    } else if (type == 'M') {
        if (rxMessage == "None" || rxMessage == "!! SOS ALERT !!") rxMessage = "";
        for (int i = 0; i < payload.length(); i++) {
            if (rxMessage.length() == 17 && payload[i] == ' ') continue; 
            if (rxMessage.length() < 38) rxMessage += payload[i];
        }
        lastRxMorseTime = millis();
        waitingForRxWord = true;
        if (isCloudReady()) {
            Blynk.virtualWrite(V1, "RX: " + rxMessage);
        }
        playI2STone(1000, 150); 
        updateDisplay();
    } else if (type == 'T') {
        rxMessage = "";
        waitingForRxWord = false;
        for (int i = 0; i < payload.length(); i++) {
            if (rxMessage.length() == 17 && payload[i] == ' ') continue;
            if (rxMessage.length() < 38) rxMessage += payload[i];
        }
        if (isCloudReady()) {
            Blynk.virtualWrite(V1, "RX: " + rxMessage);
        }
        playI2STone(1000, 150);
        updateDisplay();
    }
}

// Searches morseTable[] to convert dot-dash string into a character ('?' if invalid)
char decodeMorse(String code) {
    int tableSize = sizeof(morseTable) / sizeof(morseTable[0]);
    for (int i = 0; i < tableSize; i++) {
        if (code.equals(morseTable[i].code)) return morseTable[i].letter;
    }
    return '?'; 
}

// Renders the complete 6-row UI on the 128x64 OLED display
void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(34, 0); 
    display.print("Echo: Zero");

    // Top-right status badge: [B] = Wi-Fi + Blynk Cloud active, [W] = Wi-Fi active only
    if (isCloudReady()) {
        display.setCursor(110, 0);
        display.print("[B]");
    } else if (WiFi.status() == WL_CONNECTED) {
        display.setCursor(110, 0);
        display.print("[W]");
    }

    // Row 1 (y=8): Active radio protocol (ESP-NOW or LoRa) and distance to peer
    display.setCursor(0, 8);
    display.printf("%s | Dist:%dm", useEspNow ? "ESP-NOW" : "LoRa", (int)distanceMeters);
    
    // Row 2 (y=16): Live GPS fix, Cached GPS fix (*), or satellite search status
    display.setCursor(0, 16);
    if (currentLat != 0.0) {
        display.printf("Lat:%.2f Lon:%.2f", currentLat, currentLon);
    } else if (hasCachedLocation) {
        display.printf("Lat*:%.2f Lon*:%.2f", lastValidLat, lastValidLon); 
    } else {
        display.print("Loc: Searching sat..");
    }

    // Rows 3-4 (y=24..39): Received message buffer (wraps across 2 lines, max 38 chars)
    display.setCursor(0, 24);
    display.print("RX: ");
    display.print(rxMessage);
    
    // Rows 5-6 (y=40..55): Transmitted message buffer
    display.setCursor(0, 40);
    display.print("TX: ");
    display.print(assembledWord);
    
    // Bottom Row (y=56): Live dot-dash buffer + instant character preview [A]
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

// Temporarily displays an action confirmation ("Backspace!", "RX cleared!", etc.) for 800ms
void showFeedbackOLED(String msg) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(34, 0); 
    display.print("Echo: Zero");

    if (isCloudReady()) {
        display.setCursor(110, 0);
        display.print("[B]");
    } else if (WiFi.status() == WL_CONNECTED) {
        display.setCursor(110, 0);
        display.print("[W]");
    }

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

// Reads incoming NMEA bytes from NEO-6M GPS over UART2 and caches valid fixes to NVS flash
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

// Haversine Formula: calculates great-circle distance (in meters) between two GPS coordinates
float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    if (lat2 == 0.0 || lon2 == 0.0 || lat1 == 0.0) return 9999.0;
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371000.0 * c; // 6,371,000 meters = Earth's mean radius
}
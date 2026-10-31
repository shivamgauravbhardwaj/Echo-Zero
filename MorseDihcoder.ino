#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DOT_PIN = 18;  
const int DASH_PIN = 19; 

String currentMorse = "";        
String assembledWord = "";       
unsigned long lastPressTime = 0; 
const unsigned long CHAR_TIMEOUT = 700;  // 700ms pause finishes a letter
const unsigned long WORD_TIMEOUT = 2200; // 2.2 seconds pause adds a space
bool waitingForNextWord = false;

// Timers and flags for hold actions
unsigned long dotPressTime = 0;
bool dotActionFired = false;

unsigned long dashPressTime = 0;
bool dashActionFired = false;

unsigned long bothPressTime = 0;
bool bothActionFired = false;

const unsigned long BACKSPACE_HOLD = 2000; // Hold 2s for backspace
const unsigned long CLEAR_ALL_HOLD = 5000; // Hold 5s for clear all

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

// Function prototypes so compiler scope rules are satisfied
char decodeMorse(String code);
void showFeedback(String msg);
void updateDisplay();

void setup() {
    Serial.begin(115200);
    pinMode(DOT_PIN, INPUT_PULLUP);
    pinMode(DASH_PIN, INPUT_PULLUP);

    lcd.init();
    lcd.backlight();
    updateDisplay();
}

void loop() {
    bool dotPressed = (digitalRead(DOT_PIN) == LOW);
    bool dashPressed = (digitalRead(DASH_PIN) == LOW);

    // 1. Check if BOTH buttons are held down simultaneously
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
            showFeedback("   Cleared All! ");
            bothActionFired = true;
        }
        return; // Skip normal button handling while holding both
    } else {
        bothPressTime = 0;
        bothActionFired = false;
    }

    // 2. Dot Button Logic (Hold for Backspace, Tap for Dot)
    if (dotPressed) {
        if (dotPressTime == 0) {
            dotPressTime = millis();
            dotActionFired = false;
        } else if (!dotActionFired && (millis() - dotPressTime >= BACKSPACE_HOLD)) {
            if (assembledWord.length() > 0) {
                assembledWord.remove(assembledWord.length() - 1);
                showFeedback("   Backspace    ");
            }
            dotActionFired = true; // Prevent repeating while held
        }
    } else {
        if (dotPressTime > 0) {
            unsigned long holdDuration = millis() - dotPressTime;
            if (!dotActionFired && holdDuration < BACKSPACE_HOLD) {
                currentMorse += ".";
                lastPressTime = millis();
                waitingForNextWord = false;
                updateDisplay();
            }
            dotPressTime = 0;
            dotActionFired = false;
        }
    }

    // 3. Dash Button Logic (Hold for Backspace, Tap for Dash)
    if (dashPressed) {
        if (dashPressTime == 0) {
            dashPressTime = millis();
            dashActionFired = false;
        } else if (!dashActionFired && (millis() - dashPressTime >= BACKSPACE_HOLD)) {
            if (assembledWord.length() > 0) {
                assembledWord.remove(assembledWord.length() - 1);
                showFeedback("   Backspace    ");
            }
            dashActionFired = true; // Prevent repeating while held
        }
    } else {
        if (dashPressTime > 0) {
            unsigned long holdDuration = millis() - dashPressTime;
            if (!dashActionFired && holdDuration < BACKSPACE_HOLD) {
                currentMorse += "-";
                lastPressTime = millis();
                waitingForNextWord = false;
                updateDisplay();
            }
            dashPressTime = 0;
            dashActionFired = false;
        }
    }

    // 4. Character completion timeout (700ms pause)
    if (currentMorse.length() > 0 && (millis() - lastPressTime > CHAR_TIMEOUT)) {
        char translatedChar = decodeMorse(currentMorse);
        
        if (assembledWord.length() < 16) {
            assembledWord += translatedChar;
        } else {
            assembledWord = String(translatedChar); 
        }
        
        currentMorse = ""; 
        lastPressTime = millis(); 
        waitingForNextWord = true;
        updateDisplay();
    }

    // 5. Word completion timeout (2.2 seconds pause adds a space)
    if (waitingForNextWord && (millis() - lastPressTime > WORD_TIMEOUT)) {
        if (assembledWord.length() < 16 && !assembledWord.endsWith(" ")) {
            assembledWord += " "; 
        }
        waitingForNextWord = false; 
        updateDisplay();
    }
}

char decodeMorse(String code) {
    int tableSize = sizeof(morseTable) / sizeof(morseTable[0]);
    for (int i = 0; i < tableSize; i++) {
        if (code.equals(morseTable[i].code)) {
            return morseTable[i].letter;
        }
    }
    return '?';
}

void showFeedback(String msg) {
    lcd.setCursor(0, 0);
    lcd.print("                "); 
    lcd.setCursor(0, 0);
    lcd.print(msg);
    delay(800);
    updateDisplay();
}

void updateDisplay() {
    lcd.setCursor(0, 0);
    lcd.print("                "); 
    lcd.setCursor(0, 0);
    lcd.print(assembledWord);

    lcd.setCursor(0, 1);
    lcd.print("                "); 
    lcd.setCursor(0, 1);
    lcd.print("Morse: ");
    lcd.print(currentMorse);
}
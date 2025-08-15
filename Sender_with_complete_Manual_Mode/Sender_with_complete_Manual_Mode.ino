#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>              // GFX লাইব্রেরি
#include <Fonts/FreeSansBold9pt7b.h>

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
#define CHANNEL 1
#define CURSOR_UP_PIN 14    // Cursor Up
#define CURSOR_DOWN_PIN 15  // Cursor Down
#define ANGLE_UP_PIN 16     // Angle Up
#define ANGLE_DOWN_PIN 17   // Angle Down
#define ENTER_PIN 18        // Enter Button

// Joystick pins for manual mode
#define BUTTON1_PIN 19
#define BUTTON2_PIN 4
#define BUTTON3_PIN 27
#define BUTTON4_PIN 26
#define JOYSTICK1_X_PIN 34
#define JOYSTICK1_Y_PIN 35
#define JOYSTICK2_X_PIN 32
#define JOYSTICK2_Y_PIN 33
#define DISPLAY_RESET_PIN 23 // Display Reset Button

// Receiver MAC Address
uint8_t receiverMacAddress[] = {0x94, 0x54, 0xC5, 0x75, 0x89, 0xD4};

// Antenna icon (8x8 pixels) for connection status
const unsigned char antennaIcon[] PROGMEM = {
    0b00011000,
    0b00011000,
    0b00111100,
    0b00111100,
    0b01111110,
    0b01111110,
    0b00011000,
    0b00011000
};

// Structure to send control data to Receiver
typedef struct {
    uint8_t servo1Signal; // Buttons: 1 (Up), 2 (Down), 0 (None)
    uint8_t servo2Signal; // Joystick 1 X: 3 (Right), 4 (Left), 0 (None)
    uint8_t servo3Signal; // Joystick 1 Y: 6 (Up), 7 (Down), 0 (None)
    uint8_t servo4Signal; // Joystick 2 X: 9 (Right), 10 (Left), 0 (None)
    uint8_t servo5Signal; // Joystick 2 Y: 12 (Up), 13 (Down), 0 (None)
    uint8_t servo6Signal; // Buttons: 15 (Up), 16 (Down), 0 (None)
    uint8_t mode;        // 0: Operating, 1: Manual, 2: Auto
    int initialAngles[6]; // Auto mode initial angles
    int finalAngles[6];   // Auto mode final angles
    bool servoRunning;    // Auto mode running state
    bool restartRequested; // Auto mode restart flag
} ServoControl;
ServoControl servoData;

// Structure to receive servo angles from Receiver
typedef struct {
    int angle1;
    int angle2;
    int angle3;
    int angle4;
    int angle5;
    int angle6;
    bool servoRunning;
    bool restartRequested;
} AngleData;
AngleData angleData;

// ESP-NOW peer
esp_now_peer_info_t slave;

// System mode
enum SystemMode { OPERATING, MANUAL, AUTO };
SystemMode currentMode = OPERATING;

// UI state
int currentMenu = 0; // 0: Servo Select, 1: Angle Set
int selectedServo = 1; // 1 to 6
int cursorPosition = 0; 
unsigned long lastButtonPress = 0; // Tracks the last time a button was pressed for debouncing
bool isEspNowConnected = false; // Tracks ESP-NOW connection status
bool isFirstRun = true; // Tracks if this is the first run after power-on
bool anglesChanged = false; // Tracks if angles were changed during running
bool restartPressed = false; // Tracks if Restart was pressed

// Servo reset variables
int currentResetServo = 1; // Tracks which servo is being reset (1 to 6)
unsigned long servoResetStartTime = 0; // Tracks start time of servo reset
const int servoResetDelay = 500; // Delay between servo resets (ms)
int angleChangeCount = 0; // Tracks number of angle changes in Auto mode

unsigned long restartToastUntil = 0; // show "Reseted" overlay until this time


// Button timing variables
const int debounceDelay = 50; // Debounce delay for button press (ms)
bool showPress1secMessage = false; // Tracks if "Press 1sec" message should be shown
unsigned long pressStartTime = 0; // Tracks when ENTER_PIN is pressed
bool enterActionTriggered = false; // Tracks if an ENTER action has been triggered
bool isReturningToOperating = false; // Tracks if returning to Operating mode
bool isServoResetting = false; // Tracks if servos are resetting to 90 degrees
unsigned long lastRepeatTime = 0; // Tracks the last time a repeated action was processed
bool manualResetTriggered = false; // Tracks if manual mode reset is triggered
unsigned long pauseStartTime = 0; // Tracks the start time of pause before reset
unsigned long lastJoystickPulseTime = 0; // Tracks the last time a joystick pulse was sent


//Servo return speed from Manual to Operation Mode
const int resetStepSize = 1; // প্রতিটি ধাপে সার্ভোর কোণ পরিবর্তন হবে (slow = decrease)
const int resetStepDelay = 100; // প্রতিটি ধাপের মধ্যে বিলম্ব থাকবে (slow = increase)
const int pauseBeforeReset = 500; // 0.5 seconds pause before moves to the 90 when we out from one Mode to another Mode


//Servo delay with Joystick button (low-delay=Decrease)
const int joystickPulseDuration = 150; // Duration of joystick signal pulse (ms)


//Auto মোড এ একটানা বোতাম চাপলে কতক্ষণ পর কারসর নোড়বে
const int repeatDelay = 200; // Delay between repeated actions for continuous press (ms)

//Manual মোড এ Back বোতাম চাপার কতক্ষণ পর অ্যাকশন হবে
const int backButtonHoldThreshold = 2000; // 2 seconds for back button continuous press

//Back বোতাম কতক্ষণ প্রেস করে রাখলে ডিসপ্লেতে “Press 2sec” মেসেজ দেখাবে
const int backButtonShortPressThreshold = 2000; // 2 seconds for showing "Press 2sec" // 

// Auto mode variables
int initialAngles[6] = {90, 90, 90, 90, 90, 90};
int finalAngles[6] = {90, 90, 90, 90, 90, 90};
bool servoRunning = false;



// Function prototypes
float getAverageAnalogValue(int pin);
void sendData();
void displayOperatingMode();
void displayManualMode();
void displayAutoMode();
void handleButtons();
int servoInitialAngle();
int servoFinalAngle();
void setServoInitialAngle(int);
void setServoFinalAngle(int);




// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    isEspNowConnected = (status == ESP_OK);
    if (!isEspNowConnected) {
        Serial.println("Delivery failed");
    }
}

// ESP-NOW receive callback
 void OnDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(AngleData)) {
        memcpy(&angleData, data, sizeof(AngleData));
        Serial.print("Received Angles: ");
        Serial.print(angleData.angle1); Serial.print(", ");
        Serial.print(angleData.angle2); Serial.print(", ");
        Serial.print(angleData.angle3); Serial.print(", ");
        Serial.print(angleData.angle4); Serial.print(", ");
        Serial.print(angleData.angle5); Serial.print(", ");
        Serial.println(angleData.angle6);
        if (currentMode == AUTO) {
            servoRunning = angleData.servoRunning;
            restartPressed = angleData.restartRequested;
            if (!servoRunning && !restartPressed) {
                anglesChanged = false;
                angleChangeCount = 0;
            }
            Serial.println("Received: servoRunning=" + String(servoRunning) + ", restartRequested=" + String(angleData.restartRequested));
        }
    } else {
        Serial.println("Invalid data length");
    }
}


// Send control data to Receiver
void sendData() {
    const int maxRetries = 3;
    int retries = 0;
    esp_err_t result;
    do {
        result = esp_now_send(slave.peer_addr, (uint8_t *)&servoData, sizeof(servoData));
        if (result == ESP_OK) {
            Serial.print("Sending Data: S1="); Serial.print(servoData.servo1Signal);
            Serial.print(" S2="); Serial.print(servoData.servo2Signal);
            Serial.print(" S3="); Serial.print(servoData.servo3Signal);
            Serial.print(" S4="); Serial.print(servoData.servo4Signal);
            Serial.print(" S5="); Serial.print(servoData.servo5Signal);
            Serial.print(" S6="); Serial.print(servoData.servo6Signal);
            Serial.print(" Mode="); Serial.println(servoData.mode);
            break;
        }
        Serial.print("Send error: "); Serial.println(result);
        retries++;
        delay(10);
    } while (retries < maxRetries);
    if (result != ESP_OK) {
        Serial.println("Failed to send data after retries");
    }
}

// Display Operating mode menu
void displayOperatingMode() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Select Mode:");
    display.println(cursorPosition == 0 ? "> Manual" : "  Manual");
    display.println(cursorPosition == 1 ? "> Auto" : "  Auto");
    if (isEspNowConnected) {
        display.drawBitmap(120, 0, antennaIcon, 8, 8, SSD1306_WHITE);
    }
    display.display();
}

// Display Manual mode
void displayManualMode() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Manual Mode");
    display.print("Servo-1: "); display.println(angleData.angle1);
    display.print("Servo-2: "); display.println(angleData.angle2);
    display.print("Servo-3: "); display.println(angleData.angle3);
    display.print("Servo-4: "); display.println(angleData.angle4);
    display.print("Servo-5: "); display.println(angleData.angle5);
    display.print("Servo-6: "); display.println(angleData.angle6);
    if (isEspNowConnected) {
        display.drawBitmap(120, 0, antennaIcon, 8, 8, SSD1306_WHITE);
    }
    // Draw Back button
    display.setCursor(5, 56);
    display.print("Back");
    display.drawRect(2, 54, 28, 11, SSD1306_WHITE);
    // Show "Press 1sec" message if triggered
    if (showPress1secMessage) {
        display.setCursor(40, 56);
        display.print("Press 2sec");
    }
    display.display();
}

// Display Auto mode
// Display Auto mode
void displayAutoMode() {
    display.clearDisplay();
    display.fillScreen(SSD1306_BLACK); // Ensure screen is fully cleared
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (isEspNowConnected) {
        display.drawBitmap(120, 0, antennaIcon, 8, 8, SSD1306_WHITE);
    }

    if (currentMenu == 0) { // Servo Select menu
        display.fillScreen(SSD1306_BLACK); // Clear screen for Servo Select
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        if (isEspNowConnected) {
            display.drawBitmap(120, 0, antennaIcon, 8, 8, SSD1306_WHITE);
        }
        display.setCursor(0, 0);
        display.println("Select Servo:");
        for (int i = 0; i < 6; i++) {
            display.setCursor(5, 10 + i * 8);
            display.print(cursorPosition == i ? "> Servo-" : "  Servo-");
            display.print(i + 1);
            display.setCursor(80, 10 + i * 8);
            display.print(initialAngles[i]);
            display.setCursor(100, 10 + i * 8);
            display.print(finalAngles[i]);
        }
        // --- Bottom buttons: Back (idx 8), Restart (idx 6), Start/Stop (idx 7)
String startStopText = servoRunning ? "Stop" : "Start";

// Back (left)
display.setCursor(6, 56);
display.print("Back");
if (cursorPosition == 8) {
    display.drawRect(2, 54, 30, 12, SSD1306_WHITE);
    display.drawRect(1, 53, 32, 14, SSD1306_WHITE);
}

// Restart (middle)
display.setCursor(52, 56);
display.print("Reset");
if (cursorPosition == 6) {
    display.drawRect(48, 54, 40, 10, SSD1306_WHITE);
    display.drawRect(47, 53, 42, 12, SSD1306_WHITE);
}

// Start/Stop (right)
int rightX = 100;
int boxW = (startStopText == "Stop") ? 28 : 32;
display.setCursor(rightX, 56);
display.print(startStopText);
if (cursorPosition == 7) {
    display.drawRect(rightX - 2, 54, boxW, 12, SSD1306_WHITE);
    display.drawRect(rightX - 3, 53, boxW + 2, 14, SSD1306_WHITE);
}

    } else { // Angle Set menu
        display.fillScreen(SSD1306_BLACK); // Clear screen for Angle Set
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        if (isEspNowConnected) {
            display.drawBitmap(120, 0, antennaIcon, 8, 8, SSD1306_WHITE);
        }
        display.setCursor(0, 0);
        display.print("Servo-");
        display.print(selectedServo);
        display.println(":");
        display.drawRect(10, 16, 40, 12, SSD1306_WHITE);
        if (cursorPosition == 0) {
            display.drawRect(9, 15, 42, 14, SSD1306_WHITE);
            display.drawRect(8, 14, 44, 16, SSD1306_WHITE);
        }
        int initialTextX = 10 + (40 - (String(servoInitialAngle()).length() * 6)) / 2;
        display.setCursor(initialTextX, 18);
        display.print(servoInitialAngle());
        if (servoInitialAngle() < 180) {
            display.drawTriangle(55, 20, 59, 20, 57, 18, SSD1306_WHITE); // Up arrow
        }
        if (servoInitialAngle() > 0) {
            display.drawTriangle(55, 24, 59, 24, 57, 26, SSD1306_WHITE); // Down arrow
        }
        display.drawRect(70, 16, 40, 12, SSD1306_WHITE);
        if (cursorPosition == 1) {
            display.drawRect(69, 15, 42, 14, SSD1306_WHITE);
            display.drawRect(68, 14, 44, 16, SSD1306_WHITE);
        }
        int finalTextX = 70 + (40 - (String(servoFinalAngle()).length() * 6)) / 2;
        display.setCursor(finalTextX, 18);
        display.print(servoFinalAngle());
        if (servoFinalAngle() < 180) {
            display.drawTriangle(115, 20, 119, 20, 117, 18, SSD1306_WHITE); // Up arrow
        }
        if (servoFinalAngle() > 0) {
            display.drawTriangle(115, 24, 119, 24, 117, 26, SSD1306_WHITE); // Down arrow
        }
        display.setCursor(10, 32);
        display.println("Initial");
        display.setCursor(70, 32);
        display.println("Final");
        int okTextX = 58;
        display.setCursor(okTextX, 52);
        display.print("OK");
        if (cursorPosition == 2) {
            display.drawRect(54, 50, 20, 12, SSD1306_WHITE);
            display.drawRect(53, 49, 22, 14, SSD1306_WHITE);
            display.drawRect(52, 48, 24, 16, SSD1306_WHITE);
        }
    }
        // ★ Restart হলে 1 সেকেন্ডের জন্য "Reseted" মেসেজ দেখাবে
    if (currentMenu == 0 && millis() < restartToastUntil) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    int textWidth = 6 * 7 * 2; // "Reseted"
    int x = (SCREEN_WIDTH - textWidth) / 2;
    int y = (SCREEN_HEIGHT - 16) / 2;

    display.setCursor(x, y);
    display.print("Reseted");

    display.setTextSize(1);
    display.display();
}
    display.display();
}


//......................Most Important function "handleButtons()".................................

// Handle button inputs
void handleButtons() {
    unsigned long currentTime = millis();

    // Debug button states
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 500) {
        Serial.print("CURSOR_UP_PIN: "); Serial.println(digitalRead(CURSOR_UP_PIN));
        Serial.print("CURSOR_DOWN_PIN: "); Serial.println(digitalRead(CURSOR_DOWN_PIN));
        Serial.print("ANGLE_UP_PIN: "); Serial.println(digitalRead(ANGLE_UP_PIN));
        Serial.print("ANGLE_DOWN_PIN: "); Serial.println(digitalRead(ANGLE_DOWN_PIN));
        Serial.print("ENTER_PIN: "); Serial.println(digitalRead(ENTER_PIN));
        lastDebugPrint = currentTime;
    }

    // Debounce check
    if (currentTime - lastButtonPress < debounceDelay) return;
    
    // Check if enough time has passed for repeated actions
    if (currentTime - lastRepeatTime < repeatDelay) return;

    // Handle CURSOR_UP_PIN (single press only)
    if (digitalRead(CURSOR_UP_PIN) == LOW) {
        if (currentMode == OPERATING && cursorPosition > 0) {
            cursorPosition--;
            Serial.println("Cursor Up: Position " + String(cursorPosition));
        } else if (currentMode == AUTO) {
            if (currentMenu == 0 && cursorPosition > 0) {
                cursorPosition--;
                Serial.println("Cursor Up: Position " + String(cursorPosition));
            } else if (currentMenu == 1 && cursorPosition > 0) {
                cursorPosition--;
                Serial.println("Cursor Up: Position " + String(cursorPosition));
            }
        }
        lastButtonPress = currentTime;
        lastRepeatTime = currentTime; // Update repeat time
    }

    // Handle CURSOR_DOWN_PIN (single press only)
    if (digitalRead(CURSOR_DOWN_PIN) == LOW) {
        if (currentMode == OPERATING && cursorPosition < 1) {
            cursorPosition++;
            Serial.println("Cursor Down: Position " + String(cursorPosition));
        } else if (currentMode == AUTO) {
            if (currentMenu == 0 && cursorPosition < 8) {
                cursorPosition++;
                Serial.println("Cursor Down: Position " + String(cursorPosition));
            } else if (currentMenu == 1 && cursorPosition < 2) {
                cursorPosition++;
                Serial.println("Cursor Down: Position " + String(cursorPosition));
            }
        }
        lastButtonPress = currentTime;
        lastRepeatTime = currentTime; // Update repeat time
    }

    // Handle ANGLE_UP_PIN (single press only)
    if (digitalRead(ANGLE_UP_PIN) == LOW && currentMode == AUTO && currentMenu == 1) {
        if (cursorPosition == 0 && servoInitialAngle() < 180) {
            setServoInitialAngle(servoInitialAngle() + 5);
            angleChangeCount++;
            if (servoRunning) anglesChanged = true;
            Serial.println("Angle Up: Initial Angle " + String(servoInitialAngle()) + " for Servo-" + String(selectedServo));
        } else if (cursorPosition == 1 && servoFinalAngle() < 180) {
            setServoFinalAngle(servoFinalAngle() + 5);
            angleChangeCount++;
            if (servoRunning) anglesChanged = true;
            Serial.println("Angle Up: Final Angle " + String(servoFinalAngle()) + " for Servo-" + String(selectedServo));
        }
        lastButtonPress = currentTime;
        lastRepeatTime = currentTime; // Update repeat time
    }

    // Handle ANGLE_DOWN_PIN (single press only)
    if (digitalRead(ANGLE_DOWN_PIN) == LOW && currentMode == AUTO && currentMenu == 1) {
        if (cursorPosition == 0 && servoInitialAngle() > 0) {
            setServoInitialAngle(servoInitialAngle() - 5);
            angleChangeCount++;
            if (servoRunning) anglesChanged = true;
            Serial.println("Angle Down: Initial Angle " + String(servoInitialAngle()) + " for Servo-" + String(selectedServo));
        } else if (cursorPosition == 1 && servoFinalAngle() > 0) {
            setServoFinalAngle(servoFinalAngle() - 5);
            angleChangeCount++;
            if (servoRunning) anglesChanged = true;
            Serial.println("Angle Down: Final Angle " + String(servoFinalAngle()) + " for Servo-" + String(selectedServo));
        }
        lastButtonPress = currentTime;
        lastRepeatTime = currentTime; // Update repeat time
    }

    // Handle DISPLAY_RESET_PIN (single press only)
    if (digitalRead(DISPLAY_RESET_PIN) == LOW && currentTime - lastButtonPress >= debounceDelay) {
        display.clearDisplay();
        display.fillScreen(SSD1306_BLACK);
        display.display();
        Serial.println("Display cleared via reset button");
        if (currentMode == OPERATING) {
            displayOperatingMode();
        } else if (currentMode == MANUAL) {
            displayManualMode();
        } else if (currentMode == AUTO) {
            displayAutoMode();
        }
        lastButtonPress = currentTime;
    }

    // Handle ENTER_PIN
    static bool enterPressed = false;
    bool enterState = digitalRead(ENTER_PIN) == LOW;

    Serial.print("ENTER_PIN State: "); Serial.println(enterState ? "LOW" : "HIGH");

    if (enterState && !enterPressed) {
        // Button press started
        enterPressed = true;
        pressStartTime = currentTime;
        enterActionTriggered = false;
        Serial.println("ENTER_PIN: Press started");
    }

    if (enterState && enterPressed && currentMode == MANUAL && !enterActionTriggered) {
        // Check for continuous press in Manual mode
        if (currentTime - pressStartTime >= backButtonHoldThreshold) {
            // 2 seconds pressed, return to Operating mode
            currentMode = OPERATING;
            servoData.mode = 0;
            servoData.servoRunning = false;
            servoData.restartRequested = false;
            isReturningToOperating = true;
            isServoResetting = true;
            currentResetServo = 1;
            servoResetStartTime = currentTime;
            pauseStartTime = currentTime;
            cursorPosition = 0;
            showPress1secMessage = false;
            enterActionTriggered = true;
            // Stop all servo signals
            servoData.servo1Signal = 0;
            servoData.servo2Signal = 0;
            servoData.servo3Signal = 0;
            servoData.servo4Signal = 0;
            servoData.servo5Signal = 0;
            servoData.servo6Signal = 0;
            sendData();
            Serial.println("Enter: 2s press, returning to Operating mode");
        } else if (currentTime - pressStartTime >= 0) {
    int boxX = 12;
    int boxY = 19;
    int boxW = 100;
    int boxH = 20;

    // বক্স আঁকা ও ভর্তি করা
    display.fillRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);

    // Bold টেক্সট সেট করা
    display.setFont(&FreeSansBold9pt7b);
    display.setTextSize(1);
    display.setTextColor(SSD1306_BLACK);

    // প্রথম লাইন: Servo moving!
    display.setCursor(boxX + 3, boxY + 15);
    display.print("Press 2sec");

    display.setFont(); // ডিফল্ট ফন্টে ফিরে যাওয়া

    display.display();
    delay(2000);
        }
    }

    if (!enterState && enterPressed) {
        // Button released
        enterPressed = false;
        lastButtonPress = currentTime;

        if (currentMode == MANUAL && !enterActionTriggered) {
            // Short press in Manual mode
            if (currentTime - pressStartTime < backButtonHoldThreshold) {
                Serial.println("Enter: Short press, no action");
            }
        } else if (currentMode == OPERATING && !enterActionTriggered) {
            // Operating mode selection
            if (cursorPosition == 0) {
                currentMode = MANUAL;
                servoData.mode = 1;
                servoData.servoRunning = false;
                servoData.restartRequested = false;
                isServoResetting = true;
                pauseStartTime = currentTime;
                // Initialize servo signals
                servoData.servo1Signal = 0;
                servoData.servo2Signal = 0;
                servoData.servo3Signal = 0;
                servoData.servo4Signal = 0;
                servoData.servo5Signal = 0;
                servoData.servo6Signal = 0;
                enterActionTriggered = true;
                sendData();
                Serial.println("Enter: Switched to Manual mode, resetting servos to 90");
            } else if (cursorPosition == 1) {
                currentMode = AUTO;
                servoData.mode = 2;
                cursorPosition = 0;
                servoRunning = false;
                restartPressed = false;
                anglesChanged = false;
                angleChangeCount = 0; // Reset angle change counter
                servoData.servoRunning = false;
                servoData.restartRequested = false;
                enterActionTriggered = true;
                sendData();
                Serial.println("Enter: Switched to Auto mode, reset state");
            }
        } else if (currentMode == AUTO && currentMenu == 0 && !enterActionTriggered) {
    if (cursorPosition < 6) {
        // Servo select → Angle Set
        selectedServo = cursorPosition + 1;
        currentMenu = 1;
        cursorPosition = 0;
        enterActionTriggered = true;
        Serial.println("Enter: Angle Setting Menu for Servo-" + String(selectedServo));
    } else if (cursorPosition == 6) {
        if (servoRunning) {
    // বক্সের পজিশন ও সাইজ
    int boxX = 0;
    int boxY = 20;
    int boxW = 127;
    int boxH = 30;

    // বক্স আঁকা ও ভর্তি করা
    display.fillRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);

    // Bold টেক্সট সেট করা
    display.setFont(&FreeSansBold9pt7b);
    display.setTextSize(1);
    display.setTextColor(SSD1306_BLACK);

    // প্রথম লাইন: Servo moving!
    display.setCursor(boxX + 1, boxY + 15);
    display.print("Servo moving!");

    // দ্বিতীয় লাইন ছোট সাইজে: Stop servo first
    display.setFont(); // ডিফল্ট ফন্টে ফিরে যাওয়া
    display.setTextSize(1);
    display.setCursor(boxX + 10, boxY + 21);
    display.print("Please stop servos");

    display.display();
    delay(2000);

    // পরে রঙ আবার আগের মতো করা
    display.setTextColor(SSD1306_WHITE);
} else {
            // Stopped → restart request + inline toast
            restartPressed = true;
            servoData.servoRunning = false;
            servoData.restartRequested = true;
            for (int i = 0; i < 6; i++) {
                servoData.initialAngles[i] = initialAngles[i];
                servoData.finalAngles[i]   = finalAngles[i];
            }
            enterActionTriggered = true;
            sendData();
            restartToastUntil = millis() + 1500;
            Serial.println("Enter: Restart requested (stopped → reset to initial → run)");
        }
    } else if (cursorPosition == 7) {
        // Start/Stop toggle
        servoRunning = !servoRunning;
        restartPressed = false;
        servoData.servoRunning = servoRunning;
        servoData.restartRequested = false;
        for (int i = 0; i < 6; i++) {
            servoData.initialAngles[i] = initialAngles[i];
            servoData.finalAngles[i]   = finalAngles[i];
        }
        enterActionTriggered = true;
        sendData();
        Serial.println(String("Enter: ") + (servoRunning ? "Start" : "Stop"));
    } else if (cursorPosition == 8) {
        // Back
        currentMode = OPERATING;
        servoData.mode = 0;
        servoData.servoRunning = false;
        servoData.restartRequested = false;
        isReturningToOperating = true;
        isServoResetting = true;
        currentResetServo = 1;
        pauseStartTime = currentTime;
        cursorPosition = 0;
        enterActionTriggered = true;
        servoData.servo1Signal = servoData.servo2Signal = 0;
        servoData.servo3Signal = servoData.servo4Signal = 0;
        servoData.servo5Signal = servoData.servo6Signal = 0;
        sendData();
        Serial.println("Enter: Back to Operating from Auto");
    }
}

        }
    
     else if (currentMode == AUTO && currentMenu == 1 && cursorPosition == 2 && !enterActionTriggered) {
            currentMenu = 0;
            cursorPosition = selectedServo - 1;
            enterActionTriggered = true;
            Serial.println("Enter: Servo-" + String(selectedServo) + " angles saved");
            sendData();
        }
        showPress1secMessage = false;
        Serial.println("Enter: Button released");
} // End of handleButtons()


//......................Most Important function "handleButtons()".................................


 // handleManualInputs() ফাংশনের শেষ
void handleManualInputs() {
    unsigned long currentTime = millis();

    // Check if enough time has passed since the last joystick pulse
    if (currentTime - lastJoystickPulseTime < joystickPulseDuration) return;

    // Read button states (active LOW due to INPUT_PULLDOWN)
    bool button1Pressed = digitalRead(BUTTON1_PIN) == HIGH;
    bool button2Pressed = digitalRead(BUTTON2_PIN) == HIGH;
    bool button3Pressed = digitalRead(BUTTON3_PIN) == HIGH;
    bool button4Pressed = digitalRead(BUTTON4_PIN) == HIGH;

    // Read joystick values (analog)
    int joystick1X = analogRead(JOYSTICK1_X_PIN);
    int joystick1Y = analogRead(JOYSTICK1_Y_PIN);
    int joystick2X = analogRead(JOYSTICK2_X_PIN);
    int joystick2Y = analogRead(JOYSTICK2_Y_PIN);

    // Initialize servo signals
    servoData.servo1Signal = 0;
    servoData.servo2Signal = 0;
    servoData.servo3Signal = 0;
    servoData.servo4Signal = 0;
    servoData.servo5Signal = 0;
    servoData.servo6Signal = 0;

    // Handle Button 1 and Button 2 for Servo 1
    if (button1Pressed) {
        servoData.servo1Signal = 1; // Up
        Serial.println("Button 1: Servo 1 Up");
    } else if (button2Pressed) {
        servoData.servo1Signal = 2; // Down
        Serial.println("Button 2: Servo 1 Down");
    }

    // Handle Joystick 1 X for Servo 2
    if (joystick1X > 3000) { // Right
        servoData.servo2Signal = 3;
        Serial.println("Joystick 1 X: Servo 2 Right");
    } else if (joystick1X < 1000) { // Left
        servoData.servo2Signal = 4;
        Serial.println("Joystick 1 X: Servo 2 Left");
    }

    // Handle Joystick 1 Y for Servo 3
    if (joystick1Y > 3000) { // Up
        servoData.servo3Signal = 6;
        Serial.println("Joystick 1 Y: Servo 3 Up");
    } else if (joystick1Y < 1000) { // Down
        servoData.servo3Signal = 7;
        Serial.println("Joystick 1 Y: Servo 3 Down");
    }

    // Handle Joystick 2 X for Servo 4
    if (joystick2X > 3000) { // Right
        servoData.servo4Signal = 9;
        Serial.println("Joystick 2 X: Servo 4 Right");
    } else if (joystick2X < 1000) { // Left
        servoData.servo4Signal = 10;
        Serial.println("Joystick 2 X: Servo 4 Left");
    }

    // Handle Joystick 2 Y for Servo 5
    if (joystick2Y > 3000) { // Up
        servoData.servo5Signal = 12;
        Serial.println("Joystick 2 Y: Servo 5 Up");
    } else if (joystick2Y < 1000) { // Down
        servoData.servo5Signal = 13;
        Serial.println("Joystick 2 Y: Servo 5 Down");
    }

    // Handle Button 3 and Button 4 for Servo 6
    if (button3Pressed) {
        servoData.servo6Signal = 15; // Up
        Serial.println("Button 3: Servo 6 Up");
    } else if (button4Pressed) {
        servoData.servo6Signal = 16; // Down
        Serial.println("Button 4: Servo 6 Down");
    }

    // Update last joystick pulse time if any signal is sent
    if (servoData.servo1Signal || servoData.servo2Signal || servoData.servo3Signal ||
        servoData.servo4Signal || servoData.servo5Signal || servoData.servo6Signal) {
        lastJoystickPulseTime = currentTime;
        sendData();
    }
}
//........................................handleManualInputs() ফাংশনের শেষ.............




//................................এই function টা back বোতাম চাপলে servo গুলোকে ৯০ ডিগ্রীতে নিতে ব্যবহার হয়........................

void resetServosTo90() {
    if (!isServoResetting) return;
    unsigned long currentTime = millis();

    // Check if pause before reset is still active
    if ((isReturningToOperating || manualResetTriggered) && currentTime - pauseStartTime < pauseBeforeReset) {
        return;
    }

    // Set mode
    servoData.mode = isReturningToOperating ? 0 : 1;

    // Interpolate angle for the current servo
    int currentAngle = initialAngles[currentResetServo - 1];
    int targetAngle = 90;

    // Calculate the difference
    int angleDiff = targetAngle - currentAngle;

    // Update the current servo's angle
    if (abs(angleDiff) > resetStepSize) {
        if (angleDiff > 0) {
            // Move towards 90 degrees
            servoData.initialAngles[currentResetServo - 1] = currentAngle + resetStepSize;
            servoData.finalAngles[currentResetServo - 1] = currentAngle + resetStepSize;
            initialAngles[currentResetServo - 1] = currentAngle + resetStepSize;
            finalAngles[currentResetServo - 1] = currentAngle + resetStepSize;
        } else {
            // Move towards 90 degrees
            servoData.initialAngles[currentResetServo - 1] = currentAngle - resetStepSize;
            servoData.finalAngles[currentResetServo - 1] = currentAngle - resetStepSize;
            initialAngles[currentResetServo - 1] = currentAngle - resetStepSize;
            finalAngles[currentResetServo - 1] = currentAngle - resetStepSize;
        }
        // Delay for smooth movement
        delay(resetStepDelay);
    } else {
        // Set to exact target angle
        servoData.initialAngles[currentResetServo - 1] = targetAngle;
        servoData.finalAngles[currentResetServo - 1] = targetAngle;
        initialAngles[currentResetServo - 1] = targetAngle;
        finalAngles[currentResetServo - 1] = targetAngle;
        // Move to the next servo
        currentResetServo++;
    }

    // Clear servo signals to stop all servos immediately
    servoData.servo1Signal = 0;
    servoData.servo2Signal = 0;
    servoData.servo3Signal = 0;
    servoData.servo4Signal = 0;
    servoData.servo5Signal = 0;
    servoData.servo6Signal = 0;

    // Send updated angles
    sendData();

    // Check if all servos have been reset
    if (currentResetServo > 6) {
        isServoResetting = false;
        isReturningToOperating = false;
        manualResetTriggered = false;
        currentResetServo = 1;
        Serial.println("Reset all servos to 90 degrees for " + String(servoData.mode == 1 ? "Manual" : "Operating") + " mode");
    }
}

//..............................finish resetServosTo90 function...................



// Get initial angle for selected servo
int servoInitialAngle() {
    return initialAngles[selectedServo - 1];
}

// Get final angle for selected servo
int servoFinalAngle() {
    return finalAngles[selectedServo - 1];
}

// Set initial angle for selected servo
void setServoInitialAngle(int angle) {
    angle = constrain(angle, 0, 180);
    initialAngles[selectedServo - 1] = angle;
    servoData.initialAngles[selectedServo - 1] = angle;
}

// Set final angle for selected servo
void setServoFinalAngle(int angle) {
    angle = constrain(angle, 0, 180);
    finalAngles[selectedServo - 1] = angle;
    servoData.finalAngles[selectedServo - 1] = angle;
}

// Setup function
void setup() {
    Serial.begin(115200);
    Serial.println("Sender Started");

    // Initialize pins
    pinMode(JOYSTICK1_X_PIN, INPUT);
    pinMode(JOYSTICK1_Y_PIN, INPUT);
    pinMode(JOYSTICK2_X_PIN, INPUT);
    pinMode(JOYSTICK2_Y_PIN, INPUT);
    pinMode(BUTTON1_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON2_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON3_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON4_PIN, INPUT_PULLDOWN);
    pinMode(CURSOR_UP_PIN, INPUT_PULLUP);
    pinMode(CURSOR_DOWN_PIN, INPUT_PULLUP);
    pinMode(ANGLE_UP_PIN, INPUT_PULLUP);
    pinMode(ANGLE_DOWN_PIN, INPUT_PULLUP);
    pinMode(ENTER_PIN, INPUT_PULLUP);
    pinMode(DISPLAY_RESET_PIN, INPUT_PULLUP);

    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 init failed");
        while (true);
    }
    display.clearDisplay();
    display.display();

    // Initialize WiFi
    WiFi.mode(WIFI_STA);
WiFi.setChannel(CHANNEL);
delay(100);
Serial.print("Sender MAC Address: ");
Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (true);
    }

    // Register callbacks
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataReceived);

    // Setup Receiver as peer
    memset(&slave, 0, sizeof(slave));
    memcpy(slave.peer_addr, receiverMacAddress, 6);
    slave.channel = CHANNEL;
    slave.encrypt = false;
    if (esp_now_add_peer(&slave) != ESP_OK) {
        Serial.println("Peer add failed");
        while (true);
    }

    // Initialize servo data
    servoData.mode = 0;
    for (int i = 0; i < 6; i++) {
        servoData.initialAngles[i] = initialAngles[i];
        servoData.finalAngles[i] = finalAngles[i];
    }
    servoData.servoRunning = false;
    servoData.restartRequested = false;
    // Ensure servos are at 90 degrees on startup
isServoResetting = true;
currentResetServo = 1;
servoResetStartTime = millis();
}




// Main loop
void loop() {

    handleButtons();

    if (currentMode == OPERATING) {
        displayOperatingMode();
        servoData.mode = 0;
        servoData.servoRunning = false;
        servoData.restartRequested = false;
        for (int i = 0; i < 6; i++) {
            servoData.initialAngles[i] = 90;
            servoData.finalAngles[i] = 90;
            initialAngles[i] = 90;
            finalAngles[i] = 90;
        }
        servoData.servo1Signal = 0;
        servoData.servo2Signal = 0;
        servoData.servo3Signal = 0;
        servoData.servo4Signal = 0;
        servoData.servo5Signal = 0;
        servoData.servo6Signal = 0;
    } 
    else if (currentMode == MANUAL) {
        servoData.mode = 1;
        for (int i = 0; i < 6; i++) {
            servoData.initialAngles[i] = initialAngles[i];
            servoData.finalAngles[i] = finalAngles[i];
        }
        servoData.servoRunning = servoRunning;
        servoData.restartRequested = restartPressed;
        handleManualInputs(); // Add this line to handle joystick and button inputs
        displayManualMode();
    }
    else if (currentMode == AUTO) {
        servoData.mode = 2;
        servoData.servoRunning = servoRunning;
        servoData.restartRequested = restartPressed;
        displayAutoMode();
    }

    if ((isReturningToOperating || manualResetTriggered) && isServoResetting) {
        resetServosTo90();
    }

    sendData();

     // Periodic status
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000) {
        Serial.print("Mode: ");
        Serial.println(currentMode == OPERATING ? "Operating" : currentMode == MANUAL ? "Manual" : "Auto");
        if (currentMode == AUTO) {
            for (int i = 0; i < 6; i++) {
                Serial.println("Servo-" + String(i + 1) + ": Initial=" + String(initialAngles[i]) + ", Final=" + String(initialAngles[i]));
            }
            Serial.println("Servo Running: " + String(servoRunning));
            Serial.println("Restart Requested: " + String(restartPressed));
        }
        Serial.print("Angles: ");
        Serial.print(angleData.angle1); Serial.print(", ");
        Serial.print(angleData.angle2); Serial.print(", ");
        Serial.print(angleData.angle3); Serial.print(", ");
        Serial.print(angleData.angle4); Serial.print(", ");
        Serial.print(angleData.angle5); Serial.print(", ");
        Serial.println(angleData.angle6);
        lastPrintTime = millis();
    }
}
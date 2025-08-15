#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <math.h>    // fabs() ব্যবহারের জন্য

// Pin definitions for servos
#define CHANNEL 1
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 25
#define SERVO5_PIN 26
#define SERVO6_PIN 27

// Servo objects
Servo servo1, servo2, servo3, servo4, servo5, servo6;

// Current angles for each servo (Manual mode)
int currentAngle1 = 90;
int currentAngle2 = 90;
int currentAngle3 = 90;
int currentAngle4 = 90;
int currentAngle5 = 90;
int currentAngle6 = 90;

// Structure to receive control data from Sender
typedef struct {
    uint8_t servo1Signal;
    uint8_t servo2Signal;
    uint8_t servo3Signal;
    uint8_t servo4Signal;
    uint8_t servo5Signal;
    uint8_t servo6Signal;
    uint8_t mode; // 0: Operating, 1: Manual, 2: Auto
    int initialAngles[6]; // Auto mode initial angles
    int finalAngles[6];   // Auto mode final angles
    bool servoRunning;    // Auto mode running state
    bool restartRequested; // Auto mode restart flag
} ServoControl;
ServoControl servoData;
ServoControl lastReceivedData;
void sendAngles();

// Structure to send servo angles back to Sender
struct AngleData {
    int angle1;
    int angle2;
    int angle3;
    int angle4;
    int angle5;
    int angle6;
    bool servoRunning;    // Auto mode running state
    bool restartRequested; // Auto mode restart flag
};
AngleData angleData;

// Manual mode timing variables
unsigned long lastServoUpdate1 = 0, lastServoUpdate2 = 0, lastServoUpdate3 = 0;
unsigned long lastServoUpdate4 = 0, lastServoUpdate5 = 0, lastServoUpdate6 = 0;

//Control servo speed (fast=decrease; Sender- joystickPulseDuration & Receiver- updateInterval, STEP_SIZE_BUTTON, STEP_SIZE_JOYSTICK, INTERPOLATION_STEP)
const int updateInterval = 70; // Update interval for joystick-controlled servos (ms)
const int STEP_SIZE_BUTTON = 4; // Button step size
const int STEP_SIZE_JOYSTICK = 4; // Joystick step size

//Control servo smoothness (fast=decrease) 
const int INTERPOLATION_STEP = 0.8; // Step size for smooth movement

// Auto mode variables
float initialAngles[6] = {90, 90, 90, 90, 90, 90};
float finalAngles[6] = {90, 90, 90, 90, 90, 90};
float currentAngles[6] = {90, 90, 90, 90, 90, 90};
bool servoRunning = false;
int currentServo = 0; // Tracks current servo
bool isInitialToFinalPhase = true; // Tracks phase
bool restartRequested = false; // Tracks restart request
float newInitialAngles[6] = {90, 90, 90, 90, 90, 90};
float newFinalAngles[6] = {90, 90, 90, 90, 90, 90};
const float stepSize = 0.1; // 0.1 degree steps for smooth movement
const float normalDelay = 30.0 / 10; // 3ms per 0.1 deg (30ms/deg)
const float fastDelay = (10.0 / 3) / 10; // 0.333ms per 0.1 deg (~3.33ms/deg)
const int movementDelay = 500; // 500ms delay between servo movements
int resetServoIndex = 0; // Tracks current servo for manual mode reset


// System mode enumeration
enum SystemMode { OPERATING_MODE, MANUAL_MODE, AUTO_MODE };
SystemMode currentMode = OPERATING_MODE;
bool movingToAuto = false; // Flag for transitioning from Manual to Auto
esp_now_peer_info_t senderPeer;
unsigned long lastAngleSendTime = 0;
bool isReturningToOperating = false;
int returnServoIndex = 0;
bool autoModePaused = false;
unsigned long autoModePauseStart = 0;


// Restart flow control
static int restartResetIndex = 0; // নতুন: রিসেট স্টেপ ট্র্যাক
static bool isRestartPaused = false;
static unsigned long restartPauseStart = 0;


// Auto mode: Move servo to target angle
void runServoLoop() {
    // 1) রিস্টার্ট ট্রিগার হবে **শুধু স্টপড** অবস্থায়
    if (restartRequested && !servoRunning && !isRestartPaused && restartResetIndex == 0) {
        isRestartPaused = true;                // 0.5s pause শুরু
        restartPauseStart = millis();
        Serial.println("Restart: received while stopped → taking short pause");
    }

    // 2) 0.5s pause শেষ হলে রিসেট শুরু
    if (isRestartPaused) {
        if (millis() - restartPauseStart >= 500) {
            isRestartPaused = false;
            restartResetIndex = 1;             // ১ নম্বর সার্ভো থেকে রিসেট শুরু
            Serial.println("Restart: pause over → resetting servos to NEW initial angles");
        } else {
            return; // পজ চললে আর কিছু করবো না
        }
    }

    // 3) রিসেট: একেকটা সার্ভো NEW initialAngles-এ নিয়ে যাও
    if (restartResetIndex > 0 && restartResetIndex <= 6) {
        Servo* servo;
        switch (restartResetIndex - 1) {
            case 0: servo = &servo1; break;
            case 1: servo = &servo2; break;
            case 2: servo = &servo3; break;
            case 3: servo = &servo4; break;
            case 4: servo = &servo5; break;
            case 5: servo = &servo6; break;
        }
        float targetAngle = newInitialAngles[restartResetIndex - 1];
        if (fabs(currentAngles[restartResetIndex - 1] - targetAngle) > stepSize) {
            // দ্রুত রিসেট
            if (currentAngles[restartResetIndex - 1] < targetAngle) {
                currentAngles[restartResetIndex - 1] += stepSize;
            } else {
                currentAngles[restartResetIndex - 1] -= stepSize;
            }
            servo->write(currentAngles[restartResetIndex - 1]);
            delayMicroseconds((int)(fastDelay * 1000));
        } else {
            // ওই সার্ভো done
            currentAngles[restartResetIndex - 1] = targetAngle;
            servo->write(currentAngles[restartResetIndex - 1]);
            restartResetIndex++;
            if (restartResetIndex > 6) {
                // NEW angles এখন থেকে অফিশিয়াল
                for (int i = 0; i < 6; i++) {
                    initialAngles[i] = newInitialAngles[i];
                    finalAngles[i]   = newFinalAngles[i];
                }
                restartResetIndex = 0;
                restartRequested = false;
                // নতুন টাস্ক শুরু হবে Initial → Final
                servoRunning = true;
                currentServo = 0;
                isInitialToFinalPhase = true;
                Serial.println("Restart: all servos at NEW initial → running");
            }
        }
        return; // রিসেট চললে নরমাল লুপ চালাবো না
    }

    // 4) নরমাল অপারেশন (Start/Stop → resume/hold)
    if (!servoRunning) return;

    Servo* servo;
    switch (currentServo) {
        case 0: servo = &servo1; break;
        case 1: servo = &servo2; break;
        case 2: servo = &servo3; break;
        case 3: servo = &servo4; break;
        case 4: servo = &servo5; break;
        case 5: servo = &servo6; break;
        default: return;
    }

    if (initialAngles[currentServo] == finalAngles[currentServo]) {
        Serial.println("Servo-" + String(currentServo + 1) + " skipped (Initial = Final)");
        advanceServo();
        return;
    }

    if (isInitialToFinalPhase) {
        if (currentAngles[currentServo] < finalAngles[currentServo] - stepSize) {
            currentAngles[currentServo] += stepSize;
            servo->write(currentAngles[currentServo]);
            delayMicroseconds((int)(normalDelay * 1000));
        } else if (currentAngles[currentServo] > finalAngles[currentServo] + stepSize) {
            currentAngles[currentServo] -= stepSize;
            servo->write(currentAngles[currentServo]);
            delayMicroseconds((int)(normalDelay * 1000));
        } else {
            currentAngles[currentServo] = finalAngles[currentServo];
            servo->write(currentAngles[currentServo]);
            Serial.println("Servo-" + String(currentServo + 1) + " reached Final");
            delay(movementDelay);
            advanceServo();
        }
    } else {
        if (currentAngles[currentServo] < initialAngles[currentServo] - stepSize) {
            currentAngles[currentServo] += stepSize;
            servo->write(currentAngles[currentServo]);
            delayMicroseconds((int)(fastDelay * 1000));
        } else if (currentAngles[currentServo] > initialAngles[currentServo] + stepSize) {
            currentAngles[currentServo] -= stepSize;
            servo->write(currentAngles[currentServo]);
            delayMicroseconds((int)(fastDelay * 1000));
        } else {
            currentAngles[currentServo] = initialAngles[currentServo];
            servo->write(currentAngles[currentServo]);
            Serial.println("Servo-" + String(currentServo + 1) + " reached Initial");
            delay(movementDelay);
            advanceServo();
        }
    }
}


// Auto mode: Advance to next servo or phase
void advanceServo() {
    currentServo++;
    if (currentServo >= 6) {
        currentServo = 0;
        isInitialToFinalPhase = !isInitialToFinalPhase;
        Serial.println(isInitialToFinalPhase ? "Starting Initial-to-Final phase" : "Starting Final-to-Initial phase");
    }
}

// Manual mode: Control Servo 1 (buttons)
void rotateServo1(uint8_t signal) {
  if (millis() - lastServoUpdate1 >= updateInterval) {
    if (signal == 1 && currentAngle1 < 180) {
      currentAngle1 = min(currentAngle1 + STEP_SIZE_BUTTON, 180);
      servo1.write(currentAngle1);
      Serial.println("Servo1: Increasing angle");
    } else if (signal == 2 && currentAngle1 > 0) {
      currentAngle1 = max(currentAngle1 - STEP_SIZE_BUTTON, 0);
      servo1.write(currentAngle1);
      Serial.println("Servo1: Decreasing angle");
    } else {
    servo1.write(currentAngle1);
    Serial.println("Servo6: Holding angle");
}
    lastServoUpdate1 = millis();
  }
}

// Manual mode: Control Servo 2 (joystick)
void rotateServo2(uint8_t signal) {
    if (millis() - lastServoUpdate2 >= updateInterval) {
        Serial.print("Servo2 Signal: "); Serial.println(signal); // সিগন্যাল লগ করুন
        int targetAngle = currentAngle2;
        if (signal == 3 && currentAngle2 < 180) {
            targetAngle = min(currentAngle2 + STEP_SIZE_JOYSTICK, 180);
            Serial.println("Servo2: Increasing angle");
        } else if (signal == 4 && currentAngle2 > 0) {
            targetAngle = max(currentAngle2 - STEP_SIZE_JOYSTICK, 0);
            Serial.println("Servo2: Decreasing angle");
        } else {
            Serial.println("Servo2: Holding angle");
            servo2.write(currentAngle2); // নিশ্চিত করুন বর্তমান অ্যাঙ্গেলে থাকে
        }
        if (targetAngle != currentAngle2) {
            //int step = (targetAngle > currentAngleX) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
            //currentAngleX = constrain(currentAngleX + step, min(currentAngleX, targetAngle), max(currentAngleX, targetAngle));
            currentAngle2 = targetAngle;
            servo2.write(currentAngle2);
        }
        Serial.print("Servo2 Current Angle: "); Serial.println(currentAngle2); // অ্যাঙ্গেল লগ করুন
        lastServoUpdate2 = millis();
    }
}

// Manual mode: Control Servo 3 (joystick)
void rotateServo3(uint8_t signal) {
  if (millis() - lastServoUpdate3 >= updateInterval) {
    int targetAngle = currentAngle3;
    if (signal == 6 && currentAngle3 < 180) {
      targetAngle = min(currentAngle3 + STEP_SIZE_JOYSTICK, 180);
      Serial.println("Servo3: Increasing angle");
    } else if (signal == 7 && currentAngle3 > 0) {
      targetAngle = max(currentAngle3 - STEP_SIZE_JOYSTICK, 0);
      Serial.println("Servo3: Decreasing angle");
    } else {
      Serial.println("Servo3: Holding angle");
    }
    if (targetAngle != currentAngle3) {
     // int step = (targetAngle > currentAngle3) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
     // currentAngle3 = constrain(currentAngle3 + step, min(currentAngle3, targetAngle), max(currentAngle3, targetAngle));
      currentAngle3 = targetAngle;    
      servo3.write(currentAngle3);
    }
    lastServoUpdate3 = millis();
  }
}

// Manual mode: Control Servo 4 (joystick)
void rotateServo4(uint8_t signal) {
    if (millis() - lastServoUpdate4 >= updateInterval) {
        Serial.print("Servo4 Signal: "); Serial.println(signal);
        int targetAngle = currentAngle4;
        if (signal == 9 && currentAngle4 < 180) {
            targetAngle = min(currentAngle4 + STEP_SIZE_JOYSTICK, 180);
            Serial.println("Servo4: Increasing angle");
        } else if (signal == 10 && currentAngle4 > 0) {
            targetAngle = max(currentAngle4 - STEP_SIZE_JOYSTICK, 0);
            Serial.println("Servo4: Decreasing angle");
        } else {
            Serial.println("Servo4: Holding angle");
            servo4.write(currentAngle4);
        }
        if (targetAngle != currentAngle4) {
        //    int step = (targetAngle > currentAngle4) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
        //    currentAngle4 = constrain(currentAngle4 + step, min(currentAngle4, targetAngle), max(currentAngle4, targetAngle));
            currentAngle4 = targetAngle;
            servo4.write(currentAngle4);
            Serial.println("Servo4: Angle updated");
        }
        Serial.print("Servo4 Current Angle: "); Serial.println(currentAngle4);
        lastServoUpdate4 = millis();
    }
}

// Manual mode: Control Servo 5 (joystick)
void rotateServo5(uint8_t signal) {
    if (millis() - lastServoUpdate5 >= updateInterval) {
        Serial.print("Servo5 Signal: "); Serial.println(signal);
        int targetAngle = currentAngle5;
        if (signal == 12 && currentAngle5 < 180) {
            targetAngle = min(currentAngle5 + STEP_SIZE_JOYSTICK, 180);
            Serial.println("Servo5: Increasing angle");
        } else if (signal == 13 && currentAngle5 > 0) {
            targetAngle = max(currentAngle5 - STEP_SIZE_JOYSTICK, 0);
            Serial.println("Servo5: Decreasing angle");
        } else {
            Serial.println("Servo5: Holding angle");
            servo5.write(currentAngle5);
        }
        if (targetAngle != currentAngle5) {
         //   int step = (targetAngle > currentAngle5) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
         //   currentAngle5 = constrain(currentAngle5 + step, min(currentAngle5, targetAngle), max(currentAngle5, targetAngle));
            currentAngle5 = targetAngle;
            servo5.write(currentAngle5);
            Serial.println("Servo5: Angle updated");
        }
        Serial.print("Servo5 Current Angle: "); Serial.println(currentAngle5);
        lastServoUpdate5 = millis();
    }
}

// Manual mode: Control Servo 6 (buttons)
void rotateServo6(uint8_t signal) {
  if (millis() - lastServoUpdate6 >= updateInterval) {
    if (signal == 15 && currentAngle6 < 180) {
      currentAngle6 = min(currentAngle6 + STEP_SIZE_BUTTON, 180);
      servo6.write(currentAngle6);
      Serial.println("Servo6: Increasing angle");
    } else if (signal == 16 && currentAngle6 > 0) {
      currentAngle6 = max(currentAngle6 - STEP_SIZE_BUTTON, 0);
      servo6.write(currentAngle6);
      Serial.println("Servo6: Decreasing angle");
    }else {
        servo6.write(currentAngle6);
        Serial.println("Servo1: Holding angle");
    }
    lastServoUpdate6 = millis();
  }
}


//........................................resetServosForManualfunction.....................................

void resetServosForManual() {
    static unsigned long resetStartTime = 0;
    if (millis() - resetStartTime < 500) return;

    Servo* servo;
    int* currentAngle;
    float* currentAutoAngle;
    switch (resetServoIndex) {
        case 1: servo = &servo1; currentAngle = &currentAngle1; currentAutoAngle = &currentAngles[0]; break;
        case 2: servo = &servo2; currentAngle = &currentAngle2; currentAutoAngle = &currentAngles[1]; break;
        case 3: servo = &servo3; currentAngle = &currentAngle3; currentAutoAngle = &currentAngles[2]; break;
        case 4: servo = &servo4; currentAngle = &currentAngle4; currentAutoAngle = &currentAngles[3]; break;
        case 5: servo = &servo5; currentAngle = &currentAngle5; currentAutoAngle = &currentAngles[4]; break;
        case 6: servo = &servo6; currentAngle = &currentAngle6; currentAutoAngle = &currentAngles[5]; break;
        default: return;
    }
    int targetAngle = 90;
    if (abs(*currentAngle - targetAngle) > STEP_SIZE_BUTTON) {
        if (*currentAngle < targetAngle) {
            *currentAngle += STEP_SIZE_BUTTON;
        } else {
            *currentAngle -= STEP_SIZE_BUTTON;
        }
        *currentAutoAngle = *currentAngle;
        servo->write(*currentAngle);
        Serial.println("Servo-" + String(resetServoIndex) + " moving to " + String(*currentAngle));
    } else {
        *currentAngle = targetAngle;
        *currentAutoAngle = targetAngle;
        servo->write(targetAngle);
        Serial.println("Servo-" + String(resetServoIndex) + " reset to 90 degrees");
        resetServoIndex++;
        if (resetServoIndex > 6) {
            resetServoIndex = 0;
            Serial.println("All servos reset to 90 degrees for Manual mode");
        }
        resetStartTime = millis();
    }
}  // End of rotateServo6()

//.................................OnDataReceived................................................................................................................................



void sendAngles() {
    angleData.angle1 = currentMode == AUTO_MODE ? (int)currentAngles[0] : currentAngle1;
    angleData.angle2 = currentMode == AUTO_MODE ? (int)currentAngles[1] : currentAngle2;
    angleData.angle3 = currentMode == AUTO_MODE ? (int)currentAngles[2] : currentAngle3;
    angleData.angle4 = currentMode == AUTO_MODE ? (int)currentAngles[3] : currentAngle4;
    angleData.angle5 = currentMode == AUTO_MODE ? (int)currentAngles[4] : currentAngle5;
    angleData.angle6 = currentMode == AUTO_MODE ? (int)currentAngles[5] : currentAngle6;
    angleData.servoRunning = servoRunning;
    angleData.restartRequested = restartRequested;
    esp_err_t result = esp_now_send(senderPeer.peer_addr, (uint8_t *)&angleData, sizeof(angleData));
    if (result != ESP_OK) {
        Serial.println("Failed to send angles");
    }
}

// Callback for receiving control data from Sender
void OnDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
    if (len == sizeof(ServoControl)) {
        memcpy(&servoData, data, sizeof(servoData));
        Serial.print("Received: S1="); Serial.print(servoData.servo1Signal);
        Serial.print(" S2="); Serial.print(servoData.servo2Signal);
        Serial.print(" S3="); Serial.print(servoData.servo3Signal);
        Serial.print(" S4="); Serial.print(servoData.servo4Signal);
        Serial.print(" S5="); Serial.print(servoData.servo5Signal);
        Serial.print(" S6="); Serial.print(servoData.servo6Signal);
        Serial.print(" Mode="); Serial.println(servoData.mode);

        SystemMode newMode = (servoData.mode == 0) ? OPERATING_MODE : (servoData.mode == 1) ? MANUAL_MODE : AUTO_MODE;

        if (newMode == OPERATING_MODE && (currentMode == MANUAL_MODE || currentMode == AUTO_MODE)) {
            isReturningToOperating = true;
            resetServoIndex = 1; // Start with first servo for reset
            autoModePaused = true;
            autoModePauseStart = millis();
            // Update angles based on sender's initialAngles
            for (int i = 0; i < 6; i++) {
                initialAngles[i] = (float)servoData.initialAngles[i];
                finalAngles[i] = (float)servoData.finalAngles[i];
            }
            if (currentMode == AUTO_MODE) {
                servoRunning = false;
                restartRequested = false;
                currentServo = 0;
                isInitialToFinalPhase = true;
                Serial.println("Auto mode paused for returning to Operating mode");
            }
            Serial.println("Returning to Operating mode, starting sequential servo reset to 90 degrees");
        }

        if (newMode != currentMode && currentMode == MANUAL_MODE && newMode == AUTO_MODE) {
            movingToAuto = true;
            currentServo = 0;
            isInitialToFinalPhase = true;
            servoRunning = false; // Reset servoRunning for new Auto mode entry
            restartRequested = false; // Reset restartRequested
            for (int i = 0; i < 6; i++) {
                initialAngles[i] = (float)servoData.initialAngles[i];
                finalAngles[i] = (float)servoData.finalAngles[i];
                currentAngles[i] = initialAngles[i];
            }
            lastReceivedData = servoData;
            Serial.println("Entered Auto mode from Manual mode, reset to initial state");
} else if (currentMode == AUTO_MODE) {
    // ★ lastReceivedData ওভাররাইট করার আগেই রাইজিং এজ ধরো
    bool startEdge = (!lastReceivedData.servoRunning && servoData.servoRunning);

    // Start/Stop আপডেট
    servoRunning = servoData.servoRunning;

    // ★ Start ঠিক এখন ON হল → নতুন Initial/Final কপি করো (resume রাখার জন্য currentAngles ছোঁয়া যাবে না)
    if (startEdge) {
        for (int i = 0; i < 6; i++) {
            initialAngles[i] = (float)servoData.initialAngles[i];
            finalAngles[i]   = (float)servoData.finalAngles[i];
        }
        Serial.println("AUTO: Start edge → initial/final updated from Sender (resume from current)");
    }

    // Restart: শুধু স্টপড হলে গ্রহণ
    if (!servoRunning && servoData.restartRequested) {
        for (int i = 0; i < 6; i++) {
            newInitialAngles[i] = (float)servoData.initialAngles[i];
            newFinalAngles[i]   = (float)servoData.finalAngles[i];
        }
        restartRequested = true;   // runServoLoop() এর রিস্টার্ট ফ্লো ট্রিগার হবে
        Serial.println("Restart: accepted (stopped) → will reset to NEW initial then run");
    }

    // ★ সব কাজ শেষে lastReceivedData আপডেট করো
    lastReceivedData = servoData;
}


        // ← currentMode অ্যাসাইন করার আগেই ট্রানজিশন চেক করো
        if (newMode == MANUAL_MODE && currentMode != MANUAL_MODE) {
            resetServoIndex = 1; // Start with first servo
            autoModePaused = false;
            Serial.println("Entering Manual mode, starting servo reset to 90 degrees");
        }

        // এখন অ্যাসাইন করো
        currentMode = newMode;

        if (currentMode == MANUAL_MODE && !movingToAuto) {
            rotateServo1(servoData.servo1Signal);
            rotateServo2(servoData.servo2Signal);
            rotateServo3(servoData.servo3Signal);
            rotateServo4(servoData.servo4Signal);
            rotateServo5(servoData.servo5Signal);
            rotateServo6(servoData.servo6Signal);
        }
    } else {
        Serial.println("Invalid data length");
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    Serial.println("Receiver Started");

    // Attach servos
servo1.attach(SERVO1_PIN, 500, 2500);
    servo1.write(90);
    currentAngle1 = 90;
    servo2.attach(SERVO2_PIN, 500, 2500);
    servo2.write(90);
    currentAngle2 = 90;
    servo3.attach(SERVO3_PIN, 500, 2500);
    servo3.write(90);
    currentAngle3 = 90;
    servo4.attach(SERVO4_PIN, 500, 2500);
    servo4.write(90);
    currentAngle4 = 90;
    servo5.attach(SERVO5_PIN, 500, 2500);
    servo5.write(90);
    currentAngle5 = 90;
    servo6.attach(SERVO6_PIN, 500, 2500);
    servo6.write(90);
    currentAngle6 = 90;
    for (int i = 0; i < 6; i++) {
        currentAngles[i] = 90.0;
    }
    delay(500);


    // Initialize WiFi
    WiFi.mode(WIFI_STA);
WiFi.setChannel(CHANNEL);
delay(100);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (true);
    }

    // Register callbacks
    esp_now_register_recv_cb(OnDataReceived);

    // Setup Sender as peer
    memset(&senderPeer, 0, sizeof(senderPeer));
    uint8_t sender_mac[6] = {0x78, 0x42, 0x1C, 0x6D, 0x3F, 0xB4}; // Replace with Sender MAC
    memcpy(senderPeer.peer_addr, sender_mac, 6);
    senderPeer.channel = CHANNEL;
    senderPeer.encrypt = false;
    if (esp_now_add_peer(&senderPeer) != ESP_OK) {
        Serial.println("Peer add failed");
        while (true);
    }
}

// Main loop
void loop() {
if (resetServoIndex > 0) {
    resetServosForManual();
}
 


if (currentMode == AUTO_MODE) {
    // Always run — handles:
    //  - restart flow while stopped
    //  - normal motion while running
    runServoLoop();

    if (movingToAuto) {
        movingToAuto = false;
        currentAngles[0] = currentAngle1;
        currentAngles[1] = currentAngle2;
        currentAngles[2] = currentAngle3;
        currentAngles[3] = currentAngle4;
        currentAngles[4] = currentAngle5;
        currentAngles[5] = currentAngle6;
    }
}


    // Send angles every 100ms
    if (millis() - lastAngleSendTime >= 100) {
        sendAngles();
        lastAngleSendTime = millis();
    }

    // Print status every 1 second
    static unsigned long lastUpdateTime = 0;
    if (millis() - lastUpdateTime >= 1000) {
        Serial.print("Mode: ");
        Serial.print(currentMode == OPERATING_MODE ? "Operating" : currentMode == MANUAL_MODE ? "Manual" : "Auto");
        if (currentMode == MANUAL_MODE) {
            Serial.print(" S1: "); Serial.print(servoData.servo1Signal);
            Serial.print(" S2: "); Serial.print(servoData.servo2Signal);
            Serial.print(" S3: "); Serial.print(servoData.servo3Signal);
            Serial.print(" S4: "); Serial.print(servoData.servo4Signal);
            Serial.print(" S5: "); Serial.print(servoData.servo5Signal);
            Serial.print(" S6: "); Serial.print(servoData.servo6Signal);
        }
        Serial.print(" A1: "); Serial.print(currentMode == AUTO_MODE ? currentAngles[0] : currentAngle1);
        Serial.print(" A2: "); Serial.print(currentMode == AUTO_MODE ? currentAngles[1] : currentAngle2);
        Serial.print(" A3: "); Serial.print(currentMode == AUTO_MODE ? currentAngles[2] : currentAngle3);
        Serial.print(" A4: "); Serial.print(currentMode == AUTO_MODE ? currentAngles[3] : currentAngle4);
        Serial.print(" A5: "); Serial.print(currentMode == AUTO_MODE ? currentAngles[4] : currentAngle5);
        Serial.print(" A6: "); Serial.println(currentMode == AUTO_MODE ? currentAngles[5] : currentAngle6);
        lastUpdateTime = millis();
    }
}
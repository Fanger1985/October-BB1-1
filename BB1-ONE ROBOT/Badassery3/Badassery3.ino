#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Arduino.h> // Include Arduino header last to ensure macros are correctly defined

// Wi-Fi credentials
const char* ssid = "SpectrumSetup-DD";          // Replace with your Wi-Fi SSID
const char* password = "jeansrocket543";  // Replace with your Wi-Fi password

// Create PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels
int servoPanChannel = 0;
int servoTiltChannel = 1;
int servoRightEarChannel = 2;
int servoLeftEarChannel = 3;
int cameraPanChannel = 5;  // Pan servo for the camera mounted on pin 5
int cameraTiltChannel = 4; // Tilt servo for the camera mounted on pin 4

// Additional pins for automated movement (excluding 4 and 5 since they are now camera controls)
int autoPins[] = {6, 7};
int autoPinsCount = sizeof(autoPins) / sizeof(autoPins[0]);
float autoPinAngles[2] = {90, 90};  // Initial angles for auto pins

// Additional pins to stay at 90 degrees
int fixedPins[] = {8, 9, 10, 11};
int fixedPinsCount = sizeof(fixedPins) / sizeof(fixedPins[0]);

// Positions
float panAngle = 117;          // Adjusted center pan
float tiltAngle = 90;          // Tilt remains centered at 90°
float rightEarAngle = 90;      // Start right ear at 90 degrees
float leftEarAngle = 90;       // Start left ear at 90 degrees
float cameraPanAngle = 90;     // Camera's resting pan position
float cameraTiltAngle = 90;    // Camera's resting tilt position

float targetPanAngle = 117;    // Updated target pan to center at 117°
float targetTiltAngle = 90;
float targetRightEarAngle = 90;  // "Ready" position
float targetLeftEarAngle = 90;   // "Ready" position
float targetCameraPanAngle = 90;  // Camera pan target angle
float targetCameraTiltAngle = 90; // Camera tilt target angle

bool isMoving = false;
bool isCameraMoving = false;
bool isEarMoving = false;     // Track ear movement separately
bool isDemo1Running = false;  // Track if Demo 1 is running
bool isDemo2Running = false;  // Track if Demo 2 is running
bool isNodYesRunning = false; // Track if "Yes" animation is running
bool isShakeNoRunning = false;// Track if "No" animation is running
bool isStartupSequenceRunning = false; // Track if startup sequence is running

// Speed and acceleration settings
float panSpeed = 3.5;   // Adjusted for smoother movement
float tiltSpeed = 1.5;  // Adjusted for smoother movement
float panAcceleration = 0.6;  // Adjusted pan acceleration
float tiltAcceleration = 0.6; // Adjusted tilt acceleration
float cameraPanSpeed = 5.0;   // Increased speed for camera movements during startup
float cameraTiltSpeed = 5.0;  // Increased speed for camera movements during startup
float earSpeed = 2.5;         // Ear speed

// Timing
unsigned long previousMillis = 0;
unsigned long autoMoveMillis = 0;
unsigned long demo1Millis = 0;
unsigned long demo2Millis = 0;
unsigned long nodYesMillis = 0;
unsigned long shakeNoMillis = 0;
unsigned long startupMillis = 0;
const long interval = 20;             // Adjusted time between each step
const long autoMoveInterval = 2000;   // Time between auto movements (in milliseconds)
const long extendedPause = 5500;         // Pause duration for looking at hands

// Adjusted Pulse Width Range based on servo specifications
#define SERVOMIN  123  // Corresponds to 500μs pulse width at 60Hz
#define SERVOMAX  614  // Corresponds to 2500μs pulse width at 60Hz

// Counts per degree for DS3218 servos
#define COUNTS_PER_DEGREE ((float)(SERVOMAX - SERVOMIN) / 270.0)

// Adjusted Pulse Width Range for Micro 9g Servos (camera and ears)
#define MICRO_SERVOMIN 150  // Minimum pulse length out of 4096 (adjusted for MG90S/Micro 9g)
#define MICRO_SERVOMAX 600  // Maximum pulse length out of 4096 (adjusted for MG90S/Micro 9g)

// Web server
AsyncWebServer server(80);

// Function to convert angle to pulse length for the right ear
uint16_t rightEarAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 0, 180, MICRO_SERVOMIN, MICRO_SERVOMAX); // Micro 9g pulse range
}

// Function to convert angle to pulse length for the left ear (inverted mapping)
uint16_t leftEarAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 180, 0, MICRO_SERVOMIN, MICRO_SERVOMAX); // Inverted mapping
}

// Function to convert angle to pulse length for pan/tilt (DS3218 servos)
uint16_t angleToPulse(float angle, bool isTilt) {
    angle = constrain(angle, 0, 270); // DS3218 servo range is 0° to 270°
    uint16_t pulse = (uint16_t)(SERVOMIN + angle * COUNTS_PER_DEGREE);
    return pulse;
}

// Function to convert angle to pulse length for the camera (micro 9g servos)
uint16_t cameraAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 0, 180, MICRO_SERVOMIN, MICRO_SERVOMAX); // Micro 9g pulse range
}

// Function to convert angle to pulse length for auto pins
uint16_t autoPinAngleToPulse(float angle) {
    angle = constrain(angle, 0, 270); // DS3218 servo range is 0° to 270°
    uint16_t pulse = (uint16_t)(SERVOMIN + angle * COUNTS_PER_DEGREE);
    return pulse;
}

// Function to handle automatic movement of pins 6 and 7
void updateAutoPins() {
    unsigned long currentMillis = millis();

    if (currentMillis - autoMoveMillis >= autoMoveInterval) {
        autoMoveMillis = currentMillis;

        // Keep each auto pin at 90 degrees (no movement required)
        for (int i = 0; i < autoPinsCount; i++) {
            pwm.setPWM(autoPins[i], 0, autoPinAngleToPulse(90));  // Set auto pins to 90 degrees
        }
    }
}

// Function to set fixed pins to 90 degrees
void setFixedPinsToCenter() {
    for (int i = 0; i < fixedPinsCount; i++) {
        pwm.setPWM(fixedPins[i], 0, autoPinAngleToPulse(90));  // Set each fixed pin to 90 degrees
    }
}

// Function to start smooth movement
void startMovement(float newTargetPan, float newTargetTilt, float newTargetRightEar, float newTargetLeftEar) {
    targetPanAngle = constrain(newTargetPan, 0, 270);    // Apply pan limits
    targetTiltAngle = constrain(newTargetTilt, 70, 110); // Apply tilt limits
    isMoving = true;

    targetRightEarAngle = newTargetRightEar;
    targetLeftEarAngle = newTargetLeftEar;
    isEarMoving = true;  // Ensure ear movement is active
}

// Function to start smooth camera movement
void startCameraMovement(float newTargetCameraPan, float newTargetCameraTilt) {
    targetCameraPanAngle = constrain(newTargetCameraPan, 0, 180);
    targetCameraTiltAngle = constrain(newTargetCameraTilt, 0, 180);
    isCameraMoving = true;
}

// Function for non-blocking smooth movement
void updateMovement() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Smooth pan movement with adjustable speed and acceleration
        if (panAngle != targetPanAngle) {
            float step = panSpeed * panAcceleration;
            if (abs(panAngle - targetPanAngle) > step) {
                panAngle += (panAngle < targetPanAngle) ? step : -step;
            } else {
                panAngle = targetPanAngle; // Snap to target if very close
            }
            pwm.setPWM(servoPanChannel, 0, angleToPulse(panAngle, false));
        }

        // Smooth tilt movement with adjustable speed and acceleration
        if (tiltAngle != targetTiltAngle) {
            float step = tiltSpeed * tiltAcceleration;
            if (abs(tiltAngle - targetTiltAngle) > step) {
                tiltAngle += (tiltAngle < targetTiltAngle) ? step : -step;
            } else {
                tiltAngle = targetTiltAngle; // Snap to target if very close
            }
            pwm.setPWM(servoTiltChannel, 0, angleToPulse(tiltAngle, true));
        }

        // Stop movement when pan and tilt targets are reached
        if (panAngle == targetPanAngle && tiltAngle == targetTiltAngle) {
            isMoving = false;
        }
    }
}

// Function for non-blocking smooth camera movement
void updateCameraMovement() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Smooth camera pan movement
        if (cameraPanAngle != targetCameraPanAngle) {
            float step = cameraPanSpeed;
            if (abs(cameraPanAngle - targetCameraPanAngle) > step) {
                cameraPanAngle += (cameraPanAngle < targetCameraPanAngle) ? step : -step;
            } else {
                cameraPanAngle = targetCameraPanAngle; // Snap to target if very close
            }
            pwm.setPWM(cameraPanChannel, 0, cameraAngleToPulse(cameraPanAngle));
        }

        // Smooth camera tilt movement
        if (cameraTiltAngle != targetCameraTiltAngle) {
            float step = cameraTiltSpeed;
            if (abs(cameraTiltAngle - targetCameraTiltAngle) > step) {
                cameraTiltAngle += (cameraTiltAngle < targetCameraTiltAngle) ? step : -step;
            } else {
                cameraTiltAngle = targetCameraTiltAngle; // Snap to target if very close
            }
            pwm.setPWM(cameraTiltChannel, 0, cameraAngleToPulse(cameraTiltAngle));
        }

        // Stop camera movement when all targets are reached
        if (cameraPanAngle == targetCameraPanAngle && cameraTiltAngle == targetCameraTiltAngle) {
            isCameraMoving = false;
        }
    }
}

// Function for non-blocking smooth ear movement
void updateEarMovement() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Smooth right ear movement
        if (rightEarAngle != targetRightEarAngle) {
            float step = earSpeed;
            if (abs(rightEarAngle - targetRightEarAngle) > step) {
                rightEarAngle += (rightEarAngle < targetRightEarAngle) ? step : -step;
            } else {
                rightEarAngle = targetRightEarAngle; // Snap to target if very close
            }
            pwm.setPWM(servoRightEarChannel, 0, rightEarAngleToPulse(rightEarAngle));
        }

        // Smooth left ear movement
        if (leftEarAngle != targetLeftEarAngle) {
            float step = earSpeed;
            if (abs(leftEarAngle - targetLeftEarAngle) > step) {
                leftEarAngle += (leftEarAngle < targetLeftEarAngle) ? step : -step;
            } else {
                leftEarAngle = targetLeftEarAngle; // Snap to target if very close
            }
            // Apply inverted mapping for left ear
            pwm.setPWM(servoLeftEarChannel, 0, leftEarAngleToPulse(leftEarAngle));
        }

        // Stop ear movement when targets are reached
        if (rightEarAngle == targetRightEarAngle && leftEarAngle == targetLeftEarAngle) {
            isEarMoving = false;
        }
    }
}

// Function to check if any sequence is running
bool isAnySequenceRunning() {
    return isDemo1Running || isDemo2Running || isNodYesRunning || isShakeNoRunning || isStartupSequenceRunning;
}

// Start Demo 1 sequence (updated with new behaviors)
void startDemo1() {
    if (isAnySequenceRunning()) return; // Do not start if another sequence is running
    isDemo1Running = true;
    demo1Millis = millis();
}

// Function to update Demo 1 sequence (updated with new behaviors)
void updateDemo1() {
    if (!isDemo1Running) return;

    unsigned long currentMillis = millis();
    static int step = 0;
    static unsigned long stepStartTime = demo1Millis;
    static int earWiggleCount = 0;

    if (currentMillis - stepStartTime >= interval) {
        switch (step) {
            case 0:
                // Move head to center position (pan 117°, tilt 90°)
                startMovement(117, 90, 90, 90);
                startCameraMovement(90, 90);
                stepStartTime = currentMillis;
                step++;
                break;
            case 1:
                // Look right (pan to 72°)
                if (!isMoving) {
                    startMovement(72, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 2:
                // Back to center (pan 117°)
                if (!isMoving) {
                    startMovement(117, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 3:
                // Look left (pan to 173°)
                if (!isMoving) {
                    startMovement(173, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 4:
                // Back to center (pan 117°)
                if (!isMoving) {
                    startMovement(117, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 5:
                // Look up (tilt head to 110°), then tilt camera to 14° and camera pan to 117°
                if (!isMoving) {
                    startMovement(targetPanAngle, 110, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(117, 14);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 6:
                // Head back to center (tilt 90°), keep camera pan at 117°, return camera tilt to 77°
                if (!isMoving && !isCameraMoving) {
                    startMovement(targetPanAngle, 90, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(117, 77);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 7:
                // Look upper left (head pan 175°, tilt 110°)
                if (!isMoving && !isCameraMoving) {
                    startMovement(175, 110, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 8:
                // Look bottom left (head pan 175°, tilt 75°)
                if (!isMoving) {
                    startMovement(175, 75, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 9:
                // Look bottom right (head pan 76°, tilt 76°)
                if (!isMoving) {
                    startMovement(76, 76, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 10:
                // Look upper right (head pan 81°, tilt 107°)
                if (!isMoving) {
                    startMovement(81, 107, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 11:
                // Back to center (pan 117°, tilt 90°)
                if (!isMoving) {
                    startMovement(117, 90, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 12:
                // Both ears back (right ear 141°, left ear 161°)
                if (!isMoving) {
                    targetRightEarAngle = 141;
                    targetLeftEarAngle = 161;
                    isEarMoving = true;
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 13:
                // Wiggle both ears back and forth a few times
                if (!isEarMoving) {
                    if (earWiggleCount < 4) {
                        if (earWiggleCount % 2 == 0) {
                            targetRightEarAngle = 121; // Move ears forward slightly
                            targetLeftEarAngle = 141;
                        } else {
                            targetRightEarAngle = 141; // Move ears back
                            targetLeftEarAngle = 161;
                        }
                        isEarMoving = true;
                        earWiggleCount++;
                    } else {
                        earWiggleCount = 0;
                        stepStartTime = currentMillis;
                        step++;
                    }
                }
                break;
            case 14:
                // Center ears (right ear 90°, left ear 90°)
                if (!isEarMoving) {
                    targetRightEarAngle = 90;
                    targetLeftEarAngle = 90;
                    isEarMoving = true;
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 15:
                // Conclude Demo 1
                if (!isEarMoving) {
                    isDemo1Running = false;
                    step = 0;
                }
                break;
        }
    }
}

// Start Demo 2 sequence
void startDemo2() {
    if (isAnySequenceRunning()) return; // Do not start if another demo is running
    isDemo2Running = true;
    demo2Millis = millis();
}

// Function to update Demo 2 sequence
void updateDemo2() {
    if (!isDemo2Running) return;

    unsigned long currentMillis = millis();
    static int step = 0;
    static unsigned long stepStartTime = demo2Millis;

    if (currentMillis - demo2Millis >= 50) {  // Adjust timing as needed
        switch (step) {
            case 0:
                // Start by going to center
                startMovement(117, 90, 90, 90);
                startCameraMovement(90, 90);
                stepStartTime = currentMillis;
                step++;
                break;
            case 1:
                // Move camera pan smoothly between 70 and 120 degrees
                if (currentMillis - stepStartTime >= 2000) {
                    startCameraMovement(70, cameraTiltAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 2:
                if (currentMillis - stepStartTime >= 4000) {
                    startCameraMovement(120, cameraTiltAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 3:
                // Move side to side while tilting
                if (currentMillis - stepStartTime >= 6000) {
                    startCameraMovement(70, 110);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 4:
                if (currentMillis - stepStartTime >= 8000) {
                    startCameraMovement(120, 0);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 5:
                // Move ears smoothly, opposite from each other
                if (currentMillis - stepStartTime >= 10000) {
                    targetRightEarAngle = 160;
                    targetLeftEarAngle = 35;
                    isEarMoving = true;
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 6:
                if (currentMillis - stepStartTime >= 12000) {
                    targetRightEarAngle = 60;
                    targetLeftEarAngle = 135;
                    isEarMoving = true;
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 7:
                // Back to center
                if (currentMillis - stepStartTime >= 14000) {
                    startMovement(117, 90, 90, 90);
                    startCameraMovement(90, 90);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 8:
                // Pan head left to 120 degrees while moving ears
                if (currentMillis - stepStartTime >= 16000) {
                    startMovement(120, 90, 65, 45);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 9:
                // Back to center
                if (currentMillis - stepStartTime >= 18000) {
                    startMovement(117, 90, 90, 90);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 10:
                // Pan head right to 70 degrees while moving ears
                if (currentMillis - stepStartTime >= 20000) {
                    startMovement(70, 90, 125, 120);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 11:
                // Back to center
                if (currentMillis - stepStartTime >= 22000) {
                    startMovement(117, 90, 90, 90);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 12:
                // Tilt head up to 110 degrees while tilting camera down and adjusting ears
                if (currentMillis - stepStartTime >= 24000) {
                    startMovement(117, 110, 145, 35); // Tilt up
                    startCameraMovement(90, 110);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 13:
                // Back to center to conclude
                if (currentMillis - stepStartTime >= 26000) {
                    startMovement(117, 90, 90, 90);
                    startCameraMovement(90, 90);
                    isDemo2Running = false;
                    step = 0;
                }
                break;
        }
        demo2Millis = currentMillis;
    }
}

// Start "Yes" nodding animation
void startNodYes() {
    if (isAnySequenceRunning()) return; // Do not start if another animation is running
    isNodYesRunning = true;
    nodYesMillis = millis();
}

// Update "Yes" nodding animation
void updateNodYes() {
    if (!isNodYesRunning) return;

    unsigned long currentMillis = millis();
    static int step = 0;
    static unsigned long stepStartTime = nodYesMillis;
    static int nodCount = 0;

    if (currentMillis - stepStartTime >= interval) {
        switch (step) {
            case 0:
                // Move to center
                startMovement(117, 90, targetRightEarAngle, targetLeftEarAngle);
                stepStartTime = currentMillis;
                step++;
                break;
            case 1:
                // Tilt down to 75 degrees
                if (!isMoving) {
                    startMovement(targetPanAngle, 75, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 2:
                // Tilt up to 100 degrees
                if (!isMoving) {
                    startMovement(targetPanAngle, 100, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 3:
                // Increment nod count
                if (!isMoving) {
                    nodCount++;
                    if (nodCount < 3) {
                        // Repeat nodding
                        step = 1; // Go back to tilt down
                    } else {
                        // Nodding complete, reset nod count
                        nodCount = 0;
                        step++;
                    }
                    stepStartTime = currentMillis;
                }
                break;
            case 4:
                // Back to center tilt (90 degrees)
                if (!isMoving) {
                    startMovement(targetPanAngle, 90, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 5:
                // Animation complete
                if (!isMoving) {
                    isNodYesRunning = false;
                    step = 0;
                }
                break;
        }
    }
}

// Start "No" shaking animation
void startShakeNo() {
    if (isAnySequenceRunning()) return; // Do not start if another animation is running
    isShakeNoRunning = true;
    shakeNoMillis = millis();
}

// Update "No" shaking animation
void updateShakeNo() {
    if (!isShakeNoRunning) return;

    unsigned long currentMillis = millis();
    static int step = 0;
    static unsigned long stepStartTime = shakeNoMillis;
    static int shakeCount = 0;

    if (currentMillis - stepStartTime >= interval) {
        switch (step) {
            case 0:
                // Move to center
                startMovement(117, 90, targetRightEarAngle, targetLeftEarAngle);
                stepStartTime = currentMillis;
                step++;
                break;
            case 1:
                // Pan to 85 degrees
                if (!isMoving) {
                    startMovement(85, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 2:
                // Pan to 150 degrees
                if (!isMoving) {
                    startMovement(150, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 3:
                // Increment shake count
                if (!isMoving) {
                    shakeCount++;
                    if (shakeCount < 3) {
                        // Repeat shaking
                        step = 1; // Go back to pan to 85 degrees
                    } else {
                        // Shaking complete, reset shake count
                        shakeCount = 0;
                        step++;
                    }
                    stepStartTime = currentMillis;
                }
                break;
            case 4:
                // Back to center pan (117 degrees)
                if (!isMoving) {
                    startMovement(117, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 5:
                // Animation complete
                if (!isMoving) {
                    isShakeNoRunning = false;
                    step = 0;
                }
                break;
        }
    }
}

// Start the startup sequence
void startStartupSequence() {
    isStartupSequenceRunning = true;
    startupMillis = millis();
}

// Update the startup sequence with added behaviors
void updateStartupSequence() {
    if (!isStartupSequenceRunning) return;

    unsigned long currentMillis = millis();
    static int step = 0;
    static unsigned long stepStartTime = startupMillis;

    if (currentMillis - stepStartTime >= interval) {
        switch (step) {
            case 0:
                // Smoothly move head to center
                startMovement(117, 90, 90, 90);
                stepStartTime = currentMillis;
                step++;
                break;
            case 1:
                // Tilt head down to 75 degrees
                if (!isMoving) {
                    startMovement(targetPanAngle, 75, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 2:
                // Camera pan and tilt sequence
                if (!isMoving) {
                    startCameraMovement(30, 103);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 3:
                // Camera moves to 130 pan and 15 tilt
                if (!isCameraMoving) {
                    startCameraMovement(130, 15);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 4:
                // Repeat camera movement if desired
                if (!isCameraMoving) {
                    startCameraMovement(30, 103);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 5:
                // Finish camera movement
                if (!isCameraMoving) {
                    startCameraMovement(130, 15);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 6:
                // Tilt head up to 105 degrees and return to center
                if (!isCameraMoving) {
                    startMovement(targetPanAngle, 105, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 7:
                // Return head tilt to 90 degrees
                if (!isMoving) {
                    startMovement(targetPanAngle, 90, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 8:
                // Set camera to 73 pan and 73 tilt
                if (!isMoving) {
                    startCameraMovement(73, 73);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 9:
                // Move ears: right to 146, left to 167
                if (!isCameraMoving) {
                    targetRightEarAngle = 146;
                    targetLeftEarAngle = 167;
                    isEarMoving = true;
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 10:
                // Move to pan 152, tilt 75, with camera pan 117, tilt 100
                if (!isEarMoving) {
                    startMovement(152, 75, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(117, 100);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 11:
                // Extended pause to "look at hand"
                if (!isMoving && !isCameraMoving) {
                    stepStartTime = currentMillis + extendedPause;
                    step++;
                }
                break;
            case 12:
                // Return head and camera to center
                if (!isMoving && !isCameraMoving) {
                    startMovement(117, 90, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(90, 90);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 13:
                // Move to pan 80, tilt 75, with camera pan 80, tilt 106
                if (!isMoving && !isCameraMoving) {
                    startMovement(80, 75, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(80, 106);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 14:
                // Extended pause to "look at hand"
                if (!isMoving && !isCameraMoving) {
                    stepStartTime = currentMillis + extendedPause;
                    step++;
                }
                break;
            case 15:
                // Return head and camera to center
                if (!isMoving && !isCameraMoving) {
                    startMovement(117, 90, targetRightEarAngle, targetLeftEarAngle);
                    startCameraMovement(90, 90);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 16:
                // Perform "nod yes"
                startNodYes();
                stepStartTime = currentMillis;
                step++;
                break;
            case 17:
                // Move ears and tilt head
                if (!isNodYesRunning) {
                    targetRightEarAngle = 163;
                    targetLeftEarAngle = 178;
                    startMovement(117, 104, targetRightEarAngle, targetLeftEarAngle);
                    stepStartTime = currentMillis;
                    step++;
                }
                break;
            case 18:
                // Return to center to finish the startup sequence
                if (!isEarMoving) {
                    startMovement(117, 90, 90, 90);
                    isStartupSequenceRunning = false;
                    step = 0;
                }
                break;
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize PWM driver
    pwm.begin();
    pwm.setPWMFreq(60);  // Set PWM frequency to 60Hz based on servo specifications

    // Connect to Wi-Fi
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize mDNS with desired hostname
    if (!MDNS.begin("bb1head")) {
        Serial.println("Error starting mDNS");
    } else {
        Serial.println("mDNS responder started. Access via http://bb1head.local");
    }

    // Move to initial position on startup
    startMovement(117, 90, 90, 90);
    startCameraMovement(90, 90);
    Serial.println("Moved to initial position on startup.");

    // Set fixed pins to center
    setFixedPinsToCenter();

    // Start the startup sequence
    startStartupSequence();

    // Web server handlers
    // Updated HTML Control Panel
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>BB-1 Head Control Panel</title>
            <style>
                body {
                    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                    background-color: #121212;
                    color: #FFFFFF;
                    margin: 0;
                    padding: 0;
                }
                .container {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    padding: 20px;
                }
                h1 {
                    margin-bottom: 20px;
                }
                .controls {
                    display: flex;
                    flex-wrap: wrap;
                    justify-content: center;
                    gap: 20px;
                    width: 100%;
                    max-width: 1200px;
                }
                .control-group {
                    background-color: #1E1E1E;
                    padding: 20px;
                    border-radius: 10px;
                    flex: 1 1 calc(33% - 40px);
                    box-sizing: border-box;
                    min-width: 300px;
                }
                .control-group h2 {
                    margin-top: 0;
                }
                .slider-container {
                    margin: 10px 0;
                }
                .slider-label {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }
                .slider-label span {
                    font-size: 14px;
                }
                input[type="range"] {
                    width: 100%;
                }
                .button-group {
                    display: flex;
                    flex-wrap: wrap;
                    gap: 10px;
                    margin-top: 10px;
                }
                .btn {
                    padding: 10px 20px;
                    font-size: 16px;
                    border: none;
                    border-radius: 5px;
                    cursor: pointer;
                    flex: 1 1 45%;
                    box-sizing: border-box;
                }
                .btn:hover {
                    opacity: 0.9;
                }
                .btn:active {
                    opacity: 0.8;
                }
                /* Specific styles for different button types */
                .btn-primary {
                    background-color: #007BFF;
                    color: #FFFFFF;
                }
                .btn-secondary {
                    background-color: #6C757D;
                    color: #FFFFFF;
                }
                .btn-success {
                    background-color: #28A745;
                    color: #FFFFFF;
                }
                .btn-danger {
                    background-color: #DC3545;
                    color: #FFFFFF;
                }
                .btn-warning {
                    background-color: #FFC107;
                    color: #212529;
                }
                .btn-info {
                    background-color: #17A2B8;
                    color: #FFFFFF;
                }
                @media (max-width: 768px) {
                    .control-group {
                        flex: 1 1 100%;
                    }
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>BB-1 Head Control Panel</h1>
                <div class="controls">
                    <!-- Head Controls -->
                    <div class="control-group">
                        <h2>Head Controls</h2>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Pan: <span id="panAngle">117</span>°</span>
                            </div>
                            <input type="range" min="0" max="270" value="117" id="panSlider" oninput="updatePan(this.value)">
                        </div>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Tilt: <span id="tiltAngle">90</span>°</span>
                            </div>
                            <input type="range" min="75" max="110" value="90" id="tiltSlider" oninput="updateTilt(this.value)">
                        </div>
                        <div class="button-group">
                            <button class="btn btn-primary" onclick="centerHead()">Center Head</button>
                        </div>
                    </div>
                    <!-- Camera Controls -->
                    <div class="control-group">
                        <h2>Camera Controls</h2>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Pan: <span id="cameraPanAngle">90</span>°</span>
                            </div>
                            <input type="range" min="0" max="180" value="90" id="cameraPanSlider" oninput="updateCameraPan(this.value)">
                        </div>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Tilt: <span id="cameraTiltAngle">90</span>°</span>
                            </div>
                            <input type="range" min="0" max="180" value="90" id="cameraTiltSlider" oninput="updateCameraTilt(this.value)">
                        </div>
                        <div class="button-group">
                            <button class="btn btn-info" onclick="centerCamera()">Center Camera</button>
                        </div>
                    </div>
                    <!-- Ear Controls -->
                    <div class="control-group">
                        <h2>Ear Controls</h2>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Right Ear: <span id="rightEarAngle">90</span>°</span>
                            </div>
                            <input type="range" min="0" max="180" value="90" id="rightEarSlider" oninput="updateRightEar(this.value)">
                        </div>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Left Ear: <span id="leftEarAngle">90</span>°</span>
                            </div>
                            <input type="range" min="0" max="180" value="90" id="leftEarSlider" oninput="updateLeftEar(this.value)">
                        </div>
                        <div class="button-group">
                            <button class="btn btn-warning" onclick="centerEars()">Center Ears</button>
                        </div>
                    </div>
                    <!-- Speed Controls -->
                    <div class="control-group">
                        <h2>Speed Controls</h2>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Pan Speed: <span id="panSpeedValue">3.5</span></span>
                            </div>
                            <input type="range" min="0.5" max="5" step="0.1" value="3.5" id="panSpeedSlider" oninput="updatePanSpeed(this.value)">
                        </div>
                        <div class="slider-container">
                            <div class="slider-label">
                                <span>Tilt Speed: <span id="tiltSpeedValue">1.5</span></span>
                            </div>
                            <input type="range" min="0.5" max="5" step="0.1" value="1.5" id="tiltSpeedSlider" oninput="updateTiltSpeed(this.value)">
                        </div>
                    </div>
                    <!-- Demo Controls -->
                    <div class="control-group">
                        <h2>Demo Sequences</h2>
                        <div class="button-group">
                            <button class="btn btn-success" onclick="startDemo1()">Start Demo 1</button>
                            <button class="btn btn-success" onclick="startDemo2()">Start Demo 2</button>
                        </div>
                    </div>
                    <!-- Additional Animations -->
                    <div class="control-group">
                        <h2>Animations</h2>
                        <div class="button-group">
                            <button class="btn btn-primary" onclick="nodYes()">Yes</button>
                            <button class="btn btn-primary" onclick="shakeNo()">No</button>
                        </div>
                    </div>
                </div>
            </div>
            <script>
                // Update functions
                function updatePan(value) {
                    document.getElementById('panAngle').innerText = value;
                    fetch('/set_pan?value=' + value);
                }
                function updateTilt(value) {
                    document.getElementById('tiltAngle').innerText = value;
                    fetch('/set_tilt?value=' + value);
                }
                function updateCameraPan(value) {
                    document.getElementById('cameraPanAngle').innerText = value;
                    fetch('/set_camera_pan?value=' + value);
                }
                function updateCameraTilt(value) {
                    document.getElementById('cameraTiltAngle').innerText = value;
                    fetch('/set_camera_tilt?value=' + value);
                }
                function updateRightEar(value) {
                    document.getElementById('rightEarAngle').innerText = value;
                    fetch('/set_right_ear?value=' + value);
                }
                function updateLeftEar(value) {
                    document.getElementById('leftEarAngle').innerText = value;
                    fetch('/set_left_ear?value=' + value);
                }
                // Speed adjustment functions
                function updatePanSpeed(value) {
                    document.getElementById('panSpeedValue').innerText = value;
                    fetch('/set_pan_speed?value=' + value);
                }
                function updateTiltSpeed(value) {
                    document.getElementById('tiltSpeedValue').innerText = value;
                    fetch('/set_tilt_speed?value=' + value);
                }
                // Center functions
                function centerHead() {
                    document.getElementById('panSlider').value = 117;
                    document.getElementById('tiltSlider').value = 90;
                    updatePan(117);
                    updateTilt(90);
                }
                function centerCamera() {
                    document.getElementById('cameraPanSlider').value = 90;
                    document.getElementById('cameraTiltSlider').value = 90;
                    updateCameraPan(90);
                    updateCameraTilt(90);
                }
                function centerEars() {
                    document.getElementById('rightEarSlider').value = 90;
                    document.getElementById('leftEarSlider').value = 90;
                    updateRightEar(90);
                    updateLeftEar(90);
                }
                // Demo functions
                function startDemo1() {
                    fetch('/start_demo1');
                }
                function startDemo2() {
                    fetch('/start_demo2');
                }
                // Animation functions
                function nodYes() {
                    fetch('/nod_yes');
                }
                function shakeNo() {
                    fetch('/shake_no');
                }
                // Fetch current angles periodically
                function updateAngles() {
                    fetch('/angles')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('panAngle').innerText = data.pan;
                            document.getElementById('tiltAngle').innerText = data.tilt;
                            document.getElementById('cameraPanAngle').innerText = data.cameraPan;
                            document.getElementById('cameraTiltAngle').innerText = data.cameraTilt;
                            document.getElementById('rightEarAngle').innerText = data.rightEar;
                            document.getElementById('leftEarAngle').innerText = data.leftEar;
                            document.getElementById('panSlider').value = data.pan;
                            document.getElementById('tiltSlider').value = data.tilt;
                            document.getElementById('cameraPanSlider').value = data.cameraPan;
                            document.getElementById('cameraTiltSlider').value = data.cameraTilt;
                            document.getElementById('rightEarSlider').value = data.rightEar;
                            document.getElementById('leftEarSlider').value = data.leftEar;
                            document.getElementById('panSpeedValue').innerText = data.panSpeed;
                            document.getElementById('tiltSpeedValue').innerText = data.tiltSpeed;
                            document.getElementById('panSpeedSlider').value = data.panSpeed;
                            document.getElementById('tiltSpeedSlider').value = data.tiltSpeed;
                        });
                }
                setInterval(updateAngles, 1000);
            </script>
        </body>
        </html>
        )rawliteral";
        request->send(200, "text/html", html);
    });

    // API endpoint to return current angles
    server.on("/angles", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"pan\":" + String(panAngle) + ",";
        json += "\"tilt\":" + String(tiltAngle) + ",";
        json += "\"cameraPan\":" + String(cameraPanAngle) + ",";
        json += "\"cameraTilt\":" + String(cameraTiltAngle) + ",";
        json += "\"rightEar\":" + String(rightEarAngle) + ",";
        json += "\"leftEar\":" + String(leftEarAngle) + ",";
        json += "\"panSpeed\":" + String(panSpeed) + ",";
        json += "\"tiltSpeed\":" + String(tiltSpeed);
        json += "}";
        request->send(200, "application/json", json);
    });

    // Handlers for sliders
    server.on("/set_pan", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            float value = request->getParam("value")->value().toFloat();
            startMovement(value, targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
        }
        request->send(204);
    });

    server.on("/set_tilt", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            float value = request->getParam("value")->value().toFloat();
            startMovement(targetPanAngle, value, targetRightEarAngle, targetLeftEarAngle);
        }
        request->send(204);
    });

    server.on("/set_camera_pan", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            float value = request->getParam("value")->value().toFloat();
            startCameraMovement(value, targetCameraTiltAngle);
        }
        request->send(204);
    });

    server.on("/set_camera_tilt", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            float value = request->getParam("value")->value().toFloat();
            startCameraMovement(targetCameraPanAngle, value);
        }
        request->send(204);
    });

    server.on("/set_right_ear", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            targetRightEarAngle = request->getParam("value")->value().toFloat();
            isEarMoving = true;
        }
        request->send(204);
    });

    server.on("/set_left_ear", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            targetLeftEarAngle = request->getParam("value")->value().toFloat();
            isEarMoving = true;
        }
        request->send(204);
    });

    // Handlers for speed adjustments
    server.on("/set_pan_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            panSpeed = request->getParam("value")->value().toFloat();
        }
        request->send(204);
    });

    server.on("/set_tilt_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            tiltSpeed = request->getParam("value")->value().toFloat();
        }
        request->send(204);
    });

    // Demo handlers
    server.on("/start_demo1", HTTP_GET, [](AsyncWebServerRequest *request){
        startDemo1();
        request->send(204);
    });

    server.on("/start_demo2", HTTP_GET, [](AsyncWebServerRequest *request){
        startDemo2();
        request->send(204);
    });

    // Animation handlers
    server.on("/nod_yes", HTTP_GET, [](AsyncWebServerRequest *request){
        startNodYes();
        request->send(204);
    });

    server.on("/shake_no", HTTP_GET, [](AsyncWebServerRequest *request){
        startShakeNo();
        request->send(204);
    });

    // Start server
    server.begin();
    Serial.println("Server started.");
}

void loop() {
    // Non-blocking update of movement
    if (isMoving) {
        updateMovement();
    }

    // Non-blocking update of camera movement
    if (isCameraMoving) {
        updateCameraMovement();
    }

    // Non-blocking update of ear movement
    if (isEarMoving) {
        updateEarMovement();
    }

    // Update automatic movement of pins 6 and 7
    updateAutoPins();

    // Update sequences if running
    updateDemo1();
    updateDemo2();
    updateNodYes();
    updateShakeNo();
    updateStartupSequence();
}

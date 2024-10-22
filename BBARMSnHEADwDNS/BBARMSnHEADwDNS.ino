#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h> // Include mDNS library

// Wi-Fi credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

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
float panAngle = 120;  // New centered pan (looking forward)
float tiltAngle = 90;  // New centered tilt (looking forward)
float rightEarAngle = 90;  // Start right ear at 90 degrees
float leftEarAngle = 90;  // Start left ear at 90 degrees
float cameraPanAngle = 90;  // Camera's resting pan position
float cameraTiltAngle = 90; // Camera's resting tilt position
float targetPanAngle = 120;
float targetTiltAngle = 90;
float targetRightEarAngle = 90;  // "Ready" position
float targetLeftEarAngle = 90;   // "Ready" position
float targetCameraPanAngle = 90;  // Camera pan target angle
float targetCameraTiltAngle = 90; // Camera tilt target angle
bool isMoving = false;
bool isCameraMoving = false;
bool isEarMoving = false;  // Track ear movement separately

// Speed and acceleration settings
float panSpeed = 2;   // Adjust the pan speed dynamically
float tiltSpeed = 2;  // Adjust the tilt speed dynamically
float panAcceleration = 0.1;  // Adjust pan acceleration
float tiltAcceleration = 0.1; // Adjust tilt acceleration
float cameraPanSpeed = 2;     // Camera pan speed
float cameraTiltSpeed = 2;    // Camera tilt speed
float earSpeed = 2;           // Ear speed

// Timing
unsigned long previousMillis = 0;
unsigned long autoMoveMillis = 0;
const long interval = 20;  // Adjusted time between each step
const long autoMoveInterval = 2000;  // Time between auto movements (in milliseconds)

// Adjusted Pulse Width Range
#define SERVOMIN  100  // Minimum pulse length out of 4096 (adjusted for MG995)
#define SERVOMAX  650  // Maximum pulse length out of 4096 (adjusted for MG995)

// Adjusted Pulse Width Range for Micro 9g Servos (camera and ears)
#define MICRO_SERVOMIN 150  // Minimum pulse length out of 4096 (adjusted for MG90S/Micro 9g)
#define MICRO_SERVOMAX 600  // Maximum pulse length out of 4096 (adjusted for MG90S/Micro 9g)

// Web server
AsyncWebServer server(80);

// Function to convert angle to pulse length for the right ear
uint16_t rightEarAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 0, 180, MICRO_SERVOMIN, MICRO_SERVOMAX); // Micro 9g pulse range
}

// Function to convert angle to pulse length for the left ear
uint16_t leftEarAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 0, 180, MICRO_SERVOMIN, MICRO_SERVOMAX); // Micro 9g pulse range
}

// Function to convert angle to pulse length for pan/tilt (MG995 servos)
uint16_t angleToPulse(float angle, bool isTilt) {
    if (isTilt) {
        return map(angle, 50, 160, SERVOMIN, SERVOMAX);  // Tilt range: 50 (up) to 160 (down)
    } else {
        return map(angle, -10, 200, SERVOMIN, SERVOMAX); // Pan range: -10 to 200
    }
}

// Function to convert angle to pulse length for the camera (micro 9g servos)
uint16_t cameraAngleToPulse(float angle) {
    return map(constrain(angle, 0, 180), 0, 180, MICRO_SERVOMIN, MICRO_SERVOMAX); // Micro 9g pulse range
}

// Function to convert angle to pulse length for auto pins
uint16_t autoPinAngleToPulse(float angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
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

// Function to set Pin 8 to 90 degrees
void setPin8To90() {
    pwm.setPWM(8, 0, autoPinAngleToPulse(90));
    Serial.println("Pin 8 set to 90 degrees.");
}

// Function to start smooth movement
void startMovement(float newTargetPan, float newTargetTilt, float newTargetRightEar, float newTargetLeftEar) {
    targetPanAngle = newTargetPan;
    targetTiltAngle = newTargetTilt;
    targetRightEarAngle = newTargetRightEar;
    targetLeftEarAngle = newTargetLeftEar;
    isMoving = true;
    isEarMoving = true;  // Ensure ear movement is active
}

// Function to start smooth camera movement
void startCameraMovement(float newTargetCameraPan, float newTargetCameraTilt) {
    targetCameraPanAngle = newTargetCameraPan;
    targetCameraTiltAngle = newTargetCameraTilt;
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
            float constrainedTilt = constrain(targetTiltAngle, 70, 160); // Limit downward tilt to 70 degrees
            float step = tiltSpeed * tiltAcceleration;
            if (abs(tiltAngle - constrainedTilt) > step) {
                tiltAngle += (tiltAngle < constrainedTilt) ? step : -step;
            } else {
                tiltAngle = constrainedTilt; // Snap to target if very close
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
            pwm.setPWM(servoLeftEarChannel, 0, leftEarAngleToPulse(leftEarAngle));
        }

        // Stop ear movement when targets are reached
        if (rightEarAngle == targetRightEarAngle && leftEarAngle == targetLeftEarAngle) {
            isEarMoving = false;
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize PWM driver with a slightly higher frequency for MG995 servos
    pwm.begin();
    pwm.setPWMFreq(65);  // Increase PWM frequency slightly for better MG995 response

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

    // Initialize mDNS with the desired hostname
    if (!MDNS.begin("bb1arms")) { // Change "bb1arms" to your desired friendly name
        Serial.println("Error starting mDNS");
    } else {
        Serial.println("mDNS responder started. Access via http://bb1arms.local");
    }

    // Move to initial position on startup
    startMovement(120, 90, 90, 90);  // Ears set to 90 degrees
    startCameraMovement(90, 90);  // Camera pan and tilt to 90 degrees
    Serial.println("Moved to initial position on startup (Pan: 120°, Tilt: 90°, Right Ear: 90°, Left Ear: 90°, Camera Pan: 90°, Camera Tilt: 90°).");

    // Set fixed pins to center
    setFixedPinsToCenter();

    // Define web server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><head><style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; margin: 0; padding: 0; }";
        html += ".container { max-width: 100%; margin: auto; display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; }";
        html += ".btn { padding: 10px; font-size: 14px; cursor: pointer; background-color: #008CBA; color: white; border: none; border-radius: 5px; }";
        html += ".btn:active { background-color: #005f73; }";
        html += ".info { grid-column: span 2; padding: 10px; border: 1px solid #ddd; border-radius: 5px; background: #f9f9f9; }";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<div class='info'><h1>BB-1 Head Control</h1></div>";
        html += "<div class='info'><strong>Pan Angle:</strong> <span id='panAngle'>" + String(panAngle) + "</span>°</div>";
        html += "<div class='info'><strong>Tilt Angle:</strong> <span id='tiltAngle'>" + String(tiltAngle) + "</span>°</div>";
        html += "<div class='info'><strong>Right Ear Angle:</strong> <span id='rightEarAngle'>" + String(rightEarAngle) + "</span>°</div>";
        html += "<div class='info'><strong>Left Ear Angle:</strong> <span id='leftEarAngle'>" + String(leftEarAngle) + "</span>°</div>";
        html += "<div class='info'><strong>Pan Speed:</strong> <span id='panSpeed'>" + String(panSpeed) + "</span></div>";
        html += "<div class='info'><strong>Tilt Speed:</strong> <span id='tiltSpeed'>" + String(tiltSpeed) + "</span></div>";
        html += "<div class='info'><strong>Pan Acceleration:</strong> <span id='panAcceleration'>" + String(panAcceleration) + "</span></div>";
        html += "<div class='info'><strong>Tilt Acceleration:</strong> <span id='tiltAcceleration'>" + String(tiltAcceleration) + "</span></div>";
        html += "<button class='btn' onclick=\"fetch('/increase_pan_speed')\">Increase Pan Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_pan_speed')\">Decrease Pan Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_tilt_speed')\">Increase Tilt Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_tilt_speed')\">Decrease Tilt Speed</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_pan_acceleration')\">Increase Pan Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_pan_acceleration')\">Decrease Pan Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/increase_tilt_acceleration')\">Increase Tilt Acceleration</button>";
        html += "<button class='btn' onclick=\"fetch('/decrease_tilt_acceleration')\">Decrease Tilt Acceleration</button>";
        html += "<button class='btn' onmousedown=\"fetch('/up')\" onmouseup=\"fetch('/stop')\">Tilt Head Up</button>";
        html += "<button class='btn' onmousedown=\"fetch('/down')\" onmouseup=\"fetch('/stop')\">Tilt Head Down</button>";
        html += "<button class='btn' onmousedown=\"fetch('/left')\" onmouseup=\"fetch('/stop')\">Pan Left</button>";
        html += "<button class='btn' onmousedown=\"fetch('/right')\" onmouseup=\"fetch('/stop')\">Pan Right</button>";
        html += "<button class='btn' onclick=\"fetch('/center')\">Center</button>";
        html += "<button class='btn' onclick=\"fetch('/right_ear_back')\">Right Ear Back</button>";
        html += "<button class='btn' onclick=\"fetch('/right_ear_forward')\">Right Ear Forward</button>";
        html += "<button class='btn' onclick=\"fetch('/left_ear_back')\">Left Ear Back</button>";
        html += "<button class='btn' onclick=\"fetch('/left_ear_forward')\">Left Ear Forward</button>";
        html += "<button class='btn' onclick=\"fetch('/camera_pan_left')\">Camera Pan Left</button>";
        html += "<button class='btn' onclick=\"fetch('/camera_pan_right')\">Camera Pan Right</button>";
        html += "<button class='btn' onclick=\"fetch('/camera_tilt_up')\">Camera Tilt Up</button>";
        html += "<button class='btn' onclick=\"fetch('/camera_tilt_down')\">Camera Tilt Down</button>";
        html += "<button class='btn' onclick=\"fetch('/pin8_90')\">Pin 8 to 90°</button>"; // Button for Pin 8 to 90 degrees
        html += "</div>";
        html += "<script>";
        html += "function updateAngles() {";
        html += "fetch('/angles').then(response => response.json()).then(data => {";
        html += "document.getElementById('panAngle').innerText = data.pan;";
        html += "document.getElementById('tiltAngle').innerText = data.tilt;";
        html += "document.getElementById('rightEarAngle').innerText = data.rightEar;";
        html += "document.getElementById('leftEarAngle').innerText = data.leftEar;";
        html += "document.getElementById('panSpeed').innerText = data.panSpeed;";
        html += "document.getElementById('tiltSpeed').innerText = data.tiltSpeed;";
        html += "document.getElementById('panAcceleration').innerText = data.panAcceleration;";
        html += "document.getElementById('tiltAcceleration').innerText = data.tiltAcceleration;";
        html += "});";
        html += "}";
        html += "setInterval(updateAngles, 1000);";  // Update angles every second
        html += "</script>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    // API endpoint to return current angles, speeds, and accelerations
    server.on("/angles", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"pan\":" + String(panAngle) + ",";
        json += "\"tilt\":" + String(tiltAngle) + ",";
        json += "\"cameraPan\":" + String(cameraPanAngle) + ",";
        json += "\"cameraTilt\":" + String(cameraTiltAngle) + ",";
        json += "\"panSpeed\":" + String(panSpeed) + ",";
        json += "\"tiltSpeed\":" + String(tiltSpeed) + ",";
        json += "\"panAcceleration\":" + String(panAcceleration) + ",";
        json += "\"tiltAcceleration\":" + String(tiltAcceleration) + ",";
        json += "\"rightEar\":" + String(rightEarAngle) + ",";
        json += "\"leftEar\":" + String(leftEarAngle);
        json += "}";
        request->send(200, "application/json", json);
    });

    // Endpoint to set Pin 8 to 90 degrees
    server.on("/pin8_90", HTTP_GET, [](AsyncWebServerRequest *request){
        setPin8To90();
        request->send(204); // No content response
    });

    // Movement control handlers
    server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, constrain(targetTiltAngle + 10, 70, 160), targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(targetPanAngle, constrain(targetTiltAngle - 10, 70, 160), targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(constrain(targetPanAngle + 10, -10, 200), targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(constrain(targetPanAngle - 10, -10, 200), targetTiltAngle, targetRightEarAngle, targetLeftEarAngle);
        request->send(204); // No content response
    });

    server.on("/center", HTTP_GET, [](AsyncWebServerRequest *request){
        startMovement(120, 90, 90, 90);  // Updated center positions including ears
        startCameraMovement(90, 90);  // Reset camera to center
        request->send(204); // No content response
    });

    // Ear control handlers
    server.on("/right_ear_back", HTTP_GET, [](AsyncWebServerRequest *request){
        targetRightEarAngle = constrain(targetRightEarAngle + 5, 0, 180);  // Right ear moves back from 0 to 180 degrees
        isEarMoving = true;
        request->send(204);
    });

    server.on("/right_ear_forward", HTTP_GET, [](AsyncWebServerRequest *request){
        targetRightEarAngle = constrain(targetRightEarAngle - 5, 0, 180);  // Right ear moves forward from 180 to 0 degrees
        isEarMoving = true;
        request->send(204);
    });

    server.on("/left_ear_back", HTTP_GET, [](AsyncWebServerRequest *request){
        targetLeftEarAngle = constrain(targetLeftEarAngle - 5, 0, 180);  // Left ear moves back from 180 to 0 degrees
        isEarMoving = true;
        request->send(204);
    });

    server.on("/left_ear_forward", HTTP_GET, [](AsyncWebServerRequest *request){
        targetLeftEarAngle = constrain(targetLeftEarAngle + 5, 0, 180);  // Left ear moves forward from 0 to 180 degrees
        isEarMoving = true;
        request->send(204);
    });

    // Camera control handlers
    server.on("/camera_pan_left", HTTP_GET, [](AsyncWebServerRequest *request){
        startCameraMovement(constrain(targetCameraPanAngle + 10, 0, 180), targetCameraTiltAngle); // Corrected: left should move left
        request->send(204);
    });

    server.on("/camera_pan_right", HTTP_GET, [](AsyncWebServerRequest *request){
        startCameraMovement(constrain(targetCameraPanAngle - 10, 0, 180), targetCameraTiltAngle); // Corrected: right should move right
        request->send(204);
    });

    server.on("/camera_tilt_up", HTTP_GET, [](AsyncWebServerRequest *request){
        startCameraMovement(targetCameraPanAngle, constrain(targetCameraTiltAngle - 10, 0, 180));
        request->send(204);
    });

    server.on("/camera_tilt_down", HTTP_GET, [](AsyncWebServerRequest *request){
        startCameraMovement(targetCameraPanAngle, constrain(targetCameraTiltAngle + 10, 0, 180));
        request->send(204);
    });

    // Speed and Acceleration Control Endpoints
    server.on("/increase_pan_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        panSpeed += 0.5;
        request->send(204);
    });

    server.on("/decrease_pan_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        panSpeed = max(0.5, panSpeed - 0.5);  // Minimum speed is 0.5
        request->send(204);
    });

    server.on("/increase_tilt_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltSpeed += 0.5;
        request->send(204);
    });

    server.on("/decrease_tilt_speed", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltSpeed = max(0.5, tiltSpeed - 0.5);  // Minimum speed is 0.5
        request->send(204);
    });

    server.on("/increase_pan_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        panAcceleration += 0.05;
        request->send(204);
    });

    server.on("/decrease_pan_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        panAcceleration = max(0.05, panAcceleration - 0.05);  // Minimum acceleration is 0.05
        request->send(204);
    });

    server.on("/increase_tilt_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltAcceleration += 0.05;
        request->send(204);
    });

    server.on("/decrease_tilt_acceleration", HTTP_GET, [](AsyncWebServerRequest *request){
        tiltAcceleration = max(0.05, tiltAcceleration - 0.05);  // Minimum acceleration is 0.05
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
}

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>

// Initialize the PCA9685
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// WiFi credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Web server running on port 80
WebServer server(80);

#define SERVOMIN  150 // Minimum pulse length out of 4096
#define SERVOMAX  600 // Maximum pulse length out of 4096

// Demo control variables
bool demoRunning = false;
int demoNumber = 0;
int demoStep = 0;
unsigned long demoTimer = 0;

// Current angles of the servos
int leftShoulderA_Angle = 90;
int leftShoulderB_Angle = 90;
int leftWristA_Angle = 90;
int leftWristB_Angle = 90;
int leftClaw_Angle = 70; // Adjusted for closed position

int rightShoulderA_Angle = 90;
int rightShoulderB_Angle = 90;
int rightWristA_Angle = 90;
int rightWristB_Angle = 90;
int rightClaw_Angle = 170; // Adjusted for closed position

// Function to set servo angle with a delay to prevent overheating
void setServoAngle(int channel, int angle) {
    uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pca.setPWM(channel, 0, pulse);
    delay(20);  // Add a small delay between movements to reduce load on servos
}

// Function to smoothly move the servos over time
void moveServoSmooth(int channel, int startAngle, int endAngle, int stepDelay, int stepSize) {
    int currentAngle = startAngle;
    if (startAngle < endAngle) {
        for (int angle = startAngle; angle <= endAngle; angle += stepSize) {
            setServoAngle(channel, angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = startAngle; angle >= endAngle; angle -= stepSize) {
            setServoAngle(channel, angle);
            delay(stepDelay);
        }
    }
}

// Function to update the left arm
void updateLeftArm() {
    setServoAngle(0, leftShoulderA_Angle);
    setServoAngle(1, leftShoulderB_Angle);
    setServoAngle(2, leftWristA_Angle);
    setServoAngle(3, leftWristB_Angle);
    setServoAngle(4, leftClaw_Angle);
}

// Function to update the right arm
void updateRightArm() {
    setServoAngle(8, rightShoulderA_Angle);
    setServoAngle(9, rightShoulderB_Angle);
    setServoAngle(10, rightWristA_Angle);
    setServoAngle(11, rightWristB_Angle);
    setServoAngle(12, rightClaw_Angle);
}

// Function to move servos to base pose
void moveToBasePose() {
    leftShoulderA_Angle = 30;
    leftShoulderB_Angle = 49;
    leftWristA_Angle = 169;
    leftWristB_Angle = 90;
    leftClaw_Angle = 70; // Left claw closed

    rightShoulderA_Angle = 145;
    rightShoulderB_Angle = 120;
    rightWristA_Angle = 17;
    rightWristB_Angle = 90;
    rightClaw_Angle = 170; // Right claw closed

    updateLeftArm();
    updateRightArm();
}

// Function to calibrate all servos to 90 degrees
void calibrateTo90() {
    leftShoulderA_Angle = 90;
    leftShoulderB_Angle = 90;
    leftWristA_Angle = 90;
    leftWristB_Angle = 90;
    leftClaw_Angle = 0; // Left claw open (-10 might be overkill but can adjust later)

    rightShoulderA_Angle = 90;
    rightShoulderB_Angle = 90;
    rightWristA_Angle = 90;
    rightWristB_Angle = 90;
    rightClaw_Angle = 70; // Right claw open

    updateLeftArm();
    updateRightArm();
}

// Handle root URL "/"
void handleRoot() {
    String html = "<html>\
    <head>\
    <title>Servo Control</title>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <style>\
    body { font-family: Arial, sans-serif; text-align: center; background-color: #282c34; color: white; }\
    .container { margin: 20px auto; max-width: 800px; text-align: left; }\
    h1 { font-size: 24px; }\
    .section { margin-bottom: 30px; }\
    .label { font-size: 18px; margin-right: 10px; display: inline-block; width: 150px; }\
    .slider-container { margin-bottom: 15px; }\
    .angle { font-size: 16px; display: inline-block; width: 50px; }\
    .slider { width: 70%; }\
    .button { padding: 10px 20px; margin: 20px 0; font-size: 20px; border: none; cursor: pointer; transition: background-color 0.3s, transform 0.2s; }\
    .button:hover { background-color: #61dafb; transform: scale(1.1); }\
    </style>\
    </head>\
    <body>\
    <div class='container'>\
    <h1>Robot Arm Control</h1>\
    <button class='button' onclick=\"sendRequest('/basepose')\">Move to Base Pose</button>\
    <button class='button' onclick=\"sendRequest('/calibrate90')\">Calibrate to 90°</button>\
    <button class='button' onclick=\"sendRequest('/demo1')\">Run Demo 1</button>\
    <div class='section'>\
    <h2>Left Arm</h2>\
    <div class='slider-container'>\
    <label class='label'>Shoulder A</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo0' onchange=\"updateServo(0, this.value)\">\
    <span class='angle' id='angle0'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Shoulder B</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo1' onchange=\"updateServo(1, this.value)\">\
    <span class='angle' id='angle1'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Wrist A</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo2' onchange=\"updateServo(2, this.value)\">\
    <span class='angle' id='angle2'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Wrist B</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo3' onchange=\"updateServo(3, this.value)\">\
    <span class='angle' id='angle3'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Claw</label>\
    <input type='range' min='-10' max='70' value='70' class='slider' id='servo4' onchange=\"updateServo(4, this.value)\">\
    <span class='angle' id='angle4'>70° (Closed)</span>\
    </div>\
    </div>\
    <div class='section'>\
    <h2>Right Arm</h2>\
    <div class='slider-container'>\
    <label class='label'>Shoulder A</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo8' onchange=\"updateServo(8, this.value)\">\
    <span class='angle' id='angle8'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Shoulder B</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo9' onchange=\"updateServo(9, this.value)\">\
    <span class='angle' id='angle9'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Wrist A</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo10' onchange=\"updateServo(10, this.value)\">\
    <span class='angle' id='angle10'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Wrist B</label>\
    <input type='range' min='0' max='180' value='90' class='slider' id='servo11' onchange=\"updateServo(11, this.value)\">\
    <span class='angle' id='angle11'>90°</span>\
    </div>\
    <div class='slider-container'>\
    <label class='label'>Claw</label>\
    <input type='range' min='70' max='170' value='170' class='slider' id='servo12' onchange=\"updateServo(12, this.value)\">\
    <span class='angle' id='angle12'>170° (Closed)</span>\
    </div>\
    </div>\
    </div>\
    <script>\
    function sendRequest(endpoint) {\
        fetch(endpoint)\
            .then(response => {\
                console.log('Request sent to ' + endpoint);\
            })\
            .catch(err => {\
                console.error('Error:', err);\
            });\
    }\
    function updateServo(channel, angle) {\
        document.getElementById('angle' + channel).innerText = angle + '°';\
        fetch(`/moveServo?channel=${channel}&angle=${angle}`);\
    }\
    </script>\
    </body>\
    </html>";
    server.send(200, "text/html", html);
}

// Handle base pose request
void handleBasePose() {
    moveToBasePose();
    server.send(200, "text/plain", "Moved to base pose");
}

// Handle calibrate to 90 request
void handleCalibrate90() {
    calibrateTo90();
    server.send(200, "text/plain", "Calibrated all servos to 90 degrees");
}

// Handle demo request
void handleDemo1() {
    if (!demoRunning) {
        demoRunning = true;
        demoNumber = 1;
        demoStep = 0;
        demoTimer = millis();
        server.send(200, "text/plain", "Demo 1 started");
    } else {
        server.send(200, "text/plain", "Demo already running");
    }
}

// Handle move servo request
void handleMoveServo() {
    if (server.hasArg("channel") && server.hasArg("angle")) {
        int channel = server.arg("channel").toInt();
        int angle = server.arg("angle").toInt();

        // Update the specific servo based on channel
        if (channel >= 0 && channel <= 4) {
            switch (channel) {
                case 0: leftShoulderA_Angle = angle; break;
                case 1: leftShoulderB_Angle = angle; break;
                case 2: leftWristA_Angle = angle; break;
                case 3: leftWristB_Angle = angle; break;
                case 4: leftClaw_Angle = angle; break;
            }
            updateLeftArm();
        } else if (channel >= 8 && channel <= 12) {
            switch (channel) {
                case 8: rightShoulderA_Angle = angle; break;
                case 9: rightShoulderB_Angle = angle; break;
                case 10: rightWristA_Angle = angle; break;
                case 11: rightWristB_Angle = angle; break;
                case 12: rightClaw_Angle = angle; break;
            }
            updateRightArm();
        }

        server.send(200, "text/plain", "Moved servo");
    } else {
        server.send(400, "text/plain", "Invalid request");
    }
}

// Demo sequence handler
void demoSequence() {
    unsigned long currentMillis = millis();
    if (demoRunning && demoNumber == 1) {
        switch (demoStep) {
            case 0: // Move to Calibrate Pose
                calibrateTo90();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 1: // Pause
                if (currentMillis - demoTimer > 2000) {
                    demoStep++;
                }
                break;
            case 2: // Move to Base Pose
                moveToBasePose();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 3: // Pause
                if (currentMillis - demoTimer > 2000) {
                    demoStep++;
                }
                break;
            case 4: // Move to Picture 1 Pose
                // Set angles for Picture 1 (as per image 1)
                leftShoulderA_Angle = 34;
                leftShoulderB_Angle = 136;
                leftWristA_Angle = 0;
                leftWristB_Angle = 21;
                leftClaw_Angle = 58;

                rightShoulderA_Angle = 117;
                rightShoulderB_Angle = 24;
                rightWristA_Angle = 175;
                rightWristB_Angle = 135;
                rightClaw_Angle = 111;

                updateLeftArm();
                updateRightArm();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 5: // Pause
                if (currentMillis - demoTimer > 3000) {
                    demoStep++;
                }
                break;
            case 6: // Move to Picture 2 Pose
                // Set angles for Picture 2 (as per image 2)
                leftShoulderA_Angle = 106;
                leftShoulderB_Angle = 38;
                leftWristA_Angle = 58;
                leftWristB_Angle = 77;
                leftClaw_Angle = 58;

                rightShoulderA_Angle = 50;
                rightShoulderB_Angle = 81;
                rightWristA_Angle = 101;
                rightWristB_Angle = 49;
                rightClaw_Angle = 111;

                updateLeftArm();
                updateRightArm();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 7: // Pause
                if (currentMillis - demoTimer > 3000) {
                    demoStep++;
                }
                break;
            case 8: // Move wrists back and forth
                moveServoSmooth(10, 87, 175, 50, 20); // Right Wrist A
                moveServoSmooth(2, 0, 67, 50, 20);    // Left Wrist A
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 9: // Pause
                if (currentMillis - demoTimer > 1000) {
                    demoStep++;
                }
                break;
            case 10: // Move to Picture 3 Pose
                // Set angles for Picture 3 (as per image 3)
                leftShoulderA_Angle = 42;
                leftShoulderB_Angle = 59;
                leftWristA_Angle = 67;
                leftWristB_Angle = 19;
                leftClaw_Angle = 58;

                rightShoulderA_Angle = 26;
                rightShoulderB_Angle = 44;
                rightWristA_Angle = 169;
                rightWristB_Angle = 90;
                rightClaw_Angle = 111;

                updateLeftArm();
                updateRightArm();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 11: // Pause
                if (currentMillis - demoTimer > 3000) {
                    demoStep++;
                }
                break;
            case 12: // Move to Picture 4 Pose
                // Set angles for Picture 4 (as per image 4)
                leftShoulderA_Angle = 54;
                leftShoulderB_Angle = 49;
                leftWristA_Angle = 67;
                leftWristB_Angle = 25;
                leftClaw_Angle = 58;

                rightShoulderA_Angle = 91;
                rightShoulderB_Angle = 117;
                rightWristA_Angle = 169;
                rightWristB_Angle = 129;
                rightClaw_Angle = 111;

                updateLeftArm();
                updateRightArm();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 13: // Pause
                if (currentMillis - demoTimer > 3000) {
                    demoStep++;
                }
                break;
            case 14: // Return to Base Pose
                moveToBasePose();
                demoStep++;
                demoTimer = currentMillis;
                break;
            case 15: // Pause before calibrating to 90
                if (currentMillis - demoTimer > 2000) {
                    calibrateTo90();
                    demoRunning = false;
                }
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize PCA9685
    pca.begin();
    pca.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Define web server routes
    server.on("/", handleRoot);
    server.on("/basepose", handleBasePose);
    server.on("/calibrate90", handleCalibrate90);
    server.on("/demo1", handleDemo1);
    server.on("/moveServo", handleMoveServo);

    // Start the server
    server.begin();
    Serial.println("Server started");
}

void loop() {
    server.handleClient();
    demoSequence(); // Continuously check and update the demo sequence
}

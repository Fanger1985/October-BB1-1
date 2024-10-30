#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <Wire.h>
#include <MPU6050.h>
#include <stack>
#include <NewPing.h>

// Point structure for tracking coordinates
struct Point {
    int x, y;
};

// Global declarations
std::vector<int> distances;
std::vector<String> actions;
std::stack<Point> pathStack;
std::stack<Point> movementStack;

int currentPosition = 0;
int score = 0;
std::map<int, std::vector<int>> environmentMap;
const char* ssid = "BB1";
const char* password = "totallysecure";

// Motor control pins (DRV8871)
#define MOTOR_LEFT_IN1 16
#define MOTOR_LEFT_IN2 17
#define MOTOR_RIGHT_IN1 25
#define MOTOR_RIGHT_IN2 26

// Ultrasonic sensor setup
#define ULTRASONIC_TRIGGER 18  // Shared Trigger pin for all ultrasonic sensors
#define FRONT_ULTRASONIC_ECHO 15
#define BACK_ULTRASONIC_ECHO 32
#define LEFT_ULTRASONIC_ECHO 33
#define RIGHT_ULTRASONIC_ECHO 12
#define EXTRA_FRONT_ULTRASONIC_ECHO 13

// Maximum distance for ultrasonic sensors (in cm)
#define MAX_DISTANCE 500  // Maximum distance to ping

// Create NewPing objects for each sensor
NewPing frontSensor(ULTRASONIC_TRIGGER, FRONT_ULTRASONIC_ECHO, MAX_DISTANCE);
NewPing backSensor(ULTRASONIC_TRIGGER, BACK_ULTRASONIC_ECHO, MAX_DISTANCE);
NewPing leftSensor(ULTRASONIC_TRIGGER, LEFT_ULTRASONIC_ECHO, MAX_DISTANCE);
NewPing rightSensor(ULTRASONIC_TRIGGER, RIGHT_ULTRASONIC_ECHO, MAX_DISTANCE);
NewPing extraFrontSensor(ULTRASONIC_TRIGGER, EXTRA_FRONT_ULTRASONIC_ECHO, MAX_DISTANCE);

// MPU6050 setup
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool isManualControl = true;
bool isStuck = false;

unsigned long startTime = 0;
unsigned long lastActionTime = 0;

volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
int lastLeftPulseCount = 0;
int lastRightPulseCount = 0;
int speedLeft = 255;
int speedRight = 255;
volatile int leftHallPulseCount = 0;
volatile int rightHallPulseCount = 0;

WebServer server(80);

// Movement function prototypes
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void stopMotors();
void danceRoutine();
void exploreEnvironment();
void autoMove();
void cautiousApproach();
void idleWander();
void reactToCloseObstacle();
void expressEmotion(String emotion);
void adjustBehaviorBasedOnScore();
void calculateScore(bool avoidedObstacle);
void updateMap();
void navigate();
void manageMemory();
void navigateBasedOnDistance(int frontDist, int rearDist);
void moveTo(Point step);
void returnToStart();
void turnAround();
void handleFrontObstacle();
void handleRearObstacle();
void performTaskA();
void performTaskB();
void performTaskC();
void triggerEmotion(String emotion);
void handleExpressEmotion();
void moonwalk();
void checkIfStuck();
String getGyroData();
String controlPage();

void recordPath(int distance, String action);
int getUltrasonicDistance(NewPing &sensor);

void IRAM_ATTR onLeftHallSensor() {
    leftHallPulseCount++;
}

void IRAM_ATTR onRightHallSensor() {
    rightHallPulseCount++;
}

// Define safe distances
#define SAFE_DISTANCE 40  // Increased safe distance

void navigateBasedOnDistance(int frontDist, int rearDist) {
    if (frontDist < SAFE_DISTANCE) {
        handleFrontObstacle();
    } else if (rearDist < SAFE_DISTANCE) {
        handleRearObstacle();
    } else {
        moveForward();
    }
}

void handleFrontObstacle() {
    stopMotors();
    moveBackward();
    delay(500);  // Move back a bit
    spinRight();  // Try to turn away from the obstacle
    delay(200);
    stopMotors();
}

void handleRearObstacle() {
    stopMotors();
    moveForward();
    delay(500);  // Move forward a bit
    spinLeft();  // Try to turn away from the obstacle
    delay(200);
    stopMotors();
}

int frontDistance, rearDistance, leftDistance, rightDistance, extraFrontDistance;

void setup() {
    Serial.begin(115200);

    // Set up Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    Wire.begin();
    mpu.initialize();

    // Initialize MPU6050
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
    } else {
        Serial.println("MPU6050 connection successful.");
    }
    startTime = millis();

    // Set up motor control pins
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    // Set up ultrasonic sensors (Trigger pin)
    pinMode(ULTRASONIC_TRIGGER, OUTPUT);

    // Set up Hall sensors
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(32, INPUT_PULLUP);
    pinMode(33, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(34), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(35), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(32), onRightHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(33), onRightHallSensor, RISING);

    // Set up server routes
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", controlPage());
    });

    server.on("/forward", HTTP_GET, []() {
        isManualControl = true;
        moveForward();
        server.send(200, "text/plain", "Moving forward");
    });

    server.on("/backward", HTTP_GET, []() {
        isManualControl = true;
        moveBackward();
        server.send(200, "text/plain", "Moving backward");
    });

    server.on("/left", HTTP_GET, []() {
        isManualControl = true;
        spinLeft();
        server.send(200, "text/plain", "Turning left");
    });

    server.on("/right", HTTP_GET, []() {
        isManualControl = true;
        spinRight();
        server.send(200, "text/plain", "Turning right");
    });

    server.on("/stop", HTTP_GET, []() {
        isManualControl = true;
        stopMotors();
        server.send(200, "text/plain", "Stopping");
    });

    server.on("/auto", HTTP_GET, []() {
        isManualControl = false;
        server.send(200, "text/plain", "Switched to auto mode");
    });

    server.on("/explore", HTTP_GET, []() {
        isManualControl = false;
        exploreEnvironment();
        server.send(200, "text/plain", "Exploration mode activated");
    });

    server.on("/dance", HTTP_GET, []() {
        danceRoutine();
        server.send(200, "text/plain", "Dance sequence activated");
    });

    server.on("/return_home", HTTP_GET, []() {
        returnToStart();
        server.send(200, "text/plain", "Returning to start position");
    });

    server.on("/task_a", HTTP_GET, []() {
        performTaskA();
        server.send(200, "text/plain", "Task A activated");
    });

    server.on("/task_b", HTTP_GET, []() {
        performTaskB();
        server.send(200, "text/plain", "Task B activated");
    });

    server.on("/task_c", HTTP_GET, []() {
        performTaskC();
        server.send(200, "text/plain", "Task C activated");
    });

    server.on("/expressEmotion", HTTP_POST, handleExpressEmotion);

    server.on("/sensors", HTTP_GET, []() {
        StaticJsonDocument<256> doc;
        doc["front_distance"] = frontDistance;
        doc["rear_distance"] = rearDistance;
        doc["left_distance"] = leftDistance;
        doc["right_distance"] = rightDistance;
        doc["extra_front_distance"] = extraFrontDistance;

        String sensorData;
        serializeJson(doc, sensorData);
        server.send(200, "application/json", sensorData);
    });

    server.on("/gyro", HTTP_GET, []() {
        String gyroData = getGyroData();
        server.send(200, "application/json", gyroData);
    });

    server.on("/getData", HTTP_GET, []() {
        String data = "{ \"distances\": [";
        for (size_t i = 0; i < distances.size(); ++i) {
            data += String(distances[i]);
            if (i < distances.size() - 1) data += ",";
        }
        data += "], \"actions\": [";
        for (size_t i = 0; i < actions.size(); ++i) {
            data += "\"" + actions[i] + "\"";
            if (i < actions.size() - 1) data += ",";
        }
        data += "] }";

        server.send(200, "application/json", data);
    });

    server.begin();
    Serial.println("HTTP server started. Ready for commands.");
}

void loop() {
    server.handleClient();
    checkIfStuck();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Adjust for the sensor being rotated 90 degrees counterclockwise
    int16_t adjusted_ax = ay;  // X-axis is now Y-axis
    int16_t adjusted_ay = -ax; // Y-axis is now -X-axis
    int16_t adjusted_gx = gy;  // Gyro X-axis is now Gyro Y-axis
    int16_t adjusted_gy = -gx; // Gyro Y-axis is now -Gyro X-axis

    if (abs(adjusted_ax) > 15000 || abs(adjusted_ay) > 15000 || abs(az) > 15000) {
        Serial.println("Significant acceleration detected! Stopping motors.");
        stopMotors();
        return;
    }

    if (abs(adjusted_gx) > 15000 || abs(adjusted_gy) > 15000 || abs(gz) > 15000) {
        Serial.println("Significant angular rate change detected! Stopping motors.");
        stopMotors();
        return;
    }

    // Update distances
    frontDistance = getUltrasonicDistance(frontSensor);
    rearDistance = getUltrasonicDistance(backSensor);
    leftDistance = getUltrasonicDistance(leftSensor);
    rightDistance = getUltrasonicDistance(rightSensor);
    extraFrontDistance = getUltrasonicDistance(extraFrontSensor);

    if (isManualControl) {
        // Manual control mode
    } else {
        // Automatic control mode
        autoMove();
    }
}

int getUltrasonicDistance(NewPing &sensor) {
    unsigned int distance = sensor.ping_cm();
    if (distance == 0) {
        return MAX_DISTANCE;  // Maximum distance if no ping
    } else {
        return distance;
    }
}

void autoMove() {
    adjustBehaviorBasedOnScore();

    if (frontDistance > SAFE_DISTANCE && extraFrontDistance > SAFE_DISTANCE) {
        if (leftDistance > SAFE_DISTANCE && rightDistance > SAFE_DISTANCE) {
            Serial.println("Open space ahead, moving forward...");
            moveForward();
        } else if (leftDistance < SAFE_DISTANCE) {
            Serial.println("Obstacle on the left, turning right...");
            spinRight();
            delay(300);
            stopMotors();
        } else if (rightDistance < SAFE_DISTANCE) {
            Serial.println("Obstacle on the right, turning left...");
            spinLeft();
            delay(300);
            stopMotors();
        } else {
            moveForward();
        }
    } else if (frontDistance < SAFE_DISTANCE || extraFrontDistance < SAFE_DISTANCE) {
        Serial.println("Obstacle detected ahead, deciding action...");
        reactToCloseObstacle();
        calculateScore(false);
    } else {
        Serial.println("Proceeding with caution...");
        cautiousApproach();
    }

    if (score < 0) {
        Serial.println("Low score, applying extra caution...");
        cautiousApproach();
    } else {
        Serial.println("High score, exploring freely...");
        idleWander();
    }
}

// Movement functions
void moveForward() {
    Serial.println("Moving forward...");
    // Left motor moves forward (inverted)
    digitalWrite(MOTOR_LEFT_IN1, HIGH);  
    digitalWrite(MOTOR_LEFT_IN2, LOW);   
    // Right motor moves forward
    digitalWrite(MOTOR_RIGHT_IN1, LOW);  
    digitalWrite(MOTOR_RIGHT_IN2, HIGH); 
}

void moveBackward() {
    Serial.println("Moving backward...");
    // Left motor moves backward (inverted)
    digitalWrite(MOTOR_LEFT_IN1, LOW);   
    digitalWrite(MOTOR_LEFT_IN2, HIGH);  
    // Right motor moves backward
    digitalWrite(MOTOR_RIGHT_IN1, HIGH); 
    digitalWrite(MOTOR_RIGHT_IN2, LOW);  
}

void spinLeft() {
    Serial.println("Spinning left...");
    // Left motor moves forward (inverted)
    digitalWrite(MOTOR_LEFT_IN1, LOW);   
    digitalWrite(MOTOR_LEFT_IN2, HIGH);  
    // Right motor moves backward
    digitalWrite(MOTOR_RIGHT_IN1, LOW);  
    digitalWrite(MOTOR_RIGHT_IN2, HIGH); 

}

void spinRight() {
    Serial.println("Spinning right...");
    // Left motor moves forward (inverted)
    digitalWrite(MOTOR_LEFT_IN1, HIGH);  
    digitalWrite(MOTOR_LEFT_IN2, LOW);   
    // Right motor moves forward
    digitalWrite(MOTOR_RIGHT_IN1, HIGH); 
    digitalWrite(MOTOR_RIGHT_IN2, LOW);  
}


void stopMotors() {
    Serial.println("Stopping motors...");
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// Check if the robot is stuck
void checkIfStuck() {
    static unsigned long lastCheckTime = 0;
    const long checkInterval = 1000;

    if (millis() - lastCheckTime >= checkInterval) {
        if (lastLeftPulseCount == leftHallPulseCount && lastRightPulseCount == rightHallPulseCount) {
            if (!isStuck) {
                Serial.println("BB1 might be stuck, stopping motors...");
                stopMotors();
                isStuck = true;
            }
        } else {
            lastLeftPulseCount = leftHallPulseCount;
            lastRightPulseCount = rightHallPulseCount;
            isStuck = false;
        }
        lastCheckTime = millis();
    }
}

// Express emotion
void expressEmotion(String emotion) {
    Serial.print("BB1 is feeling ");
    Serial.println(emotion);
}

// Adjust behavior based on score
void adjustBehaviorBasedOnScore() {
    if (score < -10) {
        Serial.println("Score is very low. Robot is mad.");
        expressEmotion("mad");
        cautiousApproach();
    } else if (score < 0) {
        Serial.println("Score is low. Robot is grumpy.");
        expressEmotion("sad");
        cautiousApproach();
    } else if (score >= 10) {
        Serial.println("High score! Robot is happy.");
        expressEmotion("happy");
        idleWander();
    } else {
        Serial.println("Score is moderate. Robot might be bored.");
        expressEmotion("bored");
        cautiousApproach();
    }
}

// Calculate score
void calculateScore(bool avoidedObstacle) {
    if (avoidedObstacle) {
        score += 10;
    } else {
        score -= 10;
    }
    adjustBehaviorBasedOnScore();
}

// Idle wander behavior
void idleWander() {
    static int action = -1;

    if (millis() - lastActionTime < 500) {
        return; // Ensure actions are non-blocking by adding delay between actions
    }

    action = random(0, 100);
    if (action < 20) {
        // Idle
        stopMotors();
    } else if (action < 50) {
        moveForward();
    } else if (action < 75) {
        spinLeft();
    } else {
        spinRight();
    }
    expressEmotion("happy");
    lastActionTime = millis();
}

// Cautious approach
void cautiousApproach() {
    if (frontDistance > SAFE_DISTANCE) {
        moveForward();
        expressEmotion("happy");
    } else if (frontDistance < SAFE_DISTANCE) {
        reactToCloseObstacle();
        expressEmotion("startled");
    } else {
        idleWander();
    }
}

// React to close obstacle
void reactToCloseObstacle() {
    stopMotors();
    moveBackward();
    delay(500);
    stopMotors();

    if (random(0, 2)) {
        spinLeft();
        delay(200);
    } else {
        spinRight();
        delay(200);
    }
    stopMotors();
}

// Dance routine
void danceRoutine() {
    if (!checkSpaceForDance()) {
        Serial.println("Not enough space to dance :(");
        return;
    }

    Serial.println("Starting dance routine...");

    // Move forward and spin
    moveForward();
    delay(1000);
    spinLeft();
    delay(1000);
    moveForward();
    delay(1000);
    spinRight();
    delay(1000);

    // Jiggle
    for (int i = 0; i < 5; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }

    // Moonwalk
    moonwalk();
    delay(3000);

    // Spin and wiggle
    spinRight();
    delay(1000);
    for (int i = 0; i < 3; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }
    spinLeft();
    delay(1000);

    // More dance moves
    moveForward();
    delay(1000);
    moveBackward();
    delay(1000);
    spinRight();
    delay(1000);
    spinLeft();
    delay(1000);

    stopMotors();

    Serial.println("Dance routine complete!");
}

// Check space for dance
bool checkSpaceForDance() {
    int frontDistance = getUltrasonicDistance(frontSensor);
    Serial.print("Front distance: ");
    Serial.println(frontDistance);
    if (frontDistance < 100) {
        Serial.println("Not enough space to dance :(");
        return false;
    }
    return true;
}

// Moonwalk
void moonwalk() {
    Serial.println("Moonwalking...");

    // Perform a smooth backward movement with slight left-right adjustments to mimic a moonwalk
    for (int i = 0; i < 10; i++) {
        moveBackward();
        delay(100);
        spinRight();
        delay(50);
        spinLeft();
        delay(50);
    }

    stopMotors();
    Serial.println("Moonwalk complete!");
}

// Record path
void recordPath(int distance, String action) {
    distances.push_back(distance);
    actions.push_back(action);
    if (environmentMap.find(currentPosition) == environmentMap.end()) {
        environmentMap[currentPosition] = {distance};
    } else {
        environmentMap[currentPosition].push_back(distance);
    }
    currentPosition++;
}

// Get gyro data
String getGyroData() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    StaticJsonDocument<200> doc;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;

    String gyroData;
    serializeJson(doc, gyroData);
    return gyroData;
}

// Handle express emotion
void handleExpressEmotion() {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    deserializeJson(doc, body);

    String emotion = doc["emotion"];

    expressEmotion(emotion);

    server.send(200, "text/plain", "Emotion received");
}

// Update map
void updateMap() {
    int currentDistance = frontDistance;
    recordPath(currentDistance, "moveForward");
}


String controlPage() {
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body, html {
    margin: 0;
    padding: 0;
    width: 100%;
    height: 100%;
    display: flex;
    justify-content: center;
    align-items: center;
    overflow: hidden;
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background-color: #0f0f0f;
    color: #e0e0e0;
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
  }

  .container {
    display: flex;
    width: 100%;
    height: 100%;
    padding: 20px;
    box-sizing: border-box;
  }

  .left, .center, .right {
    flex: 1;
    display: flex;
    justify-content: center;
    align-items: center;
    padding: 20px;
    box-sizing: border-box;
  }

  .controls {
    display: grid;
    grid-template-areas: 
      "up up up"
      "left stop right"
      "down down down";
    gap: 10px;
  }

  .controls button {
    padding: 15px;
    background-color: #1a1a1a;
    color: #e0e0e0;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    font-size: 16px;
    box-shadow: 5px 5px 10px #0a0a0a, -5px -5px 10px #2a2a2a;
    position: relative;
  }

  .controls button:hover {
    background-color: #333;
  }

  .status-log {
    flex: 2;
    background-color: #1a1a1a;
    padding: 20px;
    box-shadow: 10px 10px 20px #0a0a0a, -10px -10px 20px #2a2a2a;
    position: relative;
    overflow: hidden;
    text-align: left;
    max-height: 200px;
    overflow-y: auto;
  }

  .sensor-data {
    margin-top: 20px;
    padding: 10px;
    background-color: #1a1a1a;
    box-shadow: 5px 5px 10px #0a0a0a, -5px -5px 10px #2a2a2a;
    border-radius: 10px;
    text-align: center;
  }

  .buttons {
    flex: 1;
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 20px;
  }

  .buttons button {
    padding: 15px;
    background-color: #1a1a1a;
    color: #e0e0e0;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    font-size: 16px;
    box-shadow: 5px 5px 10px #0a0a0a, -5px -5px 10px #2a2a2a;
    position: relative;
  }

  .buttons button:hover {
    background-color: #333;
  }
</style>
</head>
<body>
<div class="container">
  <div class="left">
    <div class="controls">
      <button data-command="/forward" onmousedown='startCommand("/forward")' onmouseup='stopCommand()' ontouchstart='startCommand("/forward")' ontouchend='stopCommand()' style="grid-area: up;">UP</button>
      <button data-command="/left" onmousedown='startCommand("/left")' onmouseup='stopCommand()' ontouchstart='startCommand("/left")' ontouchend='stopCommand()' style="grid-area: left;">LEFT</button>
      <button data-command="/stop" onmousedown='stopCommand()' style="grid-area: stop;">STOP</button>
      <button data-command="/right" onmousedown='startCommand("/right")' onmouseup='stopCommand()' ontouchstart='startCommand("/right")' ontouchend='stopCommand()' style="grid-area: right;">RIGHT</button>
      <button data-command="/backward" onmousedown='startCommand("/backward")' onmouseup='stopCommand()' ontouchstart='startCommand("/backward")' ontouchend='stopCommand()' style="grid-area: down;">DOWN</button>
    </div>
  </div>
  <div class="center">
    <div class="status-log" id="status-log">
      <p>Command log initialized...</p>
    </div>
    <div class="sensor-data">
      <p><strong>Sensor Data</strong></p>
      <p>Front Distance: <span id="front-distance">N/A</span> cm</p>
      <p>Rear Distance: <span id="rear-distance">N/A</span> cm</p>
      <p>Left Distance: <span id="left-distance">N/A</span> cm</p>
      <p>Right Distance: <span id="right-distance">N/A</span> cm</p>
      <p>Extra Front Distance: <span id="extra-front-distance">N/A</span> cm</p>
      <p>Gyro Data: <span id="gyro-data">N/A</span></p>
    </div>
  </div>
  <div class="right">
    <div class="buttons">
      <button onclick='sendCommand("/auto")'>AUTO</button>
      <button onclick='sendCommand("/explore")'>EXPLORE</button>
      <button onclick='sendCommand("/dance")'>DANCE</button>
      <button onclick='sendCommand("/expressEmotion")'>EMOTION</button>
      <button onclick='sendCommand("/task_a")'>TASK A</button>
      <button onclick='sendCommand("/task_b")'>TASK B</button>
      <button onclick='sendCommand("/task_c")'>TASK C</button>
    </div>
  </div>
</div>
<script>
let currentCommand = null;

function sendCommand(command) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', command, true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE) {
            if (xhr.status == 200) {
                var logFrame = document.getElementById("status-log");
                logFrame.innerHTML = "<p>" + command + " command executed.</p>" + logFrame.innerHTML;
            } else {
                console.error("Failed to execute command: " + xhr.status);
            }
        }
    };
    xhr.send();
}

function startCommand(command) {
    if (currentCommand !== command) {
        stopCommand();  // Ensure any previous command is stopped
        currentCommand = command;
        sendCommand(command);
    }
}

function stopCommand() {
    if (currentCommand) {
        sendCommand("/stop");
        currentCommand = null;
    }
}

document.querySelectorAll('.controls button').forEach(button => {
    button.addEventListener('mousedown', function() {
        startCommand(this.getAttribute('data-command'));
    });
    button.addEventListener('mouseup', function() {
        stopCommand();
    });
    button.addEventListener('touchstart', function() {
        startCommand(this.getAttribute('data-command'));
    });
    button.addEventListener('touchend', function() {
        stopCommand();
    });
    button.addEventListener('mouseleave', function() {
        stopCommand();
    });
});

// Periodically fetch sensor and gyro data
function updateSensorData() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/sensors', true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE && xhr.status == 200) {
            var sensorData = JSON.parse(xhr.responseText);
            document.getElementById('front-distance').innerText = sensorData.front_distance;
            document.getElementById('rear-distance').innerText = sensorData.rear_distance;
            document.getElementById('left-distance').innerText = sensorData.left_distance;
            document.getElementById('right-distance').innerText = sensorData.right_distance;
            document.getElementById('extra-front-distance').innerText = sensorData.extra_front_distance;
        }
    };
    xhr.send();
}

function updateGyroData() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/gyro', true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE && xhr.status == 200) {
            var gyroData = JSON.parse(xhr.responseText);
            document.getElementById('gyro-data').innerText = `Gx: ${gyroData.gx}, Gy: ${gyroData.gy}, Gz: ${gyroData.gz}`;
        }
    };
    xhr.send();
}

// Refresh sensor and gyro data every 1 second
setInterval(function() {
    updateSensorData();
    updateGyroData();
}, 1000);

</script>
</body>
</html>

)";
    return html;
}

// Additional functions for tasks and navigation
void exploreEnvironment() {
    Serial.println("Exploring environment...");
    // Implement your environment exploration logic here
}

void returnToStart() {
    Serial.println("Returning to start position...");
    // Implement logic to return to the starting position
}

void performTaskA() {
    Serial.println("Performing Task A...");
    // Implement Task A
}

void performTaskB() {
    Serial.println("Performing Task B...");
    // Implement Task B
}

void performTaskC() {
    Serial.println("Performing Task C...");
    // Implement Task C
}


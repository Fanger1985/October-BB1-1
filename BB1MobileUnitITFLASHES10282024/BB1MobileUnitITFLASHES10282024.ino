#include <WiFi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <MPU9250_asukiaaa.h>
#include <ESPmDNS.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h>
#include <NewPing.h>
#include <Ticker.h>

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

// Gyro setup
MPU9250_asukiaaa mpu9250;
#define GYRO_SDA 21
#define GYRO_SCL 22

// Hall sensor and encoder setup for motor control
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;

#define LEFT_HALL_SENSOR_PIN 34
#define RIGHT_HALL_SENSOR_PIN 35

// Wi-Fi network credentials
const char* ssid = "SpectrumSetup-DD";          
const char* password = "jeansrocket543";  

// Fixed domain name for mDNS
const char* hostName = "bb1mobileunit";

// HTTP server
AsyncWebServer server(80);

// Mode flags
enum RobotMode { MANUAL, AUTONOMOUS, LIFE };
RobotMode currentMode = MANUAL;

// Speed control variables (0-255)
uint8_t leftMotorSpeed = 255;
uint8_t rightMotorSpeed = 255;

// Synchronize motor speeds
bool syncMotorSpeeds = false;

// Maximum motor speed
const uint8_t MAX_SPEED = 255;

// Thresholds for sensors (in mm)
const unsigned long frontThreshold = 300;  
const unsigned long sideThreshold = 200;   
const unsigned long rearThreshold = 200;   

// Tickers for periodic tasks
Ticker autonomousTicker;
Ticker lifeModeTicker;

// Waypoint data structure
struct Waypoint {
  float x;
  float y;
};

#define MAX_WAYPOINTS 100
Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;

// Robot's current position and orientation
float posX = 0.0;
float posY = 0.0;
float orientation = 0.0;

// Occupancy Grid (Memory Map)
#define GRID_SIZE 20
#define GRID_RESOLUTION 0.5f  
int occupancyGrid[GRID_SIZE][GRID_SIZE] = {0}; 

// Behavioral states for Life Mode
enum LifeState { IDLE, EXPLORING, INTERACTING };
LifeState currentLifeState = IDLE;

// Obstacle avoidance states
enum ObstacleState { IDLE_OBSTACLE, MOVING_BACKWARD, TURNING, RESUMING };
ObstacleState obstacleState = IDLE_OBSTACLE;
unsigned long obstacleStateStartTime = 0;

// Function prototypes
void moveMotors(int leftDirection, int rightDirection, int customLeftSpeed, int customRightSpeed);
void stopMotors();
void handleMoveCommand(String direction, bool stop);
unsigned int readUltrasonicSensor(NewPing &sensor);
String getSensorData();
void updatePosition();
void autonomousNavigation();
void lifeModeBehavior();
void handleObstacles();
void updateOccupancyGrid();
Waypoint findNextFrontier();
void smoothStartMotors();
void checkIfStuck();
void IRAM_ATTR onLeftHallSensor();
void IRAM_ATTR onRightHallSensor();

// Sensor reading variables and functions
void readSensors();
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 100;  

unsigned int frontLow = MAX_DISTANCE * 10;
unsigned int frontHigh = MAX_DISTANCE * 10;
unsigned int leftSide = MAX_DISTANCE * 10;
unsigned int rightSide = MAX_DISTANCE * 10;
unsigned int rear = MAX_DISTANCE * 10;

// HTML/JS for control page with speed controls and sync option
const char controlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>BB-1 Control Panel</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #1a1a1a; color: #f2f2f2; }
    .button { padding: 15px 30px; margin: 10px; font-size: 18px; border: none; cursor: pointer; border-radius: 5px; }
    .button:hover { background-color: #555; }
    .status { margin-top: 20px; font-size: 20px; }
    #sensorData { margin-top: 20px; text-align: left; display: inline-block; }
    .control-grid {
      display: grid;
      grid-template-columns: repeat(3, 100px);
      grid-gap: 10px;
      justify-content: center;
    }
    input[type=range] { width: 200px; }
    .slider-label { margin: 10px; }
    #mapCanvas { border: 1px solid #fff; background-color: #333; }
    .mode-buttons { margin: 20px; }
    .sync-container { margin-top: 10px; }
  </style>
</head>
<body>
  <h1>BB-1 Control Panel</h1>

  <div class="mode-buttons">
    <button onclick="switchMode('manual')">Manual Mode</button>
    <button onclick="switchMode('autonomous')">Autonomous Mode</button>
    <button onclick="switchMode('life')">Life Mode</button>
  </div>

  <div id="manualControls" style="display: none;">
    <div class="control-grid">
      <button class="button" id="forwardBtn">Forward</button>
      <button class="button" id="leftBtn">Left</button>
      <button class="button" id="rightBtn">Right</button>
      <button class="button" id="curveLeftBtn">Curve Left</button>
      <button class="button" id="curveRightBtn">Curve Right</button>
      <button class="button" id="backwardBtn">Backward</button>
      <button class="button" onclick="sendMoveCommand('stop', true)">Stop</button>
    </div>

    <div class="slider-label">
      <label for="leftSpeed">Left Motor Speed:</label>
      <input type="range" id="leftSpeed" min="0" max="255" value="255" onchange="updateSpeed()">
    </div>
    <div class="slider-label">
      <label for="rightSpeed">Right Motor Speed:</label>
      <input type="range" id="rightSpeed" min="0" max="255" value="255" onchange="updateSpeed()">
    </div>
    <div class="sync-container">
      <input type="checkbox" id="syncSpeeds" onchange="toggleSync()"> Synchronize Motor Speeds
    </div>
  </div>

  <div class="status">
    <p>Current Mode: <span id="modeText">Manual</span></p>
    <p>Status: <span id="statusText">Idle</span></p>
  </div>

  <h2>Sensor Data</h2>
  <div id="sensorData">
  </div>

  <h2>Map</h2>
  <canvas id="mapCanvas" width="500" height="500"></canvas>

  <script>
    let currentMode = 'Manual';
    let syncEnabled = false;

    function switchMode(mode) {
      fetch(`/mode?mode=${mode}`)
        .then(response => response.text())
        .then(data => {
          currentMode = mode.charAt(0).toUpperCase() + mode.slice(1);
          document.getElementById('modeText').innerText = currentMode;
          if (currentMode === 'Manual') {
            document.getElementById('manualControls').style.display = 'block';
          } else {
            document.getElementById('manualControls').style.display = 'none';
          }
        });
    }

    function sendMoveCommand(direction, stop) {
      fetch(`/move?direction=${direction}&stop=${stop}`)
        .then(response => response.text())
        .then(data => {
          document.getElementById('statusText').innerText = stop ? 'Idle' : direction.charAt(0).toUpperCase() + direction.slice(1);
        });
    }

    function handleButtonEvent(buttonId, direction) {
      const button = document.getElementById(buttonId);

      button.addEventListener('mousedown', () => {
        sendMoveCommand(direction, false);
      });

      button.addEventListener('mouseup', () => {
        sendMoveCommand('stop', true);
      });

      button.addEventListener('touchstart', () => {
        sendMoveCommand(direction, false);
      });
      button.addEventListener('touchend', () => {
        sendMoveCommand('stop', true);
      });
    }

    handleButtonEvent('forwardBtn', 'forward');
    handleButtonEvent('backwardBtn', 'backward');
    handleButtonEvent('leftBtn', 'left');
    handleButtonEvent('rightBtn', 'right');
    handleButtonEvent('curveLeftBtn', 'curve_left');
    handleButtonEvent('curveRightBtn', 'curve_right');

    function updateSpeed() {
      let leftSpeed = document.getElementById('leftSpeed').value;
      let rightSpeed = document.getElementById('rightSpeed').value;
      if (syncEnabled) {
        rightSpeed = leftSpeed;
        document.getElementById('rightSpeed').value = leftSpeed;
      }
      fetch('/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `leftSpeed=${leftSpeed}&rightSpeed=${rightSpeed}&sync=${syncEnabled}`
      });
    }

    function toggleSync() {
      syncEnabled = document.getElementById('syncSpeeds').checked;
      if (syncEnabled) {
        document.getElementById('rightSpeed').disabled = true;
        document.getElementById('rightSpeed').value = document.getElementById('leftSpeed').value;
      } else {
        document.getElementById('rightSpeed').disabled = false;
      }
      updateSpeed();
    }

    function fetchSensorData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          let sensorDataDiv = document.getElementById('sensorData');
          sensorDataDiv.innerHTML = `
            <p>Front Low: ${data.front_low} mm ${data.front_low > 0 ? '' : '(Not Available)'}</p>
            <p>Front High: ${data.front_high} mm ${data.front_high > 0 ? '' : '(Not Available)'}</p>
            <p>Left: ${data.left} mm ${data.left > 0 ? '' : '(Not Available)'}</p>
            <p>Right: ${data.right} mm ${data.right > 0 ? '' : '(Not Available)'}</p>
            <p>Rear: ${data.rear} mm ${data.rear > 0 ? '' : '(Not Available)'}</p>
            <p>Accel X: ${parseFloat(data.accel_x).toFixed(2)}</p>
            <p>Accel Y: ${parseFloat(data.accel_y).toFixed(2)}</p>
            <p>Accel Z: ${parseFloat(data.accel_z).toFixed(2)}</p>
            <p>Gyro X: ${parseFloat(data.gyro_x).toFixed(2)}</p>
            <p>Gyro Y: ${parseFloat(data.gyro_y).toFixed(2)}</p>
            <p>Gyro Z: ${parseFloat(data.gyro_z).toFixed(2)}</p>
            <p>Mag X: ${parseFloat(data.mag_x).toFixed(2)}</p>
            <p>Mag Y: ${parseFloat(data.mag_y).toFixed(2)}</p>
            <p>Mag Z: ${parseFloat(data.mag_z).toFixed(2)}</p>
            <p>Position X: ${parseFloat(data.pos_x).toFixed(2)}</p>
            <p>Position Y: ${parseFloat(data.pos_y).toFixed(2)}</p>
            <p>Orientation: ${parseFloat(data.orientation).toFixed(2)}</p>
            <p>Left Motor Speed: ${data.left_motor_speed}</p>
            <p>Right Motor Speed: ${data.right_motor_speed}</p>
          `;
          drawMap(data.waypoints);
        });
    }

    function drawMap(waypoints) {
      let canvas = document.getElementById('mapCanvas');
      let ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      ctx.strokeStyle = '#00ff00';
      ctx.beginPath();
      for (let i = 0; i < waypoints.length; i++) {
        let x = canvas.width / 2 + waypoints[i].x / 10;
        let y = canvas.height / 2 - waypoints[i].y / 10;
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
        ctx.arc(x, y, 2, 0, 2 * Math.PI);
      }
      ctx.stroke();
    }

    switchMode('manual');
    setInterval(fetchSensorData, 500);
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  Wire.begin(GYRO_SDA, GYRO_SCL);
  mpu9250.setWire(&Wire);
  mpu9250.beginAccel();
  mpu9250.beginGyro();
  mpu9250.beginMag();

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  stopMotors();

  pinMode(LEFT_HALL_SENSOR_PIN, INPUT);
  pinMode(RIGHT_HALL_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_HALL_SENSOR_PIN), onLeftHallSensor, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALL_SENSOR_PIN), onRightHallSensor, RISING);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  if (!MDNS.begin(hostName)) {
    Serial.println("Error setting up mDNS responder!");
    while (1);
  }
  Serial.printf("You can now access BB-1 at http://%s.local\n", hostName);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", controlPage);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = getSensorData();
    request->send(200, "application/json", json);
  });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("direction") && request->hasParam("stop")) {
      String direction = request->getParam("direction")->value();
      bool stop = request->getParam("stop")->value() == "true";
      handleMoveCommand(direction, stop);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/speed", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("leftSpeed", true) && request->hasParam("rightSpeed", true) && request->hasParam("sync", true)) {
      leftMotorSpeed = request->getParam("leftSpeed", true)->value().toInt();
      rightMotorSpeed = request->getParam("rightSpeed", true)->value().toInt();
      syncMotorSpeeds = request->getParam("sync", true)->value() == "true";
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });

  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("mode")) {
      String mode = request->getParam("mode")->value();
      if (mode == "manual") {
        currentMode = MANUAL;
        autonomousTicker.detach();
        lifeModeTicker.detach();
        stopMotors();
      } else if (mode == "autonomous") {
        currentMode = AUTONOMOUS;
        autonomousTicker.attach_ms(100, autonomousNavigation);
        lifeModeTicker.detach();
      } else if (mode == "life") {
        currentMode = LIFE;
        lifeModeTicker.attach_ms(100, lifeModeBehavior);
        autonomousTicker.detach();
      }
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });

  server.begin();
}

void loop() {
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = millis();
    readSensors();
  }

  if (currentMode == MANUAL) {
    handleObstacles();
  }
  delay(1);
}

// ISR for Hall sensors
void IRAM_ATTR onLeftHallSensor() {
  leftPulseCount++;
}

void IRAM_ATTR onRightHallSensor() {
  rightPulseCount++;
}

// Function for handling move commands
void handleMoveCommand(String direction, bool stop) {
    if (stop) {
        stopMotors();
        return;
    }

    if (direction == "forward") {
        moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);
    } else if (direction == "backward") {
        moveMotors(-1, -1, leftMotorSpeed, rightMotorSpeed);
    } else if (direction == "left") {
        moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);
    } else if (direction == "right") {
        moveMotors(1, -1, leftMotorSpeed, rightMotorSpeed);
    }
}

// Function to read sensor values
void readSensors() {
    frontLow = readUltrasonicSensor(frontSensor);
    frontHigh = readUltrasonicSensor(extraFrontSensor);
    leftSide = readUltrasonicSensor(leftSensor);
    rightSide = readUltrasonicSensor(rightSensor);
    rear = readUltrasonicSensor(backSensor);
}

// Function for getting sensor data in JSON format
String getSensorData() {
    JSONVar sensorData;
    sensorData["front_low"] = frontLow;
    sensorData["front_high"] = frontHigh;
    sensorData["left"] = leftSide;
    sensorData["right"] = rightSide;
    sensorData["rear"] = rear;
    sensorData["accel_x"] = mpu9250.accelX();
    sensorData["accel_y"] = mpu9250.accelY();
    sensorData["accel_z"] = mpu9250.accelZ();
    sensorData["gyro_x"] = mpu9250.gyroX();
    sensorData["gyro_y"] = mpu9250.gyroY();
    sensorData["gyro_z"] = mpu9250.gyroZ();
    sensorData["mag_x"] = mpu9250.magX();
    sensorData["mag_y"] = mpu9250.magY();
    sensorData["mag_z"] = mpu9250.magZ();
    sensorData["pos_x"] = posX;
    sensorData["pos_y"] = posY;
    sensorData["orientation"] = orientation;
    sensorData["left_motor_speed"] = leftMotorSpeed;
    sensorData["right_motor_speed"] = rightMotorSpeed;
    String jsonString = JSON.stringify(sensorData);
    return jsonString;
}

// Function to handle obstacles
void handleObstacles() {
    if (frontLow < frontThreshold || rear < rearThreshold) {
        stopMotors();
        delay(200);
        spinLeft();
        delay(300);
        stopMotors();
    }
}

// Function to stop all motors
void stopMotors() {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// Autonomous navigation behavior
void autonomousNavigation() {
    updatePosition();
    // You can add more autonomous navigation code here
}

// Life mode behavior
void lifeModeBehavior() {
    // Custom behaviors for life mode
    if (frontLow < 1000 && frontLow > 0) {
        moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);
    } else {
        spinLeft();
    }
}
// Function to spin left in place
void spinLeft() {
    // Set left motor to reverse and right motor to forward
    moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);
}

// Function to spin right in place
void spinRight() {
    // Set left motor to forward and right motor to reverse
    moveMotors(1, -1, leftMotorSpeed, rightMotorSpeed);
}
// Function to control the motors
void moveMotors(int leftDirection, int rightDirection, int customLeftSpeed = -1, int customRightSpeed = -1) {
    int leftSpeed = (customLeftSpeed == -1) ? leftMotorSpeed : customLeftSpeed;
    int rightSpeed = (customRightSpeed == -1) ? rightMotorSpeed : customRightSpeed;

    // Control left motor
    if (leftDirection > 0) {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        analogWrite(MOTOR_LEFT_IN1, leftSpeed);
    } else if (leftDirection < 0) {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
        analogWrite(MOTOR_LEFT_IN2, leftSpeed);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
    }

    // Control right motor
    if (rightDirection > 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        analogWrite(MOTOR_RIGHT_IN1, rightSpeed);
    } else if (rightDirection < 0) {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
        analogWrite(MOTOR_RIGHT_IN2, rightSpeed);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
    }
}

// Function to read the ultrasonic sensor
unsigned int readUltrasonicSensor(NewPing &sensor) {
    unsigned int distance = sensor.ping_cm();
    return distance == 0 ? 5000 : distance * 10;  // Convert to mm for consistency
}

// Function to update position using MPU data (example with basic integration)
void updatePosition() {
    static unsigned long lastUpdate = millis();
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdate) / 1000.0;
    lastUpdate = currentTime;

    mpu9250.accelUpdate();
    mpu9250.gyroUpdate();
    float yawRate = mpu9250.gyroZ();

    orientation += yawRate * deltaTime;

    float avgSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
    float distanceTraveled = avgSpeed * deltaTime;  
    posX += distanceTraveled * cos(orientation);
    posY += distanceTraveled * sin(orientation);
}

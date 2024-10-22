// Include necessary libraries
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

// Wi-Fi network credentials
const char* ssid = "SpectrumSetup-DD";          // Replace with your actual Wi-Fi SSID
const char* password = "jeansrocket543";  // Replace with your actual Wi-Fi password

// Fixed domain name for mDNS
const char* hostName = "bbmobileunit";

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
const unsigned long frontThreshold = 300;  // Front sensors stop threshold
const unsigned long sideThreshold = 200;   // Side sensors threshold
const unsigned long rearThreshold = 200;   // Rear sensor threshold

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
#define GRID_RESOLUTION 0.5f  // 0.5 meters per cell
int occupancyGrid[GRID_SIZE][GRID_SIZE] = {0};  // 0 = unexplored, 1 = obstacle, 2 = free space

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

// Sensor reading variables and functions
void readSensors();
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 100;  // Read sensors every 100ms

// Global variables to store sensor data
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
    <!-- Sensor data will be populated here -->
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
      button.addEventListener('mousedown', () => sendMoveCommand(direction, false));
      button.addEventListener('touchstart', () => sendMoveCommand(direction, false));
      button.addEventListener('mouseup', () => sendMoveCommand(direction, true));
      button.addEventListener('touchend', () => sendMoveCommand(direction, true));
      button.addEventListener('mouseleave', () => sendMoveCommand(direction, true));
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
        let x = canvas.width / 2 + waypoints[i].x / 10; // Scale position
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

    // Initial mode setup
    switchMode('manual');

    setInterval(fetchSensorData, 500); // Fetch sensor data every 0.5 seconds
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // Initialize I2C and the MPU9250
  Wire.begin(GYRO_SDA, GYRO_SCL);
  mpu9250.setWire(&Wire);
  mpu9250.beginAccel();
  mpu9250.beginGyro();
  mpu9250.beginMag();

  // Motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Ensure motors are stopped at startup
  stopMotors();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Print IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Access the control panel at:");
  Serial.print("http://");
  Serial.println(WiFi.localIP());

  // Set up mDNS
  if (!MDNS.begin(hostName)) {
    Serial.println("Error setting up mDNS responder!");
    while (1);
  }
  Serial.println("mDNS responder started");
  Serial.printf("You can now access BB-1 at http://%s.local\n", hostName);

  // Serve the control page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", controlPage);
  });

  // Serve sensor data as JSON
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = getSensorData();
    request->send(200, "application/json", json);
  });

  // Handle move commands
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

  // Handle speed updates
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

  // Handle mode switching
  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("mode")) {
      String mode = request->getParam("mode")->value();
      if (mode == "manual") {
        currentMode = MANUAL;
        autonomousTicker.detach();
        lifeModeTicker.detach();
        stopMotors();  // Ensure motors are stopped when switching to manual
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

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // Update sensors at intervals
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = millis();
    readSensors();
  }

  // Handle obstacles in manual mode to prevent collisions
  if (currentMode == MANUAL) {
    handleObstacles();
  }

  // Yield to reset the watchdog timer
  delay(1);  // Use delay(1) to allow other tasks to run
}

// Handle move commands
void handleMoveCommand(String direction, bool stop) {
  if (stop) {
    stopMotors(); // Stop motors if commanded
    return;
  }

  // Synchronize motors if sync is enabled
  if (syncMotorSpeeds) {
    rightMotorSpeed = leftMotorSpeed;
  }

  if (direction == "forward") {
    moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);  // Move forward
  } else if (direction == "backward") {
    moveMotors(-1, -1, leftMotorSpeed, rightMotorSpeed); // Move backward
  } else if (direction == "left") {
    moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed); // Spin left
  } else if (direction == "right") {
    moveMotors(1, -1, leftMotorSpeed, rightMotorSpeed); // Spin right
  } else if (direction == "curve_left") {
    // Adjust left motor speed to curve left
    uint8_t tempLeftSpeed = leftMotorSpeed * 0.6; // Slower left motor for curve
    moveMotors(1, 1, tempLeftSpeed, rightMotorSpeed); 
  } else if (direction == "curve_right") {
    // Adjust right motor speed to curve right
    uint8_t tempRightSpeed = rightMotorSpeed * 0.6; // Slower right motor for curve
    moveMotors(1, 1, leftMotorSpeed, tempRightSpeed); 
  } 
}

// Function to control motors, with optional different speeds for each side
void moveMotors(int leftDirection, int rightDirection, int customLeftSpeed, int customRightSpeed) {
  int leftSpeed = (customLeftSpeed == -1) ? leftMotorSpeed : customLeftSpeed;
  int rightSpeed = (customRightSpeed == -1) ? rightMotorSpeed : customRightSpeed;

  // Control left motor
  if (leftDirection > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else if (leftDirection < 0) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
  } else {
    // Stop left motor
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }

  // Control right motor
  if (rightDirection > 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else if (rightDirection < 0) {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  } else {
    // Stop right motor
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }
}

// Stop motors
void stopMotors() {
  // Stop left motor
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);

  // Stop right motor
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

// Read sensors
void readSensors() {
  frontLow = readUltrasonicSensor(frontSensor);
  frontHigh = readUltrasonicSensor(extraFrontSensor);
  leftSide = readUltrasonicSensor(leftSensor);
  rightSide = readUltrasonicSensor(rightSensor);
  rear = readUltrasonicSensor(backSensor);
}

// Read ultrasonic sensor using NewPing library
unsigned int readUltrasonicSensor(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    return MAX_DISTANCE * 10;  // Return max distance in mm if no object detected
  } else {
    return distance * 10;  // Convert cm to mm
  }
}

// Autonomous navigation function
void autonomousNavigation() {
  // Update position and orientation
  updatePosition();
  updateOccupancyGrid();

  // Find next frontier and move towards it
  Waypoint nextFrontier = findNextFrontier();
  if (nextFrontier.x != -1 && nextFrontier.y != -1) {
    // Move towards the next frontier
    moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);
  } else {
    // No frontier found, random movement
    moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);  // Spin in place
  }

  // Yield to reset the watchdog timer
  delay(1);
}

// Life Mode behavior function
void lifeModeBehavior() {
  // Update position and orientation
  updatePosition();
  updateOccupancyGrid();

  // State machine for different behaviors
  static unsigned long lastStateChange = millis();
  unsigned long currentTime = millis();

  // Change state every 5 to 15 seconds
  if (currentTime - lastStateChange > random(5000, 15000)) {
    currentLifeState = (LifeState)random(0, 3);  // Randomly select a new state
    lastStateChange = currentTime;
    Serial.print("Life Mode State Changed to: ");
    if (currentLifeState == IDLE) Serial.println("IDLE");
    else if (currentLifeState == EXPLORING) Serial.println("EXPLORING");
    else if (currentLifeState == INTERACTING) Serial.println("INTERACTING");
  }

  switch (currentLifeState) {
    case IDLE:
      // Stay still or perform small movements
      stopMotors();
      break;

    case EXPLORING:
      // Use frontier-based navigation for exploration
      autonomousNavigation();
      break;

    case INTERACTING:
      // Use sensors to detect humans or objects and move towards them
      if (frontLow < 1000 && frontLow > 0) {
        // Object detected, move towards it
        moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);
      } else {
        // No object, wander
        moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);  // Spin in place
      }
      break;
  }

  // Yield to reset the watchdog timer
  delay(1);
}

// Update the robot's position and orientation using IMU data
void updatePosition() {
  static unsigned long lastUpdate = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdate) / 1000.0;  // Convert to seconds
  lastUpdate = currentTime;

  // Update IMU data
  mpu9250.accelUpdate();
  mpu9250.gyroUpdate();
  mpu9250.magUpdate();
  float yawRate = mpu9250.gyroZ();  // Z-axis gyro provides yaw rate

  // Update orientation
  orientation += yawRate * deltaTime;

  // Update position
  float avgSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
  float distanceTraveled = avgSpeed * deltaTime;  // Simplified estimation
  posX += distanceTraveled * cos(orientation);
  posY += distanceTraveled * sin(orientation);

  // Store waypoints
  if (waypointCount < MAX_WAYPOINTS) {
    waypoints[waypointCount].x = posX;
    waypoints[waypointCount].y = posY;
    waypointCount++;
  }

  // Yield to reset the watchdog timer
  delay(1);
}

// Update occupancy grid with the current position
void updateOccupancyGrid() {
  int gridX = (int)(posX / GRID_RESOLUTION) + (GRID_SIZE / 2);
  int gridY = (int)(posY / GRID_RESOLUTION) + (GRID_SIZE / 2);
  if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
    occupancyGrid[gridX][gridY] = 2;  // Mark as free space
  }

  // Yield to reset the watchdog timer
  delay(1);
}

// Find the next frontier to explore
Waypoint findNextFrontier() {
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      if (occupancyGrid[x][y] == 0) {  // Unexplored cell
        Waypoint frontier;
        frontier.x = (x - (GRID_SIZE / 2)) * GRID_RESOLUTION;
        frontier.y = (y - (GRID_SIZE / 2)) * GRID_RESOLUTION;
        return frontier;
      }
    }
  }
  // No frontier found
  Waypoint noFrontier = {-1, -1};
  return noFrontier;
}

// Handle obstacles and adjust movement
void handleObstacles() {
  unsigned long currentTime = millis();

  switch (obstacleState) {
    case IDLE_OBSTACLE:
      // Check for obstacles
      if ((frontLow > 0 && frontLow < frontThreshold) || (frontHigh > 0 && frontHigh < frontThreshold)) {
        stopMotors();
        obstacleState = MOVING_BACKWARD;
        obstacleStateStartTime = currentTime;
      } else if (leftSide > 0 && leftSide < sideThreshold) {
        // Obstacle on the left, turn right
        moveMotors(1, -1, leftMotorSpeed, rightMotorSpeed);
      } else if (rightSide > 0 && rightSide < sideThreshold) {
        // Obstacle on the right, turn left
        moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);
      } else if (rear > 0 && rear < rearThreshold) {
        // Obstacle at the rear, move forward
        moveMotors(1, 1, leftMotorSpeed, rightMotorSpeed);
      }
      break;

    case MOVING_BACKWARD:
      // Move backward for 500ms
      if (currentTime - obstacleStateStartTime < 500) {
        moveMotors(-1, -1, leftMotorSpeed, rightMotorSpeed);
      } else {
        stopMotors();
        obstacleState = TURNING;
        obstacleStateStartTime = currentTime;
      }
      break;

    case TURNING:
      // Turn for 500ms
      if (currentTime - obstacleStateStartTime < 500) {
        if (leftSide > rightSide) {
          moveMotors(-1, 1, leftMotorSpeed, rightMotorSpeed);  // Spin left
        } else {
          moveMotors(1, -1, leftMotorSpeed, rightMotorSpeed);  // Spin right
        }
      } else {
        stopMotors();
        obstacleState = RESUMING;
      }
      break;

    case RESUMING:
      // Resume normal operation
      obstacleState = IDLE_OBSTACLE;
      break;
  }

  // Yield to reset the watchdog timer
  delay(1);
}

// Get sensor data as JSON
String getSensorData() {
  JSONVar doc;

  // Use global sensor variables
  doc["front_low"] = frontLow;
  doc["front_high"] = frontHigh;
  doc["left"] = leftSide;
  doc["right"] = rightSide;
  doc["rear"] = rear;

  // IMU data
  mpu9250.accelUpdate();
  mpu9250.gyroUpdate();
  mpu9250.magUpdate();
  doc["accel_x"] = mpu9250.accelX();
  doc["accel_y"] = mpu9250.accelY();
  doc["accel_z"] = mpu9250.accelZ();
  doc["gyro_x"] = mpu9250.gyroX();
  doc["gyro_y"] = mpu9250.gyroY();
  doc["gyro_z"] = mpu9250.gyroZ();
  doc["mag_x"] = mpu9250.magX();
  doc["mag_y"] = mpu9250.magY();
  doc["mag_z"] = mpu9250.magZ();

  // Position data
  doc["pos_x"] = posX;
  doc["pos_y"] = posY;
  doc["orientation"] = orientation;

  // Waypoints
  JSONVar waypointArray;
  for (int i = 0; i < waypointCount; i++) {
    JSONVar point;
    point["x"] = waypoints[i].x;
    point["y"] = waypoints[i].y;
    waypointArray[i] = point;
  }
  doc["waypoints"] = waypointArray;

  // Current Mode
  if (currentMode == MANUAL) doc["current_mode"] = "Manual";
  else if (currentMode == AUTONOMOUS) doc["current_mode"] = "Autonomous";
  else if (currentMode == LIFE) doc["current_mode"] = "Life";

  // Motor speeds
  doc["left_motor_speed"] = leftMotorSpeed;
  doc["right_motor_speed"] = rightMotorSpeed;

  String jsonData = JSON.stringify(doc);
  return jsonData;
}

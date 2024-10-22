#include <WiFi.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <ESPAsyncWebServer.h>
#include <MPU9250_asukiaaa.h>
#include <ESPmDNS.h>
#include <Ticker.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h> // Required for ESPAsyncWebServer

// Motor control pins (DRV8871)
#define MOTOR_LEFT_IN1 25
#define MOTOR_LEFT_IN2 26
#define MOTOR_RIGHT_IN1 27
#define MOTOR_RIGHT_IN2 14

// Emergency Stop Pin (optional physical button)
#define EMERGENCY_STOP_PIN 12 // You can change this to an available GPIO pin

// Ultrasonic sensor setup
TCA9548A tca;
#define ULTRASONIC_ADDR 0x57 // Ultrasonic I2C address

// MPU9250 setup
MPU9250_asukiaaa mpu9250;

// Wi-Fi network credentials
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// Fixed domain name for mDNS
const char* hostName = "bbmobileunit";

// HTTP server
AsyncWebServer server(80);

// Mode flags
enum RobotMode { MANUAL, AUTONOMOUS, LIFE };
RobotMode currentMode = MANUAL;

// Speed control variables (0-255)
uint8_t leftMotorSpeed = 200;
uint8_t rightMotorSpeed = 200;

// Thresholds for sensors (in mm)
const unsigned long frontThreshold = 300;   // Front sensors stop threshold
const unsigned long sideThreshold = 200;    // Side sensors threshold
const unsigned long rearThreshold = 200;    // Rear sensor threshold

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

// Behavioral states for Life Mode
enum LifeState { IDLE, EXPLORING, INTERACTING };
LifeState currentLifeState = IDLE;

// Obstacle avoidance states
enum ObstacleState { IDLE_OBSTACLE, MOVING_BACKWARD, TURNING, RESUMING };
ObstacleState obstacleState = IDLE_OBSTACLE;
unsigned long obstacleStateStartTime = 0;

// Emergency Stop flag
volatile bool emergencyStopActivated = false;

// Function prototypes
void setLeftMotorSpeed(uint8_t speed);
void setRightMotorSpeed(uint8_t speed);
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void curveLeft();
void curveRight();
void stopMotors();
void handleMoveCommand(String direction);
unsigned long readUltrasonicSensor(uint8_t channel);
String getSensorData();
void updatePosition();
void autonomousNavigation();
void lifeModeBehavior();
void handleObstacles();
void emergencyStop();
void resetEmergencyStop();
void IRAM_ATTR handleEmergencyStopISR();

void setup() {
  Serial.begin(115200);

  // Motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Initialize I2C and the TCA9548A multiplexer
  Wire.begin();
  if (!tca.begin()) {
    Serial.println("TCA9548A not found!");
    while (1);
  }
  Serial.println("TCA9548A detected!");

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

  // Set up mDNS
  if (!MDNS.begin(hostName)) {
    Serial.println("Error setting up mDNS responder!");
    while (1);
  }
  Serial.println("mDNS responder started");
  Serial.printf("You can now access BB-1 at http://%s.local\n", hostName);

  // Initialize MPU9250
  mpu9250.setWire(&Wire);
  mpu9250.beginAccel();
  mpu9250.beginGyro();
  mpu9250.beginMag();

  // Set up emergency stop pin (if using a physical button)
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), handleEmergencyStopISR, FALLING);

  // Serve the control page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", controlPage);
  });

  // Handle move commands
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (currentMode == MANUAL && !emergencyStopActivated) {
      String response = "Move command received";
      if (request->hasParam("direction")) {
        String direction = request->getParam("direction")->value();
        handleMoveCommand(direction);
        response += ": " + direction;
      }
      if (request->hasParam("leftSpeed")) {
        leftMotorSpeed = request->getParam("leftSpeed")->value().toInt();
      }
      if (request->hasParam("rightSpeed")) {
        rightMotorSpeed = request->getParam("rightSpeed")->value().toInt();
      }
      request->send(200, "text/plain", response);
    } else {
      request->send(403, "text/plain", "Manual control disabled or emergency stop activated");
    }
  });

  // Handle speed control
  server.on("/speed", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("leftSpeed", true)) {
      leftMotorSpeed = request->getParam("leftSpeed", true)->value().toInt();
    }
    if (request->hasParam("rightSpeed", true)) {
      rightMotorSpeed = request->getParam("rightSpeed", true)->value().toInt();
    }
    request->send(200, "text/plain", "Speed updated");
  });

  // Handle mode switching
  server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("mode") && !emergencyStopActivated) {
      String mode = request->getParam("mode")->value();
      if (mode == "manual") {
        currentMode = MANUAL;
        autonomousTicker.detach();
        lifeModeTicker.detach();
        stopMotors();
        Serial.println("Switched to Manual Mode");
      } else if (mode == "autonomous") {
        currentMode = AUTONOMOUS;
        lifeModeTicker.detach();
        autonomousTicker.attach_ms(100, autonomousNavigation);
        Serial.println("Switched to Autonomous Mode");
      } else if (mode == "life") {
        currentMode = LIFE;
        autonomousTicker.detach();
        lifeModeTicker.attach_ms(100, lifeModeBehavior);
        Serial.println("Switched to Life Mode");
      }
    }
    request->send(200, "text/plain", "Mode updated");
  });

  // Real-time data endpoint
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String sensorData = getSensorData();
    request->send(200, "application/json", sensorData);
  });

  // Emergency stop endpoint
  server.on("/emergency_stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    emergencyStop();
    request->send(200, "text/plain", "Emergency Stop Activated");
  });

  // Reset emergency stop endpoint
  server.on("/reset_emergency_stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    resetEmergencyStop();
    request->send(200, "text/plain", "Emergency Stop Reset");
  });

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // In non-blocking code, the loop remains empty or handles tasks that need constant checking
  if (emergencyStopActivated) {
    stopMotors();
    return; // Skip the rest of the loop if emergency stop is activated
  }

  // Handle obstacles in manual mode to prevent collisions
  if (currentMode == MANUAL) {
    handleObstacles();
  }
}

// Handle move commands
void handleMoveCommand(String direction) {
  if (direction == "forward") {
    moveForward();
  } else if (direction == "backward") {
    moveBackward();
  } else if (direction == "left") {
    spinLeft();
  } else if (direction == "right") {
    spinRight();
  } else if (direction == "curve_left") {
    curveLeft();
  } else if (direction == "curve_right") {
    curveRight();
  } else if (direction == "stop") {
    stopMotors();
  }
}

void moveForward() {
  analogWrite(MOTOR_LEFT_IN1, leftMotorSpeed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_IN1, rightMotorSpeed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, leftMotorSpeed);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, rightMotorSpeed);
}

void spinLeft() {
  // Left motor backward, right motor forward
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, leftMotorSpeed);
  analogWrite(MOTOR_RIGHT_IN1, rightMotorSpeed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void spinRight() {
  // Left motor forward, right motor backward
  analogWrite(MOTOR_LEFT_IN1, leftMotorSpeed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, rightMotorSpeed);
}

void curveLeft() {
  // Reduce speed of left motor
  analogWrite(MOTOR_LEFT_IN1, leftMotorSpeed / 2);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_IN1, rightMotorSpeed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void curveRight() {
  // Reduce speed of right motor
  analogWrite(MOTOR_LEFT_IN1, leftMotorSpeed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_IN1, rightMotorSpeed / 2);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

// Autonomous navigation function
void autonomousNavigation() {
  if (emergencyStopActivated) {
    stopMotors();
    return;
  }

  // Update position and orientation
  updatePosition();

  // Handle obstacles
  handleObstacles();

  // Simple exploration logic
  moveForward();
}

// Life Mode behavior function
void lifeModeBehavior() {
  if (emergencyStopActivated) {
    stopMotors();
    return;
  }

  // Update position and orientation
  updatePosition();

  // Handle obstacles
  handleObstacles();

  // State machine for different behaviors
  static unsigned long lastStateChange = millis();
  unsigned long currentTime = millis();

  // Change state every 5 to 15 seconds
  if (currentTime - lastStateChange > random(5000, 15000)) {
    currentLifeState = (LifeState)random(0, 3); // Randomly select a new state
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
      // Randomly move around
      if (random(0, 100) < 50) {
        moveForward();
      } else if (random(0, 100) < 25) {
        spinLeft();
      } else {
        spinRight();
      }
      break;

    case INTERACTING:
      // Use sensors to detect humans or objects and move towards them
      unsigned long frontDistance = readUltrasonicSensor(0); // Front low sensor
      if (frontDistance < 1000 && frontDistance > 0) {
        // Object detected, move towards it
        moveForward();
      } else {
        // No object, wander
        spinLeft();
      }
      break;
  }
}

// Update the robot's position and orientation using IMU data
void updatePosition() {
  static unsigned long lastUpdate = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdate) / 1000.0; // Convert to seconds
  lastUpdate = currentTime;

  // Update IMU data
  mpu9250.update();
  float yawRate = mpu9250.gyroZ(); // Z-axis gyro provides yaw rate

  // Update orientation
  orientation += yawRate * deltaTime;

  // Update position
  float avgSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
  float distanceTraveled = avgSpeed * deltaTime; // Simplified estimation
  posX += distanceTraveled * cos(orientation);
  posY += distanceTraveled * sin(orientation);

  // Store waypoints
  if (waypointCount < MAX_WAYPOINTS) {
    waypoints[waypointCount].x = posX;
    waypoints[waypointCount].y = posY;
    waypointCount++;
  }
}

// Handle obstacles and adjust movement
void handleObstacles() {
  static unsigned long lastSensorReadTime = 0;
  const unsigned long sensorReadInterval = 50; // Read sensors every 50ms

  unsigned long currentTime = millis();

  // Check for emergency stop
  if (emergencyStopActivated) {
    stopMotors();
    return;
  }

  // Read sensors at intervals
  if (currentTime - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = currentTime;

    // Read all sensors
    frontLow = readUltrasonicSensor(0);
    frontHigh = readUltrasonicSensor(1);
    leftSide = readUltrasonicSensor(2);
    rightSide = readUltrasonicSensor(3);
    rear = readUltrasonicSensor(4);
  }

  switch (obstacleState) {
    case IDLE_OBSTACLE:
      // Check for obstacles
      if ((frontLow > 0 && frontLow < frontThreshold) || (frontHigh > 0 && frontHigh < frontThreshold)) {
        stopMotors();
        obstacleState = MOVING_BACKWARD;
        obstacleStateStartTime = currentTime;
      } else if (leftSide > 0 && leftSide < sideThreshold) {
        // Obstacle on the left, curve right
        curveRight();
      } else if (rightSide > 0 && rightSide < sideThreshold) {
        // Obstacle on the right, curve left
        curveLeft();
      } else if (rear > 0 && rear < rearThreshold) {
        // Obstacle at the rear, move forward
        moveForward();
      }
      break;

    case MOVING_BACKWARD:
      // Move backward for 500ms
      if (currentTime - obstacleStateStartTime < 500) {
        moveBackward();
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
          spinLeft();
        } else {
          spinRight();
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
}

// Read ultrasonic sensor from a specific TCA9548A channel
unsigned long readUltrasonicSensor(uint8_t channel) {
  tca.select(channel);

  Wire.beginTransmission(ULTRASONIC_ADDR);
  Wire.write(0x01); // Command to start measurement
  Wire.endTransmission();
  delay(2); // Short delay to allow sensor to process

  Wire.requestFrom(ULTRASONIC_ADDR, 3);
  if (Wire.available() == 3) {
    uint8_t highByte = Wire.read();
    uint8_t middleByte = Wire.read();
    uint8_t lowByte = Wire.read();
    unsigned long distance = ((unsigned long)highByte << 16) | ((unsigned long)middleByte << 8) | lowByte;
    return distance / 1000; // Convert to mm
  }
  return 0;
}

// Get sensor data as JSON
String getSensorData() {
  JSONVar doc;

  // Ultrasonic data
  doc["front_low"] = readUltrasonicSensor(0);
  doc["front_high"] = readUltrasonicSensor(1);
  doc["left"] = readUltrasonicSensor(2);
  doc["right"] = readUltrasonicSensor(3);
  doc["rear"] = readUltrasonicSensor(4);

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

// Emergency stop function
void emergencyStop() {
  emergencyStopActivated = true;
  stopMotors();
  // Reset any ongoing actions or states
  obstacleState = IDLE_OBSTACLE;
  currentLifeState = IDLE;
  autonomousTicker.detach();
  lifeModeTicker.detach();
  Serial.println("Emergency Stop Activated");
}

// Reset emergency stop
void resetEmergencyStop() {
  emergencyStopActivated = false;
  // You may want to reset other states or variables here
  Serial.println("Emergency Stop Reset");
}

// ISR for physical emergency stop button
void IRAM_ATTR handleEmergencyStopISR() {
  emergencyStop();
}

// Variables for obstacle handling
unsigned long frontLow = 0, frontHigh = 0, leftSide = 0, rightSide = 0, rear = 0;

// HTML/JS for control page
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
    .emergency-stop { background-color: red; color: white; }
  </style>
</head>
<body>
  <h1>BB-1 Control Panel</h1>

  <div class="mode-buttons">
    <button class="button" onclick="switchMode('manual')">Manual Mode</button>
    <button class="button" onclick="switchMode('autonomous')">Autonomous Mode</button>
    <button class="button" onclick="switchMode('life')">Life Mode</button>
    <button class="button emergency-stop" onclick="emergencyStop()">Emergency Stop</button>
  </div>

  <div id="manualControls" style="display: none;">
    <div class="control-grid">
      <button class="button" onclick="move('forward')">Forward</button>
      <button class="button" onclick="move('left')">Left</button>
      <button class="button" onclick="move('right')">Right</button>
      <button class="button" onclick="move('curve_left')">Curve Left</button>
      <button class="button" onclick="move('curve_right')">Curve Right</button>
      <button class="button" onclick="move('backward')">Backward</button>
      <button class="button" onclick="move('stop')">Stop</button>
    </div>

    <div class="slider-label">
      <label for="leftSpeed">Left Motor Speed:</label>
      <input type="range" id="leftSpeed" min="0" max="255" value="200" onchange="updateSpeed()">
    </div>
    <div class="slider-label">
      <label for="rightSpeed">Right Motor Speed:</label>
      <input type="range" id="rightSpeed" min="0" max="255" value="200" onchange="updateSpeed()">
    </div>
  </div>

  <div class="status">
    <p>Current Mode: <span id="modeText">Manual</span></p>
    <p>Status: <span id="statusText">Idle</span></p>
    <p id="emergencyStatus"></p>
  </div>

  <h2>Sensor Data</h2>
  <div id="sensorData">
    <!-- Sensor data will be populated here -->
  </div>

  <h2>Map</h2>
  <canvas id="mapCanvas" width="500" height="500"></canvas>

  <script>
    let currentMode = 'Manual';

    function move(direction) {
      fetch(`/move?direction=${direction}`)
        .then(response => response.text())
        .then(data => {
          document.getElementById('statusText').innerText = direction.charAt(0).toUpperCase() + direction.slice(1);
        });
    }

    function updateSpeed() {
      let leftSpeed = document.getElementById('leftSpeed').value;
      let rightSpeed = document.getElementById('rightSpeed').value;
      fetch('/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `leftSpeed=${leftSpeed}&rightSpeed=${rightSpeed}`
      });
    }

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

    function fetchSensorData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          let sensorDataDiv = document.getElementById('sensorData');
          sensorDataDiv.innerHTML = `
            <p>Front Low: ${data.front_low} mm</p>
            <p>Front High: ${data.front_high} mm</p>
            <p>Left: ${data.left} mm</p>
            <p>Right: ${data.right} mm</p>
            <p>Rear: ${data.rear} mm</p>
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

          // Update emergency stop status
          if (data.emergency_stop) {
            document.getElementById('emergencyStatus').innerText = "Emergency Stop Activated";
          } else {
            document.getElementById('emergencyStatus').innerText = "";
          }
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

    function emergencyStop() {
      fetch('/emergency_stop')
        .then(response => response.text())
        .then(data => {
          document.getElementById('statusText').innerText = "Emergency Stop Activated";
          document.getElementById('emergencyStatus').innerText = "Emergency Stop Activated";
        });
    }

    // Initial mode setup
    switchMode('manual');

    setInterval(fetchSensorData, 500); // Fetch sensor data every 0.5 seconds
  </script>
</body>
</html>
)rawliteral";

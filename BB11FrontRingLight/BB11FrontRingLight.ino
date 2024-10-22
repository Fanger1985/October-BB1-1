#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

// Network credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Pin for NeoPixel ring
#define LED_PIN 5 // Adjust to the actual pin
#define NUM_LEDS 12 // Number of LEDs in the ring

// Create NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Web server on port 80
AsyncWebServer server(80);

// To store LED state and effect states
bool ledOn = true;
bool rollingColors = false;
bool smoothRolling = false;
bool smootherRolling = false;
int currentColorIndex = 0;
int rollSpeed = 100;        // Milliseconds delay for rolling effect
int smoothRollSpeed = 50;   // Speed for smooth rolling effect
int smootherRollSpeed = 150; // Slower speed for the "Smoother Roll" effect

// Preset colors (RGB values)
const int colors[][3] = {
  {255, 255, 255}, // White
  {255, 0, 0},     // Red
  {0, 255, 0},     // Green
  {0, 0, 255},     // Blue
  {255, 255, 0},   // Yellow
  {0, 255, 255},   // Cyan
  {255, 0, 255}    // Magenta
};

int numColors = sizeof(colors) / sizeof(colors[0]);

// Function to set all pixels to a certain color
void setAllLEDs(int r, int g, int b) {
  if (ledOn) {  // Only set LEDs if the LED is turned on
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
  } else {
    strip.clear();  // If LED is off, turn off all pixels
    strip.show();
  }
}

// Set the current color based on the index
void setCurrentColor() {
  setAllLEDs(colors[currentColorIndex][0], colors[currentColorIndex][1], colors[currentColorIndex][2]);
}

// Function to smoothly roll through the colors
void rollThroughColors() {
  for (int j = 0; j < NUM_LEDS; j++) {
    int colorIndex = (currentColorIndex + j) % numColors;
    strip.setPixelColor(j, strip.Color(colors[colorIndex][0], colors[colorIndex][1], colors[colorIndex][2]));
  }
  strip.show();
  currentColorIndex = (currentColorIndex + 1) % numColors;  // Advance to the next color
}

// Function for the Smooth Roll (singular LED rolling with smooth transitions)
void smoothRoll() {
  static int currentPos = 0;
  strip.clear();
  
  // Set the single moving LED with smooth color transition
  int colorIndex = currentPos % numColors;
  strip.setPixelColor(currentPos % NUM_LEDS, strip.Color(colors[colorIndex][0], colors[colorIndex][1], colors[colorIndex][2]));
  
  strip.show();
  currentPos = (currentPos + 1) % NUM_LEDS; // Move to the next LED position
}

// Function for the Smoother Roll with fading effect
void smootherRoll() {
  static int currentPos = 0;
  strip.clear();
  
  // Set the single moving LED with fading effect
  for (int i = 0; i < NUM_LEDS; i++) {
    int brightness = 255 / (abs(currentPos - i) + 1);  // Fade based on distance from currentPos
    int colorIndex = currentPos % numColors;
    strip.setPixelColor(i, strip.Color(
      colors[colorIndex][0] * brightness / 255, 
      colors[colorIndex][1] * brightness / 255, 
      colors[colorIndex][2] * brightness / 255
    ));
  }
  strip.show();
  currentPos = (currentPos + 1) % NUM_LEDS; // Move to the next LED position
}

// Web control page
String processor(const String& var){
  if(var == "STATE"){
    if(ledOn){
      return "ON";
    } else {
      return "OFF";
    }
  }
  return String();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Set initial color to white
  setCurrentColor();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start mDNS service
  if (!MDNS.begin("neopixel")) {
    Serial.println("Error setting up mDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started: neopixel.local");

  // Print local IP address
  Serial.println(WiFi.localIP());

  // Set up web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", 
    "<!DOCTYPE html>"
    "<html lang='en'>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>LED Control</title>"
    "<style>"
    "body {"
    "  font-family: 'Arial', sans-serif;"
    "  background: linear-gradient(135deg, #ececec, #f0f0f0);"
    "  display: flex;"
    "  justify-content: center;"
    "  align-items: center;"
    "  height: 100vh;"
    "  margin: 0;"
    "}"
    ".container {"
    "  background: rgba(255, 255, 255, 0.2);"
    "  border-radius: 20px;"
    "  box-shadow: 10px 10px 30px rgba(0, 0, 0, 0.1);"
    "  backdrop-filter: blur(10px);"
    "  padding: 20px;"
    "  text-align: center;"
    "  width: 300px;"
    "  border: 1px solid rgba(255, 255, 255, 0.4);"
    "}"
    "h1 {"
    "  color: #333;"
    "  margin-bottom: 20px;"
    "}"
    ".button {"
    "  background-color: #6C63FF;"
    "  border: none;"
    "  border-radius: 12px;"
    "  box-shadow: 5px 5px 15px rgba(0, 0, 0, 0.2), -5px -5px 15px rgba(255, 255, 255, 0.7);"
    "  color: white;"
    "  font-size: 16px;"
    "  padding: 10px 20px;"
    "  cursor: pointer;"
    "  margin-bottom: 20px;"
    "}"
    ".button:hover {"
    "  background-color: #4B47CF;"
    "}"
    "#state {"
    "  font-size: 18px;"
    "  font-weight: bold;"
    "  color: #444;"
    "}"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    "  <h1>LED Control</h1>"
    "  <p>LED is currently <span id='state'></span></p>"
    "  <button class='button' onclick='toggleLED()'>Toggle LED</button>"
    "  <button class='button' onclick='cycleColor()'>Cycle Colors</button><br>"
    "  <button class='button' onclick='rollColors()'>Roll Colors</button><br>"
    "  <button class='button' onclick='smoothRoll()'>Smooth Roll</button><br>"
    "  <button class='button' onclick='smootherRoll()'>Smoother Roll</button><br>"
    "</div>"
    "<script>"
    "function toggleLED(){ fetch('/toggle'); }"
    "function cycleColor(){ fetch('/cycle'); }"
    "function rollColors(){ fetch('/roll'); }"
    "function smoothRoll(){ fetch('/smooth'); }"
    "function smootherRoll(){ fetch('/smoother'); }"
    "setInterval(()=>{"
    "  fetch('/state').then(response => response.text()).then(data => {"
    "    document.getElementById('state').innerHTML = data;"
    "  });"
    "}, 1000);"
    "</script>"
    "</body>"
    "</html>"
    );
  });

  // Toggle LED on/off
  server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *request){
    ledOn = !ledOn;
    rollingColors = false; // Stop other effects when toggling LEDs
    smoothRolling = false; // Stop smooth roll when toggling
    smootherRolling = false; // Stop smoother roll when toggling
    setCurrentColor(); // Reset to current color or turn off LEDs
    request->send(200, "text/plain", ledOn ? "ON" : "OFF");
  });

  // Cycle through the preset colors
  server.on("/cycle", HTTP_GET, [](AsyncWebServerRequest *request){
    if (ledOn) {
      rollingColors = false;  // Stop rolling if cycling colors
      smoothRolling = false;  // Stop smooth roll if cycling colors
      smootherRolling = false; // Stop smoother roll if cycling
      currentColorIndex = (currentColorIndex + 1) % numColors;
      setCurrentColor();  // Set the next color
    }
    request->send(200, "text/plain", "Cycling Colors");
  });

  // Start rolling through the colors
  server.on("/roll", HTTP_GET, [](AsyncWebServerRequest *request){
    rollingColors = true;  // Start the rolling effect
    smoothRolling = false; // Stop smooth roll if rolling colors
    smootherRolling = false; // Stop smoother roll if rolling
    request->send(200, "text/plain", "Rolling Colors");
  });

  // Start the smooth roll (singular LED moving around)
  server.on("/smooth", HTTP_GET, [](AsyncWebServerRequest *request){
    smoothRolling = true;  // Start the smooth roll effect
    rollingColors = false; // Stop normal rolling when smooth roll starts
    smootherRolling = false; // Stop smoother roll if smooth roll is active
    request->send(200, "text/plain", "Smooth Rolling LED");
  });

  // Start the smoother roll with fading effect
  server.on("/smoother", HTTP_GET, [](AsyncWebServerRequest *request){
    smootherRolling = true;  // Start the smoother roll effect
    rollingColors = false;   // Stop normal rolling when smoother roll starts
    smoothRolling = false;   // Stop smooth roll if smoother roll is active
    request->send(200, "text/plain", "Smoother Rolling LED with Fade");
  });

  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", ledOn ? "ON" : "OFF");
  });

  // Start server
  server.begin();
  
  // Add mDNS service for HTTP
  MDNS.addService("http", "tcp", 80);
}

void loop() {
  // Handle the rolling color effect
  if (rollingColors && ledOn) {
    rollThroughColors();
    delay(rollSpeed);  // Control the speed of the rolling effect
  }
  
  // Handle the smooth roll effect
  if (smoothRolling && ledOn) {
    smoothRoll();
    delay(smoothRollSpeed);  // Control the speed of the smooth roll effect
  }

  // Handle the smoother roll effect
  if (smootherRolling && ledOn) {
    smootherRoll();
    delay(smootherRollSpeed);  // Control the speed of the smoother roll with fade
  }
}

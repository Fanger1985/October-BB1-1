#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <ESPmDNS.h>  // mDNS for gps.local

// WiFi credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// GPS setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // GPS connected to RX1/TX1 on the ESP32

// Web server setup
WebServer server(80);

// Data placeholders
String latitude = "No fix yet";
String longitude = "No fix yet";
String satellites = "No data";
String altitude = "No data";
String speed = "No data";
String course = "No data";
String date = "No data";
String gpsTime = "No data"; // Renamed from 'time' to 'gpsTime'

// Setup function
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX = GPIO 16, TX = GPIO 17

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected!");

  // Start mDNS
  if (!MDNS.begin("gps")) {  // Set hostname to gps.local
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println("mDNS started: gps.local");

  // Start the web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Server started");
}

// Main loop
void loop() {
  // Feed GPS data and update info
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      latitude = String(gps.location.lat(), 6);
      longitude = String(gps.location.lng(), 6);
      satellites = String(gps.satellites.value());
      altitude = String(gps.altitude.meters());
      speed = String(gps.speed.kmph());
      course = String(gps.course.deg());
      
      // Format date and time
      date = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
      gpsTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    }
  }
  
  // Handle web server requests
  server.handleClient();
}

// Serve the GPS status page with animation
void handleRoot() {
  String html = "<html><head><title>BB-1 GPS Status</title>";
  html += "<style>body {font-family: Arial; text-align: center; padding: 50px;} h1 {color: #333;} ";
  html += "table {margin: 20px auto; border-collapse: collapse;} th, td {border: 1px solid #666; padding: 10px;} th {background-color: #333; color: white;}";
  html += "@keyframes pulse {0% {opacity: 1;} 50% {opacity: 0.5;} 100% {opacity: 1;}} ";
  html += ".loading {animation: pulse 2s infinite; font-size: 1.5em; color: #999;}</style></head>";
  html += "<body><h1>BB-1 GPS Status</h1>";
  html += "<div class='loading'>Trying to get GPS fix...</div>";  // Animated loading text
  html += "<table><tr><th>Status</th><td>Active</td></tr>";
  html += "<tr><th>Latitude</th><td>" + latitude + "</td></tr>";
  html += "<tr><th>Longitude</th><td>" + longitude + "</td></tr>";
  html += "<tr><th>Satellites</th><td>" + satellites + "</td></tr>";
  html += "<tr><th>Altitude (m)</th><td>" + altitude + "</td></tr>";
  html += "<tr><th>Speed (km/h)</th><td>" + speed + "</td></tr>";
  html += "<tr><th>Course (deg)</th><td>" + course + "</td></tr>";
  html += "<tr><th>Date</th><td>" + date + "</td></tr>";
  html += "<tr><th>Time</th><td>" + gpsTime + "</td></tr>";
  html += "</table></body></html>";
  
  server.send(200, "text/html", html);
}

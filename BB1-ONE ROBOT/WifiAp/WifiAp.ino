#include <WiFi.h>

// Define the SSID and password for the Wi-Fi network
const char* ssid = "BB2";
const char* password = "totallysecure";

void setup() {
  // Start the serial communication for debugging
  Serial.begin(115200);

  // Set the ESP32 as an access point
  WiFi.softAP(ssid, password);

  // Get the IP address of the access point
  IPAddress IP = WiFi.softAPIP();
  
  // Print the IP address to the serial monitor
  Serial.print("Access Point IP address: ");
  Serial.println(IP);
}

void loop() {
  // Nothing needed here for basic AP functionality
}

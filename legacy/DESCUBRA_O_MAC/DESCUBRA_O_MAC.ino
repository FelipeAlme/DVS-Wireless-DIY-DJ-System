#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("INICIO DO SETUP");

  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  Serial.println("LOOP OK");
  delay(1000);
}
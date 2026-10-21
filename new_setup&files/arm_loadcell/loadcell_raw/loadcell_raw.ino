#include <Adafruit_HX711.h>

// XIAO ESP32S3: D7 = GPIO44, D8 = GPIO7.
Adafruit_HX711 hx711(D7, D8);
bool recording = false;
unsigned long startedAt = 0;
unsigned long lastReady = 0;
String command;

void handleCommand() {
  command.trim();
  command.toLowerCase();
  if (command == "start") {
    startedAt = millis();
    lastReady = startedAt;
    recording = true;
    Serial.println("# STARTED");
  } else if (command == "stop") {
    recording = false;
    Serial.println("# STOPPED");
  } else if (command == "status") {
    Serial.println(recording ? "# RECORDING" : "# READY");
  } else {
    Serial.println("# ERROR unknown command");
  }
  command = "";
}

void setup() {
  Serial.begin(115200);
  command.reserve(32);
  hx711.begin();
  Serial.println("# READY");
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n') handleCommand();
    else if (c != '\r') {
      if (command.length() < 32) command += c;
      else command = "";
    }
  }
  if (!hx711.isBusy()) {
    // Keep draining conversions even while idle. No tare or calibration.
    const int32_t raw = hx711.readChannelRaw(CHAN_A_GAIN_128);
    const unsigned long now = millis();
    lastReady = now;
    if (recording) {
      Serial.print(now - startedAt);
      Serial.print(',');
      Serial.println(raw);
    }
  } else if (recording && millis() - lastReady >= 2000) {
    Serial.println("# ERROR HX711 not ready; check wiring and power");
    lastReady = millis();
  }
  delay(1);
}

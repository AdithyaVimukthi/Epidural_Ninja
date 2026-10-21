// Load-cell streaming firmware v2 for the OpenArm needle (XIAO ESP32S3 + Adafruit HX711).
// Plan: OpenArm_v4_GUI&ori/docs/09_loadcell_force_feedback_plan.md, Phase 1.
// Host driver: OpenArm_v4_GUI&ori/src/comm/loadcell.py (also still speaks v1,
// loadcell_raw.ino, which recorder.py needs -- keep that sketch for recorder.py).
//
// Wiring as loadcell_raw.ino: HX711 DATA -> D7, SCK -> D8, VIN -> 3V3, GND -> GND.
// Board: XIAO_ESP32S3, "USB CDC On Boot" enabled. Library: Adafruit HX711.
//
// Protocol, 115200 baud, newline-terminated commands:
//   hello        -> "# LOADCELL v2 hx711 gain128"
//   stream on    -> "# STREAMING", then one line per conversion: <seq>,<board_us>,<raw>
//   stream off   -> "# STOPPED"
//   status       -> "# STREAMING" or "# READY"
// seq counts every conversion since boot (also while not streaming), so the host
// sees each lost sample as a jump. board_us is micros() (wraps after ~71 min).
// "# ERR not_ready" if the HX711 gives no conversion for 200 ms.
// No tare or scaling here: the host keeps raw counts so logs can be re-analysed.

#include <Adafruit_HX711.h>

Adafruit_HX711 hx711(D7, D8);   // XIAO ESP32S3: D7 = GPIO44, D8 = GPIO7
bool streaming = false;
uint32_t seq = 0;
unsigned long lastReadyMs = 0;
String command;

void handleCommand() {
  command.trim();
  command.toLowerCase();
  if (command == "hello") {
    Serial.println("# LOADCELL v2 hx711 gain128");
  } else if (command == "stream on") {
    streaming = true;
    Serial.println("# STREAMING");
  } else if (command == "stream off") {
    streaming = false;
    Serial.println("# STOPPED");
  } else if (command == "status") {
    Serial.println(streaming ? "# STREAMING" : "# READY");
  } else if (command.length() > 0) {
    Serial.println("# ERROR unknown command");
  }
  command = "";
}

void setup() {
  Serial.begin(115200);
  command.reserve(32);
  hx711.begin();
  lastReadyMs = millis();
  Serial.println("# LOADCELL v2 hx711 gain128");
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
    const int32_t raw = hx711.readChannelRaw(CHAN_A_GAIN_128);
    const unsigned long us = micros();
    seq++;
    lastReadyMs = millis();
    if (streaming) {
      // One write per line keeps the line whole in the USB buffer.
      char line[40];
      snprintf(line, sizeof(line), "%lu,%lu,%ld", (unsigned long)seq, us, (long)raw);
      Serial.println(line);
    }
  } else if (millis() - lastReadyMs >= 200) {
    Serial.println("# ERR not_ready");
    lastReadyMs = millis();
  }
  delay(1);
}

#include <Wire.h>
#include <Adafruit_HX711.h>
#include <VL53L1X.h>  // VL53L1X by Pololu

// UNO R4 Minima or WiFi. Serial Monitor: 115200 baud.
// HX711 DATA=D6, SCK=D5 (updated wiring).
// PCA9548A SDA/SCL=UNO SDA/SCL; CH0=VL53L1X; CH1=MPU6500.
// TMC2209 STEP=D10, DIR=D11.
// Serial motor commands: F=forward, R=reverse, S=stop.
// Libraries: Adafruit HX711 and VL53L1X by Pololu.
const uint8_t STEP_PIN = 10, DIR_PIN = 11;
const uint16_t MOTOR_STEP_RATE = 3200;       // STEP pulses per second
const uint16_t MOTOR_TIMEOUT_MS = 350;      // Stop if keyboard updates are lost
const uint8_t MUX_ADDRESS = 0x70; // A0/A1/A2 LOW; change if strapped otherwise.
const uint8_t LASER_CHANNEL = 0, IMU_CHANNEL = 1;
Adafruit_HX711 loadcell(6, 5); // constructor order: DATA, CLOCK
VL53L1X laser;
uint8_t imuAddress = 0x68;
bool muxOK = false, laserOK = false, imuOK = false;
int32_t loadRaw = 0;
bool loadFresh = false;
uint32_t lastPrint = 0;
uint32_t lastMotorCommand = 0;
int8_t motorDirection = 0; // -1=reverse, 0=stopped, 1=forward

void stopMotor() {
  noTone(STEP_PIN);
  digitalWrite(STEP_PIN, LOW);
  motorDirection = 0;
}

void setMotorDirection(int8_t direction) {
  lastMotorCommand = millis();
  if (direction == motorDirection) return;

  stopMotor();
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(5); // DIR setup time before the first STEP edge
  tone(STEP_PIN, MOTOR_STEP_RATE);
  motorDirection = direction;
}

void serviceMotorCommands() {
  while (Serial.available()) {
    char command = char(Serial.read());
    if (command == 'F' || command == 'f') setMotorDirection(1);
    else if (command == 'R' || command == 'r') setMotorDirection(-1);
    else if (command == 'S' || command == 's' || command == ' ') stopMotor();
  }

  // The computer repeats F/R while a key is held. Stop on disconnect/key-up.
  if (motorDirection != 0 &&
      millis() - lastMotorCommand > MOTOR_TIMEOUT_MS) {
    stopMotor();
  }
}

bool selectChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(uint8_t(1U << channel)); // Only one channel enabled at a time.
  return Wire.endTransmission() == 0;
}

bool writeIMU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readIMU(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, count) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

int16_t signedWord(const uint8_t *p) {
  return int16_t((uint16_t(p[0]) << 8) | p[1]);
}

bool initIMU() {
  if (!selectChannel(IMU_CHANNEL)) return false;
  bool found = false;
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    imuAddress = address;
    uint8_t id = 0;
    if (readIMU(0x75, &id, 1) && id == 0x70) {
      found = true;
      break;
    }
  }
  if (!found) return false;
  if (!writeIMU(0x6B, 0x80)) return false; // Reset
  delay(100);
  if (!writeIMU(0x6B, 0x01)) return false; // Wake, auto clock
  delay(10);
  // All axes enabled, ~41 Hz filters, 100 Hz sample rate,
  // +/-250 degrees/s gyro, +/-2 g accelerometer.
  return writeIMU(0x6C, 0x00) && writeIMU(0x1A, 0x03) &&
         writeIMU(0x19, 9) && writeIMU(0x1B, 0x00) &&
         writeIMU(0x1C, 0x00) && writeIMU(0x1D, 0x03);
}

void setup() {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  loadcell.begin();
  Wire.begin();
  Wire.setClock(100000);
  muxOK = selectChannel(LASER_CHANNEL);
  Serial.println(muxOK ? "# PCA9548A OK" : "# PCA9548A missing: check address/power/wiring");
  if (muxOK) {
    laser.setTimeout(200);
    laserOK = laser.init();
    if (laserOK) {
      laserOK = laser.setDistanceMode(VL53L1X::Long) &&
                laser.setMeasurementTimingBudget(50000);
      if (laserOK) laser.startContinuous(100);
    }
    imuOK = initIMU();
  }
  Serial.println(laserOK ? "# VL53L1X CH0 OK" : "# VL53L1X CH0 initialization failed");
  Serial.println(imuOK ? "# MPU6500 CH1 OK" : "# MPU6500 CH1 missing or WHO_AM_I is not 0x70");
  Serial.println("# TMC2209: arrow controller sends F/R; key release sends S");
  Serial.println("# Motor speed: 400 STEP pulses/s; 350 ms communication safety stop");
  Serial.println("# nan = unavailable/no new sample; range_status 0 = valid, -1 = no sample");
  Serial.println("ms,hx711_raw,distance_mm,range_status,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C,motor");
}

void loop() {
  serviceMotorCommands();

  // Poll readiness so a disconnected HX711 does not block other sensors.
  if (!loadcell.isBusy()) {
    loadRaw = loadcell.readChannelRaw(CHAN_A_GAIN_128);
    loadFresh = true;
  }
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();
  Serial.print(lastPrint);
  Serial.print(',');
  if (loadFresh) Serial.print(loadRaw); else Serial.print("nan");
  loadFresh = false;
  Serial.print(',');

  int rangeStatus = -1;
  if (laserOK && selectChannel(LASER_CHANNEL) && laser.dataReady()) {
    uint16_t distance = laser.read(false);
    rangeStatus = int(laser.ranging_data.range_status);
    if (!laser.timeoutOccurred() && laser.last_status == 0 &&
        rangeStatus == int(VL53L1X::RangeValid)) Serial.print(distance);
    else Serial.print("nan");
  } else Serial.print("nan");
  Serial.print(',');
  Serial.print(rangeStatus);

  uint8_t data[14];
  if (imuOK && selectChannel(IMU_CHANNEL) && readIMU(0x3B, data, 14)) {
    for (uint8_t offset = 0; offset < 6; offset += 2) {
      Serial.print(',');
      Serial.print(signedWord(data + offset) / 16384.0f, 4);
    }
    for (uint8_t offset = 8; offset < 14; offset += 2) {
      Serial.print(',');
      Serial.print(signedWord(data + offset) / 131.0f, 3);
    }
    Serial.print(',');
    Serial.print(signedWord(data + 6) / 333.87f + 21.0f, 2);
  } else Serial.print(",nan,nan,nan,nan,nan,nan,nan");

  Serial.print(',');
  if (motorDirection > 0) Serial.println("forward");
  else if (motorDirection < 0) Serial.println("reverse");
  else Serial.println("stopped");
}

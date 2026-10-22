// Open loop: no feedback
//   command = (RPM + 8) / 1.84        <- your calibration equation
//   deg/s   = RPM * 6                 (1 RPM = 360 deg per 60 s)
//   time    = degrees / (deg/s)

// Serial Monitor at 115200, line ending "Newline":
//   30 / 0 / -20  -> move to that angle (clamped to +/-60)
//   z             -> "it's at 0 right now" (resets the estimate and the sensor zero)
//   s             -> wave the servo arm once (up to WAVE_MAX and back to 0)

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Servo
Servo armServo;
const int ARM_PIN      = 9;
const int WAVE_MAX     = 40;     // top of the wave. Never go above 60, or it will hit wires
const int WAVE_STEP_MS = 25;     // pause per degree; bigger = slower wave

// DC Motor
const int   MOTOR_PORT = 1;
const float MOVE_RPM   = 80;     // speed for every move (command ~48)
const float LIMIT_DEG  = 60.0;

const int CS_PIN   = 10;
const int DIR_SIGN = 1;
uint16_t  zeroRaw  = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor;
SPISettings asSPI(1000000, MSBFIRST, SPI_MODE1);

float estDeg = 0; // Shaft Reading from AS5047P

int cmdForRPM(float rpm)
{
  return (int)((rpm + 8.0) / 1.84 + 0.5);
}

void setMotor(int u) {
  motor->setSpeed(abs(u));
  motor->run(u > 0 ? FORWARD : (u < 0 ? BACKWARD : RELEASE));
}

/**
 * @brief swings the arm from 0 up to WAVE_MAX and back down to 0, then returns.
 */
void wave() {
  for (int i = 0; i <= WAVE_MAX; i++) {
    armServo.write(i);
    delay(WAVE_STEP_MS);
  }
  for (int i = WAVE_MAX; i >= 0; i--) {
    armServo.write(i);
    delay(WAVE_STEP_MS);
  }
}

/**
 * @brief moves the angle to the position by finding the time to delay the motor by (runMs)
 * It will not care for slight angle movement for less than 0.5 deg.
 * It will move the motor by MOVE_RPM speed. Tunable. Do not tune below 45, the deadband range.
 */
void moveTo(float targetDeg) {
  targetDeg = constrain(targetDeg, -LIMIT_DEG, LIMIT_DEG);
  float delta = targetDeg - estDeg;
  if (fabs(delta) < 0.5) return;

  int cmd = cmdForRPM(MOVE_RPM);
  float realRPM = 1.84 * cmd - 8.0;        // speed this whole-number command actually gives
  float degPerSec = realRPM * 6.0;
  unsigned long runMs = (unsigned long)(fabs(delta) / degPerSec * 1000.0 + 0.5);

  setMotor(delta > 0 ? cmd : -cmd);
  delay(runMs);
  setMotor(0);
  estDeg = targetDeg;

  delay(150);                                // let it coast to a stop before reporting
  Serial.print(F("Moved ")); Serial.print(delta, 1);
  Serial.print(F("deg: commanded ")); Serial.print(cmd);
  Serial.print(F(" for ")); Serial.print(runMs); 
  Serial.print(F(" ms. Estimate ")); Serial.print(estDeg, 1); 
  Serial.print(F(" deg, sensor says ")); Serial.print(readDeg(), 1); Serial.print(F(" deg"));
  Serial.println();
}

/**
 * @brief Writes 16-bit command to sensors
 */
uint16_t xfer(uint16_t w) {
  SPI.beginTransaction(asSPI);
  digitalWrite(CS_PIN, LOW);
  uint16_t r = SPI.transfer16(w);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return r;
}

/**
 * @brief reads the raw data
 * 0xFFFF represents represents a read
 * 0x3FFF represents the angle register
 */
uint16_t readRaw() {
  xfer(0xFFFF);
  return xfer(0xFFFF) & 0x3FFF;
}

float readDeg() {
  int16_t d = (int16_t)readRaw() - (int16_t)zeroRaw;

  if (d > 8192) d -= 16384;
  else if (d < -8192) d += 16384;

  return DIR_SIGN * d * 360.0 / 16384.0;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  armServo.attach(ARM_PIN);
  armServo.write(0);                         // arm starts down

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  if (!AFMS.begin()){
    Serial.println(F("Motor shield not found.")); 
    while (1);  // halt program indefinitely
  }
  motor = AFMS.getMotor(MOTOR_PORT);
  setMotor(0);
  xfer(0x4001);  
  xfer(0xC000); // clear any power-up sensor error

  Serial.println("Started");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "z") {
      // Used for debugging
      estDeg = 0;
      zeroRaw = readRaw();
      Serial.println(F("Estimate and sensor zeroed here."));
    }
    else if (s == "s") {
      // Must be a wave command
      wave();
    }
    else if (s.length() > 0) {
      // Must be an angle command
      moveTo(s.toFloat());
    }
  }
}

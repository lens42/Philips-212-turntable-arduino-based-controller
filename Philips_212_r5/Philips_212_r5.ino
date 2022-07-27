#include "QuickPID.h"   // https://github.com/Dlloydev/QuickPID
#include <movingAvg.h>  // https://github.com/JChristensen/movingAvg

/*
 * Constants specific to each GA-212 turntable
 */

// Factor that turns velocity into a number that looks like RPMs. Set by trying values while measuring turntable with a separate tachometer.
// The higher the value, the slower the motor spins
const float VEL_FACTOR = 0.846798789;
// Threshold for end-of-record auto-off photo sensor. This probably doesn't need to be adjusted...
const int PHOTO_MIN_VALUE = 800;

/*
 * Constants
 */

// Motor velocity target values for 33RPM and 45RPM
const float TARGET_VEL_33 = 33.33 * 100;
const float TARGET_VEL_45 = 45 * 100;
// Debug flag
const bool DEBUG = false;
// Whether to apply speed trimmer values
const bool APPLY_TRIM = true;
// LED pins
const byte LEDOFF = 6;
const byte LED33 = 5;
const byte LED45 = 7;
// Button pins
const byte BTNOFF = 9;
const byte BTN33 = 8;
const byte BTN45 = 10;
// 33 & 45 RPM speed trimmer pin
const byte TRIMPOT_33 = A0;
const byte TRIMPOT_45 = A1;
// Auto off photo resistor pin
const byte OPTO = A2;
// Motor tachometer pin
const byte TACH_IN = 2;
// PWM motor drive pin
const byte PWM_OUT = 11;
// DEBOUNCE_MS time (ms) can be short because OFF, 33, and 45 states latch until a different button is pressed
const byte DEBOUNCE_MS = 10;
// Number of photo measurements to average
const byte PHOTO_MEASUREMENT_COUNT = 20;
// Amount that valuePhoto needs to change between two measurements, in order to trigger auto-off
const byte PHOTO_CHANGE_THRESHOLD = 10;
// Pitch trim range (+ or -) 2%
const int TRIM_AMOUNT_33 = TARGET_VEL_33 * 0.02;
const int TRIM_AMOUNT_45 = TARGET_VEL_45 * 0.02;

/*
 * Variables
 */

// Play state
enum playStateValue {
  off,
  play33,
  play45
};
playStateValue playState = off;
// 45 RPM fine trim pot value
int valueTrim45;
// 33 RPM fine trim pot value
int valueTrim33;
// Auto-off photo sensor value
int valuePhoto = 0;
// Flag to remember whether we're measuring the auto-off photo resistor
bool photoMeasurementIsActive = false;
// Moving average of the valuePhoto measurement
movingAvg valuePhotoAvg(PHOTO_MEASUREMENT_COUNT);
// Previous valuePhoto average
int previousValuePhotoAvg = 0;

// The following variables are longs because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.

// Last time OFF button changed state
long lastTimeOff = 0;
// Last time 33 button changed state
long lastTime33 = 0;
// Last time 45 button changed state
long lastTime45 = 0;
// Last time 33 or 45 button changed state
long lastTimePlay = 0;
// Last time we started valuePhoto readings
long timePhotoMeasureStart = 0;

// PID
float pidSetpoint = 0, pidInput, pidOutput = 0;
const float pidP = 0.06, pidI = 0.4, pidD = 0.0001;
QuickPID motorPid(&pidInput, &pidOutput, &pidSetpoint);

// Use the "volatile" directive for variables
// used in an interrupt
volatile float tachPeriodInterrupt = 0;
volatile unsigned long prevTimeInterrupt = 0;

void setup()
{
  // Set pins
  pinMode(BTNOFF, INPUT);
  pinMode(BTN33, INPUT);
  pinMode(BTN45, INPUT);

  pinMode(LED45, OUTPUT);
  pinMode(LED33, OUTPUT);
  pinMode(LEDOFF, OUTPUT);

  pinMode(TACH_IN, INPUT);
  pinMode(PWM_OUT, OUTPUT);

  // Set external interrupt for tachometer period measurement
  attachInterrupt(digitalPinToInterrupt(TACH_IN), interruptReadTachometer, FALLING);

  // Set up PID speed control
  motorPid.SetTunings(pidP, pidI, pidD);
  motorPid.SetOutputLimits(0, 255);
  motorPid.SetAntiWindupMode(motorPid.iAwMode::iAwOff);
  // Roughly 500Hz PWM drive
  motorPid.SetSampleTimeUs(2000);
  // Turn PID on
  motorPid.SetMode(motorPid.Control::automatic);

  // Initialise moving averages
  valuePhotoAvg.begin();

  if (DEBUG) {
    Serial.begin(115200);
  }
}

/*
 * Interrupt function to determine motor velocity using tachometer pulses
 */
void interruptReadTachometer()
{
  long currTime = micros();
  tachPeriodInterrupt = float (currTime - prevTimeInterrupt);
  prevTimeInterrupt = currTime;
}

/*
 * Main loop
 */
void loop()
{
  readInputs();

  handleButtonPresses();

  driveLeds();

  autoOff();

  motorControl();
    
  debug();
}

void readInputs()
{
  // read the motor velocity
  noInterrupts(); // disable interrupts temporarily while reading
  float tachPeriod = tachPeriodInterrupt;
  interrupts(); // turn interrupts back on

  // Compute velocity from tach period
  // Dividing by 1.0e8 puts the number in a range that is suitable to the PID
  pidInput = 1/(tachPeriod/1.0e8) * VEL_FACTOR;

  if (APPLY_TRIM) {
    valueTrim45 = analogRead(TRIMPOT_45);
    valueTrim33 = analogRead(TRIMPOT_33);
  }
  valuePhoto = analogRead(OPTO);
}

/*
 * Get amount to add to setPoint depending on trimmer value
 */
float getTrimAmount(bool for33 = true)
{
  int trimAmount = for33 ? TRIM_AMOUNT_33 : TRIM_AMOUNT_45;
  int trimmerValue = for33 ? valueTrim33 : valueTrim45;
  return trimAmount * map(trimmerValue, 1023, 0, -512, 512) / 512;
}

void handleButtonPresses()
{
  /*
   * "OFF" button
   */
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, drive the output pin low and the other two outputs high and remember
  // the time
  if (playState != off && digitalRead(BTNOFF) == LOW && millis() - lastTimeOff > DEBOUNCE_MS) {
    playState = off;
    pidSetpoint = 0;
    lastTimeOff = millis();
  }

  /*
   * "33 RPM" button
   */
  if (playState != play33 && digitalRead(BTN33) == LOW && millis() - lastTime33 > DEBOUNCE_MS) {
    lastTime33 = millis();
    if (playState == off) {
      lastTimePlay = lastTime33;
    }
    pidSetpoint = TARGET_VEL_33;
    pidInput = pidSetpoint * 1.5;
    playState = play33;
  } else if (playState == play33 && APPLY_TRIM) {
    // We didn't press the button, but we do want to apply trim
    pidSetpoint = TARGET_VEL_33 + getTrimAmount(true);
  }

  /*
   * "45 RPM" button
   */
  if (playState != play45 && digitalRead(BTN45) == LOW && millis() - lastTime45 > DEBOUNCE_MS) {
    lastTime45 = millis();
    if (playState == off) {
      lastTimePlay = lastTime45;
    }
    pidSetpoint = TARGET_VEL_45;
    pidInput = pidSetpoint * 1.5;
    playState = play45;
  } else if (playState == play45 && APPLY_TRIM) {
    // We didn't press the button, but we do want to apply trim
    pidSetpoint = TARGET_VEL_45 + getTrimAmount(false);
  }
}

void driveLeds()
{
  digitalWrite(LEDOFF, playState == off ? LOW : HIGH);
  digitalWrite(LED33, playState == play33 ? LOW : HIGH);
  digitalWrite(LED45, playState == play45 ? LOW : HIGH);
}

void autoOff()
{
  /*
   * Auto-off logic using tone arm photo trip
   *
   * When the tonearm reaches the record end, the photo resistor is interrupted and the OFF state is set.
   */
  if (playState == off || valuePhoto < PHOTO_MIN_VALUE) {
    // Only apply auto-off logic:
    // - when the turntable is playing.
    // - when a minimum amount of light is detected by the optical sensor,
    //   which indicates that the tone arm is close to the end of the record.
    return;
  }
  // Start a new measurement every 200 ms
  if (photoMeasurementIsActive == false && millis() - timePhotoMeasureStart > 200) {
    timePhotoMeasureStart = millis();
    valuePhotoAvg.reset();
    photoMeasurementIsActive = true;
  }
  if (!photoMeasurementIsActive) {
    return;
  }
  // Add a reading up to the amount needed
  if (valuePhotoAvg.getCount() < PHOTO_MEASUREMENT_COUNT) {
    valuePhotoAvg.reading(valuePhoto);
    return;
  }
  // We have enough readings to average, check if auto-off threshold was triggered.
  if (valuePhotoAvg.getAvg() - previousValuePhotoAvg > PHOTO_CHANGE_THRESHOLD) {
    // Turn off...
    playState = off;
  }
  previousValuePhotoAvg = valuePhotoAvg.getAvg();
  // Stop measuring, we'll start a new measurement next loop...
  photoMeasurementIsActive = false;
}

void motorControl()
{
  /*
   * Hold motor off during off state, else drive with PWM output
   */
  if (playState == off) {
    analogWrite(PWM_OUT, 0);
    return;
  }

  if (millis() - lastTimePlay < 1000) {
    // Give the motor enough power initially to get going (circumvent PID)
    analogWrite(PWM_OUT, 250);
    return;
  }

  // We don't need much power afterwards to keep the motor going, this is roughly the required range
  motorPid.SetOutputLimits(50, 150);

  // Get PID value and apply
  motorPid.Compute();
  analogWrite(PWM_OUT, pidOutput);
}

void debug()
{
  /*
   * Debug output
   */
  if (!DEBUG) {
    return;
  }
  Serial.print("pidInput:");
  Serial.print(pidInput);
  Serial.print(",");
  Serial.print("pidOutput:");
  Serial.print(pidOutput * 10);
  Serial.print(",");
  Serial.print("pidSetpoint:");
  Serial.print(pidSetpoint);
  Serial.println();
}

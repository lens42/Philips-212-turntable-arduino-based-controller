#include "QuickPID.h"   // https://github.com/Dlloydev/QuickPID
#include <movingAvg.h>  // https://github.com/JChristensen/movingAvg

/*
 * Constants specific to each GA-212 turntable
 */

// Motor velocity target values for 33RPM and 45RPM. Set by trying values while measuring turntable with a separate tachometer
const float t_set_33 = 3936;
const int t_set_45 = 5318;
// Pitch trim range (+ or -)
const int trim_amount_33 = t_set_33 * 0.02;
const int trim_amount_45 = t_set_45 * 0.02;
// Whether to apply speed trimmer values
const bool APPLY_TRIM = true;
// Threshold for end-of-record auto-off photo sensor
const int photo_trip_min = 800;


/*
 * Constants
 */

// Debug flag
const bool DEBUG = false;
// "OFF" LED drive output
const byte LEDOFF = 6;
// "33" LED drive output
const byte LED33 = 5;
// "45" LED drive output
const byte LED45 = 7;
// "OFF" touch button input
const byte BTNOFF = 9;
// "33" touch button input
const byte BTN33 = 8;
// "45" touch button input
const byte BTN45 = 10;
// Analog input from trim pot to fine tune 33 RPM
const byte TRIMPOT_33 = A0;
// Analog inpit from trim pot to fine tune 45 RPM
const byte TRIMPOT_45 = A1;
// Analog input from auto off photo resistor
const byte OPTO = A2;
// Input pin for tach output from motor
const byte TACH_IN = 2;
// PWM drive output to motor
const byte PWM_OUT = 11;
// Debounce time (ms) can be short because OFF, 33, and 45 states latch until a different button is pressed
const byte debounce = 10;
// Number of v_photo measurements to average
const byte v_photo_avg_count = 20;
// Threshold v_photo when the tonearm is in the exit groove
const byte v_photo_threshold = 10;


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
int v_trim_45;
// 33 RPM fine trim pot value
int v_trim_33;
// Auto-off photo sensor value
int v_photo = 0;
// Flag to remember whether we're measuring the auto-off photo resistor
bool photo_measuring = false;
// Moving average of the v_photo measurement
movingAvg vPhotoAvg(v_photo_avg_count);
// Previous v_photo average
int previous_v_photo_avg = 0;

// The following variables are longs because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.

// Last time OFF button changed state
long timeOFF = 0;
// Last time 33 button changed state
long time33 = 0;
// Last time 45 button changed state
long time45 = 0;
// Last time 33 or 45 button changed state
long timePlay = 0;
// Last time we started v_photo readings
long time_photo_start = 0;

// PID
float pidSetpoint = 0;
float pidInput;
float pidOutput = 0;
const float pidP = 0.12, pidI = 0.6, pidD = 0.00001;
QuickPID motorPid(&pidInput, &pidOutput, &pidSetpoint);

// Use the "volatile" directive for variables
// used in an interrupt
volatile float velocity_i = 0;
volatile long prevT_i = 0;

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
  attachInterrupt(digitalPinToInterrupt(TACH_IN), tachometer, FALLING);

  // Set up PID speed control
  motorPid.SetTunings(pidP, pidI, pidD);
  motorPid.SetOutputLimits(0, 255);
  motorPid.SetAntiWindupMode(motorPid.iAwMode::iAwOff);
  // Turn PID on
  motorPid.SetMode(motorPid.Control::automatic);

  // Initialise v_photo measurement moving average
  vPhotoAvg.begin();

  if (DEBUG) {
    Serial.begin(115200);
  }
}

/*
 * Determine time between pulses from tachometer
 */
void tachometer()
{
  // Compute velocity
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e8;
  velocity_i = 1/deltaT;
  prevT_i = currT;
}

void buttonsAndLeds()
{
  /*
   * "OFF" button and LED
   */
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, drive the output pin low and the other two outputs high and remember
  // the time
  if (playState != off && digitalRead(BTNOFF) == LOW && millis() - timeOFF > debounce) {
    playState = off;
    pidSetpoint = 0;
    timeOFF = millis();
  }
  digitalWrite(LEDOFF, playState == off ? LOW : HIGH);

  /*
   * "33 RPM" button and LED
   */
  if (playState != play33 && digitalRead(BTN33) == LOW && millis() - time33 > debounce) {
    time33 = millis();
    if (playState == off) {
      timePlay = time33;
    }
    pidSetpoint = t_set_33;
    pidInput = pidSetpoint * 1.5;
    playState = play33;
  } else if (playState == play33 && APPLY_TRIM) {
    float adjust33 = map(v_trim_33, 1023, 0, -100, 100);
    pidSetpoint = t_set_33 + (trim_amount_33 * (adjust33 / 100));
  }
  digitalWrite(LED33, playState == play33 ? LOW : HIGH);

  /*
   * "45 RPM" button and LED
   */
  if (playState != play45 && digitalRead(BTN45) == LOW && millis() - time45 > debounce) {
    time45 = millis();
    if (playState == off) {
      timePlay = time45;
    }
    pidSetpoint = t_set_45;
    pidInput = pidSetpoint * 1.5;
    playState = play45;
  } else if (playState == play45 && APPLY_TRIM) {
    float adjust45 = map(v_trim_45, 1023, 0, -100, 100);
    pidSetpoint = t_set_45 + (trim_amount_45 * (adjust45 / 100));
  }
  digitalWrite(LED45, playState == play45 ? LOW : HIGH);
}

void autoOff()
{
  /*
   * Tonearm photo trip
   *
   * When the tonearm reaches the record end, the photo resistor is interrupted and the OFF state is set.
   */
  if (playState != off && v_photo > photo_trip_min) {
    if (photo_measuring == false && millis() - time_photo_start > 200) {
      // Start a new measurement every 200 ms
      time_photo_start = millis();
      photo_measuring = true;
      vPhotoAvg.reset();
    }
    if (photo_measuring) {
      // Add a reading up to the count
      if (vPhotoAvg.getCount() < v_photo_avg_count) {
        vPhotoAvg.reading(v_photo);
      } else {
        // We have sufficient readings, stop measuring
        photo_measuring = false;
        // Compare measurement, turn off if over threshold
        if (vPhotoAvg.getAvg() - previous_v_photo_avg > v_photo_threshold) {
          playState = off;
        }
        previous_v_photo_avg = vPhotoAvg.getAvg();
      }
    }
  }
}

/* switch

   Each time the input pin goes from LOW to HIGH (e.g. because of a push-button
   press), the corrosponding output pin goes low and the other two go high.
   Repeated button presses of the same button do nothing. A different button must be pressed to change state.
   There's a minimum delay between toggles to debounce the circuit (i.e. to ignore
   noise)

   David A. Mellis with modifications by L. Sherman (2/2019) for interlock mechanism
   21 November 2006
*/

void loop()
{
  // read the velocity
  noInterrupts(); // disable interrupts temporarily while reading
  pidInput = velocity_i;
  interrupts(); // turn interrupts back on

  if (APPLY_TRIM) {
    v_trim_45 = analogRead(TRIMPOT_45);
    v_trim_33 = analogRead(TRIMPOT_33);
  }
  v_photo = analogRead(OPTO);
  
  debug();

  buttonsAndLeds();

  autoOff();

  /*
   * Control motor
   *
   * Hold motor off during off state, else drive with PWM output
   */
  if (playState == off) {
    analogWrite(PWM_OUT, 0);
  } else {
    if (playState != off && millis() - timePlay < 100) {
      // Give the motor enough power initially to get going
      analogWrite(PWM_OUT, 250);
    } else {
      // We don't need that much power afterwards
      motorPid.SetOutputLimits(50, 150);
      motorPid.Compute();
      analogWrite(PWM_OUT, pidOutput);
    }
  }

}

/*
 * Debug output
 */
void debug()
{
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

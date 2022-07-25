#include <movingAvg.h>  // https://github.com/JChristensen/movingAvg

/*
 * Constants specific to each GA-212 turntable
 */

// Tach period target value for 33RPM. Set by trying values while measuring turntable with a separate tachometer
const int t_set_33 = 20070;
// Tach period target value for 45RPM. Set by trying values while measuring turntable with a separate tachometer
const int t_set_45 = 12700;
// Maps to PWM output to set gain. Lower number equals higher gain. Set while watching PWM output on oscilloscope. Set to highest values that does not result in oscillation pulse width.
const int t_err_range = 11000;
// Offset fine trim reading so center rotation = 0
const int trim_offset = 512;
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
// PWM output that drives motor at 33.3 RPM. Measured to get approximately correct output when t_diff is 0
const int n_offset = 127 - 43;
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

// Difference between set and target period
int t_diff;
// Tach period target value for the currently active RPM
int t_set;
// PWM motor drive
int n_PWM;
volatile unsigned long t_per;
unsigned long microseconds;

// The following variables are longs because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.

// Last time OFF button changed state
long timeOFF = 0;
// Last time 33 button changed state
long time33 = 0;
// Last time 45 button changed state
long time45 = 0;
// Last time we started v_photo readings
long time_photo_start = 0;


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

  // Start with long period to ensure startup
  t_per = 50000;

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
  t_per = micros() - microseconds;
  microseconds = micros();
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
  debug();

  v_trim_45 = analogRead(TRIMPOT_45);
  v_trim_33 = analogRead(TRIMPOT_33);
  v_photo = analogRead(OPTO);

  /*
   * "OFF" button and LED
   */
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, drive the output pin low and the other two outputs high and remember
  // the time
  if (playState != off && digitalRead(BTNOFF) == LOW && millis() - timeOFF > debounce) {
    playState = off;
    timeOFF = millis();
  }
  digitalWrite(LEDOFF, playState == off ? LOW : HIGH);

  /*
   * "33 RPM" button and LED
   */
  if (playState != play33 && digitalRead(BTN33) == LOW && millis() - time33 > debounce) {
    playState = play33;
    time33 = millis();
  }
  if (playState == play33) {
    t_set = t_set_33 + v_trim_33 - trim_offset;
  }
  digitalWrite(LED33, playState == play33 ? LOW : HIGH);

  /*
   * "45 RPM" button and LED
   */
  if (playState != play45 && digitalRead(BTN45) == LOW && millis() - time45 > debounce) {
    playState = play45;
    time45 = millis();
  }
  if (playState == play45) {
    t_set = t_set_45 + v_trim_45 - trim_offset;
  }
  digitalWrite(LED45, playState == play45 ? LOW : HIGH);

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

  /*
   * Read period from tachometer
   */
  t_per = constrain(t_per, 5000, 50000);
  t_diff = t_per - t_set;
  n_PWM = map(t_diff, t_err_range, -t_err_range, 255, 0);
  n_PWM = n_PWM - n_offset;
  n_PWM = constrain(n_PWM, 0, 255);

  /*
   * Control motor
   *
   * Hold motor off during off state, else drive with PWM output
   */
  analogWrite (PWM_OUT, playState == off ? 0 : n_PWM);

}

/*
 * Debug output
 */
void debug()
{
  if (DEBUG) {
    Serial.print(t_per);
    Serial.print(" ");
    Serial.print(t_diff);
    Serial.print(" ");
    Serial.print(v_trim_33);
    Serial.print(" ");
    Serial.print(v_trim_45);
    Serial.print (" ");
    Serial.print (t_set);
    Serial.print (" ");
    Serial.print (n_PWM);
    Serial.print (" ");
    Serial.print (v_photo);
    Serial.print (" ");
    Serial.print (playState);
    Serial.print (" ");
    if (vPhotoAvg.getAvg() - previous_v_photo_avg > 0) {
      Serial.print (vPhotoAvg.getAvg() - previous_v_photo_avg);
      Serial.print (" ");
    }
    Serial.println();
  }
}

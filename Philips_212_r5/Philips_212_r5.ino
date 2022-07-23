
#define LEDOFF 6    // "OFF" LED drive output, LOW = on
#define LED33 5     // "33" LED drive output, LOW = on
#define LED45 7     // "45" LED drive output, LOW = on
#define BTNOFF 9    // "OFF" touch button input
#define BTN33 8     // "33" touch button input
#define BTN45 10    // "45" touch button input
#define trimpot_33 A0  // analog input from trim pot to fine tune 33 RPM
#define trimpot_45 A1   // analog inpit from trim pot to fine tune 45 RPM
#define OPTO A2         // analog input from auto off photo resistor

byte stateOFF = LOW;      // current state of LEDOFF output, start LEDOFF low (on) at power up
byte readingOFF = LOW;   // current reading from BTNOFF input, start BTNOFF low at power up
byte previousOFF = LOW;   // the previous reading from the BTN input, start low at power up

byte state33 = HIGH;      //start 33 RPM off at power up
byte reading33 = HIGH;
byte previous33 = HIGH;

byte state45 = HIGH;      //start 45 RPM off at power up
byte reading45 = HIGH;
byte previous45 = HIGH;

int v_trim_45;            //45 RPM fine trim pot value
int v_trim_33;            //33 RPM fine trim pot value
int v_photo;               //auto-off photo sensor value
int photo_trip;             //threshold for auto-off sensing
int trim_offset;            //offset fine trim reading so center rotation = 0

byte tach_in = 2;         //input pin for tach output from motor
byte PWM_out = 11;        //PWM drive output to motor
int t_diff;              //difference between set and target period
int t_set;               //Tach period target value for the currently active RPM
int t_set_33;            //Tach period target value for 33RPM
int t_set_45;            //Tach period target value for 45RPM
int t_err_range;         //sets gain of motor control loop
int n_PWM;               //PWM motor drive
int n_offset;
volatile unsigned long t_per;
unsigned long microseconds;


// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long timeOFF = 0;         // the last time the OFF touch botton changed state
long time33 = 0;          // the last time the 33 touch botton changed state
long time45 = 0;          // the last time the 45 touch botton changed state
long debounce = 10;   // debounce time (ms) can be short because OFF, 33, and 45 states latch until a different button is pressed

void setup()
{
  pinMode(BTNOFF, INPUT);  //set up OFF, 33, and 45 buttons
  pinMode(BTN33, INPUT);
  pinMode(BTN45, INPUT);
  pinMode(LED45, OUTPUT);  //set up OFF, 33, and 45 indicator LEDs
  pinMode(LED33, OUTPUT);
  pinMode(LEDOFF, OUTPUT);
  pinMode(tach_in, INPUT);
  pinMode(PWM_out, OUTPUT);
  attachInterrupt(0, tachometer, FALLING); // set external interrupt for tachometer period measurement
  //  Serial.begin(115200);   //run when troubleshooting

  t_set_33 = 20070;         //target period for 33RPM. Set by trying values while measuring turntable with a separate tachometer

  t_set_45 = 12700;         //target period for 45RPM. Set by trying values while measuring turntable with a separate tachometer

  t_per = 50000;            //start with long per to ensure startup
  t_err_range = 11000;      //maps to PWM output to set gain. Lower number equals higher gain. Set while watching PWM output on oscilloscope. Set to highest values that does not result in oscillation pulse width.
  n_offset = 127 - 43;      // PWM output that drives motor at 33.3 RPM. Measured to get approximately correct output when t_diff is 0
  trim_offset = 512;        // offset fine adjust readings so mid rotation = 0
  photo_trip = 600;         // threshold for end-of-record auto-off photo sensor

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

  // trouble-shooting serial print parameters
  //Serial.print(t_per);
  //Serial.print(" ");
  //Serial.print(t_diff);
  //Serial.print(" ");
  //Serial.print(v_trim_33);
  //Serial.print(" ");
  //Serial.print(v_trim_45);
  //Serial.print (" ");
  //Serial.print (t_set);
  //Serial.print (" ");
  //Serial.print (n_PWM);
  //Serial.print (" ");
  //Serial.print (v_photo);
  //Serial.println();

  v_trim_45 = analogRead (trimpot_45);
  v_trim_33 = analogRead (trimpot_33);
  v_photo = analogRead (OPTO);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, drive the output pin low and the other two outputs high and remember
  // the time

  // code for "OFF" button and LED

  readingOFF = digitalRead(BTNOFF);
  if (readingOFF == LOW && millis() - timeOFF > debounce) {
    if (stateOFF == HIGH)
    { stateOFF = LOW;
      state33 = HIGH;     //turn off OFF and 45 LEDS
      state45 = HIGH;
    }    //turn off OFF and 33 LEDS

    else
      stateOFF = LOW;

    timeOFF = millis();
  }
  digitalWrite(LEDOFF, stateOFF);
  previousOFF = readingOFF;


  // code for "33 RPM" button and LED
  reading33 = digitalRead(BTN33);

  if (reading33 == LOW && millis() - time33 > debounce) {
    if (state33 == HIGH)
    { stateOFF = HIGH;
      state33 = LOW;
      state45 = HIGH;

    }
    else
      state33 = LOW;
    time33 = millis();

  }

  if (state33 == LOW) {
    t_set = t_set_33 + v_trim_33 - trim_offset;
  }

  digitalWrite(LED33, state33);

  previous33 = reading33;


  // code for "45 RPM" button and LED
  reading45 = digitalRead(BTN45);

  if (reading45 == LOW && millis() - time45 > debounce) {
    if (state45 == HIGH)
    { stateOFF = HIGH;
      state33 = HIGH;
      state45 = LOW;

    }

    else
      state45 = LOW;
    time45 = millis();

  }

  if (state45 == LOW) {
    t_set = t_set_45 + v_trim_45 - trim_offset;
  }

  digitalWrite(LED45, state45);
  previous45 = reading45;

  //end of "45 RPM" code

  if (v_photo > photo_trip) // When the tonearm reaches the record end, the photo resistor is interrupted and the OFF state is set
  { stateOFF = LOW;     // off state, OFF LED on
    state33 = HIGH;     // turn off OFF and 45 LEDS
    state45 = HIGH;
  }

  // read period from tachometer and control motor
  t_per = constrain(t_per, 5000, 50000);
  t_diff = t_per - t_set;
  n_PWM = map(t_diff, t_err_range, -t_err_range, 255, 0);
  n_PWM = n_PWM - n_offset;
  n_PWM = constrain(n_PWM, 0, 255);

  if (stateOFF == LOW) analogWrite (PWM_out, 0);    //hold motor off during OFF state
  else analogWrite(PWM_out, n_PWM);                 //else drive motor with PWM output

}

//Determine time between pulses from tachometer
void tachometer()
{
  t_per = micros() - microseconds;
  microseconds = micros();
}

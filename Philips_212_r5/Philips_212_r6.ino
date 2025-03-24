#define LED_OFF 6         // "OFF" LED drive output, LOW = on

#define LED_33 5          // "33" LED drive output, LOW = on

#define LED_45 7          // "45" LED drive output, LOW = on

#define BTN_OFF 9         // "OFF" touch button input

#define BTN_33 8          // "33" touch button input

#define BTN_45 10         // "45" touch button input

#define TRIM_POT_33 A0    // Analog input from trim pot to fine tune 33 RPM

#define TRIM_POT_45 A1    // Analog input from trim pot to fine tune 45 RPM

#define OPTO A2           // Analog input from auto-off photo resistor

#define TACH_IN 2         // Input pin for tach output from motor

#define PWM_OUT 11        // PWM drive output to motor

#define DEBOUNCE_TIME 10  // Debounce time in milliseconds

#define PHOTO_TRIP 700    // Threshold for end-of-record auto-off photo sensor

#define TRIM_OFFSET 512   // Offset fine trim reading so center rotation = 0 (512)

#define T_SET_33 24000    // Target period for 33 RPM (was 24650)

#define T_SET_45 16550    // Target period for 45 RPM (was 17700)

#define T_ERR_RANGE 7000  // Sets gain of motor control loop

#define N_OFFSET 84       // PWM output that drives motor at 33.3 RPM

byte stateOff = LOW;      // Current state of LED_OFF output, start LED_OFF low (on) at power up

byte readingOff = LOW;    // Current reading from BTN_OFF input, start BTN_OFF low at power up

byte state33 = HIGH;      // Start 33 RPM off at power up

byte reading33 = HIGH;

byte state45 = HIGH;      // Start 45 RPM off at power up

byte reading45 = HIGH;

int trim45;               // 45 RPM fine trim pot value

int trim33;               // 33 RPM fine trim pot value

int photoValue;           // Auto-off photo sensor value

int t_set;                // Tach period target value for the currently active RPM

long pwmValue = 0;        // PWM outout to motor

long tachDiff = 0;        // Tacho difference to compensate

volatile unsigned long tachPeriod;

unsigned long tachMicroseconds;

long lastDebounceTimeOff = 0;  // Last time the OFF touch button changed state

long lastDebounceTime33 = 0;   // Last time the 33 touch button changed state

long lastDebounceTime45 = 0;   // Last time the 45 touch button changed state

void setup() {

  Serial.begin(115200);   //run when troubleshooting

  pinMode(BTN_OFF, INPUT);  // Set up OFF, 33, and 45 buttons

  pinMode(BTN_33, INPUT);

  pinMode(BTN_45, INPUT);

  pinMode(LED_45, OUTPUT);  // Set up OFF, 33, and 45 indicator LEDs

  pinMode(LED_33, OUTPUT);

  pinMode(LED_OFF, OUTPUT);

  pinMode(TACH_IN, INPUT);

  pinMode(PWM_OUT, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(TACH_IN), tachometer, FALLING); // Set external interrupt for tachometer period measurement

  tachPeriod = 50000;  // Start with long period to ensure startup

} // setup

void loop() {

  trim45 = analogRead(TRIM_POT_45);

  trim33 = analogRead(TRIM_POT_33);

  photoValue = analogRead(OPTO);

  // Code for "OFF" button and LED

  readingOff = digitalRead(BTN_OFF);

  if (readingOff == LOW && millis() - lastDebounceTimeOff > DEBOUNCE_TIME) {

    stateOff = LOW;

    state33 = HIGH;  // Turn off OFF and 45 LEDs

    state45 = HIGH;  // Turn off OFF and 33 LEDs

    lastDebounceTimeOff = millis();

  } // if readingOff..

  digitalWrite(LED_OFF, stateOff);

  // Code for "33 RPM" button and LED

  reading33 = digitalRead(BTN_33);

  if (reading33 == LOW && millis() - lastDebounceTime33 > DEBOUNCE_TIME) {

    stateOff = HIGH;

    state33 = LOW;

    state45 = HIGH;

    lastDebounceTime33 = millis();

  } // if reading33..

  if (state33 == LOW) {

    t_set = T_SET_33 + trim33 - TRIM_OFFSET;

  } // if state33..

  digitalWrite(LED_33, state33);

  // Code for "45 RPM" button and LED

  reading45 = digitalRead(BTN_45);

  if (reading45 == LOW && millis() - lastDebounceTime45 > DEBOUNCE_TIME) {

    stateOff = HIGH;

    state33 = HIGH;

    state45 = LOW;        

    lastDebounceTime45 = millis();

  } // if reading45..

  if (state45 == LOW) {

    t_set = T_SET_45 + trim45 - TRIM_OFFSET;

  } // if state45..

  digitalWrite(LED_45, state45);

  // When the tonearm reaches the record end, the photo resistor is interrupted and the OFF state is set

  if (photoValue > PHOTO_TRIP) {

    stateOff = LOW;  // Off state, OFF LED on

    state33 = HIGH;  // Turn off OFF and 45 LEDs

    state45 = HIGH;

  } // if photoValue..

  // Read period from tachometer and control motor

  tachPeriod = constrain(tachPeriod, 5000, 50000);

  tachDiff = tachPeriod - t_set;  // Corrected calculation

  pwmValue = map(tachDiff, T_ERR_RANGE, -T_ERR_RANGE, 255, 0); // T_ERR_RANGE = 7000

  pwmValue = pwmValue - N_OFFSET; // N_OFFSET = 84 is pwm value for speed 33

  pwmValue = constrain(pwmValue, 0, 255);

  if (stateOff == LOW) {

    analogWrite(PWM_OUT, 0);  // Hold motor off during OFF state

  } else {

    analogWrite(PWM_OUT, pwmValue);  // Else drive motor with PWM output

    //Serial.println(pwmValue);

  } // if stateOff..

  Serial.println(photoValue);

} // loop

// Determine time between pulses from tachometer

void tachometer() {

  tachPeriod = micros() - tachMicroseconds;

  tachMicroseconds = micros();

} // tachometer

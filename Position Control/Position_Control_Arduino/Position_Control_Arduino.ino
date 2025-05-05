int stateA;
int stateB;
volatile long counter = 0;

void setup() {
  pinMode(A1, INPUT);
  pinMode(5, OUTPUT);  // Motor Direction 1
  pinMode(6, OUTPUT);  // Motor Direction 2
  pinMode(9, OUTPUT);  // PWM Pin
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(2), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderB, CHANGE);
}

void loop() {
  double a = analogRead(A1);
  double a_degrees = 360.0 * a / 1023.0;  // Potentiometer degrees
  double motor_position = counter * 0.043; // encoder tick to degrees
  double error = a_degrees - motor_position;

  double control_input = 8.0 * error;
  control_input = constrain(control_input, -5, 5);  // prevent overflow

  int abs_control_input = abs(control_input);
  int control_digital = map(abs_control_input, 0, 5, 0, 255);
  control_digital = constrain(control_digital, 0, 255);  // Just in case

  if (error < 0) {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
  } else {
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
  }

  analogWrite(9, control_digital);  // PWM based on error

  Serial.println(motor_position);  // Can change to a_degrees for testing
  delay(10); // optional debounce or smoother serial output
}

volatile bool stateA_last = LOW;

void encoderA() {
  bool stateA = digitalRead(2);
  bool stateB = digitalRead(3);
  if (stateA == stateB) {
    counter++;
  } else {
    counter--;
  }
}

void encoderB() {
  bool stateA = digitalRead(2);
  bool stateB = digitalRead(3);
  if (stateA != stateB) {
    counter++;
  } else {
    counter--;
  }
}
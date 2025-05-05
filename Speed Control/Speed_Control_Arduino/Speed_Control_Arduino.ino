volatile long counter = 0;

unsigned long prev_time = 0;
long prev_count = 0;
double prev_error = 0;

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
  unsigned long current_time = millis();
  if (current_time - prev_time >= 100) {  // Every 100 ms
    double dt = (current_time - prev_time) / 1000.0;  // Convert ms to seconds

    // Read and reset encoder count
    long current_count;
    current_count = counter;

    // Setpoint from analog
    double a = analogRead(A1);
    double a_rpm = 35.0 * a / 1023.0;

    // Feedback from encoder
    double derivative = (current_count - prev_count) / dt;
    double motor_rpm = 60.0 * derivative / 8245.0;

    // Error and derivative of error
    double error = a_rpm - motor_rpm;
    

    // PD Control (You can tune Kp and Kd)
    double Kp = 27.0;
    double control_input = Kp * error;
    int pwm = constrain(abs(control_input), 0, 255);

    // Direction control
    if (control_input > 0) {
      digitalWrite(5, HIGH); digitalWrite(6, LOW);
    } else if (control_input < 0) {
      digitalWrite(5, LOW); digitalWrite(6, HIGH);
    } else {
      digitalWrite(5, LOW); digitalWrite(6, LOW);
    }

    analogWrite(9, pwm);
    Serial.print("Setpoint: "); Serial.print(a_rpm);
    Serial.print(" RPM, Actual: "); Serial.print(motor_rpm);
    Serial.print(" RPM, PWM: "); Serial.println(pwm);
    prev_time = current_time;
    prev_count = current_count;
    prev_error = error;
  }
}
void encoderA() {
  bool a = digitalRead(2);
  bool b = digitalRead(3);
  if (a == b) {
    counter--;
  } else {
    counter++;
  }
}

void encoderB() {
  bool a = digitalRead(2);
  bool b = digitalRead(3);
  if (a != b) {
    counter--;
  } else {
    counter++;
  }
}

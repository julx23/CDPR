// PID from from https://circuitdigest.com/microcontroller-projects/arduino-based-encoder-motor-using-pid-controller

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // TX on pin 12 (D7) (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port. controls both motors

#include <PIDController.h>
/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pinns.
   for the teensy 2.0 its 5, 6, 7, 8
*/
#define ENCODER_A 8
#define ENCODER_B 7
#define ENCODER_A2 6 // long cord one
#define ENCODER_B2 5
#define loadCellPin1 A5 // load cell

#define __Kp 4 // Proportional constant
#define __Ki 0 // Integral Constant
#define __Kd 5 // Derivative Constant


#define MOTOR_LIMIT 45 // maximum driving signal value

volatile long int encoder_count = 0; // stores the current encoder count
volatile long int encoder_count2 = 500; // stores the second encoder's current count
int integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 127; // after PID computation, motor command is stored in this variable
int motor2_pwm_value = 127;
PIDController pidcontroller1;
PIDController pidcontroller2;

int trajectory[100];
int timings[100];
int traj_index = 0;
int timings_index = 0;

elapsedMicros usec; 
unsigned long currentMillis;

void setup() {
  Serial.begin(115200);
  SWSerial.begin(9600); // sabertooth
  pinMode(ENCODER_A, INPUT); 
  pinMode(ENCODER_B, INPUT); 
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT); 
  pinMode(11, OUTPUT); // IR LED
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2, RISING);
  pidcontroller1.begin(); // initialize the PID instance
  pidcontroller1.tune(__Kp, __Ki, __Kd); // Tune the PID, arguments: kP, kI, kD 
  pidcontroller1.limit(-MOTOR_LIMIT, MOTOR_LIMIT); // Limit the PID output

  pidcontroller2.begin(); // initialize the second PID instance
  pidcontroller2.tune(__Kp, __Ki, __Kd); 
  pidcontroller2.limit(-MOTOR_LIMIT, MOTOR_LIMIT);
}

void loop() {
  while (Serial.available() > 0) {
    Serial.println("Reading");
    incomingByte = Serial.read(); // stores the /n character
    if (incomingByte != 'g') {
      trajectory[traj_index] = Serial.parseInt(); // stores the position array
      timings[timings_index] = Serial.parseInt(); // stores the timing array
      Serial.println(trajectory[traj_index]);
      Serial.println(timings[timings_index]);
      traj_index++;
      timings_index++;
    } else {
      Serial.println("Going");
      runTraj(trajectory, timings, traj_index);
      memset(trajectory, 0, sizeof(trajectory));
      memset(timings, 0, sizeof(timings));
      traj_index = 0;
      timings_index = 0;
      Serial.println("reset");
      continue;
    }
  }
}

void runTraj(int trajectory[], int timings[], int traj_index) {
  unsigned long previousMillis = 0;

  Serial.println(traj_index);
  for(int i = 0; i < traj_index; i++) {
    Serial.println(trajectory[i]);
  }
  Serial.println("timings");
  for(int i = 0; i < traj_index; i++) {
    Serial.println(timings[i]);
  }
  Serial.println("going now");
  int idx = 0;
  int integerValue = 0;

  digitalWrite(11, HIGH);  // turn the LED on 
  delay(1000);                      // wait for a second
  digitalWrite(11, LOW);   // turn the LED off

  while (idx < traj_index) {
   currentMillis = millis();
    if ((unsigned long)(currentMillis - previousMillis) > timings[idx]) {
      idx++;
      previousMillis = currentMillis;
    }
    integerValue = trajectory[idx];
    pidcontroller1.setpoint(integerValue); // The "goal" the PID controller tries to "reach",
    pidcontroller2.setpoint(500-integerValue);
    Serial.print(integerValue); // print the incoming value for debugging
    motor_pwm_value = pidcontroller1.compute(encoder_count);  // Let the PID compute the value, returns the calculated optimal output
    motor2_pwm_value = pidcontroller2.compute(encoder_count2);

    Serial.print(", ");
    Serial.print(motor_pwm_value); // print the calculated value for debugging
    Serial.print(", ");
    Serial.print(encoder_count);// print the final encoder count.
    
    Serial.print(", ");
    Serial.print(motor2_pwm_value); 
    Serial.print(", ");
    Serial.print(encoder_count2);
    Serial.print(", ");

    ST.motor(1, motor_pwm_value); // using sabertooth library to do this
    ST.motor(2, motor2_pwm_value);

    Serial.println(1000000/usec); // prints out the sampling frequency (Hz)
    usec = 0;
  }
  ST.motor(1, 0); // stop motors from  moving at the end
  ST.motor(2, 0);
}

void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) { // if ENCODER_B is high increase the count
    encoder_count++; // increment the count
  } else {// else decrease the count
    encoder_count--;  // decrement the count
  }
}

void encoder2() {
  if (digitalRead(ENCODER_B2) == HIGH) // if ENCODER_B is high increase the count
    encoder_count2++; // increment the count
  else // else decrease the count
    encoder_count2--;  // decrement the count
}

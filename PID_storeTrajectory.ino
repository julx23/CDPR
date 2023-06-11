// PID from from https://circuitdigest.com/microcontroller-projects/arduino-based-encoder-motor-using-pid-controller
// motor control from sabertooth library

// inputted with sequence of goal positions and durations (in milliseconds)
// after that duration, it goes to the next goal. read until that line is finished

// send: m 0 200 50 200 100 200  150 200 200 200 150 200 100 200 50 200 0 200, then send g
// remember to re upload to restore to 0 when spooling

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 12 (D7) (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port. controls both motors

#include <PIDController.h>
/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pins
*/
// for the teensy 2.0 it says 5, 6, 7, 8 are interrupt pins and 11 shouldnt be used as its for LED
#define ENCODER_A 8
#define ENCODER_B 7
#define ENCODER_A2 6 // long cord one
#define ENCODER_B2 5
#define loadCellPin1 A5 // load cell
/*In this section we have defined the gain values for the
   proportional,integral, and derivative controller i have set
   the gain values with the help of trial and error methods.
*/
#define __Kp 4 // Proportional constant
#define __Ki 0 // Integral Constant
#define __Kd 5 // Derivative Constant
// 12, 0, 0 works, with large oscillations

#define MOTOR_LIMIT 45

volatile long int encoder_count = 0; // stores the current encoder count
volatile long int encoder_count2 = 500; // stores the second encoder's current count
int integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 127; // after PID computation data is stored in this variable.
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
  Serial.begin(115200); // Serial for Debugging CHANGE TO 9600??
  SWSerial.begin(9600); // sabertooth
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(ENCODER_A2, INPUT); // motor 2
  pinMode(ENCODER_B2, INPUT); 
  pinMode(11, OUTPUT); // IR LED
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2, RISING);
  pidcontroller1.begin(); // initialize the PID instance
  pidcontroller1.tune(__Kp, __Ki, __Kd); // Tune the PID, arguments: kP, kI, kD REPLACE with the variable names???
  pidcontroller1.limit(-MOTOR_LIMIT, MOTOR_LIMIT); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!

  pidcontroller2.begin(); // initialize the PID instance
  pidcontroller2.tune(__Kp, __Ki, __Kd); // Tune the PID, arguments: kP, kI, kD REPLACE with the variable names???
  pidcontroller2.limit(-MOTOR_LIMIT, MOTOR_LIMIT); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!


}
void loop() {
  while (Serial.available() > 0) {
    Serial.println("Reading");
    incomingByte = Serial.read(); // stores the /n character
    // Serial.println(incomingByte);
    if (incomingByte != 'g') {
      trajectory[traj_index] = Serial.parseInt(); // stores the integerValue
      timings[timings_index] = Serial.parseInt(); // stores the integerValue
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
  // if (integerValue == 1) { // stop
  //   ST.motor(1, 0);
  //   ST.motor(2, 0);
  //   delay(50000000000000); 
  //   // add to restart when another command comes in
  // }

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

  digitalWrite(11, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(11, LOW);   // turn the LED off by making the voltage LOW

  while (idx < traj_index) {
   currentMillis = millis();
    // Serial.println((unsigned long)(currentMillis - previousMillis));
    if ((unsigned long)(currentMillis - previousMillis) > timings[idx]) {
      // Serial.println("next goal");
      idx++;
      previousMillis = currentMillis;
    }
    integerValue = trajectory[idx];
    pidcontroller1.setpoint(integerValue); // The "goal" the PID controller tries to "reach",
    pidcontroller2.setpoint(500-integerValue);
    Serial.print(integerValue); // print the incoming value for debugging
    motor_pwm_value = pidcontroller1.compute(encoder_count);  //Let the PID compute the value, returns the calculated optimal output
    motor2_pwm_value = pidcontroller2.compute(encoder_count2);
    // the max function ensures no slack, it never "pushes", instead lets the other motor pod pull it

    // int motor_pwm_value_m = max(motor_pwm_value, -5);  //Let the PID compute the value, returns the calculated optimal output
    // int motor2_pwm_value_m = max(motor2_pwm_value, -5); 

    // Serial.print(motor_pwm_value); // print the calculated value for debugging
    Serial.print(", ");
    Serial.print(motor_pwm_value); // print the calculated value for debugging
    Serial.print(", ");
    Serial.print(encoder_count);// print the final encoder count.
    
    // Serial.print("                                                     m2: ");
    // Serial.print(motor2_pwm_value); // print the calculated value for debugging
    // Serial.print("      ");
    Serial.print(", ");
    Serial.print(motor2_pwm_value); // print the calculated value for debugging
    Serial.print(", ");
    Serial.print(encoder_count2);
    Serial.print(", ");

    // if (abs(motor_pwm_value) < 2) { //deadband, stabilising?
    //   ST.motor(1, 0); 
    // } else {
    ST.motor(1, motor_pwm_value); // using sabertooth library to do this
    // }
    ST.motor(2, motor2_pwm_value);

    // Serial.print("                                                                             Load cell: ");
    // Serial.print(analogRead(loadCellPin1));
    // Serial.print("    ");
    Serial.println(1000000/usec); // prints out the sampling frequency (Hz)
    usec = 0;
    // if(currentMillis - lastMillis > 1000){
    //   Serial.print("Loops last second:");
    //   Serial.println(loops);
      
    //   lastMillis = currentMillis;
    //   loops = 0;
    // }
  }
  ST.motor(1, 0); // using sabertooth library to do this
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
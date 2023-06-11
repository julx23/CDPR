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
#define ENCODER_A 7
#define ENCODER_B 8
// #define ENCODER_A2 5
// #define ENCODER_B2 6
// #define loadCellPin1 A5 // load cell

#define MOTOR_LIMIT 100

volatile long int encoder_count1 = 0; // stores the current encoder count
// volatile long int encoder_count2 = 500; // stores the second encoder's current count
int integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 127; // after PID computation data is stored in this variable.
// int motor2_pwm_value = 127;
int motor_1_commands[400]; // store the entire trajectory
// int motor_2_commands[400];
int motor_1_commands_index = 0;
// int motor_2_commands_index = 0;

elapsedMicros usec; 
unsigned long currentMillis;

void setup() {
  Serial.begin(115200); // Serial for Debugging CHANGE TO 9600??
  SWSerial.begin(9600); // sabertooth
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  // pinMode(ENCODER_A2, INPUT); // motor 2
  // pinMode(ENCODER_B2, INPUT); 
  pinMode(11, OUTPUT); // IR LED
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2, RISING);
}

void loop() {
  while (Serial.available() > 0) {
    Serial.println("Reading");
    incomingByte = Serial.read(); // stores the /n character
    // Serial.println(incomingByte);
    if (incomingByte != 'g') {
      motor_1_commands[motor_1_commands_index] = Serial.parseInt(); // stores the integerValue
      // motor_2_commands[motor_2_commands_index] = Serial.parseInt(); // stores the integerValue
      Serial.println(motor_1_commands[motor_1_commands_index]);
      // Serial.println(motor_2_commands[motor_2_commands_index]);
      motor_1_commands_index++;
      // motor_2_commands_index++;
    } else {
      Serial.println("Going");
      runTraj(motor_1_commands, motor_1_commands_index);
      memset(motor_1_commands, 0, sizeof(motor_1_commands));
      // memset(motor_2_commands, 0, sizeof(motor_2_commands));
      motor_1_commands_index = 0;
      // motor_2_commands_index = 0;
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

void runTraj(int motor_1_commands[], int length) {
  unsigned long previousMillis = 0;

  // Serial.println("motor_1_commands");
  // for(int i = 0; i < length; i++) {
  //   Serial.println(motor_1_commands[i]);
  // }
  // Serial.println("motor_2_commands");
  // for(int i = 0; i < length; i++) {
  //   Serial.println(motor_2_commands[i]);
  // }
  Serial.println("going now");
  int idx = -1;
  int interval = 50; // ms between commands
  int motor1_pwm_value = 0;
  // int motor2_pwm_value = 0;

  digitalWrite(11, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(11, LOW);   // turn the LED off by making the voltage LOW

  while (idx < length) {
   currentMillis = millis();
    // Serial.println((unsigned long)(currentMillis - previousMillis)); // sampling rate is 2.331 ms
    if ((unsigned long)(currentMillis - previousMillis) > interval) {
      // Serial.println("next goal");
      idx++;
      previousMillis = currentMillis;
      motor1_pwm_value = motor_1_commands[idx];
      // motor2_pwm_value = motor_2_commands[idx];
    }

    // the max function ensures no slack, it never "pushes", instead lets the other motor pod pull it
    // int motor_pwm_value_m = max(motor_pwm_value, -1);  //Let the PID compute the value, returns the calculated optimal output
    // int motor2_pwm_value_m = max(motor2_pwm_value, -1); 

    Serial.print(motor1_pwm_value); // the motor command
    Serial.print(",");
    Serial.print(encoder_count1);// the encoder count
    // Serial.print(",");
    // Serial.print(motor2_pwm_value); 
    // Serial.print(",");
    // Serial.print(encoder_count2);

    ST.motor(1, motor1_pwm_value); // using sabertooth library to do this
    // ST.motor(2, motor2_pwm_value);
    
    Serial.print(",");
    Serial.println(1000000/usec); // prints out the sampling frequency (Hz)
    usec = 0;
  }
  ST.motor(1, 0); // stop the motors at the end
  ST.motor(2, 0);
}

void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) { // if ENCODER_B is high increase the count
    encoder_count1++; // increment the count
  } else {// else decrease the count
    encoder_count1--;  // decrement the count
  }
}

// void encoder2() {
//   if (digitalRead(ENCODER_B2) == HIGH) // if ENCODER_B is high increase the count
//     encoder_count2++; // increment the count
//   else // else decrease the count
//     encoder_count2--;  // decrement the count
// }
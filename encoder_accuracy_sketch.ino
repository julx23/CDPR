// PID from from https://circuitdigest.com/microcontroller-projects/arduino-based-encoder-motor-using-pid-controller
// motor control from sabertooth library

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // TX on pin 12 (D7) (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port. controls both motors

#include <PIDController.h>
/* ENCODER_A and ENCODER_B pins are used to read the encoder
   data from the microcontroller, the data from the encoder
   comes very fast so these two pins have to be interrupt enabled
   pins
*/
#define ENCODER_A 7
#define ENCODER_B 8

#define MOTOR_LIMIT 100

volatile long int encoder_count1 = 0; // stores the current encoder count
int integerValue = 0; // stores the incoming serial value.
char incomingByte; // parses and stores each individual character one by one
int motor_pwm_value = 127; 
int motor_1_commands[400]; // store the entire trajectory
int motor_1_commands_index = 0;

elapsedMicros usec; 
unsigned long currentMillis;

void setup() {
  Serial.begin(115200); 
  SWSerial.begin(9600); // sabertooth
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(11, OUTPUT); // IR LED
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
}

void loop() {
  while (Serial.available() > 0) {
    Serial.println("Reading");
    incomingByte = Serial.read(); // stores the /n character
    if (incomingByte != 'g') {
      motor_1_commands[motor_1_commands_index] = Serial.parseInt(); 
      Serial.println(motor_1_commands[motor_1_commands_index]);
      motor_1_commands_index++;
    } else {
      Serial.println("Going");
      runTraj(motor_1_commands, motor_1_commands_index);
      memset(motor_1_commands, 0, sizeof(motor_1_commands));
      motor_1_commands_index = 0;
      Serial.println("reset");
      continue;
    }
  }
}

void runTraj(int motor_1_commands[], int length) {
  unsigned long previousMillis = 0;
  Serial.println("going now");
  int idx = -1;
  int interval = 50; // ms between commands
  int motor1_pwm_value = 0;

  digitalWrite(11, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(11, LOW);   // turn the LED off by making the voltage LOW

  while (idx < length) {
   currentMillis = millis();
    if ((unsigned long)(currentMillis - previousMillis) > interval) {
      idx++;
      previousMillis = currentMillis;
      motor1_pwm_value = motor_1_commands[idx];
    }

    Serial.print(motor1_pwm_value); // the motor command
    Serial.print(",");
    Serial.print(encoder_count1);// the encoder count

    ST.motor(1, motor1_pwm_value); // using sabertooth library to do this
    
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

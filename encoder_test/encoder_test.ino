const int PIN_ENCOD_A_MOTOR_LEFT = 15;               // A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 14;               // B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 16;              // A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 17;              // B channel for encoder of right motor 

volatile float pos_left = 0;       // Left motor encoder position
volatile float pos_right = 0;      // Right motor encoder position

volatile int interrupt_count_left = 0;  // Interrupt count for left motor
volatile int interrupt_count_right = 0; // Interrupt count for right motor

void setup() {
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT); // A_motor_1
  pinMode(3, OUTPUT); // A_motor_2
  pinMode(5, OUTPUT); // B_motor_1
  pinMode(6, OUTPUT); // B_motor_2
  pinMode(7, OUTPUT);

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH); // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

void loop() {
  analogWrite(2, 255);
  digitalWrite(4, 1);  
  digitalWrite(3, 0);

  analogWrite(5, 255);
  digitalWrite(6, 1);  
  digitalWrite(7, 0);

  // Print the interrupt counts for debugging
  Serial.print("Left Motor Interrupts: ");
  Serial.println(pos_left/300);
  Serial.print("Right Motor Interrupts: ");
  Serial.println(pos_right/300);

  delay(1000); // Adjust the delay as needed
}

void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left--;
  else pos_left++;
 // Increment the interrupt count for left motor
}

void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right++;
  else pos_right--;
}

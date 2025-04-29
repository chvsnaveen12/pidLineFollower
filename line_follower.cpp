// Motor driver connections
#define MOTOR_L_A   4
#define MOTOR_L_B   16
#define MOTOR_L_E   0
#define MOTOR_R_A   18
#define MOTOR_R_B   5
#define MOTOR_R_E   19
#define MOTOR_STDBY 17

// Sensor connections
#define ARRAY_0 36
#define ARRAY_1 39
#define ARRAY_2 27
#define ARRAY_3 26
#define ARRAY_4 25
#define ARRAY_5 33
#define ARRAY_6 32 
#define ARRAY_7 35
#define ARRAY_IR 13

// Calibration parameters
#define VALS 3000
#define DELAY 1000
#define AVG_VALS 128

// Motor parameter
#define REV_SPEED 70
#define BASE_SPEED 240
#define MAX_SPEED 255

// Calibration struct
typedef struct calibration {
  int vals[VALS];
  int min_val;
  int max_val;
} calibration_td;

calibration_td calibration_vals[8];

// PID parameters
float Kp = 100.0;
float Ki = 0.000;
float Kd = 100000.0;

// PID logic values
float integral = 0;
float lastError = 0;

// Sensor positions for error calculation
const int sensorPos[8] = {-4, -3, -3, 1, 1, 2, 3, 4};
int sensor[8];

// Sorting function
void bubbleSort(int arr[], int n) {
  for (int i = 0; i < n-1; ++i) {
    for (int j = 0; j < n-i-1; ++j) {
      if (arr[j] > arr[j+1]) {
        int temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
  }
}

// Setup
void setup() {
  // Set motor pins to outputs
  pinMode(MOTOR_L_A, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  pinMode(MOTOR_R_A, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);

  // Take the motor driver out of standby and enable both channels
  pinMode(MOTOR_L_E, OUTPUT);
  pinMode(MOTOR_R_E, OUTPUT);
  pinMode(MOTOR_STDBY, OUTPUT);
  digitalWrite(MOTOR_L_E, HIGH);
  digitalWrite(MOTOR_R_E, HIGH);
  digitalWrite(MOTOR_STDBY, HIGH);

  // Set the sensor pins as inputs
  pinMode(ARRAY_0, INPUT);
  pinMode(ARRAY_1, INPUT);
  pinMode(ARRAY_2, INPUT);
  pinMode(ARRAY_3, INPUT);
  pinMode(ARRAY_4, INPUT);
  pinMode(ARRAY_5, INPUT);
  pinMode(ARRAY_6, INPUT);
  pinMode(ARRAY_7, INPUT);
  
  // Turn on the IR sensor
  pinMode(ARRAY_IR, OUTPUT);
  digitalWrite(ARRAY_IR, HIGH);

  // Set the LED as output
  pinMode(LED_BUILTIN, OUTPUT);

  // Begin serial monitor
  Serial.begin(115200);
  Serial.println("Init Done");

  // FLASH LED before calibration
  for(int i = 0; i < 90; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
    delay(30);
  }
    digitalWrite(LED_BUILTIN, HIGH);

  // Calibration sequence
  Serial.println("Starting Calibration");

  // Start the motors to rotate the bot
  analogWrite(MOTOR_L_A, 0);
  analogWrite(MOTOR_R_B, 0);
  analogWrite(MOTOR_L_B, 200);
  analogWrite(MOTOR_R_A, 200);

  // Read the sensors
  for(int i = 0; i < VALS; i++){
      calibration_vals[0].vals[i] = analogRead(ARRAY_0);
      calibration_vals[1].vals[i] = analogRead(ARRAY_1);
      calibration_vals[2].vals[i] = analogRead(ARRAY_2);
      calibration_vals[3].vals[i] = analogRead(ARRAY_3);
      calibration_vals[4].vals[i] = analogRead(ARRAY_4);
      calibration_vals[5].vals[i] = analogRead(ARRAY_5);
      calibration_vals[6].vals[i] = analogRead(ARRAY_6);
      calibration_vals[7].vals[i] = analogRead(ARRAY_7);
    delayMicroseconds(DELAY);
  }

  // Turn the motors off
  analogWrite(MOTOR_L_B, 0);
  analogWrite(MOTOR_R_A, 0);

  // Figure out the min and max values for each sensor and print them
  for(int i = 0; i < 8; i++){
    bubbleSort(calibration_vals[i].vals, VALS);
    int sum_val = 0;
    for(int j = 0; j < AVG_VALS; j++)
      sum_val += calibration_vals[i].vals[j];
    calibration_vals[i].min_val = sum_val / AVG_VALS;
    sum_val = 0;
    for(int j = 0; j < AVG_VALS; j++)
      sum_val += calibration_vals[i].vals[VALS - j - 1];
    calibration_vals[i].max_val = sum_val / AVG_VALS;
    Serial.print("Sensor");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(calibration_vals[i].min_val);
    Serial.print(" - ");
    Serial.println(calibration_vals[i].max_val);
  }

  Serial.println("Calibration Done");
  
  // FLASH LED before start
  for(int i = 0; i < 90; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
    delay(30);
  }
}

// Reads all sensors and update sensor[] array
// Uncomment for debug
void read_sensors() {
//  Serial.print((float)(analogRead(ARRAY_0) - calibration_vals[0].min_val) / (float)(calibration_vals[0].max_val + calibration_vals[0].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_1) - calibration_vals[1].min_val) / (float)(calibration_vals[1].max_val + calibration_vals[1].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_2) - calibration_vals[2].min_val) / (float)(calibration_vals[2].max_val + calibration_vals[2].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_3) - calibration_vals[3].min_val) / (float)(calibration_vals[3].max_val + calibration_vals[3].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_4) - calibration_vals[4].min_val) / (float)(calibration_vals[4].max_val + calibration_vals[4].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_5) - calibration_vals[5].min_val) / (float)(calibration_vals[5].max_val + calibration_vals[5].min_val));
//  Serial.print(" - ");
//  Serial.print((float)(analogRead(ARRAY_6) - calibration_vals[6].min_val) / (float)(calibration_vals[6].max_val + calibration_vals[6].min_val));
//  Serial.print(" - ");
//  Serial.println((float)(analogRead(ARRAY_7) - calibration_vals[7].min_val) / (float)(calibration_vals[7].max_val + calibration_vals[7].min_val));
  sensor[0] = analogRead(ARRAY_0) > ((calibration_vals[0].min_val + calibration_vals[0].max_val) >> 1);
  sensor[1] = analogRead(ARRAY_1) > ((calibration_vals[1].min_val + calibration_vals[1].max_val) >> 1);
  sensor[2] = analogRead(ARRAY_2) > ((calibration_vals[2].min_val + calibration_vals[2].max_val) >> 1);
  sensor[3] = analogRead(ARRAY_3) > ((calibration_vals[3].min_val + calibration_vals[3].max_val) >> 1);
  sensor[4] = analogRead(ARRAY_4) > ((calibration_vals[4].min_val + calibration_vals[4].max_val) >> 1);
  sensor[5] = analogRead(ARRAY_5) > ((calibration_vals[5].min_val + calibration_vals[5].max_val) >> 1);
  sensor[6] = analogRead(ARRAY_6) > ((calibration_vals[6].min_val + calibration_vals[6].max_val) >> 1);
  sensor[7] = analogRead(ARRAY_7) > ((calibration_vals[7].min_val + calibration_vals[7].max_val) >> 1);
}

// Calculate weighted average error based on sensor positions
float calculateError() {
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensor[i]) {
      sum += sensorPos[i];
      count++;
    }
  }
  if (count == 0) {
    return lastError;
  }
  return (float)sum / count;
}

// Sets motor speeds (0-255 PWM) for both sides
// If speed is negative, rotate the motor in the oppsite direction
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -100000, 255);
  rightSpeed = constrain(rightSpeed, -100000, 255);
  if(leftSpeed < 0){
    analogWrite(MOTOR_L_A, REV_SPEED);
    analogWrite(MOTOR_L_B, 0);
  }
  else{
    analogWrite(MOTOR_L_B, leftSpeed);
    analogWrite(MOTOR_L_A, 0);
  }
  if(rightSpeed < 0){
    analogWrite(MOTOR_R_A, REV_SPEED);
    analogWrite(MOTOR_R_B, 0);
  }
  else{
    analogWrite(MOTOR_R_B, rightSpeed);
    analogWrite(MOTOR_R_A, 0);
  }
}

// Main loop
void loop() {
  // Read the sensors and calculate the error
  read_sensors();
  float error = calculateError();

  // PID calculations
  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Set the motor speeds
  int leftSpeed = BASE_SPEED  + correction;
  int rightSpeed = BASE_SPEED - correction;
  setMotors(leftSpeed, rightSpeed);


  // Print debug info
  Serial.print("Error: "); Serial.print(error);
  Serial.print("\tCorrection: "); Serial.print(correction);
  Serial.print("\tL: "); Serial.print(leftSpeed);
  Serial.print("\tR: "); Serial.println(rightSpeed);

  // Attaining a refresh rate of 100Hz
  delay(10);
}

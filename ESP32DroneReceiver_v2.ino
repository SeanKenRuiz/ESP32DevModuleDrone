// Creator: Sean Ruiz
// File Name: ESP32DroneReceiver
// Including code from:
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
*/
/*
  Carbon Aeronautic - Build and program your own Arduino drone
*/

// -- Libraries --

// ESP32 wireless communication
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xa0, 0xb7, 0x65, 0xdd, 0xf3, 0x60};

// -- Receiving data from transmitter ESP32 --

// Structure to receive data
// Must match the sender structure
typedef struct struct_message {
  int throttle_x_adc;
  int throttle_y_adc;
  int pitch_x_adc;
  int pitch_y_adc;
} struct_message;

// Create a struct_message called myData
struct_message myData;

typedef struct struct_onboardData {
  int Motor_1;
  int Motor_2;
  int Motor_3;
  int Motor_4;
  float AnglePitch;
  float PIDOutputPitch;
  float ErrorPitch;
} struct_onboardData;

// onboardData struct_message 
struct_onboardData onboardData;

esp_now_peer_info_t peerInfo;


// callback function that will be executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t) {
  //Serial.print("Motor 1 (TL):");
  //Serial.print("Motor 2 (TR):");
  //Serial.print("Motor 3 (BL):");
  //Serial.print("Motor 4 (BR):");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  /*
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("LEFT X: ");
  Serial.println(myData.throttle_x_adc);
  Serial.print("LEFT Y: ");
  Serial.println(myData.throttle_y_adc);
  Serial.print("RIGHT X: ");
  Serial.println(myData.pitch_x_adc);
  Serial.print("RIGHT Y: ");
  Serial.println(myData.pitch_y_adc);
  Serial.println();
  */
}

// ESP32 library to interface with drone motors
#include <ESP32Servo.h>

// I2C communication
#include <Wire.h>

// Gyro + Acceleratometer Variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer; // variable to track loop times

// Define predicted angles and the uncertainties
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0, 0};

// Outer-loop PID Controller Variables
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PAngleRoll=0; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

// function to calculate the predicted angle and uncretainty using Kalman equations
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Predict the current state
  KalmanState = KalmanState + 0.004 * KalmanInput;
  // Calculate the uncertainty
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  // Calculate the Kalman gain from the uncertainties on predictions and measurements
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  // Update the predicted state of the system with measurements of the state through the Kalman gain
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  // Update the uncertainty of predicted state 
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// Inner-loop PID Controller Variables
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0,0,0};

float PRateRoll = 2.5; float PRatePitch = PRateRoll; float PRateYaw = 0;
float IRateRoll = 0; float IRatePitch = IRateRoll; float IRateYaw = 0.0;
float DRateRoll = 0; float DRatePitch = DRateRoll; float DRateYaw = 0.0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Interfacing with MPU6050's gyroscope
void gyro_signals(void){
  // turn on low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure acclerometer output
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Pull the accelerometer measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();
  
  // Configure the gyroscope and pull rotation rate measurements from sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  // Rotate 90 degrees clockwise to account for the fact that MPU6050 isn't aligned with front of drone
  // int16_t ax =  AccYLSB;
  // int16_t ay = -AccXLSB;
  // int16_t az =  AccZLSB;

  // int16_t gx =  GyroY;
  // int16_t gy = -GyroX;
  // int16_t gz =  GyroZ;  

  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  // REMEMBER TO CALIBRATE OWN ACCELEROMETER VALUES HERE 14
  // AccX = (float)AccXLSB / 4096 - 0.06;
  // AccY = (float)AccYLSB / 4096 + 0.16;
  // AccZ = (float)AccZLSB / 4096 - 0.04;

  AccX = (float)AccXLSB / 4096 - 0.06;
  AccY = (float)AccYLSB / 4096 + 0.16;
  AccZ = (float)AccZLSB / 4096 - 0.04;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ)) * (1 / (3.142/180));
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ)) * (1 / (3.142/180));
}

// PID function
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm = P*Error;
  float Iterm = PrevIterm + I*(Error + PrevError) * 0.004/2;  // 0.004 s = 4 ms = 250 hz
  
  if(Iterm > 400){  // To avoid integral wind-up, limit ITerm to 400 ms
    Iterm = 400;
  } else if (Iterm < -400) Iterm = -400;

  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  
  if (PIDOutput > 400) {  // To avoid integral wind-up, limit PIDOutput to 400 ms
    PIDOutput = 400;
  } else if (PIDOutput < -400) {
    PIDOutput = -400;
  }

  // Return the output from the PID function
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {  // PID reset function when motors turned off
  PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;

  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}


///////////////////////////////////

Servo topLeft;
Servo topRight;
Servo bottomRight;
Servo bottomLeft;  

// ESP32 MOTOR PINS
int motor3 = 32;  // top left working
int motor4 = 33;  // top right
int motor2 = 18;  // bottom left
int motor1 = 19;   // bottom right working

// int motor1 = 32;  // top left working
// int motor2 = 33;  // top right
// int motor3 = 18;  // bottom left
// int motor4 = 19;   // bottom right working

int setMicroseconds;
int prevMicroseconds = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // WIFI SETUP
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // GYROSCOPE SETUP
  Wire.begin(21, 22, 400000);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyroscope calibration (LAY QUADCOPTER ON FLAT-SURFACE & DO NOT TOUCH)
  for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    // Serial.println("Failed to add peer");
    return;
  }

  // MOTOR PIN SETUP
  topLeft.attach(motor1); // CCW
  topRight.attach(motor2);  // CW
  bottomLeft.attach(motor3); // CW
  bottomRight.attach(motor4); // CCW

  topLeft.writeMicroseconds(2000);
  topRight.writeMicroseconds(2000);
  bottomLeft.writeMicroseconds(2000);
  bottomRight.writeMicroseconds(2000);
  //Serial.begin(115200);

  LoopTimer = micros();
}

void loop() {
  // For testing purposes: changing speed with serial monitor
  /*
  if(Serial.available())
  {
    int microsecondsInput = Serial.parseInt(); // ONE MOTOR: 1000 to 2000 Microseconds!!! ANOTHER MOTOR: 1500 to 2000 (START 1585)

    if(prevMicroseconds != microsecondsInput && !(microsecondsInput == 0))
    {
      setMicroseconds = microsecondsInput;
    }

    myServo.writeMicroseconds(setMicroseconds);
    prevMicroseconds = microsecondsInput;
  }
  */

  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Start the iteration for the Kalman filter with the roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  /* // Kalman Filter TEST
  Serial.print("RollAngle[deg]:");
  Serial.print(KalmanAngleRoll);
  Serial.print(", PitchAngle[deg]:");
  Serial.println(KalmanAnglePitch);
  */

  // Calculate the desired angle roll, pitch, and yaw rates
  DesiredAngleRoll = 0.10 * (myData.pitch_x_adc - 1500);
  DesiredAnglePitch = 0.10 * (myData.pitch_y_adc - 1500);
  InputThrottle = myData.throttle_y_adc;
  DesiredRateYaw = 0.15 * (myData.throttle_y_adc - 1500);
  
  // Calculate the errors for the PID calculations
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorRatePitch = DesiredAnglePitch - DesiredAnglePitch;
  
  // Execute outer-loop PID calculations
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];


  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  // Execute inner-loop PID calculations
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch=PIDReturn[0]; 
  PrevErrorRatePitch=PIDReturn[1]; 
  PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw=PIDReturn[0]; 
  PrevErrorRateYaw=PIDReturn[1]; 
  PrevItermRateYaw=PIDReturn[2];
  



  // Throttle output limiter
  if(InputThrottle > 1800) {
    InputThrottle = 1800;
  }

  // Quadcopter dynamics
  MotorInput1 = 1.024*(InputThrottle - InputRoll + InputPitch - InputYaw); // TR
  MotorInput2 = 1.024*(InputThrottle - InputRoll - InputPitch + InputYaw); // BR
  MotorInput3 = 1.024*(InputThrottle + InputRoll - InputPitch - InputYaw); // BL
  MotorInput4 = 1.024*(InputThrottle + InputRoll + InputPitch + InputYaw); // TL


  // Motor max limiter
  if(MotorInput1 > 2000) {
    MotorInput1 = 1999;
  }
  if(MotorInput2 > 2000) {
    MotorInput2 = 1999;
  }
  if(MotorInput3 > 2000) {
    MotorInput3 = 1999;
  }
  if(MotorInput4 > 2000) {
    MotorInput4 = 1999;
  }

  // Keep quadcopters motors running at 18% during expected flight
  int ThrottleIdle = 1180;
  if(MotorInput1 < ThrottleIdle) {
    MotorInput1 = ThrottleIdle;
  }
  if(MotorInput2 < ThrottleIdle) {
    MotorInput2 = ThrottleIdle;
  }
  if(MotorInput3 < ThrottleIdle) {
    MotorInput3 = ThrottleIdle;
  }
  if(MotorInput4 < ThrottleIdle) {
    MotorInput4 = ThrottleIdle;
  }

  // Allow for motors to turn off when throttle at lowest position
  int ThrottleCutOff = 1000;
  if(myData.throttle_y_adc < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  // write to motors
  topLeft.writeMicroseconds(MotorInput1);
  topRight.writeMicroseconds(MotorInput2);
  bottomLeft.writeMicroseconds(MotorInput3);
  bottomRight.writeMicroseconds(MotorInput4);
  
  //Motor input
  // Serial.print("Motor1:");
  // Serial.print(MotorInput1);
  // Serial.print(", Motor2:");
  // Serial.print(MotorInput2);
  // Serial.print(", Motor3:");
  // Serial.print(MotorInput3);
  // Serial.print(", Motor4:");
  // Serial.println(MotorInput4);

  // // PID readings
  // Serial.print(GyroX);
  // Serial.print(GyroY);
  // Serial.print(GyroZ);
  
  // Initializing variables to send back to controller
  onboardData.Motor_1 = KalmanAngleRoll;
  onboardData.Motor_2 = KalmanAnglePitch;
  onboardData.Motor_3 = MotorInput3;
  onboardData.Motor_4 = MotorInput4;


  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &onboardData, sizeof(onboardData));

  // wait for 4 ms and finish the 250 Hz control loop to proceed to next iteration
  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();

  // REMEMBER TO PUT THROTTLE TO MAX WHEN PLUGGING BATTERY IN
  // ONE MOTOR: 1000 to 2000 Microseconds!!!
  
  // LEFT X: 1790, min 4095, max 0
  // LWFT Y: 1790, min 0, max 4095

  // RIGHT X: 1760 min 0, max 4095
  // RIGHT Y: 0-4095, mid 1760, 0 "max", 4095 "min"

  

}

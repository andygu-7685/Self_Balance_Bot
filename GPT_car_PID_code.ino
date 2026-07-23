#include <AccelStepper.h>
#include <AltSoftSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"



//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Direction Convention:
// direction toward battery opening is backward
// positive speed rotate the wheel CCW
//---------------------------------------------------------------------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Macros and Compile Options
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
// #define MPU_VERBOSE
// #define DEBUG

inline int fast_round(float x) {
  return (x >= 0) ? (int)(x + 0.5) : (int)(x - 0.5);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Initialize Object and Timing
//---------------------------------------------------------------------------------------------------------------------------------------------------------

MPU6050 mpu;
AccelStepper stepperR(AccelStepper::DRIVER, 6, 5);
AccelStepper stepperL(AccelStepper::DRIVER, 11, 10);
// Pins are fixed: RX=8, TX=9
AltSoftSerial altSerial;

unsigned long lastPrintTime = 0;
unsigned long lastSetSpeedTime = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// MPU6050 Variables
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const float ACCEL_FACTOR = 9.80665 / 16384;
const float ROT_FACTOR = 1.0 / 131.0;
int16_t OFFSET_AX = 0;
int16_t OFFSET_AY = 0;
int16_t OFFSET_AZ = 0;
#ifdef MPU_VERBOSE
int16_t OFFSET_GX = 0;
int16_t OFFSET_GY = 0;
int16_t OFFSET_GZ = 0;
#endif
int8_t Z_DIR = 1;                     // 1 if mpu is upright, -1 if upside down

int16_t ax, ay, az;
int16_t gx, gy, gz;
float _ax, _ay, _az;
#ifdef MPU_VERBOSE
float _gx, _gy, _gz;
#endif

const uint8_t INTERRUPT_PIN = 2;      // Use Pin 2 as Interrupt 0, only 2 or 3 can be interrupt
volatile bool mpuInterrupt = false;   // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Movement and UART
//---------------------------------------------------------------------------------------------------------------------------------------------------------

// 1 = forward, 2 = backward, 3 = left, 4 = right
uint8_t move_dir = 0;
// ~500 for some speed
// must be signed or else the calculated speed would underflow
// due to implicit conversion uint16_t + int16_t = uint16_t
int16_t move_bias = 0;
// move time in seconds
uint8_t move_time = 0;
// bias sign
int8_t bias_sgn_L = 0;
int8_t bias_sgn_R = 0;

unsigned long lastCmdTime = 0;

// max number of char is 10 in a message
const uint8_t numChars = 10;
char receivedChars[numChars];
bool newData = false;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Proportional Control (Balance Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const uint8_t bal_err_q_size = 5;
#ifdef MPU_VERBOSE
int16_t bal_err_q_x[bal_err_q_size] = {0, 0, 0, 0, 0};
#endif
int16_t bal_err_q_y[bal_err_q_size] = {0, 0, 0, 0, 0};
int16_t bal_err_q_z[bal_err_q_size] = {0, 0, 0, 0, 0};
uint8_t bal_err_q_ctr = 0;
float bal_K_a = 0.05;
float bal_K_p = 0.5 * 0.2 * 3.5;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Derivative Control (Balance Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const uint8_t bal_avg_q_size = 5;
int16_t bal_avg_err_q[bal_avg_q_size] = {0, 0, 0, 0, 0};
uint8_t bal_dt_q[bal_avg_q_size] = {0, 0, 0, 0, 0};
uint8_t bal_avg_q_ctr = 0;
unsigned long bal_last_time = 0;
float bal_K_d = 0.3 * 0.2 * 1.0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Proportional Control (Speed Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

// maintain speed correction that changes every 50-100ms
int16_t spd_correction_L = 0;
int16_t spd_correction_R = 0;

int16_t spd_last_pos_L = 0;
int16_t spd_last_pos_R = 0;
float spd_K_p = 0;
unsigned long spd_last_time = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Integral Control (Speed Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

float spd_K_i = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Setup
//---------------------------------------------------------------------------------------------------------------------------------------------------------



void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  altSerial.begin(9600);
  // Wait for serial port to connect
  while (!Serial);

  // Initialize I2C communication
  Wire.begin();
  // fast Clock allow fast MPU processing
  Wire.setClock(400000);
  // prevent stall from missing MPU packet
  Wire.setWireTimeout(3000, true);

  // Initialize the MPU6050
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  // slows the MPU interrupt down to 100 times per second
  // mpu.setRate(9);        

  // Verify the connection
  Serial.println("Testing device connections...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed. Please check your wiring!");
    while (1); // Halt program if connection fails
  }

  // Initialize the AccelStepper
  stepperR.setPinsInverted(false, false, true);
  stepperR.setEnablePin(7);
  stepperR.enableOutputs();
  stepperR.setMinPulseWidth(50);
  stepperR.setMaxSpeed(3000);
  stepperR.setCurrentPosition(0);

  stepperL.setPinsInverted(false, false, true);
  stepperL.setEnablePin(12);
  stepperL.enableOutputs();
  stepperL.setMinPulseWidth(50);
  stepperL.setMaxSpeed(3000);
  stepperR.setCurrentPosition(0);

  // Set the interrupt pin of the MPU6050
  pinMode(INTERRUPT_PIN, INPUT);
  // Enable Data Ready interrupt on the MPU6050
  mpu.setIntDataReadyEnabled(true);
  // Attach the Arduino interrupt to Pin 2, looking for a RISING edge
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  // MPU6050 calibration
  MPU_calibration();

  bal_last_time = millis();
  spd_last_time = millis();
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Main Loop
//---------------------------------------------------------------------------------------------------------------------------------------------------------



void loop() {

  if (mpuInterrupt) {
    int16_t net_speed_L = 0;
    int16_t net_speed_R = 0;
    mpuInterrupt = false;
    runSpeed();
    
    // IMPORTANT: This clears the hardware interrupt pin on the MPU6050!
    // Without this, the physical pin stays HIGH and never triggers another RISING edge.
    {
      uint8_t mpuIntStatus = mpu.getIntStatus();
      runSpeed();
    }

    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    // MPU6050 measurements
    //---------------------------------------------------------------------------------------------------------------------------------------------------------

    // Read raw accelerometer and gyroscope measurements
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long curr_time = millis();
    runSpeed();

    uint8_t bal_delta_time = min(curr_time - bal_last_time, 200);
    bal_last_time = curr_time;
    runSpeed();

    // unit in m/s^2
    _ax = (ax+OFFSET_AX) * ACCEL_FACTOR;
    _ay = (ay+OFFSET_AY) * ACCEL_FACTOR;
    _az = (az+OFFSET_AZ) * ACCEL_FACTOR;
    #ifdef MPU_VERBOSE
    // CCW->+ive, CW->-ive, unit in degree per second
    _gx = (gx+OFFSET_GX) * ROT_FACTOR;
    _gy = (gy+OFFSET_GY) * ROT_FACTOR;
    _gz = (gz+OFFSET_GZ) * ROT_FACTOR;
    #endif
    runSpeed();

    #ifdef MPU_VERBOSE
    bal_err_q_x[bal_err_q_ctr] = (ax + OFFSET_AX);
    #endif
    bal_err_q_y[bal_err_q_ctr] = (ay + OFFSET_AY);
    bal_err_q_z[bal_err_q_ctr] = (az + OFFSET_AZ) - 16384 * Z_DIR;
    (bal_err_q_ctr >= bal_err_q_size - 1) ? bal_err_q_ctr = 0 : bal_err_q_ctr++;
    runSpeed();



    {
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Proportional Control (Balance Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      int32_t sum = 0;
      {
        int16_t *ptr = bal_err_q_y;
        int16_t *end = bal_err_q_y + bal_err_q_size;
        while (ptr < end) sum += *ptr++;
        runSpeed();
      }
      // +ive = CCW->speed +ive, -ive = CW->speed -ive
      float avg_error = (float)(sum / bal_err_q_size);
      int32_t _p = fast_round(avg_error * bal_K_p);
      if(_p < 300 * bal_K_p && _p > -300 * bal_K_p) _p = 0;
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Acceleration Control (Commented Out)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      
      // // acceleration term
      // sum = 0;
      // {
      //   int16_t *ptr = bal_err_q_z;
      //   int16_t *end = bal_err_q_z + bal_err_q_size;
      //   while (ptr < end) sum += *ptr++;
      //   runSpeed();
      // }
      // // +ive = downward_accel, -ive = upward_accel
      // int16_t avg_error_z = sum / bal_err_q_size;
      // int16_t _a = avg_error_z * bal_K_a;
      // if(_a < 400 * bal_K_a && _a > -400 * bal_K_a) _a = 0;
      // runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Derivative Control (Balance Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      bal_avg_err_q[bal_avg_q_ctr] = avg_error;
      bal_dt_q[bal_avg_q_ctr] = bal_delta_time;
      (bal_avg_q_ctr >= bal_avg_q_size - 1) ? bal_avg_q_ctr = 0 : bal_avg_q_ctr++;
      runSpeed();

      uint8_t last_error_index = (bal_avg_q_ctr + 1) % bal_avg_q_size;
      // +ive = moving CCW, -ive = moving CW
      int16_t delta_error = avg_error - bal_avg_err_q[last_error_index];
      uint8_t elapsed_time = 0;
      runSpeed();
      
      {
        uint8_t *ptr = bal_dt_q;
        uint8_t *end = bal_dt_q + bal_avg_q_size;
        while (ptr < end) elapsed_time += *ptr++;
        elapsed_time -= bal_dt_q[last_error_index];
        runSpeed();
      }

      int16_t _d = fast_round(delta_error * bal_K_d / elapsed_time);
      if(_d < 2 && _d > -2) _d = 0;
      runSpeed();

      net_speed_L += _p - sgn(_p) * abs(_d);
      net_speed_R += _p - sgn(_p) * abs(_d);
      runSpeed();
    }



    curr_time = millis();
    if(curr_time - spd_last_time >= 80){

      int16_t curr_pos_L = stepperL.currentPosition();
      int16_t curr_pos_R = stepperR.currentPosition();
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Proportional Control (Speed Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      int16_t delta_pos_L = curr_pos_L - spd_last_pos_L;
      int16_t delta_pos_R = curr_pos_R - spd_last_pos_R;
      int16_t spd_delta_time = curr_time - spd_last_time;
      spd_last_pos_L = curr_pos_L;
      spd_last_pos_R = curr_pos_R;
      spd_last_time = curr_time;
      runSpeed();

      int16_t _p_L = fast_round((delta_pos_L / spd_delta_time) * spd_K_p);
      int16_t _p_R = fast_round((delta_pos_R / spd_delta_time) * spd_K_p);
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Integral Control (Speed Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      int16_t _i_L = fast_round(curr_pos_L * spd_K_i);
      int16_t _i_R = fast_round(curr_pos_R * spd_K_i);
      runSpeed();

      spd_correction_L = -_p_L - _i_L;
      spd_correction_R = -_p_R - _i_R;
    }



    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Set Motor Speeds
    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    
    net_speed_L += spd_correction_L + bias_sgn_L * move_bias;
    net_speed_R += spd_correction_R + bias_sgn_R * move_bias;

    if(millis() - lastSetSpeedTime >= 15){
      lastSetSpeedTime = millis();
      stepperL.setSpeed(net_speed_L);
      stepperR.setSpeed(net_speed_R);
    }
    runSpeed();

    #ifdef DEBUG
    if(millis() - lastPrintTime >= 500){
      lastPrintTime = millis();
      //Print the values to the Serial Monitor
      Serial.print("a:\t");
      Serial.print(_ax); Serial.print("\t");
      Serial.print(_ay); Serial.print("\t");
      Serial.println(_az);
      #ifdef MPU_VERBOSE
      Serial.print("g:\t");
      Serial.print(_gx); Serial.print("\t");
      Serial.print(_gy); Serial.print("\t");
      Serial.println(_gz);
      #endif
      Serial.print(_p);
      Serial.print(" ||| ");
      Serial.print(_d);
      Serial.print(" ||| ");
      // Serial.print(_a);
      // Serial.print(" ||| ");
      Serial.print(bias_sgn_R * move_bias);
      Serial.print(" ||| ");
      Serial.print(bias_sgn_L * move_bias);
      Serial.print(" ||| ");
      Serial.print(_p - sgn(_p) * abs(_d) + bias_sgn_R * move_bias);
      Serial.print(" ||| ");
      Serial.print(_p - sgn(_p) * abs(_d) + bias_sgn_L * move_bias);
      Serial.print(" ||| ");
      Serial.print(stepperL.speed());
      Serial.print(" ||| ");
      Serial.println(stepperR.speed());
    }
    #endif
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------------------
  // UART
  //---------------------------------------------------------------------------------------------------------------------------------------------------------

  receiveData();
  runSpeed();

  if (newData == true) {
    altSerial.println(receivedChars);
    runSpeed();

    update_cmd();
    runSpeed();

    #ifdef DEBUG
    Serial.print("Pico message: ");
    Serial.println(receivedChars);
    Serial.print(move_dir);
    Serial.print(" // ");
    Serial.print(move_bias);
    Serial.print(" // ");
    Serial.print(move_time);
    Serial.print(" // ");
    Serial.print(bias_sgn_L);
    Serial.print(" // ");
    Serial.println(bias_sgn_R);
    #endif

    newData = false;
  }
  runSpeed();

  //---------------------------------------------------------------------------------------------------------------------------------------------------------
  // Movement Timer
  //---------------------------------------------------------------------------------------------------------------------------------------------------------

  if (move_dir != 0 && (millis() - lastCmdTime) / 1000.0 > move_time) {
    move_dir = 0;
    move_bias = 0;
    move_time = 0;
    bias_sgn_L = 0;
    bias_sgn_R = 0;
  }
  runSpeed();
  
}



void runSpeed(){
  stepperR.runSpeed();
  stepperL.runSpeed();
}



void update_cmd(){
  bool set_Kp = false;
  bool set_Kd = false;

  if(!receivedChars ||
     receivedChars[0] == '\0' || receivedChars[1] == '\0' ||
     receivedChars[2] == '\0' || receivedChars[1] != ':'){
     move_dir = 0;
     move_bias = 0;
     move_time = 0;
     bias_sgn_L = 0;
     bias_sgn_R = 0;
     return;
  }
  runSpeed();

  switch(receivedChars[0]){
    case 'b':
      move_dir = 1;
      break;
    case 'l':
      move_dir = 2;
      break;
    case 'r':
      move_dir = 3;
      break;
    case 'f':
      move_dir = 4;
      break;
    case 'p':
      set_Kp = true;
      break;
    case 'd':
      set_Kd = true;
      break;
    default:
      move_dir = 0;
      move_bias = 0;
      move_time = 0;
      bias_sgn_L = 0;
      bias_sgn_R = 0;
      return;
  }
  runSpeed();

  char* numStr = &receivedChars[2];
  char* endPtr = NULL;

  if(set_Kp){
    bal_K_p = static_cast<uint8_t>(strtol(numStr, &endPtr, 10)) / 100.0;
  }
  else if(set_Kd){
    bal_K_d = static_cast<uint8_t>(strtol(numStr, &endPtr, 10)) / 100.0;
  }
  else{
    move_time = static_cast<uint8_t>(strtol(numStr, &endPtr, 10));
    if(endPtr && *endPtr != '\0'){
        move_dir = 0;
        move_bias = 0;
        move_time = 0;
        bias_sgn_L = 0;
        bias_sgn_R = 0;
        return;
    }

    move_bias = 200;
    bias_sgn_L = (int8_t)((move_dir - 1) / 2) * 2 - 1;
    bias_sgn_R = (int8_t)((move_dir - 1) % 2) * 2 - 1;
    lastCmdTime = millis();
  }
  runSpeed();
}



void receiveData() {
  static uint8_t index = 0;
  char endMarker = '\n';
  char rc;
  runSpeed();

  // AltSoftSerial handles timing in the background via hardware timers
  while (altSerial.available() > 0 && newData == false) {
    rc = altSerial.read();
    runSpeed();

    if (rc != endMarker) {
      receivedChars[index] = rc;
      index++;
      if (index >= numChars) index = numChars - 1;
    }
    else {
      receivedChars[index] = '\0'; // Terminate string
      index = 0;
      newData = true;
    }
  }
}



void MPU_calibration(){
  const uint16_t samples = 100;
  const uint16_t totalWindowMs = 2000;
  const uint16_t samplePeriodMs = totalWindowMs / samples;

  int32_t sum_ax = 0;
  int32_t sum_ay = 0;
  int32_t sum_az = 0;
  #ifdef MPU_VERBOSE
  int32_t sum_gx = 0;
  int32_t sum_gy = 0;
  int32_t sum_gz = 0;
  #endif

  Serial.println("\nStarting MPU calibration: keep sensor still and level...");
  delay(500);

  unsigned long nextSampleAt = millis();
  for (uint16_t i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    #ifdef MPU_VERBOSE
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    #endif

    nextSampleAt += samplePeriodMs;
    while ((long)(nextSampleAt - millis()) > 0) {
      // Wait to keep a stable sampling period.
    }
  }

  int16_t avg_ax = (int16_t)(sum_ax / samples);
  int16_t avg_ay = (int16_t)(sum_ay / samples);
  int16_t avg_az = (int16_t)(sum_az / samples);
  #ifdef MPU_VERBOSE
  int16_t avg_gx = (int16_t)(sum_gx / samples);
  int16_t avg_gy = (int16_t)(sum_gy / samples);
  int16_t avg_gz = (int16_t)(sum_gz / samples);
  #endif

  OFFSET_AX = -avg_ax;
  OFFSET_AY = -avg_ay;
  OFFSET_AZ = -avg_az + 16384 * Z_DIR;
  #ifdef MPU_VERBOSE
  OFFSET_GX = -avg_gx;
  OFFSET_GY = -avg_gy;
  OFFSET_GZ = -avg_gz;
  #endif

  Serial.println("MPU calibration complete. Offsets:");
  Serial.print("AX: "); Serial.print(OFFSET_AX); Serial.print("\t");
  Serial.print("AY: "); Serial.print(OFFSET_AY); Serial.print("\t");
  Serial.print("AZ: "); Serial.print(OFFSET_AZ); Serial.print("\t");
  #ifdef MPU_VERBOSE
  Serial.print("GX: "); Serial.print(OFFSET_GX); Serial.print("\t");
  Serial.print("GY: "); Serial.print(OFFSET_GY); Serial.print("\t");
  Serial.print("GZ: "); Serial.println(OFFSET_GZ);
  #endif
}
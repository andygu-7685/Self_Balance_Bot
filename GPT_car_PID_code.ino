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
//p=100
//d=5
//s=-30000
//i=55


//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Macros and Compile Options
//---------------------------------------------------------------------------------------------------------------------------------------------------------

#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
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

#ifdef DEBUG
unsigned long lastPrintTime1 = 0;
unsigned long lastPrintTime2 = 0;
unsigned long lastPrintTime3 = 0;
#endif
unsigned long lastSetSpeedTime = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// MPU6050 Variables
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const float ACCEL_FACTOR = 9.80665 / 16384;
const float ROT_FACTOR = 1.0 / 131.0;
int16_t OFFSET_AX = 0;
int16_t OFFSET_AY = 0;
int16_t OFFSET_AZ = 0;
int16_t OFFSET_GX = 0;
int16_t OFFSET_GY = 0;
int16_t OFFSET_GZ = 0;
const int8_t Z_DIR = 1;                     // 1 if mpu is upright, -1 if upside down

int16_t ax, ay, az;
int16_t gx, gy, gz;
float _ax, _ay, _az;
float _gx, _gy, _gz;

const uint8_t INTERRUPT_PIN = 2;      // Use Pin 2 as Interrupt 0, only 2 or 3 can be interrupt
volatile bool mpuInterrupt = false;   // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Movement and UART
//---------------------------------------------------------------------------------------------------------------------------------------------------------

// 1 = backward, 2 = left, 3 = right, 4 = forward
uint8_t move_dir = 0;
unsigned long last_cmd_time = 0;

// max number of char is 10 in a message
const uint8_t numChars = 10;
char receivedChars[numChars];
bool newData = false;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Proportional Control (Balance Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

// angle of tilt in degree * 100
int16_t angle = 0;
const uint8_t bal_q_size = 5;
int16_t bal_acc_q_y[bal_q_size] = {0, 0, 0, 0, 0};
int16_t bal_acc_q_z[bal_q_size] = {0, 0, 0, 0, 0};
int16_t bal_gyr_q_x[bal_q_size] = {0, 0, 0, 0, 0};
uint8_t bal_q_ctr = 0;
float bal_K_p = 2.0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Derivative Control (Balance Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const uint8_t bal_avg_q_size = 5;
int16_t bal_avg_err_q[bal_avg_q_size] = {0, 0, 0, 0, 0};
uint8_t bal_avg_q_ctr = 0;
float bal_K_d = 15.0;

uint8_t delta_time_q[bal_avg_q_size] = {0, 0, 0, 0, 0};
unsigned long last_delta_time = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Proportional Control (Speed Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

int16_t spd_last_pos_L = 0;
int16_t spd_last_pos_R = 0;
float spd_K_p = -300.0;
unsigned long spd_last_time = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Integral Control (Speed Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

float spd_K_i = 1.1;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Proportional Control (Steer Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

// expected steer speed in degree per second
int8_t bias_steer_spd = 10;
int16_t bias_mag = 0;
// 1 = backward, left; -1 = forward, right
int8_t bias_sgn = 0;

const uint8_t steer_q_size = bal_avg_q_size;
int16_t steer_gyr_q_z[steer_q_size] = {0, 0, 0, 0, 0};
uint8_t steer_q_ctr = 0;
float steer_K_p = 1.0;

//---------------------------------------------------------------------------------------------------------------------------------------------------------
// Integral Control (Steer Loop)
//---------------------------------------------------------------------------------------------------------------------------------------------------------

const uint8_t steer_avg_window = 3;
int16_t steer_net_angle = 0;
float steer_K_i = 0.05;

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
  stepperR.setMaxSpeed(10000);
  stepperR.setCurrentPosition(0);

  stepperL.setPinsInverted(false, false, true);
  stepperL.setEnablePin(12);
  stepperL.enableOutputs();
  stepperL.setMinPulseWidth(50);
  stepperL.setMaxSpeed(10000);
  stepperL.setCurrentPosition(0);

  // Set the interrupt pin of the MPU6050
  pinMode(INTERRUPT_PIN, INPUT);
  // Enable Data Ready interrupt on the MPU6050
  mpu.setIntDataReadyEnabled(true);
  // Attach the Arduino interrupt to Pin 2, looking for a RISING edge
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  // MPU6050 calibration
  MPU_calibration();

  last_delta_time = millis();
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

    uint8_t delta_time = min(curr_time - last_delta_time, 200);
    last_delta_time = curr_time;

    // unit in m/s^2
    _ax = (ax+OFFSET_AX) * ACCEL_FACTOR;
    _ay = (ay+OFFSET_AY) * ACCEL_FACTOR;
    _az = (az+OFFSET_AZ) * ACCEL_FACTOR;
    // CCW->+ive, CW->-ive, unit in degree per second
    _gx = (gx+OFFSET_GX) * ROT_FACTOR;
    _gy = (gy+OFFSET_GY) * ROT_FACTOR;
    _gz = (gz+OFFSET_GZ) * ROT_FACTOR;

    bal_acc_q_y[bal_q_ctr] = (ay + OFFSET_AY);
    bal_acc_q_z[bal_q_ctr] = (az + OFFSET_AZ);
    bal_gyr_q_x[bal_q_ctr] = (gx + OFFSET_GX);
    (bal_q_ctr >= bal_q_size - 1) ? bal_q_ctr = 0 : bal_q_ctr++;
    steer_gyr_q_z[steer_q_ctr] = (gz + OFFSET_GZ);
    (steer_q_ctr >= steer_q_size - 1) ? steer_q_ctr = 0 : steer_q_ctr++;
    runSpeed();



    {
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Proportional Control (Balance Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      int32_t sum = 0;
      {
        int16_t *ptr_a = bal_acc_q_y;
        int16_t *end_a = bal_acc_q_y + bal_q_size;
        while (ptr_a < end_a) sum += *ptr_a++;
        runSpeed();
      }
      // +ive = CCW->speed +ive, -ive = CW->speed -ive
      int16_t avg_acc_y = sum / bal_q_size;

      // acceleration term
      sum = 0;
      {
        int16_t *ptr_a = bal_acc_q_z;
        int16_t *end_a = bal_acc_q_z + bal_q_size;
        while (ptr_a < end_a) sum += *ptr_a++;
        runSpeed();
      }
      // +ive = downward_accel, -ive = upward_accel
      int16_t avg_acc_z = sum / bal_q_size;

      sum = 0;
      {
        int16_t *ptr_g = bal_gyr_q_x;
        int16_t *end_g = bal_gyr_q_x + bal_q_size;
        while (ptr_g < end_g) sum += *ptr_g++;
        runSpeed();
      }
      // +ive = CCW, -ive = CW rot spd in degree per s
      float avg_rot_x = (sum / bal_q_size) * ROT_FACTOR;

      // +ive = CCW, -ive = CW in degree
      float angle_a = atan2(avg_acc_y, avg_acc_z) * 180 / PI;
      uint16_t net_acc_mag = sqrt((int32_t)(avg_acc_y) * avg_acc_y + (int32_t)(avg_acc_z) * avg_acc_z);
      runSpeed();

      float acc_w = 1.0 / (50000.0 * ((net_acc_mag / 16384.0) - 1) * ((net_acc_mag / 16384.0) - 1) + 50.0);
      int16_t pred_angle = (1.0 - acc_w) * (angle + avg_rot_x * delta_time / 10.0) + acc_w * angle_a * 100.0;
      if (abs(pred_angle - angle) < 1500) angle = pred_angle;
      int16_t _p_L = fast_round(angle * bal_K_p);
      int16_t _p_R = fast_round(angle * bal_K_p);
      // if(_p_L < 10 * bal_K_p && _p_L > -10 * bal_K_p) _p_L = 0;
      // if(_p_R < 10 * bal_K_p && _p_R > -10 * bal_K_p) _p_R = 0;
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Derivative Control (Balance Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      bal_avg_err_q[bal_avg_q_ctr] = angle;
      delta_time_q[bal_avg_q_ctr] = delta_time;
      (bal_avg_q_ctr >= bal_avg_q_size - 1) ? bal_avg_q_ctr = 0 : bal_avg_q_ctr++;

      uint8_t last_error_index = (bal_avg_q_ctr + 1) % bal_avg_q_size;
      // +ive = moving CCW, -ive = moving CW
      int16_t delta_error = angle - bal_avg_err_q[last_error_index];
      uint8_t elapsed_time = 0;
      runSpeed();
      
      {
        uint8_t *ptr = delta_time_q;
        uint8_t *end = delta_time_q + bal_avg_q_size;
        while (ptr < end) elapsed_time += *ptr++;
        elapsed_time -= delta_time_q[last_error_index];
        runSpeed();
      }

      int16_t _d = fast_round(delta_error * bal_K_d / elapsed_time);
      // if(_d < 2 && _d > -2) _d = 0;

      net_speed_L += _p_L - sgn(_p_L) * abs(_d);
      net_speed_R += _p_R - sgn(_p_R) * abs(_d);
      runSpeed();

      #ifdef DEBUG
      if(curr_time - lastPrintTime1 >= 500){
        lastPrintTime1 = curr_time;
        Serial.print("balance: _p_L\t");
        Serial.print(_p_L);
        Serial.print(" |||_p_R ");
        Serial.print(_p_R);
        Serial.print(" |||_d ");
        Serial.print(_d);
        Serial.print(" |||acc_y ");
        Serial.print(avg_acc_y);
        Serial.print(" |||acc_z ");
        Serial.print(avg_acc_z);
        Serial.print(" |||ang ");
        Serial.print(angle);
        Serial.print(" |||n_spd_L ");
        Serial.print(net_speed_L);
        Serial.print(" |||n_spd_R ");
        Serial.println(net_speed_R);
      }
      #endif
    }



    curr_time = millis();
    if(curr_time - spd_last_time >= 150){

      int32_t curr_pos_L = stepperL.currentPosition();
      int16_t curr_pos_R = stepperR.currentPosition();
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Proportional Control (Speed Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      
      int16_t delta_pos = curr_pos_L - spd_last_pos_L;
      delta_pos += curr_pos_R - spd_last_pos_R;
      delta_pos /= 2;
      float spd_delta_time = curr_time - spd_last_time;
      spd_last_pos_L = curr_pos_L;
      spd_last_pos_R = curr_pos_R;
      spd_last_time = curr_time;
      runSpeed();

      int16_t _p = fast_round((delta_pos / spd_delta_time) * spd_K_p);
      if (_p < 10 && _p > -10) _p = 0;
      runSpeed();

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Integral Control (Speed Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      int16_t _i = fast_round((curr_pos_L + curr_pos_R) * spd_K_i / 2.0);
      _i = max(min(_i, 500), -500);
      if (_i < 5 && _i > -5) _i = 0;
      runSpeed();

      if ( (angle < -150 && _i < 0) || (angle > 150 && _i > 0)) angle += _i;
      angle += - _p;

      if ((delta_pos / spd_delta_time) > 5.0){
        // halt program if speed is too high
        while(true){
          delay(1000);
          Serial.println("speed too high");
        }
      }

      #ifdef DEBUG
      if(curr_time - lastPrintTime2 >= 300){
        lastPrintTime2 = curr_time;
        Serial.print("speed: _p \t");
        Serial.print(_p);
        Serial.print(" |||_i ");
        Serial.print(_i);
        Serial.print(" |||spd ");
        Serial.println((delta_pos / spd_delta_time));
      }
      #endif
    }


    {

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Proportional Control (Steer Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------

      if(move_dir == 2 || move_dir == 3) {
        steer_net_angle = 0;
        int32_t sum = 0;
        {
          int16_t *ptr = steer_gyr_q_z;
          int16_t *end = steer_gyr_q_z + steer_q_size;
          while (ptr < end) sum += *ptr++;
          runSpeed();
        }
        // +ive = CCW, -ive = CW looking from top of MPU
        // rot spd in degree per s
        float avg_rot_z = (sum / steer_q_size) * ROT_FACTOR;
        float rot_error = bias_sgn * bias_steer_spd - avg_rot_z;
        int16_t _p = fast_round(rot_error * steer_K_p);

        net_speed_L += _p;
        net_speed_R -= _p;
        runSpeed();

      }

      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      // Integral Control (Steer Loop)
      //---------------------------------------------------------------------------------------------------------------------------------------------------------
      
      else{
        // halt program if window size is too big
        if (steer_avg_window > steer_q_size){
          while(true){
            delay(1000);
            Serial.println("avg window too big"); 
          }
        }

        int32_t sum = 0;
        {
          int16_t *ptr = (steer_gyr_q_z + steer_q_ctr) - steer_avg_window;
          int16_t *end = (steer_gyr_q_z + steer_q_ctr);
          while (ptr < end) sum += *ptr++;
          runSpeed();
        }
        // +ive = CCW, -ive = CW looking from top of MPU
        // rot spd in degree per s
        float avg_rot_z = (sum / steer_avg_window) * ROT_FACTOR;

        uint8_t elapsed_time = 0;
        {
          uint8_t *ptr = (delta_time_q + steer_q_ctr) - steer_avg_window + 1;
          uint8_t *end = (delta_time_q + steer_q_ctr);
          while (ptr < end) elapsed_time += *ptr++;
          runSpeed();
        }
        steer_net_angle += avg_rot_z * elapsed_time;

        int16_t _i = fast_round(steer_K_i * steer_net_angle);
        if (_i < 3 && _i > -3) _i = 0;

        net_speed_L += _i;
        net_speed_R -= _i;
        runSpeed();
      }

    }

    //---------------------------------------------------------------------------------------------------------------------------------------------------------
    // Set Motor Speeds
    //---------------------------------------------------------------------------------------------------------------------------------------------------------

    if(curr_time - lastSetSpeedTime >= 15){
      lastSetSpeedTime = curr_time;
      stepperL.setSpeed(net_speed_L);
      stepperR.setSpeed(net_speed_R);
    }
    runSpeed();

    #ifdef DEBUG
    if(curr_time - lastPrintTime3 >= 500){
      lastPrintTime3 = curr_time;
      //Print the values to the Serial Monitor
      Serial.print("a:\t");
      Serial.print(_ax); Serial.print("\t");
      Serial.print(_ay); Serial.print("\t");
      Serial.println(_az);
      Serial.print("g:\t");
      Serial.print(_gx); Serial.print("\t");
      Serial.print(_gy); Serial.print("\t");
      Serial.println(_gz);
      Serial.print("net_speed: b_ang_L\t");
      Serial.print(bias_sgn_L * move_bias);
      Serial.print(" |||b_ang_R ");
      Serial.print(bias_sgn_R * move_bias);
      Serial.print(" |||n_spd_L ");
      Serial.print(net_speed_L);
      Serial.print(" |||n_spd_R ");
      Serial.print(net_speed_R);
      Serial.print(" |||m_spd_L ");
      Serial.print(stepperL.speed());
      Serial.print(" |||m_spd_R ");
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

  if((move_dir == 2 || move_dir == 3) && (millis() - last_cmd_time) / 1000.0 >= bias_mag){
    move_dir = 0;
    bias_mag = 0;
    bias_sgn = 0;
  }
  runSpeed();
  
}



void runSpeed(){
  stepperR.runSpeed();
  stepperL.runSpeed();
}



void update_cmd(){
  char* numStr = &receivedChars[2];
  char* endPtr = NULL;

  if(!receivedChars ||
     receivedChars[0] == '\0' || receivedChars[1] == '\0' ||
     receivedChars[2] == '\0' || receivedChars[1] != ':'){
     return;
  }

  switch(receivedChars[0]){
    case 'b':
      move_dir = 1;
      bias_sgn = 1;
      break;
    case 'l':
      move_dir = 2;
      bias_sgn = 1;
      break;
    case 'r':
      move_dir = 3;
      bias_sgn = -1;
      break;
    case 'f':
      move_dir = 4;
      bias_sgn = -1;
      break;
    case 'p':
      bal_K_p = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("bal_K_p: ");
      Serial.println(bal_K_p);
      return;
    case 'd':
      bal_K_d = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("bal_K_d: ");
      Serial.println(bal_K_d);
      return;
    case 's':
      spd_K_p = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("spd_K_p: ");
      Serial.println(spd_K_p);
      return;
    case 'i':
      spd_K_i = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("spd_K_i: ");
      Serial.println(spd_K_i);
      return;
    case 't':
      steer_K_p = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("steer_K_p: ");
      Serial.println(steer_K_p);
      return;
    case 'k':
      steer_K_i = static_cast<int16_t>(strtol(numStr, &endPtr, 10)) / 100.0;
      Serial.print("steer_K_i: ");
      Serial.println(steer_K_i);
      return;
    case 'q':
      bias_steer_spd = static_cast<int8_t>(strtol(numStr, &endPtr, 10));
      Serial.print("bias_steer_spd: ");
      Serial.println(bias_steer_spd);
      return;
    default:
      return;
  }
  runSpeed();

  // the input shall not exceed 500 or the program will freeze due to safety protection
  bias_mag = static_cast<int16_t>(strtol(numStr, &endPtr, 10));
  if(endPtr && *endPtr != '\0'){
      move_dir = 0;
      bias_mag = 0;
      bias_sgn = 0;
      return;
  }
  runSpeed();

  if(move_dir == 1 || move_dir == 4){
    stepperL.setCurrentPosition(stepperL.currentPosition() + bias_mag * bias_sgn);
    stepperR.setCurrentPosition(stepperR.currentPosition() + bias_mag * bias_sgn);
    Serial.print(" bias: ");
    Serial.print(bias_mag * bias_sgn);
    Serial.print(" current_pos_L: ");
    Serial.print(stepperL.currentPosition());
    Serial.print(" current_pos_R: ");
    Serial.println(stepperR.currentPosition());
    move_dir = 0;
    bias_mag = 0;
    bias_sgn = 0;
  }
  else if(move_dir == 2 || move_dir == 3){
    last_cmd_time = millis();
  }
  runSpeed();
}



void receiveData() {
  static uint8_t index = 0;
  char endMarker = '\n';
  char rc;

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
  int32_t sum_gx = 0;
  int32_t sum_gy = 0;
  int32_t sum_gz = 0;

  Serial.println("\nStarting MPU calibration: keep sensor still and level...");
  delay(500);

  unsigned long nextSampleAt = millis();
  for (uint16_t i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    nextSampleAt += samplePeriodMs;
    while ((long)(nextSampleAt - millis()) > 0) {
      // Wait to keep a stable sampling period.
    }
  }

  int16_t avg_ax = (int16_t)(sum_ax / samples);
  int16_t avg_ay = (int16_t)(sum_ay / samples);
  int16_t avg_az = (int16_t)(sum_az / samples);
  int16_t avg_gx = (int16_t)(sum_gx / samples);
  int16_t avg_gy = (int16_t)(sum_gy / samples);
  int16_t avg_gz = (int16_t)(sum_gz / samples);

  OFFSET_AX = -avg_ax;
  OFFSET_AY = -avg_ay;
  OFFSET_AZ = -avg_az + 16384 * Z_DIR;
  OFFSET_GX = -avg_gx;
  OFFSET_GY = -avg_gy;
  OFFSET_GZ = -avg_gz;

  Serial.println("MPU calibration complete. Offsets:");
  Serial.print("AX: "); Serial.print(OFFSET_AX); Serial.print("\t");
  Serial.print("AY: "); Serial.print(OFFSET_AY); Serial.print("\t");
  Serial.print("AZ: "); Serial.print(OFFSET_AZ); Serial.print("\t");
  Serial.print("GX: "); Serial.print(OFFSET_GX); Serial.print("\t");
  Serial.print("GY: "); Serial.print(OFFSET_GY); Serial.print("\t");
  Serial.print("GZ: "); Serial.println(OFFSET_GZ);
}
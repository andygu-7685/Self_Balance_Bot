#include <AccelStepper.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))



MPU6050 mpu;
const float accel_factor = 9.80665 / 16384;
const float rot_factor = 1.0 / 131.0;
int16_t OFFSET_AX = 0;
int16_t OFFSET_AY = 0;
int16_t OFFSET_AZ = 0;
int16_t OFFSET_GX = 0;
int16_t OFFSET_GY = 0;
int16_t OFFSET_GZ = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float _ax, _ay, _az;
float _gx, _gy, _gz;

const uint8_t INTERRUPT_PIN = 2;          // Use Pin 2 as Interrupt 0, only 2 or 3 can be interrupt
volatile bool mpuInterrupt = false;   // Indicates whether MPU interrupt pin has gone high

AccelStepper stepperL(AccelStepper::DRIVER, 6, 5);
AccelStepper stepperR(AccelStepper::DRIVER, 9, 8);

unsigned long lastPrintTime = 0;
unsigned long lastSetSpeedTime = 0;

// ~500 for some speed
int16_t dir_bias = 0;

//-----------------------------------------------------------------------------------------------
//Proportional
const uint8_t error_q_size = 5;
int16_t error_q_x[error_q_size] = {0, 0, 0, 0, 0};
int16_t error_q_y[error_q_size] = {0, 0, 0, 0, 0};
int16_t error_q_z[error_q_size] = {0, 0, 0, 0, 0};
uint8_t error_q_ctr = 0;
float K_a = 0.05;
float K_p = 0.5 * 0.2;

//-----------------------------------------------------------------------------------------------
//Derivative
const uint8_t avg_q_size = 5;
int16_t avg_error_q[avg_q_size] = {0, 0, 0, 0, 0};
uint8_t delta_time_q[avg_q_size] = {0, 0, 0, 0, 0};
uint8_t avg_q_ctr = 0;
unsigned long last_delta_time = 0;
float K_d = 0.3 * 0.2;



void dmpDataReady() {
  mpuInterrupt = true;
}



void setup() {
  // Initialize serial communication
  Serial.begin(115200);
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

  stepperL.setPinsInverted(false, false, true);
  stepperL.setEnablePin(7);
  stepperL.enableOutputs();
  stepperL.setMinPulseWidth(50);
  stepperL.setMaxSpeed(10000);

  stepperR.setPinsInverted(false, false, true);
  stepperR.setEnablePin(10);
  stepperR.enableOutputs();
  stepperR.setMinPulseWidth(50);
  stepperR.setMaxSpeed(10000);

  pinMode(INTERRUPT_PIN, INPUT);
  // Enable Data Ready interrupt on the MPU6050
  mpu.setIntDataReadyEnabled(true);
  // Attach the Arduino interrupt to Pin 2, looking for a RISING edge
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  MPU_calibration();

  last_delta_time = millis();
}


void loop() {

  if (mpuInterrupt) {
    mpuInterrupt = false;

    // IMPORTANT: This clears the hardware interrupt pin on the MPU6050!
    // Without this, the physical pin stays HIGH and never triggers another RISING edge.
    uint8_t mpuIntStatus = mpu.getIntStatus();

    // Read raw accelerometer and gyroscope measurements
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    uint8_t delta_time = min(millis() - last_delta_time, 200);
    last_delta_time = millis();

    // unit in m/s^2
    _ax = (ax+OFFSET_AX) * accel_factor;
    _ay = (ay+OFFSET_AY) * accel_factor;
    _az = (az+OFFSET_AZ) * accel_factor;
    
    // CCW->+ive, CW->-ive, unit in degree per second
    _gx = (gx+OFFSET_GX) * rot_factor;
    _gy = (gy+OFFSET_GY) * rot_factor;
    _gz = (gz+OFFSET_GZ) * rot_factor;

    error_q_x[error_q_ctr] = (ax + OFFSET_AX);
    error_q_y[error_q_ctr] = (ay + OFFSET_AY);
    error_q_z[error_q_ctr] = (az + OFFSET_AZ) + 16384;
    (error_q_ctr >= error_q_size - 1) ? error_q_ctr = 0 : error_q_ctr++;



    // proportional control
    int16_t sum = 0;
    {
      int16_t *ptr = error_q_x;
      int16_t *end = error_q_x + error_q_size;
      while (ptr < end) sum += *ptr++;
    }
    // +ive = outward_accel->downward_from_0, -ive = inward_accel->upward_from_0
    int16_t avg_error_x = sum / error_q_size;
    int16_t _p = avg_error_x * K_p;
    if(_p < 300 * K_p && _p > -300 * K_p) _p = 0;

    // acceleration term
    sum = 0;
    {
      int16_t *ptr = error_q_z;
      int16_t *end = error_q_z + error_q_size;
      while (ptr < end) sum += *ptr++;
    }
    // +ive = upward_accel, -ive = downward_accel
    int16_t avg_error_z = sum / error_q_size;
    int16_t _a = avg_error_z * K_a;
    if(_a < 400 * K_a && _a > -400 * K_a) _a = 0;



    // derivative control
    avg_error_q[avg_q_ctr] = avg_error_x;
    delta_time_q[avg_q_ctr] = delta_time;
    (avg_q_ctr >= avg_q_size - 1) ? avg_q_ctr = 0 : avg_q_ctr++;

    uint8_t last_error_index = (avg_q_ctr + 1) % avg_q_size;
    // +ive = moving downward, -ive = moving upward
    int16_t delta_error = avg_error_x - avg_error_q[last_error_index];
    uint8_t elapsed_time = 0;
    {
      uint8_t *ptr = delta_time_q;
      uint8_t *end = delta_time_q + avg_q_size;
      while (ptr < end) elapsed_time += *ptr++;
      elapsed_time -= delta_time_q[last_error_index];
    }
    int16_t _d = round(delta_error * K_d / elapsed_time);
    if(_d < 2 && _d > -2) _d = 0;



    if(millis() - lastSetSpeedTime >= 15){
      lastSetSpeedTime = millis();
      stepperL.setSpeed(-_p + sgn(_p) * abs(_d) + dir_bias);
      stepperR.setSpeed(-_p + sgn(_p) * abs(_d) + dir_bias);
    }



    // if(millis() - lastPrintTime >= 500){
    //   lastPrintTime = millis();
    //   //Print the values to the Serial Monitor
    //   Serial.print("a/g:\t");
    //   Serial.print(_ax); Serial.print("\t");
    //   Serial.print(_ay); Serial.print("\t");
    //   Serial.print(_az); Serial.print("\t");
    //   Serial.print(_gx); Serial.print("\t");
    //   Serial.print(_gy); Serial.print("\t");
    //   Serial.println(_gz);
    //   Serial.print(_p);
    //   Serial.print(" ||| ");
    //   Serial.print(_d);
    //   Serial.print(" ||| ");
    //   Serial.println(_a);
    // }
  }

  // These MUST run constantly with no delays in the loop
  stepperL.runSpeed();
  stepperR.runSpeed();
  
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
  OFFSET_AZ = -avg_az - 16384;
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
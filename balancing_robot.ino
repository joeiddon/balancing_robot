#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//define the 4 stepper motor pins
#define DIR_PIN_1              2
#define STEP_PIN_1             3
#define DIR_PIN_2              5
#define STEP_PIN_2             4

#define ACC_SAMPLES   64       //number of times to sample accelerometer (max 255)
#define PID_SAMPLES   32       //number of times to sample pid (max 255)

#define MIN_HALFPULSE 600      //minimum halfpulse
#define MAX_PID       12288    //maximum expected pid

#define KP            0.1      //proportion
#define KD            2        //derivative

#define DEAD_ZONE 0
#define FALL_ZONE 1000

uint32_t loop_counter = 0;      //how many loops have we done?

int16_t acc_arr[ACC_SAMPLES];   //array to store accelerations
int8_t  acc_arr_p = 0;          //pointer to our averaging array

int16_t pid_arr[PID_SAMPLES];
int8_t pid_arr_p = 0;

uint32_t lastpulse_us = 0;      //when did we last send a pulse?
uint32_t halfpulse_us = 250;    //what would be the ideal time between pulses?

uint8_t halfpulse_count;        //how many half pulses have we done

int8_t dir = 1;                 //what direction

int16_t last_avg = 0;           //what was the last average?

uint8_t buf[10];                //buffer to store accelerometer readings

void setup() {
  Serial.begin(500000);
  pinMode(13, OUTPUT);
  configureSteppers();
  configureMPU9250();
}

void loop() {

  loop_counter++;
  uint32_t loop_us = micros(); //what's the time?

  if ((loop_us - lastpulse_us) >= halfpulse_us) {   //time to step the motors
    lastpulse_us += halfpulse_us;                   //make it so it looks like we stepped at the right time (even if didn't)
    halfpulse_count++;                              //doing a change, so increment halfpulse_count

    if (dir) {
      digitalWrite(DIR_PIN_1, dir == -1);
      digitalWrite(DIR_PIN_2, dir == 1);
      digitalWrite(STEP_PIN_1, halfpulse_count & 0x1);
      digitalWrite(STEP_PIN_2, halfpulse_count & 0x1);
    }
  }

  switch (loop_counter & 0x0f) {
    case 0:
      Wire.beginTransmission(MPU9250_ADDRESS);
      Wire.write(0x3B);
      Wire.endTransmission();
      break;
    case 1:
      Wire.requestFrom(MPU9250_ADDRESS, 10);
      break;
    case 2:
      uint8_t i;
      while (Wire.available()) {
        buf[i++] = Wire.read();
      }
      break;
    case 3:
      int16_t ax = buf[0] << 8 | buf[1];
      int16_t gx = buf[8] << 8 | buf[9];


      //set element at pointer in accelereation array
      //to current accelerometer reading
      //then update the pointer (by adding 1 and modding)

      acc_arr[acc_arr_p] = ax;
      acc_arr_p = (acc_arr_p + 1) % ACC_SAMPLES;

      //average the accelerometer readings
      //for a smooth reading to do PID with
      int32_t sum = 0;
      for (int i = 0; i < ACC_SAMPLES; i++) {
        sum += acc_arr[i];
      }
      int16_t avg_acc =  sum / ACC_SAMPLES;

      //work out our propotional and derivative terms
      //and update the last average to the current one
      //for next time
      int32_t p = KP * avg_acc;
      int32_t d = KD * (last_avg - avg_acc);
      last_avg = avg_acc;

      //add them together to get our output
      int32_t pid = p + d;

      //set element at pointer in pid array to
      //current pid reading
      //then update the pointer
      pid_arr[pid_arr_p] = pid;
      pid_arr_p = (pid_arr_p + 1) % PID_SAMPLES;

      //average the pid readings
      //for a smooth reading to calculate the half pulse with
      sum = 0;
      for (int i = 0; i < PID_SAMPLES; i++) {
        sum += pid_arr[i];
      }
      int16_t avg_pid =  sum / PID_SAMPLES;

      //avg_pid is now the right "throttle" to work with
      //just assign to motors

      dir = avg_pid > 0 ? -1 : 1;

      if (avg_pid < 0) avg_pid *= -1;

      halfpulse_us = MIN_HALFPULSE + (avg_pid ? MAX_PID / avg_pid : 10000);

      //check if in fall or dead zone
      if ((avg_acc > -DEAD_ZONE && avg_acc < DEAD_ZONE) || (avg_acc < -FALL_ZONE || avg_acc > FALL_ZONE)) dir = 0;

      //set led to indicate if in a motionless zone
      digitalWrite(13, !dir);

      break;
  }
}

void configureSteppers() {
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
}

void configureMPU9250() {
  //begin i2c communications
  Wire.begin();
  //set the clock speed to max
  Wire.setClock(400000);

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  //set low past filter to highest possible
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x16);

  //set low past filter to highest possible
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x16);
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

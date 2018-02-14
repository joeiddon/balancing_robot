#include <Wire.h>

#define        MPU9250_ADDRESS                        0x68
#define        MAG_ADDRESS                                0x0C

#define        GYRO_FULL_SCALE_250_DPS        0x00
#define        GYRO_FULL_SCALE_500_DPS        0x08
#define        GYRO_FULL_SCALE_1000_DPS     0x10
#define        GYRO_FULL_SCALE_2000_DPS     0x18

#define        ACC_FULL_SCALE_2_G                0x00
#define        ACC_FULL_SCALE_4_G                0x08
#define        ACC_FULL_SCALE_8_G                0x10
#define        ACC_FULL_SCALE_16_G             0x18

//define the 4 stepper motor pins
#define DIR_PIN_1                            2
#define STEP_PIN_1                         3
#define DIR_PIN_2                            5
#define STEP_PIN_2                         4

/*
 * MAX ACCELERATION IS 1000 (90deg), MAX GYRO IS ~2000 (big push)
 */

#define ACC_OFFSET      0
#define GYRO_OFFSET     -65

#define ACC_SAMPLES     255                     //number of times to sample accelerometer (max 255)
#define GYRO_SAMPLES    32                     //number of times to sample gyro (max 255)

#define TARGET  0           //target angle (units are in accels, so -1000 <--> 1000)

#define POINTS  17           //number of points to map acceleration between
#define MAX_X 1000          //max value acc+gyro will be

#define KD      0  //derivative

#define DEBUG   0           //debug mode (serial output) mode 0 = no mode

#define DEAD_ZONE 0
#define FALL_ZONE 850

int16_t speeds[] = {999, 
    20000, 10000,  4000,  1700,  1100, 
      900,   450,   390,   250,   180, 
      140,   130,   120,   120,   120, 
      120,   120,   120,   120,   120, 
      120,   120,   120,   120,   120, 
      120,   120  };
const int16_t interval_width = MAX_X / POINTS;

uint32_t loop_counter = 0;        //how many loops have we done? - used for main switch statement

int16_t acc_arr[ACC_SAMPLES];     //array to store accelerations - for averaging
int16_t  acc_arr_p = 0;            //pointer to our averaging array - where should our next sample be placed

int16_t gyro_arr[GYRO_SAMPLES];   //array to store accelerations - for averaging
int16_t  gyro_arr_p = 0;           //pointer to our averaging array - where should our next sample be placed

uint32_t lastpulse_us = 0;        //when did we last send a pulse? - so we know when to send our next one
uint16_t halfpulse_us = 250;      //what would be the ideal time between pulses? - this is just to start, obvs modified by PID
uint8_t  halfpulse_count;         //how many half pulses have we done - so we know which way to set the motor pins

int8_t dir = 1;                   //what direction

uint8_t buf[12];                  //buffer to store accelerometer readings
uint8_t buf_p;                    //buffer pointer

int32_t sum;
uint32_t loop_us;

int16_t ax, gx, avg_acc, avg_gyro, x, l;
//uint16_t ;

void setup() {
    Serial.begin(500000);
    pinMode(13, OUTPUT);
    configureSteppers();
    configureMPU9250();
}

void loop() {
    loop_counter++;
    loop_us = micros();
    
    if ((loop_us - lastpulse_us) >= halfpulse_us) { //time to step the motors
        lastpulse_us += halfpulse_us;  //make it so it looks like we stepped at the right time (even if didn't)
        halfpulse_count++;  //doing a change, so increment halfpulse_count

        if (dir) {          //so that if in a dead zone, we can not step (when 0)
            digitalWrite(DIR_PIN_1, dir == -1);
            digitalWrite(DIR_PIN_2, dir == 1);
            digitalWrite(STEP_PIN_1, halfpulse_count & 0x1);
            digitalWrite(STEP_PIN_2, halfpulse_count & 0x1);
        }
    }

    switch (loop_counter & 0x0f) {
        case 0:
            //point to register 59 - start of accel, temp and gyro registers
            Wire.beginTransmission(MPU9250_ADDRESS);
            Wire.write(0x3B);
            Wire.endTransmission();
            break;
        case 1:
            Wire.requestFrom(MPU9250_ADDRESS, 12);
            break;
        case 2:
            buf_p = 0;
            while (Wire.available()) {
                buf[buf_p++] = Wire.read();
            }
            break;
        case 4:
            //calculate the current readings
            ax = (buf[0] << 8 | buf[1]) + ACC_OFFSET;
            gx = (buf[10] << 8 | buf[11]) + GYRO_OFFSET;

            //set element at pointer in array to current reading
            //then update the pointer (by adding 1 and modding)
            acc_arr[acc_arr_p] = ax;
            acc_arr_p = (acc_arr_p + 1) % ACC_SAMPLES;

            gyro_arr[gyro_arr_p] = gx;
            gyro_arr_p = (gyro_arr_p + 1) % GYRO_SAMPLES;
            break;
        case 5:
            //average the accel and gyro readings
            sum = 0;
            for (int i = 0; i < ACC_SAMPLES; i++) {
                sum += acc_arr[i];
            }
            avg_acc = sum / ACC_SAMPLES;
            
            sum = 0;
            for (int i = 0; i < GYRO_SAMPLES; i++) {
                sum += gyro_arr[i];
            }
            avg_gyro = sum / GYRO_SAMPLES;
            break;
        case 6:
            //calculate proportional and derivative terms
            x = (avg_acc - TARGET) + KD * avg_gyro * -1;
            
            if (x < 0){
                dir = 1;
                x *= -1;
            } else {
                dir = -1;
            }

            if (x < MAX_X){
                l = x / interval_width;
                halfpulse_us = speeds[l];
                if (l == 0) dir = 0;
                //halfpulse_us = map_range(x, l * interval_width, (l+1) * interval_width, speeds[l], speeds[l+1]);
            } else {
                halfpulse_us = speeds[POINTS];
            }

            /*
            //check if in fall or dead zone
            if ((avg_acc > -DEAD_ZONE && avg_acc speeds[x / interval_width < DEAD_ZONE) || (avg_acc < -FALL_ZONE || avg_acc > FALL_ZONE)) dir = 0;

            //set led to indicate if in a motionless zone
            digitalWrite(13, !dir);*/
            break;
        case 7:
            switch (DEBUG){
                case 1:
                    Serial.print(3000);
                    Serial.print("\t");
                    Serial.print(-3000);
                    Serial.print("\t");
                    Serial.print(avg_acc);
                    Serial.print("\t");
                    Serial.print(avg_gyro);
                    Serial.print("\t");
                    Serial.println(x);
                    break;
                case 2:
                    Serial.print(1400);
                    Serial.print("\t");
                    Serial.print(-1400);
                    Serial.print("\t");
                    Serial.println(halfpulse_us);
                    break;
                case 3:
                    Serial.print(buf[10]);
                    Serial.print(" ");
                    Serial.print(buf[11]);
                    Serial.println();
                    break;
            }
            break;
    }
}

uint16_t map_range(int16_t i, int16_t a, int16_t b, int16_t c, int16_t d){
    return (i - a) * ((float)(d-c)/(b-a)) + c;
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

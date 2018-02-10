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


#define ACC_OFFSET      0
#define GYRO_OFFSET     -60


#define ACC_SAMPLES     64                     //number of times to sample accelerometer (max 255)
#define GYRO_SAMPLES    16                     //number of times to sample gyro (max 255)

#define MIN_HALFPULSE 116                    //minimum halfpulse
#define MAX_PID   1048576 //131078 //12288     //maximum expected pid

#define TARGET  0                     //target angle

#define KP      34                 //proportion  ///3 AND 0.2 ARE OK W/ 131078
#define KD      2        //derivative

#define DEBUG   -1 //modes 0 and 1 for serial debugging

#define DEAD_ZONE 0
#define FALL_ZONE 750   

uint32_t loop_counter = 0;            //how many loops have we done?

int16_t acc_arr[ACC_SAMPLES];     //array to store accelerations
int8_t    acc_arr_p = 0;                    //pointer to our averaging array

int16_t gyro_arr[GYRO_SAMPLES];     //array to store accelerations
int8_t    gyro_arr_p = 0;                    //pointer to our averaging array

uint32_t lastpulse_us = 0;            //when did we last send a pulse?
uint32_t halfpulse_us = 250;        //what would be the ideal time between pulses?
uint8_t halfpulse_count;                //how many half pulses have we done

int8_t dir = 1;                                 //what direction

uint8_t buf[12];                                //buffer to store accelerometer readings

void setup() {
    Serial.begin(500000);
    pinMode(13, OUTPUT);
    configureSteppers();
    configureMPU9250();
}

void loop() {
    int32_t sum, p, d, pid;
    uint32_t loop_us;
    
    int16_t ax, gx, avg_acc, avg_gyro;
    //uint16_t ;
    
    loop_counter++;
    loop_us = micros();

    //halfpulse_us = MIN_HALFPULSE;

    if ((loop_us - lastpulse_us) >= halfpulse_us) { //time to step the motors
        lastpulse_us += halfpulse_us;  //make it so it looks like we stepped at the right time (even if didn't)
        halfpulse_count++;  //doing a change, so increment halfpulse_count

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
            Wire.requestFrom(MPU9250_ADDRESS, 12);
            break;
        case 2:
            uint8_t i;
            while (Wire.available()) {
                buf[i++] = Wire.read();
            }
            break;
        case 4:
            ax = buf[0] << 8 | buf[1] + ACC_OFFSET;
            gx = buf[10] << 8 | buf[11] + GYRO_OFFSET;

            //set element at pointer in accelereation array
            //to current accelerometer reading
            //then update the pointer (by adding 1 and modding)
            acc_arr[acc_arr_p] = ax;
            acc_arr_p = (acc_arr_p + 1) % ACC_SAMPLES;

            gyro_arr[gyro_arr_p] = gx;
            gyro_arr_p = (gyro_arr_p + 1) % GYRO_SAMPLES;

            //average the accelerometer readings
            //for a smooth reading to do PID with
            sum = 0;
            for (int i = 0; i < ACC_SAMPLES; i++) {
                sum += acc_arr[i];
            }
            avg_acc =    sum / ACC_SAMPLES;

            //average the gyro readings
            //for a smooth reading to do PID with
            sum = 0;
            for (int i = 0; i < GYRO_SAMPLES; i++) {
                sum += gyro_arr[i];
            }
            avg_gyro =    sum / GYRO_SAMPLES;

            //calculate proportional and derivative terms

            p = KP * (avg_acc - TARGET);
            d = KD * avg_gyro * -1;
            
            //add them together to get our output
            pid = p + d;

            //pid is now the right "throttle" to work with
            //just now assign to motors

            dir = pid > 0 ? -1 : 1;

            if (pid < 0) pid *= -1;

            halfpulse_us = pid ? MIN_HALFPULSE + MAX_PID / pid : 4294967295;        //using ternary to remove divide by 0 error

            //check if in fall or dead zone
            if ((avg_acc > -DEAD_ZONE && avg_acc < DEAD_ZONE) || (avg_acc < -FALL_ZONE || avg_acc > FALL_ZONE)) dir = 0;

            //set led to indicate if in a motionless zone
            digitalWrite(13, !dir);

            switch (DEBUG){
                case 1:
                    Serial.print(4000);
                    Serial.print("\t");
                    Serial.print(-4000);
                    Serial.print("\t");
                    Serial.print(gx);
                    Serial.print("\t");
                    //Serial.print(p);
                    //Serial.print("\t");
                    //Serial.print(d);
                    //Serial.print("\t");
                    Serial.println(avg_gyro);
                    break;
                case 2:      
                    Serial.print(pid);
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

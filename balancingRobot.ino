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

#define DIR_PIN_1              2
#define STEP_PIN_1             3
#define DIR_PIN_2              5
#define STEP_PIN_2             4

int mpuVals[6];      //array to store the mpu values

uint32_t loop_speed  = 200; //microseconds
uint32_t loop_counter = 0;  //how many loops have we done?

// accelerometer approx -2000..+2000 -90deg ..+90deg
int32_t eax;
int16_t ax;
    int acc;
    int n;

uint32_t lastcalc_us;      // update periodically
uint32_t lastpulse_us;      //when did we last send a pulse?
uint32_t halfpulse_us;      //what would be the ideal time between pulses?
uint32_t thalfpulse_us;      //what would be the ideal time between pulses?
int dir, tdir;
uint16_t halfpulse_count;        //how many half pulses have we done

void setup(){
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    configureSteppers();
    configureMPU9250();
    lastpulse_us = micros();
    lastcalc_us = micros();
    halfpulse_us = 250;
    thalfpulse_us = 250;
    tdir = dir = 0;
}


#define ACC_SPIKE 700
#define ACC_DEAD  45
#define ACC_SMOOTH 20
#define PULSE_DAMP 20
#define MIN_HALF_PULSE 120
#define MAX_HALF_PULSE 3000
#define DIR_CHANGE_HALF_PULSE 4000

void loop(){
    
    loop_counter++;
    uint32_t loop_us = micros(); //what's the time?

    
    if ((loop_us - lastcalc_us) >= 500) {
       lastcalc_us += 500;
       halfpulse_us += (thalfpulse_us - halfpulse_us) / PULSE_DAMP;
       if (tdir != dir && halfpulse_us >= DIR_CHANGE_HALF_PULSE - PULSE_DAMP)
            dir = tdir;
    }

    //halfpulse_us = 250;
    //dir = 1;
    
    if ((loop_us - lastpulse_us) >= halfpulse_us) { //time to step the motors
        lastpulse_us += halfpulse_us;               //make it so it looks like we stepped at the right time (even if didn't)
        halfpulse_count++;                          //doing a change, so increment halfpulse_count                 
        
        //boolean dir = 1; //(halfpulse_count & 0x100) != 0;
        digitalWrite(DIR_PIN_1, dir == 1);
        digitalWrite(DIR_PIN_2, dir == -1);
        if (dir || (halfpulse_count & 0x1) ==0) {
        digitalWrite(STEP_PIN_1, halfpulse_count & 0x1);
        digitalWrite(STEP_PIN_2, halfpulse_count & 0x1);
        }
     }

     switch (loop_counter & 0x0f){
        case 0:
            Wire.beginTransmission(MPU9250_ADDRESS);
            Wire.write(0x3B);
            Wire.endTransmission();
            break;
        case 1:
            ax = 0;
            Wire.requestFrom(MPU9250_ADDRESS, 2);
            break;
        case 2:
            if (Wire.available()) ax = (int32_t)(Wire.read() << 8);
            break;
        case 3:
            if (Wire.available()) ax = (int32_t)((uint32_t)ax | Wire.read());
            break;
        case 4:
//            if (n < 10 && (ax < -ACC_SPIKE || ax > ACC_SPIKE)) { ++n; break; } 
//            n = 0;
            eax += ax;
            // acc = (int)(eax >> 7);
            acc = eax / ACC_SMOOTH;
            eax -= acc;
            /*
            Serial.print(2000);
            Serial.print(" ");
            Serial.print(acc);
            Serial.print(" ");
            Serial.println(-2000);*/
            if (tdir != (acc < -ACC_DEAD ? -1 : acc > ACC_DEAD ? +1 : 0)) {
                tdir = acc < -ACC_DEAD ? -1 : acc > ACC_DEAD ? +1 : 0;
                thalfpulse_us = DIR_CHANGE_HALF_PULSE;
            }
            digitalWrite(13, dir ==0 );
            if (tdir == dir) {
                if (acc < 0) acc = -acc;
                thalfpulse_us = MIN_HALF_PULSE + 4000 / (acc / 1);
                if (thalfpulse_us < MIN_HALF_PULSE) thalfpulse_us = MIN_HALF_PULSE;
                if (thalfpulse_us > MAX_HALF_PULSE) thalfpulse_us = MAX_HALF_PULSE;
            }
            break; 
     }
}

void configureSteppers(){
    pinMode(DIR_PIN_1, OUTPUT);
    pinMode(STEP_PIN_1, OUTPUT);
    pinMode(DIR_PIN_2, OUTPUT);
    pinMode(STEP_PIN_2, OUTPUT);
}

void configureMPU9250(){
    //begin i2c communications
    Wire.begin();
    //set the clock speed to max
    Wire.setClock(400000);
    
    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);

    //set low past filter to highest possible
    I2CwriteByte(MPU9250_ADDRESS,29,0x16);
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
 }

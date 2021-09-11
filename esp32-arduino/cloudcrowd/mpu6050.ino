
#include <Wire.h>

#define SIGNAL_PATH_RESET 0x68
#define INT_PIN_CFG 0x37
#define ACCEL_CONFIG 0x1C
#define MOT_THR 0x1F // Motion detection threshold bits [7:0]
#define MOT_DUR 0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL 0x69
#define INT_ENABLE 0x38
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define INT_STATUS 0x3A
//when nothing connected to AD0 than address is 0x68
#define ADO 1
#if ADO
#define MPU6050_ADDRESS 0x69 // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68 // Device address when ADO = 0
#endif


void mpu6050_setup() {

  // turn off sleep
  i2c_write_reg(MPU6050_ADDRESS, 0x6B, 0x00); 

  //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  i2c_write_reg(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); 

  //write register 0x37 to select how to use the interrupt pin. For an
  //active high, push-pull signal that stays until register (decimal) 58 is
  //read, write 0x20.
  //
  // For active low open drain, 50us pulse we want 0x80|0x40 = 0xA0
  // For active low open drain, latching we want 0x80|0x40|0x20 = 0xC0
  // 
  i2c_write_reg(MPU6050_ADDRESS, INT_PIN_CFG, 0xC0); 

  //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. 
  // For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, 
  // but they are used! Leaving them 0 means the filter always outputs 0.)
  i2c_write_reg(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01); 

  //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  //i2c_write_reg(MPU6050_ADDRESS, MOT_THR, 10); 
  i2c_write_reg(MPU6050_ADDRESS, MOT_THR, 2); 

  //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
  //i2c_write_reg(MPU6050_ADDRESS, MOT_DUR, 40); 
  i2c_write_reg(MPU6050_ADDRESS, MOT_DUR, 20); 

  //to register 0x69, write the motion detection decrement and a few other
  //settings (for example write 0x15 to set both free-fall and motion decrements 
  // to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
  //
  i2c_write_reg(MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); 

  //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
  i2c_write_reg(MPU6050_ADDRESS, INT_ENABLE, 0x40); 
}




#include "MPU9250.h"
#include <Wire.h>
int i=0;
float caliAcc[3];
float caliGyro[3];
void setup()
{
  int i=0;
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  resetMPU9250();
  //Serial.println("MPU9250 reseted"); 
  calibrateMPU9250(caliGyro, caliAcc);
  // Configure range
  writeByte(MPU9250_ADDRESS,GYRO_CONFIG,GYRO_FULL_SCALE_250_DPS);
  writeByte(MPU9250_ADDRESS,ACCEL_CONFIG,ACC_FULL_SCALE_2_G);
  writeByte(MPU9250_ADDRESS,ACCEL_CONFIG2,0x04);

  // Request first magnetometer single measurement
//  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  /*Serial.println("Acc cali in main");
  for (i=0; i<3; i++){
    Serial.print(caliAcc[i]);
    Serial.print("   "); 
  }  
  Serial.print("\n");
  Serial.println("Gyro cali");
  for (i=0; i<3; i++){
    Serial.print(caliGyro[i]);
    Serial.print("   ");
  }  
  Serial.print("**************************\n");
  */
  //enable FIFO
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x04);   // Reset FIFO
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);    // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);  //enable accel and gyro data in FIFO
  
  uint8_t tmp=readByte(MPU9250_ADDRESS, CONFIG);
  writeByte(MPU9250_ADDRESS, CONFIG, 0x40 | tmp);
}


int count=0;
int16_t accel_x=0;
int16_t accel_y=0;
int16_t accel_z=0;
int16_t gyro_x=0;
int16_t gyro_y=0;
int16_t gyro_z=0;

// Main loop, read and display data
void loop()
{  
  uint8_t dest[512];
  uint16_t fifo_count=sampleFIFO(dest);
  for (int i=0;i<fifo_count;i++)
  {
    switch (count)
    {
    case 0:
      accel_x=dest[i];
      break;
    case 1:
      accel_x=(int16_t)((int16_t)accel_x<<8) | dest[i];
      break;
    case 2:
      accel_y=dest[i];
      break;
    case 3:
      accel_y=(int16_t)((int16_t)accel_y<<8) | dest[i];
      break;
    case 4:
      accel_z=dest[i];
      break;
    case 5:
      accel_z=(int16_t)((int16_t)accel_z<<8) | dest[i];
      break;
    case 6:
      gyro_x=dest[i];
      break;
    case 7:
      gyro_x=(int16_t)((int16_t)gyro_x<<8) | dest[i];
      break;  
    case 8:
      gyro_y=dest[i];
      break;
    case 9:
      gyro_y=(int16_t)((int16_t)gyro_y<<8) | dest[i];
      break; 
    case 10:
      gyro_z=dest[i];
      break;
    case 11:
      gyro_z=(int16_t)((int16_t)gyro_z<<8) | dest[i];
      String temp_str;
      temp_str=temp_str+String(accel_x)+" ";
      temp_str=temp_str+String(accel_y)+" ";
      temp_str=temp_str+String(accel_z)+" ";
      temp_str=temp_str+String(gyro_x)+" ";
      temp_str=temp_str+String(gyro_y)+" ";
      temp_str=temp_str+String(gyro_z)+" \n";
      Serial.print(temp_str);
      break;    
    }
    count=(count+1)%12;
    //Serial.println(count);
  }
}

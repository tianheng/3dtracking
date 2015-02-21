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
  Serial.begin(9600);
  resetMPU9250();
  Serial.println("MPU9250 reseted"); 
  calibrateMPU9250(caliGyro, caliAcc);
  // Configure range
  writeByte(MPU9250_ADDRESS,GYRO_CONFIG,GYRO_FULL_SCALE_250_DPS);
  writeByte(MPU9250_ADDRESS,ACCEL_CONFIG,ACC_FULL_SCALE_2_G);
  writeByte(MPU9250_ADDRESS,ACCEL_CONFIG2,0x04);

  // Request first magnetometer single measurement
//  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  Serial.println("Acc cali in main");
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
  Serial.print("\n");
}
 
// Main loop, read and display data
void loop()
{
  int16_t acc[3];
  int16_t gyro[3];
  int16_t mag[3];
  int16_t dest[6];
  sampleFIFO(20, dest);
  
/*  readAccelData(acc);
  readGyroData(gyro);
*/  
  Serial.println("Acc data and gyro data");
  for (i=0; i<3; i++){
    Serial.print(dest[i]-caliAcc[i]);
    Serial.print("   ");    
  }
  for (i=3; i<6; i++){
    Serial.print(dest[i]);
    Serial.print("   ");
    
  }
  Serial.print("\n");

/*  Serial.println("Gyro data");
    for (i=0; i<3; i++){
    Serial.print(gyro[i]);
    Serial.print("   ");
  }
  Serial.print("\n");
  delay(200);*/ 
}

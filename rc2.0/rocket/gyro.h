#ifndef __GYRO_H__
#define __GYRO_H__

// Gyro globals OBSOLETE
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,MaX,MaY,MaZ;


void init_gyro();
void update_gyro_data();
String get_gyro_data();

#endif
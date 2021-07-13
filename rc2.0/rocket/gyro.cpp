#include "gyro.h"
#include <SparkFunMPU9250-DMP.h>


MPU9250_DMP imu;


void init_gyro()
{
    if (imu.begin() != INV_SUCCESS)
    {
        #ifdef DEBUG
        Serial.println("Gyro init failed");
        #endif
    }

    // Configure

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(16); // Set accel to +/-16g
    // Note: the MPU-9250's magnetometer FSR is set at
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz

    #ifdef DEBUG
    Serial.println("Gyro init OK");
    update_gyro_data();
    Serial.println(get_gyro_data);
    #endif
}


void update_gyro_data()
{
    if ( imu.dataReady() )
    {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS | UPDATE_TEMP);
    }
}


String get_gyro_data()
{
    String ret_str = "";
    ret_str = String(imu.calcAccel(imu.ax)) + "," + String(imu.calcAccel(imu.ay)) + "," + String(imu.calcAccel(imu.az));
    ret_str += "," + (((float)imu.temperature)/340.00+36.53) + ",";
    ret_str += String(imu.calcGyro(imu.gx)) + "," + String(imu.calcGyro(imu.gy)) + "," + String(imu.calcGyro(imu.gz));
    ret_str += String(imu.calcMag(imu.mx)) + "," + String(imu.calcMag(imu.my)) + "," + String(imu.calcMag(imu.mz))
    return ret_str;
}

// Deprecated

void init_gyro_old()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    // 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);  // Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x18);  // Set the register bits as 00011000 (+/- 16g full scale range)
    Wire.endTransmission(true);
    #ifdef DEBUG
    Serial.println("Gyro start");
    #endif
}


String getGyroData_old()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    String retVal = String(AcX) + "," + String(AcY) + "," + String(AcZ) + ",";
    retVal += String(Tmp/340.00+36.53) + "," + String(GyX) + "," + String(GyY) + "," + String(GyZ);
    return retVal;
}
#include <Arduino.h>
#include <Wire.h>
#include <BMX_055.h>

BMX_055::BMX_055()
{
}


void BMX_055::updateAcc()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(m_addr_acc);
        Wire.write((2 + i));// Select data register
        Wire.endTransmission();
        Wire.requestFrom(m_addr_acc, 1);// Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    m_x_acc = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (m_x_acc > 2047)  m_x_acc -= 4096;
    m_y_acc = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (m_y_acc > 2047)  m_y_acc -= 4096;
    m_z_acc = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (m_z_acc > 2047)  m_z_acc -= 4096;
    m_x_acc = m_x_acc * 0.0098; // renge +-2g
    m_y_acc = m_y_acc * 0.0098; // renge +-2g
    m_z_acc = m_z_acc * 0.0098; // renge +-2g
}

void BMX_055::updateGyr()
{
    int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(m_addr_gyr);
        Wire.write((2 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(m_addr_gyr, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    m_x_gyr = (data[1] * 256) + data[0];
    if (m_x_gyr > 32767)  m_x_gyr -= 65536;
    m_y_gyr = (data[3] * 256) + data[2];
    if (m_y_gyr > 32767)  m_y_gyr -= 65536;
    m_z_gyr = (data[5] * 256) + data[4];
    if (m_z_gyr > 32767)  m_z_gyr -= 65536;

    m_x_gyr = m_x_gyr * 0.0038; //  Full scale = +/- 125 degree/s
    m_y_gyr = m_y_gyr * 0.0038; //  Full scale = +/- 125 degree/s
    m_z_gyr = m_z_gyr * 0.0038; //  Full scale = +/- 125 degree/s
}

void BMX_055::updateMag()
{
    int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(m_addr_mag);
        Wire.write((0x42 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(m_addr_mag, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        if (Wire.available() == 1)
            data[i] = Wire.read();
    }
    // Convert the data
    m_x_mag = ((data[1] <<8) | (data[0]>>3));
    if (m_x_mag > 4095)  m_x_mag -= 8192;
    m_y_mag = ((data[3] <<8) | (data[2]>>3));
    if (m_y_mag > 4095)  m_y_mag -= 8192;
    m_z_mag = ((data[5] <<8) | (data[4]>>3));
    // if (m_z_mag > 16383)  m_z_mag -= 32768;
    if (m_z_mag > 4095)  m_z_mag -= 8192;
}

void BMX_055::start()
{
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x0F); // Select PMU_Range register
    Wire.write(0x03);   // Range = +/- 2g
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x10);  // Select PMU_BW register
    Wire.write(0x08);  // Bandwidth = 7.81 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_acc);
    Wire.write(0x11);  // Select PMU_LPW register
    Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x0F);  // Select Range register
    Wire.write(0x04);  // Full scale = +/- 125 degree/s
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x10);  // Select Bandwidth register
    Wire.write(0x07);  // ODR = 100 Hz
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_gyr);
    Wire.write(0x11);  // Select LPM1 register
    Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x83);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4B);  // Select Mag register
    Wire.write(0x01);  // Soft reset
    Wire.endTransmission();
    delay(100);
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4C);  // Select Mag register
    // Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.write(0x00);  // Normal Mode, ODR = 10 Hz   ODR: output data rate
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x4E);  // Select Mag register
    // Wire.write(0x84);  // X, Y, Z-Axis enabled
    Wire.write(0xC7);  // X, Y, Z-Axis enabled
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x51);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for X-Y Axis = 9
    Wire.endTransmission();
    //------------------------------------------------------------//
    Wire.beginTransmission(m_addr_mag);
    Wire.write(0x52);  // Select Mag register
    Wire.write(0xFF);  // No. of Repetitions for Z-Axis = 15
    Wire.endTransmission();

}

void BMX_055::update()
{
    updateAcc() ;
    updateGyr() ;
    updateMag() ;
}


float BMX_055::getXAcc()
{
    return m_x_acc ;
}


float BMX_055::getYAcc()
{
    return m_y_acc ;
}


float BMX_055::getZAcc()
{
    return m_z_acc ;
}


float BMX_055::getXGyr()
{
    return m_x_gyr ;
}


float BMX_055::getYGyr()
{
    return m_y_gyr ;
}


float BMX_055::getZGyr()
{
    return m_z_gyr ;
}


float BMX_055::getXMag()
{
    return m_x_mag ;
}


float BMX_055::getYMag()
{
    return m_y_mag ;
}


float BMX_055::getZMag()
{
    return m_z_mag ;
}

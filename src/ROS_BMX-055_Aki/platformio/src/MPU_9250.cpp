#include <MPU_9250.h>

MPU_9250::MPU_9250()
{
    m_acc_range = 16.0 ;
    m_gyr_range = 2000.0 ;
}

void MPU_9250::updateAcc()
{
  this->m_raw_x_acc = (this->m_acc_gyr_tmp_data[0] << 8) | this->m_acc_gyr_tmp_data[1];
  this->m_raw_y_acc = (this->m_acc_gyr_tmp_data[2] << 8) | this->m_acc_gyr_tmp_data[3];
  this->m_raw_z_acc = (this->m_acc_gyr_tmp_data[4] << 8) | this->m_acc_gyr_tmp_data[5];

  m_x_acc = (m_raw_x_acc / 32768.0) * m_acc_range * 9.80665 ;
  m_y_acc = (m_raw_y_acc / 32768.0) * m_acc_range * 9.80665 ;
  m_z_acc = (m_raw_z_acc / 32768.0) * m_acc_range * 9.80665 ;
}


void MPU_9250::updateGyr()
{
  m_raw_x_gyr = (m_acc_gyr_tmp_data[8]  << 8) | m_acc_gyr_tmp_data[9];
  m_raw_y_gyr = (m_acc_gyr_tmp_data[10] << 8) | m_acc_gyr_tmp_data[11];
  m_raw_z_gyr = (m_acc_gyr_tmp_data[12] << 8) | m_acc_gyr_tmp_data[13];

  m_x_gyr = (m_raw_x_gyr / 32768.0) * m_gyr_range * (1/360.0) ;
  m_y_gyr = (m_raw_y_gyr / 32768.0) * m_gyr_range * (1/360.0) ;
  m_z_gyr = (m_raw_z_gyr / 32768.0) * m_gyr_range * (1/360.0) ;
}


void MPU_9250::updateMag()
{
  m_raw_x_mag = (m_mag_data[3] << 8) | m_mag_data[2];
  m_raw_y_mag = (m_mag_data[1] << 8) | m_mag_data[0];
  m_raw_z_mag = (m_mag_data[5] << 8) | m_mag_data[4];

  m_x_mag = (m_raw_x_mag / 32768.0) * 4800.0f;
  m_y_mag = (m_raw_y_mag / 32768.0) * 4800.0f;
  m_z_mag = (m_raw_z_mag / 32768.0) * 4800.0f;
}


void MPU_9250::updateTmp()
{
  m_raw_tmp = (m_acc_gyr_tmp_data[6] << 8) | m_acc_gyr_tmp_data[7];
  m_tmp     = (m_raw_tmp/333.87) + 21.0f ;
}


void MPU_9250::start()
{
    //I2C通信を開始する
    Wire.begin();
    delay(100) ;

    // スリープモードの解除
    Wire.beginTransmission(m_mpu9250_addr) ;
    Wire.write(0x6b) ;
    Wire.write(0x00) ;
    Wire.endTransmission() ;
    delay(100) ;

    // 加速度センサの設定
    Wire.beginTransmission(m_mpu9250_addr) ;
    Wire.write(m_acc_config) ;
    Wire.write(m_acc_config_16g) ;
    Wire.endTransmission() ;
    delay(100) ;

    // ジャイロセンサの設定
    Wire.beginTransmission(m_mpu9250_addr) ;
    Wire.write(m_gyr_config) ;
    Wire.write(m_gyr_config_2000dps) ;
    Wire.endTransmission() ;
    delay(100) ;

    // 磁気センサのバイパスモードを切り替える
    Wire.beginTransmission(m_mpu9250_addr) ;
    Wire.write(0x37) ;
    Wire.write(0x02) ;
    Wire.endTransmission() ;
    delay(100) ;

    // 磁気センサの設定
    Wire.beginTransmission(m_ak8963_addr) ;
    Wire.write(m_mag_config) ;
    Wire.write(m_mag_config_100hz) ;
    Wire.endTransmission() ;
    delay(100) ;
}


void MPU_9250::update()
{
    uint8_t index = 0;

    // 加速度,ジャイロ,温度の観測モジュールからデータを取得
    Wire.beginTransmission(m_mpu9250_addr) ;
    Wire.write(0x3b) ;
    Wire.endTransmission() ;
    Wire.requestFrom(m_mpu9250_addr, 14) ;
    index = 0 ;
    while(Wire.available())
    {
        this->m_acc_gyr_tmp_data[index++] = Wire.read() ;
    }

    // 磁気センサの読み取りフラッグを更新
    Wire.beginTransmission(m_ak8963_addr) ;
    Wire.write(m_mag_flag) ;
    Wire.endTransmission() ;
    Wire.requestFrom(m_ak8963_addr, 1) ;
    index = 0;
    while(Wire.available())
    {
        this->m_mag_flag_data = Wire.read() ;
    }

    // 地磁気センサのモジュールからデータを取得
    if((m_mag_flag_data & 0x01))
    {
        Wire.beginTransmission(m_ak8963_addr) ;
        Wire.write(0x03) ;
        Wire.endTransmission() ;
        Wire.requestFrom(m_ak8963_addr, 7) ;
        index = 0;
        while(Wire.available())
        {
            this->m_mag_data[index++] = Wire.read() ;
        }
    }

    // 加速度を更新
    updateAcc() ;
    // ジャイロを更新
    updateGyr() ;
    // 地磁気を更新
    updateMag() ;
    // センサ温度を更新
    updateTmp() ;
}


float MPU_9250::getXAcc()
{
    return this->m_x_acc ;
}


float MPU_9250::getYAcc()
{
    return this->m_y_acc ;
}


float MPU_9250::getZAcc()
{
    return this->m_z_acc ;
}


float MPU_9250::getXGyr()
{
    return this->m_x_gyr ;
}


float MPU_9250::getYGyr()
{
    return this->m_y_gyr ;
}


float MPU_9250::getZGyr()
{
    return this->m_z_gyr ;
}


float MPU_9250::getXMag()
{
    return this->m_x_mag ;
}


float MPU_9250::getYMag()
{
    return this->m_y_mag ;
}


float MPU_9250::getZMag()
{
    return this->m_z_mag ;
}




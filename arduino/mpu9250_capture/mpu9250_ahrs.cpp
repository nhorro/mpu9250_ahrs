#include "mpu9250_ahrs.h"
#include <math.h>

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



// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void i2c_read(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    // Read Nbytes
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index=0;
    while (Wire.available())
      Data[index++]=Wire.read();
}



// Write a byte (Data) in device (Address) at register (Register)
void i2c_write_byte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}



mpu9250::mpu9250()
    :
        mag_max({-32767, -32767, -32767})
      , mag_min({32767, 32767, 32767})
      , mag_bias_correction({0,0,0})

      , gyro_angles({0,0,0})
      , last_gyro_update(millis())
{
  
}
  


int8_t mpu9250::setup() 
{
    uint8_t whoami;
    i2c_read(MPU9250_ADDRESS,117,1,&whoami);
    //Serial.println (whoami); 
    if(0x71 == whoami)
    {
        this->status = 0;

        /* General configuration
         */

         /* Accelerometer configuration */
         // Set accelerometers low pass filter at 5Hz
         i2c_write_byte(MPU9250_ADDRESS,29,0x06);
         i2c_write_byte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
         //i2c_write_byte(MPU9250_ADDRESS,19, ( acc_bias_correction[0]       & 0xFF00)  );
         //i2c_write_byte(MPU9250_ADDRESS,20, ((acc_bias_correction[0] >> 8) & 0x00FF) );
         //i2c_write_byte(MPU9250_ADDRESS,21, ( acc_bias_correction[1]       & 0xFF00)  );
         //i2c_write_byte(MPU9250_ADDRESS,22, ((acc_bias_correction[1] >> 8) & 0x00FF) );
                 
         /* Gyroscope configuration */
         // Set gyroscope low pass filter at 5Hz
         i2c_write_byte(MPU9250_ADDRESS,26,0x06);
         // Configure gyroscope range
         i2c_write_byte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
         // Configure accelerometers range

         // Set by pass mode for the magnetometers
         i2c_write_byte(MPU9250_ADDRESS,0x37,0x02);

         /* Magnetometer configuration */   
         // Request continuous magnetometer measurements in 16 bits
         i2c_write_byte(MAG_ADDRESS,0x0A,0x16);  
    }
    else
    {
        this->status = -1;
    }

    return this->status;
}



void mpu9250::set_mag_bias_correction(int16_t x, int16_t y, int16_t z)
{
    this->mag_bias_correction[0] = x;
    this->mag_bias_correction[1] = y;
    this->mag_bias_correction[2] = z;
}


void mpu9250::read() 
{    
  // Read accelerometer and gyroscope
  uint8_t buf[14];
  i2c_read(MPU9250_ADDRESS,0x3B,14,buf);

  // Accelerometer (TODO: check axis inversions)
  this->raw_values[acc_x] =  (buf[0]<<8 | buf[1]);
  this->raw_values[acc_y] = -(buf[2]<<8 | buf[3]);
  this->raw_values[acc_z] =   buf[4]<<8 | buf[5];
  
  // Gyroscope (TODO: check axis inversions)
  this->raw_values[gyro_x] = -(buf[10]<<8 | buf[11]);
  this->raw_values[gyro_y] = -(buf[8]<<8 | buf[9]);
  this->raw_values[gyro_z] =   buf[12]<<8 | buf[13];

  // Magnetometer     
  // Read register Status 1 and wait for the DRDY: Data Ready
  this->mag_valid = false;
  uint8_t ST1;
  uint32_t timeout = 100;
  do {
    i2c_read(MAG_ADDRESS,0x02,1,&ST1);
  }
  while ( !(ST1&0x01) && (--timeout) );

  if(timeout)
  {      
      // Read magnetometer data  
      i2c_read(MAG_ADDRESS,0x03,7,buf);
      this->raw_values[mag_x] = -( buf[3]<<8 | buf[2] );
      this->raw_values[mag_y] = -( buf[1]<<8 | buf[0] );
      this->raw_values[mag_z] = -( buf[5]<<8 | buf[4] );
      this->mag_valid = true;
  }    
}

void mpu9250::transform_units()
{
  // Accelerometer
  this->processed_values[acc_x] = this->raw_values[acc_x] / (2.0 * G_EARTH);
  this->processed_values[acc_y] = this->raw_values[acc_y] / (2.0 * G_EARTH);
  this->processed_values[acc_z] = this->raw_values[acc_z] / (2.0 * G_EARTH);

  // Gyro
  this->processed_values[gyro_x] = this->raw_values[gyro_x] / 131.0;
  this->processed_values[gyro_y] = this->raw_values[gyro_y] / 131.0;
  this->processed_values[gyro_z] = this->raw_values[gyro_z] / 131.0;

  // Mag  
  this->processed_values[mag_x] = this->raw_values[mag_x]* (4912.0/32760.0) * 10.0 + this->mag_bias_correction[0];
  this->processed_values[mag_y] = this->raw_values[mag_y]* (4912.0/32760.0) * 10.0 + this->mag_bias_correction[1];
  this->processed_values[mag_z] = this->raw_values[mag_z]* (4912.0/32760.0) * 10.0 + this->mag_bias_correction[2];
}



void mpu9250::calc_euler_angles_from_accmag()
{  
  // TODO
}



void mpu9250::integrate_gyro_angles(uint32_t t)
{
    uint32_t dt = this->last_gyro_update > t ? 1 + this->last_gyro_update + ~t : t - this->last_gyro_update;
    this->gyro_angles[0] += (this->raw_values[gyro_x] / 131.0)*dt/1000;
    this->gyro_angles[1] += (this->raw_values[gyro_y] / 131.0)*dt/1000;
    this->gyro_angles[2] += (this->raw_values[gyro_z] / 131.0)*dt/1000;
    this->last_gyro_update = t;
}


// DEBUG

void mpu9250::debug_print_raw_values(int flags)
{  
   Serial.print (millis()); 
   Serial.print (CSV_SEP);
    
  // Accelerometer
  if (flags & DEBUG_PRINT_FLAGS_ACC)
  {
    Serial.print (this->raw_values[acc_x]); 
    Serial.print (CSV_SEP);
    Serial.print (this->raw_values[acc_y]);
    Serial.print (CSV_SEP);
    Serial.print (this->raw_values[acc_z]);  
    Serial.print (CSV_SEP);
  }
  
  // Gyroscope
  if (flags & DEBUG_PRINT_FLAGS_GYRO)
  {
      Serial.print (this->raw_values[gyro_x]); 
      Serial.print (CSV_SEP);
      Serial.print (this->raw_values[gyro_y]);
      Serial.print (CSV_SEP);
      Serial.print (this->raw_values[gyro_z]);  
      Serial.print (CSV_SEP);
  }
 
  // Magnetometer
  if (flags & DEBUG_PRINT_FLAGS_MAG)
  {
    Serial.print (this->mag_valid); 
    Serial.print (CSV_SEP);    
    Serial.print (this->raw_values[mag_x]); 
    Serial.print (CSV_SEP);
    Serial.print (this->raw_values[mag_y]);
    Serial.print (CSV_SEP);
    Serial.print (this->raw_values[mag_z]);  
    Serial.print (CSV_SEP);    
  }

}

void mpu9250::debug_print_processed_values(int flags)
{
    // Accelerometer
    if (flags & DEBUG_PRINT_FLAGS_ACC)
    {  
      Serial.print (this->processed_values[acc_x]); 
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[acc_y]);
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[acc_z]);  
    }

    // Gyroscope
    if (flags & DEBUG_PRINT_FLAGS_GYRO)
    {
      Serial.print (this->processed_values[gyro_x]); 
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[gyro_y]);
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[gyro_z]);  
      Serial.print (CSV_SEP);  
    }

    // Magnetometer
    if (flags & DEBUG_PRINT_FLAGS_MAG)
    {
      Serial.print (this->mag_valid); 
      Serial.print (CSV_SEP);  

      Serial.print (this->processed_values[mag_x]); 
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[mag_y]);
      Serial.print (CSV_SEP);  
      Serial.print (this->processed_values[mag_z]);  
      Serial.print (CSV_SEP);  
    }   
}



void mpu9250::debug_print_euler_angles(int flags)
{    
    if (flags & DEBUG_PRINT_MAGACC_EULER)
    {    
        Serial.print(this->euler_angles_magacc[0]);
        Serial.print (CSV_SEP);  
        Serial.print(this->euler_angles_magacc[1]);
        Serial.print (CSV_SEP);  
        Serial.print(this->euler_angles_magacc[2]);
        Serial.print (CSV_SEP);  
    }

    if (flags & DEBUG_PRINT_GYRO_EULER)
    {    
        Serial.print(this->euler_angles_gyro[0]);
        Serial.print (CSV_SEP);  
        Serial.print(this->euler_angles_gyro[1]);
        Serial.print (CSV_SEP);  
        Serial.print(this->euler_angles_gyro[2]);
        Serial.print (CSV_SEP);  
    }    
}
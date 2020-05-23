#include <Wire.h>
#include "mpu9250_ahrs.h"

struct period_descr {
  uint32_t t0;
  uint32_t period;
};

#define IMU_UPDATE_FREQ              100
#define IMU_UPDATE_PERIOD_IN_MS      (1000/IMU_UPDATE_FREQ)
#define STDOUT_UPDATE_FREQ           100
#define STDOUT_UPDATE_PERIOD_IN_MS   (1000/STDOUT_UPDATE_FREQ)

int application_mode = 0; // 0 = Idle
mpu9250 imu;
period_descr periodic_tasks[2];


inline bool check_period(uint32_t& t0, uint32_t period, uint32_t curr_time)
{
  uint32_t dt = t0 > curr_time ? 1 + t0 + ~curr_time : curr_time - t0;
  if (dt >= period)
  {
    t0 = millis();
    return true;
  }

  return false;
}

// Initializations
void setup()
{
  // Arduino initializations
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Serial.begin(115200);  
  while (!Serial);

  for(int i=0;i<5;i++)
  {
    delay(1000);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
  }
  if ( !imu.setup() )
  {
    imu.set_mag_bias_correction(110.0, 12.0, 12.0);
    //Serial.println("IMU initialized OK");
    uint32_t curr_time = millis();

    periodic_tasks[0].t0 = curr_time;
    periodic_tasks[0].period = IMU_UPDATE_PERIOD_IN_MS;

    periodic_tasks[1].t0 = curr_time;
    periodic_tasks[1].period = STDOUT_UPDATE_PERIOD_IN_MS;


    
  }
  else
  {
    Serial.println("IMU initialiation failed.");
  }
}

void process_input()
{
  if (Serial.available())
  {
    int c = Serial.read();
    switch (c)
    {
      case '1': {
          Serial.println("Application mode changed to default");
          application_mode = 0;
        } break;

      case '2': {
          Serial.println("Application mode changed to magnetometer callibration");
          application_mode = 1;
        } break;

      default: {
        } break;
    }
  }
}


// Main loop, read and display data
void loop()
{
  process_input();
  switch (application_mode)
  {
    // Idle (default)
    case 0:
      {
        uint32_t t1 = millis();
        if ( check_period( periodic_tasks[0].t0, periodic_tasks[0].period, t1) )
        {
          //Serial.println("Read");
          imu.read();
          imu.process();
          imu.calc_euler_angles();
          imu.update_gyro_angles(t1);
        }

        if ( check_period( periodic_tasks[1].t0, periodic_tasks[1].period, t1) )
        {
          //imu.debug_print_euler_angles();
          //imu.debug_print_processed_values(DEBUG_PRINT_FLAGS_MAG);
          imu.debug_print_raw_values( DEBUG_PRINT_FLAGS_ACC  | DEBUG_PRINT_FLAGS_GYRO | DEBUG_PRINT_FLAGS_MAG );
          //imu.debug_print_raw_values(  DEBUG_PRINT_FLAGS_MAG );
          //imu.debug_print_raw_values( DEBUG_PRINT_FLAGS_ACC );
          //imu.debug_print_gyro_angles();
          Serial.println();
        }

      } break;

    // Magnetometer callibration
    case 1:
      {
        imu.calc_mag_bias_correction();
        delay(5000);
      } break;

  }

}











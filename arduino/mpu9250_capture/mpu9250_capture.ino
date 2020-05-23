#include <Wire.h>
#include "mpu9250_ahrs.h"

struct period_descr {
  uint32_t t0;
  uint32_t period;
};

static constexpr const char* CSV_SEP = ",";

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
  if ( !imu.setup() )
  {
    imu.set_mag_bias_correction( -4.798046398046397, 231.2808302808303, 30.96239316239316); // see notebook


    for(int i=0;i<3000;i+=500)
    {
        delay(500);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
    }
    
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


// Main loop, read and display data
void loop()
{
    uint32_t t1 = millis();

    // Process 
    if ( check_period( periodic_tasks[0].t0, periodic_tasks[0].period, t1) )
    {
        imu.read();
        imu.transform_units();
        imu.calc_euler_angles_from_accmag();  
        imu.integrate_gyro_angles(t1);
    }

    // STDOUT
    if ( check_period( periodic_tasks[1].t0, periodic_tasks[1].period, t1) )
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  

      //Serial.print (millis()); 
      //Serial.print (CSV_SEP);
      //imu.debug_print_raw_values( DEBUG_PRINT_FLAGS_ACC  | DEBUG_PRINT_FLAGS_GYRO | DEBUG_PRINT_FLAGS_MAG );
      //imu.debug_print_processed_values( DEBUG_PRINT_FLAGS_ACC  | DEBUG_PRINT_FLAGS_GYRO | DEBUG_PRINT_FLAGS_MAG );        
      //imu.debug_print_euler_angles(DEBUG_PRINT_MAGACC_EULER);
      imu.debug_print_euler_angles(DEBUG_PRINT_GYRO_EULER);
      Serial.println();
    }
}











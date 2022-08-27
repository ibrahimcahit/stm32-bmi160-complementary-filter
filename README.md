# BMI160 6 DoF IMU Library for STM32 and addition of Complementary Filter 
BMI160 6 DoF IMU Library for STM32 and addition of Complementary Filter 

Board used in project: NUCLEO-G474RE

Detailed information: [NUCLEO-G474RE](https://www.st.com/en/evaluation-tools/nucleo-g474re.html#overview)

## BMI160:

![](https://raw.githubusercontent.com/ibrahimcahit/stm32-bmi160-complementary-filter/main/bmi160.jpg)

## How to use?

On common_porting.h file, lines 21 and 27, specify your i2c port structure:

```
#define I2C_HANDLE	(hi2c1)
extern I2C_HandleTypeDef hi2c1;
```

After that, on main.c file:

Include needed librarys:

```
#include <stdio.h>
#include "bmi160_wrapper.h"
```

Create instance:

```
BMI160_t imu_t;
```

Define needed variables:

```
float aX_f32, aY_f32, aZ_f32;
float gX_f32, gY_f32, gZ_f32;
```

Init sensor and wait until init completes.

```
while (BMI160_init(imu_t) == 1);

  if (imu_t.INIT_OK_i8 != TRUE)
  {
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
  }

```

Read all sensor data:

```
bmi160ReadAccelGyro(&imu_t);
```

Access read data

```
aX_f32 = imu_t.BMI160_Ax_f32;
aY_f32 = imu_t.BMI160_Ay_f32; 
aZ_f32 = imu_t.BMI160_Az_f32;

gX_f32 = imu_t.BMI160_Gx_f32;
gY_f32 = imu_t.BMI160_Gy_f32;
gZ_f32 = imu_t.BMI160_Gz_f32;
```

## Power, Bandwidth and Sensitivity settings for BMI160

### All settings are avaliable in bmi160_wrapper.c

There are two variables for Accel and Gyro Sensitivity.

BMI160_Asens and BMI160_Gsens determines the sensitivities at which the sensors will be configured. This is for hardware-based config

BMI160_Ascale and BMI160_Gscale determines the scales which readings will be divided to get g for accel and deg/s for gyro. 

```
uint8_t BMI160_Asens = AFS_2G;
uint8_t BMI160_Gsens = GFS_1000DPS;

uint8_t BMI160_Ascale = AFS_2G;
uint8_t BMI160_Gscale = GFS_1000DPS;
```

Select the Output data rate and range of accelerometer sensor

```
sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ; 

sensor.accel_cfg.range = BMI160_Ascale_bit;
```

Select the power mode and bandwidth of accelerometer sensor

```
sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4; 
```

Select the Output data rate and range of Gyroscope sensor

```
sensor.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ; 

sensor.gyro_cfg.range = BMI160_Gscale_bit; 
```

Select the power mode and bandwidth of Gyroscope sensor

```
sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE; 
```

## Complementary Filter 

```
  timer_u64 = micros();

  // Read and process data at 1000 Hz rate

  if ( ((timer_u64 - lastTime_u64) >= 1000) && (imu_t.INIT_OK_i8 != TRUE) ) // && (PIN_LOW == 1))
  {
	  bmi160ReadAccelGyro(&imu_t);

	  aX_f32 = imu_t.BMI160_Ax_f32; // Read RAW and unscaled acceleration values from all 3 axes, unit: (g)
	  aY_f32 = imu_t.BMI160_Ay_f32; //
	  aZ_f32 = imu_t.BMI160_Az_f32; //

	  gX_f32 = imu_t.BMI160_Gx_f32 * 0.001f; // Read scaled gyro values from all 3 axes, unit: (deg/s)
	  gY_f32 = imu_t.BMI160_Gy_f32 * 0.001f; // 0.001 is 1 ms whic is represents 1000 Hz rate
	  gZ_f32 = imu_t.BMI160_Gz_f32 * 0.001f; // Multiply readings with calculation period for integration


	  gyro_pitch_f32 += gX_f32; // integrate gyro readings for pitch and roll calculation
	  gyro_roll_f32 += gY_f32;

	  gyro_pitch_f32 += gyro_roll_f32 * sin(gZ_f32 * 0.01745329f); // correct X andy Y axis readings with respect to Z axis readings
	  gyro_roll_f32 -= gyro_pitch_f32 * sin(gZ_f32 * 0.01745329f); // sin function accepts radians, 0.01745329 = pi / 180

	  acc_total_vector_f32 = sqrt((aX_f32*aX_f32)+(aY_f32*aY_f32)+(aZ_f32*aZ_f32)); // Calculate total acceleration vector

	  accel_pitch_f32 = asin(aY_f32/acc_total_vector_f32) * 57.296f; // Calculate pitch and roll respect to acceleration readings
	  accel_roll_f32 = asin(aX_f32/acc_total_vector_f32) * -57.296f;

	  accel_pitch_f32 -= 0.0f; // Corrections or acceleration calculations.
	  accel_roll_f32 -= 0.0f; // Leave 0 if accel values are ~0.0 when resting

	  // initial pitch and roll readings should be aceel-based
	  if(set_gyro_angles_u8)
	  {
		  gyro_pitch_f32 = gyro_pitch_f32 * 0.999f + accel_pitch_f32 * 0.001f; // to calculate final pitch and roll, we get most of
		  gyro_roll_f32 = gyro_roll_f32 * 0.999f + accel_roll_f32 * 0.001f;    // gyro readings and small amount of accel readings
	  }
	  else
	  {
		  gyro_pitch_f32 = accel_pitch_f32;
		  gyro_roll_f32 = accel_roll_f32;
		  set_gyro_angles_u8 = 1;
	  }

	  // integrate calculated pitch and roll with previous values
	  pitch_f32 = pitch_f32 * 0.75f + gyro_pitch_f32 * 0.25f;
	  roll_f32 = roll_f32 * 0.75f + gyro_roll_f32 * 0.25f;

	  lastTime_u64 = micros();

  }
```

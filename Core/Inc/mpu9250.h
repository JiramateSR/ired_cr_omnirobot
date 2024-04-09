#ifndef MPU9250_H__
#define MPU9250_H__

#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

typedef struct {
		double accel_x;
		double accel_y;
		double accel_z;
		double gyro_x;
		double gyro_y;
		double gyro_z;
		double temp;
} IMU_HandleTypeDef;

#define MPU9250_ADDR                  	0xD0
#define MPU9250_ADDR_CHECK							0x75
#define MPU9250_SMPLRT_DIV_REGISTER   	0x19
#define MPU9250_CONFIG_REGISTER       	0x1A
#define MPU9250_GYRO_CONFIG_REGISTER  	0x1B
#define MPU9250_ACCEL_CONFIG_REGISTER 	0x1C
#define MPU9250_PWR_MGMT_1_REGISTER   	0x6B

#define MPU9250_GYRO_OUT_REGISTER     0x43
#define MPU9250_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0
#define TEMP_LSB_OFFSET       12412.0

#define DEFAULT_GYRO_COEFF    0.98

class MPU9250 {
	public:
		MPU9250(I2C_HandleTypeDef &w);
		uint8_t begin(int gyro_config_num = 1, int acc_config_num = 0);
		uint8_t writeData(uint8_t reg, uint8_t data);

		void calcOffsets(bool is_calc_gyro = true, bool is_calc_acc = true);
		void calcGyroOffsets() {
			calcOffsets(true, false);
		}
		;
		void calcAccOffsets() {
			calcOffsets(false, true);
		}
		;

		void setAddress(uint8_t addr) {
			address = addr;
		}
		;
		uint8_t getAddress() {
			return address;
		}
		;

		uint8_t setGyroConfig(int config_num);
		uint8_t setAccConfig(int config_num);
		void setGyroOffsets(double x, double y, double z);
		void setAccOffsets(double x, double y, double z);
		void setFilterGyroCoef(double gyro_coeff);
		void setFilterAccCoef(double acc_coeff);

		double getGyroXoffset() {
			return gyroXoffset;
		}
		;
		double getGyroYoffset() {
			return gyroYoffset;
		}
		;
		double getGyroZoffset() {
			return gyroZoffset;
		}
		;

		double getAccXoffset() {
			return accXoffset;
		}
		;
		double getAccYoffset() {
			return accYoffset;
		}
		;
		double getAccZoffset() {
			return accZoffset;
		}
		;

		double getFilterGyroCoef() {
			return filterGyroCoef;
		}
		;
		double getFilterAccCoef() {
			return 1.0 - filterGyroCoef;
		}
		;

		// DATA GETTER
		double getTemp() {
			return temp;
		}
		;

		double getAccX() {
			return accX;
		}
		;
		double getAccY() {
			return accY;
		}
		;
		double getAccZ() {
			return accZ;
		}
		;

		double getGyroX() {
			return gyroX;
		}
		;
		double getGyroY() {
			return gyroY;
		}
		;
		double getGyroZ() {
			return gyroZ;
		}
		;

		double getAccAngleX() {
			return angleAccX;
		}
		;
		double getAccAngleY() {
			return angleAccY;
		}
		;

		double getAngleX() {
			return angleX;
		}
		;
		double getAngleY() {
			return angleY;
		}
		;
		double getAngleZ() {
			return angleZ;
		}
		;


		uint8_t fetchData();
		uint8_t update();

		bool upsideDownMounting = false;

	private:
		I2C_HandleTypeDef *wire;
		uint8_t address = MPU9250_ADDR;
		double gyro_lsb_to_degsec, acc_lsb_to_g;
		double gyroXoffset, gyroYoffset, gyroZoffset;
		double accXoffset, accYoffset, accZoffset;
		double temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
		double angleAccX, angleAccY;
		double angleX, angleY, angleZ;
		uint32_t preInterval;
		double filterGyroCoef;
};

#endif

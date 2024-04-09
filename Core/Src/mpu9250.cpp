#include "mpu9250.h"

static double wrap(double angle, double limit) {
	while (angle > limit)
		angle -= 2 * limit;
	while (angle < -limit)
		angle += 2 * limit;
	return angle;
}

MPU9250::MPU9250(I2C_HandleTypeDef &w) {
	wire = &w;
	setFilterGyroCoef(DEFAULT_GYRO_COEFF);
	setGyroOffsets(0, 0, 0);
	setAccOffsets(0, 0, 0);
}

uint8_t MPU9250::begin(int gyro_config_num, int acc_config_num) {
	uint8_t check, status;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	status = HAL_I2C_Mem_Read(wire, address, MPU9250_ADDR_CHECK, 1, &check, 1, 1000);
	if (check == 113) {
		writeData(MPU9250_PWR_MGMT_1_REGISTER, 0x01);
		writeData(MPU9250_SMPLRT_DIV_REGISTER, 0x00);
		writeData(MPU9250_CONFIG_REGISTER, 0x00);
		setGyroConfig(gyro_config_num);
		setAccConfig(acc_config_num);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	return status;
}

uint8_t MPU9250::writeData(uint8_t reg, uint8_t data) {
	uint8_t status = HAL_I2C_Mem_Write(wire, address, reg, 1, &data, 1, 1000);
	return status;
}

uint8_t MPU9250::setGyroConfig(int config_num) {
	uint8_t status;
	switch (config_num) {
		case 0: // range = +- 250 deg/s
			gyro_lsb_to_degsec = 131.0;
			status = writeData(MPU9250_GYRO_CONFIG_REGISTER, 0x00);
			break;
		case 1: // range = +- 500 deg/s
			gyro_lsb_to_degsec = 65.5;
			status = writeData(MPU9250_GYRO_CONFIG_REGISTER, 0x08);
			break;
		case 2: // range = +- 1000 deg/s
			gyro_lsb_to_degsec = 32.8;
			status = writeData(MPU9250_GYRO_CONFIG_REGISTER, 0x10);
			break;
		case 3: // range = +- 2000 deg/s
			gyro_lsb_to_degsec = 16.4;
			status = writeData(MPU9250_GYRO_CONFIG_REGISTER, 0x18);
			break;
		default: // error
			status = 1;
			break;
	}
	return status;
}

uint8_t MPU9250::setAccConfig(int config_num) {
	uint8_t status;
	switch (config_num) {
		case 0: // range = +- 2 g
			acc_lsb_to_g = 16384.0;
			status = writeData(MPU9250_ACCEL_CONFIG_REGISTER, 0x00);
			break;
		case 1: // range = +- 4 g
			acc_lsb_to_g = 8192.0;
			status = writeData(MPU9250_ACCEL_CONFIG_REGISTER, 0x08);
			break;
		case 2: // range = +- 8 g
			acc_lsb_to_g = 4096.0;
			status = writeData(MPU9250_ACCEL_CONFIG_REGISTER, 0x10);
			break;
		case 3: // range = +- 16 g
			acc_lsb_to_g = 2048.0;
			status = writeData(MPU9250_ACCEL_CONFIG_REGISTER, 0x18);
			break;
		default: // error
			status = 1;
			break;
	}
	return status;
}

void MPU9250::setGyroOffsets(double x, double y, double z) {
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void MPU9250::setAccOffsets(double x, double y, double z) {
	accXoffset = x;
	accYoffset = y;
	accZoffset = z;
}

void MPU9250::setFilterGyroCoef(double gyro_coeff) {
	if ((gyro_coeff < 0) or (gyro_coeff > 1)) {
		gyro_coeff = DEFAULT_GYRO_COEFF;
	}
	filterGyroCoef = gyro_coeff;
}

void MPU9250::setFilterAccCoef(double acc_coeff) {
	setFilterGyroCoef(1.0 - acc_coeff);
}

void MPU9250::calcOffsets(bool is_calc_gyro, bool is_calc_acc) {
	if (is_calc_gyro) {
		setGyroOffsets(0, 0, 0);
	}
	if (is_calc_acc) {
		setAccOffsets(0, 0, 0);
	}
	double ag[6] = { 0, 0, 0, 0, 0, 0 };

	for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
		this->fetchData();
		ag[0] += accX;
		ag[1] += accY;
		ag[2] += (accZ - 1.0);
		ag[3] += gyroX;
		ag[4] += gyroY;
		ag[5] += gyroZ;
		HAL_Delay(1);
	}

	if (is_calc_acc) {
		accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
		accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
		accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
	}

	if (is_calc_gyro) {
		gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
		gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
		gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
	}
}

uint8_t MPU9250::fetchData() {
	uint8_t i2cData[14], status;
	status = HAL_I2C_Mem_Read(wire, address, MPU9250_ACCEL_OUT_REGISTER, 1, i2cData, 14, 1000);

	int16_t rawData[7];
	int j = 0;
	for (int i = 0; i < 14; i += 2) {
		rawData[j++] = (int16_t) (i2cData[i] << 8 | i2cData[i + 1]);
	}

	accX = ((double) rawData[0]) / acc_lsb_to_g - accXoffset;
	accY = ((double) rawData[1]) / acc_lsb_to_g - accYoffset;
	accZ = (!upsideDownMounting - upsideDownMounting) * ((double) rawData[2]) / acc_lsb_to_g - accZoffset;
	temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
	gyroX = ((double) rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
	gyroY = ((double) rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
	gyroZ = ((double) rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;

	return status;
}

uint8_t MPU9250::update() {
	uint8_t status = this->fetchData();

	double sgZ = (accZ >= 0) - (accZ < 0);
	angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
	angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;

	uint32_t Tnew = HAL_GetTick();
	double dt = (Tnew - preInterval) * 1e-3;
	preInterval = Tnew;

	angleX = wrap(
			filterGyroCoef * (angleAccX + wrap(angleX + gyroX * dt - angleAccX, 180)) + (1.0 - filterGyroCoef) * angleAccX,
			180);
	angleY = wrap(
			filterGyroCoef * (angleAccY + wrap(angleY + sgZ * gyroY * dt - angleAccY, 90))
					+ (1.0 - filterGyroCoef) * angleAccY, 90);
	angleZ += gyroZ * dt;

	return status;
}

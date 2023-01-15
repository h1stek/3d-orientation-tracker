/*************************************NOTES*****************************************
 *
 * For reference see Adafruit official libraries: 
 * https://github.com/adafruit/Adafruit_9DOF/blob/master/Adafruit_9DOF.cpp
 * https://github.com/adafruit/Adafruit_LSM303DLHC
 * https://github.com/adafruit/Adafruit_L3GD20_U
 * https://github.com/adafruit/Adafruit_Sensor
 *
 * Datasheets: 
 * https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF 
 * https://cdn-shop.adafruit.com/datasheets/L3GD20.pdf
 *
 ***********************************************************************************/

#include <Wire.h>
#include "filter.h"

//#define PLOT_MODE

#ifndef PLOT_MODE
#define INTERVAL 0
#else 
#define INTERVAL 100 // time steps in ms for gnuplot
#endif

#define PRINT_ORIENTATION // define one
//#define PRINT_DATA_ACCEL
//#define PRINT_DATA_MAG
//#define PRINT_DATA_GYRO

//#define USE_EULERS // use Euler's angles instead of quaternion 

#define LSM303_ADDRESS_ACCEL 0x19
#define LSM303_REGISTER_CTRL_REG1_A 0x20	   
//#define LSM303_REGISTER_CTRL_REG4_A 0x23	
#define LSM303_REGISTER_OUT_X_L_A 0x28 | 0x80 // MSB set to 1 to enable auto-increment

#define LSM303_ADDRESS_MAG 0x1E
#define LSM303_REGISTER_MR_M 0x02 
#define LSM303_REGISTER_CRA_M 0x00
//#define LSM303_REGISTER_CRB_M 0x01
#define LSM303_REGISTER_OUT_X_H_M 0x03 // auto-increment enabled by default

#define L3GD20H_ADDRESS 0x69
#define L3GD20H_REGISTER_CTRL_REG1 0x20
//#define L3GD20H_REGISTER_CTRL_REG4 0x23 
#define L3GD20H_REGISTER_OUT_X_L 0x28 | 0x80 // MSB set to 1 to enable auto-increment

#define LINEAR_ACCELERATION_SENSITIVITY 0.001f // selected scale +/- 2g
#define GYRO_SENSITIVITY 0.00875f // selected scale 245 dps	
#define MAGNETIC_GAIN_XY 1100.0f  // selected scale +/- 1.3 gauss	
#define MAGNETIC_GAIN_Z 980.0f	  // selected scale +/- 1.3 gauss 

#define DPS_TO_RADS 0.017453293f // degrees/s to rad/s multiplier
#define RAD_TO_DGS 180 / M_PI	 // radians to degrees multiplier

static int xlo, xhi, ylo, yhi, zlo, zhi;
static int raw_x, raw_y, raw_z;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
static float offset_ax, offset_ay, offset_az;
static float offset_gx, offset_gy, offset_gz;
static float accel_roll, accel_pitch;
static float gyro_angle_x, gyro_angle_y, gyro_angle_z;

float dt;
static int time;
static unsigned long time_start, time_print,  time_end;

static float roll, pitch, yaw;  // euler's angle attitude representation
static quat q;                  // quaternion attitude representation 

void setup()
{
	Wire.begin();
	Serial.begin(115200);
	delay(100);

	if (!start_sensor(LSM303_ADDRESS_ACCEL)) {
		Serial.print("Accelometer failed to start.\n");
		while (1);
	}

	if (!start_sensor(LSM303_ADDRESS_MAG)) {
		Serial.print("Magnetometer failed to start.\n");
		while (1);
	}

	if (!start_sensor(L3GD20H_ADDRESS)) {
		Serial.print("Gyroscope failed to start.\n");
		while (1);
	}

	calib(); 
}

void loop()
{
	time_start = millis();
	read_acc();
	read_mag();
	read_gyro();
	
	if (time_start - time_print > INTERVAL) {
#ifdef PLOT_MODE
	Serial.print(time);
	Serial.print(" ");
#endif
	time_print = time_start;	
#if defined(PRINT_ORIENTATION) && defined(USE_EULERS)
	print(roll, pitch, yaw);
#elif defined(PRINT_ORIENTATION)
	print(q.x, q.y, q.z);
#elif defined(PRINT_DATA_ACCEL)
	print(ax, ay, az);
#elif defined(PRINT_DATA_MAG)
	print(mx, my, mz);
#elif defined(PRINT_DATA_GYRO)
	print(gx, gy, gz);
#endif
	}

#if defined(PRINT_ORIENTATION) && defined(USE_EULERS)
	get_roll();
	get_pitch();
	get_yaw();
	delay(50); // wait for visualization software
#elif defined(PRINT_ORIENTATION)
	get_quaternion();
	delay(15); // wait for visualization software
#endif
	time_end = millis();
	dt = (float)(time_end - time_start) * 10E-4;
}

void print(float x, float y, float z)
{
#ifndef USE_EULERS  
	Serial.print(q.w);
	Serial.print(" ");
#endif
	Serial.print(x);
	Serial.print(" ");
	Serial.print(y);
	Serial.print(" ");
	Serial.println(z);
}

bool start_sensor(byte sensor_i2c_address)
{
	byte reg_value;

	switch (sensor_i2c_address) {
	case LSM303_ADDRESS_ACCEL:
		// set normal mode 100 hz
		write_byte(sensor_i2c_address, LSM303_REGISTER_CTRL_REG1_A, 0x57);

		// check if powered up
		reg_value = read_byte(sensor_i2c_address, LSM303_REGISTER_CTRL_REG1_A);
		if (reg_value != 0x57)
			return false;

		break;
	case LSM303_ADDRESS_MAG:
		// set continous mode
		write_byte(sensor_i2c_address, LSM303_REGISTER_MR_M, 0x00);

		// check if powered up
		reg_value =
			read_byte(sensor_i2c_address, LSM303_REGISTER_CRA_M);
		if (reg_value != 0x10)
			return false;
		
		break;
	case L3GD20H_ADDRESS:
		// set normal mode 95 hz
		write_byte(sensor_i2c_address, L3GD20H_REGISTER_CTRL_REG1, 0x0F);

		// check if powered up
		reg_value = read_byte(sensor_i2c_address, L3GD20H_REGISTER_CTRL_REG1);
		if (reg_value != 0x0F)
			return false;
	}
	return true;
}

// read sensor output
void read_sensor(byte sensor_i2c_address, byte output_register)
{
	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(output_register);
	Wire.endTransmission();

	Wire.requestFrom(sensor_i2c_address, (uint8_t)6);

	while (Wire.available() < 6);

	switch (sensor_i2c_address) {
	case LSM303_ADDRESS_ACCEL:
		xlo = Wire.read();
		xhi = Wire.read();
		ylo = Wire.read();
		yhi = Wire.read();
		zlo = Wire.read();
		zhi = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8)) >> 4;
		raw_y = (int16_t)(ylo | (yhi << 8)) >> 4;
		raw_z = (int16_t)(zlo | (zhi << 8)) >> 4;

		// data in g's
		ax = (float)raw_x * LINEAR_ACCELERATION_SENSITIVITY;
		ay = (float)raw_y * LINEAR_ACCELERATION_SENSITIVITY;
		az = (float)raw_z * LINEAR_ACCELERATION_SENSITIVITY;

		// zero level bias correction
		ax -= offset_ax;
		ay -= offset_ay;
		az -= offset_az;
		
		break;
	case LSM303_ADDRESS_MAG:
		xhi = Wire.read();
		xlo = Wire.read();
		zhi = Wire.read();
		zlo = Wire.read();
		yhi = Wire.read();
		ylo = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8));
		raw_y = (int16_t)(ylo | (yhi << 8));
		raw_z = (int16_t)(zlo | (zhi << 8));

		// data in Gauss
		mx = (float)raw_x / MAGNETIC_GAIN_XY;
		my = (float)raw_y / MAGNETIC_GAIN_XY;
		mz = (float)raw_z / MAGNETIC_GAIN_Z;
		
		break;
	case L3GD20H_ADDRESS:
		xlo = Wire.read();
		xhi = Wire.read();
		ylo = Wire.read();
		yhi = Wire.read();
		zlo = Wire.read();
		zhi = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8));
		raw_y = (int16_t)(ylo | (yhi << 8));
		raw_z = (int16_t)(zlo | (zhi << 8));

		// angular velocities deg/s
		gx = (float)raw_x * GYRO_SENSITIVITY;
		gy = (float)raw_y * GYRO_SENSITIVITY;
		gz = (float)raw_z * GYRO_SENSITIVITY;

#ifdef USE_EULERS
		gyro_angle_x += gx * dt;
		gyro_angle_y += gy * dt;
		gyro_angle_z += gz * dt;
#else
		gx *= DPS_TO_RADS;
		gy *= DPS_TO_RADS;
		gz *= DPS_TO_RADS;
#endif
		// zero level bias correction
		gx -= offset_gx;
		gy -= offset_gy;
		gz -= offset_gz;
		
		break;
	}
}

void read_acc()
{
	read_sensor(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_OUT_X_L_A);
}

void read_mag()
{
	read_sensor(LSM303_ADDRESS_MAG, LSM303_REGISTER_OUT_X_H_M);
}

void read_gyro()
{
	read_sensor(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_X_L);
}

// resource for implementing the complementary filter:
// https://web.archive.org/web/20091121085323/http://www.mikroquad.com/bin/view/Research/ComplementaryFilter
void get_roll()
{
	accel_roll = atan2(ay, sqrt(ax * ax + az * az));
	gyro_angle_x = 0.96 * gyro_angle_x + 0.04 * (accel_roll * RAD_TO_DGS);
	roll = gyro_angle_x;
}

void get_pitch()
{
	accel_pitch = atan2(-1.0f * ax, sqrt(ay * ay + az * az));
	gyro_angle_y = 0.96 * gyro_angle_y + 0.04 * (accel_pitch * RAD_TO_DGS);
	pitch = gyro_angle_y;
}

// no yaw drift correction implemented
void get_yaw()
{
	yaw = gyro_angle_z;
}

void get_quaternion()
{
	q = update_filter(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
}

void tilt_compensation(float roll, float pitch)
{
	//TBD
}

// calculate zero-rate levels
void calib()
{
	//=================
	// ACCELEROMETER
	//=================

	int n = 100;
	static float sumx, sumy, sumz;

	for (int i = 1; i <= n; i++) {
		read_acc();
		sumx += ax;
		sumy += ay;
		sumz += az;
	}

	offset_ax = (float)sumx / n;
	offset_ay = (float)sumy / n;
	offset_az = 1 - ((float)sumz / n);

	//=================
	// GYROSCOPE
	//=================

	sumx = 0;
	sumy = 0;
	sumz = 0;

	for (int i = 1; i <= n; i++) {
		read_gyro();
		sumx += gx;
		sumy += gy;
		sumz += gz;
	}

	offset_gx = (float)sumx / n;
	offset_gy = (float)sumy / n;
	offset_gz = (float)sumz / n;

	//=================
	// MAGNETOMETER
	//=================
	
	// not implemented
}

void get_hard_iron()
{
	// TBD
}

void get_soft_iron()
{
	// TBD
}

void write_byte(byte sensor_i2c_address, byte reg, byte value)
{
	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

byte read_byte(byte sensor_i2c_address, byte reg)
{
	byte value;

	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(sensor_i2c_address, (uint8_t)1);

	while (Wire.available() < 1);

	value = Wire.read();

	return value;
}

#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define XGOFFS_TC          0x00
#define YGOFFS_TC          0x01
#define ZGOFFS_TC          0x02
#define X_FINE_GAIN        0x03
#define Y_FINE_GAIN        0x04
#define Z_FINE_GAIN        0x05
#define XA_OFFSET_H        0x06
#define XA_OFFSET_L_TC     0x07
#define YA_OFFSET_H        0x08
#define YA_OFFSET_L_TC     0x09
#define ZA_OFFSET_H        0x0A
#define ZA_OFFSET_L_TC     0x0B
#define SELF_TEST_X        0x0D
#define SELF_TEST_Y        0x0E
#define SELF_TEST_Z        0x0F
#define SELF_TEST_A        0x10
#define XG_OFFS_USRH       0x13
#define XG_OFFS_USRL       0x14
#define YG_OFFS_USRH       0x15
#define YG_OFFS_USRL       0x16
#define ZG_OFFS_USRH       0x17
#define ZG_OFFS_USRL       0x18
#define SMPLRT_DIV         0x19
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define FF_THR             0x1D
#define FF_DUR             0x1E
#define MOT_THR            0x1F
#define MOT_DUR            0x20
#define ZMOT_THR           0x21
#define ZRMOT_DUR          0x22
#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A
#define PWR_MGMT_1         0x6B
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D
#define DMP_RW_PNT         0x6E
#define DMP_REG            0x6F
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU6050   0x75
#define MPU6050_ADDRESS    0x68

#define AFS_2G                0
#define AFS_4G                1
#define AFS_8G                2
#define AFS_16G               3

#define GFS_250DPS            0
#define GFS_500DPS            1
#define GFS_1000DPS           2
#define GFS_2000DPS           3

/* accelerometer and gyroscope scale */
#define A_SCALE                AFS_2G
#define G_SCALE                GFS_250DPS

/* accelerometer and gyroscope resolution */
#if A_SCALE == AFS_2G
#define A_RES                  (2.0f / 32768.0f)
#elif A_SCALE == AFS_4G
#define A_RES                  (4.0f / 32768.0f)
#elif A_SCALE == AFS_8G
#define A_RES                  (8.0f / 32768.0f)
#elif A_SCALE == AFS_16G
#define A_RES                  (16.0f / 32768.0f)
#endif

#if G_SCALE == GFS_250DPS
#define G_RES                  (250.0f / 32768.0f)
#elif G_SCALE == GFS_500DPS
#define G_RES                  (500.0f / 32768.0f)
#elif G_SCALE == GFS_1000DPS
#define G_RES                  (1000.0f / 32768.0f)
#elif G_SCALE == GFS_2000DPS
#define G_RES                  (2000.0f / 32768.0f)
#endif

static float beta, zeta;

/* real acceleration values in g's */
static float ax, ay, az;

/* real gyroscope values in degrees per second */
static float gx, gy, gz;

/* accelerometer and gyroscope bias corrections */
static float accel_bias[3] = { 0, 0, 0 }, gyro_bias[3] = { 0, 0, 0 };
static float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float pitch, yaw, roll, temperature;

static int16_t pitch_deg = 0, roll_deg = 0;

static float time_delta = 0.0f;
static uint32_t last_update, now;

static volatile uint32_t timer0_overflow_count;
uint32_t us(void);

static void mpu6050_self_test(float *dst);
static void mpu6050_calibrate(float *gyro_dst, float *accel_dst);
static void mpu6050_prepare(void);
static void i2c_write_byte(uint8_t addr, uint8_t cmd, uint8_t data);
static uint8_t i2c_read_byte(uint8_t addr, uint8_t cmd);
static void i2c_read_bytes
	(uint8_t addr, uint8_t cmd, uint8_t cnt, uint8_t *dst);

static void quaternion_update
		(float ax, float ay, float az, float gx, float gy, float gz);

static void mpu6050_self_test(float *dst)
{
	uint8_t i, raw[4], self_test[6];
	float factory_trim[6];

	i2c_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0);
	i2c_write_byte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0);
	_delay_ms(250);

	raw[0] = i2c_read_byte(MPU6050_ADDRESS, SELF_TEST_X);
	raw[1] = i2c_read_byte(MPU6050_ADDRESS, SELF_TEST_Y);
	raw[2] = i2c_read_byte(MPU6050_ADDRESS, SELF_TEST_Z);
	raw[3] = i2c_read_byte(MPU6050_ADDRESS, SELF_TEST_A);

	self_test[0] = (raw[0] >> 3) | (raw[3] & 0x30) >> 4;
	self_test[1] = (raw[1] >> 3) | (raw[3] & 0x0C) >> 2;
	self_test[2] = (raw[2] >> 3) | (raw[3] & 0x03);

	self_test[3] = raw[0] & 0x1F;
	self_test[4] = raw[1] & 0x1F;
	self_test[5] = raw[2] & 0x1F;

	factory_trim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)self_test[0] - 1.0) / 30.0)));
	factory_trim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)self_test[1] - 1.0) / 30.0)));
	factory_trim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)self_test[2] - 1.0) / 30.0)));
	factory_trim[3] = (25.0 * 131.0) * (pow(1.046, ((float)self_test[3] - 1.0)));
	factory_trim[4] = (-25.0 * 131.0) * (pow(1.046, ((float)self_test[4] - 1.0)));
	factory_trim[5] = ( 25.0 * 131.0) * (pow(1.046, ((float)self_test[5] - 1.0)));

	for(i = 0; i < 6; ++i)
	{
		dst[i] = 100.0 + 100.0 * ((float)self_test[i] - factory_trim[i]) / factory_trim[i];
	}
}

static void mpu6050_calibrate(float *gyro_dst, float *accel_dst)
{
	uint8_t i, data[12];
	uint16_t packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	uint32_t mask = 1UL;
	uint8_t mask_bit[3] = {0, 0, 0};
	uint16_t gyro_sensitivity  = 131, accel_sensitivity = 16384;
	int32_t accel_bias_reg[3] = {0, 0, 0};

	i2c_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
	_delay_ms(100);

	i2c_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	i2c_write_byte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
	_delay_ms(200);

	i2c_write_byte(MPU6050_ADDRESS, INT_ENABLE, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x0C);
	_delay_ms(15);

	i2c_write_byte(MPU6050_ADDRESS, CONFIG, 0x01);
	i2c_write_byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);
	i2c_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);

	i2c_write_byte(MPU6050_ADDRESS, USER_CTRL, 0x40);
	i2c_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x78);
	_delay_ms(80);

	i2c_write_byte(MPU6050_ADDRESS, FIFO_EN, 0x00);
	i2c_read_bytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;

	for(i = 0; i < packet_count; ++i)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		i2c_read_bytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]);
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);
		accel_bias[0] += (int32_t)accel_temp[0];
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}

	accel_bias[0] /= (int32_t)packet_count;
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;

	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if(accel_bias[2] > 0L)
	{
		accel_bias[2] -= (int32_t)accel_sensitivity;
	}
	else
	{
		accel_bias[2] += (int32_t)accel_sensitivity;
	}

	data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	data[1] = (-gyro_bias[0] / 4) & 0xFF;
	data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	i2c_write_byte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
	i2c_write_byte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
	i2c_write_byte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
	i2c_write_byte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
	i2c_write_byte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
	i2c_write_byte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

	gyro_dst[0] = (float)gyro_bias[0] / (float)gyro_sensitivity;
	gyro_dst[1] = (float)gyro_bias[1] / (float)gyro_sensitivity;
	gyro_dst[2] = (float)gyro_bias[2] / (float)gyro_sensitivity;

	i2c_read_bytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	i2c_read_bytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	i2c_read_bytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	for(i = 0; i < 3; ++i)
	{
		if(accel_bias_reg[i] & mask)
		{
			mask_bit[i] = 0x01;
		}
	}

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0];
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1];
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2];

	i2c_write_byte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
	i2c_write_byte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
	i2c_write_byte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
	i2c_write_byte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
	i2c_write_byte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
	i2c_write_byte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

	accel_dst[0] = (float)accel_bias[0] / (float)accel_sensitivity;
	accel_dst[1] = (float)accel_bias[1] / (float)accel_sensitivity;
	accel_dst[2] = (float)accel_bias[2] / (float)accel_sensitivity;
}

static void mpu6050_prepare(void)
{
	uint8_t c;
	i2c_write_byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	i2c_write_byte(MPU6050_ADDRESS, CONFIG, 0x03);
	i2c_write_byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);
	c = i2c_read_byte(MPU6050_ADDRESS, GYRO_CONFIG);
	i2c_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0);
	i2c_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);
	i2c_write_byte(MPU6050_ADDRESS, GYRO_CONFIG, c | G_SCALE << 3);
	c = i2c_read_byte(MPU6050_ADDRESS, ACCEL_CONFIG);
	i2c_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0);
	i2c_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);
	i2c_write_byte(MPU6050_ADDRESS, ACCEL_CONFIG, c | A_SCALE << 3);
	i2c_write_byte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
	i2c_write_byte(MPU6050_ADDRESS, INT_ENABLE, 0x01);
}

static void i2c_write_byte(uint8_t addr, uint8_t cmd, uint8_t data)
{
	i2c_begin_transmission(addr);
	i2c_write(cmd);
	i2c_write(data);
	i2c_end_transmission();
}

static uint8_t i2c_read_byte(uint8_t addr, uint8_t cmd)
{
	i2c_begin_transmission(addr);
	i2c_write(cmd);
	i2c_end_transmission();
	i2c_request_from(addr, 1);
	return i2c_read();
}

static void i2c_read_bytes
	(uint8_t addr, uint8_t cmd, uint8_t cnt, uint8_t *dst)
{
	uint8_t i = 0;
	i2c_begin_transmission(addr);
	i2c_write(cmd);
	i2c_end_transmission();
	i2c_request_from(addr, cnt);
	while(i2c_available())
	{
		dst[i++] = i2c_read();
	}
}

static void quaternion_update
		(float ax, float ay, float az, float gx, float gy, float gz)
{
	float q1, q2, q3, q4, norm, f1, f2, f3,
		g_err_x, g_err_y, g_err_z, g_bias_x, g_bias_y, g_bias_z,
		_2q1, _2q2, _2q3, _2q4,
		hat_dot1, hat_dot2, hat_dot3, hat_dot4,
		j11or24, j12or23, j13or22, j14or21, j32, j33,
		q_dot1, q_dot2, q_dot3, q_dot4,
		half_q1, half_q2, half_q3, half_q4;

	q1 = q[0];
	q2 = q[1];
	q3 = q[2];
	q4 = q[3];

	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q4 = 2.0f * q4;

	/* normalize accelerometer movement */
	norm = sqrt(ax * ax + ay * ay + az * az);
	if(norm == 0.0f)
	{
		return;
	}

	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	/* compute the objective function and jacobian */
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	j11or24 = _2q3;
	j12or23 = _2q4;
	j13or22 = _2q1;
	j14or21 = _2q2;
	j32 = 2.0f * j14or21;
	j33 = 2.0f * j11or24;

	/* compute the gradient */
	hat_dot1 = j14or21 * f2 - j11or24 * f1;
	hat_dot2 = j12or23 * f1 + j13or22 * f2 - j32 * f3;
	hat_dot3 = j12or23 * f2 - j33 *f3 - j13or22 * f1;
	hat_dot4 = j14or21 * f1 + j11or24 * f2;

	/* normalize gradient */
	norm = sqrt(hat_dot1 * hat_dot1 + hat_dot2 * hat_dot2 + hat_dot3 * hat_dot3 + hat_dot4 * hat_dot4);
	hat_dot1 /= norm;
	hat_dot2 /= norm;
	hat_dot3 /= norm;
	hat_dot4 /= norm;

	/* compute gyroscope biases */
	g_err_x = _2q1 * hat_dot2 - _2q2 * hat_dot1 - _2q3 * hat_dot4 + _2q4 * hat_dot3;
	g_err_y = _2q1 * hat_dot3 + _2q2 * hat_dot4 - _2q3 * hat_dot1 - _2q4 * hat_dot2;
	g_err_z = _2q1 * hat_dot4 - _2q2 * hat_dot3 + _2q3 * hat_dot2 - _2q4 * hat_dot1;

	/* remove gyroscope biases */
	g_bias_x = g_err_x * time_delta * zeta;
	g_bias_y = g_err_y * time_delta * zeta;
	g_bias_z = g_err_z * time_delta * zeta;
	gx -= g_bias_x;
	gy -= g_bias_y;
	gz -= g_bias_z;

	half_q1 = 0.5f * q1;
	half_q2 = 0.5f * q2;
	half_q3 = 0.5f * q3;
	half_q4 = 0.5f * q4;

	/* compute quaternion derivative */
	q_dot1 = -half_q2 * gx - half_q3 * gy - half_q4 * gz;
	q_dot2 = half_q1 * gx + half_q3 * gz - half_q4 * gy;
	q_dot3 = half_q1 * gy - half_q2 * gz + half_q4 * gx;
	q_dot4 = half_q1 * gz + half_q2 * gy - half_q3 * gx;

	/* integrate quaternion derivative */
	q1 += (q_dot1 - (beta * hat_dot1)) * time_delta;
	q2 += (q_dot2 - (beta * hat_dot2)) * time_delta;
	q3 += (q_dot3 - (beta * hat_dot3)) * time_delta;
	q4 += (q_dot4 - (beta * hat_dot4)) * time_delta;

	/* normalize quaternion */
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

uint8_t mpu6050_init(void)
{
	float self_test[6];
	uint8_t c;
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);
	TIMSK0 = (1 << TOIE0);
	beta = sqrt(3.0f / 4.0f) * (M_PI * (40.0f / 180.0f));
	zeta = sqrt(3.0f / 4.0f) * (M_PI * (2.0f / 180.0f));
	if((c = i2c_read_byte(MPU6050_ADDRESS, WHO_AM_I_MPU6050)) != 0x68)
	{
		return 1;
	}

	mpu6050_self_test(self_test);
	if(!(self_test[0] < 1.0f && self_test[1] < 1.0f &&
		self_test[2] < 1.0f && self_test[3] < 1.0f &&
		self_test[4] < 1.0f && self_test[5] < 1.0f))
	{
		return 1;
	}

	mpu6050_calibrate(gyro_bias, accel_bias);
	mpu6050_prepare();
	return 0;
}

void mpu6050_update(void)
{
	/* check if new data is available */
	if(i2c_read_byte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
	{
		/* get accelerometer, gyroscope and temperature data */
		uint8_t raw[14];
		i2c_read_bytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 14, raw);
		ax = (float)((int16_t)((raw[0] << 8) | raw[1])) * A_RES;
		ay = (float)((int16_t)((raw[2] << 8) | raw[3])) * A_RES;
		az = (float)((int16_t)((raw[4] << 8) | raw[5])) * A_RES;
		temperature = ((float)(((int16_t)raw[6]) << 8 | raw[7])) / 340.0f + 36.53f;
		gx = (float)((int16_t)((raw[8] << 8) | raw[9])) * G_RES;
		gy = (float)((int16_t)((raw[10] << 8) | raw[11])) * G_RES;
		gz = (float)((int16_t)((raw[12] << 8) | raw[13])) * G_RES;
	}

	now = us();
	time_delta = ((now - last_update) / 1000000.0f);
	last_update = now;
	quaternion_update(ax, ay, az, gx * M_PI / 180.0f, gy * M_PI / 180.0f, gz * M_PI / 180.0f);
	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0f / M_PI;
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180.0f / M_PI;
	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / M_PI;
	pitch_deg = (int16_t)pitch;
	roll_deg = (int16_t)roll;
}

/* timer */
uint32_t us(void)
{
	uint32_t m;
	uint8_t sreg = SREG, t;

	cli();
	m = timer0_overflow_count;
	t = TCNT0;
	if((TIFR0 & (1 << TOV0)) && (t < 255))
	{
		m++;
	}

	SREG = sreg;
	return ((m << 8) + t) * (64 / (F_CPU / 1000000));
}

ISR(TIMER0_OVF_vect)
{
	++timer0_overflow_count;
}


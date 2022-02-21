#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "bmi160.h"




static const char *device = "/dev/spidev2.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 50000;
static uint16_t delay;
static int fd = -1;

void spi_delay(uint32_t period) {
	// Do nothing
}

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static int8_t spi_transfer(int fd, uint8_t * tx, uint8_t * rx, uint16_t len) {
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	return BMI160_OK;
}

static int8_t spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t * tx = malloc(1 + len);
	int8_t ret = 0;
	//tx[0] = reg_addr >> 1;
	tx[0] = reg_addr;
	memcpy(tx + 1, data, len);
	ret = spi_transfer(fd, tx, NULL, 1 + len);
	usleep(50000); //Wait for loading the data
	free(tx);
	return ret;
}


static int8_t spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t * tx = malloc(len);
	uint8_t * rx = malloc(len);
	int8_t ret = 0;
	memset(tx, 0xFF, len);
	tx[0] = reg_addr;
	ret = spi_transfer(fd, tx, rx, len);
	memcpy(data, rx + 1, len - 1); // len -1 => remove 1 dummy byte
	free(tx);
	free(rx);
	return ret;
}

int main(int argc, char argv[]) {

	int ret = 0;

	fd = open(device, O_RDWR);

	if (fd < 0) pabort("can't open device");

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) pabort("can't get spi mode");

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't get bits per word");

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't get max speed hz");

	struct bmi160_dev sensor;
	sensor.id = 0;
	sensor.interface = BMI160_SPI_INTF;
	sensor.delay_ms = spi_delay;
	sensor.write = spi_write;
	sensor.read = spi_read;
	int8_t rslt = BMI160_OK;
	rslt = bmi160_init(&sensor);
	if (rslt != BMI160_OK) pabort("Init Module Failed");
	
	/*!
	After self test, all register values are overwritten with "default parameters".
	*/
	rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);
	if (rslt == BMI160_OK) {
		printf("OK\n");
	} else if (rslt > BMI160_OK) {
		printf("Test Faild. Check the Failed Value:%d\n",rslt);
	} else {
		pabort("Self Test Error!!");
	}


	/*Set the Accel power mode as normal mode*/
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	/*Select the power mode of Gyroscope sensor*/
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/*Setting the sensor config*/
	rslt = bmi160_set_sens_conf(&sensor);
	if (rslt != BMI160_OK) pabort("Setting Sensor Conf Failed");

	uint8_t data=0;
	uint16_t len = 2;
	/*Read Chip ID*/
	rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, &data, len, &sensor);
	printf("CHIP ID:%02x\n",data);
	/*Read PMU Status*/
	rslt = bmi160_get_regs(BMI160_PMU_STATUS_ADDR , &data, len, &sensor);
	printf("PMU_STATUS:%02x\n",data);

	/*Show Accel and Gyro data*/
	struct bmi160_sensor_data bmi160_accel;
	struct bmi160_sensor_data bmi160_gyro;
	int times_to_read = 0;
	//Show 50 points of data.
	while (times_to_read < 50) {
		/* To read both Accel and Gyro data */
		rslt = bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &bmi160_accel, &bmi160_gyro, &sensor);
		//rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &bmi160_accel, NULL, &sensor);// only Accel data
		//rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, NULL, &bmi160_gyro, &sensor);// only Gyro data
		
		if(rslt == BMI160_OK) {
			printf("OK get data ; ");
		} else {
			printf("FAILED get data ; ");
		}

		printf("ax:%d\tay:%d\taz:%d\t", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);
		printf("gx:%d\tgy:%d\tgz:%d\n", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);
		fflush(stdout);
		usleep(100000);
		times_to_read = times_to_read + 1;
	}

	close(fd);
	return 0;
}
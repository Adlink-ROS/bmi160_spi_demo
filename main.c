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

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
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

static int8_t spi_transfer(int fd, uint8_t * tx, uint8_t * rx) {
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
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
	ret = spi_transfer(fd, tx, NULL);
	free(tx);
	return ret;
}

static int8_t spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t * tx = malloc(len);
	uint8_t * rx = malloc(len);
	int8_t ret = 0;
	memset(tx, 0xFF, len);
	tx[0] = reg_addr;
	ret = spi_transfer(fd, tx, rx);
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
	rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);
	if (rslt >= BMI160_OK) {
		printf("OK\n");
	} else {
		printf("FAILED\n");
	}
	close(fd);
	return 0;
}

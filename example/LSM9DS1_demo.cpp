#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"

class LSM9DS1printCallback : public LSM9DS1callback {
	virtual void hasSample(float gx,
			       float gy,
			       float gz,
			       float ax,
			       float ay,
			       float az,
			       float mx,
			       float my,
			       float mz) {
		printf("Gyro:\t%3.10f,\t%3.10f,\t%3.10f [deg/s]\n", gx, gy, gz);
		printf("Accel:\t%3.10f,\t%3.10f,\t%3.10f [Gs]\n", ax, ay, az);
		printf("Mag:\t%3.10f,\t%3.10f,\t%3.10f [gauss]\n", mx, my, mz);
		printf("\n");
	}
};

int main(int argc, char *argv[]) {
    LSM9DS1 imu;
    LSM9DS1printCallback callback;
    imu.setCallback(&callback);
    imu.begin();
    do {
	sleep(1);
    } while (getchar() < 10);
    imu.end();
    exit(EXIT_SUCCESS);
}
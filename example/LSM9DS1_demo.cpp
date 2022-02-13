#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"

class LSM9DS1printCallback : public LSM9DS1callback {
	virtual void hasSample(LSM9DS1Sample s) {
		printf("Gyro:\t%3.10f,\t%3.10f,\t%3.10f [deg/s]\n", s.gx, s.gy, s.gz);
		printf("Accel:\t%3.10f,\t%3.10f,\t%3.10f [Gs]\n", s.ax, s.ay, s.az);
		printf("Mag:\t%3.10f,\t%3.10f,\t%3.10f [gauss]\n", s.mx, s.my, s.mz);
		printf("\n");
	}
};

int main(int argc, char *argv[]) {
    fprintf(stderr,"Press <RETURN> any time to stop the acquisition.\n");
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

#include "NED.h"
#include <sys/time.h>
#include "Qt/inc/QtWidgets/QApplication"
#include <iostream>
using namespace std;
int main()
{
    fprintf(stderr,"Press <RETURN> any time to stop the acquisition.\n");
        LSM9DS1 imu;
        NED callback;
        imu.setCallback(&callback);
        GyroSettings gyroSettings;
        gyroSettings.sampleRate = GyroSettings::G_ODR_14_9; // 14.9Hz for acc and gyr
        cout<<"block3"<<endl;
        imu.begin(gyroSettings);
        cout<<"block4"<<endl;
        
        imu.end();
        exit(EXIT_SUCCESS);
}

#include "mainwindow.h"

#include <QApplication>
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"
#include "ECEF.h"
#include "Eigen/Dense"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.InitMainWindow();
    w.showMaximized();
    LSM9DS1 imu;
    /*
    Eigen::Vector3d P;
    ECEF callback(55.86,-4.294,0);
    imu.setCallback(&callback);
    callback.setCallBack(&w);
    GyroSettings gyroSettings;
    gyroSettings.sampleRate = GyroSettings::G_ODR_14_9; // 14.9Hz for acc and gyr
    imu.begin(gyroSettings);
    */

    return a.exec();
}

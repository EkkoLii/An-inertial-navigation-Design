#ifndef __ECEF_H__
#define __ECEF_H__

#include "LSM9DS1.h"
#include <math.h>
#include "Eigen/Dense"
#include "update_trigger.h"


/**
 * Callback interface where the callback needs to be
 * implemented by the host application.
 **/
class ECEFcallback {
public:
        /**
         * Called after a sample has arrived.
         **/
        virtual void GetPosition(Eigen::Vector3d Position) = 0;
};

class ECEF : public LSM9DS1callback 
{
public:
    double wie = 7.2921150e-5; //sideral angular rate 
    double e = 0.0818191908425;//the eccentricity of earth
    double R0= 6378137;//the radius of equator
    double d2r = M_PI/180;

    /**
     * variables defined for coordinate transformation
     **/
    Eigen::Matrix3d Cbe; //DCM from body axis to ECEF
    Eigen::Matrix3d Cen; //DCM from body axis to ECEF
    Eigen::Matrix3d Cne; //DCM from body axis to ECEF
    Eigen::Matrix3d Cpos_pre; 
    Eigen::Matrix3d skew_alpha;
    Eigen::Matrix3d Cbn {{ cos(M_PI/2), sin(M_PI/2), 0},
                         {-sin(M_PI/2), cos(M_PI/2), 0},
                         {0, 0, 1}};//Initialing the DCM
    Eigen::Matrix3d I {{1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1}}; //Initialising the unit matrix
    Eigen::Matrix3d Om_ie_e {{0, -wie_e(2), wie_e(1)},
                             {wie_e(2), 0, -wie_e(0)},
                             {-wie_e(1), wie_e(0), 0}}; //skew matrix of wie_e
    
    Eigen::Vector3d wib_b;
    Eigen::Vector3d wie_e {{0,0,wie}};
    Eigen::Vector3d fib_b;
    Eigen::Vector3d alpha; //vector of incremental angles, alpha_bn_n = dt*wbn_n
    Eigen::Vector3d v_eb_e {{0,0,0}};;
    Eigen::Vector3d v_eb_e_;
    Eigen::Vector3d vd_eb_e {{0,0,0}};
    Eigen::Vector3d vd_eb_e_;
    Eigen::Vector3d r_eb_e; //the displcesment of IMU
    Eigen::Vector3d Pos; // the position in Map.
    Eigen::Vector3d Pos_0; //the last position GPS provided in cartesian coordinate.
    Eigen::Vector3d g_0; //the somigliana model
    Eigen::Vector3d ba {{1, 0.8, 7}}; //the bias of accelerometer
    Eigen::Vector3d bg {{0.5, 0.5, 0.5}}; //the bias of accelerometer
    ECEFcallback* PositionCallback = nullptr;

    double norm_alpha;
    double dt = 0;

    ECEF(double Lb, double longi, double hb);
    virtual void hasSample(LSM9DS1Sample sample);

    void DCMUpdate();
    void VelocityUpdate();
    void PositionUpdate();
    double RE(double Lb);
    double RN(double Lb);
    void PositionReady();
    void BiasRemove();
    Eigen::Vector3d Getvelocity();
    Eigen::Vector3d GetPosition();

private:
    update_trigger* callback;
public:
    void setCallBack(update_trigger* callback)
    {
        this->callback=callback;
        return;
    }
};
#endif 

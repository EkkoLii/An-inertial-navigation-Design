#include "NED.h"
#include <math.h>



void NED::hasSample(LSM9DS1Sample sample) {
    fib_b <<sample.ax, sample.ay, sample.az;
    wib_b <<sample.gx, sample.gy, sample.gz;
    dt = sample.dt;
    DCMUpdate();
    ForceFrameTrans();
    VelocityUpdate();
    PositionUpdate();
}
void NED::DCMUpdate(){
    Eigen::MatrixXd I;
    I.Identity(3,3);
    wie_n << wie*cos(lati), 0, -wie*sin(lati);
    Om_ie_n << 0, -wie_n(2), 0,
               wie_n(2), 0,-wie_n(0),
               0, wie_n(0), 0;

    wen_n << veb_n(1)/(RE(lati)+hb), -veb_n(0)/(RN(lati)+hb), -veb_n(1)*tan(lati)/(RE(lati)+hb);
    Om_en_n << 0, -wen_n(2), wen_n(1),
               wen_n(2), 0,-wen_n(0),
               -wen_n(1), wen_n(0), 0;
    alpha = dt*(Cbn*wib_b - wie_n - wen_n);//om_nb_b incremental of angles
    norm_alpha = alpha.norm(); //normalise the value of incremental angles
    skew_alpha << 0,-alpha(2),alpha(1),
                  alpha(2),0,-alpha(0),
                  -alpha(1),alpha(0),0; //skew matrix of alpha
    Cpos_pri = I + skew_alpha*sin(norm_alpha)/norm_alpha + skew_alpha*skew_alpha*(1-cos(norm_alpha))/pow(norm_alpha,2);
    Cbn = Cbn*Cpos_pri; 
}
void NED::ForceFrameTrans(){
    fib_n = Cbn*fib_b;
}
void NED::VelocityUpdate(){
    vd_priori = vd;
    veb_priori = veb_n;
    vd = fib_n - (Om_en_n +2*Om_ie_n)*veb_priori;
    
    veb_n = veb_priori + dt*(vd+vd_priori)/2;
}
void NED::PositionUpdate(){
    lati = lati + dt*(veb_n(0)/(RE(lati)+hb) + latid_pri)/2 ;//lattitude of IMU
    Logi = Logi + dt*(veb_n(1)/((RE(lati)+hb)*cos(lati)) + Logid_pri)/2; //logitude of IMU
    hb = hb + dt*(-veb_n(2)+hbd_pri)/2; //altitude of IMU
}
void NED::PositionInCartesian(){
    P_cart(0) = (RE(lati)+hb)*cos(lati)*cos(Logi);//xeb_e;
    P_cart(1) = (RE(lati)+hb)*cos(lati)*sin(Logi);//yeb_e;
    P_cart(2) = ((1-e*e)*RE(lati)+hb)*sin(Logi);//zeb_e;

}
double NED::RE(double Lb){
    double re = R0/sqrt(1-e*e*pow(sin(Lb),2));
    return re;
}
double NED::RN(double Lb){
    double rn = (1-e*e)*R0/pow((1-e*e*pow(sin(Lb),2)),1.5);
    return rn;
}
Eigen::Vector3d NED::Getvelocity(){

        return veb_n;

    }
    /*method that provide position;*/
Eigen::Vector3d NED::GetpositionInCart(){
    void PositionInCartesian();
    return P_cart;


}


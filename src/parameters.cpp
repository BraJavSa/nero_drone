#include "nero_drone/parameters.hpp"

DroneParams::DroneParams() {
    model = "Bebop2";
    ip = "192.168.0.1";

    Ts = 1.0/30.0;
    g  = 9.8;
    Altmax = 2000;

    Ku = Eigen::MatrixXd::Zero(4,4);
    Ku(0,0)=0.8417; Ku(1,1)=0.8354; Ku(2,2)=3.966; Ku(3,3)=9.8524;

    Kv = Eigen::MatrixXd::Zero(4,4);
    Kv(0,0)=0.18227; Kv(1,1)=0.17095; Kv(2,2)=4.001; Kv(3,3)=4.7295;
}

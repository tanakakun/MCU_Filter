#include "Kalman.h"

double KalmanFilter(const double ResrcData,double Q,double R)

{
    static double x_last;

    double x_mid = x_last;
    double x_now;

    static double p_last;

    double p_mid;
    double p_now;
    double kg;

    x_mid=x_last;
    p_mid=p_last+Q;
    kg=p_mid/(p_mid+R);
    x_now=x_mid+kg*(ResrcData-x_mid);

    p_now=(1-kg)*p_mid;

    p_last = p_now;

    x_last = x_now;

    return x_now;
}

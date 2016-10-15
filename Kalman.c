/********************************************************************************
The MIT License

Copyright (c) 2016 田中くんはいつもけだるげ

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

--------------------------------------------------------------------------------

File:   Kalman.c
Brief:  a Kalman Filter for MCU

Author:  Yeonji
Email:   bh2rfq@gmail.com
Version: v0.1.0
License: The MIT License

--------------------------------------------------------------------------------

Function: KalmanFilter(const double RawData,double Q,double R)
Entry:  RawData: Raw Data from ADC(or SAW)
        Q:  the covariance of the process noise
        R:  the covariance of the observation noise
Return: Filter output (double)

********************************************************************************/

#include "Kalman.h"

double KalmanFilter(const double RawData,double Q,double R)

{
    /* Declear State Variables */
    static double x_last;

    double x_now;

    /* Covariance of the estimated value */
    static double p_last;

    double p_mid;
    double p_now;

    /* Kalman Gain */
    double kg;

    /* Kalman Filter */

    /* Pre estimate of state variable x */
    /* x(k) = x(k - 1) */
    x_now=x_last;
    /* Convariance of estimate */
    /* p = p + Q */
    p_now=p_last+Q;

    /* Kalman Gain */
    /* kg = p / (p + R) */
    kg=p_now/(p_now+R);

    /* Estimate of state variable x */
    /* x(k) = x(k) + kg * (y(k) - x(k))*/
    x_now=x_now+kg*(RawData-x_now);

    /* Update convariance of estimate */
    /* p = (1 - kg) * p */
    p_now=(1-kg)*p_now;

    /* End of Kalman Filter */

    /* Cache state variable */
    x_last = x_now;
    p_last = p_now;

    return x_now;
}

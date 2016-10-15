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

File:   Kalman.h
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

#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

double KalmanFilter(const double RawData,double Q,double R);

#endif

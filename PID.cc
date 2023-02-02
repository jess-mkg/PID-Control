



/*
 * PID.cc
 * Copyright (C) 2022 Jessica McGrahan <jessica.mcgrahan@griffithuni.edu.au>
*/

#ifndef PID_H
#define PID_H

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <time.h>
#include <stdint.h>
#include <stdio.h>

#include "ros/ros.h"


class PID {

        private:

                double P = 3.0;
                double I = 2.0;
                double D = 2.0;

                double integral = 0;
                double derivative = 0;

                uint16_t prev_error = 0;
                long prev_time = 0;
                double result = 0;

        public:

                int formula(uint16_t error) {

                        struct timeval time_now{};
                        gettimeofday(&time_now, nullptr);
                        time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
                        long current_time = msecs_time;

                        long delta_time = current_time - prev_time;
                        prev_time = current_time;

                        if (delta_time == 0) {
                                //to avoid dividing by zero
                                delta_time = 1;
                        }

                        integral = ((prev_error - error) / 2) * delta_time;
                        derivative = (error - prev_error) / delta_time;
                        result = (double) ((error * P) + (integral * I) + (derivative * D));

                        prev_error = error;

                        return result;
                }
};

#endif


#ifndef KALMAN_H
#define KALMAN_H



class Kalman {

        private:

                double R = 318.89;
                double P = R;
                double Y = 0;
                int Z = 0;

                double K = 0;

                int count = 0;

        public:
                float formula(int distance) {

                        count++;
                        Z = distance;
                        if (count == 1) {
                                Y = Z;
                        }

                        K = (P/(P + R));
                        Y = (Y + K * (Z - Y));
                        P = (double) (1 - K) * P;

                        return Y;
                }
};

#endif




/*
 * controller2.cc
 * Copyright (C) 2022 Jessica McGrahan <jessica.mcgrahan@griffithuni.edu.au>
*/


#include "ros/ros.h"
#include "assignment1_setup/Sonars.h"
#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "PID.cc"
#include "kalman.cc"
#include "assignment1_setup/ModelState.h"

#include <time.h>
#include <unistd.h>



//initalise variable to be published
geometry_msgs::Twist next_move;

class Control {

        private:
                ros::Publisher pub;
                ros::Subscriber sub;
                double direction = 0.0;
                PID obj;
                Kalman f;

        public:
                uint16_t dist = 0;
                double velocity = 0.0;
                double kal = 0.0;
                double pos_x = 0.0;
                double pos_y = 0.0;

                Control(ros::NodeHandle *n) {
                        pub = n->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
                        sub = n->subscribe("sonars", 1000, &Control::CB_GetDist, this);
                }

        void CB_GetDist(const assignment1_setup::Sonars::Ptr& msg) {

                uint16_t dist0 = msg->distance0;
                uint16_t dist1 = msg->distance1;
                uint16_t dist2 = msg->distance2;

                uint16_t distance = 0;

                if (dist0 != 65535) {
                        distance = dist0;
                        direction = 0.3;
                }
                else if (dist1 != 65535) {
                        distance = dist1;
                        direction = 0.0;
                }
                else if (dist2 != 65535) {
                        distance = dist2;
                        direction = -0.3;
                }
                else {
                        distance = 65535;
                }

                // get dist from distance found to compare with later
                // find the PID with the found distance
                dist = distance;

                double v = f.formula(distance);
                kal = v;

                double vel = obj.formula(kal);
                velocity = vel/1000;
        }

        bool Next_Move() {
                //set all variables to zero
                next_move.linear.x = 0.0;
                next_move.linear.y = 0.0;
                next_move.linear.z = 0.0;
                next_move.angular.x = 0.0;
                next_move.angular.y = 0.0;
                next_move.angular.z = 0.0;

                if (dist == 65535) {
                        //stop, cant see bowl
                        pub.publish(next_move);
                        printf("Cant see the Bowl!\n");
                        return false;
                }

                else if ((dist <= 13) || (kal <=13.0)) {
                        //reached bowl
                        pub.publish(next_move);
                        printf("Found the Bowl\n");
                        return false;
                }

                else if ((velocity > 0.0) && (velocity <= 1.0)) {
                        //if the move is possible and within a good velocity
                        next_move.linear.x = velocity;
                        next_move.angular.z = direction;
                        pub.publish(next_move);
                        return true;
                }

                return true;
        }
};


int main(int argc, char** argv) {

        ros::init(argc, argv, "control");
        ros::NodeHandle n;
        ros::Rate rate(700);

        gazebo_msgs::GetModelState srv;
        std::string robotName = "turtlebot3_burger";
        srv.request.model_name = robotName;
        ros::ServiceClient location = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        //control class the handles the pid class to get the wanted output to be published
        Control c = Control(&n);

        while(ros::ok()) {

                if (!location.call(srv)) {
                        rate.sleep();
                        continue;
                }
                if (!srv.response.success) {
                        ROS_ERROR("Can't find target object %s. Is the target and robot name correct?", robotName.c_str());
                        rate.sleep();
                        continue;
                }

                c.pos_x = srv.response.pose.position.x * 100;
                c.pos_y = srv.response.pose.position.y * 100;

                if (!(c.Next_Move())) {
                        printf("Robot stopped ... \n");
                }
                rate.sleep();
                ros::spinOnce();
        }
}

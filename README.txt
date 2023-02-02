        //This is the readme file for Jessica McGrahans 3806ICT assignment 1 2022 files//

                The folder is called /MyFiles to clearly show what files I created since
                many files were given to create this project.

        //SYSTEM
                created on a MACPRO with an M1 chip using UTM as the VM.
                UBUNTU 20.04.4
                ARM64
                4 GB RAM


        //IN THE MYFILES FOLDER:

                - controller.cc
                - controller2.cc
                - PID.cc (due to compiling issues this also holds the kalman class)



        controll.cc:
                contrains my controller for task 1 which impliments the PID control method

                TO RUN:
                        1. launch the turtlebot3 gazebo with this command:
                                roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

                        2. Once the gazebo is opened make sure the turtlebot is the one named burger

                        3. Insert a bowl in front of the robot or atleast 45 degrees left or right of its forward peripheral

                        4. Open a new window and run the sonars command
                                rosrun assignment1_setup sonars

                        5. Open ANOTHER window and run the controller command
                                rosrun assignment1_setup controller

                        6. the robot should move to the bowl if it is in view. If the robot
                        reaches the bowl the bowl can still be moved for the robot to "chase".

                        7. To stop the program press control+c and it should stop.


        controller2.cc
                Contrains the same controller elements but with the Kalman Filter.
                To preserve the first task to not make errors to it task 2 was done in a different file.

                TO RUN
                        1. repreat steps 1 - 3 from the controller.cc TO RUN instructions.

                        2. Open a new window and run the noisy sonars command
                                rosrun assignment1_setup noisy_sonars

                        3. open ANOTHER window and run the controller2 command
                                rosrun assignment2_setup controller2

                        4. follow steps 6 and 7 form the controller.cc TO RUN instructions


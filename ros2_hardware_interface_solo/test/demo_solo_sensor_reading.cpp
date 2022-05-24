/* Utile pour la lecture de capteur ---> read() de system_bolt.cpp*/
/* Inspiré de demo_create_bolt_robot.cpp*/



#include <system_solo.hpp>

#include <odri_control_interface/utils.hpp>
#include <odri_control_interface/imu.hpp>


using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main()
{
    nice(-20);  // Give the process a high priority.

    // Define the robot from a yaml file.
    auto robot = RobotFromYamlFile(CONFIG_SOLO_YAML);

    // Start the robot.
    robot->Start();

    // Define controller to calibrate the joints from yaml file.
    auto calib_ctrl = JointCalibratorFromYamlFile(
        CONFIG_SOLO_YAML, robot->joints);

    // Initialize simple pd controller.
    Vector6d torques;
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    bool is_calibrated = false;
    
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now()-last)).count() > 0.001)
        {
            last = std::chrono::system_clock::now();  // last+dt would be better
            robot->ParseSensorData();

            if (robot->IsReady())
            {
                if (!is_calibrated)
                {
                    is_calibrated = calib_ctrl->Run();
                    if (is_calibrated)
                    {
                        std::cout << "Calibration is done." << std::endl;
                    }
                }
                else
                {
                    // Run the main controller.
                    auto pos = robot->joints->GetPositions();
                    auto vel = robot->joints->GetVelocities();
                    // Reverse the positions;
                    for (int i = 0; i < 6; i++)
                    {
                        torques[i] = 0.0; //-kp * pos[i] - kd * vel[i];
                    }
                    robot->joints->SetTorques(torques);
                }
            }

            // Checks if the robot is in error state (that is, if any component
            // returns an error). If there is an error, the commands to send
            // are changed to send the safety control.
            robot->SendCommand();

            c++;
            if (c % 1000 == 0)
            {
                std::cout << "Count :                        " << c << "\n";
                std::cout << "\n";

                std::cout << "Joints : \n";
                std::cout << "Position:                      ";
                robot->joints->PrintVector(robot->joints->GetPositions());
                std::cout << "\n";
                std::cout << "Velocities:                    ";
                robot->joints->PrintVector(robot->joints->GetVelocities());
                std::cout << "\n";
                std::cout << "Measured Torques:              ";
                robot->joints->PrintVector(robot->joints->GetMeasuredTorques());
                std::cout << "\n";
                std::cout << "\n";

                std::cout << "IMU : \n";
                std::cout << "Gyroscope                      ";
                robot->joints->PrintVector(robot->imu->GetGyroscope());
                std::cout << "\n";
                std::cout << "Accelerometer                  ";
                robot->joints->PrintVector(robot->imu->GetAccelerometer());
                std::cout << "\n";
                std::cout << "Linear Acceleration            ";
                robot->joints->PrintVector(robot->imu->GetLinearAcceleration());
                std::cout << "\n";
                std::cout << "Attitude Euler                 ";
                robot->joints->PrintVector(robot->imu->GetAttitudeEuler());
                std::cout << "\n";
                std::cout << "Attitude Quaternion            ";
                robot->joints->PrintVector(robot->imu->GetAttitudeQuaternion());
                std::cout << "\n";
                std::cout << "\n";
                std::cout << "\n";
                std::cout << "\n";
                std::cout << std::endl;
                
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}


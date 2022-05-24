/*Inspiré de demo_solo12_from_yaml.cpp*/


/*Modifs :
   
   Vector12d --> Vector6d
   Boucles for pour les couples : i<12 --> i<6
   CONFIG_SOLO12_YAML --> CONFIG_SOLO_YAML
 
*/



#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

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
    Eigen::Matrix<double,6,1> zero6=Eigen::Matrix<double,6,1>::Zero();
    calib_ctrl->UpdatePositionOffsets(zero6);
    // Initialize simple pd controller.
    Vector6d torques;
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    bool is_calibrated = false;
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
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
                        torques[i] = 0.0; //-kp * pos[i] - kd * vel[i]
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
                std::cout << "Joints: ";
                robot->joints->PrintVector(-robot->joints->GetPositions());
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


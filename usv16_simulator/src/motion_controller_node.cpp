#include <ros/ros.h>
#include "motion_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_controller");

  // control parameters
  path_following::control_params param_pf = {
      3.0, // ku
      0.5, // k1
      1.0, // k2
      0.2, // u_acc_max
      0.5, // k_acc_max
      1.0  // u_d_yaw
  };

  path_following::vehicle_params param_pf_v = {
      Eigen::Vector3d(207, 217, 343.2345), // M
      Eigen::Vector3d(90, 300, 400)        // D
  };

  station_keeping::control_params param_sk = {
      Eigen::Vector3d(10, 10, 100),      // kp
      Eigen::Vector3d(400, 400, 60),     // kd
      Eigen::Vector3d(0.05, 0.05, 0.05), // nl_scale
      Eigen::Vector3d(0.16, 0.16, 0.16)  // lambda
  };

  station_keeping::vehicle_params param_sk_v = {
      Eigen::Vector3d(207, 217, 343.2345), // M
      Eigen::Vector3d(56.2, 300, 400)      // D
  };

  MotionController controller(2.0, param_pf_v, param_pf, param_sk_v, param_sk,
                              0.1);

  return controller.run();
}

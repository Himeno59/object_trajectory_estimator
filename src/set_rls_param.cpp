#include "ros/ros.h"
#include "object_trajectory_estimator/SetRLSParameters.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_parameters_client");
  if (argc != 4)
    {
      ROS_INFO("usage: set_parameters_client X Y Z");
      return 1;
    }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<object_trajectory_estimator::SetRLSParameters>("/ObjectTrajectoryEstimator/set_rls_parameters");
  
  object_trajectory_estimator::SetRLSParameters srv;
  srv.request.params[0] = atof(argv[1]);
  srv.request.params[1] = atof(argv[2]);
  srv.request.params[2] = atof(argv[3]);
  
  if (client.call(srv))
    {
      if (srv.response.success)
        {
	  ROS_INFO("Parameters set successfully");
        }
      else
        {
	  ROS_WARN("Failed to set parameters");
        }
    }
  else
    {
      ROS_ERROR("Failed to call service set_parameters");
      return 1;
    }
  
  return 0;
}

#include "ros/ros.h"
#include "bouncing_ball_estimator/GetRLSParameters.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_parameters_client");
    if (argc != 1)
    {
        ROS_INFO("Usage: get_parameters_client");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<bouncing_ball_estimator::GetRLSParameters>("/BouncingBallEstimator/get_rls_parameters");

    bouncing_ball_estimator::GetRLSParameters srv;

    if (client.call(srv))
    {
        ROS_INFO("Parameters received:");
        ROS_INFO("Parameter 1: %f", srv.response.params[0]);
        ROS_INFO("Parameter 2: %f", srv.response.params[1]);
    }
    else
    {
        ROS_ERROR("Failed to call service get_rls_parameters");
        return 1;
    }

    return 0;
}

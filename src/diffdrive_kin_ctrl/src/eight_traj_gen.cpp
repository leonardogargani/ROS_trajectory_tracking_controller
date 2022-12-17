#include "eight_traj_gen.h"

#include <unistd.h>


bool GenerateDesiredPath(diffdrive_kin_ctrl::GenerateDesiredPathService::Request &req,
                                            diffdrive_kin_ctrl::GenerateDesiredPathService::Response &res)
{

    // fetch parameters from parameters server
    std::string FullParamName;

    ros::NodeHandle Handle;

    double a;
    double w;

    FullParamName = ros::this_node::getName() + "/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    float t = 0;

    // generate eight-shaped trajectory
    while(t < 10) {
        res.xref.push_back(a * std::sin(w * t));
        res.yref.push_back(a * std::sin(w * t) * std::cos(w * t));
    	t += 0.001;
    }

	ROS_INFO("Size in traj_gen: %lu", res.xref.size());

    ROS_INFO("Service server: sending back response.");

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

    ros::NodeHandle Handle;
    ros::ServiceServer service = Handle.advertiseService("generate_desired_path_service", GenerateDesiredPath);
    ros::spin();

    return (0);
}

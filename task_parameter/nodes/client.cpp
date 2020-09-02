#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ReconfigureRequest.h>

#include <dynamic_parameter/PickPlaceArmConfig.h>
#include <boost/function.hpp>
#include <string>
#include <iostream>
#include <fstream>

#include "dynamic_parameter/client.h"

ParamClient::ParamClient()
{
    param_arm = new double[ 6 ] { 0, 0, 0, 0, 0, 0 };

}

ParamClient::~ParamClient() 
{
}

int main(int argc, char **argv)
{
    ParamClient param_client;
    ros::init(argc, argv, "dyn_client");
    
    ros::NodeHandle nh;

    double global_name;

    ROS_INFO("Spinning node");

    double param_arm_temp[6] = {0.00};
    // CallBack tmpdata;
    dynamic_parameter::PickPlaceArmConfig config;
    // dynamic_reconfigure::Client<dynamic_parameter::PickPlaceArmConfig> client("arm_param", dynCallBack);
    //tmpdata = boost::bind(dynCallBack, _1);

    // ros::Rate loop_rate(10);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int c = 0;
    while (ros::ok())
    {
    //     param_client.get_param_arm(nh, param_arm_temp, 6);
    //         for (int i = 0; i<6; i++)
    // {
    //     ROS_INFO("test: %f", param_arm_temp[i]);
    // }

        // for (int i = 0; i<6; i++)
        // {
        //     param_client.param_topic = "/" + param_client._server_arm + "/" + param_client.param_arm_name[i];
        //     ROS_INFO("test: %s",  param_client.param_topic.c_str());
        //     if (nh.getParam(param_client.param_topic, param_client.param_arm[i]))
        //     {
        //         ROS_INFO("test: %f",  param_client.param_arm[i]);
        //     }
        // }
    /*
        c++;
        static bool ret = true;
        static int cnt = 0;
        
        ROS_INFO("callback_T2S1: %f", config.T2S1arm);
        
        // if(!(c%10))
        // {

        //     config.T2S1arm += config.T2S1arm/10;
        //     client.setConfiguration(config);
        //     if(cnt > 1)
        //         cnt = 0;
        // }

        // ros::spinOnce();
        // loop_rate.sleep();

    */

        // client.getCurrentConfiguration(CONFIG);
    }
    ROS_INFO("Spinning node shutdown...");
    return 0;
}
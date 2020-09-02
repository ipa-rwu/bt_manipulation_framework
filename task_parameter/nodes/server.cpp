#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>         // std::string

#include <dynamic_reconfigure/server.h>
#include <dynamic_parameter/PickPlaceArmConfig.h>
#include <dynamic_parameter/PickPlaceGripConfig.h>

using namespace std;

void armcallback(dynamic_parameter::PickPlaceArmConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f", 
            config.T2S1arm, 
            config.T2S2arm, 
            config.T2S3arm, 
            config.T3S1arm, 
            config.T3S2arm,
            config.T3S3arm);
}

void gripcallback(dynamic_parameter::PickPlaceGripConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s %s %s %s %s %s", 
            config.T2S1gripper.c_str(), 
            config.T2S2gripper.c_str(), 
            config.T2S3gripper.c_str(), 
            config.T3S1gripper.c_str(), 
            config.T3S2gripper.c_str(),
            config.T3S3gripper.c_str());
}
     
std::string GetStdoutFromCommand(std::string cmd) {

  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream) {
  while (!feof(stream))
  if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
  pclose(stream);
  }
  return data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "paramcenter");
    std::string str2 ("gui");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    // dynamic_reconfigure::Server<dynamic_parameter::PickPlaceArmConfig> armserver;
    // dynamic_reconfigure::Server<dynamic_parameter::PickPlaceArmConfig>::CallbackType armcallback_f;
    
    // armcallback_f = boost::bind(&armcallback, _1, _2);
    // armserver.setCallback(armcallback_f);

    dynamic_reconfigure::Server<dynamic_parameter::PickPlaceGripConfig> gripserver;
    dynamic_reconfigure::Server<dynamic_parameter::PickPlaceGripConfig>::CallbackType gripcallback_f;
    
    gripcallback_f = boost::bind(&gripcallback, _1, _2);
    gripserver.setCallback(gripcallback_f);


    std::string rosnode; 

    std::size_t found;
    
    system("/home/rachel/kogrob/kogrob_ws/src/dynamic_tutorials/src/nodes/test.sh");
    cout << "rosnode: " << rosnode << endl;

    ROS_INFO("Spinning node");
    while (ros::ok()) {
        

        rosnode = GetStdoutFromCommand("rosnode list node");
        
        cout << "rosnode: " << rosnode << endl;

        found = rosnode.find(str2);
        if (found!=std::string::npos)
            std::cout << "first 'gui' found at: " << found << '\n';
        else
        {
           rosnode = GetStdoutFromCommand("rosnode list node");
        }
        
        // Sleep 100 milliseconds
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
#include "manipulator_skills/skills/check_collision.hpp"
#include "manipulator_skills/skill_names.hpp"

namespace manipulator_skills
{

CheckCollision::CheckCollision(
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(CHECK_COLLISION),
    pnh_(private_node_handle),
    service_name_(CHECK_COLLISION)
  {
    this->initialize();
  }

CheckCollision::~CheckCollision()
{

}

void CheckCollision::initialize()
{
    fixName();
    touch_sensor_topic_name_ = "/container_A" + mRobotName_ + "/touch_sensor/value";
    touch_sensor_sub_ = pnh_.subscribe(touch_sensor_topic_name_,
                          1,
                          &CheckCollision::TouchsensorCallback,
                          this);
    service_ = pnh_.advertiseService(CHECK_COLLISION, &CheckCollision::executeCB, this);
    ROS_INFO_STREAM_NAMED(getName(), "start service" );
}

void CheckCollision::fixName() 
{
  std::string webotsPID;
  std::string webotsHostname;
  std::ostringstream s;
  int pid;


    // retrieve Webots' PID
    #ifdef _WIN32
    HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    PROCESSENTRY32 pe = {0};
    pe.dwSize = sizeof(PROCESSENTRY32);
    if (Process32First(h, &pe)) {
        while (Process32Next(h, &pe) && s.str() == "") {
        if (!strcmp(pe.szExeFile, "ros.exe"))
            s << pe.th32ParentProcessID;
        }
    }
    CloseHandle(h);
    #else
    s << getppid();
    #endif
    //   webotsPID = s.str();
    pid = getProcIdByName("webots-bin");
    webotsPID = std::to_string(pid);
    // retrieve local hostname
    char hostname[256];
    gethostname(hostname, 256);
    webotsHostname = hostname;

    mRobotName_ = '_' + webotsPID + '_' + webotsHostname;
    // remove unhautorized symbols ('-', ' ' and '.') for ROS
    mRobotName_ = fixedNameString(mRobotName_);

    //   std::cout << mRobotName_ << std::endl;
}

int CheckCollision::getProcIdByName(std::string procName)
{
    int pid = -1;

    // Open the /proc directory
    DIR *dp = opendir("/proc");
    if (dp != NULL)
    {
        // Enumerate all entries in directory until process found
        struct dirent *dirp;
        while (pid < 0 && (dirp = readdir(dp)))
        {
            // Skip non-numeric entries
            int id = atoi(dirp->d_name);
            if (id > 0)
            {
                // Read contents of virtual /proc/{pid}/cmdline file
                cmdPath_ = std::string("/proc/") + dirp->d_name + "/cmdline";
                std::ifstream cmdFile(cmdPath_.c_str());
                std::string cmdLine;
                getline(cmdFile, cmdLine);
                if (!cmdLine.empty())
                {
                    // Keep first cmdline item which contains the program path
                    size_t pos = cmdLine.find('\0');
                    if (pos != std::string::npos)
                        cmdLine = cmdLine.substr(0, pos);
                    // Keep program name only, removing the path
                    pos = cmdLine.rfind('/');
                    if (pos != std::string::npos)
                        cmdLine = cmdLine.substr(pos + 1);
                    // Compare against requested process name
                    if (procName == cmdLine)
                        pid = id;
                }
            }
        }
    }

    closedir(dp);

    return pid;
}

std::string CheckCollision::fixedNameString(const std::string &name) 
{
  std::string fixedName = name;
  std::replace(fixedName.begin(), fixedName.end(), '-', '_');
  std::replace(fixedName.begin(), fixedName.end(), '.', '_');
  std::replace(fixedName.begin(), fixedName.end(), ' ', '_');
  std::replace(fixedName.begin(), fixedName.end(), ')', '_');
  std::replace(fixedName.begin(), fixedName.end(), '(', '_');
  return fixedName;
}

bool CheckCollision::executeCB(man_msgs::CheckCollisionService::Request  &req,
                        man_msgs::CheckCollisionService::Response &res)
{
    ROS_INFO_NAMED(getName(), "[%s] get request", getName().c_str());

    bool start = req.check;
    if(start)
    {
        res.result = result_touchsensor_;
    }

    return true;
}

void CheckCollision::TouchsensorCallback(const webots_ros::BoolStamped::ConstPtr& touchsensor_msg)
{
  // msg: {"data": "start"}
  result_touchsensor_ = touchsensor_msg->data;
//   std::cout << result_touchsensor <<std::endl;

}

bool CheckCollision::Touched()
{
    if (result_touchsensor_ == true)
    {
        return false;
        ROS_INFO_STREAM("Arm Touched");
        printf("CheckCollision::Touched");
        //  _armarker_srv.request.ar_marker_id); 
    }
    else
    {
        return true;
        printf("CheckCollision::NOTTouched");   
    }
 
}


} //namespace

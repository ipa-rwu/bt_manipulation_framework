#include "man_behavior_tree_nodes/webots_elements.hpp"

namespace man_behavior_tree_nodes
{

// WebotsSkills::WebotsSkills():
// {
//     this->initialize();
// }

// WebotsSkills::~WebotsSkills()
// {

// }

// void WebotsSkills::initialize()
// {
//     fixName();
// }

std::string WebotsSkills::fixName() 
{
  std::string webotsPID;
  std::string webotsHostname;
  std::ostringstream s;
  int pid;
  std::string mRobotName;    

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

    mRobotName = '_' + webotsPID + '_' + webotsHostname;
    // remove unhautorized symbols ('-', ' ' and '.') for ROS
    mRobotName  = fixedNameString(mRobotName);

    return mRobotName;

    //   std::cout << mRobotName_ << std::endl;
}

int WebotsSkills::getProcIdByName(std::string procName)
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

std::string WebotsSkills::fixedNameString(const std::string &name) 
{
  std::string fixedName = name;
  std::replace(fixedName.begin(), fixedName.end(), '-', '_');
  std::replace(fixedName.begin(), fixedName.end(), '.', '_');
  std::replace(fixedName.begin(), fixedName.end(), ' ', '_');
  std::replace(fixedName.begin(), fixedName.end(), ')', '_');
  std::replace(fixedName.begin(), fixedName.end(), '(', '_');
  return fixedName;
}


}   //namespace
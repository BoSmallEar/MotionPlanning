#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace std;
using namespace OpenRAVE;

int main(){ 
    string scenefilename = "data/lab1.env.xml";
    string viewername = RaveGetDefaultViewerType(); // qtcoin
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv =OpenRAVE::RaveCreateEnvironment(); // create the main environment
    RaveSetDebugLevel(Level_Debug);
 
    penv->Load(scenefilename); // load the scene 
    penv->Destroy(); // destroy
    return 0;
}
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/geometry.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>
#include <vector> 
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <unordered_map>
#include <queue> 
using namespace std;
using namespace OpenRAVE;
using namespace OpenRAVE::planningutils;
using namespace OpenRAVE::geometry;
#define MAX 41;


double euclideanDistance(RaveVector<double> start, RaveVector<double> end){ 
    RaveVector<double> v = start-end;
    return v.lengthsqr3();
};

double quatDistance(RaveVector<double> start, RaveVector<double> end){
    RaveTransformMatrix<double> a = matrixFromQuat(start);
    RaveTransformMatrix<double> b = matrixFromQuat(end); 
    b = b.inverse();
    a = a*b;
    double tr = a.m[0]+a.m[5]+a.m[10];
    return MATH_ACOS((tr-1)/2);
};

bool double_equals(double a, double b, double epsilon = 0.01){
    return abs(a-b)<epsilon;
};

class node{
    public:
        double g;
        double h;
        double f;
        RaveVector<double> previousPos;
        RaveVector<double> previousQuat;
        RaveVector<double> pos;
        RaveVector<double> quat;
        node(double g,double h,RaveVector<double> previousPos,RaveVector<double> previousQuat, 
        RaveVector<double> pos,RaveVector<double> quat):g(g),h(h),f(f),previousPos(previousPos),previousQuat(previousQuat),
        pos(pos), quat(quat){
            this->f = g+h;
        };

        node(){
        };

        node& operator=(const node& other){
            if (this!=&other){
                g = other.g;
                h = other.h;
                f = other.f;
                previousPos = other.previousPos;
                previousQuat = other.previousQuat;
                pos = other.pos;
                quat = other.quat;
            }
            return *this;
        }
};
class dictKey{
    public:
        RaveVector<double> pos;
        RaveVector<double> quat;
        bool operator== (const dictKey& key) const{
            return double_equals(pos.x,key.pos.x)&&double_equals(pos.y,key.pos.y)&&double_equals(pos.z,key.pos.z)
            &&double_equals(quat.x,key.quat.x)&&double_equals(quat.y,key.quat.y)&&double_equals(quat.z,key.quat.z)&&double_equals(quat.w,key.quat.w);
        };
        dictKey( RaveVector<double> pos, RaveVector<double> quat):pos(pos),quat(quat){};
        dictKey(){};
};

vector<double> dictKeytoVec(const dictKey& key){
    vector<double> vec{key.pos.x,key.pos.y,key.pos.z,key.quat.x,key.quat.y,key.quat.z,key.quat.w};
    return vec;
};

struct hash_fn
{ 
	std::size_t operator() (const dictKey& key) const
	{
		std::size_t h1 = hash<int>()(key.pos.x*10);
		std::size_t h2 = hash<int>()(key.pos.y*10);
		std::size_t h3 = hash<int>()(key.pos.z*10);
		std::size_t h4 = hash<int>()(key.quat.x*100);
		std::size_t h5 = hash<int>()(key.quat.y*100);
		std::size_t h6 = hash<int>()(key.quat.z*100);
		std::size_t h7 = hash<int>()(key.quat.w*100); 

		return h1 ^ h2 ^ h3 ^h4 ^h5 ^h6 ^h7;
	}
};

class compNodes{
    public:
        bool operator()(const node& a, const node& b) const{
            return a.f>b.f;
        };
};

class TestModule : public ModuleBase
{
public:
    TestModule(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        __description = "A very simple plugin.";
        RegisterCommand("numbodies",boost::bind(&TestModule::NumBodies,this,_1,_2),"returns bodies");
        RegisterCommand("load",boost::bind(&TestModule::Load,this,_1,_2),"loads a given file");
        RegisterCommand("generate",boost::bind(&TestModule::Generate,this,_1,_2),"generate random objects");
        RegisterCommand("find",boost::bind(&TestModule::Find,this,_1,_2),"find a path from start to goal");
    }
    virtual ~TestModule() {
    }
 
    bool NumBodies(ostream& sout, istream& sinput)
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        sout << vbodies.size();     // publish the results
        return  true;  
    }
    bool Load(ostream& sout, istream& sinput)
    {
        string filename;
        sinput >> filename;
        bool bSuccess = GetEnv()->Load(filename.c_str());  
	    sout << "good";   // load the file
        return bSuccess;
    }

    double randomPos(){
        int seed = rand()%MAX;
        return ((double)seed)/10-2;
    }

    bool Generate(ostream& sout, istream& sinput){  
        srand((unsigned) time (NULL)); 
        int num; 
        char* tag = "randomobj%d";
        char name[20];
        sinput>>num;
        for (int i=0;i<num;i++){
            KinBodyPtr body = RaveCreateKinBody(GetEnv(),"");
            std::vector<AABB> boxes(1);
            boxes[0].pos = Vector(randomPos(),randomPos(),randomPos());
            boxes[0].extents = Vector(0.1,0.1,0.1);
            body->InitFromBoxes(boxes,true);
            sprintf(name,tag,i); 
            body->SetName(string(name));
            GetEnv()->Add(body);
        }
        sout<<num;
        return true;
    }

    bool Find(ostream& sout, istream& sinput){
        vector<double> goal;
        double d;
        while(sinput>>d){
            goal.push_back(d);
        }
        RobotBasePtr robot = GetEnv()->GetRobot("torso");   
        vector<vector<double>> path = findPath(GetEnv(),robot,goal);
        if (path.size()==0){
            sout<<"No solution found";
            return false;
        }
        else{
            sout<<"Find path successfully!";
            TrajectoryBasePtr traj = ConvertPathToTrajectory(robot, path);
            robot->GetController()->SetPath(traj); 
            // unlock the environment and wait for the robot to finish
            while(!robot->GetController()->IsDone()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
            return true;
        }
    }

    vector<vector<double>> findPath(EnvironmentBasePtr env,RobotBasePtr robot, vector<double>& goal){
        vector<double> start;
        robot->GetActiveDOFValues(start); 
        RaveVector<double> startPos(start[0],start[1],start[2]);
        RaveVector<double> startQuat(start[3],start[4],start[5],start[6]);
        RaveVector<double> goalPos(goal[0],goal[1],goal[2]);
        RaveVector<double> goalQuat(goal[3],goal[4],goal[5],goal[6]); 
        vector<RaveVector<double>> quats;
        quats.push_back(RaveVector<double>(0.966,0.259,0,0));
        quats.push_back(RaveVector<double>(0.966,-0.259,0,0));
        quats.push_back(RaveVector<double>(0.966,0,0.259,0));
        quats.push_back(RaveVector<double>(0.966,0,-0.259,0));
        quats.push_back(RaveVector<double>(0.966,0,0,0.259));
        quats.push_back(RaveVector<double>(0.966,0,0,-0.259));
        dictKey target (goalPos,goalQuat);
        dictKey init(startPos,startQuat);
        double grid_size_x = 0.1;
        double grid_size_y = 0.1;
        double grid_size_z = 0.1; 
        vector<double> increments{grid_size_x,grid_size_y,grid_size_z};
        vector<vector<double>> coefs;
        vector<double> distances;
        vector<double> coef;
        vector<double> sel{-1,0,1};
        for (double i:sel){
            coef.push_back(i*grid_size_x);
            for (double j:sel){
                coef.push_back(j*grid_size_y);
                for (double k:sel){
                    coef.push_back(k*grid_size_z);
                    if (!(double_equals(i,0)&&double_equals(j,0)&&double_equals(k,0))){
                        coefs.push_back(coef);
                        distances.push_back(sqrt(coef[0]*coef[0]+coef[1]*coef[1]+coef[2]*coef[2]));
                    }
                    coef.pop_back();
                }
                coef.pop_back();
            }
            coef.pop_back();
        }
        priority_queue<node,vector<node>,compNodes> pq; 
        pq.push(node(0,euclideanDistance(startPos,goalPos),startPos,startQuat,goalPos,goalQuat));

        bool found = false; 

        unordered_map<dictKey,node,hash_fn> nodeDict;

        while (!pq.empty()){
            node current = pq.top();
            pq.pop();
            if (nodeDict.find(dictKey(current.pos,current.quat)) == nodeDict.end()){
                nodeDict[dictKey(current.pos,current.quat)] = current;
                //handles.append(env.plot3(points=np.array((current.pos[0],current.pos[1],0.2)),pointsize=5.0,colors =np.array((0,0,1))))
                if (dictKey(current.pos,current.quat) == target){
                    found = true;
                    break;
                }
                for (unsigned int i=0;i<coefs.size();i++){
                    dictKey candidate(current.pos,current.quat);
                    vector<double> coef = coefs[i];
                    candidate.pos.x = round((current.pos.x+coef[0])*10)/10;
                    candidate.pos.y = round((current.pos.y+coef[1])*10)/10;
                    candidate.pos.z = round((current.pos.z+coef[2])*10)/10;
                    vector<double> dofValues{candidate.pos.x,candidate.pos.y,candidate.pos.z,candidate.quat.x,candidate.quat.y,candidate.quat.z,candidate.quat.w};
                    robot->SetActiveDOFValues(dofValues);
                    if (!env->CheckCollision(robot) && nodeDict.find(candidate)!=nodeDict.end()){
                        pq.push(node(current.g +distances[i],euclideanDistance(candidate.pos,goalPos)+quatDistance(candidate.quat,goalQuat),candidate.pos,candidate.quat,current.pos,current.quat));
                    }
                   // elif env.CheckCollision(robot):
                   //     handles.append(env.plot3(points=np.array((candidate[0],candidate[1],0.2)),pointsize=5.0,colors =np.array((1,0,0))))
                }
                for (unsigned int i=0; i<quats.size();i++){
                    dictKey candidate(current.pos,current.quat);
                    RaveVector<double> coef = quats[i];
                    candidate.quat = quatMultiply(coef,current.quat);
                    candidate.quat.x = round(candidate.quat.x*1000)/1000;
                    candidate.quat.y = round(candidate.quat.y*1000)/1000;
                    candidate.quat.z = round(candidate.quat.z*1000)/1000;
                    candidate.quat.w = round(candidate.quat.w*1000)/1000;
                    vector<double> dofValues{candidate.pos.x,candidate.pos.y,candidate.pos.z,candidate.quat.x,candidate.quat.y,candidate.quat.z,candidate.quat.w};
                    robot->SetActiveDOFValues(dofValues);
                    if (!env->CheckCollision(robot) && nodeDict.find(candidate)!=nodeDict.end()){
                        pq.push(node(current.g +quatDistance(coef,startQuat),euclideanDistance(candidate.pos,goalPos)+quatDistance(candidate.quat,goalQuat),candidate.pos,candidate.quat,current.pos,current.quat));
                    }
                 }
            }
        }
        vector<vector<double>> result;
        if  (found){
            result.push_back(dictKeytoVec(target)); 
            while (!(target==init)){
                target = dictKey(nodeDict[target].previousPos,nodeDict[target].previousQuat);
                result.push_back(dictKeytoVec(target));
            }
            result.push_back(dictKeytoVec(init));
        } 
        return result;
    }

    TrajectoryBasePtr ConvertPathToTrajectory(RobotBasePtr robot,vector<vector<double>> path){   
        TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(),"");	
        traj->Init(robot->GetActiveConfigurationSpecification()); 
        for (size_t i=0;i<path.size();i++){
            traj->Insert(i,path[path.size()-i-1]);
        } 
        vector<double> maxVelocities{1,1,1,1,1,1,1};
        vector<double> maxAccelerations{5,5,5,5,5,5,1};
        RetimeAffineTrajectory(traj,maxVelocities,maxAccelerations);
        return traj;
    }
}; 
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "testmodule" ) {
        return InterfaceBasePtr(new TestModule(penv));
    }
    return InterfaceBasePtr();
}
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("TestModule");
}
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

#include "planningtorso_A*.hpp"


void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->Add(viewer);

    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

int main (){
    vector<GraphHandlePtr> handles;
    RaveInitialize(true);
    EnvironmentBasePtr env = RaveCreateEnvironment();
    torsoplanning_3D plugin(env);
    RaveSetDebugLevel(Level_Error);
    string viewername = RaveGetDefaultViewerType();
    boost::thread thviewer(boost::bind(SetViewer,env,viewername));
    string filename = "scenes/border.env.xml";
    env->Load(filename);
    TriMesh mesh;
//    vector<int> indices = {0,1,2,0,2,3};
//    vector<Vector> vertices;
//    vertices.emplace_back(-1,1,0);
//    vertices.emplace_back(-1,0,0);
//    vertices.emplace_back(1,0,0);
//    vertices.emplace_back(1,1,0);
//    KinBodyPtr board = RaveCreateKinBody(env);
//    assert(board!= nullptr);
//    mesh.indices = indices;
//    mesh.vertices = vertices;
//    board->InitFromTrimesh(mesh,true);
//    board->SetName("board");
//    env->Add(board);
    vector<int> indices2 = {0,1,2,0,2,3};
    vector<Vector> vertices2;
    vertices2.emplace_back(-1.2,0.6,0);
    vertices2.emplace_back(-1.2,-0.6,0);
    vertices2.emplace_back(0,-0.6,0);
    vertices2.emplace_back(0,0.6,0);
    KinBodyPtr board2 = RaveCreateKinBody(env);
    mesh.indices = indices2;
    mesh.vertices = vertices2;
    board2->InitFromTrimesh(mesh,true);
    board2->SetName("board2");
    env->Add(board2);
    vector<int> indices3 = {0,1,2,0,2,3};
    vector<Vector> vertices3;
    vertices3.emplace_back(0,0.6,0);
    vertices3.emplace_back(0,-0.6,0);
    vertices3.emplace_back(0,-0.6,1.2);
    vertices3.emplace_back(0,0.6,1.2);
    KinBodyPtr board3 = RaveCreateKinBody(env);
    mesh.indices = indices3;
    mesh.vertices = vertices3;
    board3->InitFromTrimesh(mesh,true);
    board3->SetName("board3");
    env->Add(board3);
    vector<int> indices4 = {0,1,2,0,2,3};
    vector<Vector> vertices4;
    vertices4.emplace_back(0,0.6,1.2);
    vertices4.emplace_back(0,-0.6,1.2);
    vertices4.emplace_back(1.2,-0.6,1.2);
    vertices4.emplace_back(1.2,0.6,1.2);
    KinBodyPtr board4 = RaveCreateKinBody(env);
    mesh.indices = indices4;
    mesh.vertices = vertices4;
    board4->InitFromTrimesh(mesh,true);
    board4->SetName("board4");
    env->Add(board4);
    CollisionCheckerBasePtr checker = RaveCreateCollisionChecker(env,"ode");
    env->SetCollisionChecker(checker);
    RobotBasePtr robot = env->GetRobot("torso");
    robot->SetVisible(false);
    robot->SetActiveDOFs(vector<int>{},DOF_Transform);
    vector<dReal> start = {-0.6,0,1,1,0,0,0};
    robot->SetActiveDOFValues(start);
    vector<dReal> goal = {1.8,0,2.2,1,0,0,0};
   /* istringstream is("1");
    ostringstream os;
    plugin.Generate(os,is);*/

    //env->GetMutex().lock();
    vector<dictKey> result = plugin.findPathAstar(env,robot,goal);
    //env->GetMutex().unlock();

    env->Destroy(); // destroy
    return 0;
}
/** \example orplanning_ik.cpp
    \author Rosen Diankov

    Shows how to use inverse kinematics and planners to move a robot's end-effector safely through the environment.
    The default manipulator is used for the robot.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningIkExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "data/kuka.env.xml";
        penv->Load(scenefilename);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        RobotBasePtr probot = vrobots.at(0);

        // find a manipulator chain to move
        for(size_t i = 0; i < probot->GetManipulators().size(); ++i) {
            if( probot->GetManipulators()[i]->GetName().find("arm") != string::npos ) {
                probot->SetActiveManipulator(probot->GetManipulators()[i]);
                break;
            }
        }
        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
		std::stringstream ssin, ssout;
        // load inverse kinematics using ikfast
        /*ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
        penv->Add(pikfast,true,"");
        stringstream ssin,ssout;
        vector<dReal> vsolution;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
        if( !pikfast->SendCommand(ssout,ssin) ) {
            RAVELOG_ERROR("failed to load iksolver\n");
        }*/
        if( !pmanip->GetIkSolver()) {
            throw OPENRAVE_EXCEPTION_FORMAT0("need ik solver",ORE_Assert);
        }

        ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
        penv->Add(pbasemanip,true,probot->GetName()); // load the module
		TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv, "");
        while(IsOk()) 
		{
			GraphHandlePtr pgraph;
			{
				EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

				Transform t = pmanip->GetEndEffectorTransform();
				t.trans += Vector(RaveRandomFloat() - 0.5f, RaveRandomFloat() - 0.5f, RaveRandomFloat() - 0.5f);
				Vector direction(RaveRandomFloat() - 0.5f, RaveRandomFloat() - 0.5f, 0);
				direction.normalize();

				t.rot = quatMultiply(t.rot, quatFromAxisAngle(Vector(RaveRandomFloat() - 0.5f, RaveRandomFloat() - 0.5f, 0)*0.2f));

				IkParameterization ik_param;
				ik_param.SetLookat3D(RAY(t.trans, direction));

				stringstream ssin, ssout;
				ssin << "MoveToHandPosition outputtraj pose " << t;
				//ssin << "MoveHandStraight starteematrix " << t;
				// start the planner and run the robot
				RAVELOG_INFO("%s\n", ssin.str().c_str());
				if (!pbasemanip->SendCommand(ssout, ssin))
				{
					continue;
				}
				ptraj->deserialize(ssout);
				RAVELOG_INFO("trajectory duration %fs\n", ptraj->GetDuration());

				// draw the end effector of the trajectory
				{
					RobotBase::RobotStateSaver saver(probot); // save the state of the robot since will be setting joint values
					std::vector<RaveVector<float> > vpoints;
					std::vector<dReal> vtrajdata;
					for (dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01)
					{
						ptraj->Sample(vtrajdata, ftime, probot->GetActiveConfigurationSpecification());
						probot->SetActiveDOFValues(vtrajdata);
						vpoints.push_back(pmanip->GetEndEffectorTransform().trans);
					}
					pgraph = penv->drawlinestrip(&vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 1.0f);
				}
			}

			// unlock the environment and wait for the robot to finish
			while (!probot->GetController()->IsDone() && IsOk())
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			}
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PlanningIkExample example;
    return example.main(argc,argv);
}

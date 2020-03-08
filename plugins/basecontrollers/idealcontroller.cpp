// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "plugindefs.h"

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

class IdealController : public ControllerBase
{
public:
	IdealController(EnvironmentBasePtr penv, std::istream& sinput)
		: ControllerBase(penv),
		cmd_id_(0),
		is_pause_(false),
		is_done_(true),
		is_check_collision_(false),
		is_throw_exceptions_(false),
		is_enable_logging_(false)
	{
		description_ = ":Interface Author: Rosen Diankov\n\nIdeal controller used for planning and non-physics simulations.\
                          Forces exact robot positions.\n\n\
                          If \ref ControllerBase::SetPath is called and the trajectory finishes, \
                          then the controller will continue to set the trajectory's final joint values and \
                          transformation until one of three things happens:\n\n\
                          1. ControllerBase::SetPath is called.\n\n\
                          2. ControllerBase::SetDesired is called.\n\n\
                          3. ControllerBase::Reset is called resetting everything\n\n\
                          If SetDesired is called, only joint values will be set at every timestep leaving the transformation alone.\n";
		RegisterCommand("Pause", boost::bind(&IdealController::_Pause, this, _1, _2),
			"pauses the controller from reacting to commands ");
		RegisterCommand("SetCheckCollisions", boost::bind(&IdealController::_SetCheckCollisions, this, _1, _2),
			"If set, will check if the robot gets into a collision during movement");
		RegisterCommand("SetThrowExceptions", boost::bind(&IdealController::_SetThrowExceptions, this, _1, _2),
			"If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");
		RegisterCommand("SetEnableLogging", boost::bind(&IdealController::_SetEnableLogging, this, _1, _2),
			"If set, will write trajectories to disk");
		command_time_ = 0;
		speed_ = 1;
		control_transformation_ = 0;
	}
	virtual ~IdealController()
	{
	}

	virtual bool Init(RobotBasePtr robot, const std::vector<int>& dof_indices, int control_transformation)
	{
		robot_ = robot;
		if (flog_.is_open()) 
		{
			flog_.close();
		}
		if (!!robot)
		{
			if (is_enable_logging_) 
			{
				std::string filename = RaveGetHomeDirectory() + std::string("/") + robot->GetName() + std::string(".traj.xml");
				flog_.open(filename.c_str());
				if (!flog_) 
				{
					RAVELOG_WARN(str(boost::format("failed to open %s\n") % filename));
				}
				//flog_ << "<" << GetXMLId() << " robot=\"" << robot->GetName() << "\"/>" << endl;
			}
			dof_indices_ = dof_indices;
			control_transformation_ = control_transformation;
			is_dof_circular_vector_.resize(0);
			for(auto it: dof_indices_) 
			{
				KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(it);
				is_dof_circular_vector_.push_back(pjoint->IsCircular(it - pjoint->GetDOFIndex()));
			}
			limits_callback_ = robot->RegisterChangeCallback(KinBody::Prop_JointLimits 
				| KinBody::Prop_JointAccelerationVelocityTorqueLimits,
				boost::bind(&IdealController::_SetJointLimits, boost::bind(&utils::sptr_from<IdealController>, weak_controller())));
			_SetJointLimits();

			if (dof_indices_.size() > 0) 
			{
				group_joint_values_.reset(new ConfigurationSpecification::Group());
				group_joint_values_->offset = 0;
				group_joint_values_->dof = dof_indices_.size();
				std::stringstream ss;
				ss << "joint_values " << robot->GetName();
				for(auto it: dof_indices_) 
				{
					ss << " " << it;
				}
				group_joint_values_->name = ss.str();
			}
			if (control_transformation) 
			{
				group_transform_.reset(new ConfigurationSpecification::Group());
				group_transform_->offset = robot->GetDOF();
				group_transform_->dof = RaveGetAffineDOF(DOF_Transform);
				group_transform_->name = str(boost::format("affine_transform %s %d") % robot->GetName() % DOF_Transform);
			}
		}
		is_pause_ = false;
		return true;
	}

	virtual void Reset(int options)
	{
		trajectory_.reset();
		desired_joints_value_vector_.resize(0);
		if (flog_.is_open()) 
		{
			flog_.close();
		}
		is_done_ = true;
	}

	virtual const std::vector<int>& GetControlDOFIndices() const
	{
		return dof_indices_;
	}
	virtual int IsControlTransformation() const 
	{
		return control_transformation_;
	}

	virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
	{
		if (values.size() != dof_indices_.size()) 
		{
			throw OpenRAVEException(str(boost::format("wrong desired dimensions %d!=%d")
				% values.size() % dof_indices_.size()), ORE_InvalidArguments);
		}
		command_time_ = 0;
		trajectory_.reset();
		// do not set done to true here! let it be picked up by the simulation thread.
		// this will also let it have consistent mechanics as SetPath
		// (there's a race condition we're avoiding where a user calls SetDesired and then state savers revert the robot)
		if (!is_pause_)
		{
			RobotBasePtr probot = robot_.lock();
			EnvironmentMutex::scoped_lock lockenv(probot->GetEnv()->GetMutex());
			desired_joints_value_vector_ = values;
			if (control_transformation_) 
			{
				if (!!trans) 
				{
					desired_transform_ = *trans;
				}
				else 
				{
					desired_transform_ = probot->GetTransform();
				}
				_SetDOFValues(desired_joints_value_vector_, desired_transform_, 0);
			}
			else 
			{
				_SetDOFValues(desired_joints_value_vector_, 0);
			}
			is_done_ = false;     // set after desired_joints_value_vector_ has changed
		}
		return true;
	}

	virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
	{
		OPENRAVE_ASSERT_FORMAT0(!ptraj || GetEnv() == ptraj->GetEnv(),
			"trajectory needs to come from the same environment as the controller", ORE_InvalidArguments);
		boost::mutex::scoped_lock lock(mutex_);
		if (is_pause_) 
		{
			RAVELOG_DEBUG("IdealController cannot start trajectories when paused\n");
			trajectory_.reset();
			is_done_ = true;
			return false;
		}
		command_time_ = 0;
		is_done_ = true;
		desired_joints_value_vector_.resize(0);
		trajectory_.reset();

		if (!!ptraj)
		{
			RobotBasePtr probot = robot_.lock();
			sample_specification_.groups_vector_.resize(0);
			is_trajectory_has_joints_ = false;
			if (!!group_joint_values_) 
			{
				// have to reset the name since group_joint_values_ can be using an old one
				std::stringstream ss;
				ss << "joint_values " << probot->GetName();
				for(auto it: dof_indices_)
				{
					ss << " " << it;
				}
				group_joint_values_->name = ss.str();
				is_trajectory_has_joints_ = ptraj->GetConfigurationSpecification().FindCompatibleGroup(group_joint_values_->name, false) != ptraj->GetConfigurationSpecification().groups_vector_.end();
				if (is_trajectory_has_joints_)
				{
					sample_specification_.groups_vector_.push_back(*group_joint_values_);
				}
			}
			is_trajectory_has_transform_ = false;
			if (!!group_transform_) 
			{
				// have to reset the name since group_transform_ can be using an old one
				group_transform_->name = str(boost::format("affine_transform %s %d") % probot->GetName() % DOF_Transform);
				is_trajectory_has_transform_ = ptraj->GetConfigurationSpecification().FindCompatibleGroup(group_transform_->name, false) != ptraj->GetConfigurationSpecification().groups_vector_.end();
				if (is_trajectory_has_transform_) 
				{
					sample_specification_.groups_vector_.push_back(*group_transform_);
				}
			}
			sample_specification_.ResetGroupOffsets();
			grab_links_vector_.resize(0);
			grab_body_links_vector_.resize(0);
			int dof = sample_specification_.GetDOF();
			for(auto itgroup: ptraj->GetConfigurationSpecification().groups_vector_) 
			{
				if (itgroup.name.size() >= 8 && itgroup.name.substr(0, 8) == "grabbody")
				{
					std::stringstream ss(itgroup.name);
					std::vector<std::string> tokens((std::istream_iterator<std::string>(ss)), std::istream_iterator<std::string>());
					if (itgroup.dof == 1 && tokens.size() >= 4)
					{
						if (tokens.at(2) == probot->GetName())
						{
							KinBodyPtr pbody = GetEnv()->GetKinBody(tokens.at(1));
							if (!!pbody)
							{
								sample_specification_.groups_vector_.push_back(itgroup);
								sample_specification_.groups_vector_.back().offset = dof;
								grab_body_links_vector_.push_back(GrabBody(dof, boost::lexical_cast<int>(tokens.at(3)), pbody));
								if (tokens.size() >= 11)
								{
									Transform trelativepose;
									trelativepose.rot[0] = boost::lexical_cast<dReal>(tokens[4]);
									trelativepose.rot[1] = boost::lexical_cast<dReal>(tokens[5]);
									trelativepose.rot[2] = boost::lexical_cast<dReal>(tokens[6]);
									trelativepose.rot[3] = boost::lexical_cast<dReal>(tokens[7]);
									trelativepose.trans[0] = boost::lexical_cast<dReal>(tokens[8]);
									trelativepose.trans[1] = boost::lexical_cast<dReal>(tokens[9]);
									trelativepose.trans[2] = boost::lexical_cast<dReal>(tokens[10]);
									grab_body_links_vector_.back().trelativepose.reset(new Transform(trelativepose));
								}
							}
							dof += sample_specification_.groups_vector_.back().dof;
						}
					}
					else
					{
						RAVELOG_WARN(str(boost::format("robot %s invalid grabbody tokens: %s") % probot->GetName() % ss.str()));
					}
				}
				else if (itgroup.name.size() >= 4 && itgroup.name.substr(0, 4) == "grab") 
				{
					std::stringstream ss(itgroup.name);
					std::vector<std::string> tokens((std::istream_iterator<std::string>(ss)), std::istream_iterator<std::string>());
					if (tokens.size() >= 2 && tokens[1] == probot->GetName()) 
					{
						sample_specification_.groups_vector_.push_back(itgroup);
						sample_specification_.groups_vector_.back().offset = dof;
						for (int idof = 0; idof < sample_specification_.groups_vector_.back().dof; ++idof) 
						{
							grab_links_vector_.emplace_back(dof + idof, boost::lexical_cast<int>(tokens.at(2 + idof)));
						}
						dof += sample_specification_.groups_vector_.back().dof;
					}
					else 
					{
						RAVELOG_WARN(str(boost::format("robot %s invalid grab tokens: %s") % probot->GetName() % ss.str()));
					}
				}
			}
			BOOST_ASSERT(sample_specification_.IsValid());

			// see if at least one point can be sampled, this make it easier to debug bad trajectories
			std::vector<dReal> v;
			ptraj->Sample(v, 0, sample_specification_);
			if (is_trajectory_has_transform_) 
			{
				Transform t;
				sample_specification_.ExtractTransform(t, v.begin(), probot);
			}

			if (!!flog_ && is_enable_logging_)
			{
				ptraj->serialize(flog_);
			}

			trajectory_ = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
			trajectory_->Clone(ptraj, 0);
			is_done_ = false;
		}

		return true;
	}

	virtual void SimulationStep(dReal time_elapsed)
	{
		if (is_pause_) 
		{
			return;
		}
		boost::mutex::scoped_lock lock(mutex_);
		TrajectoryBaseConstPtr ptraj = trajectory_; // because of multi-threading setting issues
		if (!!ptraj) 
		{
			RobotBasePtr probot = robot_.lock();
			std::vector<dReal> sampledata;
			ptraj->Sample(sampledata, command_time_, sample_specification_);

			// already sampled, so change the command times before before setting values
			// incase the below functions fail
			bool is_done = is_done_;
			if (command_time_ > ptraj->GetDuration()) 
			{
				command_time_ = ptraj->GetDuration();
				is_done = true;
			}
			else 
			{
				command_time_ += speed_ * time_elapsed;
			}

			// first process all grab info
			std::list<KinBodyPtr> listrelease;
			std::list<pair<KinBodyPtr, KinBody::LinkPtr> > listgrab;
			std::list<int> listgrabindices;
			for(auto& itgrabinfo: grab_links_vector_) 
			{
				int bodyid = int(std::floor(sampledata.at(itgrabinfo.first) + 0.5));
				if (bodyid != 0) 
				{
					KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(abs(bodyid));
					if (!pbody)
					{
						RAVELOG_WARN(str(boost::format("failed to find body id %d") % bodyid));
						continue;
					}
					if (bodyid < 0) 
					{
						if (!!probot->IsGrabbing(*pbody))
						{
							listrelease.push_back(pbody);
						}
					}
					else 
					{
						KinBody::LinkPtr pgrabbinglink = probot->IsGrabbing(*pbody);
						if (!!pgrabbinglink) 
						{
							if (pgrabbinglink->GetIndex() != itgrabinfo.second) 
							{
								listrelease.push_back(pbody);
								listgrab.emplace_back(pbody, probot->GetLinks().at(itgrabinfo.second));
							}
						}
						else 
						{
							listgrab.emplace_back(pbody, probot->GetLinks().at(itgrabinfo.second));
						}
					}
				}
			}
			FOREACH(itgrabinfo, grab_body_links_vector_) 
			{
				int dograb = int(std::floor(sampledata.at(itgrabinfo->offset) + 0.5));
				if (dograb <= 0) 
				{
					if (!!probot->IsGrabbing(*itgrabinfo->pbody))
					{
						listrelease.push_back(itgrabinfo->pbody);
					}
				}
				else 
				{
					KinBody::LinkPtr pgrabbinglink = probot->IsGrabbing(*itgrabinfo->pbody);
					if (!!pgrabbinglink) 
					{
						listrelease.push_back(itgrabinfo->pbody);
					}
					listgrabindices.push_back(static_cast<int>(itgrabinfo - grab_body_links_vector_.begin()));
				}
			}

			std::vector<dReal> dof_values_vector;
			if (is_trajectory_has_joints_ && dof_indices_.size() > 0)
			{
				dof_values_vector.resize(dof_indices_.size());
				sample_specification_.ExtractJointValues(dof_values_vector.begin(), sampledata.begin(), probot, dof_indices_, 0);
			}

			Transform t;
			if (is_trajectory_has_transform_ && control_transformation_)
			{
				sample_specification_.ExtractTransform(t, sampledata.begin(), probot);
				if (dof_values_vector.size() > 0)
				{
					_SetDOFValues(dof_values_vector, t, command_time_ > 0 ? time_elapsed : 0);
				}
				else 
				{
					probot->SetTransform(t);
				}
			}
			else if (dof_values_vector.size() > 0) 
			{
				_SetDOFValues(dof_values_vector, command_time_ > 0 ? time_elapsed : 0);
			}

			// always release after setting dof values
			for(auto& itbody: listrelease)
			{
				probot->Release(*itbody);
			}
			for(auto&itindex: listgrabindices)
			{
				const GrabBody& grabinfo = grab_body_links_vector_.at(itindex);
				KinBody::LinkPtr plink = probot->GetLinks().at(grabinfo.robotlinkindex);
				if (!!grabinfo.trelativepose) 
				{
					grabinfo.pbody->SetTransform(plink->GetTransform() * *grabinfo.trelativepose);
				}
				probot->Grab(grabinfo.pbody, plink);
			}
			for(auto& it: listgrab) 
			{
				probot->Grab(it.first, it.second);
			}
			// set is_done_ after all computation is done!
			is_done_ = is_done;
			if (is_done) 
			{
				// trajectory is done, so reset it so that the controller doesn't continously set the dof values (which can get annoying)
				trajectory_.reset();
			}
		}

		if (desired_joints_value_vector_.size() > 0) 
		{
			if (control_transformation_)
			{
				_SetDOFValues(desired_joints_value_vector_, desired_transform_, 0);
			}
			else 
			{
				_SetDOFValues(desired_joints_value_vector_, 0);
			}
			is_done_ = true;
			// don't need to set it anymore
			desired_joints_value_vector_.resize(0);
		}
	}

	virtual bool IsDone() 
	{
		return is_done_;
	}
	virtual dReal GetTime() const 
	{
		return command_time_;
	}
	virtual RobotBasePtr GetRobot() const 
	{
		return robot_.lock();
	}

private:
	virtual bool _Pause(std::ostream& os, std::istream& is)
	{
		is >> is_pause_;
		return !!is;
	}
	virtual bool _SetCheckCollisions(std::ostream& os, std::istream& is)
	{
		is >> is_check_collision_;
		if (is_check_collision_)
		{
			report_.reset(new CollisionReport());
		}
		return !!is;
	}
	virtual bool _SetThrowExceptions(std::ostream& os, std::istream& is)
	{
		is >> is_throw_exceptions_;
		return !!is;
	}
	virtual bool _SetEnableLogging(std::ostream& os, std::istream& is)
	{
		is >> is_enable_logging_;
		return !!is;
	}

	inline std::shared_ptr<IdealController> shared_controller()
	{
		return std::static_pointer_cast<IdealController>(shared_from_this());
	}
	inline std::shared_ptr<IdealController const> shared_controller_const() const 
	{
		return std::static_pointer_cast<IdealController const>(shared_from_this());
	}
	inline std::weak_ptr<IdealController> weak_controller()
	{
		return shared_controller();
	}

	virtual void _SetJointLimits()
	{
		RobotBasePtr probot = robot_.lock();
		if (!!probot) 
		{
			probot->GetDOFLimits(lower_vector_[0], upper_vector_[0]);
			probot->GetDOFVelocityLimits(upper_vector_[1]);
			probot->GetDOFAccelerationLimits(upper_vector_[2]);
		}
	}

	virtual void _SetDOFValues(const std::vector<dReal>&values, dReal timeelapsed)
	{
		RobotBasePtr probot = robot_.lock();

		std::vector<dReal> prevvalues, curvalues, curvel;
		probot->GetDOFValues(prevvalues);
		curvalues = prevvalues;
		probot->GetDOFVelocities(curvel);
		Vector linearvel, angularvel;
		probot->GetLinks().at(0)->GetVelocity(linearvel, angularvel);
		int i = 0;
		for(auto& it: dof_indices_) 
		{
			curvalues.at(it) = values.at(i++);
			curvel.at(it) = 0;
		}
		_CheckLimits(probot, prevvalues, curvalues, timeelapsed);
		probot->SetDOFValues(curvalues, true);
		probot->SetDOFVelocities(curvel, linearvel, angularvel);
		_CheckConfiguration(probot);
	}
	virtual void _SetDOFValues(const std::vector<dReal>&values, const Transform &t, dReal timeelapsed)
	{
		RobotBasePtr probot = robot_.lock();
		BOOST_ASSERT(control_transformation_);
		std::vector<dReal> prevvalues, curvalues, curvel;
		probot->GetDOFValues(prevvalues);
		curvalues = prevvalues;
		probot->GetDOFVelocities(curvel);
		int i = 0;
		for(auto& it: dof_indices_)
		{
			curvalues.at(it) = values.at(i++);
			curvel.at(it) = 0;
		}
		_CheckLimits(probot, prevvalues, curvalues, timeelapsed);
		probot->SetDOFValues(curvalues, t, true);
		probot->SetDOFVelocities(curvel, Vector(), Vector());
		_CheckConfiguration(probot);
	}

	void _CheckLimits(RobotBasePtr probot, std::vector<dReal>& prevvalues, std::vector<dReal>&curvalues, dReal timeelapsed)
	{
		for (size_t i = 0; i < lower_vector_[0].size(); ++i) 
		{
			if (!is_dof_circular_vector_[i])
			{
				if (curvalues.at(i) < lower_vector_[0][i] - g_fEpsilonJointLimit)
				{
					_ReportError(str(boost::format("robot %s dof %d is violating lower limit %e < %e, time=%f")
						% probot->GetName() % i%lower_vector_[0][i] % curvalues[i] % command_time_));
				}
				if (curvalues.at(i) > upper_vector_[0][i] + g_fEpsilonJointLimit)
				{
					_ReportError(str(boost::format("robot %s dof %d is violating upper limit %e > %e, time=%f")
						% probot->GetName() % i%upper_vector_[0][i] % curvalues[i] % command_time_));
				}
			}
		}
		if (timeelapsed > 0) 
		{
			std::vector<dReal> vdiff = curvalues;
			probot->SubtractDOFValues(vdiff, prevvalues);
			for (size_t i = 0; i < upper_vector_[1].size(); ++i) 
			{
				dReal maxallowed = timeelapsed * upper_vector_[1][i] + 1e-6;
				if (RaveFabs(vdiff.at(i)) > maxallowed) 
				{
					_ReportError(str(boost::format("robot %s dof %d is violating max velocity displacement %.15e > %.15e, time=%f") 
						% probot->GetName() % i%RaveFabs(vdiff.at(i)) % maxallowed%command_time_));
				}
			}
		}
	}

	void _CheckConfiguration(RobotBasePtr probot)
	{
		if (is_check_collision_) 
		{
			if (GetEnv()->CheckCollision(KinBodyConstPtr(probot), report_))
			{
				_ReportError(str(boost::format("collsion in trajectory: %s, time=%f\n")
					% report_->__str__() % command_time_));
			}
			if (probot->CheckSelfCollision(report_))
			{
				_ReportError(str(boost::format("self collsion in trajectory: %s, time=%f\n")
					% report_->__str__() % command_time_));
			}
		}
	}

	void _ReportError(const std::string& s)
	{
		if (!!trajectory_)
		{
			if (IS_DEBUGLEVEL(Level_Verbose))
			{
				std::string filename = str(boost::format("%s/failedtrajectory%d.xml") 
					% RaveGetHomeDirectory() % (RaveRandomInt() % 1000));
				std::ofstream f(filename.c_str());
				f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);     /// have to do this or otherwise precision gets lost
				trajectory_->serialize(f);
				RAVELOG_VERBOSE(str(boost::format("trajectory dumped to %s") % filename));
			}
		}
		if (is_throw_exceptions_) 
		{
			throw OpenRAVEException(s, ORE_Assert);
		}
		else 
		{
			RAVELOG_WARN(s);
		}
	}

	RobotBaseWeakPtr robot_;               //!< controlled body
	dReal speed_;                    //!< how fast the robot should go
	TrajectoryBasePtr trajectory_;         //!< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()
	bool is_trajectory_has_joints_, is_trajectory_has_transform_;
	std::vector< pair<int, int> > grab_links_vector_; /// (data offset, link index) pairs
	struct GrabBody
	{
		GrabBody() : offset(0), robotlinkindex(0) 
		{
		}
		GrabBody(int offset, int robotlinkindex, KinBodyPtr pbody)
			: offset(offset), robotlinkindex(robotlinkindex), pbody(pbody) 
		{
		}
		int offset;
		int robotlinkindex;
		KinBodyPtr pbody;
		std::shared_ptr<Transform> trelativepose; //!< relative pose of body with link when grabbed. if it doesn't exist, then do not pre-transform the pose
	};
	std::vector<GrabBody> grab_body_links_vector_;
	dReal command_time_;

	std::vector<dReal> desired_joints_value_vector_;         //!< desired values of the joints
	Transform desired_transform_;

	std::vector<int> dof_indices_;
	std::vector<uint8_t> is_dof_circular_vector_;
	std::array< std::vector<dReal>, 3> lower_vector_, upper_vector_; //!< position, velocity, acceleration limits
	int control_transformation_;
	std::ofstream flog_;
	int cmd_id_;
	bool is_pause_, is_done_, is_check_collision_, is_throw_exceptions_, is_enable_logging_;
	CollisionReportPtr report_;
	UserDataPtr limits_callback_;
	ConfigurationSpecification sample_specification_;
	std::shared_ptr<ConfigurationSpecification::Group> group_joint_values_, group_transform_;
	boost::mutex mutex_;
};

ControllerBasePtr CreateIdealController(EnvironmentBasePtr penv, std::istream& sinput)
{
	return ControllerBasePtr(new IdealController(penv, sinput));
}

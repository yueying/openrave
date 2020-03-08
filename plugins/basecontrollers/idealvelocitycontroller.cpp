// -*- coding: utf-8 -*-
// Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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

class IdealVelocityController : public ControllerBase
{
public:
    IdealVelocityController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv)
    {
        description_ = ":Interface Authors: Rosen Diankov\n\nIdeal Velocity controller.";
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int control_transformation)
    {
        robot_ = robot;
        dof_indices_vector_ = dofindices;
        if( control_transformation ) 
		{
            RAVELOG_WARN("odevelocity controller cannot control transformation\n");
        }
        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
//        if( !!robot_ ) {
//            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
//            //robot_->GetDOFVelocities(_vPreviousVelocities,_dofindices);
//        }
        is_velocity_mode_ = false;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const 
	{
        return dof_indices_vector_;
    }

    virtual int IsControlTransformation() const 
	{
        return 0;
    }

    virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans) 
	{
        OPENRAVE_ASSERT_OP(values.size(),==,dof_indices_vector_.size());
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        desired_velocities_vector_ = values;
        std::vector<dReal> vallvelocities;
        robot_->GetDOFVelocities(vallvelocities);
        for(size_t i = 0; i < dof_indices_vector_.size(); ++i) 
		{
            vallvelocities.at(dof_indices_vector_[i]) = desired_velocities_vector_.at(i);
        }
        robot_->SetDOFVelocities(vallvelocities);
        is_velocity_mode_ = true;
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
	{
        return false;
    }
    virtual void SimulationStep(OpenRAVE::dReal time_elapsed) 
	{
        if( is_velocity_mode_ ) 
		{
            std::vector<dReal> vallvelocities;
            robot_->GetDOFVelocities(vallvelocities);
            for(size_t i = 0; i < dof_indices_vector_.size(); ++i)
			{
                vallvelocities.at(dof_indices_vector_[i]) = desired_velocities_vector_.at(i);
            }

            std::vector<dReal> vprevvalues;
            robot_->GetDOFValues(vprevvalues,dof_indices_vector_);
            for(size_t i = 0; i < dof_indices_vector_.size(); ++i) 
			{
                vprevvalues[i] += time_elapsed*desired_velocities_vector_[i];
            }
            robot_->SetDOFValues(vprevvalues,true,dof_indices_vector_);
            robot_->SetDOFVelocities(vallvelocities); // set after SetDOFValues in order to get correct link velocities
        }
    }
    virtual bool IsDone()
	{
        return !is_velocity_mode_;
    }
    virtual OpenRAVE::dReal GetTime() const 
	{
        return 0;
    }
    virtual RobotBasePtr GetRobot() const
	{
        return robot_;
    }

protected:
    RobotBasePtr robot_;
    std::vector<int> dof_indices_vector_;
    std::vector<dReal> desired_velocities_vector_; //, _vPreviousVelocities;
    bool is_velocity_mode_;
    OpenRAVE::UserDataPtr _torquechangedhandle;
};

ControllerBasePtr CreateIdealVelocityController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new IdealVelocityController(penv,sinput));
}

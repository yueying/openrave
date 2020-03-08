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

class RedirectController : public ControllerBase
{
public:
    RedirectController(EnvironmentBasePtr penv, std::istream& sinput)
		: ControllerBase(penv), is_auto_sync_(true) 
	{
        description_ = ":Interface Author: Rosen Diankov\n\nRedirects all input and output to another controller\
        (this avoides cloning the other controller while still allowing it to be used from cloned environments)";
    }

    virtual ~RedirectController()
	{
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>&dofindices, int control_transformation)
    {
        dof_indices_vector_.clear();
        controller_.reset();
        robot_ = GetEnv()->GetRobot(robot->GetName());
        if( robot_ != robot ) 
		{
            controller_ = robot->GetController();
            if( !!controller_ ) 
			{
                dof_indices_vector_ = controller_->GetControlDOFIndices();
            }
        }
        if( is_auto_sync_ ) 
		{
            _sync();
        }
        return true;
    }

    // don't touch the referenced controller, since could be just destroying clones
    virtual void Reset(int options)
	{
    }

    virtual bool SetDesired(const std::vector<dReal>&values, TransformConstPtr trans)
    {
        if( !controller_->SetDesired(values, trans) )
		{
            return false;
        }
        if(is_auto_sync_)
		{
            _sync();
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !controller_->SetPath(ptraj) ) 
		{
            return false;
        }
        if(is_auto_sync_) 
		{
            _sync();
        }
        return true;
    }

    virtual void SimulationStep(dReal time_elapsed)
	{
        if( !!controller_ ) 
		{
            controller_->SimulationStep(time_elapsed);
            if(is_auto_sync_) 
			{
                _sync();
            }
        }
    }

    virtual const std::vector<int>& GetControlDOFIndices() const
	{
        return dof_indices_vector_;
    }
    virtual int IsControlTransformation() const 
	{
        return !controller_ ? 0 : controller_->IsControlTransformation();
    }
    virtual bool IsDone()
	{
        return is_auto_sync_ ? is_sync_done_&&controller_->IsDone() : controller_->IsDone();
    }

    virtual dReal GetTime() const 
	{
        return controller_->GetTime();
    }

    virtual void GetVelocity(std::vector<dReal>&vel) const 
	{
        return controller_->GetVelocity(vel);
    }

    virtual void GetTorque(std::vector<dReal>&torque) const 
	{
        return controller_->GetTorque(torque);
    }

    virtual RobotBasePtr GetRobot() const 
	{
        return robot_;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        ControllerBase::Clone(preference,cloningoptions);
        std::shared_ptr<RedirectController const> r = std::dynamic_pointer_cast<RedirectController const>(preference);
        robot_ = GetEnv()->GetRobot(r->robot_->GetName());
        controller_ = r->controller_;     // hmm......... this requires some thought
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        std::string cmd;
		std::streampos pos = is.tellg();
        is >> cmd;
        if( !is )
		{
            throw OpenRAVEException("invalid argument",ORE_InvalidArguments);
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "sync" ) 
		{
            _sync();
            return true;
        }
        else if( cmd == "autosync" ) 
		{
            is >> is_auto_sync_;
            if( !is ) 
			{
                return false;
            }
            if( is_auto_sync_ )
			{
                _sync();
            }
            return true;
        }

        is.seekg(pos);
        if( !controller_ ) 
		{
            return false;
        }
        return controller_->SendCommand(os,is);
    }

private:
    virtual void _sync()
    {
        if( !!controller_ ) 
		{
			std::vector<Transform> vtrans;
            controller_->GetRobot()->GetLinkTransformations(vtrans);
            robot_->SetLinkTransformations(vtrans);
            is_sync_done_ = controller_->IsDone();
        }
    }

    std::vector<int> dof_indices_vector_;
    bool is_auto_sync_, is_sync_done_;
    RobotBasePtr robot_;               //!< controlled body
    ControllerBasePtr controller_;
};

ControllerBasePtr CreateRedirectController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new RedirectController(penv,sinput));
}

// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "libopenrave.h"

namespace OpenRAVE 
{

bool IkReturn::Append(const IkReturn& r)
{
    bool is_clashing = false;
    if( !!r.user_data_ ) 
	{
        if( !!user_data_ ) 
		{
            RAVELOG_WARN("IkReturn already has user_data_ set, but overwriting anyway\n");
            is_clashing = true;
        }
        user_data_ = r.user_data_;
    }
    if( custom_data_.size() == 0 ) 
	{
        custom_data_ = r.custom_data_;
    }
    else 
	{
        for(auto& itr:r.custom_data_) 
		{
            if( !custom_data_.insert(itr).second ) 
			{
                // actually this is pretty normal if a previous iksolution failed during the filters and it left old data...
                //RAVELOG_WARN(str(boost::format("IkReturn custom_data_ %s overwritten")%itr->first));
                is_clashing = true;
            }
        }
    }
    if( r.solution_.size() > 0 ) 
	{
        if( solution_.size() > 0 )
		{
            RAVELOG_WARN("IkReturn already has solution_ set, but overwriting anyway\n");
            is_clashing = true;
        }
        solution_ = r.solution_;
    }
    return is_clashing;
}

void IkReturn::Clear()
{
    custom_data_.clear();
    user_data_.reset();
    solution_.resize(0);
    //_reports.resize(0); // TODO
}

class CustomIkSolverFilterData : public std::enable_shared_from_this<CustomIkSolverFilterData>, public UserData
{
public:
    CustomIkSolverFilterData(int32_t priority,
		const IkSolverBase::IkFilterCallbackFn& filterfn, IkSolverBasePtr iksolver) 
		: _priority(priority), _filterfn(filterfn), _iksolverweak(iksolver) 
	{
    }
    virtual ~CustomIkSolverFilterData() 
	{
        IkSolverBasePtr iksolver = _iksolverweak.lock();
        if( !!iksolver ) 
		{
            iksolver->registered_filters_.erase(_iterator);
        }
    }

    int32_t _priority; //!< has to be 32bit
    IkSolverBase::IkFilterCallbackFn _filterfn;
    IkSolverBaseWeakPtr _iksolverweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef std::shared_ptr<CustomIkSolverFilterData> CustomIkSolverFilterDataPtr;

class IkSolverFinishCallbackData : public std::enable_shared_from_this<IkSolverFinishCallbackData>, public UserData
{
public:
    IkSolverFinishCallbackData(const IkSolverBase::IkFinishCallbackFn& finishfn, IkSolverBasePtr iksolver)
		: _finishfn(finishfn), _iksolverweak(iksolver) 
	{
    }
    virtual ~IkSolverFinishCallbackData()
	{
        IkSolverBasePtr iksolver = _iksolverweak.lock();
        if( !!iksolver ) 
		{
            iksolver->registered_finish_callbacks_.erase(_iterator);
        }
    }

    IkSolverBase::IkFinishCallbackFn _finishfn;
    IkSolverBaseWeakPtr _iksolverweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef std::shared_ptr<IkSolverFinishCallbackData> IkSolverFinishCallbackDataPtr;

bool CustomIkSolverFilterDataCompare(UserDataPtr data0, UserDataPtr data1)
{
    return std::dynamic_pointer_cast<CustomIkSolverFilterData>(data0)->_priority > std::dynamic_pointer_cast<CustomIkSolverFilterData>(data1)->_priority;
}

bool IkSolverBase::Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, IkReturnPtr ikreturn)
{
    if( !ikreturn ) 
	{
        return Solve(param,q0,filteroptions,std::shared_ptr< vector<dReal> >());
    }
    ikreturn->Clear();
    std::shared_ptr< vector<dReal> > psolution(&ikreturn->solution_, utils::null_deleter());
    if( !Solve(param,q0,filteroptions,psolution) )
	{
        ikreturn->action_ = IKRA_Reject;
        return false;
    }
    ikreturn->action_ = IKRA_Success;
    return true;
}

bool IkSolverBase::SolveAll(const IkParameterization& param, int filteroptions, std::vector<IkReturnPtr>& ikreturns)
{
    ikreturns.resize(0);
    std::vector< std::vector<dReal> > vsolutions;
    if( !SolveAll(param,filteroptions,vsolutions) ) 
	{
        return false;
    }
    ikreturns.resize(vsolutions.size());
    for(size_t i = 0; i < ikreturns.size(); ++i) 
	{
        ikreturns[i].reset(new IkReturn(IKRA_Success));
        ikreturns[i]->solution_ = vsolutions[i];
        ikreturns[i]->action_ = IKRA_Success;
    }
    return vsolutions.size() > 0;
}

bool IkSolverBase::Solve(const IkParameterization& param, const std::vector<dReal>& q0, 
	const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn)
{
    if( !ikreturn ) 
	{
        return Solve(param,q0,vFreeParameters,filteroptions,std::shared_ptr< vector<dReal> >());
    }
    ikreturn->Clear();
    std::shared_ptr< vector<dReal> > psolution(&ikreturn->solution_, utils::null_deleter());
    if( !Solve(param,q0,vFreeParameters,filteroptions,psolution) )
	{
        ikreturn->action_ = IKRA_Reject;
        return false;
    }
    ikreturn->action_ = IKRA_Success;
    return true;
}

bool IkSolverBase::SolveAll(const IkParameterization& param, 
	const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& ikreturns)
{
    ikreturns.resize(0);
    std::vector< std::vector<dReal> > vsolutions;
    if( !SolveAll(param,vFreeParameters,filteroptions,vsolutions) )
	{
        return false;
    }
    ikreturns.resize(vsolutions.size());
    for(size_t i = 0; i < ikreturns.size(); ++i)
	{
        ikreturns[i].reset(new IkReturn(IKRA_Success));
        ikreturns[i]->solution_ = vsolutions[i];
        ikreturns[i]->action_ = IKRA_Success;
    }
    return vsolutions.size() > 0;
}

UserDataPtr IkSolverBase::RegisterCustomFilter(int32_t priority, const IkSolverBase::IkFilterCallbackFn &filterfn)
{
    CustomIkSolverFilterDataPtr pdata(new CustomIkSolverFilterData(priority,filterfn,shared_iksolver()));
    std::list<UserDataWeakPtr>::iterator it;
	for (it = registered_filters_.begin(); it != registered_filters_.end(); ++(it))
	{
        CustomIkSolverFilterDataPtr pitdata = std::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata && pdata->_priority > pitdata->_priority )
		{
            break;
        }
    }
    pdata->_iterator = registered_filters_.insert(it,pdata);
    return pdata;
}

UserDataPtr IkSolverBase::RegisterFinishCallback(const IkFinishCallbackFn& finishfn)
{
    IkSolverFinishCallbackDataPtr pdata(new IkSolverFinishCallbackData(finishfn,shared_iksolver()));
    pdata->_iterator = registered_finish_callbacks_.insert(registered_finish_callbacks_.end(), pdata);
    return pdata;
}

IkReturnAction IkSolverBase::_CallFilters(std::vector<dReal>& solution, 
	RobotBase::ManipulatorPtr manipulator, const IkParameterization& param, 
	IkReturnPtr filterreturn, int32_t minpriority, int32_t maxpriority)
{
    std::vector<dReal> vtestsolution,vtestsolution2;
    if( IS_DEBUGLEVEL(Level_Verbose) || (RaveGetDebugLevel() & Level_VerifyPlans) ) 
	{
        RobotBasePtr robot = manipulator->GetRobot();
        robot->GetConfigurationValues(vtestsolution);
        for(size_t i = 0; i < manipulator->GetArmIndices().size(); ++i) 
		{
            int dofindex = manipulator->GetArmIndices()[i];
            dReal fdiff = 0;
            KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(dofindex);
            fdiff = pjoint->SubtractValue(vtestsolution.at(dofindex), solution.at(i), dofindex-pjoint->GetDOFIndex());
            if( fdiff > g_fEpsilonJointLimit ) 
			{
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                ss << "dof " << i << " of solution=[";
                for(auto& itvalue: solution) 
				{
                    ss << itvalue << ", ";
                }
                ss << "] != dof " << dofindex << " of currentvalues=[";
                for(auto& itvalue: vtestsolution)
				{
                    ss << itvalue << ", ";
                }
                ss << "]";
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("_CallFilters on robot %s manip %s need to start with robot configuration set to the solution, most likely a problem with internal ik solver call. %s"),robot->GetName()%manipulator->GetName()%ss.str(), ORE_InconsistentConstraints);
            }
        }
    }

    FOREACHC(it,registered_filters_) {
        CustomIkSolverFilterDataPtr pitdata = std::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata && pitdata->_priority >= minpriority && pitdata->_priority <= maxpriority) {
            IkReturn ret = pitdata->_filterfn(solution,manipulator,param);
            if( ret != IKRA_Success ) {
                return ret.action_; // just return the action
            }
            if( vtestsolution.size() > 0 ) {
                // check that the robot is set to solution
                RobotBasePtr robot = manipulator->GetRobot();
                std::vector<dReal> vtestsolution2;
                robot->GetConfigurationValues(vtestsolution2);
                for(size_t i = 0; i < manipulator->GetArmIndices().size(); ++i) {
                    vtestsolution.at(manipulator->GetArmIndices()[i]) = solution.at(i);
                }
                for(size_t i = 0; i < vtestsolution.size(); ++i) {
                    if( RaveFabs(vtestsolution.at(i)-vtestsolution2.at(i)) > g_fEpsilonJointLimit ) {
                        int dofindex = manipulator->GetArmIndices()[i];
                        KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(dofindex); // for debugging
                        
                        std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        ss << "dof " << dofindex << " of solution=[";
                        FOREACH(itvalue, vtestsolution) {
                            ss << *itvalue << ", ";
                        }
                        ss << "] != dof " << dofindex << " of currentvalues=[";
                        FOREACH(itvalue, vtestsolution2) {
                            ss << *itvalue << ", ";
                        }
                        ss << "]";

                        robot->GetConfigurationValues(vtestsolution2);
                        pitdata->_filterfn(solution,manipulator,param); // for debugging internals
                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("one of the filters set on robot %s manip %s did not restore the robot configuraiton. %s"),robot->GetName()%manipulator->GetName()%ss.str(), ORE_InconsistentConstraints);
                    }
                }
            }
            if( !!filterreturn ) {
                filterreturn->Append(ret);
            }
        }
    }
    if( !!filterreturn ) {
        filterreturn->action_ = IKRA_Success;
    }
    return IKRA_Success;
}

bool IkSolverBase::_HasFilterInRange(int32_t minpriority, int32_t maxpriority) const
{
    // priorities are descending
    FOREACHC(it,registered_filters_) {
        CustomIkSolverFilterDataPtr pitdata = std::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata ) {
            if( pitdata->_priority <= maxpriority && pitdata->_priority >= minpriority ) {
                return true;
            }
        }
    }
    return false;
}

void IkSolverBase::_CallFinishCallbacks(IkReturnPtr ikreturn, 
	RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikparam)
{
    for(auto& it: registered_finish_callbacks_) 
	{
        IkSolverFinishCallbackDataPtr pitdata = std::dynamic_pointer_cast<IkSolverFinishCallbackData>(it.lock());
        if( !!pitdata ) 
		{
            pitdata->_finishfn(ikreturn, pmanip, ikparam);
        }
    }
}

}

// -*- coding: utf-8 -*-
// Copyright (C) 2006-2019 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <openrave/grabbed.h>
#include <openrave/openrave_macros.h>
namespace OpenRAVE 

{
void RobotBase::ManipulatorInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{

    openravejson::SetJsonValueByKey(value, "name", name_, allocator);
    openravejson::SetJsonValueByKey(value, "transform", local_tool_transform_, allocator);
    openravejson::SetJsonValueByKey(value, "chuckingDirections", chucking_direction_vector_, allocator);
    openravejson::SetJsonValueByKey(value, "direction", direction_, allocator);
    openravejson::SetJsonValueByKey(value, "baseLinkName", base_link_name_, allocator);
    openravejson::SetJsonValueByKey(value, "effectorLinkName", effector_link_name_, allocator);
    openravejson::SetJsonValueByKey(value, "ikSolverType", ik_solver_xml_id_, allocator);
    openravejson::SetJsonValueByKey(value, "gripperJointNames", gripper_joint_names_vector_, allocator);
}

void RobotBase::ManipulatorInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
    openravejson::LoadJsonValueByKey(value, "name", name_);
    openravejson::LoadJsonValueByKey(value, "transform", local_tool_transform_);
    openravejson::LoadJsonValueByKey(value, "chuckingDirections", chucking_direction_vector_);
    openravejson::LoadJsonValueByKey(value, "direction", direction_);
    openravejson::LoadJsonValueByKey(value, "baseLinkName", base_link_name_);
    openravejson::LoadJsonValueByKey(value, "effectorLinkName", effector_link_name_);
    openravejson::LoadJsonValueByKey(value, "iksolverType", ik_solver_xml_id_);
    openravejson::LoadJsonValueByKey(value, "gripperJointNames", gripper_joint_names_vector_);
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, const RobotBase::ManipulatorInfo& info) 
	: info_(info), robot_(probot) 
{
}
RobotBase::Manipulator::~Manipulator() 
{
}

RobotBase::Manipulator::Manipulator(const RobotBase::Manipulator& r)
{
    *this = r;
    ik_solver_.reset();
    if( info_.ik_solver_xml_id_.size() > 0 ) {
        ik_solver_ = RaveCreateIkSolver(GetRobot()->GetEnv(), info_.ik_solver_xml_id_);
    }
}

RobotBase::Manipulator::Manipulator(RobotBasePtr probot, std::shared_ptr<RobotBase::Manipulator const> r)
{
    *this = *r.get();
    robot_ = probot;
    if( !!r->GetBase() ) {
        base_link_ = probot->GetLinks().at(r->GetBase()->GetIndex());
    }
    if( !!r->GetEndEffector() ) {
        effector_link_ = probot->GetLinks().at(r->GetEndEffector()->GetIndex());
    }
    ik_solver_.reset(); // will be initialized when needed. Problem: the freeinc parameters will not get transferred to the new iksolver
//    if( info_.ik_solver_xml_id_.size() > 0 ) {
//        //ik_solver_ = RaveCreateIkSolver(probot->GetEnv(), info_.ik_solver_xml_id_);
//        // cannot call ik_solver_->Init since this is the constructor...
//    }
}

int RobotBase::Manipulator::GetArmDOF() const
{
    return static_cast<int>(arm_dof_indices_.size());
}

int RobotBase::Manipulator::GetGripperDOF() const
{
    return static_cast<int>(gripper_dof_indices_.size());
}

void RobotBase::Manipulator::SetChuckingDirection(const std::vector<dReal>& chuckingdirection)
{
    OPENRAVE_ASSERT_OP((int)chuckingdirection.size(),==,GetGripperDOF());
    info_.chucking_direction_vector_ = chuckingdirection;
    GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
}

void RobotBase::Manipulator::SetLocalToolTransform(const Transform& t)
{
    info_.ik_solver_xml_id_.resize(0);
    // only reset the iksolver if transform is different. note that the correct way is to see if the hash changes, but that could take too much computation. also hashes truncate the numbers.
    if( !!ik_solver_ && TransformDistance2(info_.local_tool_transform_, t) > g_fEpsilonLinear ) {
        ik_solver_.reset();
    }
    info_.local_tool_transform_ = t;
    hash_kinematics_structure_.resize(0);
    hash_structure_.resize(0);
    hash_ik_structure_map_.clear();
    GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
}

void RobotBase::Manipulator::SetLocalToolDirection(const Vector& direction)
{
    info_.ik_solver_xml_id_.resize(0);
    ik_solver_.reset();
    // only reset the iksolver if transform is different. note that the correct way is to see if the hash changes, but that could take too much computation. also hashes truncate the numbers.
    if( (info_.direction_-direction).lengthsqr3() > g_fEpsilonLinear ) {
        // only reset if direction is actually used
        if( !!ik_solver_ && (ik_solver_->Supports(IKP_TranslationDirection5D) || ik_solver_->Supports(IKP_Direction3D)) ) {
            ik_solver_.reset();
        }
    }
    info_.direction_ = direction*(1/RaveSqrt(direction.lengthsqr3())); // should normalize
    hash_kinematics_structure_.resize(0);
    hash_structure_.resize(0);
    hash_ik_structure_map_.clear();
    GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorTool);
}

void RobotBase::Manipulator::SetName(const std::string& name)
{
    RobotBasePtr probot=GetRobot();
    FOREACHC(itmanip,probot->GetManipulators()) {
        if( *itmanip != shared_from_this() && name == (*itmanip)->GetName() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("manipulator name change '%s'->'%s' is colliding with other manipulator"), info_.name_%name, ORE_InvalidArguments);
        }
    }
    info_.name_=name;
    probot->_PostprocessChangedParameters(Prop_RobotManipulatorName);
}

Transform RobotBase::Manipulator::GetTransform() const
{
    return effector_link_->GetTransform() * info_.local_tool_transform_;
}

std::pair<Vector,Vector> RobotBase::Manipulator::GetVelocity() const
{
    Vector vdifference = effector_link_->GetTransform().rotate(info_.local_tool_transform_.trans);
    std::pair<Vector,Vector> velocity = effector_link_->GetVelocity();
    velocity.first += velocity.second.cross(vdifference);
    return velocity;
}

IkSolverBasePtr RobotBase::Manipulator::GetIkSolver() const
{
    if( !!ik_solver_ || info_.ik_solver_xml_id_.size() == 0 ) 
	{
        return ik_solver_;
    }

    // initialize ik solver
    try 
	{
        if( info_.ik_solver_xml_id_.size() > 0 ) 
		{
            RobotBasePtr probot(robot_);
            ik_solver_ = RaveCreateIkSolver(probot->GetEnv(), info_.ik_solver_xml_id_);
            if( !!ik_solver_ )
			{
                // note that ik solvers might look at the manipulator hashes for verification
                ik_solver_->Init(shared_from_this());
            }
        }
    }
    catch(const std::exception& e) 
	{
        RAVELOG_WARN(str(boost::format("failed to init ik solver: %s\n")%e.what()));
        ik_solver_.reset();
    }
    return ik_solver_;
}

bool RobotBase::Manipulator::SetIkSolver(IkSolverBasePtr iksolver)
{
    if( !iksolver ) 
	{
        ik_solver_.reset();
        return true;
    }

    if( iksolver->GetXMLId().size() == 0 )
	{
        RAVELOG_WARN(str(boost::format("robot %s manip %s IkSolver XML is not initialized\n")
			%GetRobot()->GetName()%GetName()));
    }
    if( iksolver == ik_solver_ && info_.ik_solver_xml_id_ == iksolver->GetXMLId() )
	{
        return true;
    }

    // only call the changed message if something changed
    if( iksolver->Init(shared_from_this()) )
	{
        ik_solver_ = iksolver;
        info_.ik_solver_xml_id_ = iksolver->GetXMLId();
        GetRobot()->_PostprocessChangedParameters(Prop_RobotManipulatorSolver);
        return true;
    }

    return false;
}

void RobotBase::Manipulator::GetArmDOFValues(std::vector<dReal>& v) const
{
    GetRobot()->GetDOFValues(v, arm_dof_indices_);
}

void RobotBase::Manipulator::GetGripperDOFValues(std::vector<dReal>& v) const
{
    GetRobot()->GetDOFValues(v, gripper_dof_indices_);
}


bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, vector<dReal>& solution, int filteroptions) const
{
    return FindIKSolution(goal, vector<dReal>(), solution, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, vector<dReal>& solution, int filteroptions) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(robot_)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    solution.resize(arm_dof_indices_.size());
    for(size_t i = 0; i < arm_dof_indices_.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(arm_dof_indices_[i]);
        solution[i] = pjoint->GetValue(arm_dof_indices_[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!base_link_ ) {
        localgoal = base_link_->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    std::shared_ptr< vector<dReal> > psolution(&solution, utils::null_deleter());
    return vFreeParameters.size() == 0 ? pIkSolver->Solve(localgoal, solution, filteroptions, psolution) : pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, psolution);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, std::vector<std::vector<dReal> >& solutions, int filteroptions) const
{
    return FindIKSolutions(goal, vector<dReal>(), solutions, filteroptions);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal,
	const std::vector<dReal>& free_parameters, 
	std::vector<std::vector<dReal> >& solutions, int filter_options) const
{
    IkSolverBasePtr ik_solver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!ik_solver, "manipulator %s:%s does not have an IK solver set",
		RobotBasePtr(robot_)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(ik_solver->GetManipulator() == shared_from_this() );
    IkParameterization local_goal;
    if( !!base_link_ )
	{
        local_goal = base_link_->GetTransform().inverse()*goal;
    }
    else 
	{
        local_goal=goal;
    }
    return free_parameters.size() == 0
		? ik_solver->SolveAll(local_goal,filter_options,solutions) 
		: ik_solver->SolveAll(local_goal,free_parameters,filter_options,solutions);
}


bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, int filteroptions, IkReturnPtr ikreturn) const
{
    return FindIKSolution(goal, vector<dReal>(), filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolution(const IkParameterization& goal, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn) const
{
    IkSolverBasePtr pIkSolver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",RobotBasePtr(robot_)->GetName()%GetName(),ORE_Failed);
    RobotBasePtr probot = GetRobot();
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    vector<dReal> solution(arm_dof_indices_.size());
    for(size_t i = 0; i < arm_dof_indices_.size(); ++i) {
        JointConstPtr pjoint = probot->GetJointFromDOFIndex(arm_dof_indices_[i]);
        solution[i] = pjoint->GetValue(arm_dof_indices_[i]-pjoint->GetDOFIndex());
    }
    IkParameterization localgoal;
    if( !!base_link_ ) {
        localgoal = base_link_->GetTransform().inverse()*goal;
    }
    else {
        localgoal=goal;
    }
    return vFreeParameters.size() == 0 ? pIkSolver->Solve(localgoal, solution, filteroptions, ikreturn) : pIkSolver->Solve(localgoal, solution, vFreeParameters, filteroptions, ikreturn);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const
{
    return FindIKSolutions(goal, vector<dReal>(), filteroptions, vikreturns);
}

bool RobotBase::Manipulator::FindIKSolutions(const IkParameterization& goal,
	const std::vector<dReal>& free_parameters, int filter_options, std::vector<IkReturnPtr>& ik_returns) const
{
    IkSolverBasePtr ik_solver = GetIkSolver();
    OPENRAVE_ASSERT_FORMAT(!!ik_solver, "manipulator %s:%s does not have an IK solver set",
		RobotBasePtr(robot_)->GetName()%GetName(),ORE_Failed);
    BOOST_ASSERT(ik_solver->GetManipulator() == shared_from_this() );
    IkParameterization local_goal;
    if( !!base_link_ ) 
	{
        local_goal = base_link_->GetTransform().inverse()*goal;
    }
    else 
	{
        local_goal=goal;
    }
    return free_parameters.size() == 0 
		? ik_solver->SolveAll(local_goal,filter_options,ik_returns)
		: ik_solver->SolveAll(local_goal,free_parameters,filter_options,ik_returns);
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(IkParameterizationType iktype, bool inworld) const
{
    IkParameterization ikp;
    Transform t = GetTransform();
    if( !inworld ) 
	{
        t = GetBase()->GetTransform().inverse()*t;
    }
    switch(iktype) 
	{
    case IKP_Transform6D: ikp.SetTransform6D(t); break;
    case IKP_Rotation3D: ikp.SetRotation3D(t.rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(t.trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(info_.direction_)); break;
    case IKP_Ray4D: 
	{
        ikp.SetRay4D(RAY(t.trans,t.rotate(info_.direction_)));
        break;
    }
    case IKP_Lookat3D: 
	{
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization:\
                      Lookat3D type setting goal a distance of 1 from the origin.\n");
        Vector vdir = t.rotate(info_.direction_);
        ikp.SetLookat3D(RAY(t.trans + vdir,vdir));
        break;
    }
    case IKP_TranslationDirection5D: 
	{
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(info_.direction_)));
        break;
    }
    case IKP_TranslationXY2D: 
	{
        ikp.SetTranslationXY2D(t.trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: 
	{
        //dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        Vector vglobaldirection = t.rotate(info_.direction_);
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,
			RaveAtan2(vglobaldirection.y,vglobaldirection.x)));
        break;
    }
    case IKP_TranslationLocalGlobal6D:
	{
        RAVELOG_WARN("RobotBase::Manipulator::GetIkParameterization: TranslationLocalGlobal6D type setting local translation to (0,0,0).\n");
        ikp.SetTranslationLocalGlobal6D(Vector(0,0,0),t.trans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        break;
    }
    // velocities
    case IKP_Transform6DVelocity: {
        Vector vlinearvel, vangularvel;
        GetEndEffector()->GetVelocity(vlinearvel, vangularvel);

        Transform tee = GetTransform();
        //effector_link_->GetTransform() * info_.local_tool_transform_
        // TODO, have to convert relative to the end effector's velocity
        vlinearvel += vangularvel.cross(tee.trans - GetEndEffector()->GetTransform().trans); // t.trans = GetTransform().trans - tbase.trans

        if( !inworld ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            //Transform tbase = GetBase()->GetTransform();
            // TODO, have to convert relative to the base's velocity!
            vlinearvel -= vangularvel.cross(t.trans); // t.trans = GetTransform().trans - tbase.trans
            //Vector vbaselinear, vbaseangular;
            //GetBase()->GetVelocity(vlinearvel, vangularvel);
            //Vector voffset = t.trans - tbase.trans;
            //vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
            //vLinkVelocities[2][i].second = vbaseangular;
        }
        Transform tvel;
        tvel.trans = vlinearvel;
        tvel.rot = quatMultiply(t.rot,Vector(0,vangularvel.x,vangularvel.y,vangularvel.z)) * 0.5;
        ikp.SetTransform6DVelocity(tvel);
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("invalid ik type 0x%x"),iktype,ORE_InvalidArguments);
    }
    return ikp;
}

IkParameterization RobotBase::Manipulator::GetIkParameterization(const IkParameterization& ikparam, bool inworld) const
{
    IkParameterization ikp = ikparam; // copies the custom data
    Transform t = GetTransform();
    if( !inworld ) {
        t = GetBase()->GetTransform().inverse()*t;
    }
    switch(ikparam.GetType()) {
    case IKP_Transform6D: ikp.SetTransform6D(t); break;
    case IKP_Rotation3D: ikp.SetRotation3D(t.rot); break;
    case IKP_Translation3D: ikp.SetTranslation3D(t.trans); break;
    case IKP_Direction3D: ikp.SetDirection3D(t.rotate(info_.direction_)); break;
    case IKP_Ray4D: {
        ikp.SetRay4D(RAY(t.trans,t.rotate(info_.direction_)));
        break;
    }
    case IKP_Lookat3D: {
        // find the closest point to ikparam.GetLookat3D() to the current ray
        Vector vdir = t.rotate(info_.direction_);
        ikp.SetLookat3D(RAY(t.trans + vdir*vdir.dot(ikparam.GetLookat3D()-t.trans),vdir));
        break;
    }
    case IKP_TranslationDirection5D: {
        ikp.SetTranslationDirection5D(RAY(t.trans,t.rotate(info_.direction_)));
        break;
    }
    case IKP_TranslationXY2D: {
        ikp.SetTranslationXY2D(t.trans);
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        Vector vglobaldirection = t.rotate(info_.direction_);
        ikp.SetTranslationXYOrientation3D(Vector(t.trans.x,t.trans.y,RaveAtan2(vglobaldirection.y,vglobaldirection.x)));
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        Vector localtrans = ikparam.GetTranslationLocalGlobal6D().first;
        ikp.SetTranslationLocalGlobal6D(localtrans,t * localtrans);
        break;
    }
    case IKP_TranslationXAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.x,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.y,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        else {
            const Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngle4D(t.trans,RaveAcos(OpenRAVE::utils::ClampOnRange(vlocaldirection.z,-1.0,1.0)));
        }
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationXAxisAngleZNorm4D(t.trans,RaveAtan2(vlocaldirection.y,vlocaldirection.x));
        }
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationYAxisAngleXNorm4D(t.trans,RaveAtan2(vlocaldirection.z,vlocaldirection.y));
        }
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        if (inworld) {
            Transform localt = GetBase()->GetTransform().inverse()*t;
            const Vector vlocaldirection = localt.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        else {
            Vector vlocaldirection = t.rotate(info_.direction_);
            ikp.SetTranslationZAxisAngleYNorm4D(t.trans,RaveAtan2(vlocaldirection.x,vlocaldirection.z));
        }
        break;
    }
    // velocities
    case IKP_Transform6DVelocity: {
        Vector vlinearvel, vangularvel;
        GetEndEffector()->GetVelocity(vlinearvel, vangularvel);

        Transform tee = GetTransform();
        //effector_link_->GetTransform() * info_.local_tool_transform_
        // TODO, have to convert relative to the end effector's velocity
        vlinearvel += vangularvel.cross(tee.trans - GetEndEffector()->GetTransform().trans); // t.trans = GetTransform().trans - tbase.trans

        if( !inworld ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            //Transform tbase = GetBase()->GetTransform();
            // TODO, have to convert relative to the base's velocity!
            vlinearvel -= vangularvel.cross(t.trans); // t.trans = GetTransform().trans - tbase.trans
            //Vector vbaselinear, vbaseangular;
            //GetBase()->GetVelocity(vlinearvel, vangularvel);
            //Vector voffset = t.trans - tbase.trans;
            //vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
            //vLinkVelocities[2][i].second = vbaseangular;
        }
        Transform tvel;
        tvel.trans = vlinearvel;
        tvel.rot = quatMultiply(t.rot,Vector(0,vangularvel.x,vangularvel.y,vangularvel.z)) * 0.5;
        ikp.SetTransform6DVelocity(tvel);
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("invalid ik type 0x%x"),ikparam.GetType(),ORE_InvalidArguments);
    }
    return ikp;
}

void RobotBase::Manipulator::GetChildJoints(std::vector<JointPtr>& joints) const
{
    RobotBasePtr probot(robot_);
    joints.resize(0);
    int iattlink = effector_link_->GetIndex();
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmjoint)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        FOREACHC(itjoint, probot->GetJoints()) {
            if( !(*itjoint)->IsStatic() && !vhasjoint[(*itjoint)->GetJointIndex()] && probot->DoesAffect((*itjoint)->GetJointIndex(),ilink) && !probot->DoesAffect((*itjoint)->GetJointIndex(),iattlink) ) {
                joints.push_back(*itjoint);
                vhasjoint[(*itjoint)->GetJointIndex()] = true;
            }
        }
    }
}

void RobotBase::Manipulator::GetChildDOFIndices(std::vector<int>& vdofindices) const
{
    vdofindices.resize(0);
    RobotBasePtr probot(robot_);
    int iattlink = effector_link_->GetIndex();
    vector<uint8_t> vhasjoint(probot->GetJoints().size(),false);
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmjoint,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmjoint)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        FOREACHC(itjoint, probot->GetJoints()) {
            if( !(*itjoint)->IsStatic() && !vhasjoint[(*itjoint)->GetJointIndex()] && probot->DoesAffect((*itjoint)->GetJointIndex(),ilink) && !probot->DoesAffect((*itjoint)->GetJointIndex(),iattlink) ) {
                vhasjoint[(*itjoint)->GetJointIndex()] = true;
                int idofbase = (*itjoint)->GetDOFIndex();
                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    vdofindices.push_back(idofbase+idof);
                }
            }
        }
    }
}

void RobotBase::Manipulator::GetChildLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(robot_);
    // get all child links of the manipualtor
    vlinks.clear();
    effector_link_->GetRigidlyAttachedLinks(vlinks);
    int iattlink = effector_link_->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        // there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
        if( find(vlinks.begin(), vlinks.end(), *itlink) == vlinks.end() ) { // if joint is static but still in the dof indices, it could already be rigidly attached to the end effector link and therefore inside vlinks
            vlinks.push_back(*itlink);
        }
    }
}

bool RobotBase::Manipulator::IsChildLink(const KinBody::Link &link) const
{
    if( effector_link_->IsRigidlyAttached(link) ) {
        return true;
    }

    RobotBasePtr probot(robot_);
    // get all child links of the manipualtor
    int iattlink = effector_link_->GetIndex();
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if( ilink == iattlink ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                return true;
            }
        }
    }
    return false;
}

void RobotBase::Manipulator::GetIndependentLinks(std::vector<LinkPtr>& vlinks) const
{
    RobotBasePtr probot(robot_);
    vlinks.clear();
    FOREACHC(itlink, probot->GetLinks()) {
        bool bAffected = false;
        FOREACHC(itindex,arm_dof_indices_) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        FOREACHC(itindex,gripper_dof_indices_) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }

        if( !bAffected ) {
            vlinks.push_back(*itlink);
        }
    }
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(CollisionReportPtr report) const
{
    RobotBasePtr probot(robot_);
    // get all child links of the manipualtor
    int iattlink = effector_link_->GetIndex();
    vector<LinkPtr> vattachedlinks;
    effector_link_->GetRigidlyAttachedLinks(vattachedlinks);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    FOREACHC(itlink,vattachedlinks) {
        if( probot->CheckLinkCollision((*itlink)->GetIndex(), report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        bool bIsAffected = false;
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                bIsAffected = true;
                if( probot->CheckLinkCollision(ilink, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
                break;
            }
        }

        if( !bIsAffected ) {
            // link is not affected by any of the joints, perhaps there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
            if( probot->CheckLinkCollision(ilink, report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    return true;
                }
                bincollision = true;
            }
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report) const
{
    RobotBasePtr probot(robot_);
    Transform toldEE = GetTransform();
    Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor
    int iattlink = effector_link_->GetIndex();
    vector<LinkPtr> vattachedlinks;
    effector_link_->GetRigidlyAttachedLinks(vattachedlinks);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    FOREACHC(itlink,vattachedlinks) {
        if( probot->CheckLinkCollision((*itlink)->GetIndex(),tdelta*(*itlink)->GetTransform(),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        bool bIsAffected = false;
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                bIsAffected = true;
                if( probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
                break;
            }
        }

        if( !bIsAffected ) {
            // link is not affected by any of the joints, perhaps there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
            if( probot->CheckLinkCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    return true;
                }
                bincollision = true;
            }
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(CollisionReportPtr report, bool bIgnoreManipulatorLinks) const
{
    RobotBasePtr probot(robot_);
    // get all child links of the manipualtor
    int iattlink = effector_link_->GetIndex();
    vector<LinkPtr> vattachedlinks;
    effector_link_->GetRigidlyAttachedLinks(vattachedlinks);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    if( bIgnoreManipulatorLinks ) {
        CollisionCheckerBasePtr pselfchecker = !!probot->GetSelfCollisionChecker() ? probot->GetSelfCollisionChecker() : probot->GetEnv()->GetCollisionChecker();
        std::vector<LinkPtr> vindependentinks;
        GetIndependentLinks(vindependentinks);
        FOREACHC(itlink,vattachedlinks) {
            KinBody::LinkPtr plink = *itlink;
            if( plink->IsEnabled() ) {
                std::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
                FOREACHC(itindependentlink,vindependentinks) {
                    if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                        if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                RAVELOG_VERBOSE_FORMAT("link self collision with link %s", (*itlink)->GetName());
                                return true;
                            }
                            bincollision = true;
                        }
                    }
                }
            }
        }
    }
    else {
        // unfortunately we cannot check for self collisions of links since after IK they could be different for links that move because of the manipulator!
        FOREACHC(itlink,vattachedlinks) {
            if( probot->CheckLinkSelfCollision((*itlink)->GetIndex(),report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    RAVELOG_VERBOSE_FORMAT("link self collision with link %s", (*itlink)->GetName());
                    return true;
                }
                bincollision = true;
            }
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        bool bIsAffected = false;
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                bIsAffected = true;
                if( probot->CheckLinkSelfCollision(ilink,report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
                break;
            }
        }

        if( !bIsAffected ) {
            // link is not affected by any of the joints, perhaps there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
            if( probot->CheckLinkSelfCollision(ilink,report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    return true;
                }
                bincollision = true;
            }
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const Transform& tEE, CollisionReportPtr report, bool bIgnoreManipulatorLinks) const
{
    RobotBasePtr probot(robot_);
    Transform toldEE = GetTransform();
    Transform tdelta = tEE*toldEE.inverse();
    // get all child links of the manipualtor
    int iattlink = effector_link_->GetIndex();
    vector<LinkPtr> vattachedlinks;
    effector_link_->GetRigidlyAttachedLinks(vattachedlinks);

    CollisionCheckerBasePtr pchecker = probot->GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;

    // parameters used only when bIgnoreManipulatorLinks is true
    CollisionCheckerBasePtr pselfchecker;
    std::vector<LinkPtr> vindependentinks;
    if( bIgnoreManipulatorLinks ) {
        GetIndependentLinks(vindependentinks);
        pselfchecker = !!probot->GetSelfCollisionChecker() ? probot->GetSelfCollisionChecker() : probot->GetEnv()->GetCollisionChecker();
    }

    if( bIgnoreManipulatorLinks ) {
        GetIndependentLinks(vindependentinks);
        FOREACHC(itlink,vattachedlinks) {
            KinBody::LinkPtr plink = *itlink;
            if( plink->IsEnabled() ) {
                std::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
                Transform tlinktrans = tdelta*plink->GetTransform();
                plink->SetTransform(tlinktrans);

                FOREACHC(itindependentlink,vindependentinks) {
                    if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                        if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                RAVELOG_VERBOSE_FORMAT("link self collision with link %s", (*itlink)->GetName());
                                return true;
                            }
                            bincollision = true;
                        }
                    }
                }
            }
        }
    }
    else {
        // unfortunately we cannot check for self collisions of links since after IK they could be different for links that move because of the manipulator!
        FOREACHC(itlink,vattachedlinks) {
            if( probot->CheckLinkSelfCollision((*itlink)->GetIndex(),tdelta*(*itlink)->GetTransform(),report) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    RAVELOG_VERBOSE_FORMAT("link self collision with link %s", (*itlink)->GetName());
                    return true;
                }
                bincollision = true;
            }
        }
    }
    FOREACHC(itlink, probot->GetLinks()) {
        int ilink = (*itlink)->GetIndex();
        if((ilink == iattlink)|| !(*itlink)->IsEnabled() ) {
            continue;
        }
        // gripper needs to be affected by all joints
        bool bGripperLink = true;
        FOREACHC(itarmdof,arm_dof_indices_) {
            if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                bGripperLink = false;
                break;
            }
        }
        if( !bGripperLink ) {
            continue;
        }

        bool bIsAffected = false;
        for(size_t ijoint = 0; ijoint < probot->GetJoints().size(); ++ijoint) {
            if( probot->DoesAffect(ijoint,ilink) && !probot->DoesAffect(ijoint,iattlink) ) {
                bIsAffected = true;
                if( bIgnoreManipulatorLinks ) {
                    std::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(*itlink)); // gcc optimization bug when linksaver is on stack?
                    Transform tlinktrans = tdelta*(*itlink)->GetTransform();
                    (*itlink)->SetTransform(tlinktrans);

                    FOREACHC(itindependentlink,vindependentinks) {
                        if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                            if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                    RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                                    return true;
                                }
                                bincollision = true;
                            }
                        }
                    }
                }
                else {
                    if( probot->CheckLinkSelfCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                            return true;
                        }
                        bincollision = true;
                    }
                }
                break;
            }
        }

        if( !bIsAffected ) {
            // link is not affected by any of the joints, perhaps there could be passive joints that are attached to the end effector that are non-static. if a link is affected by all the joints in the chain, then it is most likely a child just by the fact that all the arm joints affect it.
            if( bIgnoreManipulatorLinks ) {
                std::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(*itlink)); // gcc optimization bug when linksaver is on stack?
                Transform tlinktrans = tdelta*(*itlink)->GetTransform();
                (*itlink)->SetTransform(tlinktrans);

                FOREACHC(itindependentlink,vindependentinks) {
                    if( *itlink != *itindependentlink && (*itindependentlink)->IsEnabled() ) {
                        if( pselfchecker->CheckCollision(*itlink, *itindependentlink,report) ) {
                            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                RAVELOG_VERBOSE_FORMAT("gripper link self collision with link %s", (*itlink)->GetName());
                                return true;
                            }
                            bincollision = true;
                        }
                    }
                }
            }
            else {
                if( probot->CheckLinkSelfCollision(ilink,tdelta*(*itlink)->GetTransform(),report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
            break;
        }
    }
    return bincollision;
}

bool RobotBase::Manipulator::CheckEndEffectorCollision(const IkParameterization& ikparam,
	CollisionReportPtr report, int numredundantsamples) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorCollision(ikparam.GetTransform6D(),report);
    }
    RobotBasePtr probot = GetRobot();
    if( numredundantsamples > 0 ) {
        if( ikparam.GetType() == IKP_TranslationDirection5D ) {
            Transform tStartEE;
            tStartEE.rot = quatRotateDirection(info_.direction_, ikparam.GetTranslationDirection5D().dir);
            tStartEE.trans = ikparam.GetTranslationDirection5D().pos;
            Vector qdelta = quatFromAxisAngle(info_.direction_, 2*M_PI/dReal(numredundantsamples));
            bool bNotInCollision = false;
            for(int i = 0; i < numredundantsamples; ++i) {
                if( !CheckEndEffectorCollision(tStartEE,report) ) {
                    // doesn't collide, but will need to verify that there actually exists an IK solution there...
                    // if we accidentally return here even there's no IK solution, then later processes could waste a lot of time looking for it.
                    bNotInCollision = true;
                    break;
                }
                tStartEE.rot = quatMultiply(tStartEE.rot, qdelta);
            }
            if( !bNotInCollision ) {
                return true;
            }
        }
        else {
            RAVELOG_WARN_FORMAT("do not support redundant checking for iktype 0x%x", ikparam.GetType());
        }
    }

    IkSolverBasePtr pIkSolver = GetIkSolver();
    //OPENRAVE_ASSERT_OP_FORMAT(GetArmDOF(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",probot->GetName()%GetName(),ORE_Failed);
    OPENRAVE_ASSERT_FORMAT(pIkSolver->Supports(ikparam.GetType()),"manipulator %s:%s ik solver %s does not support ik type 0x%x",probot->GetName()%GetName()%pIkSolver->GetXMLId()%ikparam.GetType(),ORE_InvalidState);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!base_link_ ) {
        localgoal = base_link_->GetTransform().inverse()*ikparam;
    }
    else {
        localgoal=ikparam;
    }

    // if IK can be solved, then there exists a solution for the end effector that is not in collision
    IkReturn ikreturn(IKRA_Success);
    IkReturnPtr pikreturn(&ikreturn,OpenRAVE::utils::null_deleter());

    // need to use free params here since sometimes IK can have 3+ free DOF and it would freeze searching for all of them
    std::vector<dReal> vFreeParameters;
    pIkSolver->GetFreeParameters(vFreeParameters);
    if( pIkSolver->Solve(localgoal, std::vector<dReal>(), vFreeParameters, IKFO_CheckEnvCollisions|IKFO_IgnoreCustomFilters, pikreturn) ) { // TODO need to specify IKFO_IgnoreSelfCollisions?
        return false;
    }
    else {
        if( (ikreturn.action_&IKRA_RejectSelfCollision) != IKRA_RejectSelfCollision && (ikreturn.action_&IKRA_RejectEnvCollision) != IKRA_RejectEnvCollision ) {
            RAVELOG_VERBOSE_FORMAT("ik solution not found due to non-collision reasons (0x%x), returning true anway...", ikreturn.action_);
            // is this a good idea?
        }
        if( !!report ) {
            // solver failed, should have some way of initializing the report...
            if( numredundantsamples > 0 ) {
                if( ikparam.GetType() == IKP_TranslationDirection5D || ikparam.GetType() == IKP_TranslationXAxisAngleZNorm4D || ikparam.GetType() == IKP_TranslationYAxisAngleXNorm4D || ikparam.GetType() == IKP_TranslationZAxisAngleYNorm4D ) {
                    // if here, then already determined that there is a roll that is collision free, so return False
                    return false;
                }
            }
        }
        return true;
    }

//    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
//    // is it necessary to call with IKFO_IgnoreJointLimits knowing that the robot will never reach those solutions?
//    std::vector< std::vector<dReal> > vsolutions;
//    if( !pIkSolver->SolveAll(localgoal, IKFO_IgnoreSelfCollisions,vsolutions) ) {
//        throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to find ik solution for type 0x%x"),ikparam.GetType(),ORE_InvalidArguments);
//    }
//    RobotStateSaver saver(probot);
//    probot->SetActiveDOFs(GetArmIndices());
//    // have to check all solutions since the 6D transform can change even though the ik parameterization doesn't
//    std::list<Transform> listprevtransforms;
//    FOREACH(itsolution,vsolutions) {
//        probot->SetActiveDOFValues(*itsolution,false);
//        Transform t = GetTransform();
//        // check if previous transforms exist
//        bool bhassimilar = false;
//        FOREACH(ittrans,listprevtransforms) {
//            if( TransformDistanceFast(t,*ittrans) < g_fEpsilonLinear*10 ) {
//                bhassimilar = true;
//                break;
//            }
//        }
//        if( !bhassimilar ) {
//            if( !CheckEndEffectorCollision(GetTransform(),report) ) {
//                return false;
//            }
//            listprevtransforms.push_back(t);
//        }
//    }
//
//    return true;
}

bool RobotBase::Manipulator::CheckEndEffectorSelfCollision(const IkParameterization& ikparam, CollisionReportPtr report, int numredundantsamples, bool bIgnoreManipulatorLinks) const
{
    if( ikparam.GetType() == IKP_Transform6D ) {
        return CheckEndEffectorSelfCollision(ikparam.GetTransform6D(),report,bIgnoreManipulatorLinks);
    }
    RobotBasePtr probot = GetRobot();
    if( numredundantsamples > 0 ) {
        if( ikparam.GetType() == IKP_TranslationDirection5D ) {
            Transform tStartEE;
            tStartEE.rot = quatRotateDirection(info_.direction_, ikparam.GetTranslationDirection5D().dir);
            tStartEE.trans = ikparam.GetTranslationDirection5D().pos;
            Vector qdelta = quatFromAxisAngle(info_.direction_, 2*M_PI/dReal(numredundantsamples));
            bool bNotInCollision = false;
            for(int i = 0; i < numredundantsamples; ++i) {
                if( !CheckEndEffectorSelfCollision(tStartEE,report, bIgnoreManipulatorLinks) ) {
                    // doesn't collide, but will need to verify that there actually exists an IK solution there...
                    // if we accidentally return here even there's no IK solution, then later processes could waste a lot of time looking for it.
                    bNotInCollision = true;
                    break;
                }
                tStartEE.rot = quatMultiply(tStartEE.rot, qdelta);
            }
            if( !bNotInCollision ) {
                return true;
            }
        }
        else {
            RAVELOG_WARN_FORMAT("do not support redundant checking for iktype 0x%x", ikparam.GetType());
        }
    }

    IkSolverBasePtr pIkSolver = GetIkSolver();
    //OPENRAVE_ASSERT_OP_FORMAT(GetArmDOF(), <=, ikparam.GetDOF(), "ikparam type 0x%x does not fully determine manipulator %s:%s end effector configuration", ikparam.GetType()%probot->GetName()%GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pIkSolver, "manipulator %s:%s does not have an IK solver set",probot->GetName()%GetName(),ORE_Failed);
    OPENRAVE_ASSERT_FORMAT(pIkSolver->Supports(ikparam.GetType()),"manipulator %s:%s ik solver %s does not support ik type 0x%x",probot->GetName()%GetName()%pIkSolver->GetXMLId()%ikparam.GetType(),ORE_InvalidState);
    BOOST_ASSERT(pIkSolver->GetManipulator() == shared_from_this() );
    IkParameterization localgoal;
    if( !!base_link_ ) {
        localgoal = base_link_->GetTransform().inverse()*ikparam;
    }
    else {
        localgoal=ikparam;
    }

    // if IK can be solved, then there exists a solution for the end effector that is not in collision
    IkReturn ikreturn(IKRA_Success);
    IkReturnPtr pikreturn(&ikreturn,OpenRAVE::utils::null_deleter());

    // need to use free params here since sometimes IK can have 3+ free DOF and it would freeze searching for all of them
    std::vector<dReal> vFreeParameters;
    pIkSolver->GetFreeParameters(vFreeParameters);
    if( pIkSolver->Solve(localgoal, std::vector<dReal>(), vFreeParameters, IKFO_IgnoreCustomFilters, pikreturn) ) {
        return false;
    }
    else {
        if( (ikreturn.action_&IKRA_RejectSelfCollision) != IKRA_RejectSelfCollision && (ikreturn.action_&IKRA_RejectEnvCollision) != IKRA_RejectEnvCollision ) {
            RAVELOG_VERBOSE_FORMAT("ik solution not found due to non-collision reasons (0x%x), returning true anway...", ikreturn.action_);
            // is this a good idea?
        }
        if( !!report ) {
            // solver failed, should have some way of initializing the report...
            if( numredundantsamples > 0 ) {
                if( ikparam.GetType() == IKP_TranslationDirection5D || ikparam.GetType() == IKP_TranslationXAxisAngleZNorm4D || ikparam.GetType() == IKP_TranslationYAxisAngleXNorm4D || ikparam.GetType() == IKP_TranslationZAxisAngleYNorm4D ) {
                    // if here, then already determined that there is a roll that is collision free, so return False
                    return false;
                }
            }
        }
        return true;
    }

//    // only care about the end effector position, so disable all time consuming options. still leave the custom options in case the user wants to call some custom stuff?
//    // is it necessary to call with IKFO_IgnoreJointLimits knowing that the robot will never reach those solutions?
//    std::vector< std::vector<dReal> > vsolutions;
//    if( !ik_solver->SolveAll(local_goal, vector<dReal>(), IKFO_IgnoreSelfCollisions,vsolutions) ) {
//        throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to find ik solution for type 0x%x"),ikparam.GetType(),ORE_InvalidArguments);
//    }
//    RobotStateSaver saver(probot);
//    probot->SetActiveDOFs(GetArmIndices());
//    // have to check all solutions since the 6D transform can change even though the ik parameterization doesn't
//    std::list<Transform> listprevtransforms;
//    FOREACH(itsolution,vsolutions) {
//        probot->SetActiveDOFValues(*itsolution,false);
//        Transform t = GetTransform();
//        // check if previous transforms exist
//        bool bhassimilar = false;
//        FOREACH(ittrans,listprevtransforms) {
//            if( TransformDistanceFast(t,*ittrans) < g_fEpsilonLinear*10 ) {
//                bhassimilar = true;
//                break;
//            }
//        }
//        if( !bhassimilar ) {
//            if( CheckEndEffectorSelfCollision(GetTransform(),report) ) {
//                return true;
//            }
//            listprevtransforms.push_back(t);
//        }
//    }
//
//    return false;
}

bool RobotBase::Manipulator::CheckIndependentCollision(CollisionReportPtr report) const
{
    RobotBasePtr probot(robot_);
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;

    FOREACHC(itlink, probot->GetLinks()) {
        if( !(*itlink)->IsEnabled() ) {
            continue;
        }
        bool bAffected = false;
        FOREACHC(itindex,arm_dof_indices_) {
            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                bAffected = true;
                break;
            }
        }
        if( !bAffected ) {
            FOREACHC(itindex,gripper_dof_indices_) {
                if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                    bAffected = true;
                    break;
                }
            }
        }

        if( !bAffected ) {
            if( probot->GetEnv()->GetCollisionChecker()->GetCollisionOptions() & CO_ActiveDOFs ) {
                // collision checker has the CO_ActiveDOFs option set, so check if link is active
                if( !probot->GetAffineDOF() ) {
                    bool bActive = false;
                    FOREACHC(itdofindex, probot->GetActiveDOFIndices()) {
                        if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itdofindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                            bActive = true;
                            break;
                        }
                    }
                    if( !bActive ) {
                        continue;
                    }
                }
            }
            if( probot->GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink),report) ) {
                return true;
            }

            // check if any grabbed bodies are attached to this link
            FOREACHC(itgrabbed,probot->grabbed_bodies_) {
                GrabbedConstPtr pgrabbed = std::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
                if( pgrabbed->link_robot_ == *itlink ) {
                    if( vbodyexcluded.empty() ) {
                        vbodyexcluded.push_back(KinBodyConstPtr(probot));
                    }
                    KinBodyPtr pbody = pgrabbed->grabbed_body_.lock();
                    if( !!pbody && probot->GetEnv()->CheckCollision(KinBodyConstPtr(pbody),vbodyexcluded, vlinkexcluded, report) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool RobotBase::Manipulator::IsGrabbing(const KinBody &body) const
{
    RobotBasePtr probot(robot_);
    KinBody::LinkPtr plink = probot->IsGrabbing(body);
    if( !!plink ) {
        if( plink == effector_link_ || plink == base_link_ ) {
            return true;
        }
        int iattlink = effector_link_->GetIndex();
        FOREACHC(itlink, probot->GetLinks()) {
            int ilink = (*itlink)->GetIndex();
            if( ilink == iattlink ) {
                continue;
            }
            // gripper needs to be affected by all joints
            bool bGripperLink = true;
            FOREACHC(itarmdof,arm_dof_indices_) {
                if( !probot->DoesAffect(probot->GetJointFromDOFIndex(*itarmdof)->GetJointIndex(),ilink) ) {
                    bGripperLink = false;
                    break;
                }
            }
            if( bGripperLink &&(plink == *itlink)) {
                return true;
            }
        }
    }
    return false;
}

void RobotBase::Manipulator::CalculateJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(robot_);
    probot->ComputeJacobianTranslation(effector_link_->GetIndex(), effector_link_->GetTransform() * info_.local_tool_transform_.trans, jacobian, arm_dof_indices_);
}

void RobotBase::Manipulator::CalculateJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][arm_dof_indices_.size()]);
    if( arm_dof_indices_.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(robot_);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianTranslation(effector_link_->GetIndex(), effector_link_->GetTransform() * info_.local_tool_transform_.trans, vjacobian, arm_dof_indices_);
    OPENRAVE_ASSERT_OP(vjacobian.size(),==,3*arm_dof_indices_.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+arm_dof_indices_.size(),itdst->begin());
        itsrc += arm_dof_indices_.size();
    }
}

void RobotBase::Manipulator::CalculateRotationJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(robot_);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(arm_dof_indices_);
    probot->CalculateActiveRotationJacobian(effector_link_->GetIndex(),quatMultiply(effector_link_->GetTransform().rot, info_.local_tool_transform_.rot),jacobian);
}

void RobotBase::Manipulator::CalculateRotationJacobian(boost::multi_array<dReal,2>& jacobian) const
{
    RobotBasePtr probot(robot_);
    RobotBase::RobotStateSaver saver(probot,RobotBase::Save_ActiveDOF);
    probot->SetActiveDOFs(arm_dof_indices_);
    probot->CalculateActiveRotationJacobian(effector_link_->GetIndex(),quatMultiply(effector_link_->GetTransform().rot, info_.local_tool_transform_.rot),jacobian);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(std::vector<dReal>& jacobian) const
{
    RobotBasePtr probot(robot_);
    probot->ComputeJacobianAxisAngle(effector_link_->GetIndex(), jacobian, arm_dof_indices_);
}

void RobotBase::Manipulator::CalculateAngularVelocityJacobian(boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][arm_dof_indices_.size()]);
    if( arm_dof_indices_.size() == 0 ) {
        return;
    }
    RobotBasePtr probot(robot_);
    std::vector<dReal> vjacobian;
    probot->ComputeJacobianAxisAngle(effector_link_->GetIndex(), vjacobian, arm_dof_indices_);
    OPENRAVE_ASSERT_OP(vjacobian.size(),==,3*arm_dof_indices_.size());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+arm_dof_indices_.size(),itdst->begin());
        itsrc += arm_dof_indices_.size();
    }
}

void RobotBase::Manipulator::serialize(std::ostream& o, int options, IkParameterizationType iktype) const
{
    if( options & SO_RobotManipulators ) {
        o << (!base_link_ ? -1 : base_link_->GetIndex()) << " " << (!effector_link_ ? -1 : effector_link_->GetIndex()) << " ";
        // don't include arm_dof_indices_ and gripper_dof_indices_ since they are generated from the data
        o << info_.gripper_joint_names_vector_.size() << " ";
        FOREACHC(it,info_.gripper_joint_names_vector_) {
            o << *it << " ";
        }
        FOREACHC(it,info_.chucking_direction_vector_) {
            SerializeRound(o,*it);
        }
        SerializeRound(o,info_.local_tool_transform_);
    }
    if( options & (SO_Kinematics|SO_InverseKinematics) ) {
        RobotBasePtr probot(robot_);
        Transform tcur;
        std::vector<JointPtr> vjoints;
        if( probot->GetChain(base_link_->GetIndex(),effector_link_->GetIndex(), vjoints) ) {
            // due to back compat issues, have to compute the end effector transform first
            FOREACH(itjoint, vjoints) {
                tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform() * (*itjoint)->GetInternalHierarchyRightTransform();
            }
            tcur = tcur;
            // if 6D transform IK, then don't inlucde the local tool transform!
            if( !!(options & SO_Kinematics) || iktype != IKP_Transform6D ) {
                tcur *= info_.local_tool_transform_;
            }
            SerializeRound(o,tcur);
            o << arm_dof_indices_.size() << " ";

            tcur = Transform();
            int index = 0;
            FOREACH(itjoint, vjoints) {
                if( !(*itjoint)->IsStatic() ) {
                    o << (*itjoint)->GetType() << " ";
                }

                if( (*itjoint)->GetDOFIndex() >= 0 && find(arm_dof_indices_.begin(),arm_dof_indices_.end(),(*itjoint)->GetDOFIndex()) != arm_dof_indices_.end() ) {
                    tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform();
                    for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                        SerializeRound3(o,tcur.trans);
                        SerializeRound3(o,tcur.rotate((*itjoint)->GetInternalHierarchyAxis(idof)));
                    }
                    tcur = tcur * (*itjoint)->GetInternalHierarchyRightTransform();
                }
                else {
                    // not sure if this is correct for a mimic joint...
                    tcur = tcur * (*itjoint)->GetInternalHierarchyLeftTransform() * (*itjoint)->GetInternalHierarchyRightTransform();
                    if( (*itjoint)->IsMimic() ) {
                        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                            if( (*itjoint)->IsMimic(idof) ) {
                                o << "mimic " << index << " ";
                                for(int ieq = 0; ieq < 3; ++ieq) {
                                    o << (*itjoint)->GetMimicEquation(idof,ieq) << " ";
                                }
                            }
                        }
                    }
                }
                index += 1;
            }
        }
    }

    // output direction if SO_Kinematics|SO_RobotManipulators. if IK hash, then output only on certain ik types.
    if( !!(options & (SO_Kinematics|SO_RobotManipulators)) || (!!(options&SO_InverseKinematics) && (iktype == IKP_TranslationDirection5D || iktype == IKP_Direction3D || iktype == IKP_Ray4D)) ) {
        SerializeRound3(o,info_.direction_);
    }
}

ConfigurationSpecification RobotBase::Manipulator::GetArmConfigurationSpecification(const std::string& interpolation) const
{
    if( interpolation.size() == 0 ) {
        return arm_spec_;
    }
    ConfigurationSpecification spec = arm_spec_;
    FOREACH(itgroup,spec.groups_vector_) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

ConfigurationSpecification RobotBase::Manipulator::GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation) const
{
    return IkParameterization::GetConfigurationSpecification(iktype, GetRobot()->GetName(), GetName());
}

const std::string& RobotBase::Manipulator::GetStructureHash() const
{
    if( hash_structure_.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotManipulators|SO_Kinematics);
        hash_structure_ = utils::GetMD5HashString(ss.str());
    }
    return hash_structure_;
}

const std::string& RobotBase::Manipulator::GetKinematicsStructureHash() const
{
    if( hash_kinematics_structure_.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics);
        hash_kinematics_structure_ = utils::GetMD5HashString(ss.str());
    }
    return hash_kinematics_structure_;
}

const std::string& RobotBase::Manipulator::GetInverseKinematicsStructureHash(IkParameterizationType iktype) const
{
    std::map<IkParameterizationType, std::string>::const_iterator it = hash_ik_structure_map_.find(iktype);
    if( it == hash_ik_structure_map_.end() ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_InverseKinematics,iktype);
        hash_ik_structure_map_[iktype] = utils::GetMD5HashString(ss.str());
        return hash_ik_structure_map_[iktype];
    }
    else {
        return it->second;
    }
}

void RobotBase::Manipulator::_ComputeInternalInformation()
{
    if( !utils::IsValidName(info_.name_) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("manipulator name \"%s\" is not valid"), GetName(), ORE_Failed);
    }
    RobotBasePtr probot(robot_);
    base_link_ = probot->GetLink(info_.base_link_name_);
    effector_link_ = probot->GetLink(info_.effector_link_name_);
    arm_dof_indices_.resize(0);
    gripper_dof_indices_.resize(0);
    ik_solver_.reset();
    hash_structure_.resize(0);
    hash_kinematics_structure_.resize(0);
    hash_ik_structure_map_.clear();
    if( !base_link_ || !effector_link_ ) {
        RAVELOG_WARN(str(boost::format("manipulator %s has undefined base and end effector links %s, %s\n")%GetName()%info_.base_link_name_%info_.effector_link_name_));
        arm_spec_ = ConfigurationSpecification();
    }
    else {
        vector<JointPtr> vjoints;
        std::vector<int> vmimicdofs;
        if( probot->GetChain(base_link_->GetIndex(),effector_link_->GetIndex(), vjoints) ) {
            FOREACH(it,vjoints) {
                if( (*it)->IsStatic() ) {
                    // ignore
                }
                else if( (*it)->IsMimic() ) {
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( (*it)->IsMimic(i) ) {
                            (*it)->GetMimicDOFIndices(vmimicdofs,i);
                            FOREACHC(itmimicdof,vmimicdofs) {
                                if( find(arm_dof_indices_.begin(),arm_dof_indices_.end(),*itmimicdof) == arm_dof_indices_.end() ) {
                                    arm_dof_indices_.push_back(*itmimicdof);
                                }
                            }
                        }
                        else if( (*it)->GetDOFIndex() >= 0 ) {
                            arm_dof_indices_.push_back((*it)->GetDOFIndex()+i);
                        }
                    }
                }
                else if( (*it)->GetDOFIndex() < 0) {
                    RAVELOG_WARN(str(boost::format("manipulator arm contains joint %s without a dof index, ignoring...\n")%(*it)->GetName()));
                }
                else { // ignore static joints
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        arm_dof_indices_.push_back((*it)->GetDOFIndex()+i);
                    }
                }
            }
            // initialize the arm configuration spec
            arm_spec_ = probot->GetConfigurationSpecificationIndices(arm_dof_indices_);
        }
        else {
            RAVELOG_WARN(str(boost::format("manipulator %s failed to find chain between %s and %s links\n")%GetName()%base_link_->GetName()%effector_link_->GetName()));
        }
    }

    // init the gripper dof indices
    std::vector<dReal> vChuckingDirection;
    size_t ichuckingdirection = 0;
    FOREACHC(itjointname,info_.gripper_joint_names_vector_) {
        JointPtr pjoint = probot->GetJoint(*itjointname);
        if( !pjoint ) {
            RAVELOG_WARN(str(boost::format("could not find gripper joint %s for manipulator %s")%*itjointname%GetName()));
            ichuckingdirection++;
        }
        else {
            if( pjoint->GetDOFIndex() >= 0 ) {
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( find(arm_dof_indices_.begin(), arm_dof_indices_.end(), pjoint->GetDOFIndex()+i) != arm_dof_indices_.end() ) {
                        RAVELOG_ERROR(str(boost::format("manipulator %s gripper dof %d is also part of arm dof! excluding from gripper...")%GetName()%(pjoint->GetDOFIndex()+i)));
                    }
                    else {
                        gripper_dof_indices_.push_back(pjoint->GetDOFIndex()+i);
                        if( ichuckingdirection < info_.chucking_direction_vector_.size() ) {
                            vChuckingDirection.push_back(info_.chucking_direction_vector_[ichuckingdirection++]);
                        }
                        else {
                            vChuckingDirection.push_back(0);
                            RAVELOG_WARN(str(boost::format("manipulator %s chucking direction not correct length, might get bad chucking/release grasping")%GetName()));
                        }
                    }
                }
            }
            else {
                ++ichuckingdirection;
                RAVELOG_WARN(str(boost::format("manipulator %s gripper joint %s is not active, so has no dof index. ignoring.")%GetName()%*itjointname));
            }
        }
    }
    info_.chucking_direction_vector_.swap(vChuckingDirection);
}

}

﻿// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file   robot.h
    \brief  Base robot and manipulator description.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_ROBOT_H
#define OPENRAVE_ROBOT_H

#include <openrave/kinbody.h>
#include <openrave/numerical.h>
#include <openrave/sensor_base.h>

namespace OpenRAVE
{

/** \brief <b>[interface]</b> 
    A robot is a kinematic body that has attached manipulators, sensors, and controllers. 
	<b>If not specified, method is not multi-thread safe.</b> 
	See \ref arch_robot.
    \ingroup interfaces
 */
class OPENRAVE_API RobotBase : public KinBody 
{
public:
    /// \brief holds all user-set manipulator information used to initialize the Manipulator class.
    ///
    /// This is serializable and independent of environment.
    class OPENRAVE_API ManipulatorInfo 
	{
    public:
        ManipulatorInfo()
            : direction_(0, 0, 1)
        {
        }
        virtual ~ManipulatorInfo()
        {
        }

        virtual void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator,
			dReal unit_scale = 1.0, int options = 0) const;
        virtual void DeserializeJSON(const rapidjson::Value& value, dReal unit_scale = 1.0);

        std::string name_;
        std::string base_link_name_, effector_link_name_; //!< name of the base and effector links of the robot used to determine the chain
        Transform local_tool_transform_;
        std::vector<dReal> chucking_direction_vector_; //!< the normal direction to move joints for the hand to grasp something
        Vector direction_;
        std::string ik_solver_xml_id_; //!< xml id of the IkSolver interface to attach
        std::vector<std::string> gripper_joint_names_vector_; //!< names of the gripper joints
    };
    typedef std::shared_ptr<ManipulatorInfo> ManipulatorInfoPtr;
    typedef std::shared_ptr<ManipulatorInfo const> ManipulatorInfoConstPtr;

    /// \brief Defines a chain of joints for an arm and set of joints for a gripper. Simplifies operating with them.
    class OPENRAVE_API Manipulator : public std::enable_shared_from_this<Manipulator> 
	{
        Manipulator(RobotBasePtr probot, const ManipulatorInfo& info);
        Manipulator(const Manipulator& r);

        /// \brief can switch the underyling robot
        Manipulator(RobotBasePtr probot, std::shared_ptr<Manipulator const> r);

    public:
        virtual ~Manipulator();

        /// \brief return a serializable info holding everything to initialize a manipulator
        inline const ManipulatorInfo& GetInfo() const
        {
            return info_;
        }

        /// \brief Return the transformation of the manipulator frame
        ///
        /// The manipulator frame is defined by the the end effector link position * GetLocalToolTransform()
        /// All inverse kinematics and jacobian queries are specifying this frame.
        virtual Transform GetTransform() const;

        /// \brief return the linear/angular velocity of the manipulator coordinate system
        virtual std::pair<Vector, Vector> GetVelocity() const;

        virtual Transform GetEndEffectorTransform() const
        {
            return GetTransform();
        }

        virtual const std::string& GetName() const
        {
            return info_.name_;
        }

        /// \brief get robot that manipulator belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline RobotBasePtr GetRobot(bool trylock = false) const
        {
            if (trylock) 
			{
                return robot_.lock();
            } else 
			{
                return RobotBasePtr(robot_);
            }
        }

        /// \brief Sets the ik solver and initializes it with the current manipulator.
        ///
        /// Due to complications with translation,rotation,direction,and ray ik,
        /// the ik solver should take into account the grasp transform (info_.local_tool_transform_) internally.
        /// The actual ik primitives are transformed into the base frame only.
        virtual bool SetIkSolver(IkSolverBasePtr iksolver);

        /// \brief Returns the currently set ik solver
        virtual IkSolverBasePtr GetIkSolver() const;

        /// \brief the base used for the iksolver
        virtual LinkPtr GetBase() const
        {
            return base_link_;
        }

        /// \brief the end effector link (used to define workspace distance)
        virtual LinkPtr GetEndEffector() const
        {
            return effector_link_;
        }

        /// \brief Release all bodies grabbed by the end effector of this manipualtor
        virtual void ReleaseAllGrabbed()
        {
            RobotBasePtr probot(robot_);
            probot->ReleaseAllGrabbedWithLink(*effector_link_);
        }

        /// \brief Return transform with respect to end effector defining the grasp coordinate system
        virtual const Transform& GetLocalToolTransform() const
        {
            return info_.local_tool_transform_;
        }

        /// \brief Sets the local tool transform with respect to the end effector link.
        ///
        /// Because this call will change manipulator hash, it resets the loaded IK and sends the Prop_RobotManipulatorTool message.
        virtual void SetLocalToolTransform(const Transform& t);

        /// \brief new name for manipulator
        ///
        /// \throw OpenRAVEException if name is already used in another manipulator
        virtual void SetName(const std::string& name);

        /// \brief Gripper indices of the joints that the  manipulator controls.
        virtual const std::vector<int>& GetGripperIndices() const
        {
            return gripper_dof_indices_;
        }

        /// \brief Return the indices of the DOFs of the manipulator, which are used for inverse kinematics.
        ///
        /// Usually the DOF indices of the chain from pBase to pEndEffector
        virtual const std::vector<int>& GetArmIndices() const
        {
            return arm_dof_indices_;
        }

        /// \brief returns the number of DOF for the arm indices. Equivalent to GetArmIndices().size()
        virtual int GetArmDOF() const;

        /// \brief returns the number of DOF for the gripper indices. Equivalent to GetGripperIndices().size()
        virtual int GetGripperDOF() const;

        virtual const std::vector<dReal>& GetChuckingDirection() const
        {
            return info_.chucking_direction_vector_;
        }

        /// \brief sets the normal gripper direction to move joints to close/chuck the hand
        virtual void SetChuckingDirection(const std::vector<dReal>& chuckingdirection);

        /// \brief Sets the local tool direction with respect to the end effector link.
        ///
        /// Because this call will change manipulator hash, it resets the loaded IK and sends the Prop_RobotManipulatorTool message.
        virtual void SetLocalToolDirection(const Vector& direction);

        /// \brief direction of palm/head/manipulator used for approaching. defined inside the manipulator/grasp coordinate system
        virtual const Vector& GetLocalToolDirection() const
        {
            return info_.direction_;
        }

        /// \brief returns the current values of the manipulator arm.
        ///
        /// Aquivalent to GetRobot()->GetDOFValues(v, GetArmIndices())
        virtual void GetArmDOFValues(std::vector<dReal>& v) const;

        /// \brief returns the current values of the manipulator gripper
        ///
        /// Aquivalent to GetRobot()->GetDOFValues(v, GetGripperIndices())
        virtual void GetGripperDOFValues(std::vector<dReal>& v) const;

        /// \brief Find a close solution to the current robot's joint values.
        ///
        /// The function is a wrapper around the IkSolver interface.
        /// Note that the solution returned is not guaranteed to be the closest solution. In order to compute that, will have to
        /// compute all the ik solutions using FindIKSolutions.
        /// \param param The transformation of the end-effector in the global coord system
        /// \param solution Will be of size GetArmIndices().size() and contain the best solution
        /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        virtual bool FindIKSolution(const IkParameterization& param, std::vector<dReal>& solution, int filteroptions) const;
        virtual bool FindIKSolution(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions) const;
        virtual bool FindIKSolution(const IkParameterization& param, int filteroptions, IkReturnPtr ikreturn) const;
        virtual bool FindIKSolution(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn) const;

        /// \brief Find all the IK solutions for the given end effector transform
        ///
        /// \param param The transformation of the end-effector in the global coord system
        /// \param solutions An array of all solutions, each element in solutions is of size GetArmIndices().size()
        /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        virtual bool FindIKSolutions(const IkParameterization& param, std::vector<std::vector<dReal>>& solutions, int filteroptions) const;
        virtual bool FindIKSolutions(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal>>& solutions, int filteroptions) const;
        virtual bool FindIKSolutions(const IkParameterization& param, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const;
        virtual bool FindIKSolutions(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns) const;

        /** \brief returns the parameterization of a given IK type for the current manipulator position.

            Ideally pluging the returned ik parameterization into FindIkSolution should return the a manipulator configuration
            such that a new call to GetIkParameterization returns the same values. In other words:
            \code
            ikparam = manip->GetIkParameterization(iktype);
            ... move robot
            std::vector<dReal> sol;
            if( FindIKSolution(ikparam,sol, filteroptions) ) {
                manip->GetRobot()->SetActiveDOFs(manip->GetArmIndices());
                manip->GetRobot()->SetActiveDOFValues(sol);
                BOOST_ASSERT( dist(manip->GetIkParameterization(iktype), ikparam) <= epsilon );
            }
            \endcode
            \param iktype the type of parameterization to request
            \param inworld if true will return the parameterization in the world coordinate system, otherwise in the base link (\ref GetBase()) coordinate system
         */
        virtual IkParameterization GetIkParameterization(IkParameterizationType iktype, bool inworld = true) const;

        /** \brief returns a full parameterization of a given IK type for the current manipulator position using an existing IkParameterization as the seed.

            Custom data is copied to the new parameterization. Furthermore, some IK types like Lookat3D and TranslationLocalGlobal6D set constraints in the global coordinate system of the manipulator. Because these values are not stored in manipulator itself, they have to be passed in through an existing IkParameterization.
            Ideally pluging the returned ik parameterization into FindIkSolution should return the a manipulator configuration
            such that a new call to GetIkParameterization returns the same values.
            \param ikparam The parameterization to use as seed.
            \param inworld if true will return the parameterization in the world coordinate system, otherwise in the base link (\ref GetBase()) coordinate system
         */
        virtual IkParameterization GetIkParameterization(const IkParameterization& ikparam, bool inworld = true) const;

        /// \brief Get all child joints of the manipulator starting at the pEndEffector link
        virtual void GetChildJoints(std::vector<JointPtr>& vjoints) const;

        /// \brief Get all child DOF indices of the manipulator starting at the pEndEffector link
        virtual void GetChildDOFIndices(std::vector<int>& vdofndices) const;

        /// \brief returns true if a link is part of the child links of the manipulator.
        ///
        /// The child links do not include the arm links.
        bool IsChildLink(LinkConstPtr plink) const RAVE_DEPRECATED
        {
            return IsChildLink(*plink);
        }
        virtual bool IsChildLink(const KinBody::Link& link) const;

        /// \brief Get all child links of the manipulator starting at pEndEffector link.
        ///
        /// The child links do not include the arm links.
        virtual void GetChildLinks(std::vector<LinkPtr>& vlinks) const;

        /** \brief Get all links that are independent of the arm and gripper joints

            In other words, returns all links not on the path from the base to the end effector and not children of the end effector. The base and all links rigidly attached to it are also returned.
         */
        virtual void GetIndependentLinks(std::vector<LinkPtr>& vlinks) const;

        /** \brief Checks collision with only the gripper and the rest of the environment with the current link transforms. Ignores disabled links.

            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        virtual bool CheckEndEffectorCollision(CollisionReportPtr report = CollisionReportPtr()) const;

        /** \brief Checks collision with only the gripper and the rest of the environment given a new end-effector transform. Ignores disabled links.

            \param tEE the end effector transform
            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        virtual bool CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report = CollisionReportPtr()) const;

        /** \brief Checks self-collision with only the gripper with the rest of the robot. Ignores disabled links.

            \param[out] report [optional] collision report
            \param[in] bIgnoreManipulatorLinks if true, then will ignore any links that can potentially move because of manipulator moving.
            \return true if a collision occurred
         */
        virtual bool CheckEndEffectorSelfCollision(CollisionReportPtr report = CollisionReportPtr(), bool bIgnoreManipulatorLinks = false) const;

        /** \brief Checks self-collision with only the gripper given its end-effector transform with the rest of the robot. Ignores disabled links.

            \param tEE the end effector transform
            \param[out] report [optional] collision report
            \param[in] bIgnoreManipulatorLinks if true, then will ignore any links that can potentially move because of manipulator moving.
            \return true if a collision occurred
         */
        virtual bool CheckEndEffectorSelfCollision(const Transform& tEE, CollisionReportPtr report = CollisionReportPtr(), bool bIgnoreManipulatorLinks = false) const;

        /** \brief Checks environment collisions with only the gripper given an IK parameterization of the gripper.

            Some IkParameterizations can fully determine the gripper 6DOF location. If the type is Transform6D or the manipulator arm DOF <= IkParameterization DOF, then this would be possible. In the latter case, an ik solver is required to support the ik parameterization.
            \param ikparam the ik parameterization determining the gripper transform
            \param[inout] report [optional] collision report
            \param[in] numredundantsamples If > 0, will check collision using the full redundant degree of freedom of the IkParameterization. For example, if ikparam is IKP_TranslationDirection5D, then there's 1 degree of freedom around the axis. The manipulator will have numredundantsamples samples around this degree of freedom, and check each one. If == 0, then will use the manipulator's IK solver to get the end effector transforms to sample.
            \return true if a collision occurred
            /// \throw OpenRAVEException if the gripper location cannot be fully determined from the passed in ik parameterization.
         */
        virtual bool CheckEndEffectorCollision(const IkParameterization& ikparam, CollisionReportPtr report = CollisionReportPtr(), int numredundantsamples = 0) const;

        /** \brief Checks self-collisions with only the gripper given an IK parameterization of the gripper.

            Some IkParameterizations can fully determine the gripper 6DOF location. If the type is Transform6D or the manipulator arm DOF <= IkParameterization DOF, then this would be possible. In the latter case, an ik solver is required to support the ik parameterization.
            \param ikparam the ik parameterization determining the gripper transform
            \param[in] numredundantsamples see CheckEndEffectorCollision
            \param[out] report [optional] collision report
            \param[in] bIgnoreManipulatorLinks if true, then will ignore any links that can potentially move because of manipulator moving.
            \return true if a collision occurred
            /// \throw OpenRAVEException if the gripper location cannot be fully determined from the passed in ik parameterization.
         */
        virtual bool CheckEndEffectorSelfCollision(const IkParameterization& ikparam, CollisionReportPtr report = CollisionReportPtr(), int numredundantsamples = 0, bool bIgnoreManipulatorLinks = false) const;

        /** \brief Checks collision with the environment with all the independent links of the robot. Ignores disabled links.

            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        virtual bool CheckIndependentCollision(CollisionReportPtr report = CollisionReportPtr()) const;

        /** \brief Checks collision with a target body and all the independent links of the robot. Ignores disabled links.

            \param[in] the body to check the independent links with
            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        //virtual bool CheckIndependentCollision(KinBodyConstPtr body, CollisionReportPtr report = CollisionReportPtr()) const;

        /// \brief return true if the body is being grabbed by any link on this manipulator
        bool IsGrabbing(KinBodyConstPtr body) const RAVE_DEPRECATED
        {
            return IsGrabbing(*body);
        }
        virtual bool IsGrabbing(const KinBody& body) const;

        /// \brief computes the jacobian of the manipulator arm indices of the current manipulator frame world position.
        ///
        /// The manipulator frame is computed from Manipulator::GetTransform()
        virtual void CalculateJacobian(std::vector<dReal>& jacobian) const;

        /// \brief calls std::vector version of CalculateJacobian internally, a little inefficient since it copies memory
        virtual void CalculateJacobian(boost::multi_array<dReal, 2>& jacobian) const;

        /// \brief computes the quaternion jacobian of the manipulator arm indices from the current manipulator frame rotation.
        virtual void CalculateRotationJacobian(std::vector<dReal>& jacobian) const;

        /// \brief calls std::vector version of CalculateRotationJacobian internally, a little inefficient since it copies memory
        virtual void CalculateRotationJacobian(boost::multi_array<dReal, 2>& jacobian) const;

        /// \brief computes the angule axis jacobian of the manipulator arm indices.
        virtual void CalculateAngularVelocityJacobian(std::vector<dReal>& jacobian) const;

        /// \brief calls std::vector version of CalculateAngularVelocityJacobian internally, a little inefficient since it copies memory
        virtual void CalculateAngularVelocityJacobian(boost::multi_array<dReal, 2>& jacobian) const;

        /// \brief return a copy of the configuration specification of the arm indices
        ///
        /// Note that the return type is by-value, so should not be used in iteration
        virtual ConfigurationSpecification GetArmConfigurationSpecification(const std::string& interpolation = "") const;

        /// \brief return a copy of the configuration specification of this arm under a particular IkParameterizationType
        ///
        /// Note that the return type is by-value, so should not be used in iteration
        virtual ConfigurationSpecification GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation = "") const;

        /// \brief returns the serialization of the manipulator. If options & SO_InverseKinematics, then use iktype
        virtual void serialize(std::ostream& o, int options, IkParameterizationType iktype = IKP_None) const;

        /// \brief Return hash of just the manipulator definition.
        virtual const std::string& GetStructureHash() const;

        /// \brief Return hash of all kinematics information of just the manipulator
        ///
        /// This includes joint axes, joint positions, and final grasp transform. Hash is used to cache the solvers.
        virtual const std::string& GetKinematicsStructureHash() const;

        /// \brief Return hash of the information necessary to compute a certain ik
        ///
        /// This includes joint axes, joint positions, and final grasp transform. Hash is used to cache the solvers.
        virtual const std::string& GetInverseKinematicsStructureHash(IkParameterizationType iktype) const;

    protected:
        /// \brief compute internal information from user-set info
        virtual void _ComputeInternalInformation();

        ManipulatorInfo info_; //!< user-set information
    private:
        RobotBaseWeakPtr robot_;
        LinkPtr base_link_, effector_link_; //!< contains weak links to robot
        std::vector<int> gripper_dof_indices_, arm_dof_indices_;
        ConfigurationSpecification arm_spec_; //!< reflects arm_dof_indices_
        mutable IkSolverBasePtr ik_solver_;
        mutable std::string hash_structure_, hash_kinematics_structure_;
        mutable std::map<IkParameterizationType, std::string> hash_ik_structure_map_;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class OpenRAVEXMLParser::ManipulatorXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
        friend class XFileReader;
#else
        friend class ::OpenRAVEXMLParser::ManipulatorXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
        friend class ::XFileReader;
#endif
#endif
        friend class ColladaReader;
        friend class RobotBase;
    };
    typedef std::shared_ptr<RobotBase::Manipulator> ManipulatorPtr;
    typedef std::shared_ptr<RobotBase::Manipulator const> ManipulatorConstPtr;
    typedef std::weak_ptr<RobotBase::Manipulator> ManipulatorWeakPtr;

    /// \brief holds all user-set attached sensor information used to initialize the AttachedSensor class.
    ///
    /// This is serializable and independent of environment.
    class OPENRAVE_API AttachedSensorInfo 
	{
    public:
        AttachedSensorInfo()
        {
        }
        virtual ~AttachedSensorInfo()
        {
        }

        std::string name_;
        std::string link_name_; //!< the robot link that the sensor is attached to
        Transform relative_transform_; //!< relative transform of the sensor with respect to the attached link
        std::string sensor_name_; //!< name of the sensor interface to create, in other words the sensor type
        SensorBase::SensorGeometryPtr sensor_geometry_; //!< the sensor geometry to initialize the sensor with
        virtual void SerializeJSON(rapidjson::Value& value,
			rapidjson::Document::AllocatorType& allocator,
			dReal unit_scale = 1.0, int options = 0) const;
        virtual void DeserializeJSON(const rapidjson::Value& value, dReal unit_scale = 1.0);
    };
    typedef std::shared_ptr<AttachedSensorInfo> AttachedSensorInfoPtr;
    typedef std::shared_ptr<AttachedSensorInfo const> AttachedSensorInfoConstPtr;

    /// \brief Attaches a sensor to a link on the robot.
    class OPENRAVE_API AttachedSensor : public std::enable_shared_from_this<AttachedSensor> 
	{
    public:
        AttachedSensor(RobotBasePtr probot);
        AttachedSensor(RobotBasePtr probot, const AttachedSensor& sensor, int cloningoptions);
        AttachedSensor(RobotBasePtr probot, const AttachedSensorInfo& info);
        virtual ~AttachedSensor();

        virtual SensorBasePtr GetSensor() const
        {
            return sensor_;
        }

        virtual LinkPtr GetAttachingLink() const
        {
            return LinkPtr(pattachedlink);
        }
        virtual const Transform& GetRelativeTransform() const
        {
            return info_.relative_transform_;
        }
        virtual Transform GetTransform() const
        {
            return LinkPtr(pattachedlink)->GetTransform() * info_.relative_transform_;
        }

        /// \brief get robot that manipulator belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline RobotBasePtr GetRobot(bool trylock = false) const
        {
            if (trylock) {
                return _probot.lock();
            } else {
                return RobotBasePtr(_probot);
            }
        }
        virtual const std::string& GetName() const
        {
            return info_.name_;
        }

        /// retrieves the current data from the sensor
        virtual SensorBase::SensorDataPtr GetData() const;

        virtual void SetRelativeTransform(const Transform& t);

        virtual void serialize(std::ostream& o, int options) const;

        /// \brief return hash of the sensor definition
        virtual const std::string& GetStructureHash() const;

        /// \brief Updates several fields in \ref info_ depending on the current state of the attached sensor
        ///
        /// \param type the type of sensor geometry that should be updated in info_
        virtual void UpdateInfo(SensorBase::SensorType type = SensorBase::ST_Invalid);

        /// \brief returns the attached sensor info
        ///
        /// \param type the type of sensor geometry that should be updated in info_
        inline const AttachedSensorInfo& UpdateAndGetInfo(SensorBase::SensorType type = SensorBase::ST_Invalid)
        {
            UpdateInfo(type);
            return info_;
        }

        /// \brief returns the attached sensor info
        inline const AttachedSensorInfo& GetInfo() const
        {
            return info_;
        }

    private:
        /// \brief compute internal information from user-set info
        //virtual void _ComputeInternalInformation();

        AttachedSensorInfo info_; //!< user specified data

        RobotBaseWeakPtr _probot;
        SensorBasePtr sensor_; //!< initialized by _ComputeInternalInformation when added to the environment
        LinkWeakPtr pattachedlink; //!< the robot link that the sensor is attached to
        SensorBase::SensorDataPtr pdata; //!< pointer to a preallocated data using psensor->CreateSensorData()
        mutable std::string hash_structure_;
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class OpenRAVEXMLParser::AttachedSensorXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
        friend class XFileReader;
#else
        friend class ::OpenRAVEXMLParser::AttachedSensorXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
        friend class ::XFileReader;
#endif
#endif
        friend class ColladaReader;
        friend class RobotBase;
    };
    typedef std::shared_ptr<RobotBase::AttachedSensor> AttachedSensorPtr;
    typedef std::shared_ptr<RobotBase::AttachedSensor const> AttachedSensorConstPtr;
    typedef std::weak_ptr<RobotBase::AttachedSensor> AttachedSensorWeakPtr;

    /// \brief holds all user-set attached kinbody information used to initialize the AttachedKinBody class.
    ///
    /// This is serializable and independent of environment.
    class OPENRAVE_API ConnectedBodyInfo
	{
    public:
        ConnectedBodyInfo();
        virtual ~ConnectedBodyInfo()
        {
        }

        /// \brief Updates the infos depending on the robot at the identity and zero position.
        virtual void InitInfoFromBody(RobotBase& robot);
        virtual void SerializeJSON(rapidjson::Value& value, 
			rapidjson::Document::AllocatorType& allocator, dReal unit_scale = 1.0, int options = 0) const;
        virtual void DeserializeJSON(const rapidjson::Value& value, dReal unit_scale = 1.0);

        std::string name_; //!< the name of the connected body info
        std::string link_name_; //!< the robot link that the body is attached to
        std::string uri_; //< the uri where the connected body came from. this is used when writing back to the filename.
        Transform relative_transform_; //!< relative transform of the body with respect to the attached link. The link transforms are multiplied by the transform of link_name_ and relative_transform_ to put them on the real robot.
        std::vector<KinBody::LinkInfoPtr> link_infos_vector_; //!< extracted link infos representing the connected body. The names are the original "desired" names. Should not consider link_name_ and relative_transform_.
        std::vector<KinBody::JointInfoPtr> joint_infos_vector_; //!< extracted joint infos (inluding passive) representing the connected body. The names are the original "desired" names.
        std::vector<RobotBase::ManipulatorInfoPtr> manipulator_infos_vector_; //!< extracted manip infos representing the connected body. The names are the original "desired" names.
        std::vector<RobotBase::AttachedSensorInfoPtr> attached_sensor_infos_vector_; //!< extracted sensor infos representing the connected body. The names are the original "desired" names.
        bool is_active_; //!< if true, then add the connected body. Otherwise do not add it.
    };
    typedef std::shared_ptr<ConnectedBodyInfo> ConnectedBodyInfoPtr;
    typedef std::shared_ptr<ConnectedBodyInfo const> ConnectedBodyInfoConstPtr;

    /// \brief Attaches a kinbody to a link on the robot.
    class OPENRAVE_API ConnectedBody : public std::enable_shared_from_this<ConnectedBody> 
	{
    public:
        ConnectedBody(RobotBasePtr probot);
        ConnectedBody(RobotBasePtr probot, const ConnectedBody& connectedBody, int cloningoptions);
        ConnectedBody(RobotBasePtr probot, const ConnectedBodyInfo& info);
        virtual ~ConnectedBody();

        /// \brief have the connected body to be added to the robot kinematics. The active level has nothing to do with visibility or enabling of the links.
        ///
        /// Can only be called when robot is not added to the environment
        virtual bool SetActive(bool active);

        /// \brief return true
        virtual bool IsActive();

        /// \brief if the connected body is activated and added to the robot, this is a helper functions to enable/disable all the links
        virtual void SetLinkEnable(bool benable);

        /// \brief if the connected body is activated and added to the robot, this is a helper functions to enable/disable all the links
        virtual void SetLinkVisible(bool bvisible);

        /// \brief gets the resolved links added to the robot.
        ///
        /// Has one-to-one correspondence with info_.link_infos_vector_
        virtual void GetResolvedLinks(std::vector<KinBody::LinkPtr>& links);

        /// \brief gets the resolved links added to the robot.
        ///
        /// Has one-to-one correspondence with info_.joint_infos_vector_
        virtual void GetResolvedJoints(std::vector<KinBody::JointPtr>& joints);

        /// \brief gets the resolved links added to the robot.
        ///
        /// Has one-to-one correspondence with info_.manipulator_infos_vector_
        virtual void GetResolvedManipulators(std::vector<RobotBase::ManipulatorPtr>& manipulators);

        /// \brief gets the resolved links added to the robot.
        ///
        /// Has one-to-one correspondence with info_.attached_sensor_infos_vector_
        virtual void GetResolvedAttachedSensors(std::vector<RobotBase::AttachedSensorPtr>& attachedsensors);

        virtual LinkPtr GetAttachingLink() const
        {
            return LinkPtr(_pattachedlink);
        }
        virtual const Transform& GetRelativeTransform() const
        {
            return info_.relative_transform_;
        }

        /// \brief return the transform of the base link of the connecting body
        virtual Transform GetTransform() const
        {
            return LinkPtr(_pattachedlink)->GetTransform() * info_.relative_transform_;
        }

        /// \brief get robot that manipulator belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline RobotBasePtr GetRobot(bool trylock = false) const
        {
            if (trylock) 
			{
                return attached_robot_.lock();
            } else
			{
                return RobotBasePtr(attached_robot_);
            }
        }

        inline const std::string& GetName() const
        {
            return info_.name_;
        }

        // virtual void serialize(std::ostream& o, int options) const;

        /// \brief return hash of the attached kinbody definition
        // virtual const std::string& GetStructureHash() const;

        /// \brief returns the attached kinbody info
        inline const ConnectedBodyInfo& GetInfo() const
        {
            return info_;
        }

    private:
        ConnectedBodyInfo info_; //!< user specified data (to be serialized and saved), should not contain dynamically generated parameters.

        std::string _nameprefix; //!< the name prefix to use for all the resolved link names.
        std::string _dummyPassiveJointName; //!< the joint that is used to attach the connected body to the robot link
        KinBody::JointPtr _pDummyJointCache; //!< cached Joint used for _dummyPassiveJointName
        std::vector<std::pair<std::string, RobotBase::LinkPtr>> _vResolvedLinkNames; //!< for every entry in info_.link_infos_vector_, the resolved link names added to the robot. Also serves as cache for pointers
        std::vector<std::pair<std::string, RobotBase::JointPtr>> _vResolvedJointNames; //!< for every entry in info_.joint_infos_vector_, the resolved link names. Also serves as cache for pointers.
        std::vector<std::pair<std::string, RobotBase::ManipulatorPtr>> _vResolvedManipulatorNames; //!< for every entry in info_._vManipInfos. Also serves as cache for pointers
        std::vector<std::pair<std::string, RobotBase::AttachedSensorPtr>> _vResolvedAttachedSensorNames; //!< for every entry in info_._vAttachedSensorResolvedNames. Also serves as cache for pointers

        RobotBaseWeakPtr attached_robot_; //!< the robot that the body is attached to
        LinkWeakPtr _pattachedlink; //!< the robot link that the body is attached to
        mutable std::string __hashstructure;

        friend class ColladaReader;
        friend class RobotBase;
    };

    typedef std::shared_ptr<RobotBase::ConnectedBody> ConnectedBodyPtr;
    typedef std::shared_ptr<RobotBase::ConnectedBody const> ConnectedBodyConstPtr;
    typedef std::weak_ptr<RobotBase::ConnectedBody> ConnectedBodyWeakPtr;

    /// \brief Helper class derived from KinBodyStateSaver to additionaly save robot information.
    class OPENRAVE_API RobotStateSaver : public KinBodyStateSaver 
	{
    public:
        RobotStateSaver(RobotBasePtr probot, int options = Save_LinkTransformation 
			| Save_LinkEnable | Save_ActiveDOF | Save_ActiveManipulator);

        virtual ~RobotStateSaver();

        /// \brief restore the state
        ///
        /// \param robot if set, will attempt to restore the stored state to the passed in body, otherwise will restore it for the original body.
        /// \throw OpenRAVEException if the passed in body is not compatible with the saved state, will throw
        virtual void Restore(std::shared_ptr<RobotBase> robot = std::shared_ptr<RobotBase>());

        /// \brief release the body state. _pbody will not get restored on destruction
        ///
        /// After this call, it will still be possible to use \ref Restore.
        virtual void Release();

    protected:
        RobotBasePtr robot_;
        std::vector<int> active_dof_indices_;
        int affine_dofs_;
        Vector affine_rotation_axis_;
        ManipulatorPtr active_manipulator_;
        Transform active_manip_local_tool_transform_;
        Vector active_manip_local_direction_;
        IkSolverBasePtr active_manip_ik_solver_;
        std::vector<Transform> manips_local_tool_vector_;
        std::vector<Vector> manips_local_direction_vector_;
        std::vector<IkSolverBasePtr> manips_ik_solver_vector_;
        std::vector<uint8_t> connected_body_active_states_vector_; //!< GetConnectedBodyActiveStates
    private:
        virtual void _RestoreRobot(std::shared_ptr<RobotBase> robot);
    };

    typedef std::shared_ptr<RobotStateSaver> RobotStateSaverPtr;

    virtual ~RobotBase();

    /// \brief Return the static interface type this class points to (used for safe casting).
    static inline InterfaceType GetInterfaceTypeStatic()
    {
        return PT_Robot;
    }

    virtual void Destroy();

    /// \brief initializes a robot with links, joints, manipulators, and sensors
    ///
    /// Calls \ref KinBody::Init(linkinfos, jointinfos) and then adds the robot-specific information afterwards
    /// \param linkinfos information for all the links. Links will be created in this order
    /// \param jointinfos information for all the joints. Joints might be rearranged depending on their mimic properties
    virtual bool Init(const std::vector<LinkInfoConstPtr>& linkinfos, const std::vector<JointInfoConstPtr>& jointinfos, const std::vector<ManipulatorInfoConstPtr>& manipinfos, const std::vector<AttachedSensorInfoConstPtr>& attachedsensorinfos, const std::vector<ConnectedBodyInfoConstPtr>& connectedbodyinfos, const std::string& uri = std::string());

    /// \brief Returns the manipulators of the robot
    virtual const std::vector<ManipulatorPtr>& GetManipulators() const;

    /// \brief Returns a manipulator from its name. If no manipulator with that name is present, returns empty pointer.
    virtual ManipulatorPtr GetManipulator(const std::string& name) const;

    virtual const std::vector<AttachedSensorPtr>& GetAttachedSensors() const
    {
        return attached_sensors_vector_;
    }

    virtual const std::vector<ConnectedBodyPtr>& GetConnectedBodies() const
    {
        return connected_bodies_vector_;
    }

    // \brief gets the active states of all connected bodies
    virtual void GetConnectedBodyActiveStates(std::vector<uint8_t>& activestates) const;

    /// \brief sets the active states for connected bodies
    virtual void SetConnectedBodyActiveStates(const std::vector<uint8_t>& activestates);

    virtual void SetName(const std::string& name);

    virtual void SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t checklimits = 1, 
		const std::vector<int>& dofindices = std::vector<int>());
    virtual void SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, uint32_t checklimits = 1);

    virtual void SetLinkTransformations(const std::vector<Transform>& transforms);
    virtual void SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues);

    virtual bool SetVelocity(const Vector& linearvel, const Vector& angularvel);
    virtual void SetDOFVelocities(const std::vector<dReal>& dofvelocities, const Vector& linearvel,
		const Vector& angularvel, uint32_t checklimits = 1);
    virtual void SetDOFVelocities(const std::vector<dReal>& dofvelocities, uint32_t checklimits = 1,
		const std::vector<int>& dofindices = std::vector<int>());

    /// \see SetTransform
    /// Also transforms the robot and updates the attached sensors and grabbed bodies.
    virtual void SetTransform(const Transform& trans);

    /** Methods using the active degrees of freedoms of the robot. Active DOFs are a way for the
        user to specify degrees of freedom of interest for a current execution block. All planners
        by default use the robot's active DOF and active manipultor. For every Get* method, there is
        a corresponding GetActive* method rather than the methods when setting joints.  The active
        DOFs also include affine transfomrations of the robot's base. Affine transformation DOFs can
        be found after the joint DOFs in this order: X, Y, Z, Rotation where rotation can be around
        a specified axis a full 3D rotation.  Usually the affine transforamtion is with respect to
        the first link in the body
        @name Affine DOFs
        @{
     */

    /// \deprecated (11/10/04), use OpenRAVE:: global variables
    static const DOFAffine DOF_NoTransform RAVE_DEPRECATED = OpenRAVE::DOF_NoTransform;
    static const DOFAffine DOF_X RAVE_DEPRECATED = OpenRAVE::DOF_X;
    static const DOFAffine DOF_Y RAVE_DEPRECATED = OpenRAVE::DOF_Y;
    static const DOFAffine DOF_Z RAVE_DEPRECATED = OpenRAVE::DOF_Z;
    static const DOFAffine DOF_RotationAxis RAVE_DEPRECATED = OpenRAVE::DOF_RotationAxis;
    static const DOFAffine DOF_Rotation3D RAVE_DEPRECATED = OpenRAVE::DOF_Rotation3D;
    static const DOFAffine DOF_RotationQuat RAVE_DEPRECATED = OpenRAVE::DOF_RotationQuat;

    /** \brief Set the joint indices and affine transformation dofs that the planner should use. If \ref DOF_RotationAxis is specified, the previously set axis is used.

        \param dofindices the indices of the original degrees of freedom to use.
        \param affine A bitmask of \ref DOFAffine values
     */
    virtual void SetActiveDOFs(const std::vector<int>& dofindices, int affine = OpenRAVE::DOF_NoTransform);

    /** \brief Set the joint indices and affine transformation dofs that the planner should use. If \ref DOF_RotationAxis is specified, then rotationaxis is set as the new axis.

        \param dofindices the indices of the original degrees of freedom to use.
        \param affine A bitmask of \ref DOFAffine values
        \param rotationaxis if \ref DOF_RotationAxis is specified, pRotationAxis is used as the new axis
     */
    virtual void SetActiveDOFs(const std::vector<int>& dofindices, int affine, const Vector& rotationaxis);
    virtual int GetActiveDOF() const
    {
        return active_dof_ >= 0 ? active_dof_ : GetDOF();
    }
    virtual int GetAffineDOF() const
    {
        return affine_dofs_;
    }

    /// \deprecated (11/10/07)
    virtual int GetAffineDOFIndex(DOFAffine dof) const
    {
        return GetActiveDOFIndices().size() + RaveGetIndexFromAffineDOF(GetAffineDOF(), dof);
    }

    /// \brief return a copy of the configuration specification of the active dofs
    ///
    /// Note that the return type is by-value, so should not be used in iteration
    virtual ConfigurationSpecification GetActiveConfigurationSpecification(const std::string& interpolation = "") const;

    /// \brief Return the set of active dof indices of the joints.
    virtual const std::vector<int>& GetActiveDOFIndices() const;

    virtual const Vector& GetAffineRotationAxis() const
    {
        return vActvAffineRotationAxis;
    }
    virtual void SetAffineTranslationLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotation3DLimits(const Vector& lower, const Vector& upper);

    /// \brief sets the quaternion limits using a starting rotation and the max angle deviation from it.
    ///
    /// \param quatangle quaternion_start * max_angle. acos(q dot quaternion_start) <= max_angle.
    /// If max_angle is 0, then will take the current transform of the robot
    virtual void SetAffineRotationQuatLimits(const Vector& quatangle);
    virtual void SetAffineTranslationMaxVels(const Vector& vels);
    virtual void SetAffineRotationAxisMaxVels(const Vector& vels);
    virtual void SetAffineRotation3DMaxVels(const Vector& vels);
    virtual void SetAffineRotationQuatMaxVels(dReal vels);
    virtual void SetAffineTranslationResolution(const Vector& resolution);
    virtual void SetAffineRotationAxisResolution(const Vector& resolution);
    virtual void SetAffineRotation3DResolution(const Vector& resolution);
    virtual void SetAffineRotationQuatResolution(dReal resolution);
    virtual void SetAffineTranslationWeights(const Vector& weights);
    virtual void SetAffineRotationAxisWeights(const Vector& weights);
    virtual void SetAffineRotation3DWeights(const Vector& weights);
    virtual void SetAffineRotationQuatWeights(dReal weights);

    virtual void GetAffineTranslationLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotation3DLimits(Vector& lower, Vector& upper) const;

    /// \brief gets the quaternion limits
    ///
    /// \param quatangle quaternion_start * max_angle. acos(q dot quaternion_start) <= max_angle
    virtual Vector GetAffineRotationQuatLimits() const
    {
        return _vRotationQuatLimitStart * _fQuatLimitMaxAngle;
    }
    virtual const Vector& GetAffineTranslationMaxVels() const
    {
        return _vTranslationMaxVels;
    }
    virtual const Vector& GetAffineRotationAxisMaxVels() const
    {
        return _vRotationAxisMaxVels;
    }
    virtual const Vector& GetAffineRotation3DMaxVels() const
    {
        return _vRotation3DMaxVels;
    }
    virtual dReal GetAffineRotationQuatMaxVels() const
    {
        return _fQuatMaxAngleVelocity;
    }
    virtual const Vector& GetAffineTranslationResolution() const
    {
        return _vTranslationResolutions;
    }
    virtual const Vector& GetAffineRotationAxisResolution() const
    {
        return _vRotationAxisResolutions;
    }
    virtual const Vector& GetAffineRotation3DResolution() const
    {
        return _vRotation3DResolutions;
    }
    virtual dReal GetAffineRotationQuatResolution() const
    {
        return _fQuatAngleResolution;
    }
    virtual const Vector& GetAffineTranslationWeights() const
    {
        return _vTranslationWeights;
    }
    virtual const Vector& GetAffineRotationAxisWeights() const
    {
        return _vRotationAxisWeights;
    }
    virtual const Vector& GetAffineRotation3DWeights() const
    {
        return _vRotation3DWeights;
    }
    virtual dReal GetAffineRotationQuatWeights() const
    {
        return _fQuatAngleResolution;
    }

    virtual void SetActiveDOFValues(const std::vector<dReal>& values, uint32_t checklimits = 1);
    virtual void GetActiveDOFValues(std::vector<dReal>& v) const;
    virtual void SetActiveDOFVelocities(const std::vector<dReal>& velocities, uint32_t checklimits = 1);
    virtual void GetActiveDOFVelocities(std::vector<dReal>& velocities) const;
    virtual void GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const;
    virtual void GetActiveDOFResolutions(std::vector<dReal>& v) const;
    virtual void GetActiveDOFWeights(std::vector<dReal>& v) const;
    virtual void GetActiveDOFVelocityLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFAccelerationLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFJerkLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFHardVelocityLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFHardAccelerationLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFHardJerkLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxVel(std::vector<dReal>& v) const
    {
        return GetActiveDOFVelocityLimits(v);
    }
    virtual void GetActiveDOFMaxAccel(std::vector<dReal>& v) const
    {
        return GetActiveDOFAccelerationLimits(v);
    }
    virtual void GetActiveDOFMaxJerk(std::vector<dReal>& v) const
    {
        return GetActiveDOFJerkLimits(v);
    }
    virtual void GetActiveDOFHardMaxVel(std::vector<dReal>& v) const
    {
        return GetActiveDOFHardVelocityLimits(v);
    }
    virtual void GetActiveDOFHardMaxAccel(std::vector<dReal>& v) const
    {
        return GetActiveDOFHardAccelerationLimits(v);
    }
    virtual void GetActiveDOFHardMaxJerk(std::vector<dReal>& v) const
    {
        return GetActiveDOFHardJerkLimits(v);
    }

    /// computes the configuration difference q1-q2 and stores it in q1. Takes into account joint limits and circular joints
    virtual void SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const;

    /// \brief sets the active manipulator of the robot
    ///
    /// \param manipname manipulator name
    /// \throw OpenRAVEException if manipulator not present, will throw an exception
    virtual ManipulatorPtr SetActiveManipulator(const std::string& manipname);
    virtual void SetActiveManipulator(ManipulatorConstPtr pmanip);
    virtual ManipulatorPtr GetActiveManipulator();
    virtual ManipulatorConstPtr GetActiveManipulator() const;

    /// \brief adds a manipulator the list
    ///
    /// Will change the robot structure hash..
    /// \return the new manipulator attached to the robot
    /// \throw OpenRAVEException If removeduplicate is false and there exists a manipulator with the same name, will throw an exception
    virtual ManipulatorPtr AddManipulator(const ManipulatorInfo& manipinfo, bool removeduplicate = false);

    /// \brief removes a manipulator from the robot list. if successful, returns true
    ///
    /// Will change the robot structure hash..
    /// if the active manipulator is set to this manipulator, it will be set to None afterwards
    virtual bool RemoveManipulator(ManipulatorPtr manip);

    /// \brief attaches a sensor to a link the list
    ///
    /// Will change the robot structure hash.
    /// \return the new attached sensor
    /// \throw OpenRAVEException If removeduplicate is false and there exists a manipulator with the same name, will throw an exception
    virtual AttachedSensorPtr AddAttachedSensor(const AttachedSensorInfo& attachedsensorinfo, bool removeduplicate = false);

    /// \brief Returns an attached sensor from its name. If no sensor is with that name is present, returns empty pointer.
    virtual AttachedSensorPtr GetAttachedSensor(const std::string& name) const;

    /// \brief tries to remove the attached sensor. If successful, returns true.
    ///
    /// Will change the robot structure hash..
    virtual bool RemoveAttachedSensor(RobotBase::AttachedSensor& attsensor);

    /// \brief adds a connected body to the list
    ///
    /// Will change the robot structure hash.
    /// \return the new connected body
    /// \throw OpenRAVEException If removeduplicate is false and there exists a manipulator with the same name, will throw an exception
    virtual ConnectedBodyPtr AddConnectedBody(const ConnectedBodyInfo& connectedBodyInfo, bool removeduplicate = false);

    /// \brief get connected body with given name active.
    virtual ConnectedBodyPtr GetConnectedBody(const std::string& name) const;

    /// \brief tries to remove the connected body. If successful, returns true.
    ///
    /// Will change the robot structure hash..
    virtual bool RemoveConnectedBody(RobotBase::ConnectedBody& connectedBody);

    /** \brief Calculates the translation jacobian with respect to a link.

        Calculates the partial differentials for the active degrees of freedom that in the path from the root node to _veclinks[index]
        (doesn't touch the rest of the values).
        \param mjacobian a 3 x ActiveDOF matrix
     */
    virtual void CalculateActiveJacobian(int index, const Vector& offset, std::vector<dReal>& jacobian) const;
    virtual void CalculateActiveJacobian(int index, const Vector& offset, boost::multi_array<dReal, 2>& jacobian) const;

    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, std::vector<dReal>& jacobian) const;
    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, boost::multi_array<dReal, 2>& jacobian) const;

    /** Calculates the angular velocity jacobian of a specified link about the axes of world coordinates.

        \param index of the link that the rotation is attached to
        \param mjacobian 3x(num ACTIVE DOF) matrix
     */
    virtual void CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& jacobian) const;
    virtual void CalculateActiveAngularVelocityJacobian(int index, boost::multi_array<dReal, 2>& jacobian) const;

    virtual const std::vector<int>& GetNonAdjacentLinks(int adjacentoptions = 0) const;

    /// \brief \ref KinBody::SetNonCollidingConfiguration, also regrabs all bodies
    virtual void SetNonCollidingConfiguration();

    //@}

    /** A grabbed body becomes part of the robot and its relative pose with respect to a robot's
        link will be fixed. KinBody::_AttachBody is called for every grabbed body in order to make
        the grabbed body a part of the robot. Once grabbed, the inter-collisions between the robot
        and the body are regarded as self-collisions; any outside collisions of the body and the
        environment are regarded as environment collisions with the robot.
        @name Grabbing Bodies
        @{
     */

    /** \brief Grab the body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pRobotLinkToGrabWith the link of this robot that will perform the grab
        \param[in] setRobotLinksToIgnore Additional robot link indices that collision checker ignore
        when checking collisions between the grabbed body and the robot.
        \return true if successful and body is grabbed.
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore);

    /** \brief Grab a body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pRobotLinkToGrabWith the link of this robot that will perform the grab
        \return true if successful and body is grabbed/
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith);

    /** \brief Grabs the body with the active manipulator's end effector.

        \param[in] body the body to be grabbed
        \param[in] setRobotLinksToIgnore Additional robot link indices that collision checker ignore
        when checking collisions between the grabbed body and the robot.
        \return true if successful and body is grabbed
     */
    virtual bool Grab(KinBodyPtr body, const std::set<int>& setRobotLinksToIgnore);

    /** \brief Grabs the body with the active manipulator's end effector.

        \param[in] body the body to be grabbed
        \return true if successful and body is grabbed
     */
    virtual bool Grab(KinBodyPtr body);

    //@}

    /** \brief Simulate the robot and update the grabbed bodies and attached sensors

        Do not call SimulationStep for the attached sensors in this function.
     */
    virtual void SimulationStep(dReal fElapsedTime);

    /// does not clone the grabbed bodies since it requires pointers from other bodies (that might not be initialized yet)
    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const
    {
        return true;
    }

    virtual void serialize(std::ostream& o, int options) const;

    /// A md5 hash unique to the particular robot structure that involves manipulation and sensing components
    /// The serialization for the attached sensors will not involve any sensor specific properties (since they can change through calibration)
    virtual const std::string& GetRobotStructureHash() const;

    /// \brief gets the robot controller
    virtual ControllerBasePtr GetController() const
    {
        return ControllerBasePtr();
    }

    /// \brief set a controller for a robot
    /// \param pController - if NULL, sets the controller of this robot to NULL. otherwise attemps to set the controller to this robot.
    /// \param args - the argument list to pass when initializing the controller
    virtual bool SetController(ControllerBasePtr controller, const std::vector<int>& dofindices, int control_transformation);

    inline RobotBasePtr shared_robot()
    {
        return std::static_pointer_cast<RobotBase>(shared_from_this());
    }

    inline RobotBaseConstPtr shared_robot_const() const
    {
        return std::static_pointer_cast<RobotBase const>(shared_from_this());
    }

protected:
    RobotBase(EnvironmentBasePtr penv);

    /// \brief Proprocess the manipulators and sensors and build the specific robot hashes.
    virtual void _ComputeInternalInformation();

    virtual void _DeinitializeInternalInformation();

    /// \brief Proprocess with connected_bodies_vector_ and reinitialize robot.
    virtual void _ComputeConnectedBodiesInformation();

    virtual void _DeinitializeConnectedBodiesInformation();

    /// \brief Called to notify the body that certain groups of parameters have been changed.
    ///
    /// This function in calls every registers calledback that is tracking the changes.
    virtual void _PostprocessChangedParameters(uint32_t parameters);

    virtual void _UpdateAttachedSensors();
    std::vector<ManipulatorPtr> manipulators_vector_; //!< \see GetManipulators
    ManipulatorPtr manipulator_active_;

    std::vector<AttachedSensorPtr> attached_sensors_vector_; //!< \see GetAttachedSensors

    std::vector<ConnectedBodyPtr> connected_bodies_vector_; //!< \see GetConnectedBodies

    std::vector<int> active_dof_indices_vector_, all_dof_indices_vector_;
    Vector vActvAffineRotationAxis;
    int active_dof_; //!< Active degrees of freedom; if -1, use robot dofs
    int affine_dofs_; //!< dofs describe what affine transformations are allowed

    Vector _vTranslationLowerLimits, _vTranslationUpperLimits, _vTranslationMaxVels, _vTranslationResolutions, _vTranslationWeights;
    /// the xyz components are used if the rotation axis is solely about X,Y,or Z; otherwise the W component is used.
    Vector _vRotationAxisLowerLimits, _vRotationAxisUpperLimits, _vRotationAxisMaxVels, _vRotationAxisResolutions, _vRotationAxisWeights;
    Vector _vRotation3DLowerLimits, _vRotation3DUpperLimits, _vRotation3DMaxVels, _vRotation3DResolutions, _vRotation3DWeights;
    Vector _vRotationQuatLimitStart;
    dReal _fQuatLimitMaxAngle, _fQuatMaxAngleVelocity, _fQuatAngleResolution, _fQuatAngleWeight;

    ConfigurationSpecification active_spec_;

private:
    virtual const char* GetHash() const
    {
        return OPENRAVE_ROBOT_HASH;
    }
    virtual const char* GetKinBodyHash() const
    {
        return OPENRAVE_KINBODY_HASH;
    }
    mutable std::string hash_robot_structure_;
    mutable std::vector<dReal> _vTempRobotJoints;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
    friend class OpenRAVEXMLParser::RobotXMLReader;
    friend class OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class OpenRAVEXMLParser::AttachedSensorXMLReader;
    friend class XFileReader;
#else
    friend class ::Environment;
    friend class ::OpenRAVEXMLParser::RobotXMLReader;
    friend class ::OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class ::OpenRAVEXMLParser::AttachedSensorXMLReader;
    friend class ::XFileReader;
#endif
#endif
    friend class ColladaWriter;
    friend class ColladaReader;
    friend class PluginDatabase;
    friend class Grabbed;
};

} // end namespace OpenRAVE

#endif // ROBOT_H

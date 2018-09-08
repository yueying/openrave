// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_CONFIGURATION_SPECIFICATION_H_
#define OPENRAVE_CONFIGURATION_SPECIFICATION_H_

#include <openrave/xml_process.h>
#include <vector>
#include <sstream>
#include <boost/function.hpp>
#include <openrave/type.h>
#include <openrave/numerical.h>

namespace OpenRAVE
{
	/** \brief A configuration specification references values in the environment that then define a configuration-space which can be searched for.

	It is composed of several groups targetting values for individual bodies. It is serialized into XML. The XML syntax is as follows:

	\code
	<configuration>
	<group name="string" offset="#OFF1" dof="#D1" interpolation="string"/>
	<group name="string" offset="#OFF2" dof="#D2" interpolation="string"/>
	</configuration>
	\endcode
	*/
	class OPENRAVE_API ConfigurationSpecification
	{
	public:

		/// \brief A group referencing the values of one body in the environment
		class OPENRAVE_API Group
		{
		public:
			Group() : offset(0), dof(0) 
			{
			}

			inline bool operator==(const Group& r) const 
			{
				return offset == r.offset 
					&& dof == r.dof 
					&& name == r.name 
					&& interpolation == r.interpolation;
			}
			inline bool operator!=(const Group& r) const 
			{
				return offset != r.offset 
					|| dof != r.dof 
					|| name != r.name 
					|| interpolation != r.interpolation;
			}

			/// \brief For each data point, the number of values to offset before data for this group starts.
			int offset;
			/// \brief The number of values in this group.
			int dof;
			/** \brief semantic information on what part of the environment the group refers to.

			Can be composed of multiple workds; the first word is the group type, and the words following narrow the specifics. Common types are:

			- \b joint_values - The joint values of a kinbody/robot. The joint names with the name of the body can follow.
			- \b joint_velocities - The joint velocities (1/second) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_accelerations - The joint accelerations (1/second^2) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_jerks - The joint jerks (1/second^3) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_snaps - The joint snaps (1/second^4) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_crackles - The joint crackles (1/second^5) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_pops - The joint pops (1/second^6) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b joint_torques - The joint torques (Newton meter) of a kinbody/robot. The name of the body with the joint names can follow.
			- \b affine_transform - An affine transformation [quaternion, translation]. The name of the body with selected affine dofs (see \ref DOFAffine) can follow.
			- \b affine_velocities - The velocity (1/second) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
			- \b affine_accelerations - The acceleration (1/second^2) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
			- \b affine_jerks - The jerk (1/second^3) of the affine transformation [rotation axis, translation velocity], the name of the body can follow.
			- \b ikparam_values - The values of an IkParmeterization. The ikparam type is stored as the second value in name
			- \b ikparam_velocities - acceleration of an IkParmeterization. The ikparam type is stored as the second value in name
			- \b ikparam_jerks - jerk of an IkParmeterization. The ikparam type is stored as the second value in name
			- \b iswaypoint - non-zero if the point represents a major knot point of the trajectory
			- \b grabbody - Grabs the body. The configuration values are 1 for grab and 0 for release. The group name format is: bodyname robotname robotlinkindex [relative_grab_pose]. relative_grab_pose is a 7 value (quaterion + translation) pose of the relative location of the body with respect to the grabbed link. Only 1 DOF is accepted.
			*/
			std::string name;
			/** \brief Describes how the data should be interpolated. Common methods are:

			- \b previous - the previous waypoint's value is always chosen
			- \b next - the next waypoint's value is always chosen
			- \b linear - linear interpolation (default)
			- \b quadratic - position is piecewise-quadratic, velocity is piecewise-linear, acceleration is one of -amax, 0, or amax. needs velocity info
			- \b cubic - 3 degree polynomial. needs velocity info.
			- \b quartic - 4 degree polynomial. needs velocity and acceleration info.
			- \b quintic - 5 degree polynomial. needs velocity and acceleration info.
			- \b sextic - 6 degree polynomial. needs velocity, acceleration, and jerk info
			*/
			std::string interpolation;
		};

		class Reader : public BaseXMLReader
		{
		public:
			Reader(ConfigurationSpecification& spec);
			virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
			virtual bool endElement(const std::string& name);
			virtual void characters(const std::string& ch);
		protected:
			ConfigurationSpecification& _spec;
			std::stringstream _ss;
			BaseXMLReaderPtr _preader;
		};

		ConfigurationSpecification();
		ConfigurationSpecification(const Group& g);
		ConfigurationSpecification(const ConfigurationSpecification& c);

		virtual ~ConfigurationSpecification() {
		}

		/// \brief return the dimension of the configuraiton space (degrees of freedom)
		virtual int GetDOF() const;

		/// \brief check if the groups form a continguous space
		///
		/// If there are two or more groups with the same semantic names, will fail. Theese groups should be merged into one.
		/// \return true if valid, otherwise false
		virtual bool IsValid() const;

		/// \brief check if the groups form a continguous space
		///
		/// If there are two or more groups with the same semantic names, will fail. Theese groups should be merged into one.
		/// \throw OpenRAVEException if not valid
		virtual void Validate() const;

		virtual bool operator==(const ConfigurationSpecification& r) const;
		virtual bool operator!=(const ConfigurationSpecification& r) const;

		/// \brief return the group whose name begins with a particular string.
		///
		/// If multiple groups exist that begin with the same string, then the shortest one is returned.
		/// \throw OpenRAVEException if a group is not found
		virtual const Group& GetGroupFromName(const std::string& name) const;

		/// \brief return the group whose name begins with a particular string.
		///
		/// If multiple groups exist that begin with the same string, then the shortest one is returned.
		/// \throw OpenRAVEException if a group is not found
		virtual Group& GetGroupFromName(const std::string& name);

		/// \brief finds the most compatible group to the given group
		///
		/// \param g the group to query, only the Group::name and Group::dof values are used
		/// \param exactmatch if true, will only return groups whose name exactly matches with g.name
		/// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		virtual std::vector<Group>::const_iterator FindCompatibleGroup(const Group& g, bool exactmatch = false) const;

		/// \brief finds the most compatible group to the given group
		///
		/// \param name the name of the group to query
		/// \param exactmatch if true, will only return groups whose name exactly matches with g.name
		/// \return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		virtual std::vector<Group>::const_iterator FindCompatibleGroup(const std::string& name, bool exactmatch = false) const;

		/** \brief Return the most compatible group that represents the time-derivative data of the group.

		For example given a 'joint_values' group, this will return the closest 'joint_velocities' group.
		\param g the group to query, only the Group::name and Group::dof values are used
		\param exactmatch if true, will only return groups whose name exactly matches with g.name
		\return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		*/
		virtual std::vector<Group>::const_iterator FindTimeDerivativeGroup(const Group& g, bool exactmatch = false) const;

		/** \brief Return the most compatible group that represents the time-derivative data of the group.

		For example given a 'joint_values' group, this will return the closest 'joint_velocities' group.
		\param name the name of the group to query
		\param exactmatch if true, will only return groups whose name exactly matches with g.name
		\return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		*/
		virtual std::vector<Group>::const_iterator FindTimeDerivativeGroup(const std::string& name, bool exactmatch = false) const;

		/** \brief Return the most compatible group that represents the time-integral data of the group.

		For example given a 'joint_velocities' group, this will return the closest 'joint_values' group.
		\param g the group to query, only the Group::name and Group::dof values are used
		\param exactmatch if true, will only return groups whose name exactly matches with g.name
		\return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		*/
		virtual std::vector<Group>::const_iterator FindTimeIntegralGroup(const Group& g, bool exactmatch = false) const;

		/** \brief Return the most compatible group that represents the time-integral data of the group.

		For example given a 'joint_velocities' group, this will return the closest 'joint_values' group.
		\param name the name of the group to query
		\param exactmatch if true, will only return groups whose name exactly matches with g.name
		\return an iterator part of _vgroups that represents the most compatible group. If no group is found, will return _vgroups.end()
		*/
		virtual std::vector<Group>::const_iterator FindTimeIntegralGroup(const std::string& name, bool exactmatch = false) const;

		/** \brief adds velocity, acceleration, etc groups for every position group.

		If the derivative groups already exist, they are checked for and/or modified. Note that the configuration space
		might be re-ordered as a result of this function call. If a new group is added, its interpolation will be
		the derivative of the position group as returned by \ref GetInterpolationDerivative.
		\param deriv The position derivative to add, this must be greater than 0. If 2 is specified, only the acceleration groups of the alread present position groups will be added.
		\param adddeltatime If true will add the 'deltatime' tag, which is necessary for trajectory sampling
		*/
		virtual void AddDerivativeGroups(int deriv, bool adddeltatime = false);

		/// \deprecated (12/07/30)
		inline void AddVelocityGroups(bool adddeltatime) RAVE_DEPRECATED {
			AddDerivativeGroups(1, adddeltatime);
		}

		/** \brief converts all the groups to the corresponding velocity groups and returns the specification

		The velocity configuration space will have a one-to-one correspondence with the original configuration.
		The interpolation of each of the groups will correspondingly represent the derivative as returned by \ref GetInterpolationDerivative.
		Only position specifications will be converted, any other groups will be left untouched.
		*/
		virtual ConfigurationSpecification ConvertToVelocitySpecification() const;

		/** \brief converts all the groups to the corresponding derivative group and returns the specification

		The new derivative configuration space will have a one-to-one correspondence with the original configuration.
		The interpolation of each of the groups will correspondingly represent the derivative as returned by \ref GetInterpolationDerivative(deriv).
		Only position specifications will be converted, any other groups will be left untouched.
		\param timederivative the number of times to take the time derivative of the position
		*/
		virtual ConfigurationSpecification ConvertToDerivativeSpecification(uint32_t timederivative = 1) const;

		/// \brief returns a new specification of just particular time-derivative groups.
		///
		/// \param timederivative the time derivative to query groups from. 0 is positions/joint values, 1 is velocities, 2 is accelerations, etc
		virtual ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

		/** \brief set the offsets of each group in order to get a contiguous configuration space
		*/
		virtual void ResetGroupOffsets();

		/// \brief adds the deltatime tag to the end if one doesn't exist and returns the index into the configuration space
		virtual int AddDeltaTimeGroup();

		/** \brief Adds a new group to the specification and returns its new offset.

		If the new group's semantic name does not exist in the current specification, adds it and returns the new offset.
		If the new group's semantic name exists in the current specification and it exactly matches, then function returns the old group's index. If the semantic names match, but parameters do not match, then an OpenRAVEException is thrown.
		This method is not responsible for merging semantic information
		*/
		virtual int AddGroup(const std::string& name, int dof, const std::string& interpolation = "");

		/** \brief Merges all the information from the input group into this group

		For groups that are merged, the interpolation is overwritten if the source group has an empty string.
		\throw OpenRAVEException throws if groups do not contain enough information to be merged or interpolations do not match.
		*/
		virtual ConfigurationSpecification& operator+= (const ConfigurationSpecification& r);

		/** \brief Return a new specification that holds the merged information from the current and input specification and the input parameter..

		For groups that are merged, the interpolation either has to match for both groups, or one of the groups needs an empty interpolation.
		\throw OpenRAVEException throws if groups do not contain enough information to be merged or interpolations do not match.
		*/
		virtual ConfigurationSpecification operator+ (const ConfigurationSpecification& r) const;

		/** \brief extracts an affine transform given the start of a configuration space point

		Looks for 'affine_transform' groups. If pbody is not initialized, will choose the first affine_transform found.
		\param[inout] t the transform holding the default values, which will be overwritten with the new values.
		\param[in] itdata data in the format of this configuration specification.
		\param[in] timederivative the time derivative of the data to extract
		\return true if at least one group was found for extracting
		*/
		virtual bool ExtractTransform(Transform& t, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int timederivative = 0) const;

		/** \brief extracts an ikparameterization given the start of a configuration space point

		Looks for 'ikparam' groups.
		\param[inout] ikparam filled with ikparameterization (if found)
		\param[in] itdata data in the format of this configuration specification
		\param[in] timederivative the time derivative of the data to extract
		\param[in] robotname optional name of robot to filter by
		\param[in] manipulatorname optional name of manipulator to filter by
		\return true if at least one group was found for extracting
		*/
		virtual bool ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative = 0, std::string const &robotname = "", std::string const &manipulatorname = "") const;

		/** \brief extracts the affine values

		Looks for 'affine_X' groups. If pbody is not initialized, will choose the first affine_X found.
		\param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
		\param[in] itdata data in the format of this configuration specification.
		\param[in] affinedofs the format of the affine dofs requested
		\param[in] timederivative the time derivative of the data to extract
		\return true if at least one group was found for extracting
		*/
		virtual bool ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative = 0) const;

		/** \brief extracts a body's joint values given the start of a configuration space point

		Looks for 'joint_X' groups. If pbody is not initialized, will choose the first joint_X found.
		\param[inout] itvalues iterator to vector that holds the default values and will be overwritten with the new values. must be initialized
		\param[in] itdata data in the format of this configuration specification.
		\param[in] indices the set of DOF indices of the body to extract and write into itvalues.
		\param[in] timederivative the time derivative of the data to extract
		\return true if at least one group was found for extracting
		*/
		virtual bool ExtractJointValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative = 0) const;

		/// \brief extracts the delta time from the configuration if one exists
		///
		/// \return true if deltatime exists in the current configuration, otherwise false
		virtual bool ExtractDeltaTime(dReal& deltatime, std::vector<dReal>::const_iterator itdata) const;

		/** \brief inserts a set of joint values into a configuration space point

		Looks for 'joint_X' groups. If pbody is not initialized, will use the first joint_X found.
		\param[inout] itdata data in the format of this configuration specification.
		\param[in] itvalues iterator to joint values to write
		\param[in] indices the set of DOF indices that itvalues represents.
		\param[in] timederivative the time derivative of the data to insert
		\return true if at least one group was found for inserting
		*/
		virtual bool InsertJointValues(std::vector<dReal>::iterator itdata, std::vector<dReal>::const_iterator itvalues, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative = 0) const;

		/** \brief sets the deltatime field of the data if one exists

		\param[inout] itdata data in the format of this configuration specification.
		\param[in] deltatime the delta time of the time stamp (from previous point)
		\return true if at least one group was found for inserting
		*/
		virtual bool InsertDeltaTime(std::vector<dReal>::iterator itdata, dReal deltatime) const;

		/** \brief Adds a new group to the specification and returns its new offset.

		\param g the group whose name, dof, and interpolation are extracted.
		If the new group's semantic name does not exist in the current specification, adds it and returns the new offset.
		If the new group's semantic name exists in the current specification and it exactly matches, then function returns the old group's index. If the semantic names match, but parameters do not match, then an OpenRAVEException is thrown.
		This method is not responsible for merging semantic information
		*/
		virtual int AddGroup(const Group& g);

		/** \brief removes all groups that match a name

		ResetGroupOffsets will be called internally to fix the indices.
		\param groupname the name used to look for groups
		\param exactmatch if true, will remove groups only if the full name matches, otherwise will remove groups that start with groupname
		\return number of groups removed
		*/
		virtual int RemoveGroups(const std::string& groupname, bool exactmatch = true);

		/** \brief extracts all the bodies that are used inside this specification

		Because the specification contains names of bodies, an environment is necessary to get the body pointers.
		\param[in] env the environment to extract the bodies from
		\param[out] usedbodies a list of the bodies being used
		*/
		virtual void ExtractUsedBodies(EnvironmentBasePtr env, std::vector<KinBodyPtr>& usedbodies) const;

		/** \brief extracts all the unique dof indices that the configuration holds for a particular body

		\param[in] body the body to query for
		\param[out] useddofindices a vector of unique DOF indices targetted for the body
		\param[out] usedconfigindices for every used index, returns the first configuration space index it came from
		*/
		virtual void ExtractUsedIndices(KinBodyPtr body, std::vector<int>& useddofindices, std::vector<int>& usedconfigindices) const;

		/// \brief swaps the data between the two configuration specifications as efficiently as possible
		virtual void Swap(ConfigurationSpecification& spec);

		typedef boost::function<int(const std::vector<dReal>&)> SetConfigurationStateFn;
		typedef boost::function<void(std::vector<dReal>&)> GetConfigurationStateFn;

		/// \brief return a function to set the states of the configuration in the environment
		virtual std::shared_ptr<SetConfigurationStateFn> GetSetFn(EnvironmentBasePtr env) const;

		/// \brief return a function to get the states of the configuration in the environment
		virtual std::shared_ptr<GetConfigurationStateFn> GetGetFn(EnvironmentBasePtr env) const;

		/** \brief given two compatible groups, convers data represented in the source group to data represented in the target group

		\param ittargetdata iterator pointing to start of target group data that should be overwritten
		\param targetstride the number of elements that to go from the next target point. Necessary if numpoints > 1.
		\param gtarget the target configuration group
		\param itsourcedata iterator pointing to start of source group data that should be read
		\param sourcestride the number of elements that to go from the next source point. Necessary if numpoints > 1.
		\param gsource the source configuration group
		\param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
		\param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
		\param filluninitialized If there exists target groups that cannot be initialized, then will set default values using the current environment. For example, the current joint values of the body will be used.
		\throw OpenRAVEException throw f groups are incompatible
		*/
		static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

		/** \brief Converts from one specification to another.

		\param ittargetdata iterator pointing to start of target group data that should be overwritten
		\param targetspec the target configuration specification
		\param itsourcedata iterator pointing to start of source group data that should be read
		\param sourcespec the source configuration specification
		\param numpoints the number of points to convert. The target and source strides are gtarget.dof and gsource.dof
		\param penv [optional] The environment which might be needed to fill in unknown data. Assumes environment is locked.
		\param filluninitialized If there exists target groups that cannot be initialized, then will set default values using the current environment. For example, the current joint values of the body will be used.
		*/
		static void ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification& targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification& sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

		/// \brief gets the name of the interpolation that represents the derivative of the passed in interpolation.
		///
		/// For example GetInterpolationDerivative("quadratic") -> "linear"
		/// \param interpolation the interpolation to start at
		/// \param deriv the number of derivatives to take, should be > 0
		static std::string GetInterpolationDerivative(const std::string& interpolation, int deriv = 1);

		std::vector<Group> _vgroups;
	};

	OPENRAVE_API std::ostream& operator<<(std::ostream& O, const ConfigurationSpecification &spec);
	OPENRAVE_API std::istream& operator >> (std::istream& I, ConfigurationSpecification& spec);

	typedef std::shared_ptr<ConfigurationSpecification> ConfigurationSpecificationPtr;
	typedef std::shared_ptr<ConfigurationSpecification const> ConfigurationSpecificationConstPtr;

}

#endif // OPENRAVE_CONFIGURATION_SPECIFICATION_H_
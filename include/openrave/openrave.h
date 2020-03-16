// -*- coding: utf-8 -*-
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
/** \file openrave.h
    \brief  Defines the public headers that every plugin must include in order to use openrave properly.
 */
#ifndef OPENRAVE_H
#define OPENRAVE_H




/// The entire %OpenRAVE library
namespace OpenRAVE 
{

#include <openrave/config.h>
#include <openrave/interfacehashes.h>

}

#include <rapidjson/document.h>
#include <openrave/logging.h>
#include <openrave/openrave_exception.h>
#include <openrave/numerical.h>
#include <openrave/user_data.h>
#include <openrave/xml_process.h>
#include <openrave/configuration_specification.h>
#include <openrave/ik_parameterization.h>
#include <openrave/openrave_macros.h>
#include <openrave/multi_controller_base.h>

namespace OpenRAVE 
{

/// \brief user data that can serialize/deserialize itself
class OPENRAVE_API SerializableData : public UserData
{
public:
    virtual ~SerializableData()
	{
    }

    /// \brief output the data of the object
    virtual void Serialize(std::ostream& O, int options=0) const = 0;

    /// \brief initialize the object
    virtual void Deserialize(std::istream& I) = 0;
};
typedef std::shared_ptr<SerializableData> SerializableDataPtr;
typedef std::weak_ptr<SerializableData> SerializableDataWeakPtr;


class IkParameterization;
class ConfigurationSpecification;



//!< Cloning Options for interfaces and environments
enum CloningOptions 
{
    Clone_Bodies = 1, //!< clone all the bodies/robots of the environment, exclude attached interfaces like sensors/controllers
    Clone_Viewer = 2, //!< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, //!< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8, //!< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
    Clone_Sensors = 0x0010, //!< if specified, will clone the sensors attached to the robot and added to the environment
    Clone_Modules = 0x0020, //!< if specified, will clone the modules attached to the environment
    Clone_IgnoreAttachedBodies = 0x00010001, //!< if set, then ignore cloning any attached bodies so attached_bodies_list_ becomes empty. Usually used to control grabbing states.
    Clone_All = 0xffffffff,
};


} // end namespace OpenRAVE


namespace OpenRAVE 
{

/// \brief Selects which DOFs of the affine transformation to include in the active configuration.
enum DOFAffine
{
    DOF_NoTransform = 0,
    DOF_X = 1,     //!< can move in the x direction
    DOF_Y = 2,     //!< can move in the y direction
    DOF_Z = 4,     //!< can move in the z direction
    DOF_XYZ=DOF_X|DOF_Y|DOF_Z,     //!< moves in xyz direction

    // DOF_RotationX fields are mutually exclusive
    DOF_RotationAxis = 8,     //!< can rotate around an axis (1 dof)
    DOF_Rotation3D = 16,     //!< can rotate freely (3 dof), the parameterization is
                             //!< theta * v, where v is the rotation axis and theta is the angle about that axis
    DOF_RotationQuat = 32,     //!< can rotate freely (4 dof), parameterization is a quaternion. In order for limits to work correctly, the quaternion is in the space of _vRotationQuatLimitStart. _vRotationQuatLimitStart is always left-multiplied before setting the transform!
    DOF_RotationMask=(DOF_RotationAxis|DOF_Rotation3D|DOF_RotationQuat), //!< mask for all bits representing 3D rotations
    DOF_Transform = (DOF_XYZ|DOF_RotationQuat), //!< translate and rotate freely in 3D space
};

/** \brief Given a mask of affine dofs and a dof inside that mask, returns the index where the value could be found.

    \param affinedofs a mask of \ref DOFAffine values
    \param dof a set of values inside affinedofs, the index of the first value is returned
    \throw OpenRAVEException throws if dof is not present in affinedofs
 */
OPENRAVE_API int RaveGetIndexFromAffineDOF(int affinedofs, DOFAffine dof);

/** \brief Given a mask of affine dofs and an index into the array, returns the affine dof that is being referenced

    \param affinedofs a mask of \ref DOFAffine values
    \param index an index into the affine dof array
    \throw OpenRAVEException throws if dof if index is out of bounds
 */
OPENRAVE_API DOFAffine RaveGetAffineDOFFromIndex(int affinedofs, int index);

/// \brief Returns the degrees of freedom needed to represent all the values in the affine dof mask.
///
/// \throw OpenRAVEException throws if
OPENRAVE_API int RaveGetAffineDOF(int affinedofs);

/** \brief Converts the transformation matrix into the specified affine values format.

    \param[out] itvalues an iterator to the vector to write the values to. Will write exactly \ref RaveGetAffineDOF(affinedofs) values.
    \param[in] t the affine transformation to convert
    \param[in] affinedofs the affine format to output values in
    \param[in] axis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetAffineDOFValuesFromTransform(std::vector<dReal>::iterator itvalues, const Transform& t, int affinedofs, const Vector& axis=Vector(0,0,1));

/** \brief Converts the linar and angular velocities into the specified affine values format.

    \param[out] itvalues an iterator to the vector to write the values to. Will write exactly \ref RaveGetAffineDOF(affinedofs) values.
    \param[in] linearvel the linear velocity to convert
    \param[in] angularvel the angular velocity to convert
    \param[in] quatrotation the rotation (in quaternion) of the frame that has the linearvel and angularvel. Used if affinedofs specified \ref DOF_RotationAxis.
    \param[in] affinedofs the affine format to output values in
    \param[in] axis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
 */
OPENRAVE_API void RaveGetAffineDOFValuesFromVelocity(std::vector<dReal>::iterator itvalues, const Vector& linearvel, const Vector& angularvel, const Vector& quatrotation, int affinedofs, const Vector& axis=Vector(0,0,1));

/** \brief Converts affine dof values into a transform.

    Note that depending on what the dof values holds, only a part of the transform will be updated.
    \param[inout] t the output transform
    \param[in] itvalues the start iterator of the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] axis optional rotation axis if affinedofs specified (vActvAffineRotationAxis of RobotBase) \ref DOF_RotationAxis
    \param[in] normalize if true will normalize rotations, should set to false if extracting velocity data
 */
OPENRAVE_API void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, 
	int affinedofs, const Vector& axis=Vector(0,0,1), bool normalize=true);

/** \brief Converts affine dof velocities into linear and angular velocity vectors

    Note that depending on what the dof values holds, only a part of the transform will be updated.
    \param[out] linearvel the output transform
    \param[in] itvalues the start iterator of the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] axis optional rotation axis if affinedofs specified (vActvAffineRotationAxis of RobotBase) \ref DOF_RotationAxis
    \param[in] normalize if true will normalize rotations, should set to false if extracting velocity data
    \param[in] quatrotation the quaternion of 3d rotation of the frame where the velocity is being measured at.
 */
OPENRAVE_API void RaveGetVelocityFromAffineDOFVelocities(Vector& linearvel, Vector& angularvel, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& axis=Vector(0,0,1), const Vector& quatrotation = Vector(1,0,0,0));

OPENRAVE_API ConfigurationSpecification RaveGetAffineConfigurationSpecification(int affinedofs,KinBodyConstPtr pbody=KinBodyConstPtr(),const std::string& interpolation="");

}

#include <openrave/plugin_info.h>
#include <openrave/interface_base.h>
#include <openrave/space_sampler_base.h>
#include <openrave/kinbody.h>
#include <openrave/trajectory_base.h>
#include <openrave/module.h>
#include <openrave/collision_checker_base.h>
#include <openrave/sensor_base.h>
#include <openrave/robot_base.h>
#include <openrave/ik_solver_base.h>
#include <openrave/planner.h>
#include <openrave/controller_base.h>
#include <openrave/physics_engine_base.h>
#include <openrave/sensor_system_base.h>
#include <openrave/viewer_base.h>
#include <openrave/environment_base.h>
#include <openrave/dummy_xml_reader.h>
#include <openrave/multi_controller_base.h>

namespace OpenRAVE {

/// \name Global Functionality - Interface Creation, Plugin Management, Logging
/// \anchor global_functionality
//@{

/// \brief Returns the a 16 character null-terminated string specifying a hash of the interfaces used for checking changes.
inline const char* RaveGetInterfaceHash(InterfaceType type)
{
    switch(type) {
    case PT_Planner: return OPENRAVE_PLANNER_HASH;
    case PT_Robot: return OPENRAVE_ROBOT_HASH;
    case PT_SensorSystem: return OPENRAVE_SENSORSYSTEM_HASH;
    case PT_Controller: return OPENRAVE_CONTROLLER_HASH;
    case PT_Module: return OPENRAVE_MODULE_HASH;
    case PT_InverseKinematicsSolver: return OPENRAVE_IKSOLVER_HASH;
    case PT_KinBody: return OPENRAVE_KINBODY_HASH;
    case PT_PhysicsEngine: return OPENRAVE_PHYSICSENGINE_HASH;
    case PT_Sensor: return OPENRAVE_SENSOR_HASH;
    case PT_CollisionChecker: return OPENRAVE_COLLISIONCHECKER_HASH;
    case PT_Trajectory: return OPENRAVE_TRAJECTORY_HASH;
    case PT_Viewer: return OPENRAVE_VIEWER_HASH;
    case PT_SpaceSampler: return OPENRAVE_SPACESAMPLER_HASH;
    default:
        throw OpenRAVEException("failed to find openrave interface type",ORE_InvalidArguments);
        return NULL;
    }
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline std::shared_ptr<T> RaveInterfaceCast(InterfaceBasePtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return std::static_pointer_cast<T>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return std::static_pointer_cast<T>(pinterface);
        }
    }
    return std::shared_ptr<T>();
}

/// \brief Safely casts from the base interface class to an openrave interface using static_pointer_cast.
///
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline std::shared_ptr<T const> RaveInterfaceConstCast(InterfaceBaseConstPtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return std::static_pointer_cast<T const>(pinterface);
        }
        // encode special cases
        if((pinterface->GetInterfaceType() == PT_Robot)&&(T::GetInterfaceTypeStatic() == PT_KinBody)) {
            return std::static_pointer_cast<T const>(pinterface);
        }
    }
    return std::shared_ptr<T>();
}

/// \brief returns a lower case string of the interface type
OPENRAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
OPENRAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// \brief Returns the openrave home directory where settings, cache, and other files are stored.
///
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
OPENRAVE_API std::string RaveGetHomeDirectory();

/// \brief Searches for a filename in the database and returns a full path/URL to it
///
/// \param filename the relative filename in the database
/// \param bRead if true will only return a file if it exists. If false, will return the filename of the first valid database directory.
/// \return a non-empty string if a file could be found.
OPENRAVE_API std::string RaveFindDatabaseFile(const std::string& filename, bool bRead=true);

/// \brief Explicitly initializes the global OpenRAVE state (optional).
///
/// Optional function to initialize openrave plugins, logging, and read OPENRAVE_* environment variables.
/// Although environment creation will automatically make sure this function is called, users might want
/// explicit control of when this happens.
/// Will not do anything if OpenRAVE runtime is already initialized. If OPENRAVE_* environment variables must be re-read, first call \ref RaveDestroy.
/// \param is_load_all_plugins If true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
/// \return 0 if successful, otherwise an error code
OPENRAVE_API int RaveInitialize(bool is_load_all_plugins=true, int level = Level_Info);

/// \brief Initializes the global state from an already loaded OpenRAVE environment.
///
/// Because of shared object boundaries, it is necessary to pass the global state pointer
/// around. If using plugin.h, this function is automatically called by \ref CreateInterfaceValidated.
/// It is also called by and every InterfaceBase constructor.
/// \param[in] globalstate
OPENRAVE_API void RaveInitializeFromState(UserDataPtr globalstate);

/// \brief A pointer to the global openrave state
/// \return a managed pointer to the state.
OPENRAVE_API UserDataPtr RaveGlobalState();

/// \brief Destroys the entire OpenRAVE state and all loaded environments.
///
/// This functions should be always called before program shutdown in order to assure all
/// resources are relased appropriately.
OPENRAVE_API void RaveDestroy();

/// \brief Add a callback when the OpenRAVE global runtime is destroyed.
///
/// The callback is called after all OpenRAVE environments have been destroyed and
/// before plugins are unloaded.
/// Callback is added only for this run-time. Once the run-time is destroyed/swapped, it will have to be re-added.
/// OpenRAVE runtime is destroyed when \ref RaveDestroy is called or on system exits.
OPENRAVE_API void RaveAddCallbackForDestroy(const boost::function<void()>& fn);

/// \brief Get all the loaded plugins and the interfaces they support.
///
/// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
OPENRAVE_API void RaveGetPluginInfo(std::list< std::pair<std::string, PluginInfo> >& plugins);

/// \brief Get a list of all the loaded interfaces.
OPENRAVE_API void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames);

/// \brief Reloads all the plugins.
///
/// The interfaces currently created remain will continue using the old plugins, so this function is safe in that plugins currently loaded remain loaded until the last interface that uses them is released.
OPENRAVE_API void RaveReloadPlugins();

/// \brief Load a plugin and its interfaces.
///
/// If the plugin is already loaded, will reload it.
/// \param name the filename of the plugin to load
OPENRAVE_API bool RaveLoadPlugin(const std::string& libraryname);

/// \brief Returns true if interface can be created, otherwise false.
OPENRAVE_API bool RaveHasInterface(InterfaceType type, const std::string& interfacename);

OPENRAVE_API InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr env, InterfaceType type,const std::string& interfacename);
OPENRAVE_API RobotBasePtr RaveCreateRobot(EnvironmentBasePtr env, const std::string& name="");
OPENRAVE_API PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ControllerBasePtr RaveCreateController(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API MultiControllerBasePtr RaveCreateMultiController(EnvironmentBasePtr env, const std::string& name="");
OPENRAVE_API ModuleBasePtr RaveCreateModule(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblem(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ModuleBasePtr RaveCreateProblemInstance(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SensorBasePtr RaveCreateSensor(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr env, const std::string& name);
OPENRAVE_API KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr env, const std::string& name="");
/// \brief Return an empty trajectory instance.
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr env, const std::string& name="");

/// \deprecated (11/10/01)
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr env, int dof) RAVE_DEPRECATED;

/// \brief returned a fully cloned interface
///
/// \param preference the InterfaceBasePtr to clone
/// \param cloningoptions combination of CO_*
/// \param pcloneenv the environment to create the new clone in. If not specified, will use preference->GetEnv()
template <typename T>
inline std::shared_ptr<T> RaveClone(std::shared_ptr<T const> preference, int cloningoptions, EnvironmentBasePtr pcloneenv=EnvironmentBasePtr())
{
    InterfaceBasePtr pcloned = RaveCreateInterface(!pcloneenv ? preference->GetEnv() : pcloneenv, preference->GetInterfaceType(), preference->GetXMLId());
    OPENRAVE_ASSERT_FORMAT(!!pcloned, "Failed to clone interface=%s id=%s", RaveGetInterfaceName(preference->GetInterfaceType())%preference->GetXMLId(), ORE_InvalidArguments);
    std::shared_ptr<T> pclonedcast = std::dynamic_pointer_cast<T>(pcloned);
    OPENRAVE_ASSERT_FORMAT(!!pclonedcast, "Interface created but failed to cast interface=%s id=%s", RaveGetInterfaceName(preference->GetInterfaceType())%preference->GetXMLId(), ORE_InvalidArguments);
    pclonedcast->Clone(preference,cloningoptions);
    return pclonedcast;
}

/** \brief Registers a function to create an interface, this allows the interface to be created by other modules.

    \param type interface type
    \param name interface name
    \param interfacehash the hash of the interface being created (use the global defines OPENRAVE_X_HASH)
    \param envhash the hash of the environment (use the global define OPENRAVE_ENVIRONMENT_HASH)
    \param createfn functions to create the interface it takes two parameters: the environment and an istream to the rest of the interface creation arguments.
    \return a handle if function is successfully registered. By destroying the handle, the interface will be automatically unregistered.
    \throw OpenRAVEException Will throw with ORE_InvalidInterfaceHash if hashes do not match
 */
OPENRAVE_API UserDataPtr RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

/** \brief Registers a custom xml reader for a particular interface.

    Once registered, anytime an interface is created through XML and
    the xmltag is seen, the function CreateXMLReaderFn will be called to get a reader for that tag
    \param xmltag the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    \param fn CreateXMLReaderFn(pinterface,atts) - passed in the pointer to the interface where the tag was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
 */
OPENRAVE_API UserDataPtr RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn);

/** \brief Registers a custom json reader for a particular interface.

    Once registered, anytime an interface is created through JSON and
    the id is seen, the function CreateJSONReaderFn will be called to get a reader
    \param id the id specified is seen in the interface, the the custom reader will be created.
    \param fn CreateJSONReaderFn(pinterface,atts) - passed in the pointer to the interface where the id was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
 */
OPENRAVE_API UserDataPtr RaveRegisterJSONReader(InterfaceType type, const std::string& id, const CreateJSONReaderFn& fn);

/// \brief return the environment's unique id, returns 0 if environment could not be found or not registered
OPENRAVE_API int RaveGetEnvironmentId(EnvironmentBaseConstPtr env);

/// \brief get the environment from its unique id
/// \param id the unique environment id returned by \ref RaveGetEnvironmentId
OPENRAVE_API EnvironmentBasePtr RaveGetEnvironment(int id);

/// \brief Return all the created OpenRAVE environments.
OPENRAVE_API void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments);

/// \brief Returns the current registered reader for the interface type/xmlid
///
/// \throw OpenRAVEException Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts);

/// \brief Returns the current registered json reader for the interface type/id
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseJSONReaderPtr RaveCallJSONReader(InterfaceType type, const std::string& id, InterfaceBasePtr pinterface, const AttributesList& atts);

/** \brief Returns the absolute path of the filename on the local filesystem resolving relative paths from OpenRAVE paths.

    The OpenRAVE paths consist of a list of directories specified by $OPENRAVE_DATA environment variable and custom added user paths.
    Requires std::filesystem to be installed
    \param filename the filename to look for
    \param curdir the current directory in case the filename is relative
    \return an empty string if file isn't found, otherwise path to full filename on local filesystem
 */
OPENRAVE_API std::string RaveFindLocalFile(const std::string& filename, const std::string& curdir="");

/** \brief Given the absolute filename, return the relative path from one of the OPENRAVE_DATA directories.

    Will check if filename is inside one of the OPENRAVE_DATA directories, and set newfilename to the relative path.
    \return true if inside a OPENRAVE_DATA directory.
 */
OPENRAVE_API bool RaveInvertFileLookup(std::string& newfilename, const std::string& filename);

/// \brief Sets the default data access options for cad resources/robot files
///
/// Controls how files are processed in functions like \ref RaveFindLocalFile
/// \param accessoptions - if 1 will only allow resources inside directories specified from OPERNAVE_DATA environment variable. This allows reject of full paths from unsecure/unauthenticated resources.
OPENRAVE_API void RaveSetDataAccess(int accessoptions);

/// \brief Returns the acess options set
///
/// \see RaveSetDataAccess
OPENRAVE_API int RaveGetDataAccess();

/// \brief Gets the default viewer type name
OPENRAVE_API std::string RaveGetDefaultViewerType();

//@}

/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API uint32_t RaveRandomInt();
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API float RaveRandomFloat(IntervalType interval=IT_Closed);
/// \deprecated (11/06/03), use \ref SpaceSamplerBase
OPENRAVE_API double RaveRandomDouble(IntervalType interval=IT_Closed);

/// \deprecated (12/02/06) see \ref OpenRAVE::utils::TokenizeString
bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs) RAVE_DEPRECATED;

inline bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs)
{
    vdirs.resize(0);
    if( !pdirs ) {
        return false;
    }
    // search for all directories separated by ':'
    std::string tmp = pdirs;
    std::string::size_type pos = 0, newpos=0;
    while( pos < tmp.size() ) {
#ifdef _WIN32
        newpos = tmp.find(';', pos);
#else
        newpos = tmp.find(':', pos);
#endif
        std::string::size_type n = newpos == std::string::npos ? tmp.size()-pos : (newpos-pos);
        vdirs.push_back(tmp.substr(pos, n));
        if( newpos == std::string::npos ) {
            break;
        }
        pos = newpos+1;
    }
    return true;
}

/// \brief Create the interfaces, see \ref CreateInterfaceValidated.
/// \ingroup plugin_exports
typedef InterfaceBasePtr (*PluginExportFn_OpenRAVECreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, const char* envhash, EnvironmentBasePtr env);

/// \brief Called to fill information about the plugin, see \ref GetPluginAttributesValidated.
/// \ingroup plugin_exports
typedef bool (*PluginExportFn_OpenRAVEGetPluginAttributes)(PluginInfo* pinfo, int size, const char* infohash);

/// \brief Called before plugin is unloaded from openrave. See \ref DestroyPlugin.
/// \ingroup plugin_exports
typedef void (*PluginExportFn_DestroyPlugin)();

/// \brief Called when OpenRAVE global runtime is finished initializing. See \ref OnRaveInitialized
/// \ingroup plugin_exports
typedef void (*PluginExportFn_OnRaveInitialized)();

/// \brief Called when OpenRAVE global runtime is about to be destroyed. See \ref OnRavePreDestroy.
/// \ingroup plugin_exports
typedef void (*PluginExportFn_OnRavePreDestroy)();

/// \deprecated (12/01/01)
typedef InterfaceBasePtr (*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr env) RAVE_DEPRECATED;

/// \deprecated (12/01/01)
typedef bool (*PluginExportFn_GetPluginAttributes)(PluginInfo* pinfo, int size) RAVE_DEPRECATED;



} // end namespace OpenRAVE


BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MAJOR>=0&&OPENRAVE_VERSION_MAJOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MINOR>=0&&OPENRAVE_VERSION_MINOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_PATCH>=0&&OPENRAVE_VERSION_PATCH<=255);


#endif

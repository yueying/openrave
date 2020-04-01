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
#ifndef RAVE_ENVIRONMENT_H
#define RAVE_ENVIRONMENT_H

#include "ravep.h"
#include "colladaparser/colladacommon.h"
#include "jsonparser/jsoncommon.h"

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#include <pcrecpp.h>
#include <regex>

#define CHECK_INTERFACE(pinterface) { \
        if( (pinterface)->GetEnv() != shared_from_this() ) \
            throw OpenRAVEException(str(boost::format(_tr("Interface %s:%s is from a different environment"))%RaveGetInterfaceName((pinterface)->GetInterfaceType())%(pinterface)->GetXMLId()),ORE_InvalidArguments); \
} \

#define CHECK_COLLISION_BODY(body) { \
        CHECK_INTERFACE(body); \
}
//namespace OpenRAVE
//{
class Environment : public EnvironmentBase
{
    class GraphHandleMulti : public GraphHandle
    {
public:
        GraphHandleMulti()
		{
        }
        virtual ~GraphHandleMulti() 
		{
        }
        void SetTransform(const RaveTransform<float>& t)
        {
            for(auto& it:listhandles) 
			{
                it->SetTransform(t);
            }
        }

        void SetShow(bool bshow)
        {
            for(auto& it:listhandles) 
			{
                it->SetShow(bshow);
            }
        }

        void Add(OpenRAVE::GraphHandlePtr phandle)
		{
            if( !!phandle) 
			{
                listhandles.push_back(phandle);
            }
        }

        std::list<OpenRAVE::GraphHandlePtr> listhandles;
    };
    typedef std::shared_ptr<GraphHandleMulti> GraphHandleMultiPtr;

    class CollisionCallbackData : public UserData
    {
public:
        CollisionCallbackData(const CollisionCallbackFn& callback, std::shared_ptr<Environment> penv)
			: _callback(callback), _pweakenv(penv) 
		{
        }

        virtual ~CollisionCallbackData() 
		{
            std::shared_ptr<Environment> penv = _pweakenv.lock();
            if( !!penv ) 
			{
                boost::timed_mutex::scoped_lock lock(penv->mutex_interfaces_);
                penv->_listRegisteredCollisionCallbacks.erase(_iterator);
            }
        }

        std::list<UserDataWeakPtr>::iterator _iterator;
        CollisionCallbackFn _callback;
protected:
        std::weak_ptr<Environment> _pweakenv;
    };
    friend class CollisionCallbackData;
    typedef std::shared_ptr<CollisionCallbackData> CollisionCallbackDataPtr;

    class BodyCallbackData : public UserData
    {
public:
        BodyCallbackData(const BodyCallbackFn& callback, std::shared_ptr<Environment> penv)
			: _callback(callback), _pweakenv(penv) {
        }
        virtual ~BodyCallbackData() 
		{
            std::shared_ptr<Environment> penv = _pweakenv.lock();
            if( !!penv ) 
			{
                boost::timed_mutex::scoped_lock lock(penv->mutex_interfaces_);
                penv->_listRegisteredBodyCallbacks.erase(_iterator);
            }
        }

        std::list<UserDataWeakPtr>::iterator _iterator;
        BodyCallbackFn _callback;
protected:
        std::weak_ptr<Environment> _pweakenv;
    };
    friend class BodyCallbackData;
    typedef std::shared_ptr<BodyCallbackData> BodyCallbackDataPtr;

public:
	Environment();

	virtual ~Environment();

	virtual void Init(bool is_start_simulation_thread = true);

	virtual void Destroy();

	virtual void Reset();

    virtual UserDataPtr GlobalState()
	{
        return RaveGlobalState();
    }

    virtual void OwnInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        _listOwnedInterfaces.push_back(pinterface);
    }
    virtual void DisownInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        _listOwnedInterfaces.remove(pinterface);
    }

    virtual EnvironmentBasePtr CloneSelf(int options)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::shared_ptr<Environment> penv(new Environment());
        penv->_Clone(std::static_pointer_cast<Environment const>(shared_from_this()),options,false);
        return penv;
    }

    virtual void Clone(EnvironmentBaseConstPtr preference, int cloningoptions)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _Clone(std::static_pointer_cast<Environment const>(preference),cloningoptions,true);
    }

    virtual int AddModule(ModuleBasePtr module, const std::string& cmdargs)
    {
        CHECK_INTERFACE(module);
        int ret = module->main(cmdargs);
        if( ret != 0 ) {
            RAVELOG_WARN_FORMAT("Error %d with executing module %s", ret%module->GetXMLId());
        }
        else {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            modules_list_.emplace_back(module,  cmdargs);
        }

        return ret;
    }

    void GetModules(std::list<ModuleBasePtr>& listModules, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            listModules.clear();
            FOREACHC(it, modules_list_) {
                listModules.push_back(it->first);
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            listModules.clear();
            FOREACHC(it, modules_list_) {
                listModules.push_back(it->first);
            }
        }
    }

	virtual bool LoadURI(const std::string& uri, const AttributesList& atts);

	virtual bool Load(const std::string& filename, const AttributesList& atts);

	virtual bool LoadData(const std::string& data, const AttributesList& atts);

	virtual bool LoadJSON(const rapidjson::Document& doc, const AttributesList& atts);

    virtual void Save(const std::string& filename, SelectionOptions options, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            if( _IsJSONFile(filename) ) {
                RaveWriteJSONFile(shared_from_this(),filename,atts);
            }
            else {
                RaveWriteColladaFile(shared_from_this(),filename,atts);
            }
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            FOREACH(itbody,bodies_vector_) {
                if( !(*itbody)->IsRobot() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        case SO_Robots:
            FOREACH(itrobot,robots_vector_) {
                listbodies.push_back(*itrobot);
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            FOREACH(itbody,bodies_vector_) {
                if( find(listignore.begin(),listignore.end(),(*itbody)->GetName()) == listignore.end() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        }
        }

        if( _IsJSONFile(filename) ) {
            if( listbodies.size() == 1 ) {
                RaveWriteJSONFile(listbodies.front(),filename,atts);
            }
            else {
                RaveWriteJSONFile(listbodies,filename,atts);
            }
        }
        else {
            if( listbodies.size() == 1 ) {
                RaveWriteColladaFile(listbodies.front(),filename,atts);
            }
            else {
                RaveWriteColladaFile(listbodies,filename,atts);
            }
        }
    }

    virtual void SaveJSON(rapidjson::Document& doc, SelectionOptions options, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            RaveWriteJSON(shared_from_this(), doc, atts);
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            FOREACH(itbody,bodies_vector_) {
                if( !(*itbody)->IsRobot() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        case SO_Robots:
            FOREACH(itrobot,robots_vector_) {
                listbodies.push_back(*itrobot);
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            FOREACH(itbody,bodies_vector_) {
                if( find(listignore.begin(),listignore.end(),(*itbody)->GetName()) == listignore.end() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        }
        }

        if( listbodies.size() == 1 ) {
            RaveWriteJSON(listbodies.front(), doc, atts);
        }
        else {
            RaveWriteJSON(listbodies, doc, atts);
        }
    }

    virtual void WriteToMemory(const std::string& filetype, std::vector<char>& output, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList())
    {
        if (filetype != "collada" && filetype != "json") {
            throw OPENRAVE_EXCEPTION_FORMAT("got invalid filetype %s, only support collada and json", filetype, ORE_InvalidArguments);
        }

        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            if (filetype == "collada") {
                RaveWriteColladaMemory(shared_from_this(), output, atts);
            }
            else if (filetype == "json") {
                RaveWriteJSONMemory(shared_from_this(), output, atts);
            }
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            FOREACH(itbody,bodies_vector_) {
                if( !(*itbody)->IsRobot() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        case SO_Robots:
            FOREACH(itrobot,robots_vector_) {
                listbodies.push_back(*itrobot);
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            FOREACH(itbody,bodies_vector_) {
                if( find(listignore.begin(),listignore.end(),(*itbody)->GetName()) == listignore.end() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        }
        }

        if( listbodies.size() == 1 ) {
            if (filetype == "collada") {
                RaveWriteColladaMemory(listbodies.front(), output, atts);
            }
            else if (filetype == "json") {
                RaveWriteJSONMemory(listbodies.front(), output, atts);
            }
        }
        else {
            if (filetype == "collada") {
                RaveWriteColladaMemory(listbodies, output, atts);
            }
            else if (filetype == "json") {
                RaveWriteJSONMemory(listbodies, output, atts);
            }
        }
    }

    virtual void Add(InterfaceBasePtr pinterface, bool bAnonymous, const std::string& cmdargs)
    {
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_Robot: _AddRobot(RaveInterfaceCast<RobotBase>(pinterface),bAnonymous); break;
        case PT_KinBody: _AddKinBody(RaveInterfaceCast<KinBody>(pinterface),bAnonymous); break;
        case PT_Module: {
            int ret = AddModule(RaveInterfaceCast<ModuleBase>(pinterface),cmdargs);
            OPENRAVE_ASSERT_OP_FORMAT(ret,==,0,"module %s failed with args: %s",pinterface->GetXMLId()%cmdargs,ORE_InvalidArguments);
            break;
        }
        case PT_Viewer: _AddViewer(RaveInterfaceCast<ViewerBase>(pinterface)); break;
        case PT_Sensor: _AddSensor(RaveInterfaceCast<SensorBase>(pinterface),bAnonymous); break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("Interface %d cannot be added to the environment"),pinterface->GetInterfaceType(),ORE_InvalidArguments);
        }
    }

    virtual void _AddKinBody(KinBodyPtr pbody, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pbody);
        if( !utils::IsValidName(pbody->GetName()) ) {
            throw OpenRAVEException(str(boost::format(_tr("kinbody name: \"%s\" is not valid"))%pbody->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(pbody),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=pbody->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                pbody->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(pbody), false) ) {
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            bodies_vector_.push_back(pbody);
            SetEnvironmentId(pbody);
            bodies_modified_stamp_++;
        }
        pbody->_ComputeInternalInformation();
        current_collision_checker_->InitKinBody(pbody);
        if( !!pbody->GetSelfCollisionChecker() && pbody->GetSelfCollisionChecker() != current_collision_checker_ ) {
            // also initialize external collision checker if specified for this body
            pbody->GetSelfCollisionChecker()->InitKinBody(pbody);
        }
        physics_engine_->InitKinBody(pbody);
        // send all the changed callbacks of the body since anything could have changed
        pbody->_PostprocessChangedParameters(0xffffffff&~KinBody::Prop_JointMimic&~KinBody::Prop_LinkStatic&~KinBody::Prop_BodyRemoved);
        _CallBodyCallbacks(pbody, 1);
    }

    virtual void _AddRobot(RobotBasePtr robot, bool is_anonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(robot);
        if( !robot->IsRobot() )
		{
            throw OpenRAVEException(str(boost::format(_tr("kinbody \"%s\" is not a robot"))%robot->GetName()));
        }
        if( !utils::IsValidName(robot->GetName()) ) 
		{
            throw OpenRAVEException(str(boost::format(_tr("kinbody name: \"%s\" is not valid"))%robot->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(robot),!is_anonymous) ) 
		{
            // continue to add random numbers until a unique name is found
            std::string oldname=robot->GetName(),newname;
            for(int i = 0;; ++i)
			{
                newname = str(boost::format("%s%d")%oldname%i);
                robot->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(robot),false) )
				{
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            bodies_vector_.push_back(robot);
            robots_vector_.push_back(robot);
            SetEnvironmentId(robot);
            bodies_modified_stamp_++;
        }
        robot->_ComputeInternalInformation(); // have to do this after robots_vector_ is added since SensorBase::SetName can call EnvironmentBase::GetSensor to initialize itself
        current_collision_checker_->InitKinBody(robot);
        if( !!robot->GetSelfCollisionChecker() && robot->GetSelfCollisionChecker() != current_collision_checker_ ) 
		{
            // also initialize external collision checker if specified for this body
            robot->GetSelfCollisionChecker()->InitKinBody(robot);
        }
        physics_engine_->InitKinBody(robot);
        // send all the changed callbacks of the body since anything could have changed
        robot->_PostprocessChangedParameters(
			0xffffffff
			&~KinBody::Prop_JointMimic
			&~KinBody::Prop_LinkStatic
			&~KinBody::Prop_BodyRemoved);
        _CallBodyCallbacks(robot, 1);
    }

    virtual void _AddSensor(SensorBasePtr psensor, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(psensor);
        if( !utils::IsValidName(psensor->GetName()) ) {
            throw OpenRAVEException(str(boost::format(_tr("sensor name: \"%s\" is not valid"))%psensor->GetName()));
        }
        if( !_CheckUniqueName(SensorBaseConstPtr(psensor),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=psensor->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                psensor->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(SensorBaseConstPtr(psensor),false) ) {
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            sensors_list_.push_back(psensor);
        }
        psensor->Configure(SensorBase::CC_PowerOn);
    }

    virtual bool Remove(InterfaceBasePtr pinterface)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_KinBody:
        case PT_Robot: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
            {
                boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
                vector<KinBodyPtr>::iterator it = std::find(bodies_vector_.begin(), bodies_vector_.end(), pbody);
                if( it == bodies_vector_.end() ) {
                    return false;
                }
                _RemoveKinBodyFromIterator(it);
            }
            // pbody is valid so run any callbacks and exit
            _CallBodyCallbacks(pbody, 0);
            return true;
        }
        case PT_Sensor: {
            SensorBasePtr psensor = RaveInterfaceCast<SensorBase>(pinterface);
            list<SensorBasePtr>::iterator it = std::find(sensors_list_.begin(), sensors_list_.end(), psensor);
            if( it != sensors_list_.end() ) {
                (*it)->Configure(SensorBase::CC_PowerOff);
                sensors_list_.erase(it);
                return true;
            }
            break;
        }
        case PT_Module: {
            ModuleBasePtr pmodule = RaveInterfaceCast<ModuleBase>(pinterface);
            FOREACH(itmodule, modules_list_) {
                if( itmodule->first == pmodule ) {
                    itmodule->first->Destroy();
                    modules_list_.erase(itmodule);
                    return true;
                }
            }
            break;
        }
        case PT_Viewer: {
            ViewerBasePtr pviewer = RaveInterfaceCast<ViewerBase>(pinterface);
            list<ViewerBasePtr>::iterator itviewer = find(viewers_list_.begin(), viewers_list_.end(), pviewer);
            if( itviewer != viewers_list_.end() ) {
                (*itviewer)->quitmainloop();
                viewers_list_.erase(itviewer);
                return true;
            }
            break;
        }
        default:
            RAVELOG_WARN_FORMAT("unmanaged interfaces of type %s cannot be removed", RaveGetInterfaceName(pinterface->GetInterfaceType()));
            break;
        }
        return false;
    }

    virtual bool RemoveKinBodyByName(const std::string& name)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        KinBodyPtr pbody;
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            vector<KinBodyPtr>::iterator it = bodies_vector_.end();
            FOREACHC(itbody, bodies_vector_) {
                if( (*itbody)->GetName() == name ) {
                    it = itbody;
                    break;
                }
            }
            if( it == bodies_vector_.end() ) {
                return false;
            }
            pbody = *it;
            _RemoveKinBodyFromIterator(it);
        }
        // pbody is valid so run any callbacks and exit
        _CallBodyCallbacks(pbody, 0);
        return true;
    }

    virtual UserDataPtr RegisterBodyCallback(const BodyCallbackFn& callback)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        BodyCallbackDataPtr pdata(new BodyCallbackData(callback,std::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredBodyCallbacks.insert(_listRegisteredBodyCallbacks.end(),pdata);
        return pdata;
    }

    virtual KinBodyPtr GetKinBody(const std::string& pname) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        FOREACHC(it, bodies_vector_) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        return KinBodyPtr();
    }

    virtual RobotBasePtr GetRobot(const std::string& pname) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        FOREACHC(it, robots_vector_) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        return RobotBasePtr();
    }

    virtual SensorBasePtr GetSensor(const std::string& name) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        FOREACHC(itrobot,robots_vector_) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor &&( psensor->GetName() == name) ) {
                    return psensor;
                }
            }
        }
        FOREACHC(itsensor,sensors_list_) {
            if( (*itsensor)->GetName() == name ) {
                return *itsensor;
            }
        }
        return SensorBasePtr();
    }

    virtual bool SetPhysicsEngine(PhysicsEngineBasePtr pengine)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( !!physics_engine_ ) {
            physics_engine_->DestroyEnvironment();
        }
        physics_engine_ = pengine;
        if( !physics_engine_ ) {
            RAVELOG_DEBUG_FORMAT("env %d, disabling physics for", GetId());
            physics_engine_ = RaveCreatePhysicsEngine(shared_from_this(),"GenericPhysicsEngine");
            _SetDefaultGravity();
        }
        else {
            RAVELOG_DEBUG_FORMAT("setting %s physics engine", physics_engine_->GetXMLId());
        }
        physics_engine_->InitEnvironment();
        return true;
    }

    virtual PhysicsEngineBasePtr GetPhysicsEngine() const {
        return physics_engine_;
    }

    virtual UserDataPtr RegisterCollisionCallback(const CollisionCallbackFn& callback)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        CollisionCallbackDataPtr pdata(new CollisionCallbackData(callback,std::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredCollisionCallbacks.insert(_listRegisteredCollisionCallbacks.end(),pdata);
        return pdata;
    }
    virtual bool HasRegisteredCollisionCallbacks() const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        return _listRegisteredCollisionCallbacks.size() > 0;
    }

    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>& listcallbacks) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        listcallbacks.clear();
        FOREACHC(it, _listRegisteredCollisionCallbacks) {
            CollisionCallbackDataPtr pdata = std::dynamic_pointer_cast<CollisionCallbackData>(it->lock());
            listcallbacks.push_back(pdata->_callback);
        }
    }

    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( current_collision_checker_ == pchecker ) {
            return true;
        }
        if( !!current_collision_checker_ ) {
            current_collision_checker_->DestroyEnvironment();     // delete all resources
        }
        current_collision_checker_ = pchecker;
        if( !current_collision_checker_ ) {
            RAVELOG_DEBUG("disabling collisions\n");
            current_collision_checker_ = RaveCreateCollisionChecker(shared_from_this(),"GenericCollisionChecker");
        }
        else {
            RAVELOG_DEBUG_FORMAT("setting %s collision checker", current_collision_checker_->GetXMLId());
            FOREACH(itbody,bodies_vector_) {
                (*itbody)->_ResetInternalCollisionCache();
            }
        }
        return current_collision_checker_->InitEnvironment();
    }

    virtual CollisionCheckerBasePtr GetCollisionChecker() const {
        return current_collision_checker_;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        return current_collision_checker_->CheckCollision(pbody1,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        CHECK_COLLISION_BODY(pbody2);
        return current_collision_checker_->CheckCollision(pbody1,pbody2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report )
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return current_collision_checker_->CheckCollision(plink,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink1->GetParent());
        CHECK_COLLISION_BODY(plink2->GetParent());
        return current_collision_checker_->CheckCollision(plink1,plink2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        CHECK_COLLISION_BODY(pbody);
        return current_collision_checker_->CheckCollision(plink,pbody,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return current_collision_checker_->CheckCollision(plink,vbodyexcluded,vlinkexcluded,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return current_collision_checker_->CheckCollision(pbody,vbodyexcluded,vlinkexcluded,report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return current_collision_checker_->CheckCollision(ray,plink,report);
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return current_collision_checker_->CheckCollision(ray,pbody,report);
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report)
    {
        return current_collision_checker_->CheckCollision(ray,report);
    }

    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return current_collision_checker_->CheckCollision(trimesh,pbody,report);
    }

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return current_collision_checker_->CheckStandaloneSelfCollision(pbody,report);
    }

    virtual void StepSimulation(dReal time_step)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        uint64_t step = (uint64_t)ceil(1000000.0 * (double)time_step);
        time_step = (dReal)((double)step * 0.000001);

        // call the physics first to get forces
        physics_engine_->SimulateStep(time_step);

        // make a copy instead of locking the mutex pointer since will be calling into user functions
        std::vector<KinBodyPtr> bodies_vector;
        std::vector<RobotBasePtr> robots_vector;
        std::list<SensorBasePtr> sensors_list;
        std::list<std::pair<ModuleBasePtr, std::string>> modules_list;
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            bodies_vector = bodies_vector_;
            robots_vector = robots_vector_;
            sensors_list = sensors_list_;
            modules_list = modules_list_;
        }

        for(auto& it: bodies_vector)
		{
            if( it->GetEnvironmentId() ) 
			{     // have to check if valid
                it->SimulationStep(time_step);
            }
        }

        for(auto& itmodule: modules_list) 
		{
            itmodule.first->SimulationStep(time_step);
        }

        // simulate the sensors last (ie, they always reflect the most recent bodies
        for(auto& itsensor: sensors_list) 
		{
            itsensor->SimulationStep(time_step);
        }

        for(auto& itrobot: robots_vector)
		{
            for(auto& itsensor: itrobot->GetAttachedSensors())
			{
                if( !!itsensor->GetSensor() ) 
				{
                    itsensor->GetSensor()->SimulationStep(time_step);
                }
            }
        }
        cur_simulation_time_ += step;
    }

    virtual EnvironmentMutex& GetMutex() const {
        return _mutexEnvironment;
    }

    virtual void GetBodies(std::vector<KinBodyPtr>& bodies, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            bodies = bodies_vector_;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            bodies = bodies_vector_;
        }
    }

    virtual void GetRobots(std::vector<RobotBasePtr>& robots, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            robots = robots_vector_;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            robots = robots_vector_;
        }
    }

    virtual void GetSensors(std::vector<SensorBasePtr>& vsensors, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            _GetSensors(vsensors);
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            _GetSensors(vsensors);
        }
    }

    virtual void _GetSensors(std::vector<SensorBasePtr>& vsensors) const
    {
        vsensors.resize(0);
        FOREACHC(itrobot,robots_vector_) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor ) {
                    vsensors.push_back(psensor);
                }
            }
        }
    }

    virtual void Triangulate(TriMesh& trimesh, const KinBody &body)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());     // reading collision data, so don't want anyone modifying it
        FOREACHC(it, body.GetLinks()) {
            trimesh.Append((*it)->GetCollisionData(), (*it)->GetTransform());
        }
    }

    virtual void TriangulateScene(TriMesh& trimesh, SelectionOptions options,const std::string& selectname)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        FOREACH(itbody, bodies_vector_) {
            RobotBasePtr robot;
            if( (*itbody)->IsRobot() ) {
                robot = RaveInterfaceCast<RobotBase>(*itbody);
            }
            switch(options) {
            case SO_NoRobots:
                if( !robot ) {
                    Triangulate(trimesh, **itbody);
                }
                break;

            case SO_Robots:
                if( !!robot ) {
                    Triangulate(trimesh, **itbody);
                }
                break;
            case SO_Everything:
                Triangulate(trimesh, **itbody);
                break;
            case SO_Body:
                if( (*itbody)->GetName() == selectname ) {
                    Triangulate(trimesh, **itbody);
                }
                break;
            case SO_AllExceptBody:
                if( (*itbody)->GetName() != selectname ) {
                    Triangulate(trimesh, **itbody);
                }
                break;
//            case SO_BodyList:
//                if( find(listnames.begin(),listnames.end(),(*itbody)->GetName()) != listnames.end() ) {
//                    Triangulate(trimesh,*itbody);
//                }
            }
        }
    }

    virtual void TriangulateScene(TriMesh& trimesh, TriangulateOptions options)
    {
        TriangulateScene(trimesh,options,"");
    }

    virtual RobotBasePtr ReadRobotURI(RobotBasePtr robot, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) 
		{
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            for(auto& itviewer: viewers_list_)
			{
                itviewer->RemoveKinBody(robot);
            }
            if( std::find(robots_vector_.begin(),robots_vector_.end(),robot) != robots_vector_.end() )
			{
                throw OpenRAVEException(str(boost::format(_tr("KinRobot::Init for %s,\
                cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        if( _IsColladaURI(filename) )
		{
            if( !RaveParseColladaURI(shared_from_this(), robot, filename, atts) )
			{
                return RobotBasePtr();
            }
        }
        else if( _IsJSONURI(filename) )
		{
            if( !RaveParseJSONURI(shared_from_this(), robot, filename, atts) ) 
			{
                return RobotBasePtr();
            }
        }
        else if( _IsColladaFile(filename) ) 
		{
            if( !RaveParseColladaFile(shared_from_this(), robot, filename, atts) )
			{
                return RobotBasePtr();
            }
        }
        else if( _IsJSONFile(filename) ) 
		{
            if( !RaveParseJSONFile(shared_from_this(), robot, filename, atts) )
			{
                return RobotBasePtr();
            }
        }
        else if( _IsXFile(filename) ) 
		{
            if( !RaveParseXFile(shared_from_this(), robot, filename, atts) ) 
			{
                return RobotBasePtr();
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) 
		{
            if( !robot )
			{
                robot = RaveCreateRobot(shared_from_this(),"GenericRobot");
            }
            if( !robot ) 
			{
                robot = RaveCreateRobot(shared_from_this(),"");
            }
            if( !!robot )
			{
                std::list<KinBody::GeometryInfo> geometries_list;
                std::string fullfilename = _ReadGeometriesFile(filename,atts, geometries_list);
                if( fullfilename.size() > 0 ) 
				{
                    std::string extension;
                    if( filename.find_last_of('.') != std::string::npos ) 
					{
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
					std::string norender = std::string("__norenderif__:")+extension;
                    for(auto& itinfo:geometries_list) 
					{
                        itinfo.is_visible_ = true;
                        itinfo.render_file_name_ = norender;
                    }
                    geometries_list.front().render_file_name_ = fullfilename;
                    if( robot->InitFromGeometries(geometries_list) )
					{
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
                        boost::filesystem::path pfilename(filename);
                        robot->SetName(utils::ConvertToOpenRAVEName(pfilename.stem().string()));
#else
                        boost::filesystem::path pfilename(filename, boost::filesystem::native);
                        robot->SetName(utils::ConvertToOpenRAVEName(pfilename.stem()));
#endif
#else
                        robot->SetName("object");
#endif
                        return robot;
                    }
                }
                robot.reset();
            }
        }
        else 
		{
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), 
				PT_Robot, pinterface, "robot", atts);
            if( !preader ) 
			{
                return RobotBasePtr();
            }
            bool is_success = _ParseXMLFile(preader, filename);
            preader->endElement("robot");     // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !is_success || !robot ) 
			{
                return RobotBasePtr();
            }
            robot->str_uri_ = filename;
        }

        return robot;
    }

    virtual RobotBasePtr ReadRobotData(RobotBasePtr robot, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            FOREACH(itviewer, viewers_list_) {
                (*itviewer)->RemoveKinBody(robot);
            }
            if( std::find(robots_vector_.begin(),robots_vector_.end(),robot) != robots_vector_.end() ) {
                throw OpenRAVEException(str(boost::format(_tr("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            if( !RaveParseJSONData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsXData(data) ) {
            // have to copy since it takes vector<char>
            std::vector<char> newdata(data.size()+1, 0);  // need a null-terminator
            std::copy(data.begin(),data.end(),newdata.begin());
            if( !RaveParseXData(shared_from_this(), robot, newdata, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsIVData(data) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_tr("iv data not supported"),ORE_InvalidArguments);
        }
        else {
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_Robot, pinterface, "robot", atts);
            if( !preader ) {
                return RobotBasePtr();
            }
            bool is_success = _ParseXMLData(preader, data);
            preader->endElement("robot");     // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !is_success || !robot ) {
                return RobotBasePtr();
            }
            robot->str_uri_ = preader->file_name_;
        }

        if( !!robot ) {
            // check if have to reset the URI
            FOREACHC(itatt, atts) {
                if( itatt->first == "uri" ) {
                    robot->str_uri_ = itatt->second;
                }
            }
        }

        return robot;
    }

    virtual KinBodyPtr ReadKinBodyURI(KinBodyPtr body, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body )
		{
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            FOREACH(itviewer, viewers_list_) 
			{
                (*itviewer)->RemoveKinBody(body);
            }
            if( std::find(bodies_vector_.begin(),bodies_vector_.end(),body) != bodies_vector_.end() )
			{
                throw OpenRAVEException(str(boost::format(_tr("KinBody::Init for %s, \
                   cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        if( _IsColladaURI(filename) ) 
		{
            if( !RaveParseColladaURI(shared_from_this(), body, filename, atts) )
			{
                return KinBodyPtr();
            }
        }
        else if( _IsJSONURI(filename) )
		{
            if( !RaveParseJSONURI(shared_from_this(), body, filename, atts) )
			{
                return KinBodyPtr();
            }
        }
        else if( _IsColladaFile(filename) )
		{
            if( !RaveParseColladaFile(shared_from_this(), body, filename, atts) ) 
			{
                return KinBodyPtr();
            }
        }
        else if( _IsJSONFile(filename) ) 
		{
            if( !RaveParseJSONFile(shared_from_this(), body, filename, atts) )
			{
                return KinBodyPtr();
            }
        }
        else if( _IsXFile(filename) )
		{
            if( !RaveParseXFile(shared_from_this(), body, filename, atts) ) 
			{
                return KinBodyPtr();
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) 
		{
            if( !body ) 
			{
                body = RaveCreateKinBody(shared_from_this(),"");
            }
            if( !!body )
			{
                std::list<KinBody::GeometryInfo> geometries_list;
                std::string fullfilename = _ReadGeometriesFile(filename,atts, geometries_list);
                if( fullfilename.size() > 0 ) 
				{
                    std::string extension;
                    if( filename.find_last_of('.') != std::string::npos ) 
					{
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
					std::string norender = std::string("__norenderif__:")+extension;
                    for(auto& itinfo:geometries_list) 
					{
                        itinfo.is_visible_ = true;
                        itinfo.render_file_name_ = norender;
                    }
                    geometries_list.front().render_file_name_ = fullfilename;
                    if( body->InitFromGeometries(geometries_list) ) 
					{
                        body->str_uri_ = fullfilename;
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
                        boost::filesystem::path pfilename(filename);
                        body->SetName(utils::ConvertToOpenRAVEName(pfilename.stem().string()));
#else
                        boost::filesystem::path pfilename(filename, boost::filesystem::native);
                        body->SetName(utils::ConvertToOpenRAVEName(pfilename.stem()));
#endif
#else
                        body->SetName("object");
#endif
                        return body;
                    }
                }
                body.reset();
            }
        }
        else
		{
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),
				PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) 
			{
                return KinBodyPtr();
            }
            bool is_success = _ParseXMLFile(preader, filename);
            preader->endElement("kinbody");     // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !is_success || !body ) 
			{
                return KinBodyPtr();
            }
            body->str_uri_ = filename;
        }

        return body;
    }

    virtual KinBodyPtr ReadKinBodyData(KinBodyPtr body, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            FOREACH(itviewer, viewers_list_) {
                (*itviewer)->RemoveKinBody(body);
            }
            if( std::find(bodies_vector_.begin(),bodies_vector_.end(),body) != bodies_vector_.end() ) {
                throw OpenRAVEException(str(boost::format(_tr("KinBody::Init for %s, cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            if( !RaveParseJSONData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsXData(data) ) {
            // have to copy since it takes vector<char>
            std::vector<char> newdata(data.size()+1, 0);  // need a null-terminator
            std::copy(data.begin(),data.end(),newdata.begin());
            if( !RaveParseXData(shared_from_this(), body, newdata, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsIVData(data) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_tr("iv data not supported"),ORE_InvalidArguments);
        }
        else {
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) {
                return KinBodyPtr();
            }
            bool is_success = _ParseXMLData(preader, data);
            preader->endElement("kinbody");     // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !is_success || !body ) {
                return KinBodyPtr();
            }
            body->str_uri_ = preader->file_name_;
        }

        if( !!body ) {
            // check if have to reset the URI
            FOREACHC(itatt, atts) {
                if( itatt->first == "uri" ) {
                    body->str_uri_ = itatt->second;
                }
            }
        }
        return body;
    }

    virtual InterfaceBasePtr ReadInterfaceURI(const std::string& filename, const AttributesList& atts)
    {
        try {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),atts,false);
            if( !preader ) {
                return InterfaceBasePtr();
            }
            bool is_success = _ParseXMLFile(preader, filename);
            std::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = std::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !is_success || !preadable || !preadable->_pinterface) {
                return InterfaceBasePtr();
            }
            preader->endElement(RaveGetInterfaceName(preadable->_pinterface->GetInterfaceType()));     // have to end the tag!
            preadable->_pinterface->str_uri_ = filename;
            return preadable->_pinterface;
        }
        catch(const std::exception &ex) {
            RAVELOG_ERROR_FORMAT("ReadInterfaceXMLFile exception: %s", ex.what());
        }
        return InterfaceBasePtr();
    }

    virtual InterfaceBasePtr ReadInterfaceURI(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        bool bIsColladaURI = false;
        bool bIsColladaFile = false;
        bool bIsJSONURI = false;
        bool bIsJSONFile = false;
        bool bIsXFile = false;
        if( _IsColladaURI(filename) ) {
            bIsColladaURI = true;
        }
        else if( _IsJSONURI(filename) ) {
            bIsJSONURI = true;
        }
        else if( _IsColladaFile(filename) ) {
            bIsColladaFile = true;
        }
        else if( _IsJSONFile(filename) ) {
            bIsJSONFile = true;
        }
        else if( _IsXFile(filename) ) {
            bIsXFile = true;
        }

        if( (type == PT_KinBody ||type == PT_Robot ) && (bIsColladaURI||bIsJSONURI||bIsColladaFile||bIsJSONFile||bIsXFile) ) {
            if( type == PT_KinBody ) {
                BOOST_ASSERT(!pinterface|| (pinterface->GetInterfaceType()==PT_KinBody||pinterface->GetInterfaceType()==PT_Robot));
                KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
                if( bIsColladaURI ) {
                    if( !RaveParseColladaURI(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONURI ) {
                    if( !RaveParseJSONURI(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsColladaFile ) {
                    if( !RaveParseColladaFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONFile ) {
                    if( !RaveParseJSONFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsXFile ) {
                    if( !RaveParseXFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                pinterface = pbody;
            }
            else if( type == PT_Robot ) {
                BOOST_ASSERT(!pinterface||pinterface->GetInterfaceType()==PT_Robot);
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
                if( bIsColladaURI ) {
                    if( !RaveParseColladaURI(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONURI ) {
                    if( !RaveParseJSONURI(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsColladaFile ) {
                    if( !RaveParseColladaFile(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONFile ) {
                    if( !RaveParseJSONFile(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsXFile ) {
                    if( !RaveParseXFile(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                pinterface = probot;
            }
            else {
                return InterfaceBasePtr();
            }
            pinterface->str_uri_ = filename;
        }
        else {
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
            std::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = std::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !!preadable ) {
                if( !_ParseXMLFile(preader, filename) ) {
                    return InterfaceBasePtr();
                }
                preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType()));     // have to end the tag!
                pinterface = preadable->_pinterface;
            }
            else {
                pinterface = ReadInterfaceURI(filename,AttributesList());
                if( !!pinterface &&( pinterface->GetInterfaceType() != type) ) {
                    return InterfaceBasePtr();
                }
            }
            pinterface->str_uri_ = filename;
        }
        return pinterface;
    }

    virtual InterfaceBasePtr ReadInterfaceData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        // check for collada?
        BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
        if( !preader ) {
            return InterfaceBasePtr();
        }
        bool is_success = _ParseXMLData(preader, data);
        preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType()));     // have to end the tag!
        if( !is_success ) {
            return InterfaceBasePtr();
        }
        pinterface->str_uri_ = preader->file_name_;
        return pinterface;
    }

    virtual std::shared_ptr<TriMesh> ReadTrimeshURI(std::shared_ptr<TriMesh> ptrimesh, const std::string& filename, const AttributesList& atts)
    {
        RaveVector<float> diffuseColor, ambientColor;
        return _ReadTrimeshURI(ptrimesh,filename,diffuseColor, ambientColor, atts);
    }

    virtual std::shared_ptr<TriMesh> _ReadTrimeshURI(std::shared_ptr<TriMesh> ptrimesh, const std::string& filename, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, const AttributesList& atts)
    {
        //EnvironmentMutex::scoped_lock lockenv(GetMutex()); // don't lock!
        string filedata = RaveFindLocalFile(filename);
        if( filedata.size() == 0 ) {
            return std::shared_ptr<TriMesh>();
        }
        Vector vScaleGeometry(1,1,1);
        float ftransparency;
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                ss >> vScaleGeometry.x >> vScaleGeometry.y >> vScaleGeometry.z;
                if( !ss ) {
                    vScaleGeometry.z = vScaleGeometry.y = vScaleGeometry.x;
                }
            }
        }
        if( !ptrimesh ) {
            ptrimesh.reset(new TriMesh());
        }
        if( !OpenRAVEXMLParser::CreateTriMeshFromFile(shared_from_this(),filedata, vScaleGeometry, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
            ptrimesh.reset();
        }
        return ptrimesh;
    }

    virtual std::shared_ptr<TriMesh> ReadTrimeshData(std::shared_ptr<TriMesh> ptrimesh, const std::string& data, const std::string& formathint, const AttributesList& atts)
    {
        RaveVector<float> diffuseColor, ambientColor;
        return _ReadTrimeshData(ptrimesh, data, formathint, diffuseColor, ambientColor, atts);
    }

    virtual std::shared_ptr<TriMesh> _ReadTrimeshData(std::shared_ptr<TriMesh> ptrimesh, const std::string& data, const std::string& formathint, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, const AttributesList& atts)
    {
        if( data.size() == 0 ) {
            return std::shared_ptr<TriMesh>();
        }

        Vector vScaleGeometry(1,1,1);
        float ftransparency;
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                ss >> vScaleGeometry.x >> vScaleGeometry.y >> vScaleGeometry.z;
                if( !ss ) {
                    vScaleGeometry.z = vScaleGeometry.y = vScaleGeometry.x;
                }
            }
        }
        if( !ptrimesh ) {
            ptrimesh.reset(new TriMesh());
        }
        if( !OpenRAVEXMLParser::CreateTriMeshFromData(data, formathint, vScaleGeometry, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
            ptrimesh.reset();
        }
        return ptrimesh;
    }

    /// \brief parses the file into GeometryInfo and returns the full path of the file opened
    ///
    /// \param[in] geometries_list geometry list to be filled
    virtual std::string _ReadGeometriesFile(const std::string& filename,
		const AttributesList& atts, std::list<KinBody::GeometryInfo>& geometries_list)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::string filedata = RaveFindLocalFile(filename);
        if( filedata.size() == 0 ) 
		{
            return std::string();
        }
        Vector vScaleGeometry(1,1,1);
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                ss >> vScaleGeometry.x >> vScaleGeometry.y >> vScaleGeometry.z;
                if( !ss ) {
                    vScaleGeometry.z = vScaleGeometry.y = vScaleGeometry.x;
                }
            }
        }
        if( OpenRAVEXMLParser::CreateGeometries(shared_from_this(),filedata, vScaleGeometry, geometries_list) && geometries_list.size() > 0 ) {
            return filedata;
        }
        geometries_list.clear();
        return std::string();
    }

    virtual void _AddViewer(ViewerBasePtr pnewviewer)
    {
        CHECK_INTERFACE(pnewviewer);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        BOOST_ASSERT(find(viewers_list_.begin(),viewers_list_.end(),pnewviewer) == viewers_list_.end() );
        _CheckUniqueName(ViewerBaseConstPtr(pnewviewer),true);
        viewers_list_.push_back(pnewviewer);
    }

    virtual ViewerBasePtr GetViewer(const std::string& name) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( name.size() == 0 ) {
            return viewers_list_.size() > 0 ? viewers_list_.front() : ViewerBasePtr();
        }
        FOREACHC(itviewer, viewers_list_) {
            if( (*itviewer)->GetName() == name ) {
                return *itviewer;
            }
        }
        return ViewerBasePtr();
    }

    void GetViewers(std::list<ViewerBasePtr>& listViewers) const
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        listViewers = viewers_list_;
    }

    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->plot3(ppoints, numPoints, stride, fPointSize, color, drawstyle));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->plot3(ppoints, numPoints, stride, fPointSize, colors, drawstyle, bhasalpha));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawlinestrip(ppoints, numPoints, stride, fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawlinestrip(ppoints, numPoints, stride, fwidth,colors));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawlinelist(ppoints, numPoints, stride, fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawlinelist(ppoints, numPoints, stride, fwidth,colors));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawarrow(p1,p2,fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawbox(vpos, vextents));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawplane(tplane, vextents, vtexture));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawtrimesh(ppoints, stride, pIndices, numTriangles, color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        if( viewers_list_.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, viewers_list_) {
            handles->Add((*itviewer)->drawtrimesh(ppoints, stride, pIndices, numTriangles, colors));
        }
        return handles;
    }

    virtual KinBodyPtr GetBodyFromEnvironmentId(int id)
    {
        boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
        boost::mutex::scoped_lock locknetwork(_mutexEnvironmentIds);
        map<int, KinBodyWeakPtr>::iterator it = _mapBodies.find(id);
        if( it != _mapBodies.end() ) {
            return KinBodyPtr(it->second);
        }
        return KinBodyPtr();
    }

    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime)
    {
        {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            is_enable_simulation_ = true;
            delta_simulation_time_ = fDeltaTime;
            is_real_time_ = bRealTime;
            //cur_simulation_time_ = 0; // don't reset since it is important to keep time monotonic
            simulation_start_time_ = utils::GetMicroTime()-cur_simulation_time_;
        }
        _StartSimulationThread();
    }

    virtual bool IsSimulationRunning() const {
        return is_enable_simulation_;
    }

    virtual void StopSimulation(int shutdownthread=1)
    {
        {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            is_enable_simulation_ = false;
            delta_simulation_time_ = 1.0f;
        }
        if( shutdownthread ) {
            _StopSimulationThread();
        }
    }

    virtual uint64_t GetSimulationTime() {
        return cur_simulation_time_;
    }

    virtual void SetDebugLevel(int level) {
        RaveSetDebugLevel(level);
    }
    virtual int GetDebugLevel() const {
        return RaveGetDebugLevel();
    }

    virtual void GetPublishedBodies(std::vector<KinBody::BodyState>& vbodies, uint64_t timeout)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            vbodies = published_bodies_vector_;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            vbodies = published_bodies_vector_;
        }
    }

    virtual bool GetPublishedBody(const std::string &name, KinBody::BodyState& bodystate, uint64_t timeout=0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( published_bodies_vector_[ibody].strname == name) {
                    bodystate = published_bodies_vector_[ibody];
                    return true;
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( published_bodies_vector_[ibody].strname == name) {
                    bodystate = published_bodies_vector_[ibody];
                    return true;
                }
            }
        }

        return false;
    }

    virtual bool GetPublishedBodyJointValues(const std::string& name, std::vector<dReal> &jointValues, uint64_t timeout=0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( published_bodies_vector_[ibody].strname == name) {
                    jointValues = published_bodies_vector_[ibody].jointvalues;
                    return true;
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( published_bodies_vector_[ibody].strname == name) {
                    jointValues = published_bodies_vector_[ibody].jointvalues;
                    return true;
                }
            }
        }

        return false;
    }

    void GetPublishedBodyTransformsMatchingPrefix(const std::string& prefix, std::vector<std::pair<std::string, Transform> >& nameTransfPairs, uint64_t timeout = 0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            nameTransfPairs.resize(0);
            if( nameTransfPairs.capacity() < published_bodies_vector_.size() ) {
                nameTransfPairs.reserve(published_bodies_vector_.size());
            }
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( strncmp(published_bodies_vector_[ibody].strname.c_str(), prefix.c_str(), prefix.size()) == 0 ) {
                    nameTransfPairs.emplace_back(published_bodies_vector_[ibody].strname,  published_bodies_vector_[ibody].vectrans.at(0));
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }

            nameTransfPairs.resize(0);
            if( nameTransfPairs.capacity() < published_bodies_vector_.size() ) {
                nameTransfPairs.reserve(published_bodies_vector_.size());
            }
            for ( size_t ibody = 0; ibody < published_bodies_vector_.size(); ++ibody) {
                if ( strncmp(published_bodies_vector_[ibody].strname.c_str(), prefix.c_str(), prefix.size()) == 0 ) {
                    nameTransfPairs.emplace_back(published_bodies_vector_[ibody].strname,  published_bodies_vector_[ibody].vectrans.at(0));
                }
            }
        }
    }

    virtual void UpdatePublishedBodies(uint64_t timeout=0)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( timeout == 0 )
		{
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            _UpdatePublishedBodies();
        }
        else 
		{
            boost::timed_mutex::scoped_timed_lock lock(mutex_interfaces_, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) 
			{
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            _UpdatePublishedBodies();
        }
    }

    virtual void _UpdatePublishedBodies()
    {
        // updated the published bodies, resize dynamically in case an exception occurs
        // when creating an item and bad data is left inside published_bodies_vector_
        published_bodies_vector_.resize(0);
        if( published_bodies_vector_.capacity() < bodies_vector_.size() ) 
		{
            published_bodies_vector_.reserve(bodies_vector_.size());
        }

        std::vector<dReal> vdoflastsetvalues;
        for(auto& itbody: bodies_vector_) 
		{
            if( itbody->hierarchy_computed_ != 2 )
			{
                // skip
                continue;
            }

            published_bodies_vector_.push_back(KinBody::BodyState());
            KinBody::BodyState& state = published_bodies_vector_.back();
            state.pbody = itbody;
            itbody->GetLinkTransformations(state.vectrans, vdoflastsetvalues);
            itbody->GetLinkEnableStates(state.vLinkEnableStates);
            itbody->GetDOFValues(state.jointvalues);
            state.strname =itbody->GetName();
            state.uri = itbody->GetURI();
            state.updatestamp = itbody->GetUpdateStamp();
            state.environmentid = itbody->GetEnvironmentId();
            if( itbody->IsRobot() ) 
			{
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(itbody);
                if( !!probot ) 
				{
                    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
                    if( !!pmanip ) 
					{
                        state.activeManipulatorName = pmanip->GetName();
                        state.activeManipulatorTransform = pmanip->GetTransform();
                    }
                    probot->GetConnectedBodyActiveStates(state.vConnectedBodyActiveStates);
                }
            }
            published_bodies_vector_.push_back(state);
        }
    }

    virtual std::pair<std::string, dReal> GetUnit() const
    {
        return unit_;
    }

    virtual void SetUnit(std::pair<std::string, dReal> unit)
    {
        unit_ = unit;
    }


protected:

    /// \brief assumes environment and mutex_interfaces_ are locked
    ///
    /// \param[in] it the iterator into bodies_vector_ to erase
    void _RemoveKinBodyFromIterator(vector<KinBodyPtr>::iterator it)
    {
        // before deleting, make sure no robots are grabbing it!!
        FOREACH(itrobot, bodies_vector_) {
            KinBody &body = **it;
            if( (*itrobot)->IsGrabbing(body) ) {
                RAVELOG_WARN_FORMAT("env=%d, remove %s already grabbed by robot %s!", GetId()%body.GetName()%(*itrobot)->GetName());
                (*itrobot)->Release(body);
            }
        }

        if( (*it)->IsRobot() ) {
            vector<RobotBasePtr>::iterator itrobot = std::find(robots_vector_.begin(), robots_vector_.end(), RaveInterfaceCast<RobotBase>(*it));
            if( itrobot != robots_vector_.end() ) {
                robots_vector_.erase(itrobot);
            }
        }
        if( !!current_collision_checker_ ) {
            current_collision_checker_->RemoveKinBody(*it);
        }
        if( !!physics_engine_ ) {
            physics_engine_->RemoveKinBody(*it);
        }
        (*it)->_PostprocessChangedParameters(KinBody::Prop_BodyRemoved);
        RemoveEnvironmentId(*it);
        bodies_vector_.erase(it);
        bodies_modified_stamp_++;
    }

    void _SetDefaultGravity()
    {
        if( !!physics_engine_ )
		{
            // At a latitude of L with altitude H (above sea level), the acceleration due to gravity at sea level is approximately
            // g= 9.780327 * ( 1 + .0053024*sin(L)**2 - .0000058*sin(2L)**2 ) - 0.000003086*H meters per second**2.
            // tokyo,japan 35.6894875 deg
            // rate of change with respect to altitude is da/dH= -2*g*R**2/(R+H)3 = -2g*a*/(R+H)
            physics_engine_->SetGravity(Vector(0,0,-9.797930195020351));
        }
    }

    virtual bool _ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLFile(preader, filename);
    }

    virtual bool _ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLData(preader, pdata);
    }

    virtual void _Clone(std::shared_ptr<Environment const> r, int options, bool bCheckSharedResources=false)
    {
        if( !bCheckSharedResources ) {
            Destroy();
        }

        boost::mutex::scoped_lock lockinit(_mutexInit);
        if( !bCheckSharedResources ) {
            SetCollisionChecker(CollisionCheckerBasePtr());
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        bodies_modified_stamp_ = r->bodies_modified_stamp_;
        home_directory_ = r->home_directory_;
        delta_simulation_time_ = r->delta_simulation_time_;
        cur_simulation_time_ = 0;
        simulation_start_time_ = utils::GetMicroTime();
        environment_index_ = r->environment_index_;
        is_real_time_ = r->is_real_time_;

        is_init_ = true;
        is_enable_simulation_ = r->is_enable_simulation_;

        SetDebugLevel(r->GetDebugLevel());

        if( !bCheckSharedResources || !(options & Clone_Bodies) ) {
            {
                // clear internal interface lists
                boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
                // release all grabbed
                FOREACH(itrobot,bodies_vector_) {
                    (*itrobot)->ReleaseAllGrabbed();
                }
                FOREACH(itbody,bodies_vector_) {
                    (*itbody)->Destroy();
                }
                bodies_vector_.clear();
                FOREACH(itrobot,robots_vector_) {
                    (*itrobot)->Destroy();
                }
                robots_vector_.clear();
                published_bodies_vector_.clear();
            }
            // a little tricky due to a deadlocking situation
            std::map<int, KinBodyWeakPtr> mapBodies;
            {
                boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
                mapBodies = _mapBodies;
                _mapBodies.clear();
            }
            mapBodies.clear();
        }

        list<ViewerBasePtr> listViewers = viewers_list_;
        list< pair<ModuleBasePtr, std::string> > listModules = modules_list_;
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            viewers_list_.clear();
            modules_list_.clear();
        }

        if( !(options & Clone_Viewer) ) {
            RAVELOG_VERBOSE("resetting raveviewer\n");
            FOREACH(itviewer, listViewers) {
                // don't reset the viewer since it can already be dead
                // todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
                (*itviewer)->quitmainloop();
            }
            listViewers.clear();
        }

        if( !(options & Clone_Modules) ) {
            listModules.clear();
        }

        EnvironmentMutex::scoped_lock lock(GetMutex());
        //boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds); // why is this here? if locked, then KinBody::_ComputeInternalInformation freezes on GetBodyFromEnvironmentId call

        bool bCollisionCheckerChanged = false;
        if( !!r->GetCollisionChecker() ) {
            if( !bCheckSharedResources || (!!current_collision_checker_ && current_collision_checker_->GetXMLId() != r->GetCollisionChecker()->GetXMLId()) ) {
                try {
                    CollisionCheckerBasePtr p = RaveCreateCollisionChecker(shared_from_this(),r->GetCollisionChecker()->GetXMLId());
                    p->Clone(r->GetCollisionChecker(),options);
                    SetCollisionChecker(p);
                    bCollisionCheckerChanged = true;
                }
                catch(const std::exception& ex) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to clone collision checker %s: %s"), r->GetCollisionChecker()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
                }
            }
        }
        else {
            SetCollisionChecker(CollisionCheckerBasePtr());
        }

        bool bPhysicsEngineChanged = false;
        if( !!r->GetPhysicsEngine() ) {
            if( !bCheckSharedResources || (!!physics_engine_ && physics_engine_->GetXMLId() != r->GetPhysicsEngine()->GetXMLId()) ) {
                try {
                    PhysicsEngineBasePtr p = RaveCreatePhysicsEngine(shared_from_this(),r->GetPhysicsEngine()->GetXMLId());
                    p->Clone(r->GetPhysicsEngine(),options);
                    SetPhysicsEngine(p);
                    bPhysicsEngineChanged = true;
                }
                catch(const std::exception& ex) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("failed to clone physics engine %s: %s"), r->GetPhysicsEngine()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
                }
            }
        }
        else {
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        if( options & Clone_Bodies ) {
            boost::timed_mutex::scoped_lock lock(r->mutex_interfaces_);
            std::vector<RobotBasePtr> vecrobots;
            std::vector<KinBodyPtr> vecbodies;
            std::vector<std::pair<Vector,Vector> > linkvelocities;
            _mapBodies.clear();
            if( bCheckSharedResources ) {
                // delete any bodies/robots from mapBodies that are not in r->robots_vector_ and r->bodies_vector_
                vecrobots.swap(robots_vector_);
                vecbodies.swap(bodies_vector_);
            }
            // first initialize the pointers
            list<KinBodyPtr> listToClone, listToCopyState;
            FOREACHC(itrobot, r->robots_vector_) {
                try {
                    RobotBasePtr pnewrobot;
                    if( bCheckSharedResources ) {
                        FOREACH(itrobot2,vecrobots) {
                            if( (*itrobot2)->GetName() == (*itrobot)->GetName() && (*itrobot2)->GetKinematicsGeometryHash() == (*itrobot)->GetKinematicsGeometryHash() ) {
                                pnewrobot = *itrobot2;
                                break;
                            }
                        }
                    }
                    if( !pnewrobot ) {
                        pnewrobot = RaveCreateRobot(shared_from_this(), (*itrobot)->GetXMLId());
                        pnewrobot->name_ = (*itrobot)->name_; // at least copy the names
                        listToClone.push_back(*itrobot);
                    }
                    else {
                        //TODO
                        //pnewrobot->ReleaseAllGrabbed(); // will re-grab later?
                        listToCopyState.push_back(*itrobot);
                    }
                    pnewrobot->environment_id_ = (*itrobot)->GetEnvironmentId();
                    BOOST_ASSERT( _mapBodies.find(pnewrobot->GetEnvironmentId()) == _mapBodies.end() );
                    bodies_vector_.push_back(pnewrobot);
                    robots_vector_.push_back(pnewrobot);
                    _mapBodies[pnewrobot->GetEnvironmentId()] = pnewrobot;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone robot %s: %s", (*itrobot)->GetName()%ex.what());
                }
            }
            FOREACHC(itbody, r->bodies_vector_) {
                if( _mapBodies.find((*itbody)->GetEnvironmentId()) != _mapBodies.end() ) {
                    continue;
                }
                try {
                    KinBodyPtr pnewbody;
                    if( bCheckSharedResources ) {
                        FOREACH(itbody2,vecbodies) {
                            if( !(*itbody2) ) {
                                RAVELOG_WARN_FORMAT("env=%d, a body in vecbodies is not initialized", GetId());
                            }
                            else {
                                if( (*itbody2)->GetName() == (*itbody)->GetName() && (*itbody2)->GetKinematicsGeometryHash() == (*itbody)->GetKinematicsGeometryHash() ) {
                                    pnewbody = *itbody2;
                                    break;
                                }
                            }
                        }
                    }
                    if( !pnewbody ) {
                        pnewbody.reset(new KinBody(PT_KinBody,shared_from_this()));
                        pnewbody->name_ = (*itbody)->name_; // at least copy the names
                        listToClone.push_back(*itbody);
                    }
                    else {
                        listToCopyState.push_back(*itbody);
                    }
                    pnewbody->environment_id_ = (*itbody)->GetEnvironmentId();
                    bodies_vector_.push_back(pnewbody);
                    _mapBodies[pnewbody->GetEnvironmentId()] = pnewbody;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to clone body %s: %s", GetId()%(*itbody)->GetName()%ex.what());
                }
            }

            // copy state before cloning
            if( listToCopyState.size() > 0 ) {
                FOREACH(itbody,listToCopyState) {
                    KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                    if( bCollisionCheckerChanged ) {
                        GetCollisionChecker()->InitKinBody(pnewbody);
                    }
                    if( bPhysicsEngineChanged ) {
                        GetPhysicsEngine()->InitKinBody(pnewbody);
                    }
                    pnewbody->__hashkinematics = (*itbody)->__hashkinematics;
                    if( pnewbody->IsRobot() ) {
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                        // don't clone grabbed bodies!
                        RobotBase::RobotStateSaver saver(poldrobot, 0xffffffff&~KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewrobot);
                        pnewrobot->hash_robot_structure_ = poldrobot->hash_robot_structure_;
                    }
                    else {
                        KinBody::KinBodyStateSaver saver(*itbody, 0xffffffff&~KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewbody);
                    }
                }
            }

            // now clone
            FOREACHC(itbody, listToClone) {
                try {
                    KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                    if( !!pnewbody ) {
                        pnewbody->Clone(*itbody,options);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone body %s: %s", (*itbody)->GetName()%ex.what());
                }
            }
            FOREACH(itbody,listToClone) {
                KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                pnewbody->_ComputeInternalInformation();
                GetCollisionChecker()->InitKinBody(pnewbody);
                GetPhysicsEngine()->InitKinBody(pnewbody);
                pnewbody->__hashkinematics = (*itbody)->__hashkinematics; /// _ComputeInternalInformation resets the hashes
                if( pnewbody->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                    pnewrobot->hash_robot_structure_ = poldrobot->hash_robot_structure_;
                }
            }
            // update the state after every body is initialized!
            FOREACH(itbody,listToClone) {
                KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                if( (*itbody)->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_mapBodies[(*itbody)->GetEnvironmentId()].lock());
                    // need to also update active dof/active manip since it is erased by _ComputeInternalInformation
                    RobotBase::RobotStateSaver saver(poldrobot, KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator);
                    saver.Restore(pnewrobot);
                }
                else {
                    KinBody::KinBodyStateSaver saver(*itbody, KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities); // all the others should have been saved?
                    saver.Restore(pnewbody);
                }
            }
            if( listToCopyState.size() > 0 ) {
                // check for re-grabs after cloning is done
                FOREACH(itbody,listToCopyState) {
                    if( (*itbody)->IsRobot() ) {
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_mapBodies[(*itbody)->GetEnvironmentId()].lock());
                        RobotBase::RobotStateSaver saver(poldrobot, KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewrobot);
                    }
                }
            }
        }
        if( options & Clone_Sensors ) {
            boost::timed_mutex::scoped_lock lock(r->mutex_interfaces_);
            FOREACHC(itsensor,r->sensors_list_) {
                try {
                    SensorBasePtr pnewsensor = RaveCreateSensor(shared_from_this(), (*itsensor)->GetXMLId());
                    pnewsensor->Clone(*itsensor, options);
                    sensors_list_.push_back(pnewsensor);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone sensor %: %s", (*itsensor)->GetName()%ex.what());
                }
            }
        }
        // sensors might be attached on a robot?, so have to re-update
        FOREACH(itrobot, robots_vector_) {
            (*itrobot)->_UpdateAttachedSensors();
        }

        if( options & Clone_Simulation ) {
            is_enable_simulation_ = r->is_enable_simulation_;
            cur_simulation_time_ = r->cur_simulation_time_;
            simulation_start_time_ = r->simulation_start_time_;
        }

        if( options & Clone_Modules ) {
            list< pair<ModuleBasePtr, std::string> > listModules2 = r->modules_list_;
            FOREACH(itmodule2, listModules2) {
                try {
                    ModuleBasePtr pmodule;
                    std::string cmdargs;
                    if( bCheckSharedResources ) {
                        FOREACH(itmodule,listModules) {
                            if( itmodule->first->GetXMLId() == itmodule2->first->GetXMLId() ) {
                                pmodule = itmodule->first;
                                cmdargs = itmodule->second;
                                listModules.erase(itmodule);
                                break;
                            }
                        }
                    }
                    if( !pmodule ) {
                        pmodule = RaveCreateModule(shared_from_this(),itmodule2->first->GetXMLId());
                        cmdargs = itmodule2->second;
                    }
                    // add first before cloning!
                    AddModule(pmodule, cmdargs);
                    pmodule->Clone(itmodule2->first, options);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone module %s: %s", itmodule2->first->GetXMLId()%ex.what());
                }
            }
        }

        listModules.clear(); // have to clear the unused modules

        if( options & Clone_Viewer ) {
            list<ViewerBasePtr> listViewers2;
            r->GetViewers(listViewers2);
            FOREACH(itviewer2, listViewers2) {
                try {
                    ViewerBasePtr pviewer;
                    if( bCheckSharedResources ) {
                        FOREACH(itviewer,listViewers) {
                            if( (*itviewer)->GetXMLId() == (*itviewer2)->GetXMLId() ) {
                                pviewer = *itviewer;
                                listViewers.erase(itviewer);
                                break;
                            }
                        }
                    }
                    if( !pviewer ) {
                        pviewer = RaveCreateViewer(shared_from_this(),(*itviewer2)->GetXMLId());
                    }
                    pviewer->Clone(*itviewer2,options);
                    AddViewer(pviewer);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone viewer %s: %s", (*itviewer2)->GetName()%ex.what());
                }
            }
        }

        // reset left-over viewers
        FOREACH(itviewer, listViewers) {
            (*itviewer)->quitmainloop();
        }
        listViewers.clear();

        if( !bCheckSharedResources ) {
            if( !!simulation_thread_ && is_enable_simulation_ ) {
                _StartSimulationThread();
            }
        }
    }

    virtual bool _CheckUniqueName(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        FOREACHC(itbody,bodies_vector_) {
            if(( *itbody != pbody) &&( (*itbody)->GetName() == pbody->GetName()) ) {
                if( bDoThrow ) {
                    throw OpenRAVEException(str(boost::format(_tr("env=%d, body %s does not have unique name"))%GetId()%pbody->GetName()));
                }
                return false;
            }
        }
        return true;
    }
    virtual bool _CheckUniqueName(SensorBaseConstPtr psensor, bool bDoThrow=false) const
    {
        FOREACHC(itsensor,sensors_list_) {
            if(( *itsensor != psensor) &&( (*itsensor)->GetName() == psensor->GetName()) ) {
                if( bDoThrow ) {
                    throw OpenRAVEException(str(boost::format(_tr("env=%d, sensor %s does not have unique name"))%GetId()%psensor->GetName()));
                }
                return false;
            }
        }
        return true;
    }
    virtual bool _CheckUniqueName(ViewerBaseConstPtr pviewer, bool bDoThrow=false) const
    {
        FOREACHC(itviewer,viewers_list_) {
            if(( *itviewer != pviewer) &&( (*itviewer)->GetName() == pviewer->GetName()) ) {
                if( bDoThrow ) {
                    throw OpenRAVEException(str(boost::format(_tr("env=%d, viewer '%s' does not have unique name"))%GetId()%pviewer->GetName()));
                }
                return false;
            }
        }
        return true;
    }

    virtual void SetEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
        int id = environment_index_++;
        BOOST_ASSERT( _mapBodies.find(id) == _mapBodies.end() );
        pbody->environment_id_=id;
        _mapBodies[id] = pbody;
    }

    virtual void RemoveEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
        _mapBodies.erase(pbody->environment_id_);
        pbody->environment_id_ = 0;
        pbody->_DeinitializeInternalInformation();
    }

    void _StartSimulationThread()
    {
        if( !simulation_thread_ ) 
		{
            is_shutdown_simulation_ = false;
            simulation_thread_.reset(new boost::thread(boost::bind(&Environment::_SimulationThread, this)));
        }
    }

    void _StopSimulationThread()
    {
        is_shutdown_simulation_ = true;
        if( !!simulation_thread_ ) 
		{
            simulation_thread_->join();
            simulation_thread_.reset();
        }
    }

    void _SimulationThread()
    {
        int environmentid = RaveGetEnvironmentId(shared_from_this());

        uint64_t last_update_time = utils::GetMicroTime();
        uint64_t last_slept_time = utils::GetMicroTime();
        RAVELOG_VERBOSE_FORMAT("starting simulation thread envid=%d", environmentid);
        while( is_init_ && !is_shutdown_simulation_ ) 
		{
            bool is_need_sleep = true;
            std::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv;
            if( is_enable_simulation_ ) 
			{
                is_need_sleep = false;
                lockenv = _LockEnvironmentWithTimeout(100000);
                if( !!lockenv ) 
				{
                    //Get deltasimtime in microseconds
                    int64_t deltasimtime = (int64_t)(delta_simulation_time_*1000000.0f);
                    try 
					{
                        StepSimulation(delta_simulation_time_);
                    }
                    catch(const std::exception &ex) 
					{
                        RAVELOG_ERROR("simulation thread exception: %s\n",ex.what());
                    }
                    uint64_t passedtime = utils::GetMicroTime()-simulation_start_time_;
                    int64_t sleeptime = cur_simulation_time_-passedtime;
                    //Hardcoded tolerance for now
                    const int tol=2;
                    if( is_real_time_ ) 
					{
                        if(( sleeptime > deltasimtime/tol) &&( sleeptime > 1000) )
						{
                            lockenv.reset();
                            // sleep for less time since sleep isn't accurate at all and we have a 7ms buffer
                            int actual_sleep=max((int)sleeptime*6/8,1000);
                            boost::this_thread::sleep (boost::posix_time::microseconds(actual_sleep));
                            //RAVELOG_INFO("sleeptime ideal %d, actually slept: %d\n",(int)sleeptime,(int)actual_sleep);
                            last_slept_time = utils::GetMicroTime();
                            //Since already slept this cycle, wait till next time to sleep.
                            is_need_sleep = false;
                        }
                        else if( sleeptime < -deltasimtime/tol && ( sleeptime < -1000) )
						{
                            // simulation is getting late, so catch up (doesn't happen often in light loads)
                            //RAVELOG_INFO("sim catching up: %d\n",-(int)sleeptime);
                            simulation_start_time_ += -sleeptime;     //deltasimtime;
                        }
                    }
                    else 
					{
                        last_slept_time = utils::GetMicroTime();
                    }

                    //RAVELOG_INFOA("sim: %f, real: %f\n",cur_simulation_time_*1e-6f,(utils::GetMicroTime()-simulation_start_time_)*1e-6f);
                }
            }

            if( utils::GetMicroTime()-last_slept_time > 20000 )
			{     // 100000 freezes the environment
                lockenv.reset();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                is_need_sleep = false;
                last_slept_time = utils::GetMicroTime();
            }

            if( utils::GetMicroTime()-last_update_time > 10000 )
			{
                if( !lockenv ) 
				{
                    lockenv = _LockEnvironmentWithTimeout(100000);
                }
                if( !!lockenv ) 
				{
                    last_update_time = utils::GetMicroTime();
                    // environment might be getting destroyed during this call, so to avoid a potential deadlock, add a timeout
                    try 
					{
                        UpdatePublishedBodies(1000000); // 1.0s
                    }
                    catch(const std::exception& ex)
					{
                        RAVELOG_WARN("timeout of UpdatePublishedBodies\n");
                    }
                }
            }

            //TODO: Verify if this always has to happen even if thread slept in RT if statement above
            lockenv.reset(); // always release at the end of loop to give other threads time
            if( is_need_sleep ) 
			{
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }

    /// mutex_interfaces_ should not be locked
    void _CallBodyCallbacks(KinBodyPtr pbody, int action)
    {
        std::list<UserDataWeakPtr> listRegisteredBodyCallbacks;
        {
            boost::timed_mutex::scoped_lock lock(mutex_interfaces_);
            listRegisteredBodyCallbacks = _listRegisteredBodyCallbacks;
        }
        FOREACH(it, listRegisteredBodyCallbacks) {
            BodyCallbackDataPtr pdata = std::dynamic_pointer_cast<BodyCallbackData>(it->lock());
            if( !!pdata ) {
                pdata->_callback(pbody, action);
            }
        }
    }

    std::shared_ptr<EnvironmentMutex::scoped_try_lock> _LockEnvironmentWithTimeout(uint64_t timeout)
    {
        // try to acquire the lock
#if BOOST_VERSION >= 103500
        std::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetMutex(),boost::defer_lock_t()));
#else
        std::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetMutex(),false));
#endif
        uint64_t basetime = utils::GetMicroTime();
        while(utils::GetMicroTime()-basetime<timeout )
		{
            lockenv->try_lock();
            if( !!*lockenv )
			{
                break;
            }
        }

        if( !*lockenv )
		{
            lockenv.reset();
        }
        return lockenv;
    }

    static bool _IsColladaURI(const std::string& uri)
    {
        std::string scheme, authority, path, query, fragment;
        std::string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);
		//TODO: test
		std::match_results<string::const_iterator> result;
		std::regex pattern("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
		bool matched = std::regex_match(uri, result, pattern);	

        return bmatch && scheme.size() > 0 && _IsColladaFile(path); //scheme.size() > 0;
    }

    static bool _IsColladaFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 4 ) 
		{
            return false;
        }
        if( filename[len-4] == '.' 
			&& ::tolower(filename[len-3]) == 'd' 
			&& ::tolower(filename[len-2]) == 'a' 
			&& ::tolower(filename[len-1]) == 'e' ) 
		{
            return true;
        }
        if( filename[len-4] == '.' 
			&& ::tolower(filename[len-3]) == 'z' 
			&& ::tolower(filename[len-2]) == 'a' 
			&& ::tolower(filename[len-1]) == 'e' )
		{
            return true;
        }
        return false;
    }
    static bool _IsColladaData(const std::string& data)
    {
        return data.find("<COLLADA") != std::string::npos;
    }

    static bool _IsXFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 2 ) 
		{
            return false;
        }
        return filename[len-2] == '.' && ::tolower(filename[len-1]) == 'x';
    }

    static bool _IsXData(const std::string& data)
    {
        return data.size() >= 4 && data[0] == 'x' && data[1] == 'o' && data[2] == 'f' && data[3] == ' ';
    }

    static bool _IsIVData(const std::string& data)
    {
        if( data.size() >= 10 ) 
		{
            if( data.substr(0,10) == std::string("#Inventor ") )
			{
                return true;
            }
        }
        return false;
    }

    static bool _IsOpenRAVEFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 4 ) 
		{
            return false;
        }
        if(( filename[len-4] == '.') 
			&&( ::tolower(filename[len-3]) == 'x') 
			&&( ::tolower(filename[len-2]) == 'm') 
			&&( ::tolower(filename[len-1]) == 'l') ) 
		{
            return true;
        }
        return false;
    }
    static bool _IsRigidModelFile(const std::string& filename)
    {
        static std::array<std::string,21> s_geometryextentsions = { 
			{ "iv","vrml","wrl","stl","blend","3ds","ase","obj","ply",
			"dxf","lwo","lxo","ac","ms3d","x","mesh.xml","irrmesh","irr","nff","off","raw"}};
        for(auto &it: s_geometryextentsions) 
		{
            if( filename.size() > it.size()+1 )
			{
                size_t len = filename.size();
                if( filename.at(len-it.size()-1) == '.' ) 
				{
                    bool is_success = true;
                    for(size_t i = 0; i < it.size(); ++i) 
					{
                        if( ::tolower(filename[len-i-1]) != it[it.size()-i-1] )
						{
                            is_success = false;
                            break;
                        }
                    }
                    if( is_success ) 
					{
                        return true;
                    }
                }
            }
        }
        return false;
    }

    static bool _IsJSONURI(const std::string& uri)
    {
        std::string scheme, authority, path, query, fragment;
		std::string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);

		//TODO: test
		std::match_results<string::const_iterator> result;
		std::regex pattern("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
		bool matched = std::regex_match(uri, result, pattern);

        return bmatch && scheme.size() > 0 && _IsJSONFile(path); //scheme.size() > 0;
    }

    static bool _IsJSONFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 5 ) 
		{
            return false;
        }
        if( filename[len-5] == '.' 
			&& ::tolower(filename[len-4]) == 'j'
			&& ::tolower(filename[len-3]) == 's' 
			&& ::tolower(filename[len-2]) == 'o' 
			&& ::tolower(filename[len-1]) == 'n' ) 
		{
            return true;
        }
        return false;
    }

    static bool _IsJSONData(const std::string& data)
    {
        return data.size() >= 2 && data[0] == '{';
    }

    std::vector<RobotBasePtr> robots_vector_;      //!< robots (possibly controlled)
    std::vector<KinBodyPtr> bodies_vector_;     //!< all objects that are collidable (includes robots)

    std::list< std::pair<ModuleBasePtr, std::string> > modules_list_;     //!< modules loaded in the environment and the strings they were intialized with. Initialization strings are used for cloning.
	std::list<SensorBasePtr> sensors_list_;     //!< sensors loaded in the environment
	std::list<ViewerBasePtr> viewers_list_;     //!< viewers loaded in the environment

    dReal delta_simulation_time_;                    //!< delta time for simulate step
    uint64_t cur_simulation_time_;                        //!< simulation time since the start of the environment
    uint64_t simulation_start_time_;
    int bodies_modified_stamp_;     //!< incremented every tiem bodies vector is modified

    CollisionCheckerBasePtr current_collision_checker_;
    PhysicsEngineBasePtr physics_engine_;

    int environment_index_;                   //!< next network index
    std::map<int, KinBodyWeakPtr> _mapBodies;     //!< a map of all the bodies in the environment. Controlled through the KinBody constructor and destructors

    std::shared_ptr<boost::thread> simulation_thread_;                      //!< main loop for environment simulation

    mutable EnvironmentMutex _mutexEnvironment;          //!< protects internal data from multithreading issues
    mutable boost::mutex _mutexEnvironmentIds;      //!< protects bodies_vector_/robots_vector_ from multithreading issues
    mutable boost::timed_mutex mutex_interfaces_;     //!< lock when managing interfaces like _listOwnedInterfaces, modules_list_, _mapBodies
    mutable boost::mutex _mutexInit;     //!< lock for destroying the environment

    std::vector<KinBody::BodyState> published_bodies_vector_;
    std::string home_directory_;
    std::pair<std::string, dReal> unit_; //!< unit name mm, cm, inches, m and the conversion for meters

    UserDataPtr generic_robot_handle_, generic_trajectory_handle_, multi_controller_handle_, generic_physics_engine_handle_, generic_collision_checker_handle_;

    std::list<InterfaceBasePtr> _listOwnedInterfaces;

    std::list<UserDataWeakPtr> _listRegisteredCollisionCallbacks;     //!< see EnvironmentBase::RegisterCollisionCallback
    std::list<UserDataWeakPtr> _listRegisteredBodyCallbacks;     //!< see EnvironmentBase::RegisterBodyCallback

    bool is_init_;                   //!< environment is initialized
    bool is_enable_simulation_;            //!< enable simulation loop
    bool is_shutdown_simulation_; //!< if true, the simulation thread should shutdown
    bool is_real_time_;

    friend class EnvironmentXMLReader;
};
//}
#endif

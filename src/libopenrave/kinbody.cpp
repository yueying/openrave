// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <algorithm>
#include <openrave/electric_motor_actuator_info.h>
#include <openrave/grabbed.h>

// used for functions that are also used internally
#define CHECK_NO_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(hierarchy_computed_ == 0, "body %s cannot be added to environment when doing this operation, current value is %d", GetName()%hierarchy_computed_, ORE_InvalidState);
#define CHECK_INTERNAL_COMPUTATION0 OPENRAVE_ASSERT_FORMAT(hierarchy_computed_ != 0, "body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%hierarchy_computed_, ORE_NotInitialized);
#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(hierarchy_computed_ == 2, "body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%hierarchy_computed_, ORE_NotInitialized);

namespace OpenRAVE {

class ChangeCallbackData : public UserData
{
public:
    ChangeCallbackData(int properties, const boost::function<void()>& callback, KinBodyConstPtr pbody) : _properties(properties), _callback(callback), _pweakbody(pbody) {
    }
    virtual ~ChangeCallbackData() {
        KinBodyConstPtr pbody = _pweakbody.lock();
        if( !!pbody ) {
            boost::unique_lock< boost::shared_mutex > lock(pbody->GetInterfaceMutex());
            FOREACH(itinfo, _iterators) {
                pbody->_vlistRegisteredCallbacks.at(itinfo->first).erase(itinfo->second);
            }
        }
    }

    list< std::pair<uint32_t, list<UserDataWeakPtr>::iterator> > _iterators;
    int _properties;
    boost::function<void()> _callback;
protected:
    std::weak_ptr<KinBody const> _pweakbody;
};

class CallFunctionAtDestructor
{
public:
    CallFunctionAtDestructor(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallFunctionAtDestructor() {
        _fn();
    }

protected:
    boost::function<void()> _fn;
};

typedef std::shared_ptr<ChangeCallbackData> ChangeCallbackDataPtr;



KinBody::KinBody(InterfaceType type, EnvironmentBasePtr penv) : InterfaceBase(type, penv)
{
    hierarchy_computed_ = 0;
    parameters_changed_ = 0;
    _bMakeJoinedLinksAdjacent = true;
    environment_id_ = 0;
    non_adjacent_link_cache_ = 0x80000000;
    update_stamp_id_ = 0;
    is_all_joints_1dof_and_no_circular_ = false;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSE_FORMAT("env=%d, destructing kinbody '%s'", GetEnv()->GetId()%GetName());
    Destroy();
}

void KinBody::Destroy()
{
    ReleaseAllGrabbed();
    if( attached_bodies_list_.size() > 0 )
	{
        // could be in the environment destructor?
        std::stringstream ss; ss << GetName() << " still has attached bodies: ";
        for(auto it:attached_bodies_list_) 
		{
            KinBodyPtr pattached = it.lock();
            if( !!pattached ) 
			{
                ss << pattached->GetName();
            }
        }
        RAVELOG_VERBOSE(ss.str());
    }
    attached_bodies_list_.clear();

    links_vector_.clear();
    joints_vector_.clear();
    topologically_sorted_joints_vector_.clear();
    topologically_sorted_joints_all_vector_.clear();
    dof_ordered_joints_vector_.clear();
    passive_joints_vector_.clear();
    joints_affecting_links_vector_.clear();
    dof_indices_vector_.clear();

    _setAdjacentLinks.clear();
    _vInitialLinkTransformations.clear();
    all_pairs_shortest_paths_vector_.clear();
    closed_loops_vector_.clear();
    closed_loop_indices_vector_.clear();
    _vForcedAdjacentLinks.clear();
    hierarchy_computed_ = 0;
    parameters_changed_ = 0;
    _pManageData.reset();

    _ResetInternalCollisionCache();
    self_collision_checker_.reset();
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index_ = 0;
    plink->info_.name_ = "base";
    plink->info_.is_static_ = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itab, vaabbs) {
        GeometryInfo info;
        info.type_ = GT_Box;
        info.transform_.trans = itab->pos;
        info.is_visible_ = visible;
        info.gemo_outer_extents_data_ = itab->extents;
        info.diffuse_color_vec_=Vector(1,0.5f,0.5f,1);
        info.ambient_color_vec_=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->info_.InitCollisionMesh();
        numvertices += geom->GetCollisionMesh().vertices.size();
        numindices += geom->GetCollisionMesh().indices.size();
        plink->geometries_vector_.push_back(geom);
    }

    plink->collision_.vertices.reserve(numvertices);
    plink->collision_.indices.reserve(numindices);
    TriMesh trimesh;
    FOREACH(itgeom,plink->geometries_vector_) {
        trimesh = (*itgeom)->GetCollisionMesh();
        trimesh.ApplyTransform((*itgeom)->GetTransform());
        plink->collision_.Append(trimesh);
    }
    links_vector_.push_back(plink);
    str_uri_ = uri;
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index_ = 0;
    plink->info_.name_ = "base";
    plink->info_.is_static_ = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itobb, vobbs) {
        TransformMatrix tm;
        tm.trans = itobb->pos;
        tm.m[0] = itobb->right.x; tm.m[1] = itobb->up.x; tm.m[2] = itobb->dir.x;
        tm.m[4] = itobb->right.y; tm.m[5] = itobb->up.y; tm.m[6] = itobb->dir.y;
        tm.m[8] = itobb->right.z; tm.m[9] = itobb->up.z; tm.m[10] = itobb->dir.z;
        GeometryInfo info;
        info.type_ = GT_Box;
        info.transform_ = tm;
        info.is_visible_ = visible;
        info.gemo_outer_extents_data_ = itobb->extents;
        info.diffuse_color_vec_=Vector(1,0.5f,0.5f,1);
        info.ambient_color_vec_=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->info_.InitCollisionMesh();
        numvertices += geom->GetCollisionMesh().vertices.size();
        numindices += geom->GetCollisionMesh().indices.size();
        plink->geometries_vector_.push_back(geom);
    }

    plink->collision_.vertices.reserve(numvertices);
    plink->collision_.indices.reserve(numindices);
    TriMesh trimesh;
    FOREACH(itgeom,plink->geometries_vector_) {
        trimesh = (*itgeom)->GetCollisionMesh();
        trimesh.ApplyTransform((*itgeom)->GetTransform());
        plink->collision_.Append(trimesh);
    }
    links_vector_.push_back(plink);
    str_uri_ = uri;
    return true;
}

bool KinBody::InitFromSpheres(const std::vector<Vector>& vspheres, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index_ = 0;
    plink->info_.name_ = "base";
    plink->info_.is_static_ = true;
    TriMesh trimesh;
    FOREACHC(itv, vspheres) {
        GeometryInfo info;
        info.type_ = GT_Sphere;
        info.transform_.trans.x = itv->x; info.transform_.trans.y = itv->y; info.transform_.trans.z = itv->z;
        info.is_visible_ = visible;
        info.gemo_outer_extents_data_.x = itv->w;
        info.diffuse_color_vec_=Vector(1,0.5f,0.5f,1);
        info.ambient_color_vec_=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->info_.InitCollisionMesh();
        plink->geometries_vector_.push_back(geom);
        trimesh = geom->GetCollisionMesh();
        trimesh.ApplyTransform(geom->GetTransform());
        plink->collision_.Append(trimesh);
    }
    links_vector_.push_back(plink);
    str_uri_ = uri;
    return true;
}

bool KinBody::InitFromTrimesh(const TriMesh& trimesh, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index_ = 0;
    plink->info_.name_ = "base";
    plink->info_.is_static_ = true;
    plink->collision_ = trimesh;
    GeometryInfo info;
    info.type_ = GT_TriMesh;
    info.is_visible_ = visible;
    info.diffuse_color_vec_=Vector(1,0.5f,0.5f,1);
    info.ambient_color_vec_=Vector(0.1,0.0f,0.0f,0);
    info.mesh_collision_ = trimesh;
    Link::GeometryPtr geom(new Link::Geometry(plink,info));
    plink->geometries_vector_.push_back(geom);
    links_vector_.push_back(plink);
    str_uri_ = uri;
    return true;
}

bool KinBody::InitFromGeometries(const std::list<KinBody::GeometryInfo>& geometries, const std::string& uri)
{
    std::vector<GeometryInfoConstPtr> newgeometries; newgeometries.reserve(geometries.size());
    FOREACHC(it, geometries) {
        newgeometries.push_back(GeometryInfoConstPtr(&(*it), utils::null_deleter()));
    }
    return InitFromGeometries(newgeometries, uri);
}

bool KinBody::InitFromGeometries(const std::vector<KinBody::GeometryInfoConstPtr>& geometries, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(geometries.size(),>,0);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index_ = 0;
    plink->info_.name_ = "base";
    plink->info_.is_static_ = true;
    FOREACHC(itinfo,geometries) {
        Link::GeometryPtr geom(new Link::Geometry(plink,**itinfo));
        geom->info_.InitCollisionMesh();
        plink->geometries_vector_.push_back(geom);
        plink->collision_.Append(geom->GetCollisionMesh(),geom->GetTransform());
    }
    links_vector_.push_back(plink);
    str_uri_ = uri;
    return true;
}

void KinBody::SetLinkGeometriesFromGroup(const std::string& geomname)
{
    // need to call _PostprocessChangedParameters at the very end, even if exception occurs
    CallFunctionAtDestructor callfn(boost::bind(&KinBody::_PostprocessChangedParameters, this, Prop_LinkGeometry));
    FOREACHC(itlink, links_vector_) {
        std::vector<KinBody::GeometryInfoPtr>* pvinfos = NULL;
        if( geomname.size() == 0 ) {
            pvinfos = &(*itlink)->info_.geometry_infos_vector_;
        }
        else {
            std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = (*itlink)->info_.extra_geometries_map_.find(geomname);
            if( it == (*itlink)->info_.extra_geometries_map_.end() ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("could not find geometries %s for link %s"),geomname%GetName(),ORE_InvalidArguments);
            }
            pvinfos = &it->second;
        }
        (*itlink)->geometries_vector_.resize(pvinfos->size());
        for(size_t i = 0; i < pvinfos->size(); ++i) 
		{
            (*itlink)->geometries_vector_[i].reset(new Link::Geometry(*itlink,*pvinfos->at(i)));
            if( (*itlink)->geometries_vector_[i]->GetCollisionMesh().vertices.size() == 0 ) { // try to avoid recomputing
                (*itlink)->geometries_vector_[i]->InitCollisionMesh();
            }
        }
        (*itlink)->_Update(false);
    }
    // have to reset the adjacency cache
    _ResetInternalCollisionCache();
}

void KinBody::SetLinkGroupGeometries(const std::string& geomname, const std::vector< std::vector<KinBody::GeometryInfoPtr> >& linkgeometries)
{
    OPENRAVE_ASSERT_OP( linkgeometries.size(), ==, links_vector_.size() );
    FOREACH(itlink, links_vector_) {
        Link& link = **itlink;
        std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = link.info_.extra_geometries_map_.insert(make_pair(geomname,std::vector<KinBody::GeometryInfoPtr>())).first;
        const std::vector<KinBody::GeometryInfoPtr>& geometries = linkgeometries.at(link.GetIndex());
        it->second.resize(geometries.size());
        std::copy(geometries.begin(),geometries.end(),it->second.begin());
    }
    _PostprocessChangedParameters(Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

bool KinBody::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(linkinfos.size(),>,0);
    Destroy();
    links_vector_.reserve(linkinfos.size());
    FOREACHC(itlinkinfo, linkinfos) {
        LinkPtr plink(new Link(shared_kinbody()));
        plink->info_ = **itlinkinfo;
        _InitAndAddLink(plink);
    }
    joints_vector_.reserve(jointinfos.size());
    FOREACHC(itjointinfo, jointinfos) {
        JointInfoConstPtr rawinfo = *itjointinfo;
        JointPtr pjoint(new Joint(shared_kinbody()));
        pjoint->info_ = *rawinfo;
        _InitAndAddJoint(pjoint);
    }
    str_uri_ = uri;
    return true;
}

void KinBody::SetName(const std::string& newname)
{
    OPENRAVE_ASSERT_OP(newname.size(), >, 0);
    if( name_ != newname ) {
        // have to replace the 2nd word of all the groups with the robot name
        FOREACH(itgroup, _spec.groups_vector_) {
            stringstream ss(itgroup->name);
            string grouptype, oldname;
            ss >> grouptype >> oldname;
            stringbuf buf;
            ss.get(buf,0);
            itgroup->name = str(boost::format("%s %s %s")%grouptype%newname%buf.str());
        }
        name_ = newname;
        _PostprocessChangedParameters(Prop_Name);
    }
}

void KinBody::SetDOFTorques(const std::vector<dReal>& torques, bool bAdd)
{
    OPENRAVE_ASSERT_OP_FORMAT((int)torques.size(), >=, GetDOF(), "not enough values %d<%d", torques.size()%GetDOF(),ORE_InvalidArguments);
    if( !bAdd ) {
        FOREACH(itlink, links_vector_) {
            (*itlink)->SetForce(Vector(),Vector(),false);
            (*itlink)->SetTorque(Vector(),false);
        }
    }
    std::vector<dReal> jointtorques;
    FOREACH(it, joints_vector_) {
        jointtorques.resize((*it)->GetDOF());
        std::copy(torques.begin()+(*it)->GetDOFIndex(),torques.begin()+(*it)->GetDOFIndex()+(*it)->GetDOF(),jointtorques.begin());
        (*it)->AddTorque(jointtorques);
    }
}

int KinBody::GetDOF() const
{
    return joints_vector_.size() > 0 ? joints_vector_.back()->GetDOFIndex()+joints_vector_.back()->GetDOF() : 0;
}

void KinBody::GetDOFValues(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( dofindices.size() == 0 ) {
        v.clear();
        v.reserve(GetDOF());
        FOREACHC(it, dof_ordered_joints_vector_) {
            int toadd = (*it)->GetDOFIndex()-(int)v.size();
            if( toadd > 0 ) {
                v.insert(v.end(),toadd,0);
            }
            else if( toadd < 0 ) {
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ss << "values=[";
                FOREACH(itvalue, v) {
                    ss << *itvalue << ", ";
                }
                ss << "]; jointorder=[";
                FOREACH(itj, dof_ordered_joints_vector_) {
                    ss << (*itj)->GetName() << ", ";
                }
                ss << "];";
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof indices mismatch joint %s (dofindex=%d), toadd=%d, v.size()=%d in call GetDOFValues with %s"), (*it)->GetName()%(*it)->GetDOFIndex()%toadd%v.size()%ss.str(), ORE_InvalidState);
            }
            (*it)->GetValues(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetValue(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFVelocities(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetVelocities(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetVelocity(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        vLowerLimit.resize(0);
        if( (int)vLowerLimit.capacity() < GetDOF() ) {
            vLowerLimit.reserve(GetDOF());
        }
        vUpperLimit.resize(0);
        if( (int)vUpperLimit.capacity() < GetDOF() ) {
            vUpperLimit.reserve(GetDOF());
        }
        FOREACHC(it,dof_ordered_joints_vector_) {
            (*it)->GetLimits(vLowerLimit,vUpperLimit,true);
        }
    }
    else {
        vLowerLimit.resize(dofindices.size());
        vUpperLimit.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = pjoint->GetLimit(dofindices[i]-pjoint->GetDOFIndex());
            vLowerLimit[i] = res.first;
            vUpperLimit[i] = res.second;
        }
    }
}

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        vlower.resize(0);
        vupper.resize(0);
        if( (int)vlower.capacity() < GetDOF() ) {
            vlower.reserve(GetDOF());
        }
        if( (int)vupper.capacity() < GetDOF() ) {
            vupper.reserve(GetDOF());
        }
        FOREACHC(it,dof_ordered_joints_vector_) {
            (*it)->GetVelocityLimits(vlower,vupper,true);
        }
    }
    else {
        vlower.resize(dofindices.size());
        vupper.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = pjoint->GetVelocityLimit(dofindices[i]-pjoint->GetDOFIndex());
            vlower[i] = res.first;
            vupper[i] = res.second;
        }
    }
}

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetVelocityLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetMaxVel(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFAccelerationLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetAccelerationLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetAccelerationLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFJerkLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetJerkLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetJerkLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardVelocityLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetHardVelocityLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardVelocityLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardAccelerationLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetHardAccelerationLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardAccelerationLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardJerkLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, dof_ordered_joints_vector_) {
            (*it)->GetHardJerkLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardJerkLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFTorqueLimits(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, dof_ordered_joints_vector_) {
        (*it)->GetTorqueLimits(v,true);
    }
}

void KinBody::GetDOFMaxTorque(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, dof_ordered_joints_vector_) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxTorque());
    }
}

void KinBody::GetDOFResolutions(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() )
            v.reserve(GetDOF());
        FOREACHC(it, dof_ordered_joints_vector_) {
            v.insert(v.end(),(*it)->GetDOF(),(*it)->GetResolution());
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetResolution(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFWeights(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(GetDOF());
        std::vector<dReal>::iterator itv = v.begin();
        FOREACHC(it, dof_ordered_joints_vector_) {
            for(int i = 0; i < (*it)->GetDOF(); ++i) {
                *itv++ = (*it)->GetWeight(i);
            }
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetWeight(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::SetDOFWeights(const std::vector<dReal>& v, const std::vector<int>& dofindices)
{
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)v.size(),>=,GetDOF());
        for(int i = 0; i < GetDOF(); ++i) {
            OPENRAVE_ASSERT_OP_FORMAT(v[i], >, 0, "dof %d weight %f has to be >= 0", i%v[i], ORE_InvalidArguments);
        }
        std::vector<dReal>::const_iterator itv = v.begin();
        FOREACHC(it, dof_ordered_joints_vector_) {
            std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.weights_vector_.begin());
            itv += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(v.size(),==,dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            pjoint->info_.weights_vector_.at(dofindices[i]-pjoint->GetDOFIndex()) = v[i];
        }
    }
    _PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::SetDOFResolutions(const std::vector<dReal>& v, const std::vector<int>& dofindices)
{
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)v.size(),>=,GetDOF());
        for(int i = 0; i < GetDOF(); ++i) {
            OPENRAVE_ASSERT_OP_FORMAT(v[i], >, 0, "dof %d resolution %f has to be >= 0", i%v[i], ORE_InvalidArguments);
        }
        std::vector<dReal>::const_iterator itv = v.begin();
        FOREACHC(it, dof_ordered_joints_vector_) {
            std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.resolution_vector_.begin());
            itv += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(v.size(),==,dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            pjoint->info_.resolution_vector_.at(dofindices[i]-pjoint->GetDOFIndex()) = v[i];
        }
    }
    _PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::SetDOFLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper, const std::vector<int>& dofindices)
{
    bool bChanged = false;
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)lower.size(),==,GetDOF());
        OPENRAVE_ASSERT_OP((int)upper.size(),==,GetDOF());
        std::vector<dReal>::const_iterator itlower = lower.begin(), itupper = upper.begin();
        FOREACHC(it, dof_ordered_joints_vector_) {
            for(int i = 0; i < (*it)->GetDOF(); ++i) {
                if( (*it)->info_.lower_limit_vector_.at(i) != *(itlower+i) || (*it)->info_.upper_limit_vector_.at(i) != *(itupper+i) ) {
                    bChanged = true;
                    std::copy(itlower,itlower+(*it)->GetDOF(), (*it)->info_.lower_limit_vector_.begin());
                    std::copy(itupper,itupper+(*it)->GetDOF(), (*it)->info_.upper_limit_vector_.begin());
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( (*it)->IsRevolute(i) && !(*it)->IsCircular(i) ) {
                            // TODO, necessary to set wrap?
                            if( (*it)->info_.lower_limit_vector_.at(i) < -PI || (*it)->info_.upper_limit_vector_.at(i) > PI) {
                                (*it)->SetWrapOffset(0.5f * ((*it)->info_.lower_limit_vector_.at(i) + (*it)->info_.upper_limit_vector_.at(i)),i);
                            }
                            else {
                                (*it)->SetWrapOffset(0,i);
                            }
                        }
                    }
                    break;
                }
            }
            itlower += (*it)->GetDOF();
            itupper += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(lower.size(),==,dofindices.size());
        OPENRAVE_ASSERT_OP(upper.size(),==,dofindices.size());
        for(size_t index = 0; index < dofindices.size(); ++index) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[index]);
            int iaxis = dofindices[index]-pjoint->GetDOFIndex();
            if( pjoint->info_.lower_limit_vector_.at(iaxis) != lower[index] || pjoint->info_.upper_limit_vector_.at(iaxis) != upper[index] ) {
                bChanged = true;
                pjoint->info_.lower_limit_vector_.at(iaxis) = lower[index];
                pjoint->info_.upper_limit_vector_.at(iaxis) = upper[index];
                if( pjoint->IsRevolute(iaxis) && !pjoint->IsCircular(iaxis) ) {
                    // TODO, necessary to set wrap?
                    if( pjoint->info_.lower_limit_vector_.at(iaxis) < -PI || pjoint->info_.upper_limit_vector_.at(iaxis) > PI) {
                        pjoint->SetWrapOffset(0.5f * (pjoint->info_.lower_limit_vector_.at(iaxis) + pjoint->info_.upper_limit_vector_.at(iaxis)),iaxis);
                    }
                    else {
                        pjoint->SetWrapOffset(0,iaxis);
                    }
                }
            }
        }
    }
    if( bChanged ) {
        _PostprocessChangedParameters(Prop_JointLimits);
    }
}

void KinBody::SetDOFVelocityLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.max_velocity_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFAccelerationLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.max_accelerate_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFJerkLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.max_jerk_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardVelocityLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.hard_max_velocity_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardAccelerationLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.hard_max_accelerate_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardJerkLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.hard_max_jerk_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFTorqueLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, dof_ordered_joints_vector_) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->info_.max_torque_vector_.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SimulationStep(dReal fElapsedTime)
{
    _UpdateGrabbedBodies();
}

void KinBody::SubtractDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2, const std::vector<int>& dofindices) const
{
    OPENRAVE_ASSERT_OP(q1.size(), ==, q2.size() );
    if (is_all_joints_1dof_and_no_circular_) {
        for(size_t i = 0; i < q1.size(); ++i) {
            q1[i] -= q2[i];
        }
        return;
    }

    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)q1.size(), ==, GetDOF() );
        FOREACHC(itjoint,joints_vector_) {
            int dof = (*itjoint)->GetDOFIndex();
            for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                if( (*itjoint)->IsCircular(i) ) {
                    q1[dof+i] = utils::NormalizeCircularAngle(q1[dof+i]-q2[dof+i],(*itjoint)->_vcircularlowerlimit.at(i), (*itjoint)->_vcircularupperlimit.at(i));
                }
                else {
                    q1[dof+i] -= q2[dof+i];
                }
            }
        }
    }
    else {
        OPENRAVE_ASSERT_OP(q1.size(), ==, dofindices.size() );
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            if( pjoint->IsCircular(dofindices[i]-pjoint->GetDOFIndex()) ) {
                int iaxis = dofindices[i]-pjoint->GetDOFIndex();
                q1[i] = utils::NormalizeCircularAngle(q1[i]-q2[i], pjoint->_vcircularlowerlimit.at(iaxis), pjoint->_vcircularupperlimit.at(iaxis));
            }
            else {
                q1[i] -= q2[i];
            }
        }
    }
}

// like apply transform except everything is relative to the first frame
void KinBody::SetTransform(const Transform& trans)
{
    if( links_vector_.size() == 0 ) {
        return;
    }
    Transform tbaseinv = links_vector_.front()->GetTransform().inverse();
    Transform tapply = trans * tbaseinv;
    FOREACH(itlink, links_vector_) {
        (*itlink)->SetTransform(tapply * (*itlink)->GetTransform());
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

Transform KinBody::GetTransform() const
{
    return links_vector_.size() > 0 ? links_vector_.front()->GetTransform() : Transform();
}

bool KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if( links_vector_.size() > 0 ) {
        std::vector<std::pair<Vector,Vector> > velocities(links_vector_.size());
        velocities.at(0).first = linearvel;
        velocities.at(0).second = angularvel;
        Vector vlinktrans = links_vector_.at(0)->GetTransform().trans;
        for(size_t i = 1; i < links_vector_.size(); ++i) {
            velocities[i].first = linearvel + angularvel.cross(links_vector_[i]->GetTransform().trans-vlinktrans);
            velocities[i].second = angularvel;
        }

        bool bSuccess = GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
        _UpdateGrabbedBodies();
        return bSuccess;
    }
    return false;
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, const Vector& linearvel, const Vector& angularvel, uint32_t checklimits)
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_OP_FORMAT((int)vDOFVelocities.size(), >=, GetDOF(), "not enough values %d!=%d", vDOFVelocities.size()%GetDOF(),ORE_InvalidArguments);
    std::vector<std::pair<Vector,Vector> > velocities(links_vector_.size());
    velocities.at(0).first = linearvel;
    velocities.at(0).second = angularvel;

    vector<dReal> vlower,vupper,vtempvalues, veval;
    if( checklimits != CLA_Nothing ) {
        GetDOFVelocityLimits(vlower,vupper);
    }

    // have to compute the velocities ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointVelocities(passive_joints_vector_.size());
    for(size_t i = 0; i < vPassiveJointVelocities.size(); ++i) {
        if( !passive_joints_vector_[i]->IsMimic() ) {
            passive_joints_vector_[i]->GetVelocities(vPassiveJointVelocities[i]);
        }
        else {
            vPassiveJointVelocities[i].resize(passive_joints_vector_[i]->GetDOF(),0);
        }
    }

    std::vector<uint8_t> vlinkscomputed(links_vector_.size(),0);
    vlinkscomputed[0] = 1;
    std::array<dReal,3> dummyvalues; // dummy values for a joint

    for(size_t ijoint = 0; ijoint < topologically_sorted_joints_all_vector_.size(); ++ijoint) {
        JointPtr pjoint = topologically_sorted_joints_all_vector_[ijoint];
        int jointindex = topologically_sorted_joint_indices_all_vector_[ijoint];
        int dofindex = pjoint->GetDOFIndex();
        const dReal* pvalues=dofindex >= 0 ? &vDOFVelocities.at(dofindex) : NULL;
        if( pjoint->IsMimic() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        JointPtr pj = itdof->jointindex < (int)joints_vector_.size() ? joints_vector_[itdof->jointindex] : passive_joints_vector_.at(itdof->jointindex-joints_vector_.size());
                        vtempvalues.push_back(pj->GetValue(itdof->axis));
                    }
                    dummyvalues[i] = 0;
                    int err = pjoint->_Eval(i,1,vtempvalues,veval);
                    if( err ) {
                        RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            err = pjoint->_Eval(i,1,vtempvalues,veval);
                        }
                    }
                    else {
                        for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                            dReal partialvelocity;
                            if( vdofformat[ipartial].dofindex >= 0 ) {
                                partialvelocity = vDOFVelocities.at(vdofformat[ipartial].dofindex);
                            }
                            else {
                                partialvelocity = vPassiveJointVelocities.at(vdofformat[ipartial].jointindex-joints_vector_.size()).at(vdofformat[ipartial].axis);
                            }
                            if( ipartial < veval.size() ) {
                                dummyvalues[i] += veval.at(ipartial) * partialvelocity;
                            }
                            else {
                                RAVELOG_DEBUG_FORMAT("cannot evaluate partial velocity for mimic joint %s, perhaps equations don't exist", pjoint->GetName());
                            }
                        }
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = vDOFVelocities.at(dofindex+i); // is this correct? what is a joint has a mimic and non-mimic axis?
                }
                else {
                    // preserve passive joint values
                    dummyvalues[i] = vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(i);
                }
            }
            pvalues = &dummyvalues[0];
        }
        // do the test after mimic computation!
        if( vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] ) {
            continue;
        }
        if( !pvalues ) {
            // has to be a passive joint
            pvalues = &vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(0);
        }

        if( checklimits != CLA_Nothing && dofindex >= 0 ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pvalues[i] < vlower.at(dofindex+i)-g_fEpsilonJointLimit ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("dof %d velocity is not in limits %.15e<%.15e")%(dofindex+i)%pvalues[i]%vlower.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof %d velocity is not in limits %.15e<%.15e"), (dofindex+i)%pvalues[i]%vlower.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vlower[dofindex+i];
                }
                else if( pvalues[i] > vupper.at(dofindex+i)+g_fEpsilonJointLimit ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("dof %d velocity is not in limits %.15e>%.15e")%(dofindex+i)%pvalues[i]%vupper.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof %d velocity is not in limits %.15e>%.15e"), (dofindex+i)%pvalues[i]%vupper.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vupper[dofindex+i];
                }
                else {
                    dummyvalues[i] = pvalues[i];
                }
            }
            pvalues = &dummyvalues[0];
        }

        // compute for global coordinate system
        Vector vparent, wparent;
        Transform tparent;
        if( !pjoint->GetHierarchyParentLink() ) {
            tparent = links_vector_.at(0)->GetTransform();
            vparent = velocities.at(0).first;
            wparent = velocities.at(0).second;
        }
        else {
            tparent = pjoint->GetHierarchyParentLink()->GetTransform();
            vparent = velocities[pjoint->GetHierarchyParentLink()->GetIndex()].first;
            wparent = velocities[pjoint->GetHierarchyParentLink()->GetIndex()].second;
        }

        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Transform tchild = pjoint->GetHierarchyChildLink()->GetTransform();
        Vector xyzdelta = tchild.trans - tparent.trans;
        Transform tdelta = tparent * pjoint->GetInternalHierarchyLeftTransform();
//        if( pjoint->GetType() & JointSpecialBit ) {
//            switch(pjoint->GetType()) {
//            case JointHinge2: {
//                Transform tfirst;
//                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pjoint->GetValue(0));
//                w = pvalues[0]*pjoint->GetInternalHierarchyAxis(0) + tfirst.rotate(pvalues[1]*pjoint->GetInternalHierarchyAxis(1));
//                break;
//            }
//            case JointSpherical:
//                w.x = pvalues[0]; w.y = pvalues[1]; w.z = pvalues[2];
//                break;
//            default:
//                RAVELOG_WARN(str(boost::format("forward kinematic type %d not supported")%pjoint->GetType()));
//                break;
//            }
//        }
//        else {
        if( pjoint->GetType() == JointRevolute ) {
            Vector gw = tdelta.rotate(pvalues[0]*pjoint->GetInternalHierarchyAxis(0));
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans), wparent + gw);
        }
        else if( pjoint->GetType() == JointPrismatic ) {
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + tdelta.rotate(pvalues[0]*pjoint->GetInternalHierarchyAxis(0)), wparent);
        }
        else if( pjoint->GetType() == JointTrajectory ) {
            Transform tlocalvelocity, tlocal;
            if( pjoint->IsMimic(0) ) {
                // vtempvalues should already be init from previous _Eval call
                int err = pjoint->_Eval(0,0,vtempvalues,veval);
                if( err != 0 ) {
                    RAVELOG_WARN(str(boost::format("error with evaluation of joint %s")%pjoint->GetName()));
                }
                dReal fvalue = veval[0];
                if( pjoint->IsCircular(0) ) {
                    fvalue = utils::NormalizeCircularAngle(fvalue,pjoint->_vcircularlowerlimit.at(0), pjoint->_vcircularupperlimit.at(0));
                }
                pjoint->info_._trajfollow->Sample(vtempvalues,fvalue);
            }
            else {
                // calling GetValue() could be extremely slow
                pjoint->info_._trajfollow->Sample(vtempvalues,pjoint->GetValue(0));
            }
            pjoint->info_._trajfollow->GetConfigurationSpecification().ExtractTransform(tlocal, vtempvalues.begin(), KinBodyConstPtr(),0);
            pjoint->info_._trajfollow->GetConfigurationSpecification().ExtractTransform(tlocalvelocity, vtempvalues.begin(), KinBodyConstPtr(),1);
            Vector gw = tdelta.rotate(quatMultiply(tlocalvelocity.rot, quatInverse(tlocal.rot))*2*pvalues[0]); // qvel = [0,axisangle] * qrot * 0.5 * vel
            gw = Vector(gw.y,gw.z,gw.w);
            Vector gv = tdelta.rotate(tlocalvelocity.trans*pvalues[0]);
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans) + gv, wparent + gw);
        }
        else if( pjoint->GetType() == JointSpherical ) {
            Vector gw = tdelta.rotate(Vector(pvalues[0],pvalues[1],pvalues[2]));
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans), wparent + gw);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint 0x%x not supported for querying velocities"),pjoint->GetType(),ORE_Assert);
//                //todo
//                Transform tjoint;
//                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
//                    Transform tdelta;
//                    if( pjoint->IsRevolute(iaxis) ) {
//                        w += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis));
//                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
//                    }
//                    else {
//                        tdelta.trans = pjoint->GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
//                        v += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis)) + w.cross(tdelta.trans);
//                    }
//                    tjoint = tjoint * tdelta;
//                }
        }
//        }


        vlinkscomputed[childindex] = 1;
    }
    SetLinkVelocities(velocities);
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, uint32_t checklimits, const std::vector<int>& dofindices)
{
    Vector linearvel,angularvel;
    links_vector_.at(0)->GetVelocity(linearvel,angularvel);
    if( dofindices.size() == 0 ) {
        return SetDOFVelocities(vDOFVelocities,linearvel,angularvel,checklimits);
    }

    // check if all dofindices are supplied
    if( (int)dofindices.size() == GetDOF() ) {
        bool bordereddof = true;
        for(size_t i = 0; i < dofindices.size(); ++i) {
            if( dofindices[i] != (int)i ) {
                bordereddof = false;
                break;
            }
        }
        if( bordereddof ) {
            return SetDOFVelocities(vDOFVelocities,linearvel,angularvel,checklimits);
        }
    }
    OPENRAVE_ASSERT_OP_FORMAT0(vDOFVelocities.size(),==,dofindices.size(),"index sizes do not match", ORE_InvalidArguments);
    // have to recreate the correct vector
    std::vector<dReal> vfulldof(GetDOF());
    std::vector<int>::const_iterator it;
    for(size_t i = 0; i < dofindices.size(); ++i) {
        it = find(dofindices.begin(), dofindices.end(), i);
        if( it != dofindices.end() ) {
            vfulldof[i] = vDOFVelocities.at(static_cast<size_t>(it-dofindices.begin()));
        }
        else {
            JointPtr pjoint = GetJointFromDOFIndex(i);
            if( !!pjoint ) {
                vfulldof[i] = joints_vector_.at(dof_indices_vector_.at(i))->GetVelocity(i-dof_indices_vector_.at(i));
            }
        }
    }
    return SetDOFVelocities(vfulldof,linearvel,angularvel,checklimits);
}

void KinBody::GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocities(shared_kinbody_const(),velocities);
}

void KinBody::GetLinkTransformations(vector<Transform>& vtrans) const
{
    if( RaveGetDebugLevel() & Level_VerifyPlans ) {
        RAVELOG_VERBOSE("GetLinkTransformations should be called with doflastsetvalues\n");
    }
    vtrans.resize(links_vector_.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = vtrans.begin(), itlink = links_vector_.begin(); it != vtrans.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }
}

void KinBody::GetLinkTransformations(std::vector<Transform>& transforms, std::vector<dReal>& doflastsetvalues) const
{
    transforms.resize(links_vector_.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = transforms.begin(), itlink = links_vector_.begin(); it != transforms.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }

    doflastsetvalues.resize(0);
    if( (int)doflastsetvalues.capacity() < GetDOF() ) {
        doflastsetvalues.reserve(GetDOF());
    }
    FOREACHC(it, dof_ordered_joints_vector_) {
        int toadd = (*it)->GetDOFIndex()-(int)doflastsetvalues.size();
        if( toadd > 0 ) {
            doflastsetvalues.insert(doflastsetvalues.end(),toadd,0);
        }
        else if( toadd < 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof indices mismatch joint %s, toadd=%d"), (*it)->GetName()%toadd, ORE_InvalidState);
        }
        for(int i = 0; i < (*it)->GetDOF(); ++i) {
            doflastsetvalues.push_back((*it)->_doflastsetvalues[i]);
        }
    }
}

void KinBody::GetLinkEnableStates(std::vector<uint8_t>& enablestates) const
{
    enablestates.resize(links_vector_.size());
    for(size_t ilink = 0; ilink < links_vector_.size(); ++ilink) {
        enablestates[ilink] = links_vector_[ilink]->IsEnabled();
    }
}

uint64_t KinBody::GetLinkEnableStatesMask() const
{
    if( links_vector_.size() > 64 ) {
        RAVELOG_WARN_FORMAT("%s has too many links and will only return enable mask for first 64", name_);
    }
    uint64_t linkstate = 0;
    for(size_t ilink = 0; ilink < links_vector_.size(); ++ilink) {
        linkstate |= ((uint64_t)links_vector_[ilink]->info_.is_enabled_<<ilink);
    }
    return linkstate;
}

KinBody::JointPtr KinBody::GetJointFromDOFIndex(int dofindex) const
{
    return joints_vector_.at(dof_indices_vector_.at(dofindex));
}

AABB KinBody::ComputeAABB(bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ab;
    FOREACHC(itlink,links_vector_) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ab = (*itlink)->ComputeAABB();
        if((ab.extents.x == 0)&&(ab.extents.y == 0)&&(ab.extents.z == 0)) {
            continue;
        }
        Vector vnmin = ab.pos - ab.extents;
        Vector vnmax = ab.pos + ab.extents;
        if( !binitialized ) {
            vmin = vnmin;
            vmax = vnmax;
            binitialized = true;
        }
        else {
            if( vmin.x > vnmin.x ) {
                vmin.x = vnmin.x;
            }
            if( vmin.y > vnmin.y ) {
                vmin.y = vnmin.y;
            }
            if( vmin.z > vnmin.z ) {
                vmin.z = vnmin.z;
            }
            if( vmax.x < vnmax.x ) {
                vmax.x = vnmax.x;
            }
            if( vmax.y < vnmax.y ) {
                vmax.y = vnmax.y;
            }
            if( vmax.z < vnmax.z ) {
                vmax.z = vnmax.z;
            }
        }
    }
    if( !binitialized ) {
        ab.pos = GetTransform().trans;
        ab.extents = Vector(0,0,0);
    }
    else {
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
    }
    return ab;
}

AABB KinBody::ComputeAABBFromTransform(const Transform& tBody, bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ablocal;
    Transform tConvertToNewFrame = tBody*GetTransform().inverse();
    FOREACHC(itlink,links_vector_) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ablocal = (*itlink)->ComputeLocalAABB();
        if( ablocal.extents.x == 0 && ablocal.extents.y == 0 && ablocal.extents.z == 0 ) {
            continue;
        }

        Transform tlink = tConvertToNewFrame*(*itlink)->GetTransform();
        TransformMatrix mlink(tlink);
        Vector projectedExtents(RaveFabs(mlink.m[0]*ablocal.extents[0]) + RaveFabs(mlink.m[1]*ablocal.extents[1]) + RaveFabs(mlink.m[2]*ablocal.extents[2]),
                                RaveFabs(mlink.m[4]*ablocal.extents[0]) + RaveFabs(mlink.m[5]*ablocal.extents[1]) + RaveFabs(mlink.m[6]*ablocal.extents[2]),
                                RaveFabs(mlink.m[8]*ablocal.extents[0]) + RaveFabs(mlink.m[9]*ablocal.extents[1]) + RaveFabs(mlink.m[10]*ablocal.extents[2]));
        Vector vWorldPos = tlink * ablocal.pos;

        Vector vnmin = vWorldPos - projectedExtents;
        Vector vnmax = vWorldPos + projectedExtents;
        if( !binitialized ) {
            vmin = vnmin;
            vmax = vnmax;
            binitialized = true;
        }
        else {
            if( vmin.x > vnmin.x ) {
                vmin.x = vnmin.x;
            }
            if( vmin.y > vnmin.y ) {
                vmin.y = vnmin.y;
            }
            if( vmin.z > vnmin.z ) {
                vmin.z = vnmin.z;
            }
            if( vmax.x < vnmax.x ) {
                vmax.x = vnmax.x;
            }
            if( vmax.y < vnmax.y ) {
                vmax.y = vnmax.y;
            }
            if( vmax.z < vnmax.z ) {
                vmax.z = vnmax.z;
            }
        }
    }

    AABB ab;
    if( !binitialized ) {
        ab.pos = GetTransform().trans;
        ab.extents = Vector(0,0,0);
    }
    else {
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
    }
    return ab;
}

AABB KinBody::ComputeLocalAABB(bool bEnabledOnlyLinks) const
{
    return ComputeAABBFromTransform(Transform(), bEnabledOnlyLinks);
}

Vector KinBody::GetCenterOfMass() const
{
    // find center of mass and set the outer transform to it
    Vector center;
    dReal fTotalMass = 0;

    FOREACHC(itlink, links_vector_) {
        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 ) {
        center /= fTotalMass;
    }
    return center;
}

void KinBody::SetLinkTransformations(const std::vector<Transform>& vbodies)
{
    if( RaveGetDebugLevel() & Level_VerifyPlans ) {
        RAVELOG_WARN("SetLinkTransformations should be called with doflastsetvalues, re-setting all values\n");
    }
    else {
        RAVELOG_DEBUG("SetLinkTransformations should be called with doflastsetvalues, re-setting all values\n");
    }
    OPENRAVE_ASSERT_OP_FORMAT(vbodies.size(), >=, links_vector_.size(), "not enough links %d<%d", vbodies.size()%links_vector_.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = vbodies.begin(), itlink = links_vector_.begin(); it != vbodies.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,joints_vector_) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_doflastsetvalues[i] = (*itjoint)->GetValue(i);
        }
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

void KinBody::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues)
{
    OPENRAVE_ASSERT_OP_FORMAT(transforms.size(), >=, links_vector_.size(), "not enough links %d<%d", transforms.size()%links_vector_.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = transforms.begin(), itlink = links_vector_.begin(); it != transforms.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,joints_vector_) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_doflastsetvalues[i] = doflastsetvalues.at((*itjoint)->GetDOFIndex()+i);
        }
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

void KinBody::SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities)
{
    GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
    _UpdateGrabbedBodies();
}

void KinBody::SetLinkEnableStates(const std::vector<uint8_t>& enablestates)
{
    OPENRAVE_ASSERT_OP(enablestates.size(),==,links_vector_.size());
    bool bchanged = false;
    for(size_t ilink = 0; ilink < enablestates.size(); ++ilink) {
        bool bEnable = enablestates[ilink]!=0;
        if( links_vector_[ilink]->info_.is_enabled_ != bEnable ) {
            links_vector_[ilink]->info_.is_enabled_ = bEnable;
            non_adjacent_link_cache_ &= ~AO_Enabled;
            bchanged = true;
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transBase, uint32_t checklimits)
{
    if( links_vector_.size() == 0 ) {
        return;
    }
    Transform tbase = transBase*links_vector_.at(0)->GetTransform().inverse();
    links_vector_.at(0)->SetTransform(transBase);

    // apply the relative transformation to all links!! (needed for passive joints)
    for(size_t i = 1; i < links_vector_.size(); ++i) {
        links_vector_[i]->SetTransform(tbase*links_vector_[i]->GetTransform());
    }
    SetDOFValues(vJointValues,checklimits);
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t checklimits, const std::vector<int>& dofindices)
{
    CHECK_INTERNAL_COMPUTATION;
    if( vJointValues.size() == 0 || links_vector_.size() == 0) {
        return;
    }
    int expecteddof = dofindices.size() > 0 ? (int)dofindices.size() : GetDOF();
    OPENRAVE_ASSERT_OP_FORMAT((int)vJointValues.size(),>=,expecteddof, "not enough values %d<%d", vJointValues.size()%GetDOF(),ORE_InvalidArguments);

    const dReal* pJointValues = &vJointValues[0];
    if( checklimits != CLA_Nothing || dofindices.size() > 0 ) {
        _vTempJoints.resize(GetDOF());
        if( dofindices.size() > 0 ) {
            // user only set a certain number of indices, so have to fill the temporary array with the full set of values first
            // and then overwrite with the user set values
            GetDOFValues(_vTempJoints);
            for(size_t i = 0; i < dofindices.size(); ++i) {
                _vTempJoints.at(dofindices[i]) = pJointValues[i];
            }
            pJointValues = &_vTempJoints[0];
        }
        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<dReal> upperlim, lowerlim;
        FOREACHC(it, joints_vector_) {
            const dReal* p = pJointValues+(*it)->GetDOFIndex();
            if( checklimits == CLA_Nothing ) {
                // limits should not be checked, so just copy
                for(int i = 0; i < (*it)->GetDOF(); ++i) {
                    *ptempjoints++ = p[i];
                }
                continue;
            }
            OPENRAVE_ASSERT_OP( (*it)->GetDOF(), <=, 3 );
            (*it)->GetLimits(lowerlim, upperlim);
            if( (*it)->GetType() == JointSpherical ) {
                dReal fcurang = fmod(RaveSqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]),2*PI);
                if( fcurang < lowerlim[0] ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = lowerlim[0]; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = lowerlim[0]/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else if( fcurang > upperlim[0] ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = upperlim[0]; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = upperlim[0]/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else {
                    *ptempjoints++ = p[0]; *ptempjoints++ = p[1]; *ptempjoints++ = p[2];
                }
            }
            else {
                for(int i = 0; i < (*it)->GetDOF(); ++i) {
                    if( (*it)->IsCircular(i) ) {
                        // don't normalize since user is expecting the values he sets are exactly returned via GetDOFValues
                        *ptempjoints++ = p[i]; //utils::NormalizeCircularAngle(p[i],(*it)->_vcircularlowerlimit[i],(*it)->_vcircularupperlimit[i]);
                    }
                    else {
                        if( p[i] < lowerlim[i] ) {
                            if( p[i] < lowerlim[i]-g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("dof %d value %e is smaller than the lower limit %e")%((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof %d value %e is smaller than the lower limit %e"), ((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i], ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = lowerlim[i];
                        }
                        else if( p[i] > upperlim[i] ) {
                            if( p[i] > upperlim[i]+g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("dof %d value %e is greater than the upper limit %e")%((*it)->GetDOFIndex()+i)%p[i]%upperlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("dof %d value %e is greater than the upper limit %e"),((*it)->GetDOFIndex()+i)%p[i]%upperlim[i], ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = upperlim[i];
                        }
                        else {
                            *ptempjoints++ = p[i];
                        }
                    }
                }
            }
        }
        pJointValues = &_vTempJoints[0];
    }

    std::array<dReal,3> dummyvalues; // dummy values for a joint
    std::vector<dReal> vtempvalues, veval;

    // have to compute the angles ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointValues(passive_joints_vector_.size());
    for(size_t i = 0; i < vPassiveJointValues.size(); ++i) {
        if( !passive_joints_vector_[i]->IsMimic() ) {
            passive_joints_vector_[i]->GetValues(vPassiveJointValues[i]);
            // check if out of limits!
            for(size_t j = 0; j < vPassiveJointValues[i].size(); ++j) {
                if( !passive_joints_vector_[i]->IsCircular(j) ) {
                    if( vPassiveJointValues[i][j] < passive_joints_vector_[i]->info_.lower_limit_vector_.at(j) ) {
                        if( vPassiveJointValues[i][j] < passive_joints_vector_[i]->info_.lower_limit_vector_.at(j)-5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of lower limit! %e < %e\n")%passive_joints_vector_[i]->info_.lower_limit_vector_.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = passive_joints_vector_[i]->info_.lower_limit_vector_.at(j);
                    }
                    else if( vPassiveJointValues[i][j] > passive_joints_vector_[i]->info_.upper_limit_vector_.at(j) ) {
                        if( vPassiveJointValues[i][j] > passive_joints_vector_[i]->info_.upper_limit_vector_.at(j)+5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of upper limit! %e > %e\n")%passive_joints_vector_[i]->info_.upper_limit_vector_.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = passive_joints_vector_[i]->info_.upper_limit_vector_.at(j);
                    }
                }
            }
        }
        else {
            vPassiveJointValues[i].reserve(passive_joints_vector_[i]->GetDOF()); // do not resize so that we can catch hierarchy errors
        }
    }

    std::vector<uint8_t> vlinkscomputed(links_vector_.size(),0);
    vlinkscomputed[0] = 1;

    for(size_t ijoint = 0; ijoint < topologically_sorted_joints_all_vector_.size(); ++ijoint) {
        JointPtr pjoint = topologically_sorted_joints_all_vector_[ijoint];
        int jointindex = topologically_sorted_joint_indices_all_vector_[ijoint];
        int dofindex = pjoint->GetDOFIndex();
        const dReal* pvalues=dofindex >= 0 ? pJointValues + dofindex : NULL;
        if( pjoint->IsMimic() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        if( itdof->dofindex >= 0 ) {
                            vtempvalues.push_back(pJointValues[itdof->dofindex]);
                        }
                        else {
                            vtempvalues.push_back(vPassiveJointValues.at(itdof->jointindex-joints_vector_.size()).at(itdof->axis));
                        }
                    }
                    int err = pjoint->_Eval(i, 0, vtempvalues, veval);
                    if( err ) {
                        RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
                    }
                    else {
                        vector<dReal> vevalcopy = veval;
                        vector<dReal>::iterator iteval = veval.begin();
                        while(iteval != veval.end()) {
                            bool removevalue = false;
                            if( pjoint->GetType() == JointSpherical || pjoint->IsCircular(i) ) {
                            }
                            else if( *iteval < pjoint->info_.lower_limit_vector_[i] ) {
                                if(*iteval >= pjoint->info_.lower_limit_vector_[i]-g_fEpsilonJointLimit ) {
                                    *iteval = pjoint->info_.lower_limit_vector_[i];
                                }
                                else {
                                    removevalue=true;
                                }
                            }
                            else if( *iteval > pjoint->info_.upper_limit_vector_[i] ) {
                                if(*iteval <= pjoint->info_.upper_limit_vector_[i]+g_fEpsilonJointLimit ) {
                                    *iteval = pjoint->info_.upper_limit_vector_[i];
                                }
                                else {
                                    removevalue=true;
                                }
                            }

                            if( removevalue ) {
                                iteval = veval.erase(iteval); // invalid value so remove from candidates
                            }
                            else {
                                ++iteval;
                            }
                        }

                        if( veval.empty() ) {
                            FORIT(iteval,vevalcopy) {
                                if( checklimits == CLA_Nothing || pjoint->GetType() == JointSpherical || pjoint->IsCircular(i) ) {
                                    veval.push_back(*iteval);
                                }
                                else if( *iteval < pjoint->info_.lower_limit_vector_[i]-g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(pjoint->info_.lower_limit_vector_[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("joint %s: lower limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->info_.lower_limit_vector_[i]%*iteval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint %s: lower limit (%e) is not followed: %e"), pjoint->GetName()%pjoint->info_.lower_limit_vector_[i]%*iteval, ORE_InvalidArguments);
                                    }
                                }
                                else if( *iteval > pjoint->info_.upper_limit_vector_[i]+g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(pjoint->info_.upper_limit_vector_[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("joint %s: upper limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->info_.upper_limit_vector_[i]%*iteval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint %s: upper limit (%e) is not followed: %e"), pjoint->GetName()%pjoint->info_.upper_limit_vector_[i]%*iteval, ORE_InvalidArguments);
                                    }
                                }
                                else {
                                    veval.push_back(*iteval);
                                }
                            }
                            OPENRAVE_ASSERT_FORMAT(!veval.empty(), "no valid values for joint %s", pjoint->GetName(),ORE_Assert);
                        }
                        if( veval.size() > 1 ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                            ss << "multiplie values for joint " << pjoint->GetName() << ": ";
                            FORIT(iteval,veval) {
                                ss << *iteval << " ";
                            }
                            RAVELOG_WARN(ss.str());
                        }
                        dummyvalues[i] = veval.at(0);
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointValues.at(jointindex-(int)joints_vector_.size()).resize(pjoint->GetDOF());
                        vPassiveJointValues.at(jointindex-(int)joints_vector_.size()).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = pvalues[dofindex+i]; // is this correct? what is a joint has a mimic and non-mimic axis?
                }
                else {
                    // preserve passive joint values
                    dummyvalues[i] = vPassiveJointValues.at(jointindex-(int)joints_vector_.size()).at(i);
                }
            }
            pvalues = &dummyvalues[0];
        }
        // do the test after mimic computation!
        if( vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] ) {
            continue;
        }
        if( !pvalues ) {
            // has to be a passive joint
            pvalues = &vPassiveJointValues.at(jointindex-(int)joints_vector_.size()).at(0);
        }

        Transform tjoint;
        if( pjoint->GetType() & JointSpecialBit ) {
            switch(pjoint->GetType()) {
            case JointHinge2: {
                Transform tfirst;
                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
                Transform tsecond;
                tsecond.rot = quatFromAxisAngle(tfirst.rotate(pjoint->GetInternalHierarchyAxis(1)), pvalues[1]);
                tjoint = tsecond * tfirst;
                pjoint->_doflastsetvalues[0] = pvalues[0];
                pjoint->_doflastsetvalues[1] = pvalues[1];
                break;
            }
            case JointSpherical: {
                dReal fang = pvalues[0]*pvalues[0]+pvalues[1]*pvalues[1]+pvalues[2]*pvalues[2];
                if( fang > 0 ) {
                    fang = RaveSqrt(fang);
                    dReal fiang = 1/fang;
                    tjoint.rot = quatFromAxisAngle(Vector(pvalues[0]*fiang,pvalues[1]*fiang,pvalues[2]*fiang),fang);
                }
                break;
            }
            case JointTrajectory: {
                vector<dReal> vdata;
                tjoint = Transform();
                dReal fvalue = pvalues[0];
                if( pjoint->IsCircular(0) ) {
                    // need to normalize the value
                    fvalue = utils::NormalizeCircularAngle(fvalue,pjoint->_vcircularlowerlimit.at(0), pjoint->_vcircularupperlimit.at(0));
                }
                pjoint->info_._trajfollow->Sample(vdata,fvalue);
                if( !pjoint->info_._trajfollow->GetConfigurationSpecification().ExtractTransform(tjoint,vdata.begin(),KinBodyConstPtr()) ) {
                    RAVELOG_WARN(str(boost::format("trajectory sampling for joint %s failed")%pjoint->GetName()));
                }
                pjoint->_doflastsetvalues[0] = 0;
                break;
            }
            default:
                RAVELOG_WARN(str(boost::format("forward kinematic type 0x%x not supported")%pjoint->GetType()));
                break;
            }
        }
        else {
            if( pjoint->GetType() == JointRevolute ) {
                tjoint.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
                pjoint->_doflastsetvalues[0] = pvalues[0];
            }
            else if( pjoint->GetType() == JointPrismatic ) {
                tjoint.trans = pjoint->GetInternalHierarchyAxis(0) * pvalues[0];
            }
            else {
                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
                    Transform tdelta;
                    if( pjoint->IsRevolute(iaxis) ) {
                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
                        pjoint->_doflastsetvalues[iaxis] = pvalues[iaxis];
                    }
                    else {
                        tdelta.trans = pjoint->GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
                    }
                    tjoint = tjoint * tdelta;
                }
            }
        }

        Transform t = pjoint->GetInternalHierarchyLeftTransform() * tjoint * pjoint->GetInternalHierarchyRightTransform();
        if( !pjoint->GetHierarchyParentLink() ) {
            t = links_vector_.at(0)->GetTransform() * t;
        }
        else {
            t = pjoint->GetHierarchyParentLink()->GetTransform() * t;
        }
        pjoint->GetHierarchyChildLink()->SetTransform(t);
        vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] = 1;
    }

    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

bool KinBody::IsDOFRevolute(int dofindex) const
{
    int jointindex = dof_indices_vector_.at(dofindex);
    return joints_vector_.at(jointindex)->IsRevolute(dofindex-joints_vector_.at(jointindex)->GetDOFIndex());
}

bool KinBody::IsDOFPrismatic(int dofindex) const
{
    int jointindex = dof_indices_vector_.at(dofindex);
    return joints_vector_.at(jointindex)->IsPrismatic(dofindex-joints_vector_.at(jointindex)->GetDOFIndex());
}

KinBody::LinkPtr KinBody::GetLink(const std::string& linkname) const
{
    for(std::vector<LinkPtr>::const_iterator it = links_vector_.begin(); it != links_vector_.end(); ++it) {
        if ( (*it)->GetName() == linkname ) {
            return *it;
        }
    }
    return LinkPtr();
}

const std::vector<KinBody::JointPtr>& KinBody::GetDependencyOrderedJoints() const
{
    CHECK_INTERNAL_COMPUTATION;
    return topologically_sorted_joints_vector_;
}

const std::vector< std::vector< std::pair<KinBody::LinkPtr, KinBody::JointPtr> > >& KinBody::GetClosedLoops() const
{
    CHECK_INTERNAL_COMPUTATION;
    return closed_loops_vector_;
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<JointPtr>& vjoints) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(linkindex1>=0 && linkindex1<(int)links_vector_.size(), "body %s linkindex1 %d invalid (num links %d)", GetName()%linkindex1%links_vector_.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex2>=0 && linkindex2<(int)links_vector_.size(), "body %s linkindex2 %d invalid (num links %d)", GetName()%linkindex2%links_vector_.size(), ORE_InvalidArguments);
    vjoints.resize(0);
    if( linkindex1 == linkindex2 ) {
        return true;
    }
    int offset = linkindex2*links_vector_.size();
    int curlink = linkindex1;
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        vjoints.push_back(jointindex < (int)joints_vector_.size() ? joints_vector_.at(jointindex) : passive_joints_vector_.at(jointindex-joints_vector_.size()));
        int prevlink = curlink;
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
        OPENRAVE_ASSERT_OP(prevlink,!=,curlink); // avoid loops
    }
    return vjoints.size()>0; // otherwise disconnected
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<LinkPtr>& vlinks) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(linkindex1>=0 && linkindex1<(int)links_vector_.size(), "body %s linkindex1 %d invalid (num links %d)", GetName()%linkindex1%links_vector_.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex2>=0 && linkindex2<(int)links_vector_.size(), "body %s linkindex2 %d invalid (num links %d)", GetName()%linkindex2%links_vector_.size(), ORE_InvalidArguments);
    vlinks.resize(0);
    int offset = linkindex2*links_vector_.size();
    int curlink = linkindex1;
    if( all_pairs_shortest_paths_vector_[offset+curlink].first < 0 ) {
        return false;
    }
    vlinks.push_back(links_vector_.at(linkindex1));
    if( linkindex1 == linkindex2 ) {
        return true;
    }
    while(all_pairs_shortest_paths_vector_[offset+curlink].first != linkindex2) {
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
        if( curlink < 0 ) {
            vlinks.resize(0);
            return false;
        }
        vlinks.push_back(links_vector_.at(curlink));
    }
    vlinks.push_back(links_vector_.at(linkindex2));
    return true; // otherwise disconnected
}

bool KinBody::IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const
{
    CHECK_INTERNAL_COMPUTATION0;
    int jointindex = dof_indices_vector_.at(dofindex);
    return (DoesAffect(jointindex,linkindex1)==0) != (DoesAffect(jointindex,linkindex2)==0);
}

int KinBody::GetJointIndex(const std::string& jointname) const
{
    int index = 0;
    FOREACHC(it,joints_vector_) {
        if ((*it)->GetName() == jointname ) {
            return index;
        }
        ++index;
    }
    return -1;
}

KinBody::JointPtr KinBody::GetJoint(const std::string& jointname) const
{
    FOREACHC(it,joints_vector_) {
        if ((*it)->GetName() == jointname ) {
            return *it;
        }
    }
    FOREACHC(it,passive_joints_vector_) {
        if ((*it)->GetName() == jointname ) {
            return *it;
        }
    }
    return JointPtr();
}

void KinBody::ComputeJacobianTranslation(int linkindex, const Vector& position, vector<dReal>& vjacobian,const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%links_vector_.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    vjacobian.resize(3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);

    Vector v;
    int offset = linkindex*links_vector_.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        if( jointindex < (int)joints_vector_.size() ) {
            // active joint
            JointPtr pjoint = joints_vector_.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect != 0 ) {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof).cross(position-pjoint->GetAnchor());
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else {
                        RAVELOG_WARN("ComputeJacobianTranslation joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            size_t index = itindex-dofindices.begin();
                            vjacobian[index] += v.x; vjacobian[index+dofstride] += v.y; vjacobian[index+2*dofstride] += v.z;
                        }
                    }
                    else {
                        vjacobian[dofindex+dof] += v.x; vjacobian[dofstride+dofindex+dof] += v.y; vjacobian[2*dofstride+dofindex+dof] += v.z;
                    }
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = passive_joints_vector_.at(jointindex-joints_vector_.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxis = pjoint->GetAxis(idof).cross(position-pjoint->GetAnchor());
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vaxis = pjoint->GetAxis(idof);
                        }
                        else {
                            RAVELOG_WARN("ComputeJacobianTranslation joint %d not supported\n", pjoint->GetType());
                            continue;
                        }
                        pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                        FOREACH(itpartial,vpartials) {
                            Vector v = vaxis * itpartial->second;
                            int index = itpartial->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index = itindex-dofindices.begin();
                            }
                            vjacobian[index] += v.x;
                            vjacobian[dofstride+index] += v.y;
                            vjacobian[2*dofstride+index] += v.z;
                        }
                    }
                }
            }
        }
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
    }
}

void KinBody::CalculateJacobian(int linkindex, const Vector& trans, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianTranslation(linkindex,trans,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, std::vector<dReal>& vjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%links_vector_.size(),ORE_InvalidArguments);
    int dofstride = GetDOF();
    vjacobian.resize(4*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);
    Vector v;
    int offset = linkindex*links_vector_.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        if( jointindex < (int)joints_vector_.size() ) {
            // active joint
            JointPtr pjoint = joints_vector_.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%links_vector_.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        v = Vector(0,0,0);
                    }
                    else {
                        RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", pjoint->GetType());
                        v = Vector(0,0,0);
                    }
                    vjacobian[dofindex+dof] += dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                    vjacobian[dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                    vjacobian[2*dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                    vjacobian[3*dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = passive_joints_vector_.at(jointindex-joints_vector_.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    Vector vaxis;
                    if( pjoint->IsRevolute(idof) ) {
                        vaxis = pjoint->GetAxis(idof);
                    }
                    else if( pjoint->IsPrismatic(idof) ) {
                        vaxis = Vector(0,0,0);
                    }
                    else {
                        RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                    FOREACH(itpartial,vpartials) {
                        int dofindex = itpartial->first;
                        Vector v = vaxis * itpartial->second;
                        vjacobian[dofindex] += dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                        vjacobian[dofstride+dofindex] += dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                        vjacobian[2*dofstride+dofindex] += dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                        vjacobian[3*dofstride+dofindex] += dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
                    }
                }
            }
        }
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[4][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateRotationJacobian(linkindex,q,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,4*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::ComputeJacobianAxisAngle(int linkindex, std::vector<dReal>& vjacobian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%links_vector_.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    vjacobian.resize(3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);

    Vector v, anchor, axis;
    int offset = linkindex*links_vector_.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        if( jointindex < (int)joints_vector_.size() ) {
            // active joint
            JointPtr pjoint = joints_vector_.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect != 0 ) {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        continue;
                    }
                    else {
                        RAVELOG_WARN("ComputeJacobianAxisAngle joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            size_t index = itindex-dofindices.begin();
                            vjacobian[index] += v.x; vjacobian[index+dofstride] += v.y; vjacobian[index+2*dofstride] += v.z;
                        }
                    }
                    else {
                        vjacobian[dofindex+dof] += v.x; vjacobian[dofstride+dofindex+dof] += v.y; vjacobian[2*dofstride+dofindex+dof] += v.z;
                    }
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = passive_joints_vector_.at(jointindex-joints_vector_.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxis = pjoint->GetAxis(idof);
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            continue;
                        }
                        else {
                            RAVELOG_WARN("ComputeJacobianAxisAngle joint %d not supported\n", pjoint->GetType());
                            continue;
                        }
                        pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                        FOREACH(itpartial,vpartials) {
                            Vector v = vaxis * itpartial->second;
                            int index = itpartial->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index = itindex-dofindices.begin();
                            }
                            vjacobian[index] += v.x;
                            vjacobian[dofstride+index] += v.y;
                            vjacobian[2*dofstride+index] += v.z;
                        }
                    }
                }
            }
        }
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
    }
}

void KinBody::CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianAxisAngle(linkindex,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::ComputeHessianTranslation(int linkindex, const Vector& position, std::vector<dReal>& hessian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%links_vector_.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    hessian.resize(dofstride*3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(hessian.begin(),hessian.end(),0);

    int offset = linkindex*links_vector_.size();
    int curlink = 0;
    std::vector<Vector> vaxes, vjacobian; vaxes.reserve(dofstride); vjacobian.reserve(dofstride);
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    std::vector<int> vinsertedindices; vinsertedindices.reserve(dofstride);
    typedef std::pair< std::vector<Vector>, std::vector<std::pair<int,dReal> > > PartialInfo;
    std::map<size_t, PartialInfo > mappartialsinserted; // if vinsertedindices has -1, that index will be here
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        if( jointindex < (int)joints_vector_.size() ) {
            // active joint
            JointPtr pjoint = joints_vector_.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%links_vector_.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    size_t index = dofindex+dof;
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            index = itindex-dofindices.begin();
                        }
                        else {
                            continue;
                        }
                    }

                    if( pjoint->IsRevolute(dof) ) {
                        vaxes.push_back(pjoint->GetAxis(dof));
                        vjacobian.push_back(pjoint->GetAxis(dof).cross(position-pjoint->GetAnchor()));
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        vaxes.push_back(Vector());
                        vjacobian.push_back(pjoint->GetAxis(dof));
                    }
                    else {
                        vaxes.push_back(Vector());
                        vjacobian.push_back(Vector());
                        RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                    }
                    vinsertedindices.push_back(index);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = passive_joints_vector_.at(jointindex-joints_vector_.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxes.push_back(pjoint->GetAxis(idof));
                            vjacobian.push_back(pjoint->GetAxis(idof).cross(position-pjoint->GetAnchor()));
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vjacobian.push_back(pjoint->GetAxis(idof));
                            vaxes.push_back(Vector());
                        }
                        else {
                            vaxes.push_back(Vector());
                            vjacobian.push_back(Vector());
                            RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                        }
                        PartialInfo& partialinfo = mappartialsinserted[vinsertedindices.size()];
                        partialinfo.first.resize(vinsertedindices.size());
                        pjoint->_ComputePartialVelocities(partialinfo.second,idof,mapcachedpartials);
                        vinsertedindices.push_back(-1);
                    }
                }
            }
        }
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
    }

    for(size_t i = 0; i < vaxes.size(); ++i) {
        if( vinsertedindices[i] < 0 ) {
            PartialInfo& partialinfo = mappartialsinserted[i];
            FOREACH(itpartial,partialinfo.second) {
                int index = itpartial->first;
                if( dofindices.size() > 0 ) {
                    std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                    if( itindex == dofindices.end() ) {
                        continue;
                    }
                    index = itindex-dofindices.begin();
                }

                for(size_t j = 0; j < i; ++j) {
                    Vector v = partialinfo.first.at(j)*itpartial->second;
                    if( vinsertedindices[j] < 0 ) {
                        //RAVELOG_WARN("hessian unhandled condition with mimic\n");
                        PartialInfo& partialinfo2 = mappartialsinserted[j];
                        FOREACH(itpartial2,partialinfo2.second) {
                            int index2 = itpartial2->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index2 = itindex-dofindices.begin();
                            }

                            Vector v2 = v*itpartial2->second;
                            size_t indexoffset = 3*dofstride*index2+index;
                            hessian[indexoffset+0] += v2.x;
                            hessian[indexoffset+dofstride] += v2.y;
                            hessian[indexoffset+2*dofstride] += v2.z;
                            if( j != i ) {
                                // symmetric
                                indexoffset = 3*dofstride*index+index2;
                                hessian[indexoffset+0] += v2.x;
                                hessian[indexoffset+dofstride] += v2.y;
                                hessian[indexoffset+2*dofstride] += v2.z;
                            }
                        }
                    }
                    else {
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }

                for(size_t j = i; j < vaxes.size(); ++j) {
                    Vector v = vaxes[i].cross(vjacobian[j]);
                    if( j == i ) {
                        dReal f = itpartial->second*itpartial->second;
                        size_t indexoffset = 3*dofstride*index+index;
                        hessian[indexoffset+0] += v.x*f;
                        hessian[indexoffset+dofstride] += v.y*f;
                        hessian[indexoffset+2*dofstride] += v.z*f;
                        continue;
                    }

                    if( vinsertedindices[j] < 0 ) {
                        // only add the first time, do not multiply by itpartial->second yet?
                        if( itpartial == partialinfo.second.begin() ) {
                            mappartialsinserted[j].first.at(i) += v; // will get to it later
                        }
                    }
                    else {
                        v *= itpartial->second;
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }
            }
        }
        else {
            size_t ioffset = 3*dofstride*vinsertedindices[i];
            for(size_t j = i; j < vaxes.size(); ++j) {
                Vector v = vaxes[i].cross(vjacobian[j]);
                if( vinsertedindices[j] < 0 ) {
                    mappartialsinserted[j].first.at(i) = v; // we'll get to it later
                }
                else {
                    size_t indexoffset = ioffset+vinsertedindices[j];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                    if( j != i ) {
                        // symmetric
                        indexoffset = 3*dofstride*vinsertedindices[j]+vinsertedindices[i];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                    }
                }
            }
        }
    }
}

void KinBody::ComputeHessianAxisAngle(int linkindex, std::vector<dReal>& hessian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%links_vector_.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    hessian.resize(dofstride*3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(hessian.begin(),hessian.end(),0);

    int offset = linkindex*links_vector_.size();
    int curlink = 0;
    std::vector<Vector> vaxes; vaxes.reserve(dofstride);
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    std::vector<int> vinsertedindices; vinsertedindices.reserve(dofstride);
    typedef std::pair< std::vector<Vector>, std::vector<std::pair<int,dReal> > > PartialInfo;
    std::map<size_t, PartialInfo > mappartialsinserted; // if vinsertedindices has -1, that index will be here
    while(all_pairs_shortest_paths_vector_[offset+curlink].first>=0) {
        int jointindex = all_pairs_shortest_paths_vector_[offset+curlink].second;
        if( jointindex < (int)joints_vector_.size() ) {
            // active joint
            JointPtr pjoint = joints_vector_.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%links_vector_.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    size_t index = dofindex+dof;
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            index = itindex-dofindices.begin();
                        }
                        else {
                            continue;
                        }
                    }

                    if( pjoint->IsRevolute(dof) ) {
                        vaxes.push_back(pjoint->GetAxis(dof));
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        vaxes.push_back(Vector());
                    }
                    else {
                        vaxes.push_back(Vector());
                        RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                    }
                    vinsertedindices.push_back(index);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = passive_joints_vector_.at(jointindex-joints_vector_.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxes.push_back(pjoint->GetAxis(idof));
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vaxes.push_back(Vector());
                        }
                        else {
                            vaxes.push_back(Vector());
                            RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                        }
                        PartialInfo& partialinfo = mappartialsinserted[vinsertedindices.size()];
                        partialinfo.first.resize(vinsertedindices.size());
                        pjoint->_ComputePartialVelocities(partialinfo.second,idof,mapcachedpartials);
                        vinsertedindices.push_back(-1);
                    }
                }
            }
        }
        curlink = all_pairs_shortest_paths_vector_[offset+curlink].first;
    }

    for(size_t i = 0; i < vaxes.size(); ++i) {
        if( vinsertedindices[i] < 0 ) {
            PartialInfo& partialinfo = mappartialsinserted[i];
            FOREACH(itpartial,partialinfo.second) {
                int index = itpartial->first;
                if( dofindices.size() > 0 ) {
                    std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                    if( itindex == dofindices.end() ) {
                        continue;
                    }
                    index = itindex-dofindices.begin();
                }

                for(size_t j = 0; j < i; ++j) {
                    Vector v = partialinfo.first.at(j)*itpartial->second;
                    if( vinsertedindices[j] < 0 ) {
                        //RAVELOG_WARN("hessian unhandled condition with mimic\n");
                        PartialInfo& partialinfo2 = mappartialsinserted[j];
                        FOREACH(itpartial2,partialinfo2.second) {
                            int index2 = itpartial2->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index2 = itindex-dofindices.begin();
                            }

                            Vector v2 = v*itpartial2->second;
                            size_t indexoffset = 3*dofstride*index2+index;
                            hessian[indexoffset+0] += v2.x;
                            hessian[indexoffset+dofstride] += v2.y;
                            hessian[indexoffset+2*dofstride] += v2.z;
                            if( j != i ) {
                                // symmetric
                                indexoffset = 3*dofstride*index+index2;
                                hessian[indexoffset+0] += v2.x;
                                hessian[indexoffset+dofstride] += v2.y;
                                hessian[indexoffset+2*dofstride] += v2.z;
                            }
                        }
                    }
                    else {
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }

                for(size_t j = i+1; j < vaxes.size(); ++j) {
                    Vector v = vaxes[i].cross(vaxes[j]);
                    if( j == i ) {
                        dReal f = itpartial->second*itpartial->second;
                        size_t indexoffset = 3*dofstride*index+index;
                        hessian[indexoffset+0] += v.x*f;
                        hessian[indexoffset+dofstride] += v.y*f;
                        hessian[indexoffset+2*dofstride] += v.z*f;
                        continue;
                    }

                    if( vinsertedindices[j] < 0 ) {
                        // only add the first time, do not multiply by itpartial->second yet?
                        if( itpartial == partialinfo.second.begin() ) {
                            mappartialsinserted[j].first.at(i) += v; // will get to it later
                        }
                    }
                    else {
                        v *= itpartial->second;
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        // symmetric
                        indexoffset = 3*dofstride*vinsertedindices[j]+index;
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                    }
                }
            }
        }
        else {
            size_t ioffset = 3*dofstride*vinsertedindices[i];
            for(size_t j = i+1; j < vaxes.size(); ++j) {
                Vector v = vaxes[i].cross(vaxes[j]);
                if( vinsertedindices[j] < 0 ) {
                    mappartialsinserted[j].first.at(i) = v; // we'll get to it later
                }
                else {
                    size_t indexoffset = ioffset+vinsertedindices[j];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                    // symmetric
                    indexoffset = 3*dofstride*vinsertedindices[j]+vinsertedindices[i];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                }
            }
        }
    }
}

void KinBody::ComputeInverseDynamics(std::vector<dReal>& doftorques, const std::vector<dReal>& vDOFAccelerations, const KinBody::ForceTorqueMap& mapExternalForceTorque) const
{
    CHECK_INTERNAL_COMPUTATION;
    doftorques.resize(GetDOF());
    if( joints_vector_.size() == 0 ) {
        return;
    }

    Vector vgravity = GetEnv()->GetPhysicsEngine()->GetGravity();
    std::vector<dReal> vDOFVelocities;
    std::vector<pair<Vector, Vector> > vLinkVelocities, vLinkAccelerations; // linear, angular
    _ComputeDOFLinkVelocities(vDOFVelocities, vLinkVelocities);
    // check if all velocities are 0, if yes, then can simplify some computations since only have contributions from dofacell and external forces
    bool bHasVelocity = false;
    FOREACH(it,vDOFVelocities) {
        if( RaveFabs(*it) > g_fEpsilonLinear ) {
            bHasVelocity = true;
            break;
        }
    }
    if( !bHasVelocity ) {
        vDOFVelocities.resize(0);
    }
    AccelerationMap externalaccelerations;
    externalaccelerations[0] = make_pair(-vgravity, Vector());
    AccelerationMapPtr pexternalaccelerations(&externalaccelerations, utils::null_deleter());
    _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, pexternalaccelerations);

    // all valuess are in the global coordinate system
    // Given the velocity/acceleration of the object is on point A, to change to B do:
    // v_B = v_A + angularvel x (B-A)
    // a_B = a_A + angularaccel x (B-A) + angularvel x (angularvel x (B-A))
    // forward recursion
    std::vector<Vector> vLinkCOMLinearAccelerations(links_vector_.size()), vLinkCOMMomentOfInertia(links_vector_.size());
    for(size_t i = 0; i < vLinkVelocities.size(); ++i) {
        Vector vglobalcomfromlink = links_vector_.at(i)->GetGlobalCOM() - links_vector_.at(i)->info_.transform_.trans;
        Vector vangularaccel = vLinkAccelerations.at(i).second;
        Vector vangularvelocity = vLinkVelocities.at(i).second;
        vLinkCOMLinearAccelerations[i] = vLinkAccelerations.at(i).first + vangularaccel.cross(vglobalcomfromlink) + vangularvelocity.cross(vangularvelocity.cross(vglobalcomfromlink));
        TransformMatrix tm = links_vector_.at(i)->GetGlobalInertia();
        vLinkCOMMomentOfInertia[i] = tm.rotate(vangularaccel) + vangularvelocity.cross(tm.rotate(vangularvelocity));
    }

    // backward recursion
    std::vector< std::pair<Vector, Vector> > vLinkForceTorques(links_vector_.size());
    FOREACHC(it,mapExternalForceTorque) {
        vLinkForceTorques.at(it->first) = it->second;
    }
    std::fill(doftorques.begin(),doftorques.end(),0);

    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;

    // go backwards
    for(size_t ijoint = 0; ijoint < topologically_sorted_joints_all_vector_.size(); ++ijoint) {
        JointPtr pjoint = topologically_sorted_joints_all_vector_.at(topologically_sorted_joints_all_vector_.size()-1-ijoint);
        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Vector vcomforce = vLinkCOMLinearAccelerations[childindex]*pjoint->GetHierarchyChildLink()->GetMass() + vLinkForceTorques.at(childindex).first;
        Vector vjointtorque = vLinkForceTorques.at(childindex).second + vLinkCOMMomentOfInertia.at(childindex);

        if( !!pjoint->GetHierarchyParentLink() ) {
            Vector vchildcomtoparentcom = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetHierarchyParentLink()->GetGlobalCOM();
            int parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
            vLinkForceTorques.at(parentindex).first += vcomforce;
            vLinkForceTorques.at(parentindex).second += vjointtorque + vchildcomtoparentcom.cross(vcomforce);
        }

        Vector vcomtoanchor = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetAnchor();
        if( pjoint->GetDOFIndex() >= 0 ) {
            if( pjoint->GetType() == JointHinge ) {
                doftorques.at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
            }
            else if( pjoint->GetType() == JointSlider ) {
                doftorques.at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
            }

            dReal fFriction = 0; // torque due to friction
            dReal fRotorAccelerationTorque = 0; // torque due to accelerating motor rotor (and gear)
            // see if any friction needs to be added. Only add if the velocity is non-zero since with zero velocity do not know the exact torque on the joint...
            if( !!pjoint->info_.electric_motor_info_ ) {
                const ElectricMotorActuatorInfoPtr pActuatorInfo = pjoint->info_.electric_motor_info_;
                if( pjoint->GetDOFIndex() < (int)vDOFVelocities.size() ) {
                    if( vDOFVelocities.at(pjoint->GetDOFIndex()) > g_fEpsilonLinear ) {
                        fFriction += pActuatorInfo->coloumb_friction;
                    }
                    else if( vDOFVelocities.at(pjoint->GetDOFIndex()) < -g_fEpsilonLinear ) {
                        fFriction -= pActuatorInfo->coloumb_friction;
                    }
                    fFriction += vDOFVelocities.at(pjoint->GetDOFIndex())*pActuatorInfo->viscous_friction;

                    if (pActuatorInfo->rotor_inertia > 0.0) {
                        // converting inertia on motor side to load side requires multiplying by gear ratio squared because inertia unit is mass * distance^2
                        const dReal fInertiaOnLoadSide = pActuatorInfo->rotor_inertia * pActuatorInfo->gear_ratio * pActuatorInfo->gear_ratio;
                        fRotorAccelerationTorque += vDOFAccelerations.at(pjoint->GetDOFIndex()) * fInertiaOnLoadSide;
                    }
                }

                doftorques.at(pjoint->GetDOFIndex()) += fFriction + fRotorAccelerationTorque;
            }
        }
        else if( pjoint->IsMimic(0) ) {
            // passive joint, so have to transfer the torque to its dependent joints.
            // TODO if there's more than one dependent joint, how do we split?
            dReal faxistorque;
            if( pjoint->GetType() == JointHinge ) {
                faxistorque = pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
            }
            else if( pjoint->GetType() == JointSlider ) {
                faxistorque = pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
            }

            if( !!pjoint->info_.electric_motor_info_ ) {
                // TODO how to process this correctly? what is velocity of this joint? pjoint->GetVelocity(0)?
            }

            pjoint->_ComputePartialVelocities(vpartials,0,mapcachedpartials);
            FOREACH(itpartial,vpartials) {
                int dofindex = itpartial->first;
                doftorques.at(dofindex) += itpartial->second*faxistorque;
            }
        }
        else {
            // joint should be static
            OPENRAVE_ASSERT_FORMAT(pjoint->IsStatic(), "joint %s (%d) is expected to be static", pjoint->GetName()%ijoint, ORE_Assert);
        }
    }
}

void KinBody::ComputeInverseDynamics(std::array< std::vector<dReal>, 3>& vDOFTorqueComponents, const std::vector<dReal>& vDOFAccelerations, const KinBody::ForceTorqueMap& mapExternalForceTorque) const
{
    CHECK_INTERNAL_COMPUTATION;
    FOREACH(itdoftorques,vDOFTorqueComponents) {
        itdoftorques->resize(GetDOF());
    }
    if( joints_vector_.size() == 0 ) {
        return;
    }

    Vector vgravity = GetEnv()->GetPhysicsEngine()->GetGravity();
    std::vector<dReal> vDOFVelocities;
    std::array< std::vector<pair<Vector, Vector> >, 3> vLinkVelocities; // [0] = all zeros, [1] = dof velocities only, [2] = only velocities due to base link
    std::array< std::vector<pair<Vector, Vector> >, 3> vLinkAccelerations; // [0] = dofaccel only, [1] = dofvel only, [2] - gravity + external only (dofaccel=0, dofvel=0)
    std::array<int,3> linkaccelsimilar = {{-1,-1,-1}}; // used for tracking which vLinkAccelerations indices are similar to each other (to avoid computation)

    vLinkVelocities[0].resize(links_vector_.size());
    _ComputeDOFLinkVelocities(vDOFVelocities, vLinkVelocities[1], false);
    // check if all velocities are 0, if yes, then can simplify some computations since only have contributions from dofacell and external forces
    bool bHasVelocity = false;
    FOREACH(it,vDOFVelocities) {
        if( RaveFabs(*it) > g_fEpsilonLinear ) {
            bHasVelocity = true;
            break;
        }
    }
    if( !bHasVelocity ) {
        vDOFVelocities.resize(0);
    }

    AccelerationMap externalaccelerations;
    externalaccelerations[0] = make_pair(-vgravity, Vector());
    AccelerationMapPtr pexternalaccelerations(&externalaccelerations, utils::null_deleter());

    // all valuess are in the global coordinate system
    // try to compute as little as possible by checking what is non-zero
    Vector vbaselinear, vbaseangular;
    links_vector_.at(0)->GetVelocity(vbaselinear,vbaseangular);
    bool bHasGravity = vgravity.lengthsqr3() > g_fEpsilonLinear*g_fEpsilonLinear;
    bool bHasBaseLinkAccel = vbaseangular.lengthsqr3() > g_fEpsilonLinear*g_fEpsilonLinear;
    if( bHasBaseLinkAccel || bHasGravity ) {
        if( bHasBaseLinkAccel ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            vLinkVelocities[2].resize(links_vector_.size());
            Vector vbasepos = links_vector_.at(0)->info_.transform_.trans;
            for(size_t i = 1; i < vLinkVelocities[0].size(); ++i) {
                Vector voffset = links_vector_.at(i)->info_.transform_.trans - vbasepos;
                vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
                vLinkVelocities[2][i].second = vbaseangular;
            }
        }
        else {
            vLinkVelocities[2] = vLinkVelocities[0];
        }
        _ComputeLinkAccelerations(std::vector<dReal>(), std::vector<dReal>(), vLinkVelocities[2], vLinkAccelerations[2], pexternalaccelerations);
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1]);
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
        }
    }
    else {
        // no external forces
        vLinkVelocities[2] = vLinkVelocities[0];
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1]);
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
        }
    }

    std::array< std::vector<Vector>, 3> vLinkCOMLinearAccelerations, vLinkCOMMomentOfInertia;
    std::array< std::vector< std::pair<Vector, Vector> >, 3> vLinkForceTorques;
    for(size_t j = 0; j < 3; ++j) {
        if( vLinkAccelerations[j].size() > 0 ) {
            vLinkCOMLinearAccelerations[j].resize(links_vector_.size());
            vLinkCOMMomentOfInertia[j].resize(links_vector_.size());
            vLinkForceTorques[j].resize(links_vector_.size());
        }
    }

    for(size_t i = 0; i < links_vector_.size(); ++i) {
        Vector vglobalcomfromlink = links_vector_.at(i)->GetGlobalCOM() - links_vector_.at(i)->info_.transform_.trans;
        TransformMatrix tm = links_vector_.at(i)->GetGlobalInertia();
        for(size_t j = 0; j < 3; ++j) {
            if( vLinkAccelerations[j].size() > 0 ) {
                Vector vangularaccel = vLinkAccelerations[j].at(i).second;
                Vector vangularvelocity = vLinkVelocities[j].at(i).second;
                vLinkCOMLinearAccelerations[j][i] = vLinkAccelerations[j].at(i).first + vangularaccel.cross(vglobalcomfromlink) + vangularvelocity.cross(vangularvelocity.cross(vglobalcomfromlink));
                vLinkCOMMomentOfInertia[j][i] = tm.rotate(vangularaccel) + vangularvelocity.cross(tm.rotate(vangularvelocity));
            }
        }
    }

    FOREACH(itdoftorques,vDOFTorqueComponents) {
        std::fill(itdoftorques->begin(),itdoftorques->end(),0);
    }

    // backward recursion
    vLinkForceTorques[2].resize(links_vector_.size());
    FOREACHC(it,mapExternalForceTorque) {
        vLinkForceTorques[2].at(it->first) = it->second;
    }

    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;

    // go backwards
    for(size_t ijoint = 0; ijoint < topologically_sorted_joints_all_vector_.size(); ++ijoint) {
        JointPtr pjoint = topologically_sorted_joints_all_vector_.at(topologically_sorted_joints_all_vector_.size()-1-ijoint);
        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Vector vchildcomtoparentcom;
        int parentindex = -1;
        if( !!pjoint->GetHierarchyParentLink() ) {
            vchildcomtoparentcom = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetHierarchyParentLink()->GetGlobalCOM();
            parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
        }

        bool bIsMimic = pjoint->GetDOFIndex() < 0 && pjoint->IsMimic(0);
        if( bIsMimic ) {
            pjoint->_ComputePartialVelocities(vpartials,0,mapcachedpartials);
        }

        dReal mass = pjoint->GetHierarchyChildLink()->GetMass();
        Vector vcomtoanchor = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetAnchor();
        for(size_t j = 0; j < 3; ++j) {
            if( vLinkForceTorques[j].size() == 0 ) {
                continue;
            }
            Vector vcomforce = vLinkForceTorques[j].at(childindex).first;
            Vector vjointtorque = vLinkForceTorques[j].at(childindex).second;
            if( vLinkCOMLinearAccelerations[j].size() > 0 ) {
                vcomforce += vLinkCOMLinearAccelerations[j][childindex]*mass;
                vjointtorque += vLinkCOMMomentOfInertia[j].at(childindex);
            }

            if( parentindex >= 0 ) {
                vLinkForceTorques[j].at(parentindex).first += vcomforce;
                vLinkForceTorques[j].at(parentindex).second += vjointtorque + vchildcomtoparentcom.cross(vcomforce);
            }

            if( pjoint->GetDOFIndex() >= 0 ) {
                if( pjoint->GetType() == JointHinge ) {
                    vDOFTorqueComponents[j].at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
                }
                else if( pjoint->GetType() == JointSlider ) {
                    vDOFTorqueComponents[j].at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
                }
            }
            else if( bIsMimic ) {
                // passive joint, so have to transfer the torque to its dependent joints.
                // TODO if there's more than one dependent joint, how do we split?
                dReal faxistorque;
                if( pjoint->GetType() == JointHinge ) {
                    faxistorque = pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
                }
                else if( pjoint->GetType() == JointSlider ) {
                    faxistorque = pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
                }

                FOREACH(itpartial,vpartials) {
                    int dofindex = itpartial->first;
                    vDOFTorqueComponents[j].at(dofindex) += itpartial->second*faxistorque;
                }
            }
            else {
                // joint should be static
                BOOST_ASSERT(pjoint->IsStatic());
            }
        }
    }
}

void KinBody::GetLinkAccelerations(const std::vector<dReal>&vDOFAccelerations, std::vector<std::pair<Vector,Vector> >&vLinkAccelerations, AccelerationMapConstPtr externalaccelerations) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( links_vector_.size() == 0 ) {
        vLinkAccelerations.resize(0);
    }
    else {
        std::vector<dReal> vDOFVelocities;
        std::vector<pair<Vector, Vector> > vLinkVelocities;
        _ComputeDOFLinkVelocities(vDOFVelocities,vLinkVelocities);
        _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, externalaccelerations);
    }
}

void KinBody::_ComputeDOFLinkVelocities(std::vector<dReal>& dofvelocities, std::vector<std::pair<Vector,Vector> >& vLinkVelocities, bool usebaselinkvelocity) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocities(shared_kinbody_const(),vLinkVelocities);
    if( links_vector_.size() <= 1 ) {
        dofvelocities.resize(GetDOF());
        if( !usebaselinkvelocity && links_vector_.size() > 0 ) {
            vLinkVelocities[0].first = Vector();
            vLinkVelocities[0].second = Vector();
        }
        return;
    }
    if( !usebaselinkvelocity ) {
        Vector vbasepos = links_vector_.at(0)->info_.transform_.trans;
        // v_B = v_A + angularvel x (B-A)
        for(size_t i = 1; i < vLinkVelocities.size(); ++i) {
            Vector voffset = links_vector_.at(i)->info_.transform_.trans - vbasepos;
            vLinkVelocities[i].first -= vLinkVelocities[0].first + vLinkVelocities[0].second.cross(voffset);
            vLinkVelocities[i].second -= vLinkVelocities[0].second;
        }
        vLinkVelocities[0].first = Vector();
        vLinkVelocities[0].second = Vector();
    }
    dofvelocities.resize(0);
    if( (int)dofvelocities.capacity() < GetDOF() ) {
        dofvelocities.reserve(GetDOF());
    }
    FOREACHC(it, dof_ordered_joints_vector_) {
        int parentindex = 0;
        if( !!(*it)->attached_bodies_array_[0] ) {
            parentindex = (*it)->attached_bodies_array_[0]->GetIndex();
        }
        int childindex = (*it)->attached_bodies_array_[0]->GetIndex();
        (*it)->_GetVelocities(dofvelocities,true,vLinkVelocities.at(parentindex),vLinkVelocities.at(childindex));
    }
}

void KinBody::_ComputeLinkAccelerations(const std::vector<dReal>& vDOFVelocities, const std::vector<dReal>& vDOFAccelerations, const std::vector< std::pair<Vector, Vector> >& vLinkVelocities, std::vector<std::pair<Vector,Vector> >& vLinkAccelerations, AccelerationMapConstPtr pexternalaccelerations) const
{
    vLinkAccelerations.resize(links_vector_.size());
    if( links_vector_.size() == 0 ) {
        return;
    }

    vector<dReal> vtempvalues, veval;
    std::array<dReal,3> dummyvelocities = {{0,0,0}}, dummyaccelerations={{0,0,0}}; // dummy values for a joint

    // set accelerations of all links as if they were the base link
    for(size_t ilink = 0; ilink < vLinkAccelerations.size(); ++ilink) {
        vLinkAccelerations.at(ilink).first += vLinkVelocities.at(ilink).second.cross(vLinkVelocities.at(ilink).first);
        vLinkAccelerations.at(ilink).second = Vector();
    }

    if( !!pexternalaccelerations ) {
        FOREACHC(itaccel, *pexternalaccelerations) {
            vLinkAccelerations.at(itaccel->first).first += itaccel->second.first;
            vLinkAccelerations.at(itaccel->first).second += itaccel->second.second;
        }
    }

    // have to compute the velocities and accelerations ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointVelocities(passive_joints_vector_.size()), vPassiveJointAccelerations(passive_joints_vector_.size());
    for(size_t i = 0; i <passive_joints_vector_.size(); ++i) {
        if( vDOFAccelerations.size() > 0 ) {
            vPassiveJointAccelerations[i].resize(passive_joints_vector_[i]->GetDOF(),0);
        }
        if( vDOFVelocities.size() > 0 ) {
            if( !passive_joints_vector_[i]->IsMimic() ) {
                passive_joints_vector_[i]->GetVelocities(vPassiveJointVelocities[i]);
            }
            else {
                vPassiveJointVelocities[i].resize(passive_joints_vector_[i]->GetDOF(),0);
            }
        }
    }

    Transform tdelta;
    Vector vlocalaxis;
    std::vector<uint8_t> vlinkscomputed(links_vector_.size(),0);
    vlinkscomputed[0] = 1;

    // compute the link accelerations going through topological order
    for(size_t ijoint = 0; ijoint < topologically_sorted_joints_all_vector_.size(); ++ijoint) {
        JointPtr pjoint = topologically_sorted_joints_all_vector_[ijoint];
        int jointindex = topologically_sorted_joint_indices_all_vector_[ijoint];
        int dofindex = pjoint->GetDOFIndex();

        // have to compute the partial accelerations for each mimic dof
        const dReal* pdofaccelerations=NULL, *pdofvelocities=NULL;
        if( dofindex >= 0 ) {
            if( vDOFAccelerations.size() ) {
                pdofaccelerations = &vDOFAccelerations.at(dofindex);
            }
            if( vDOFVelocities.size() > 0 ) {
                pdofvelocities=&vDOFVelocities.at(dofindex);
            }
        }
        if( pjoint->IsMimic() && (vDOFAccelerations.size() > 0 || vDOFVelocities.size() > 0) ) {
            // compute both partial velocity and acceleration information
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        JointPtr pj = itdof->jointindex < (int)joints_vector_.size() ? joints_vector_[itdof->jointindex] : passive_joints_vector_.at(itdof->jointindex-joints_vector_.size());
                        vtempvalues.push_back(pj->GetValue(itdof->axis));
                    }
                    dummyvelocities[i] = 0;
                    dummyaccelerations[i] = 0;

                    // velocity
                    if( vDOFVelocities.size() > 0 ) {
                        int err = pjoint->_Eval(i,1,vtempvalues,veval);
                        if( err ) {
                            RAVELOG_WARN_FORMAT("failed to evaluate joint %s, fparser error %d", pjoint->GetName()%err);
                        }
                        else {
                            for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                                dReal partialvelocity;
                                if( vdofformat[ipartial].dofindex >= 0 ) {
                                    partialvelocity = vDOFVelocities.at(vdofformat[ipartial].dofindex);
                                }
                                else {
                                    partialvelocity = vPassiveJointVelocities.at(vdofformat[ipartial].jointindex-joints_vector_.size()).at(vdofformat[ipartial].axis);
                                }
                                if( ipartial < veval.size() ) {
                                    dummyvelocities[i] += veval.at(ipartial) * partialvelocity;
                                }
                                else {
                                    RAVELOG_DEBUG_FORMAT("cannot evaluate partial velocity for mimic joint %s, perhaps equations don't exist", pjoint->GetName());
                                }
                            }
                        }
                        // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                        if( dofindex < 0 ) {
                            vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(i) = dummyvelocities[i];
                        }
                    }

                    // acceleration
                    if( vDOFAccelerations.size() > 0 ) {
                        int err = pjoint->_Eval(i,2,vtempvalues,veval);
                        if( err ) {
                            RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
                        }
                        else {
                            for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                                dReal partialacceleration;
                                if( vdofformat[ipartial].dofindex >= 0 ) {
                                    partialacceleration = vDOFAccelerations.at(vdofformat[ipartial].dofindex);
                                }
                                else {
                                    partialacceleration = vPassiveJointAccelerations.at(vdofformat[ipartial].jointindex-joints_vector_.size()).at(vdofformat[ipartial].axis);
                                }
                                if( ipartial < veval.size() ) {
                                    dummyaccelerations[i] += veval.at(ipartial) * partialacceleration;
                                }
                                else {
                                    RAVELOG_DEBUG_FORMAT("cannot evaluate partial acceleration for mimic joint %s, perhaps equations don't exist", pjoint->GetName());
                                }
                            }
                        }
                        // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                        if( dofindex < 0 ) {
                            vPassiveJointAccelerations.at(jointindex-(int)joints_vector_.size()).at(i) = dummyaccelerations[i];
                        }
                    }
                }
                else if( dofindex >= 0 ) {
                    // is this correct? what is a joint has a mimic and non-mimic axis?
                    dummyvelocities[i] = vDOFVelocities.at(dofindex+i);
                    dummyaccelerations[i] = vDOFAccelerations.at(dofindex+i);
                }
                else {
                    // preserve passive joint values
                    dummyvelocities[i] = vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(i);
                    dummyaccelerations[i] = vPassiveJointAccelerations.at(jointindex-(int)joints_vector_.size()).at(i);
                }
            }
            pdofvelocities = &dummyvelocities[0];
            pdofaccelerations = &dummyaccelerations[0];
        }

        // do the test after mimic computation!?
        if( vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] ) {
            continue;
        }

        if( vDOFVelocities.size() > 0 && !pdofvelocities ) {
            // has to be a passive joint
            pdofvelocities = &vPassiveJointVelocities.at(jointindex-(int)joints_vector_.size()).at(0);
        }
        if( vDOFAccelerations.size() > 0 && !pdofaccelerations ) {
            // has to be a passive joint
            pdofaccelerations = &vPassiveJointAccelerations.at(jointindex-(int)joints_vector_.size()).at(0);
        }

        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        const Transform& tchild = pjoint->GetHierarchyChildLink()->GetTransform();
        const pair<Vector, Vector>& vChildVelocities = vLinkVelocities.at(childindex);
        pair<Vector, Vector>& vChildAccelerations = vLinkAccelerations.at(childindex);

        int parentindex = 0;
        if( !!pjoint->GetHierarchyParentLink() ) {
            parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
        }

        const pair<Vector, Vector>& vParentVelocities = vLinkVelocities.at(parentindex);
        const pair<Vector, Vector>& vParentAccelerations = vLinkAccelerations.at(parentindex);
        Vector xyzdelta = tchild.trans - links_vector_.at(parentindex)->info_.transform_.trans;
        if( !!pdofaccelerations || !!pdofvelocities ) {
            tdelta = links_vector_.at(parentindex)->info_.transform_ * pjoint->GetInternalHierarchyLeftTransform();
            vlocalaxis = pjoint->GetInternalHierarchyAxis(0);
        }

        // check out: http://en.wikipedia.org/wiki/Rotating_reference_frame
        // compute for global coordinate system
        // code for symbolic computation (python sympy)
        // t=Symbol('t'); q=Function('q')(t); dq=diff(q,t); axis=Matrix(3,1,symbols('ax,ay,az')); delta=Matrix(3,1,[Function('dx')(t), Function('dy')(t), Function('dz')(t)]); vparent=Matrix(3,1,[Function('vparentx')(t),Function('vparenty')(t),Function('vparentz')(t)]); wparent=Matrix(3,1,[Function('wparentx')(t),Function('wparenty')(t),Function('wparentz')(t)]); Mparent=Matrix(3,4,[Function('m%d%d'%(i,j))(t) for i in range(3) for j in range(4)]); c = Matrix(3,1,symbols('cx,cy,cz'))
        // hinge joints:
        // p = Mparent[0:3,3] + Mparent[0:3,0:3]*(Left[0:3,3] + Left[0:3,0:3]*Rot[0:3,0:3]*Right[0:3,3])
        // v = vparent + wparent.cross(p-Mparent[0:3,3]) + Mparent[0:3,0:3] * Left[0:3,0:3] * dq * axis.cross(Rot[0:3,0:3]*Right)
        // wparent.cross(v) = wparent.cross(vparent) + wparent.cross(wparent.cross(p-Mparent[0:3,3])) + wparent.cross(p-Mparent[0:3,3])
        // dv = vparent.diff(t) + wparent.diff(t).cross(p-Mparent[0:3,3]).transpose() + wparent.cross(v-vparent) + wparent.cross(v-vparent-wparent.cross(wparent.cross(p-Mparent[0:3,3]))) + Mparent[0:3,0:3] * Left[0:3,0:3] * (ddq * axis.cross(Rot[0:3,0:3]*Right) + dq * axis.cross(dq*axis.cross(Rot[0:3,0:3]*Right)))
        // w = wparent + Mparent[0:3,0:3]*Left[0:3,0:3]*axis*dq
        // dw = wparent.diff(t) + wparent.cross(Mparent[0:3,0:3]*Left[0:3,0:3]*axis*dq).transpose() + Mparent[0:3,0:3]*Left[0:3,0:3]*axis*ddq
        // slider:
        // v = vparent + wparent.cross(p-Mparent[0:3,3]) + Mparent[0:3,0:3]*Left[0:3,0:3]*dq*axis
        // dv = vparent.diff(t) + wparent.diff(t).cross(p-Mparent[0:3,3]).transpose() + wparent.cross(v-vparent) + wparent.cross(Mparent[0:3,0:3]*Left[0:3,0:3]*dq*axis) + Mparent[0:3,0:3]*Left[0:3,0:3]*ddq*axis
        // w = wparent
        // dw = wparent.diff(t)
        if( pjoint->GetType() == JointRevolute ) {
            vChildAccelerations.first = vParentAccelerations.first + vParentAccelerations.second.cross(xyzdelta) + vParentVelocities.second.cross((vChildVelocities.first-vParentVelocities.first)*2-vParentVelocities.second.cross(xyzdelta));
            vChildAccelerations.second = vParentAccelerations.second;
            if( !!pdofvelocities ) {
                Vector gw = tdelta.rotate(vlocalaxis*pdofvelocities[0]);
                vChildAccelerations.first += gw.cross(gw.cross(tchild.trans-tdelta.trans));
                vChildAccelerations.second += vParentVelocities.second.cross(gw);
            }
            if( !!pdofaccelerations ) {
                Vector gdw = tdelta.rotate(vlocalaxis*pdofaccelerations[0]);
                vChildAccelerations.first += gdw.cross(tchild.trans-tdelta.trans);
                vChildAccelerations.second += gdw;
            }
        }
        else if( pjoint->GetType() == JointPrismatic ) {
            Vector w = tdelta.rotate(vlocalaxis);
            vChildAccelerations.first = vParentAccelerations.first + vParentAccelerations.second.cross(xyzdelta);
            Vector angularveloctiycontrib = vChildVelocities.first-vParentVelocities.first;
            if( !!pdofvelocities ) {
                angularveloctiycontrib += w*pdofvelocities[0];
            }
            vChildAccelerations.first += vParentVelocities.second.cross(angularveloctiycontrib);
            if( !!pdofaccelerations ) {
                vChildAccelerations.first += w*pdofaccelerations[0];
            }
            vChildAccelerations.second = vParentAccelerations.second;
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint type 0x%x not supported for getting link acceleration"),pjoint->GetType(),ORE_Assert);
        }
        vlinkscomputed[childindex] = 1;
    }
}

void KinBody::SetSelfCollisionChecker(CollisionCheckerBasePtr collisionchecker)
{
    if( self_collision_checker_ != collisionchecker ) {
        self_collision_checker_ = collisionchecker;
        // reset the internal cache
        _ResetInternalCollisionCache();
        if( !!self_collision_checker_ && self_collision_checker_ != GetEnv()->GetCollisionChecker() ) {
            // collision checking will not be automatically updated with environment calls, so need to do this manually
            self_collision_checker_->InitKinBody(shared_kinbody());
        }
    }
}

CollisionCheckerBasePtr KinBody::GetSelfCollisionChecker() const
{
    return self_collision_checker_;
}


void KinBody::_ComputeInternalInformation()
{
    uint64_t starttime = utils::GetMicroTime();
    hierarchy_computed_ = 1;

    int lindex=0;
    FOREACH(itlink,links_vector_) {
        (*itlink)->index_ = lindex; // always reset, necessary since index cannot be initialized by custom links
        (*itlink)->parent_links_vector_.clear();
        if((links_vector_.size() > 1)&&((*itlink)->GetName().size() == 0)) {
            RAVELOG_WARN(str(boost::format("%s link index %d has no name")%GetName()%lindex));
        }
        lindex++;
    }

    {
        // move any enabled passive joints to the regular joints list
        vector<JointPtr>::iterator itjoint = passive_joints_vector_.begin();
        while(itjoint != passive_joints_vector_.end()) {
            bool bmimic = false;
            for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                if( !!(*itjoint)->_vmimic[idof] ) {
                    bmimic = true;
                }
            }
            if( !bmimic && (*itjoint)->info_.is_active_ ) {
                joints_vector_.push_back(*itjoint);
                itjoint = passive_joints_vector_.erase(itjoint);
            }
            else {
                ++itjoint;
            }
        }
        // move any mimic joints to the passive joints
        itjoint = joints_vector_.begin();
        while(itjoint != joints_vector_.end()) {
            bool bmimic = false;
            for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                if( !!(*itjoint)->_vmimic[idof] ) {
                    bmimic = true;
                    break;
                }
            }
            if( bmimic || !(*itjoint)->info_.is_active_) {
                passive_joints_vector_.push_back(*itjoint);
                itjoint = joints_vector_.erase(itjoint);
            }
            else {
                ++itjoint;
            }
        }
        int jointindex=0;
        int dofindex=0;
        FOREACH(itjoint,joints_vector_) {
            (*itjoint)->jointindex = jointindex++;
            (*itjoint)->dofindex = dofindex;
            (*itjoint)->info_.is_active_ = true;
            dofindex += (*itjoint)->GetDOF();
        }
        FOREACH(itjoint,passive_joints_vector_) {
            (*itjoint)->jointindex = -1;
            (*itjoint)->dofindex = -1;
            (*itjoint)->info_.is_active_ = false;
        }
    }

    vector<size_t> vorder(joints_vector_.size());
    vector<int> vJointIndices(joints_vector_.size());
    dof_indices_vector_.resize(GetDOF());
    for(size_t i = 0; i < joints_vector_.size(); ++i) {
        vJointIndices[i] = joints_vector_[i]->dofindex;
        for(int idof = 0; idof < joints_vector_[i]->GetDOF(); ++idof) {
            dof_indices_vector_.at(vJointIndices[i]+idof) = i;
        }
        vorder[i] = i;
    }
    sort(vorder.begin(), vorder.end(), utils::index_cmp<vector<int>&>(vJointIndices));
    dof_ordered_joints_vector_.resize(0);
    FOREACH(it,vorder) {
        dof_ordered_joints_vector_.push_back(joints_vector_.at(*it));
    }

    try {
        // initialize all the mimic equations
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? passive_joints_vector_ : joints_vector_;
            FOREACH(itjoint,vjoints) {
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    if( !!(*itjoint)->_vmimic[i] ) {
                        std::string poseq = (*itjoint)->_vmimic[i]->_equations[0], veleq = (*itjoint)->_vmimic[i]->_equations[1], acceleq = (*itjoint)->_vmimic[i]->_equations[2]; // have to copy since memory can become invalidated
                        (*itjoint)->SetMimicEquations(i,poseq,veleq,acceleq);
                    }
                }
            }
        }
        // fill Mimic::_vmimicdofs, check that there are no circular dependencies between the mimic joints
        std::map<Mimic::DOFFormat, std::shared_ptr<Mimic> > mapmimic;
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? passive_joints_vector_ : joints_vector_;
            int jointindex=0;
            FOREACH(itjoint,vjoints) {
                Mimic::DOFFormat dofformat;
                if( ijoints ) {
                    dofformat.dofindex = -1;
                    dofformat.jointindex = jointindex+(int)joints_vector_.size();
                }
                else {
                    dofformat.dofindex = (*itjoint)->GetDOFIndex();
                    dofformat.jointindex = (*itjoint)->GetJointIndex();
                }
                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    dofformat.axis = idof;
                    if( !!(*itjoint)->_vmimic[idof] ) {
                        // only add if depends on mimic joints
                        FOREACH(itdofformat,(*itjoint)->_vmimic[idof]->_vdofformat) {
                            JointPtr pjoint = itdofformat->GetJoint(*this);
                            if( pjoint->IsMimic(itdofformat->axis) ) {
                                mapmimic[dofformat] = (*itjoint)->_vmimic[idof];
                                break;
                            }
                        }
                    }
                }
                ++jointindex;
            }
        }
        bool bchanged = true;
        while(bchanged) {
            bchanged = false;
            FOREACH(itmimic,mapmimic) {
                std::shared_ptr<Mimic> mimic = itmimic->second;
                Mimic::DOFHierarchy h;
                h.dofformatindex = 0;
                FOREACH(itdofformat,mimic->_vdofformat) {
                    if( mapmimic.find(*itdofformat) == mapmimic.end() ) {
                        continue; // this is normal, just means that the parent is a regular dof
                    }
                    std::shared_ptr<Mimic> mimicparent = mapmimic[*itdofformat];
                    FOREACH(itmimicdof, mimicparent->_vmimicdofs) {
                        if( mimicparent->_vdofformat[itmimicdof->dofformatindex] == itmimic->first ) {
                            JointPtr pjoint = itmimic->first.GetJoint(*this);
                            JointPtr pjointparent = itdofformat->GetJoint(*this);
                            throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint index %s uses a mimic joint %s that also depends on %s! this is not allowed"), pjoint->GetName()%pjointparent->GetName()%pjoint->GetName(), ORE_Failed);
                        }
                        h.dofindex = itmimicdof->dofindex;
                        if( find(mimic->_vmimicdofs.begin(),mimic->_vmimicdofs.end(),h) == mimic->_vmimicdofs.end() ) {
                            mimic->_vmimicdofs.push_back(h);
                            bchanged = true;
                        }
                    }
                    ++h.dofformatindex;
                }
            }
        }
    }
    catch(const std::exception& ex) {
        RAVELOG_ERROR(str(boost::format("failed to set mimic equations on kinematics body %s: %s\n")%GetName()%ex.what()));
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? passive_joints_vector_ : joints_vector_;
            FOREACH(itjoint,vjoints) {
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    (*itjoint)->_vmimic[i].reset();
                }
            }
        }
    }

    topologically_sorted_joints_vector_.resize(0);
    topologically_sorted_joints_all_vector_.resize(0);
    topologically_sorted_joint_indices_all_vector_.resize(0);
    joints_affecting_links_vector_.resize(joints_vector_.size()*links_vector_.size());

    // compute the all-pairs shortest paths
    {
        all_pairs_shortest_paths_vector_.resize(links_vector_.size()*links_vector_.size());
        FOREACH(it,all_pairs_shortest_paths_vector_) {
            it->first = -1;
            it->second = -1;
        }
        vector<uint32_t> vcosts(links_vector_.size()*links_vector_.size(),0x3fffffff); // initialize to 2^30-1 since we'll be adding
        for(size_t i = 0; i < links_vector_.size(); ++i) {
            vcosts[i*links_vector_.size()+i] = 0;
        }
        FOREACHC(itjoint,joints_vector_) {
            if( !!(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() ) {
                int index = (*itjoint)->GetFirstAttached()->GetIndex()*links_vector_.size()+(*itjoint)->GetSecondAttached()->GetIndex();
                all_pairs_shortest_paths_vector_[index] = std::pair<int16_t,int16_t>((*itjoint)->GetFirstAttached()->GetIndex(),(*itjoint)->GetJointIndex());
                vcosts[index] = 1;
                index = (*itjoint)->GetSecondAttached()->GetIndex()*links_vector_.size()+(*itjoint)->GetFirstAttached()->GetIndex();
                all_pairs_shortest_paths_vector_[index] = std::pair<int16_t,int16_t>((*itjoint)->GetSecondAttached()->GetIndex(),(*itjoint)->GetJointIndex());
                vcosts[index] = 1;
            }
        }
        int jointindex = (int)joints_vector_.size();
        FOREACHC(itjoint,passive_joints_vector_) {
            if( !!(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() ) {
                int index = (*itjoint)->GetFirstAttached()->GetIndex()*links_vector_.size()+(*itjoint)->GetSecondAttached()->GetIndex();
                all_pairs_shortest_paths_vector_[index] = std::pair<int16_t,int16_t>((*itjoint)->GetFirstAttached()->GetIndex(),jointindex);
                vcosts[index] = 1;
                index = (*itjoint)->GetSecondAttached()->GetIndex()*links_vector_.size()+(*itjoint)->GetFirstAttached()->GetIndex();
                all_pairs_shortest_paths_vector_[index] = std::pair<int16_t,int16_t>((*itjoint)->GetSecondAttached()->GetIndex(),jointindex);
                vcosts[index] = 1;
            }
            ++jointindex;
        }
        for(size_t k = 0; k < links_vector_.size(); ++k) {
            for(size_t i = 0; i < links_vector_.size(); ++i) {
                if( i == k ) {
                    continue;
                }
                for(size_t j = 0; j < links_vector_.size(); ++j) {
                    if((j == i)||(j == k)) {
                        continue;
                    }
                    uint32_t kcost = vcosts[k*links_vector_.size()+i] + vcosts[j*links_vector_.size()+k];
                    if( vcosts[j*links_vector_.size()+i] > kcost ) {
                        vcosts[j*links_vector_.size()+i] = kcost;
                        all_pairs_shortest_paths_vector_[j*links_vector_.size()+i] = all_pairs_shortest_paths_vector_[k*links_vector_.size()+i];
                    }
                }
            }
        }
    }

    // Use the APAC algorithm to initialize the kinematics hierarchy: topologically_sorted_joints_vector_, joints_affecting_links_vector_, Link::parent_links_vector_.
    // SIMOES, Ricardo. APAC: An exact algorithm for retrieving cycles and paths in all kinds of graphs. Tékhne, Dec. 2009, no.12, p.39-55. ISSN 1654-9911.
    if((links_vector_.size() > 0)&&(joints_vector_.size() > 0)) {
        std::vector< std::vector<int> > vlinkadjacency(links_vector_.size());
        // joints with only one attachment are attached to a static link, which is attached to link 0
        FOREACHC(itjoint,joints_vector_) {
            vlinkadjacency.at((*itjoint)->GetFirstAttached()->GetIndex()).push_back((*itjoint)->GetSecondAttached()->GetIndex());
            vlinkadjacency.at((*itjoint)->GetSecondAttached()->GetIndex()).push_back((*itjoint)->GetFirstAttached()->GetIndex());
        }
        FOREACHC(itjoint,passive_joints_vector_) {
            vlinkadjacency.at((*itjoint)->GetFirstAttached()->GetIndex()).push_back((*itjoint)->GetSecondAttached()->GetIndex());
            vlinkadjacency.at((*itjoint)->GetSecondAttached()->GetIndex()).push_back((*itjoint)->GetFirstAttached()->GetIndex());
        }
        FOREACH(it,vlinkadjacency) {
            sort(it->begin(), it->end());
        }

        // all unique paths starting at the root link or static links
        std::vector< std::list< std::list<int> > > vuniquepaths(links_vector_.size());
        std::list< std::list<int> > closedloops;
        int s = 0;
        std::list< std::list<int> > S;
        FOREACH(itv,vlinkadjacency[s]) {
            std::list<int> P;
            P.push_back(s);
            P.push_back(*itv);
            S.push_back(P);
            vuniquepaths[*itv].push_back(P);
        }
        while(!S.empty()) {
            std::list<int>& P = S.front();
            int u = P.back();
            FOREACH(itv,vlinkadjacency[u]) {
                std::list<int>::iterator itfound = find(P.begin(),P.end(),*itv);
                if( itfound == P.end() ) {
                    S.push_back(P);
                    S.back().push_back(*itv);
                    vuniquepaths[*itv].push_back(S.back());
                }
                else {
                    // found a cycle
                    std::list<int> cycle;
                    while(itfound != P.end()) {
                        cycle.push_back(*itfound);
                        ++itfound;
                    }
                    if( cycle.size() > 2 ) {
                        // sort the cycle so that it starts with the lowest link index and the direction is the next lowest index
                        // this way the cycle becomes unique and can be compared for duplicates
                        itfound = cycle.begin();
                        std::list<int>::iterator itmin = itfound++;
                        while(itfound != cycle.end()) {
                            if( *itmin > *itfound ) {
                                itmin = itfound;
                            }
                            itfound++;
                        }
                        if( itmin != cycle.begin() ) {
                            cycle.splice(cycle.end(),cycle,cycle.begin(),itmin);
                        }
                        if( *++cycle.begin() > cycle.back() ) {
                            // reverse the cycle
                            cycle.reverse();
                            cycle.push_front(cycle.back());
                            cycle.pop_back();
                        }
                        if( find(closedloops.begin(),closedloops.end(),cycle) == closedloops.end() ) {
                            closedloops.push_back(cycle);
                        }
                    }
                }
            }
            S.pop_front();
        }
        // fill each link's parent links
        FOREACH(itlink,links_vector_) {
            if( (*itlink)->GetIndex() > 0 && vuniquepaths.at((*itlink)->GetIndex()).size() == 0 ) {
                RAVELOG_WARN(str(boost::format("_ComputeInternalInformation: %s has incomplete kinematics! link %s not connected to root %s")%GetName()%(*itlink)->GetName()%links_vector_.at(0)->GetName()));
            }
            FOREACH(itpath, vuniquepaths.at((*itlink)->GetIndex())) {
                OPENRAVE_ASSERT_OP(itpath->back(),==,(*itlink)->GetIndex());
                int parentindex = *---- itpath->end();
                if( find((*itlink)->parent_links_vector_.begin(),(*itlink)->parent_links_vector_.end(),parentindex) == (*itlink)->parent_links_vector_.end() ) {
                    (*itlink)->parent_links_vector_.push_back(parentindex);
                }
            }
        }
        // find the link depths (minimum path length to the root)
        vector<int> vlinkdepths(links_vector_.size(),-1);
        vlinkdepths.at(0) = 0;
        for(size_t i = 0; i < links_vector_.size(); ++i) {
            if( links_vector_[i]->IsStatic() ) {
                vlinkdepths[i] = 0;
            }
        }
        bool changed = true;
        while(changed) {
            changed = false;
            FOREACH(itlink,links_vector_) {
                if( vlinkdepths[(*itlink)->GetIndex()] == -1 ) {
                    int bestindex = -1;
                    FOREACH(itparent, (*itlink)->parent_links_vector_) {
                        if( vlinkdepths[*itparent] >= 0 ) {
                            if( bestindex == -1 || (bestindex >= 0 && vlinkdepths[*itparent] < bestindex) ) {
                                bestindex = vlinkdepths[*itparent]+1;
                            }
                        }
                    }
                    if( bestindex >= 0 ) {
                        vlinkdepths[(*itlink)->GetIndex()] = bestindex;
                        changed = true;
                    }
                }
            }
        }


        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            FOREACH(itlink, links_vector_) {
                std::stringstream ss; ss << GetName() << ":" << (*itlink)->GetName() << " depth=" << vlinkdepths.at((*itlink)->GetIndex()) << ", parents=[";
                FOREACHC(itparentlink, (*itlink)->parent_links_vector_) {
                    ss << links_vector_.at(*itparentlink)->GetName() << ", ";
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }
        }

        // build up a directed graph of joint dependencies
        int numjoints = (int)(joints_vector_.size()+passive_joints_vector_.size());
        // build the adjacency list
        vector<int> vjointadjacency(numjoints*numjoints,0);
        for(int ij0 = 0; ij0 < numjoints; ++ij0) {
            JointPtr j0 = ij0 < (int)joints_vector_.size() ? joints_vector_[ij0] : passive_joints_vector_[ij0-joints_vector_.size()];
            bool bj0hasstatic = (!j0->GetFirstAttached() || j0->GetFirstAttached()->IsStatic()) || (!j0->GetSecondAttached() || j0->GetSecondAttached()->IsStatic());
            // mimic joint sorting is the hardest limit
            if( j0->IsMimic() ) {
                for(int i = 0; i < j0->GetDOF(); ++i) {
                    if(j0->IsMimic(i)) {
                        FOREACH(itdofformat, j0->_vmimic[i]->_vdofformat) {
                            if( itdofformat->dofindex < 0 ) {
                                vjointadjacency[itdofformat->jointindex*numjoints+ij0] = 1;
                            }
                        }
                    }
                }
            }

            for(int ij1 = ij0+1; ij1 < numjoints; ++ij1) {
                JointPtr j1 = ij1 < (int)joints_vector_.size() ? joints_vector_[ij1] : passive_joints_vector_[ij1-joints_vector_.size()];
                bool bj1hasstatic = (!j1->GetFirstAttached() || j1->GetFirstAttached()->IsStatic()) || (!j1->GetSecondAttached() || j1->GetSecondAttached()->IsStatic());

                // test if connected to world, next in priority to mimic joints
                if( bj0hasstatic && bj1hasstatic ) {
                    continue;
                }
                if( vjointadjacency[ij1*numjoints+ij0] || vjointadjacency[ij0*numjoints+ij1] ) {
                    // already have an edge, so no reason to add any more
                    continue;
                }

                // sort by link depth
                int j0l0 = vlinkdepths[j0->GetFirstAttached()->GetIndex()];
                int j0l1 = vlinkdepths[j0->GetSecondAttached()->GetIndex()];
                int j1l0 = vlinkdepths[j1->GetFirstAttached()->GetIndex()];
                int j1l1 = vlinkdepths[j1->GetSecondAttached()->GetIndex()];
                int diff = min(j0l0,j0l1) - min(j1l0,j1l1);
                if( diff < 0 ) {
                    OPENRAVE_ASSERT_OP(min(j0l0,j0l1),<,100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-min(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    OPENRAVE_ASSERT_OP(min(j1l0,j1l1),<,100);
                    vjointadjacency[ij1*numjoints+ij0] = 100-min(j1l0,j1l1);
                    continue;
                }
                diff = max(j0l0,j0l1) - max(j1l0,j1l1);
                if( diff < 0 ) {
                    OPENRAVE_ASSERT_OP(max(j0l0,j0l1),<,100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-max(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    OPENRAVE_ASSERT_OP(max(j1l0,j1l1),<,100);
                    vjointadjacency[ij1*numjoints+ij0] = 100-max(j1l0,j1l1);
                    continue;
                }
            }
        }
        // topologically sort the joints
        topologically_sorted_joint_indices_all_vector_.resize(0); topologically_sorted_joint_indices_all_vector_.reserve(numjoints);
        std::list<int> noincomingedges;
        for(int i = 0; i < numjoints; ++i) {
            bool hasincoming = false;
            for(int j = 0; j < numjoints; ++j) {
                if( vjointadjacency[j*numjoints+i] ) {
                    hasincoming = true;
                    break;
                }
            }
            if( !hasincoming ) {
                noincomingedges.push_back(i);
            }
        }
        bool bcontinuesorting = true;
        while(bcontinuesorting) {
            bcontinuesorting = false;
            while(!noincomingedges.empty()) {
                int n = noincomingedges.front();
                noincomingedges.pop_front();
                topologically_sorted_joint_indices_all_vector_.push_back(n);
                for(int i = 0; i < numjoints; ++i) {
                    if( vjointadjacency[n*numjoints+i] ) {
                        vjointadjacency[n*numjoints+i] = 0;
                        bool hasincoming = false;
                        for(int j = 0; j < numjoints; ++j) {
                            if( vjointadjacency[j*numjoints+i] ) {
                                hasincoming = true;
                                break;
                            }
                        }
                        if( !hasincoming ) {
                            noincomingedges.push_back(i);
                        }
                    }
                }
            }

            // go backwards so we prioritize moving joints towards the end rather than the beginning (not a formal heurstic)
            int imaxadjind = vjointadjacency[numjoints*numjoints-1];
            for(int ijoint = numjoints*numjoints-1; ijoint >= 0; --ijoint) {
                if( vjointadjacency[ijoint] > vjointadjacency[imaxadjind] ) {
                    imaxadjind = ijoint;
                }
            }
            if( vjointadjacency[imaxadjind] != 0 ) {
                bcontinuesorting = true;
                int ifirst = imaxadjind/numjoints;
                int isecond = imaxadjind%numjoints;
                if( vjointadjacency[imaxadjind] <= 2 ) { // level 1 - static constraint violated, level 2 - mimic constraint
                    JointPtr pji = ifirst < (int)joints_vector_.size() ? joints_vector_[ifirst] : passive_joints_vector_.at(ifirst-joints_vector_.size());
                    JointPtr pjj = isecond < (int)joints_vector_.size() ? joints_vector_[isecond] : passive_joints_vector_.at(isecond-joints_vector_.size());
                    RAVELOG_WARN(str(boost::format("cannot sort joints topologically %d for robot %s joints %s:%s!! forward kinematics might be buggy\n")%vjointadjacency[imaxadjind]%GetName()%pji->GetName()%pjj->GetName()));
                }
                // remove this edge
                vjointadjacency[imaxadjind] = 0;
                bool hasincoming = false;
                for(int j = 0; j < numjoints; ++j) {
                    if( vjointadjacency[j*numjoints+isecond] ) {
                        hasincoming = true;
                        break;
                    }
                }
                if( !hasincoming ) {
                    noincomingedges.push_back(isecond);
                }
            }
        }
        OPENRAVE_ASSERT_OP((int)topologically_sorted_joint_indices_all_vector_.size(),==,numjoints);
        FOREACH(itindex,topologically_sorted_joint_indices_all_vector_) {
            JointPtr pj = *itindex < (int)joints_vector_.size() ? joints_vector_[*itindex] : passive_joints_vector_.at(*itindex-joints_vector_.size());
            if( *itindex < (int)joints_vector_.size() ) {
                topologically_sorted_joints_vector_.push_back(pj);
            }
            topologically_sorted_joints_all_vector_.push_back(pj);
            //RAVELOG_INFO(str(boost::format("top: %s")%pj->GetName()));
        }

        // based on this topological sorting, find the parent link for each joint
        FOREACH(itjoint,topologically_sorted_joints_all_vector_) {
            int parentlinkindex = -1;
            if( !(*itjoint)->GetFirstAttached() || (*itjoint)->GetFirstAttached()->IsStatic() ) {
                if( !!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic() ) {
                    parentlinkindex = (*itjoint)->GetSecondAttached()->GetIndex();
                }
            }
            else if( !(*itjoint)->GetSecondAttached() || (*itjoint)->GetSecondAttached()->IsStatic() ) {
                parentlinkindex = (*itjoint)->GetFirstAttached()->GetIndex();
            }
            else {
                // NOTE: possibly try to choose roots that do not involve mimic joints. ikfast might have problems
                // dealing with very complex formulas
                LinkPtr plink0 = (*itjoint)->GetFirstAttached(), plink1 = (*itjoint)->GetSecondAttached();
                if( vlinkdepths[plink0->GetIndex()] < vlinkdepths[plink1->GetIndex()] ) {
                    parentlinkindex = plink0->GetIndex();
                }
                else if( vlinkdepths[plink0->GetIndex()] > vlinkdepths[plink1->GetIndex()] ) {
                    parentlinkindex = plink1->GetIndex();
                }
                else {
                    // depths are the same, so check the adjacent joints of each link
                    size_t link0pos=topologically_sorted_joint_indices_all_vector_.size(), link1pos=topologically_sorted_joint_indices_all_vector_.size();
                    FOREACHC(itparentlink,plink0->parent_links_vector_) {
                        int jointindex = all_pairs_shortest_paths_vector_[plink0->GetIndex()*links_vector_.size()+*itparentlink].second;
                        size_t pos = find(topologically_sorted_joint_indices_all_vector_.begin(),topologically_sorted_joint_indices_all_vector_.end(),jointindex) - topologically_sorted_joint_indices_all_vector_.begin();
                        link0pos = min(link0pos,pos);
                    }
                    FOREACHC(itparentlink,plink1->parent_links_vector_) {
                        int jointindex = all_pairs_shortest_paths_vector_[plink1->GetIndex()*links_vector_.size()+*itparentlink].second;
                        size_t pos = find(topologically_sorted_joint_indices_all_vector_.begin(),topologically_sorted_joint_indices_all_vector_.end(),jointindex) - topologically_sorted_joint_indices_all_vector_.end();
                        link1pos = min(link1pos,pos);
                    }
                    if( link0pos < link1pos ) {
                        parentlinkindex = plink0->GetIndex();
                    }
                    else if( link0pos > link1pos ) {
                        parentlinkindex = plink1->GetIndex();
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("links %s and %s have joints on the same depth %d and %d?")%plink0->GetName()%plink1->GetName()%link0pos%link1pos));
                    }
                }
            }
            if( parentlinkindex == -1 ) {
                RAVELOG_WARN(str(boost::format("could not compute parent link for joint %s")%(*itjoint)->GetName()));
            }
            else if( parentlinkindex != (*itjoint)->GetFirstAttached()->GetIndex() ) {
                RAVELOG_VERBOSE(str(boost::format("swapping link order of joint %s(%d)")%(*itjoint)->GetName()%(*itjoint)->GetJointIndex()));
                // have to swap order
                Transform tswap = (*itjoint)->GetInternalHierarchyRightTransform().inverse();
                std::vector<Vector> vaxes((*itjoint)->GetDOF());
                for(size_t i = 0; i < vaxes.size(); ++i) {
                    vaxes[i] = -tswap.rotate((*itjoint)->GetInternalHierarchyAxis(i));
                }
                std::vector<dReal> vcurrentvalues;
                (*itjoint)->GetValues(vcurrentvalues);
                // have to reset the link transformations temporarily in order to avoid setting a joint offset
                TransformSaver<LinkPtr> linksaver0((*itjoint)->GetFirstAttached());
                TransformSaver<LinkPtr> linksaver1((*itjoint)->GetSecondAttached());
                // assume joint values are set to 0
                (*itjoint)->GetFirstAttached()->SetTransform(Transform());
                (*itjoint)->GetSecondAttached()->SetTransform((*itjoint)->GetInternalHierarchyLeftTransform()*(*itjoint)->GetInternalHierarchyRightTransform());
                // pass in empty joint values
                std::vector<dReal> vdummyzerovalues;
                (*itjoint)->_ComputeInternalInformation((*itjoint)->GetSecondAttached(),(*itjoint)->GetFirstAttached(),tswap.trans,vaxes,vdummyzerovalues);
                // initialize joint values to the correct value
                (*itjoint)->info_.current_values_vector_ = vcurrentvalues;
            }
        }
        // find out what links are affected by what joints.
        FOREACH(it,joints_affecting_links_vector_) {
            *it = 0;
        }
        vector<int8_t> vusedlinks;
        for(int i = 0; i < (int)links_vector_.size(); ++i) {
            vusedlinks.resize(0); vusedlinks.resize(links_vector_.size());
            FOREACH(itpath,vuniquepaths[i]) {
                FOREACH(itlink,*itpath) {
                    vusedlinks[*itlink] = 1;
                }
            }
            for(int j = 0; j < (int)links_vector_.size(); ++j) {
                if( vusedlinks[j] &&(i != j)) {
                    int jointindex = all_pairs_shortest_paths_vector_[i*links_vector_.size()+j].second;
                    OPENRAVE_ASSERT_OP( jointindex, >=, 0 );
                    JointPtr pjoint = jointindex < (int)joints_vector_.size() ? joints_vector_[jointindex] : passive_joints_vector_.at(jointindex-joints_vector_.size());
                    if( jointindex < (int)joints_vector_.size() ) {
                        joints_affecting_links_vector_[jointindex*links_vector_.size()+i] = pjoint->GetHierarchyParentLink()->GetIndex() == i ? -1 : 1;
                    }
                    if( pjoint->IsMimic() ) {
                        for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                            if( pjoint->IsMimic(idof) ) {
                                FOREACHC(itmimicdof,pjoint->_vmimic[idof]->_vmimicdofs) {
                                    JointPtr pjoint2 = GetJointFromDOFIndex(itmimicdof->dofindex);
                                    joints_affecting_links_vector_[pjoint2->GetJointIndex()*links_vector_.size()+i] = pjoint2->GetHierarchyParentLink()->GetIndex() == i ? -1 : 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        // process the closed loops, note that determining 'degrees of freedom' of the loop is very difficult and should be left to the 'fkfast' tool
        closed_loop_indices_vector_.resize(0); closed_loop_indices_vector_.reserve(closedloops.size());
        closed_loops_vector_.resize(0); closed_loops_vector_.reserve(closedloops.size());
        FOREACH(itclosedloop,closedloops) {
            closed_loop_indices_vector_.push_back(vector< std::pair<int16_t, int16_t> >());
            closed_loop_indices_vector_.back().reserve(itclosedloop->size());
            closed_loops_vector_.push_back(vector< std::pair<LinkPtr, JointPtr> >());
            closed_loops_vector_.back().reserve(itclosedloop->size());
            // fill the links
            FOREACH(itlinkindex,*itclosedloop) {
                closed_loop_indices_vector_.back().emplace_back(*itlinkindex, 0);
                closed_loops_vector_.back().emplace_back(links_vector_.at(*itlinkindex), JointPtr());
            }
            // fill the joints
            for(size_t i = 0; i < closed_loop_indices_vector_.back().size(); ++i) {
                int nextlink = i+1 < closed_loop_indices_vector_.back().size() ? closed_loop_indices_vector_.back()[i+1].first : closed_loop_indices_vector_.back()[0].first;
                int jointindex = all_pairs_shortest_paths_vector_[nextlink*links_vector_.size()+closed_loop_indices_vector_.back()[i].first].second;
                closed_loop_indices_vector_.back()[i].second = jointindex;
                if( jointindex < (int)joints_vector_.size() ) {
                    closed_loops_vector_.back().at(i).second = joints_vector_.at(jointindex);
                }
                else {
                    closed_loops_vector_.back().at(i).second = passive_joints_vector_.at(jointindex-joints_vector_.size());
                }
            }

            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                stringstream ss;
                ss << GetName() << " closedloop found: ";
                FOREACH(itlinkindex,*itclosedloop) {
                    LinkPtr plink = links_vector_.at(*itlinkindex);
                    ss << plink->GetName() << "(" << plink->GetIndex() << ") ";
                }
                RAVELOG_VERBOSE(ss.str());
            }
        }
    }

    // compute the rigidly attached links
    for(size_t ilink = 0; ilink < links_vector_.size(); ++ilink) {
        vector<int>& vattachedlinks = links_vector_[ilink]->_vRigidlyAttachedLinks;
        vattachedlinks.resize(0);
        vattachedlinks.push_back(ilink);
        if((ilink == 0)|| links_vector_[ilink]->IsStatic() ) {
            FOREACHC(itlink,links_vector_) {
                if( (*itlink)->IsStatic() ) {
                    if( (*itlink)->GetIndex() != (int)ilink ) {
                        vattachedlinks.push_back((*itlink)->GetIndex());
                    }
                }
            }
            FOREACHC(itjoint, GetJoints()) {
                if( (*itjoint)->IsStatic() ) {
                    if( !(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itjoint)->GetSecondAttached()->GetIndex());
                    }
                    if( !(*itjoint)->GetSecondAttached() && !!(*itjoint)->GetFirstAttached() && !(*itjoint)->GetFirstAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itjoint)->GetFirstAttached()->GetIndex());
                    }
                }
            }
            FOREACHC(itpassive, GetPassiveJoints()) {
                if( (*itpassive)->IsStatic() ) {
                    if( !(*itpassive)->GetFirstAttached() && !!(*itpassive)->GetSecondAttached() && !(*itpassive)->GetSecondAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itpassive)->GetSecondAttached()->GetIndex());
                    }
                    if( !(*itpassive)->GetSecondAttached() && !!(*itpassive)->GetFirstAttached() && !(*itpassive)->GetFirstAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itpassive)->GetFirstAttached()->GetIndex());
                    }
                }
            }
        }

        // breadth first search for rigid links
        for(size_t icurlink = 0; icurlink<vattachedlinks.size(); ++icurlink) {
            LinkPtr plink=links_vector_.at(vattachedlinks[icurlink]);
            FOREACHC(itjoint, joints_vector_) {
                if( (*itjoint)->IsStatic() ) {
                    if(((*itjoint)->GetFirstAttached() == plink)&& !!(*itjoint)->GetSecondAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetSecondAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itjoint)->GetSecondAttached()->GetIndex());
                    }
                    if(((*itjoint)->GetSecondAttached() == plink)&& !!(*itjoint)->GetFirstAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetFirstAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itjoint)->GetFirstAttached()->GetIndex());
                    }
                }
            }
            FOREACHC(itpassive, passive_joints_vector_) {
                if( (*itpassive)->IsStatic() ) {
                    if(((*itpassive)->GetFirstAttached() == plink)&& !!(*itpassive)->GetSecondAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetSecondAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itpassive)->GetSecondAttached()->GetIndex());
                    }
                    if(((*itpassive)->GetSecondAttached() == plink)&& !!(*itpassive)->GetFirstAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetFirstAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itpassive)->GetFirstAttached()->GetIndex());
                    }
                }
            }
        }
    }

    for(size_t ijoint = 0; ijoint < joints_vector_.size(); ++ijoint ) {
        if( joints_vector_[ijoint]->GetName().size() == 0 ) {
            RAVELOG_WARN(str(boost::format("%s joint index %d has no name")%GetName()%ijoint));
        }
    }
    for(size_t ijoint = 0; ijoint < passive_joints_vector_.size(); ++ijoint ) {
        if( passive_joints_vector_[ijoint]->GetName().size() == 0 ) {
            RAVELOG_WARN(str(boost::format("%s passive joint index %d has no name")%GetName()%ijoint));
        }
    }
    for(size_t ijoint0 = 0; ijoint0 < topologically_sorted_joints_all_vector_.size(); ++ijoint0 ) {
        JointPtr pjoint0 = topologically_sorted_joints_all_vector_[ijoint0];
        for(size_t ijoint1 = ijoint0+1; ijoint1 < topologically_sorted_joints_all_vector_.size(); ++ijoint1 ) {
            JointPtr pjoint1 = topologically_sorted_joints_all_vector_[ijoint1];
            if( pjoint0->GetName() == pjoint1->GetName() && (pjoint0->GetJointIndex() >= 0 || pjoint1->GetJointIndex() >= 0) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint indices %d and %d share the same name '%s'"), pjoint0->GetJointIndex()%pjoint1->GetJointIndex()%pjoint0->GetName(), ORE_InvalidState);
            }
        }
    }

    __hashkinematics.resize(0);

    // create the adjacency list
    {
        _setAdjacentLinks.clear();
        FOREACH(itadj, _vForcedAdjacentLinks) {
            LinkPtr pl0 = GetLink(itadj->first);
            LinkPtr pl1 = GetLink(itadj->second);
            if( !!pl0 && !!pl1 ) {
                int ind0 = pl0->GetIndex();
                int ind1 = pl1->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }
        }

        // make no-geometry links adjacent to all other links
        FOREACH(itlink0, links_vector_) {
            if( (*itlink0)->GetGeometries().size() == 0 ) {
                int ind0 = (*itlink0)->GetIndex();
                FOREACH(itlink1,links_vector_) {
                    if( *itlink0 != *itlink1 ) {
                        int ind1 = (*itlink1)->GetIndex();
                        if( ind1 < ind0 ) {
                            _setAdjacentLinks.insert(ind1|(ind0<<16));
                        }
                        else {
                            _setAdjacentLinks.insert(ind0|(ind1<<16));
                        }
                    }
                }
            }
        }

        if( _bMakeJoinedLinksAdjacent ) {
            FOREACH(itj, joints_vector_) {
                int ind0 = (*itj)->attached_bodies_array_[0]->GetIndex();
                int ind1 = (*itj)->attached_bodies_array_[1]->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }

            FOREACH(itj, passive_joints_vector_) {
                int ind0 = (*itj)->attached_bodies_array_[0]->GetIndex();
                int ind1 = (*itj)->attached_bodies_array_[1]->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }

            // if a pair links has exactly one non-static joint in the middle, then make the pair adjacent
            vector<JointPtr> vjoints;
            for(int i = 0; i < (int)links_vector_.size()-1; ++i) {
                for(int j = i+1; j < (int)links_vector_.size(); ++j) {
                    GetChain(i,j,vjoints);
                    size_t numstatic = 0;
                    FOREACH(it,vjoints) {
                        numstatic += (*it)->IsStatic();
                    }
                    if( numstatic+1 >= vjoints.size() ) {
                        if( i < j ) {
                            _setAdjacentLinks.insert(i|(j<<16));
                        }
                        else {
                            _setAdjacentLinks.insert(j|(i<<16));
                        }
                    }
                }
            }
        }
        _ResetInternalCollisionCache();
    }
    hierarchy_computed_ = 2;
    // because of mimic joints, need to call SetDOFValues at least once, also use this to check for links that are off
    {
        vector<Transform> vprevtrans, vnewtrans;
        vector<dReal> vprevdoflastsetvalues, vnewdoflastsetvalues;
        GetLinkTransformations(vprevtrans, vprevdoflastsetvalues);
        vector<dReal> vcurrentvalues;
        // unfortunately if SetDOFValues is overloaded by the robot, it could call the robot's _UpdateGrabbedBodies, which is a problem during environment cloning since the grabbed bodies might not be initialized. Therefore, call KinBody::SetDOFValues
        GetDOFValues(vcurrentvalues);
        std::vector<UserDataPtr> vGrabbedBodies; vGrabbedBodies.swap(_vGrabbedBodies); // swap to get rid of _vGrabbedBodies
        KinBody::SetDOFValues(vcurrentvalues,CLA_CheckLimits, std::vector<int>());
        vGrabbedBodies.swap(_vGrabbedBodies); // swap back
        GetLinkTransformations(vnewtrans, vnewdoflastsetvalues);
        for(size_t i = 0; i < vprevtrans.size(); ++i) {
            if( TransformDistanceFast(vprevtrans[i],vnewtrans[i]) > 1e-5 ) {
                RAVELOG_VERBOSE(str(boost::format("link %d has different transformation after SetDOFValues (error=%f), this could be due to mimic joint equations kicking into effect.")%links_vector_.at(i)->GetName()%TransformDistanceFast(vprevtrans[i],vnewtrans[i])));
            }
        }
        for(int i = 0; i < GetDOF(); ++i) {
            if( vprevdoflastsetvalues.at(i) != vnewdoflastsetvalues.at(i) ) {
                RAVELOG_VERBOSE(str(boost::format("dof %d has different values after SetDOFValues %d!=%d, this could be due to mimic joint equations kicking into effect.")%i%vprevdoflastsetvalues.at(i)%vnewdoflastsetvalues.at(i)));
            }
        }
        _vInitialLinkTransformations = vnewtrans;
    }

    {
        // do not initialize interpolation, since it implies a motion sampling strategy
        int offset = 0;
        _spec.groups_vector_.resize(0);
        if( GetDOF() > 0 ) {
            ConfigurationSpecification::Group group;
            stringstream ss;
            ss << "joint_values " << GetName();
            for(int i = 0; i < GetDOF(); ++i) {
                ss << " " << i;
            }
            group.name = ss.str();
            group.dof = GetDOF();
            group.offset = offset;
            offset += group.dof;
            _spec.groups_vector_.push_back(group);
        }

        ConfigurationSpecification::Group group;
        group.name = str(boost::format("affine_transform %s %d")%GetName()%DOF_Transform);
        group.offset = offset;
        group.dof = RaveGetAffineDOF(DOF_Transform);
        _spec.groups_vector_.push_back(group);
    }

    // set the "self" extra geometry group
    std::string selfgroup("self");
    FOREACH(itlink, links_vector_) {
        if( (*itlink)->info_.extra_geometries_map_.find(selfgroup) == (*itlink)->info_.extra_geometries_map_.end() ) {
            std::vector<GeometryInfoPtr> vgeoms;
            FOREACH(itgeom, (*itlink)->geometries_vector_) {
                vgeoms.push_back(GeometryInfoPtr(new GeometryInfo((*itgeom)->GetInfo())));
            }
            (*itlink)->info_.extra_geometries_map_.insert(make_pair(selfgroup, vgeoms));
        }
    }

    is_all_joints_1dof_and_no_circular_ = true;
    for (size_t ijoint = 0; ijoint < joints_vector_.size(); ++ijoint) {
        if (joints_vector_[ijoint]->GetDOF() != 1 || joints_vector_[ijoint]->IsCircular()) {
            is_all_joints_1dof_and_no_circular_ = false;
            break;
        }
    }

    // notify any callbacks of the changes
    std::list<UserDataWeakPtr> listRegisteredCallbacks;
    uint32_t index = 0;
    uint32_t parameters = parameters_changed_;
    while(parameters && index < _vlistRegisteredCallbacks.size()) {
        if( (parameters & 1) &&  _vlistRegisteredCallbacks.at(index).size() > 0 ) {
            {
                boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
                listRegisteredCallbacks = _vlistRegisteredCallbacks.at(index); // copy since it can be changed
            }
            FOREACH(it,listRegisteredCallbacks) {
                ChangeCallbackDataPtr pdata = std::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
                if( !!pdata ) {
                    pdata->_callback();
                }
            }
        }
        parameters >>= 1;
        index += 1;
    }
    parameters_changed_ = 0;
    RAVELOG_VERBOSE_FORMAT("initialized %s in %fs", GetName()%(1e-6*(utils::GetMicroTime()-starttime)));
}

void KinBody::_DeinitializeInternalInformation()
{
    hierarchy_computed_ = 0; // should reset to inform other elements that kinematics information might not be accurate
}

bool KinBody::IsAttached(const KinBody &body) const
{
    if(this == &body ) {
        return true;
    }
    std::set<KinBodyConstPtr> dummy;
    return _IsAttached(body, dummy);
}

void KinBody::GetAttached(std::set<KinBodyPtr>&setAttached) const
{
    setAttached.insert(std::const_pointer_cast<KinBody>(shared_kinbody_const()));
    FOREACHC(itbody,attached_bodies_list_) {
        KinBodyPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second ) {
            pattached->GetAttached(setAttached);
        }
    }
}

void KinBody::GetAttached(std::set<KinBodyConstPtr>&setAttached) const
{
    setAttached.insert(shared_kinbody_const());
    FOREACHC(itbody,attached_bodies_list_) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second ) {
            pattached->GetAttached(setAttached);
        }
    }
}

bool KinBody::HasAttached() const
{
    return attached_bodies_list_.size() > 0;
}

bool KinBody::_IsAttached(const KinBody &body, std::set<KinBodyConstPtr>&setChecked) const
{
    if( !setChecked.insert(shared_kinbody_const()).second ) {
        return false;
    }
    FOREACHC(itbody,attached_bodies_list_) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && ((pattached.get() == &body)|| pattached->_IsAttached(body,setChecked)) ) {
            return true;
        }
    }
    return false;
}

void KinBody::_AttachBody(KinBodyPtr pbody)
{
    attached_bodies_list_.push_back(pbody);
    pbody->attached_bodies_list_.push_back(shared_kinbody());
    _PostprocessChangedParameters(Prop_BodyAttached);
}

bool KinBody::_RemoveAttachedBody(KinBody &body)
{
    int numremoved = 0;
    FOREACH(it,attached_bodies_list_) {
        if( it->lock().get() == &body ) {
            attached_bodies_list_.erase(it);
            numremoved++;
            break;
        }
    }

    FOREACH(it, body.attached_bodies_list_) {
        if( it->lock().get() == this ) { // need to compare lock pointer since cannot rely on shared_kinbody() since in a destructor this will crash
            body.attached_bodies_list_.erase(it);
            numremoved++;
            break;
        }
    }

    if( numremoved > 0 ) {
        _PostprocessChangedParameters(Prop_BodyAttached);
    }

    return numremoved == 2;
}

void KinBody::Enable(bool bEnable)
{
    bool bchanged = false;
    FOREACH(it, links_vector_) {
        if( (*it)->info_.is_enabled_ != bEnable ) {
            (*it)->info_.is_enabled_ = bEnable;
            non_adjacent_link_cache_ &= ~AO_Enabled;
            bchanged = true;
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

bool KinBody::IsEnabled() const
{
    FOREACHC(it, links_vector_) {
        if((*it)->IsEnabled()) {
            return true;
        }
    }
    return false;
}

bool KinBody::SetVisible(bool visible)
{
    bool is_changed = false;
    for(auto& it: links_vector_)
	{
        for(auto& itgeom:it->geometries_vector_)
		{
            if( itgeom->IsVisible() != visible )
			{
                itgeom->info_.is_visible_ = visible;
                is_changed = true;
            }
        }
    }
    if( is_changed )
	{
        _PostprocessChangedParameters(Prop_LinkDraw);
        return true;
    }
    return false;
}

bool KinBody::IsVisible() const
{
    for(auto& it: links_vector_) 
	{
        if(it->IsVisible()) 
		{
            return true;
        }
    }
    return false;
}

int KinBody::GetEnvironmentId() const
{
    return environment_id_;
}

int8_t KinBody::DoesAffect(int jointindex, int linkindex ) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(jointindex >= 0 && jointindex < (int)joints_vector_.size(), 
		"body %s jointindex %d invalid (num joints %d)", GetName()%jointindex%joints_vector_.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(),
		"body %s linkindex %d invalid (num links %d)", GetName()%linkindex%links_vector_.size(), ORE_InvalidArguments);
    return joints_affecting_links_vector_.at(jointindex*links_vector_.size()+linkindex);
}

int8_t KinBody::DoesDOFAffectLink(int dofindex, int linkindex ) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(dofindex >= 0 && dofindex < GetDOF(), 
		"body %s dofindex %d invalid (num dofs %d)", GetName()%GetDOF(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)links_vector_.size(),
		"body %s linkindex %d invalid (num links %d)", GetName()%linkindex%links_vector_.size(), ORE_InvalidArguments);
    int jointindex = dof_indices_vector_.at(dofindex);
    return joints_affecting_links_vector_.at(jointindex*links_vector_.size()+linkindex);
}

void KinBody::SetNonCollidingConfiguration()
{
    _ResetInternalCollisionCache();
    std::vector<dReal> vdoflastsetvalues;
    GetLinkTransformations(_vInitialLinkTransformations, vdoflastsetvalues);
}

void KinBody::_ResetInternalCollisionCache()
{
    non_adjacent_link_cache_ = 0x80000000;
    for(auto& it:non_adjacent_links_vector_) 
	{
        it.resize(0);
    }
}

bool CompareNonAdjacentFarthest(int pair0, int pair1)
{
    // order so that farthest links are first. if equal, then prioritize links that are furthest down the chain.
    int pair0link0 = (pair0&0xffff);
    int pair0link1 = ((pair0>>16)&0xffff);
    int dist0 = pair0link1 - pair0link0; // link1 > link0
    int pair1link0 = (pair1&0xffff);
    int pair1link1 = ((pair1>>16)&0xffff);
    int dist1 = pair1link1 - pair1link0; // link1 > link0
    if( dist0 == dist1 ) {
        if( pair0link1 == pair1link1 ) {
            return pair0link0 > pair1link0;
        }
        else {
            return pair0link1 > pair1link1;
        }
    }
    return dist0 > dist1;
}

const std::vector<int>& KinBody::GetNonAdjacentLinks(int adjacentoptions) const
{
    class TransformsSaver
    {
public:
        TransformsSaver(KinBodyConstPtr pbody) : _pbody(pbody) {
            _pbody->GetLinkTransformations(vcurtrans, _vdoflastsetvalues);
        }
        ~TransformsSaver() {
            for(size_t i = 0; i < _pbody->links_vector_.size(); ++i) {
                std::static_pointer_cast<Link>(_pbody->links_vector_[i])->info_.transform_ = vcurtrans.at(i);
            }
            for(size_t i = 0; i < _pbody->joints_vector_.size(); ++i) {
                for(int j = 0; j < _pbody->joints_vector_[i]->GetDOF(); ++j) {
                    _pbody->joints_vector_[i]->_doflastsetvalues[j] = _vdoflastsetvalues.at(_pbody->joints_vector_[i]->GetDOFIndex()+j);
                }
            }
        }
private:
        KinBodyConstPtr _pbody;
        std::vector<Transform> vcurtrans;
        std::vector<dReal> _vdoflastsetvalues;
    };

    CHECK_INTERNAL_COMPUTATION;
    if( non_adjacent_link_cache_ & 0x80000000 ) {
        // Check for colliding link pairs given the initial pose _vInitialLinkTransformations
        // this is actually weird, we need to call the individual link collisions on a const body. in order to pull this off, we need to be very careful with the body state.
        TransformsSaver saver(shared_kinbody_const());
        CollisionCheckerBasePtr collisionchecker = !!self_collision_checker_ ? self_collision_checker_ : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        for(size_t i = 0; i < links_vector_.size(); ++i) {
            std::static_pointer_cast<Link>(links_vector_[i])->info_.transform_ = _vInitialLinkTransformations.at(i);
        }
        update_stamp_id_++; // because transforms were modified
        non_adjacent_links_vector_[0].resize(0);
        for(size_t i = 0; i < links_vector_.size(); ++i) {
            for(size_t j = i+1; j < links_vector_.size(); ++j) {
                if((_setAdjacentLinks.find(i|(j<<16)) == _setAdjacentLinks.end())&& !collisionchecker->CheckCollision(LinkConstPtr(links_vector_[i]), LinkConstPtr(links_vector_[j])) ) {
                    non_adjacent_links_vector_[0].push_back(i|(j<<16));
                }
            }
        }
        std::sort(non_adjacent_links_vector_[0].begin(), non_adjacent_links_vector_[0].end(), CompareNonAdjacentFarthest);
        update_stamp_id_++; // because transforms were modified
        non_adjacent_link_cache_ = 0;
    }
    if( (non_adjacent_link_cache_&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~non_adjacent_link_cache_)&adjacentoptions;
        // find out what needs to computed
        if( requestedoptions & AO_Enabled ) {
            non_adjacent_links_vector_.at(AO_Enabled).resize(0);
            FOREACHC(itset, non_adjacent_links_vector_[0]) {
                KinBody::LinkConstPtr plink1(links_vector_.at(*itset&0xffff)), plink2(links_vector_.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    non_adjacent_links_vector_[AO_Enabled].push_back(*itset);
                }
            }
            non_adjacent_link_cache_ |= AO_Enabled;
            std::sort(non_adjacent_links_vector_[AO_Enabled].begin(), non_adjacent_links_vector_[AO_Enabled].end(), CompareNonAdjacentFarthest);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("no support for adjacentoptions %d"), adjacentoptions,ORE_InvalidArguments);
        }
    }
    return non_adjacent_links_vector_.at(adjacentoptions);
}

const std::set<int>& KinBody::GetAdjacentLinks() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _setAdjacentLinks;
}

void KinBody::SetAdjacentLinks(int linkindex0, int linkindex1)
{
    OPENRAVE_ASSERT_OP(linkindex0,!=,linkindex1);
    if( linkindex0 > linkindex1 ) {
        std::swap(linkindex0, linkindex1);
    }

    _setAdjacentLinks.insert(linkindex0|(linkindex1<<16));
    std::string linkname0 = links_vector_.at(linkindex0)->GetName();
    std::string linkname1 = links_vector_.at(linkindex1)->GetName();
    std::pair<std::string, std::string> adjpair = std::make_pair(linkname0, linkname1);
    if( find(_vForcedAdjacentLinks.begin(), _vForcedAdjacentLinks.end(), adjpair) == _vForcedAdjacentLinks.end() ) {
        _vForcedAdjacentLinks.push_back(adjpair);
    }
    _ResetInternalCollisionCache();
}

void KinBody::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    InterfaceBase::Clone(preference,cloningoptions);
    KinBodyConstPtr r = RaveInterfaceConstCast<KinBody>(preference);

    name_ = r->name_;
    hierarchy_computed_ = r->hierarchy_computed_;
    _bMakeJoinedLinksAdjacent = r->_bMakeJoinedLinksAdjacent;
    __hashkinematics = r->__hashkinematics;
    _vTempJoints = r->_vTempJoints;

    links_vector_.resize(0); links_vector_.reserve(r->links_vector_.size());
    FOREACHC(itlink, r->links_vector_) {
        LinkPtr pnewlink(new Link(shared_kinbody()));
        // TODO should create a Link::Clone method
        *pnewlink = **itlink; // be careful of copying pointers
        pnewlink->parent_ = shared_kinbody();
        // have to copy all the geometries too!
        std::vector<Link::GeometryPtr> vnewgeometries(pnewlink->geometries_vector_.size());
        for(size_t igeom = 0; igeom < vnewgeometries.size(); ++igeom) {
            vnewgeometries[igeom].reset(new Link::Geometry(pnewlink, pnewlink->geometries_vector_[igeom]->info_));
        }
        pnewlink->geometries_vector_ = vnewgeometries;
        links_vector_.push_back(pnewlink);
    }

    joints_vector_.resize(0); joints_vector_.reserve(r->joints_vector_.size());
    FOREACHC(itjoint, r->joints_vector_) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        pnewjoint->attached_bodies_array_[0] = links_vector_.at((*itjoint)->attached_bodies_array_[0]->GetIndex());
        pnewjoint->attached_bodies_array_[1] = links_vector_.at((*itjoint)->attached_bodies_array_[1]->GetIndex());
        joints_vector_.push_back(pnewjoint);
    }

    passive_joints_vector_.resize(0); passive_joints_vector_.reserve(r->passive_joints_vector_.size());
    FOREACHC(itjoint, r->passive_joints_vector_) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        pnewjoint->attached_bodies_array_[0] = links_vector_.at((*itjoint)->attached_bodies_array_[0]->GetIndex());
        pnewjoint->attached_bodies_array_[1] = links_vector_.at((*itjoint)->attached_bodies_array_[1]->GetIndex());
        passive_joints_vector_.push_back(pnewjoint);
    }

    topologically_sorted_joints_vector_.resize(0); topologically_sorted_joints_vector_.resize(r->topologically_sorted_joints_vector_.size());
    FOREACHC(itjoint, r->topologically_sorted_joints_vector_) {
        topologically_sorted_joints_vector_.push_back(joints_vector_.at((*itjoint)->GetJointIndex()));
    }
    topologically_sorted_joints_all_vector_.resize(0); topologically_sorted_joints_all_vector_.resize(r->topologically_sorted_joints_all_vector_.size());
    FOREACHC(itjoint, r->topologically_sorted_joints_all_vector_) {
        std::vector<JointPtr>::const_iterator it = find(r->joints_vector_.begin(),r->joints_vector_.end(),*itjoint);
        if( it != r->joints_vector_.end() ) {
            topologically_sorted_joints_all_vector_.push_back(joints_vector_.at(it-r->joints_vector_.begin()));
        }
        else {
            it = find(r->passive_joints_vector_.begin(), r->passive_joints_vector_.end(),*itjoint);
            if( it != r->passive_joints_vector_.end() ) {
                topologically_sorted_joints_all_vector_.push_back(passive_joints_vector_.at(it-r->passive_joints_vector_.begin()));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint %s doesn't belong to anythong?"),(*itjoint)->GetName(), ORE_Assert);
            }
        }
    }
    dof_ordered_joints_vector_ = r->dof_ordered_joints_vector_;
    joints_affecting_links_vector_ = r->joints_affecting_links_vector_;
    dof_indices_vector_ = r->dof_indices_vector_;

    _setAdjacentLinks = r->_setAdjacentLinks;
    _vInitialLinkTransformations = r->_vInitialLinkTransformations;
    _vForcedAdjacentLinks = r->_vForcedAdjacentLinks;
    all_pairs_shortest_paths_vector_ = r->all_pairs_shortest_paths_vector_;
    closed_loop_indices_vector_ = r->closed_loop_indices_vector_;
    closed_loops_vector_.resize(0); closed_loops_vector_.reserve(r->closed_loops_vector_.size());
    FOREACHC(itloop,closed_loops_vector_) {
        closed_loops_vector_.push_back(std::vector< std::pair<LinkPtr,JointPtr> >());
        FOREACHC(it,*itloop) {
            closed_loops_vector_.back().emplace_back(links_vector_.at(it->first->GetIndex()), JointPtr());
            // the joint might be in passive_joints_vector_
            std::vector<JointPtr>::const_iterator itjoint = find(r->joints_vector_.begin(),r->joints_vector_.end(),it->second);
            if( itjoint != r->joints_vector_.end() ) {
                closed_loops_vector_.back().back().second = joints_vector_.at(itjoint-r->joints_vector_.begin());
            }
            else {
                itjoint = find(r->passive_joints_vector_.begin(), r->passive_joints_vector_.end(),it->second);
                if( itjoint != r->passive_joints_vector_.end() ) {
                    closed_loops_vector_.back().back().second = passive_joints_vector_.at(itjoint-r->passive_joints_vector_.begin());
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint %s in closed loop doesn't belong to anythong?"),(*itjoint)->GetName(), ORE_Assert);
                }
            }
        }
    }

    attached_bodies_list_.clear(); // will be set in the environment
    if( !(cloningoptions & Clone_IgnoreAttachedBodies) ) {
        FOREACHC(itatt, r->attached_bodies_list_) {
            KinBodyConstPtr pattref = itatt->lock();
            if( !!pattref ) {
                attached_bodies_list_.push_back(GetEnv()->GetBodyFromEnvironmentId(pattref->GetEnvironmentId()));
            }
        }
    }

    // cannot copy the velocities since it requires the physics engine to be initialized with this kinbody, which might not happen before the clone..?
//    std::vector<std::pair<Vector,Vector> > velocities;
//    r->GetLinkVelocities(velocities);
//    SetLinkVelocities(velocities);

    // do not force-reset the callbacks!! since the ChangeCallbackData destructors will crash
    //_listRegisteredCallbacks.clear();

    // cache
    _ResetInternalCollisionCache();

    // clone the grabbed bodies, note that this can fail if the new cloned environment hasn't added the bodies yet (check out Environment::Clone)
    _vGrabbedBodies.resize(0);
    FOREACHC(itgrabbedref, r->_vGrabbedBodies) {
        GrabbedConstPtr pgrabbedref = std::dynamic_pointer_cast<Grabbed const>(*itgrabbedref);

        KinBodyPtr pbodyref = pgrabbedref->_pgrabbedbody.lock();
        KinBodyPtr pgrabbedbody;
        if( !!pbodyref ) {
            pgrabbedbody = GetEnv()->GetBodyFromEnvironmentId(pbodyref->GetEnvironmentId());
            BOOST_ASSERT(pgrabbedbody->GetName() == pbodyref->GetName());

            GrabbedPtr pgrabbed(new Grabbed(pgrabbedbody,links_vector_.at(KinBody::LinkPtr(pgrabbedref->_plinkrobot)->GetIndex())));
            pgrabbed->_troot = pgrabbedref->_troot;
            pgrabbed->_listNonCollidingLinks.clear();
            FOREACHC(itlinkref, pgrabbedref->_listNonCollidingLinks) {
                pgrabbed->_listNonCollidingLinks.push_back(links_vector_.at((*itlinkref)->GetIndex()));
            }
            _vGrabbedBodies.push_back(pgrabbed);
        }
    }

    update_stamp_id_++; // update the stamp instead of copying
}

void KinBody::_PostprocessChangedParameters(uint32_t parameters)
{
    update_stamp_id_++;
    if( hierarchy_computed_ == 1 ) {
        parameters_changed_ |= parameters;
        return;
    }

    if( (parameters & Prop_JointMimic) == Prop_JointMimic || (parameters & Prop_LinkStatic) == Prop_LinkStatic) {
        KinBodyStateSaver saver(shared_kinbody(),Save_LinkTransformation);
        vector<dReal> vzeros(GetDOF(),0);
        SetDOFValues(vzeros,Transform(),true);
        _ComputeInternalInformation();
    }
    // do not change hash if geometry changed!
    if( !!(parameters & (Prop_LinkDynamics|Prop_LinkGeometry|Prop_JointMimic)) ) {
        __hashkinematics.resize(0);
    }

    if( (parameters&Prop_LinkEnable) == Prop_LinkEnable ) {
        // check if any regrabbed bodies have the link in _listNonCollidingLinks and the link is enabled, or are missing the link in _listNonCollidingLinks and the link is disabled
        std::map<GrabbedPtr, list<KinBody::LinkConstPtr> > mapcheckcollisions;
        FOREACH(itlink,links_vector_) {
            if( (*itlink)->IsEnabled() ) {
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        std::list<KinBody::LinkConstPtr>::iterator itnoncolliding = find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink);
                        if( itnoncolliding != pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 0 ) {
                                pgrabbed->_listNonCollidingLinks.erase(itnoncolliding);
                            }
                            mapcheckcollisions[pgrabbed].push_back(*itlink);
                        }
                        else {
                            // try to restore
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 1 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
            else {
                // add since it is disabled?
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = std::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        if( find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink) == pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) != 0 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
        }

//        if( mapcheckcollisions.size() > 0 ) {
//            CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
//            FOREACH(itgrabbed, mapcheckcollisions) {
//                KinBodyPtr pgrabbedbody(itgrabbed->first->_pgrabbedbody);
//                _RemoveAttachedBody(pgrabbedbody);
//                CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
//                FOREACH(itlink, itgrabbed->second) {
//                    if( pchecker->CheckCollision(*itlink, KinBodyConstPtr(pgrabbedbody)) ) {
//                        itgrabbed->first->_listNonCollidingLinks.remove(*itlink);
//                    }
//                }
//            }
//        }
    }

    std::list<UserDataWeakPtr> listRegisteredCallbacks;
    uint32_t index = 0;
    while(parameters && index < _vlistRegisteredCallbacks.size()) {
        if( (parameters & 1) &&  _vlistRegisteredCallbacks.at(index).size() > 0 ) {
            {
                boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
                listRegisteredCallbacks = _vlistRegisteredCallbacks.at(index); // copy since it can be changed
            }
            FOREACH(it,listRegisteredCallbacks) {
                ChangeCallbackDataPtr pdata = std::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
                if( !!pdata ) {
                    pdata->_callback();
                }
            }
        }
        parameters >>= 1;
        index += 1;
    }
}

void KinBody::Serialize(BaseXMLWriterPtr writer, int options) const
{
    InterfaceBase::Serialize(writer,options);
}

void KinBody::serialize(std::ostream& o, int options) const
{
    o << links_vector_.size() << " ";
    FOREACHC(it,links_vector_) {
        (*it)->serialize(o,options);
    }
    o << joints_vector_.size() << " ";
    FOREACHC(it,joints_vector_) {
        (*it)->serialize(o,options);
    }
    o << passive_joints_vector_.size() << " ";
    FOREACHC(it,passive_joints_vector_) {
        (*it)->serialize(o,options);
    }
}

void KinBody::SetZeroConfiguration()
{
    std::vector<Vector> vaxes;
    FOREACH(itjoint,joints_vector_) {
        vaxes.resize((*itjoint)->GetDOF());
        for(size_t i = 0; i < vaxes.size(); ++i) {
            vaxes[i] = (*itjoint)->GetInternalHierarchyLeftTransform().rotate((*itjoint)->GetInternalHierarchyAxis(i));
        }
        (*itjoint)->_ComputeInternalInformation((*itjoint)->GetFirstAttached(), (*itjoint)->GetSecondAttached(),(*itjoint)->GetInternalHierarchyLeftTransform().trans,vaxes,std::vector<dReal>());
    }
}

const std::string& KinBody::GetKinematicsGeometryHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( __hashkinematics.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        // should add dynamics since that affects a lot how part is treated.
        serialize(ss,SO_Kinematics|SO_Geometry|SO_Dynamics);
        __hashkinematics = utils::GetMD5HashString(ss.str());
    }
    return __hashkinematics;
}

void KinBody::SetConfigurationValues(std::vector<dReal>::const_iterator itvalues, uint32_t checklimits)
{
    vector<dReal> vdofvalues(GetDOF());
    if( GetDOF() > 0 ) {
        std::copy(itvalues,itvalues+GetDOF(),vdofvalues.begin());
    }
    Transform t;
    RaveGetTransformFromAffineDOFValues(t,itvalues+GetDOF(),DOF_Transform);
    SetDOFValues(vdofvalues,t,checklimits);
}

void KinBody::GetConfigurationValues(std::vector<dReal>&v) const
{
    GetDOFValues(v);
    v.resize(GetDOF()+RaveGetAffineDOF(DOF_Transform));
    RaveGetAffineDOFValuesFromTransform(v.begin()+GetDOF(),GetTransform(),DOF_Transform);
}

ConfigurationSpecification KinBody::GetConfigurationSpecification(const std::string& interpolation) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( interpolation.size() == 0 ) {
        return _spec;
    }
    ConfigurationSpecification spec=_spec;
    FOREACH(itgroup,spec.groups_vector_) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

ConfigurationSpecification KinBody::GetConfigurationSpecificationIndices(const std::vector<int>&indices, const std::string& interpolation) const
{
    CHECK_INTERNAL_COMPUTATION;
    ConfigurationSpecification spec;
    if( indices.size() > 0 ) {
        spec.groups_vector_.resize(1);
        stringstream ss;
        ss << "joint_values " << GetName();
        FOREACHC(it,indices) {
            ss << " " << *it;
        }
        spec.groups_vector_[0].name = ss.str();
        spec.groups_vector_[0].dof = indices.size();
        spec.groups_vector_[0].offset = 0;
        spec.groups_vector_[0].interpolation=interpolation;
    }
    return spec;
}

UserDataPtr KinBody::RegisterChangeCallback(uint32_t properties, const boost::function<void()>&callback) const
{
    ChangeCallbackDataPtr pdata(new ChangeCallbackData(properties,callback,shared_kinbody_const()));
    boost::unique_lock< boost::shared_mutex > lock(GetInterfaceMutex());

    uint32_t index = 0;
    while(properties) {
        if( properties & 1 ) {
            if( index >= _vlistRegisteredCallbacks.size() ) {
                // have to resize _vlistRegisteredCallbacks, but have to preserve the internal lists since ChangeCallbackData keep track of the list iterators
                std::vector<std::list<UserDataWeakPtr> > vlistRegisteredCallbacks(index+1);
                for(size_t i = 0; i < _vlistRegisteredCallbacks.size(); ++i) {
                    vlistRegisteredCallbacks[i].swap(_vlistRegisteredCallbacks[i]);
                }
                _vlistRegisteredCallbacks.swap(vlistRegisteredCallbacks);
            }
            pdata->_iterators.emplace_back(index, _vlistRegisteredCallbacks.at(index).insert(_vlistRegisteredCallbacks.at(index).end(), pdata));
        }
        properties >>= 1;
        index += 1;
    }
    return pdata;
}

void KinBody::_InitAndAddLink(LinkPtr plink)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    LinkInfo& info = plink->info_;

    // check to make sure there are no repeating names in already added links
    FOREACH(itlink, links_vector_) {
        if( (*itlink)->GetName() == info.name_ ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("link %s is declared more than once in body %s"), info.name_%GetName(), ORE_InvalidArguments);
        }
    }

    plink->index_ = static_cast<int>(links_vector_.size());
    plink->geometries_vector_.clear();
    plink->collision_.vertices.clear();
    plink->collision_.indices.clear();
    FOREACHC(itgeominfo,info.geometry_infos_vector_) {
        Link::GeometryPtr geom(new Link::Geometry(plink,**itgeominfo));
        if( geom->info_.mesh_collision_.vertices.size() == 0 ) { // try to avoid recomputing
            geom->info_.InitCollisionMesh();
        }
        plink->geometries_vector_.push_back(geom);
        plink->collision_.Append(geom->GetCollisionMesh(),geom->GetTransform());
    }
    FOREACHC(itadjacentname, info.forced_adjacent_links_vector_) {
        // make sure the same pair isn't added more than once
        std::pair<std::string, std::string> adjpair = std::make_pair(info.name_, *itadjacentname);
        if( find(_vForcedAdjacentLinks.begin(), _vForcedAdjacentLinks.end(), adjpair) == _vForcedAdjacentLinks.end() ) {
            _vForcedAdjacentLinks.push_back(adjpair);
        }
    }
    links_vector_.push_back(plink);
}

void KinBody::_InitAndAddJoint(JointPtr pjoint)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    // check to make sure there are no repeating names in already added links
    JointInfo& info = pjoint->info_;
    FOREACH(itjoint, joints_vector_) {
        if( (*itjoint)->GetName() == info.name_ ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("joint %s is declared more than once in body %s"), info.name_%GetName(), ORE_InvalidArguments);
        }
    }

    for(size_t i = 0; i < info._vmimic.size(); ++i) {
        if( !!info._vmimic[i] ) {
            pjoint->_vmimic[i].reset(new Mimic());
            pjoint->_vmimic[i]->_equations = info._vmimic[i]->_equations;
        }
    }
    LinkPtr plink0, plink1;
    FOREACHC(itlink, links_vector_) {
        if( (*itlink)->info_.name_ == info.link_name0_ ) {
            plink0 = *itlink;
            if( !!plink1 ) {
                break;
            }
            }
        if( (*itlink)->info_.name_ == info.link_name1_ ) {
            plink1 = *itlink;
            if( !!plink0 ) {
                break;
            }
        }
    }
    OPENRAVE_ASSERT_FORMAT(!!plink0&&!!plink1, "cannot find links '%s' and '%s' of body '%s' joint %s ", info.link_name0_%info.link_name1_%GetName()%info.name_, ORE_Failed);
    std::vector<Vector> vaxes(pjoint->GetDOF());
    std::copy(info.axes_vector_.begin(),info.axes_vector_.begin()+vaxes.size(), vaxes.begin());
    pjoint->_ComputeInternalInformation(plink0, plink1, info.anchor_, vaxes, info.current_values_vector_);
    if( info.is_active_ ) {
        joints_vector_.push_back(pjoint);
    }
    else {
        passive_joints_vector_.push_back(pjoint);
    }
}

} // end namespace OpenRAVE

// -*- coding: utf-8 -*-
// Copyright (C) 2006-2017 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <openrave/kinbody.h>

namespace OpenRAVE 
{

KinBody::LinkInfo::LinkInfo()
    : mass_(0)
    , is_static_(false)
    , is_enabled_(true)
{
}

KinBody::LinkInfo::LinkInfo(const LinkInfo& other)
{
    *this = other;
}

void KinBody::LinkInfo::SerializeJSON(rapidjson::Value& value,
    rapidjson::Document::AllocatorType& allocator, dReal unit_scale, int options) const
{
    openravejson::SetJsonValueByKey(value, "name", name_, allocator);

    Transform tmpTransform{ transform_ };
    Transform tmpMassTransform{ mass_frame_transform_ };
    tmpTransform.trans *= unit_scale;
    tmpMassTransform.trans *= unit_scale;

    openravejson::SetJsonValueByKey(value, "transform", tmpTransform, allocator);
    openravejson::SetJsonValueByKey(value, "massTransform", tmpMassTransform, allocator);
    openravejson::SetJsonValueByKey(value, "mass", mass_, allocator);
    openravejson::SetJsonValueByKey(value, "intertialMoments", inertia_moments_vector_, allocator);

    if (float_parameters_map_.size() > 0) {
        openravejson::SetJsonValueByKey(value, "floatParameters", float_parameters_map_, allocator);
    }

    if (int_parameters_map_.size() > 0) {
        openravejson::SetJsonValueByKey(value, "intParameters", int_parameters_map_, allocator);
    }

    if (string_parameters_map_.size() > 0) {
        openravejson::SetJsonValueByKey(value, "stringParameters", string_parameters_map_, allocator);
    }

    if (forced_adjacent_links_vector_.size() > 0) {
        openravejson::SetJsonValueByKey(value, "forcedAdjacentLinks", forced_adjacent_links_vector_, allocator);
    }

    if (geometry_infos_vector_.size() > 0) {
        rapidjson::Value geometriesValue;
        geometriesValue.SetArray();
        geometriesValue.Reserve(geometry_infos_vector_.size(), allocator);
        FOREACHC(it, geometry_infos_vector_)
        {
            rapidjson::Value geometryValue;
            (*it)->SerializeJSON(geometryValue, allocator, options);
            geometriesValue.PushBack(geometryValue, allocator);
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }

    if (extra_geometries_map_.size() > 0) {
        rapidjson::Value extraGeometriesValue;
        extraGeometriesValue.SetObject();
        FOREACHC(im, extra_geometries_map_)
        {
            rapidjson::Value geometriesValue;
            geometriesValue.SetArray();
            FOREACHC(iv, im->second)
            {
                if (!!(*iv)) {
                    rapidjson::Value geometryValue;
                    (*iv)->SerializeJSON(geometryValue, allocator);
                    geometriesValue.PushBack(geometryValue, allocator);
                }
            }
            extraGeometriesValue.AddMember(rapidjson::Value(im->first.c_str(), allocator).Move(), geometriesValue, allocator);
        }
        value.AddMember("extraGeometries", extraGeometriesValue, allocator);
    }

    openravejson::SetJsonValueByKey(value, "isStatic", is_static_, allocator);
    openravejson::SetJsonValueByKey(value, "isEnabled", is_enabled_, allocator);
}

void KinBody::LinkInfo::DeserializeJSON(const rapidjson::Value& value, dReal unit_scale)
{
    openravejson::LoadJsonValueByKey(value, "name", name_);
    openravejson::LoadJsonValueByKey(value, "transform", transform_);
    openravejson::LoadJsonValueByKey(value, "massTransform", mass_frame_transform_);
    openravejson::LoadJsonValueByKey(value, "mass", mass_);
    openravejson::LoadJsonValueByKey(value, "intertialMoments", inertia_moments_vector_);
    openravejson::LoadJsonValueByKey(value, "floatParameters", float_parameters_map_);
    openravejson::LoadJsonValueByKey(value, "intParameters", int_parameters_map_);
    openravejson::LoadJsonValueByKey(value, "stringParameters", string_parameters_map_);
    openravejson::LoadJsonValueByKey(value, "forcedAdjacentLinks", forced_adjacent_links_vector_);

    transform_.trans *= unit_scale;
    mass_frame_transform_.trans *= unit_scale;

    if (value.HasMember("geometries")) {
        geometry_infos_vector_.clear();
        geometry_infos_vector_.reserve(value["geometries"].Size());
        for (size_t i = 0; i < value["geometries"].Size(); ++i) {
            GeometryInfoPtr pGeometryInfo(new GeometryInfo());
            pGeometryInfo->DeserializeJSON(value["geometries"][i], unit_scale);
            geometry_infos_vector_.push_back(pGeometryInfo);
        }
    }
    if (value.HasMember("extraGeometries")) {
        extra_geometries_map_.clear();
        for (rapidjson::Value::ConstMemberIterator it = value["extraGeometries"].MemberBegin(); it != value["extraGeometries"].MemberEnd(); ++it) {
            extra_geometries_map_[it->name.GetString()] = std::vector<GeometryInfoPtr>();
            std::vector<GeometryInfoPtr>& vgeometries = extra_geometries_map_[it->name.GetString()];
            vgeometries.reserve(it->value.Size());

            for (rapidjson::Value::ConstValueIterator im = it->value.Begin(); im != it->value.End(); ++im) {
                GeometryInfoPtr pInfo(new GeometryInfo());
                pInfo->DeserializeJSON(*im, unit_scale);
                vgeometries.push_back(pInfo);
            }
        }
    }
}

KinBody::LinkInfo& KinBody::LinkInfo::operator=(const KinBody::LinkInfo& other)
{
    geometry_infos_vector_.resize(other.geometry_infos_vector_.size());
    for (size_t i = 0; i < geometry_infos_vector_.size(); ++i) 
	{
        if (!other.geometry_infos_vector_[i])
		{
            geometry_infos_vector_[i].reset();
        } else 
		{
            geometry_infos_vector_[i].reset(new GeometryInfo(*(other.geometry_infos_vector_[i])));
        }
    }

    extra_geometries_map_.clear();
    for (std::map<std::string, std::vector<GeometryInfoPtr>>::const_iterator it 
		= other.extra_geometries_map_.begin(); it != other.extra_geometries_map_.end(); ++it) 
	{
        extra_geometries_map_[it->first] = std::vector<GeometryInfoPtr>(it->second.size());
        std::vector<GeometryInfoPtr>& extraGeometries = extra_geometries_map_[it->first];
        for (size_t i = 0; i < extraGeometries.size(); ++i) 
		{
            if (!!(it->second[i]))
			{
                extraGeometries[i].reset(new GeometryInfo(*(it->second[i])));
            }
        }
    }

    name_ = other.name_;
    transform_ = other.transform_;
    mass_frame_transform_ = other.mass_frame_transform_;
    mass_ = other.mass_;
    inertia_moments_vector_ = other.inertia_moments_vector_;
    float_parameters_map_ = other.float_parameters_map_;
    int_parameters_map_ = other.int_parameters_map_;
    string_parameters_map_ = other.string_parameters_map_;
    forced_adjacent_links_vector_ = other.forced_adjacent_links_vector_;
    is_static_ = other.is_static_;
    is_enabled_ = other.is_enabled_;

    return *this;
}

KinBody::Link::Link(KinBodyPtr parent)
{
    parent_ = parent;
    index_ = -1;
}

KinBody::Link::~Link()
{
}

void KinBody::Link::Enable(bool is_enable)
{
    if (info_.is_enabled_ != is_enable) 
	{
        KinBodyPtr parent = GetParent();
        parent->non_adjacent_link_cache_ &= ~AO_Enabled;
        info_.is_enabled_ = is_enable;
        GetParent()->_PostprocessChangedParameters(Prop_LinkEnable);
    }
}

bool KinBody::Link::IsEnabled() const
{
    return info_.is_enabled_;
}

bool KinBody::Link::SetVisible(bool visible)
{
    bool is_changed = false;
    for(auto& itgeom: geometries_vector_)
    {
        if (itgeom->info_.is_visible_ != visible)
		{
            itgeom->info_.is_visible_ = visible;
            is_changed = true;
        }
    }
    if (is_changed)
	{
        GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
        return true;
    }
    return false;
}

bool KinBody::Link::IsVisible() const
{
    for(auto& itgeom: geometries_vector_)
    {
        if (itgeom->IsVisible())
		{
            return true;
        }
    }
    return false;
}

void KinBody::Link::GetParentLinks(std::vector<std::shared_ptr<Link>>& parent_links_vector) const
{
    KinBodyConstPtr parent(parent_);
    parent_links_vector.resize(parent_links_vector_.size());
    for (size_t i = 0; i < parent_links_vector_.size(); ++i)
	{
        parent_links_vector[i] = parent->GetLinks().at(parent_links_vector_[i]);
    }
}

bool KinBody::Link::IsParentLink(const Link& link) const
{
    return std::find(parent_links_vector_.begin(), parent_links_vector_.end(), link.GetIndex())
		!= parent_links_vector_.end();
}

/** mass_frame_transform_ * PrincipalInertia * mass_frame_transform_.inverse()

		from openravepy.ikfast import *
		quat = [Symbol('q0'),Symbol('q1'),Symbol('q2'),Symbol('q3')]
		IKFastSolver.matrixFromQuat(quat)
		Inertia = eye(3)
		Inertia[0,0] = Symbol('i0'); Inertia[1,1] = Symbol('i1'); Inertia[2,2] = Symbol('i2')
		MM = M * Inertia * M.transpose()
	 */
static TransformMatrix ComputeInertia(const Transform& tMassFrame, const Vector& vinertiamoments)
{
    TransformMatrix minertia;
    dReal i0 = vinertiamoments[0], i1 = vinertiamoments[1], i2 = vinertiamoments[2];
    dReal q0 = tMassFrame.rot[0], q1 = tMassFrame.rot[1], q2 = tMassFrame.rot[2], q3 = tMassFrame.rot[3];
    dReal q1_2 = q1 * q1, q2_2 = q2 * q2, q3_2 = q3 * q3;
    minertia.m[0] = i0 * utils::Sqr(1 - 2 * q2_2 - 2 * q3_2) + i1 * utils::Sqr(-2 * q0 * q3 + 2 * q1 * q2) + i2 * utils::Sqr(2 * q0 * q2 + 2 * q1 * q3);
    minertia.m[1] = i0 * (2 * q0 * q3 + 2 * q1 * q2) * (1 - 2 * q2_2 - 2 * q3_2) + i1 * (-2 * q0 * q3 + 2 * q1 * q2) * (1 - 2 * q1_2 - 2 * q3_2) + i2 * (-2 * q0 * q1 + 2 * q2 * q3) * (2 * q0 * q2 + 2 * q1 * q3);
    minertia.m[2] = i0 * (-2 * q0 * q2 + 2 * q1 * q3) * (1 - 2 * q2_2 - 2 * q3_2) + i1 * (-2 * q0 * q3 + 2 * q1 * q2) * (2 * q0 * q1 + 2 * q2 * q3) + i2 * (2 * q0 * q2 + 2 * q1 * q3) * (1 - 2 * q1_2 - 2 * q2_2);
    minertia.m[3] = 0;
    minertia.m[4] = minertia.m[1];
    minertia.m[5] = i0 * utils::Sqr(2 * q0 * q3 + 2 * q1 * q2) + i1 * utils::Sqr(1 - 2 * q1_2 - 2 * q3_2) + i2 * utils::Sqr(-2 * q0 * q1 + 2 * q2 * q3);
    minertia.m[6] = i0 * (-2 * q0 * q2 + 2 * q1 * q3) * (2 * q0 * q3 + 2 * q1 * q2) + i1 * (2 * q0 * q1 + 2 * q2 * q3) * (1 - 2 * q1_2 - 2 * q3_2) + i2 * (-2 * q0 * q1 + 2 * q2 * q3) * (1 - 2 * q1_2 - 2 * q2_2);
    minertia.m[7] = 0;
    minertia.m[8] = minertia.m[2];
    minertia.m[9] = minertia.m[6];
    minertia.m[10] = i0 * utils::Sqr(-2 * q0 * q2 + 2 * q1 * q3) + i1 * utils::Sqr(2 * q0 * q1 + 2 * q2 * q3) + i2 * utils::Sqr(1 - 2 * q1_2 - 2 * q2_2);
    minertia.m[11] = 0;
    return minertia;
}
TransformMatrix KinBody::Link::GetLocalInertia() const
{
    return ComputeInertia(info_.mass_frame_transform_, info_.inertia_moments_vector_);
}

TransformMatrix KinBody::Link::GetGlobalInertia() const
{
    return ComputeInertia(info_.transform_ * info_.mass_frame_transform_, info_.inertia_moments_vector_);
}

void KinBody::Link::SetLocalMassFrame(const Transform& massframe)
{
    info_.mass_frame_transform_ = massframe;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetPrincipalMomentsOfInertia(const Vector& inertiamoments)
{
    info_.inertia_moments_vector_ = inertiamoments;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetMass(dReal mass)
{
    info_.mass_ = mass;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

AABB KinBody::Link::ComputeLocalAABB() const
{
    return ComputeAABBFromTransform(Transform());
}

AABB KinBody::Link::ComputeAABB() const
{
    return ComputeAABBFromTransform(info_.transform_);
}

AABB KinBody::Link::ComputeAABBFromTransform(const Transform& tLink) const
{
    if (geometries_vector_.size() == 1) 
	{
        return geometries_vector_.front()->ComputeAABB(tLink);
    } else if (geometries_vector_.size() > 1)
	{
        Vector vmin, vmax;
        bool binitialized = false;
        AABB ab;
        FOREACHC(itgeom, geometries_vector_)
        {
            ab = (*itgeom)->ComputeAABB(tLink);
            if (ab.extents.x <= 0 || ab.extents.y <= 0 || ab.extents.z <= 0) {
                continue;
            }
            Vector vnmin = ab.pos - ab.extents;
            Vector vnmax = ab.pos + ab.extents;
            if (!binitialized) {
                vmin = vnmin;
                vmax = vnmax;
                binitialized = true;
            } else {
                if (vmin.x > vnmin.x) {
                    vmin.x = vnmin.x;
                }
                if (vmin.y > vnmin.y) {
                    vmin.y = vnmin.y;
                }
                if (vmin.z > vnmin.z) {
                    vmin.z = vnmin.z;
                }
                if (vmax.x < vnmax.x) {
                    vmax.x = vnmax.x;
                }
                if (vmax.y < vnmax.y) {
                    vmax.y = vnmax.y;
                }
                if (vmax.z < vnmax.z) {
                    vmax.z = vnmax.z;
                }
            }
        }
        if (!binitialized) {
            ab.pos = tLink.trans;
            ab.extents = Vector(0, 0, 0);
        } else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    // have to at least return the correct position!
    return AABB(tLink.trans, Vector(0, 0, 0));
}

void KinBody::Link::serialize(std::ostream& o, int options) const
{
    o << index_ << " ";
    if (options & SO_Geometry) {
        o << geometries_vector_.size() << " ";
        FOREACHC(it, geometries_vector_)
        {
            (*it)->serialize(o, options);
        }
    }
    if (options & SO_Dynamics) {
        SerializeRound(o, info_.mass_frame_transform_);
        SerializeRound(o, info_.mass_);
        SerializeRound3(o, info_.inertia_moments_vector_);
    }
}

void KinBody::Link::SetStatic(bool is_static)
{
    if (info_.is_static_ != is_static) 
	{
        info_.is_static_ = is_static;
        GetParent()->_PostprocessChangedParameters(Prop_LinkStatic);
    }
}

void KinBody::Link::SetTransform(const Transform& t)
{
    info_.transform_ = t;
    GetParent()->update_stamp_id_++;
}

void KinBody::Link::SetForce(const Vector& force, const Vector& pos, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyForce(shared_from_this(), force, pos, bAdd);
}

void KinBody::Link::SetTorque(const Vector& torque, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyTorque(shared_from_this(), torque, bAdd);
}

void KinBody::Link::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

void KinBody::Link::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetParent()->GetEnv()->GetPhysicsEngine()->GetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

/// \brief return the linear/angular velocity of the link
std::pair<Vector, Vector> KinBody::Link::GetVelocity() const
{
    std::pair<Vector, Vector> velocities;
    GetParent()->GetEnv()->GetPhysicsEngine()->GetLinkVelocity(shared_from_this(), velocities.first, velocities.second);
    return velocities;
}

KinBody::Link::GeometryPtr KinBody::Link::GetGeometry(int index)
{
    return geometries_vector_.at(index);
}

void KinBody::Link::InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries, bool is_force_recompute_mesh_collision)
{
    geometries_vector_.resize(geometries.size());
    for (size_t i = 0; i < geometries.size(); ++i) {
        geometries_vector_[i].reset(new Geometry(shared_from_this(), *geometries[i]));
        if (is_force_recompute_mesh_collision || geometries_vector_[i]->GetCollisionMesh().vertices.size() == 0) {
            if (!is_force_recompute_mesh_collision) {
                RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            }
            geometries_vector_[i]->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
    }
    info_.extra_geometries_map_.clear();
    // have to reset the self group! cannot use geometries directly since we require exclusive access to the GeometryInfo objects
    std::vector<KinBody::GeometryInfoPtr> vgeometryinfos;
    vgeometryinfos.resize(geometries_vector_.size());
    for (size_t i = 0; i < vgeometryinfos.size(); ++i) {
        vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        *vgeometryinfos[i] = geometries_vector_[i]->info_;
    }
    SetGroupGeometries("self", vgeometryinfos);
    _Update();
}

void KinBody::Link::InitGeometries(std::list<KinBody::GeometryInfo>& geometries, bool is_force_recompute_mesh_collision)
{
    geometries_vector_.resize(geometries.size());
    size_t i = 0;
    FOREACH(itinfo, geometries)
    {
        geometries_vector_[i].reset(new Geometry(shared_from_this(), *itinfo));
        if (geometries_vector_[i]->GetCollisionMesh().vertices.size() == 0) {
            RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            geometries_vector_[i]->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
        ++i;
    }
    info_.extra_geometries_map_.clear();
    // have to reset the self group!
    std::vector<KinBody::GeometryInfoPtr> vgeometryinfos;
    vgeometryinfos.resize(geometries_vector_.size());
    for (size_t i = 0; i < vgeometryinfos.size(); ++i) {
        vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        *vgeometryinfos[i] = geometries_vector_[i]->info_;
    }
    SetGroupGeometries("self", vgeometryinfos);
    _Update();
}

void KinBody::Link::SetGeometriesFromGroup(const std::string& groupname)
{
    std::vector<KinBody::GeometryInfoPtr>* pvinfos = NULL;
    if (groupname.size() == 0) {
        pvinfos = &info_.geometry_infos_vector_;
    } else {
        std::map<std::string, std::vector<KinBody::GeometryInfoPtr>>::iterator it = info_.extra_geometries_map_.find(groupname);
        if (it == info_.extra_geometries_map_.end()) {
            throw OPENRAVE_EXCEPTION_FORMAT(_tr("could not find geometries %s for link %s"), groupname % GetName(), ORE_InvalidArguments);
        }
        pvinfos = &it->second;
    }
    geometries_vector_.resize(pvinfos->size());
    for (size_t i = 0; i < pvinfos->size(); ++i) {
        geometries_vector_[i].reset(new Geometry(shared_from_this(), *pvinfos->at(i)));
        if (geometries_vector_[i]->GetCollisionMesh().vertices.size() == 0) {
            RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            geometries_vector_[i]->InitCollisionMesh();
        }
    }
    _Update();
}

const std::vector<KinBody::GeometryInfoPtr>& KinBody::Link::GetGeometriesFromGroup(const std::string& groupname) const
{
    std::map<std::string, std::vector<KinBody::GeometryInfoPtr>>::const_iterator it = info_.extra_geometries_map_.find(groupname);
    if (it == info_.extra_geometries_map_.end()) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("geometry group %s does not exist for link %s"), groupname % GetName(), ORE_InvalidArguments);
    }
    return it->second;
}

void KinBody::Link::SetGroupGeometries(const std::string& groupname, const std::vector<KinBody::GeometryInfoPtr>& geometries)
{
    FOREACH(itgeominfo, geometries)
    {
        if (!(*itgeominfo)) {
            int igeominfo = itgeominfo - geometries.begin();
            throw OPENRAVE_EXCEPTION_FORMAT("GeometryInfo index %d is invalid for body %s", igeominfo % GetParent()->GetName(), ORE_InvalidArguments);
        }
    }
    std::map<std::string, std::vector<KinBody::GeometryInfoPtr>>::iterator it = info_.extra_geometries_map_.insert(make_pair(groupname, std::vector<KinBody::GeometryInfoPtr>())).first;
    it->second.resize(geometries.size());
    std::copy(geometries.begin(), geometries.end(), it->second.begin());
    GetParent()->_PostprocessChangedParameters(Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

int KinBody::Link::GetGroupNumGeometries(const std::string& groupname) const
{
    std::map<std::string, std::vector<KinBody::GeometryInfoPtr>>::const_iterator it = info_.extra_geometries_map_.find(groupname);
    if (it == info_.extra_geometries_map_.end()) {
        return -1;
    }
    return it->second.size();
}

void KinBody::Link::AddGeometry(KinBody::GeometryInfoPtr pginfo, bool addToGroups)
{
    if (!pginfo) {
        throw OPENRAVE_EXCEPTION_FORMAT(_tr("tried to add improper geometry to link %s"), GetName(), ORE_InvalidArguments);
    }

    const KinBody::GeometryInfo& ginfo = *pginfo;
    if (ginfo.name_.size() > 0) {
        // check if similar name exists and throw if it does
        FOREACH(itgeometry, geometries_vector_)
        {
            if ((*itgeometry)->GetName() == ginfo.name_) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("new added geometry %s has conflicting name for link %s"), ginfo.name_ % GetName(), ORE_InvalidArguments);
            }
        }

        FOREACH(itgeometryinfo, info_.geometry_infos_vector_)
        {
            if ((*itgeometryinfo)->name_ == ginfo.name_) {
                throw OPENRAVE_EXCEPTION_FORMAT(_tr("new added geometry %s has conflicting name for link %s"), ginfo.name_ % GetName(), ORE_InvalidArguments);
            }
        }
        if (addToGroups) {
            FOREACH(itgeometrygroup, info_.extra_geometries_map_)
            {
                FOREACH(itgeometryinfo, itgeometrygroup->second)
                {
                    if ((*itgeometryinfo)->name_ == ginfo.name_) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_tr("new added geometry %s for group %s has conflicting name for link %s"), ginfo.name_ % itgeometrygroup->first % GetName(), ORE_InvalidArguments);
                    }
                }
            }
        }
    }

    geometries_vector_.push_back(GeometryPtr(new Geometry(shared_from_this(), *pginfo)));
    geometries_vector_.back()->InitCollisionMesh();
    info_.geometry_infos_vector_.push_back(pginfo);
    if (addToGroups) {
        FOREACH(itgeometrygroup, info_.extra_geometries_map_)
        {
            itgeometrygroup->second.push_back(pginfo);
        }
    }
    _Update(true, Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

void KinBody::Link::RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups)
{
    OPENRAVE_ASSERT_OP(geometryname.size(), >, 0);
    bool bChanged = false;

    std::vector<GeometryPtr>::iterator itgeometry = geometries_vector_.begin();
    while (itgeometry != geometries_vector_.end()) {
        if ((*itgeometry)->GetName() == geometryname) {
            itgeometry = geometries_vector_.erase(itgeometry);
            bChanged = true;
        } else {
            ++itgeometry;
        }
    }
    std::vector<KinBody::GeometryInfoPtr>::iterator itgeometryinfo = info_.geometry_infos_vector_.begin();
    while (itgeometryinfo != info_.geometry_infos_vector_.end()) {
        if ((*itgeometryinfo)->name_ == geometryname) {
            itgeometryinfo = info_.geometry_infos_vector_.erase(itgeometryinfo);
            bChanged = true;
        } else {
            ++itgeometryinfo;
        }
    }

    if (removeFromAllGroups) {
        FOREACH(itgeometrygroup, info_.extra_geometries_map_)
        {
            std::vector<KinBody::GeometryInfoPtr>::iterator itgeometryinfo2 = itgeometrygroup->second.begin();
            while (itgeometryinfo2 != itgeometrygroup->second.end()) {
                if ((*itgeometryinfo2)->name_ == geometryname) {
                    itgeometryinfo2 = itgeometrygroup->second.erase(itgeometryinfo2);
                    bChanged = true;
                } else {
                    ++itgeometryinfo2;
                }
            }
        }
    }

    if (bChanged) {
        _Update(true, Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
    }
}

void KinBody::Link::SwapGeometries(std::shared_ptr<Link>& link)
{
    geometries_vector_.swap(link->geometries_vector_);
    FOREACH(itgeom, geometries_vector_)
    {
        (*itgeom)->parent_ = shared_from_this();
    }
    FOREACH(itgeom, link->geometries_vector_)
    {
        (*itgeom)->parent_ = link;
    }
    _Update();
    link->_Update();
}

bool KinBody::Link::ValidateContactNormal(const Vector& position, Vector& normal) const
{
    if (geometries_vector_.size() == 1) {
        return geometries_vector_.front()->ValidateContactNormal(position, normal);
    } else if (geometries_vector_.size() > 1) {
        RAVELOG_VERBOSE(str(boost::format("cannot validate normal when there is more than one geometry in link '%s(%d)' (do not know colliding geometry)") % info_.name_ % GetIndex()));
    }
    return false;
}

void KinBody::Link::GetRigidlyAttachedLinks(std::vector<std::shared_ptr<Link>>& vattachedlinks) const
{
    KinBodyPtr parent(parent_);
    vattachedlinks.resize(0);
    FOREACHC(it, _vRigidlyAttachedLinks)
    {
        vattachedlinks.push_back(parent->GetLinks().at(*it));
    }
}

void KinBody::Link::SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters)
{
    if (parameters.size() > 0) {
        info_.float_parameters_map_[key] = parameters;
    } else {
        info_.float_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if (parameters.size() > 0) {
        info_.int_parameters_map_[key] = parameters;
    } else {
        info_.int_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetStringParameters(const std::string& key, const std::string& value)
{
    if (value.size() > 0) {
        info_.string_parameters_map_[key] = value;
    } else {
        info_.string_parameters_map_.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

bool KinBody::Link::IsRigidlyAttached(const Link& link) const
{
    return find(_vRigidlyAttachedLinks.begin(), _vRigidlyAttachedLinks.end(), link.GetIndex()) != _vRigidlyAttachedLinks.end();
}

void KinBody::Link::UpdateInfo()
{
    // always have to recompute the geometries
    info_.geometry_infos_vector_.resize(geometries_vector_.size());
    for (size_t i = 0; i < info_.geometry_infos_vector_.size(); ++i) {
        if (!info_.geometry_infos_vector_[i]) {
            info_.geometry_infos_vector_[i].reset(new KinBody::GeometryInfo());
        }
        *info_.geometry_infos_vector_[i] = geometries_vector_[i]->GetInfo();
    }
}

void KinBody::Link::_Update(bool parameterschanged, uint32_t extraParametersChanged)
{
    // if there's only one trimesh geometry and it has identity offset, then copy it directly
    if (geometries_vector_.size() == 1 && geometries_vector_.at(0)->GetType() == GT_TriMesh && TransformDistanceFast(Transform(), geometries_vector_.at(0)->GetTransform()) <= g_fEpsilonLinear) {
        collision_ = geometries_vector_.at(0)->GetCollisionMesh();
    } else {
        collision_.vertices.resize(0);
        collision_.indices.resize(0);
        FOREACH(itgeom, geometries_vector_)
        {
            collision_.Append((*itgeom)->GetCollisionMesh(), (*itgeom)->GetTransform());
        }
    }
    if (parameterschanged || extraParametersChanged) {
        GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry | extraParametersChanged);
    }
}
}

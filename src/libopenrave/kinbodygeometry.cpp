// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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

#include <openrave/kinbody.h>
#include <openrave/tri_mesh.h>
#include <openrave/openrave_exception.h>
#include <openrave/json.h>
#include <openrave/openravejson.h>

namespace OpenRAVE
{
	KinBody::GeometryInfo::GeometryInfo()
	{
		diffuse_color_vec_ = Vector(1, 1, 1);
		type_ = GT_None;
		transparency_ = 0;
		render_scale_vec_ = collision_scale_vec_ = Vector(1, 1, 1);
		is_visible_ = true;
		is_modifiable_ = true;
	}

	bool KinBody::GeometryInfo::InitCollisionMesh(float tessellation)
	{
		if (type_ == GT_TriMesh || type_ == GT_None)
		{
			return true;
		}

		// is clear() better since it releases the memory?
		mesh_collision_.indices.resize(0);
		mesh_collision_.vertices.resize(0);

		if (tessellation < 0.01f)
		{
			tessellation = 0.01f;
		}
		// start tesselating
		switch (type_)
		{
		case GT_Sphere:
		{
			// log_2 (1+ tess)
			GenerateSphereTriangulation(mesh_collision_, 3 + (int)(logf(tessellation) / logf(2.0f)));
			dReal radius = GetSphereRadius();
			for (auto& it : mesh_collision_.vertices)
			{
				it *= radius;
			}
			break;
		}
		case GT_Box:
		{
			// trivial
			Vector ex = GetBoxExtents();
			Vector v[8] = { Vector(ex.x, ex.y, ex.z),
							Vector(ex.x, ex.y, -ex.z),
							Vector(ex.x, -ex.y, ex.z),
							Vector(ex.x, -ex.y, -ex.z),
							Vector(-ex.x, ex.y, ex.z),
							Vector(-ex.x, ex.y, -ex.z),
							Vector(-ex.x, -ex.y, ex.z),
							Vector(-ex.x, -ex.y, -ex.z) };
			const int nindices = 36;
			int indices[] = {
				0, 2, 1,
				1, 2, 3,
				4, 5, 6,
				5, 7, 6,
				0, 1, 4,
				1, 5, 4,
				2, 6, 3,
				3, 6, 7,
				0, 4, 2,
				2, 4, 6,
				1, 3, 5,
				3, 7, 5
			};
			mesh_collision_.vertices.resize(8);
			std::copy(&v[0], &v[8], mesh_collision_.vertices.begin());
			mesh_collision_.indices.resize(nindices);
			std::copy(&indices[0], &indices[nindices], mesh_collision_.indices.begin());
			break;
		}
		case GT_Cylinder:
		{
			// cylinder is on z axis
			dReal rad = GetCylinderRadius(), len = GetCylinderHeight()*0.5f;
			int numverts = (int)(tessellation*48.0f) + 3;
			dReal dtheta = 2 * PI / (dReal)numverts;
			mesh_collision_.vertices.push_back(Vector(0, 0, len));
			mesh_collision_.vertices.push_back(Vector(0, 0, -len));
			mesh_collision_.vertices.push_back(Vector(rad, 0, len));
			mesh_collision_.vertices.push_back(Vector(rad, 0, -len));
			for (int i = 0; i < numverts + 1; ++i)
			{
				dReal s = rad * RaveSin(dtheta * (dReal)i);
				dReal c = rad * RaveCos(dtheta * (dReal)i);
				int off = (int)mesh_collision_.vertices.size();
				mesh_collision_.vertices.push_back(Vector(c, s, len));
				mesh_collision_.vertices.push_back(Vector(c, s, -len));

				mesh_collision_.indices.push_back(0);       mesh_collision_.indices.push_back(off - 2);       mesh_collision_.indices.push_back(off);
				mesh_collision_.indices.push_back(1);       mesh_collision_.indices.push_back(off + 1);       mesh_collision_.indices.push_back(off - 1);
				mesh_collision_.indices.push_back(off - 2);   mesh_collision_.indices.push_back(off - 1);         mesh_collision_.indices.push_back(off);
				mesh_collision_.indices.push_back(off);   mesh_collision_.indices.push_back(off - 1);         mesh_collision_.indices.push_back(off + 1);
			}
			break;
		}
		case GT_Cage:
		{
			const Vector& vCageBaseExtents = gemo_outer_extents_data_;
			for (size_t i = 0; i < side_walls_vector_.size(); ++i)
			{
				const SideWall &s = side_walls_vector_[i];
				const size_t vBase = mesh_collision_.vertices.size();
				AppendBoxTriangulation(Vector(0, 0, s.vExtents[2]), s.vExtents, mesh_collision_);

				for (size_t j = 0; j < 8; ++j) {
					mesh_collision_.vertices[vBase + j] = s.transf * mesh_collision_.vertices[vBase + j];
				}
			}
			// finally add the base
			AppendBoxTriangulation(Vector(0, 0, vCageBaseExtents.z), vCageBaseExtents, mesh_collision_);
			break;
		}
		case GT_Container:
		{
			const Vector& outerextents = gemo_outer_extents_data_;
			const Vector& innerextents = geom_inner_extents_data_;
			const Vector& bottomcross = geom_bottom_cross_data_;
			const Vector& bottom = geom_bottom_data_;
			dReal zoffset = 0;
			if (bottom[2] > 0)
			{
				if (bottom[0] > 0 && bottom[1] > 0)
				{
					zoffset = bottom[2];
				}
			}
			// +x wall
			AppendBoxTriangulation(Vector((outerextents[0] + innerextents[0]) / 4., 0, outerextents[2] / 2. + zoffset),
				Vector((outerextents[0] - innerextents[0]) / 4., outerextents[1] / 2., outerextents[2] / 2.), mesh_collision_);
			// -x wall
			AppendBoxTriangulation(Vector(-(outerextents[0] + innerextents[0]) / 4., 0, outerextents[2] / 2. + zoffset),
				Vector((outerextents[0] - innerextents[0]) / 4., outerextents[1] / 2., outerextents[2] / 2.), mesh_collision_);
			// +y wall
			AppendBoxTriangulation(Vector(0, (outerextents[1] + innerextents[1]) / 4., outerextents[2] / 2. + zoffset),
				Vector(outerextents[0] / 2., (outerextents[1] - innerextents[1]) / 4., outerextents[2] / 2.), mesh_collision_);
			// -y wall
			AppendBoxTriangulation(Vector(0, -(outerextents[1] + innerextents[1]) / 4., outerextents[2] / 2. + zoffset),
				Vector(outerextents[0] / 2., (outerextents[1] - innerextents[1]) / 4., outerextents[2] / 2.), mesh_collision_);
			// bottom
			if (outerextents[2] - innerextents[2] >= 1e-6) { // small epsilon error can make thin triangles appear, so test with a reasonable threshold
				AppendBoxTriangulation(Vector(0, 0, (outerextents[2] - innerextents[2]) / 2. + zoffset),
					Vector(outerextents[0] / 2., outerextents[1] / 2., (outerextents[2] - innerextents[2]) / 2),
					mesh_collision_);
			}
			// cross
			if (bottomcross[2] > 0)
			{
				if (bottomcross[0] > 0)
				{
					AppendBoxTriangulation(Vector(0, 0, bottomcross[2] / 2 + outerextents[2] - innerextents[2] + zoffset), Vector(bottomcross[0] / 2, innerextents[1] / 2, bottomcross[2] / 2), mesh_collision_);
				}
				if (bottomcross[1] > 0) {
					AppendBoxTriangulation(Vector(0, 0, bottomcross[2] / 2 + outerextents[2] - innerextents[2] + zoffset), Vector(innerextents[0] / 2, bottomcross[1] / 2, bottomcross[2] / 2), mesh_collision_);
				}
			}
			// bottom
			if (bottom[2] > 0) {
				if (bottom[0] > 0 && bottom[1] > 0) {
					AppendBoxTriangulation(Vector(0, 0, bottom[2] / 2), Vector(bottom[0] / 2., bottom[1] / 2., bottom[2] / 2.), mesh_collision_);
				}
			}
			break;
		}
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(("unrecognized geom type %d!"), type_, ORE_InvalidArguments);
		}

		return true;
	}

	bool KinBody::GeometryInfo::ComputeInnerEmptyVolume(
		Transform& inner_empty_volume,
		Vector& inner_empty_extents) const
	{
		switch (type_) 
		{
		case GT_Cage: 
		{
			Vector vwallmin, vwallmax;
			vwallmax.z = vwallmin.z = gemo_outer_extents_data_.z * 2;

			// initialize to the base extents if there is no wall
			vwallmin.x = -gemo_outer_extents_data_.x;
			vwallmin.y = -gemo_outer_extents_data_.y;
			vwallmax.x = gemo_outer_extents_data_.x;
			vwallmax.y = gemo_outer_extents_data_.y;
			int sideWallExtents = 0;

			for (auto& itwall : side_walls_vector_) {
				// compute the XYZ extents of the wall
				Vector vxaxis = geometry::ExtractAxisFromQuat(itwall.transf.rot, 0);
				Vector vyaxis = geometry::ExtractAxisFromQuat(itwall.transf.rot, 1);
				Vector vzaxis = geometry::ExtractAxisFromQuat(itwall.transf.rot, 2);

				Vector vprojectedextents;
				for (int idim = 0; idim < 3; ++idim)
				{
					vprojectedextents[idim] = RaveFabs(vxaxis[idim])*itwall.vExtents.x
						+ RaveFabs(vyaxis[idim])*itwall.vExtents.y
						+ RaveFabs(vzaxis[idim])*itwall.vExtents.z;
				}

				// the origin of the side wall is the bottom center
				if (vwallmax.z < itwall.transf.trans.z + 2 * vprojectedextents.z)
				{
					vwallmax.z = itwall.transf.trans.z + 2 * vprojectedextents.z;
				}

				sideWallExtents |= (int)(1 << itwall.type);

				switch (itwall.type) {
				case GeometryInfo::SWT_NX:
					vwallmin.x = itwall.transf.trans.x + vprojectedextents.x;
					break;
				case GeometryInfo::SWT_NY:
					vwallmin.y = itwall.transf.trans.y + vprojectedextents.y;
					break;
				case GeometryInfo::SWT_PX:
					vwallmax.x = itwall.transf.trans.x - vprojectedextents.x;
					break;
				case GeometryInfo::SWT_PY:
					vwallmax.y = itwall.transf.trans.y - vprojectedextents.y;
					break;
				}
			}

			// if geom_inner_extents_data_ is greater than 0, force inner region wherever possible.
			// The only thing that will prevent geom_inner_extents_data_'s inner region is a wall present.
			// Should not use base to restrict geom_inner_extents_data_
			if (geom_inner_extents_data_.x > 0) {
				if (sideWallExtents & (1 << GeometryInfo::SWT_NX)) {
					if (vwallmin.x < -0.5*geom_inner_extents_data_.x) {
						vwallmin.x = -0.5*geom_inner_extents_data_.x;
					}
				}
				else {
					// no wall defined on NX
					vwallmin.x = -0.5*geom_inner_extents_data_.x;
				}

				if (sideWallExtents & (1 << GeometryInfo::SWT_PX)) {
					if (vwallmax.x > 0.5*geom_inner_extents_data_.x) {
						vwallmax.x = 0.5*geom_inner_extents_data_.x;
					}
				}
				else {
					// no wall defined on NX
					vwallmax.x = 0.5*geom_inner_extents_data_.x;
				}
			}

			if (geom_inner_extents_data_.y > 0) {
				if (sideWallExtents & (1 << GeometryInfo::SWT_NY)) {
					if (vwallmin.y < -0.5*geom_inner_extents_data_.y) {
						vwallmin.y = -0.5*geom_inner_extents_data_.y;
					}
				}
				else {
					vwallmin.y = -0.5*geom_inner_extents_data_.y;
				}

				if (sideWallExtents & (1 << GeometryInfo::SWT_PY)) {
					if (vwallmax.y > 0.5*geom_inner_extents_data_.y) {
						vwallmax.y = 0.5*geom_inner_extents_data_.y;
					}
				}
				else {
					vwallmax.y = 0.5*geom_inner_extents_data_.y;
				}
			}

			// the top has no constraints, so use the max of walls and force inner region
			if (vwallmax.z < gemo_outer_extents_data_.z * 2 + geom_inner_extents_data_.z) {
				vwallmax.z = gemo_outer_extents_data_.z * 2 + geom_inner_extents_data_.z;
			}

			inner_empty_extents = 0.5*(vwallmax - vwallmin);
			inner_empty_volume = transform_;
			inner_empty_volume.trans += inner_empty_volume.rotate(0.5*(vwallmax + vwallmin));
			return true;
		}
		case GT_Container: {
			Transform tempty;
			// full outer extents - full inner extents + inner extents = gemo_outer_extents_data_.z - 0.5*geom_inner_extents_data_.z
			tempty.trans.z = gemo_outer_extents_data_.z - 0.5 * geom_inner_extents_data_.z;
			if (geom_bottom_data_.x > 0 && geom_bottom_data_.y > 0 && geom_bottom_data_.z > 0) {
				// if geom_bottom_data_ is valid, need to shift the empty region up.
				tempty.trans.z += geom_bottom_data_.z;
			}
			inner_empty_volume = transform_ * tempty;
			inner_empty_extents = 0.5*geom_inner_extents_data_;
			return true;
		}
		default:
			return false;
		}
	}

	inline void SaveJsonValue(rapidjson::Value& v, const KinBody::GeometryInfo::SideWall& t,
		rapidjson::Document::AllocatorType& alloc) 
	{
		v.SetObject();
		openravejson::SetJsonValueByKey(v, "transform", t.transf, alloc);
		openravejson::SetJsonValueByKey(v, "halfExtents", t.vExtents, alloc);
		openravejson::SetJsonValueByKey(v, "type", (int)t.type, alloc);
	}

	inline void LoadJsonValue(const rapidjson::Value& v, KinBody::GeometryInfo::SideWall& t) 
	{
		if (v.IsObject()) 
		{
			openravejson::LoadJsonValueByKey(v, "transform", t.transf);
			openravejson::LoadJsonValueByKey(v, "halfExtents", t.vExtents);
			int type = 0;
			openravejson::LoadJsonValueByKey(v, "type", type);
			t.type = (KinBody::GeometryInfo::SideWallType)type;
		}
		else {
			throw openravejson::OpenRAVEJSONException("Cannot convert json type " + openravejson::GetJsonTypeName(v) + " to OpenRAVE::Geometry::SideWall");
		}
	}

	void KinBody::GeometryInfo::SerializeJSON(rapidjson::Value& value,
		rapidjson::Document::AllocatorType& allocator, const dReal unit_scale, int options) const
	{
		// RAVE_SERIALIZEJSON_ADDMEMBER(allocator, "sid", sid);
		openravejson::SetJsonValueByKey(value, "name", name_, allocator);

		Transform tscaled = transform_;
		tscaled.trans *= unit_scale;
		openravejson::SetJsonValueByKey(value, "transform", tscaled, allocator);

		switch (type_) {
		case GT_Box:
			openravejson::SetJsonValueByKey(value, "type", "box", allocator);
			openravejson::SetJsonValueByKey(value, "halfExtents", gemo_outer_extents_data_*unit_scale, allocator);
			break;

		case GT_Container:
			openravejson::SetJsonValueByKey(value, "type", "container", allocator);
			openravejson::SetJsonValueByKey(value, "outerExtents", gemo_outer_extents_data_*unit_scale, allocator);
			openravejson::SetJsonValueByKey(value, "innerExtents", geom_inner_extents_data_*unit_scale, allocator);
			openravejson::SetJsonValueByKey(value, "bottomCross", geom_bottom_cross_data_*unit_scale, allocator);
			openravejson::SetJsonValueByKey(value, "bottom", geom_bottom_data_*unit_scale, allocator);
			break;

		case GT_Cage: {
			openravejson::SetJsonValueByKey(value, "type", "cage", allocator);
			openravejson::SetJsonValueByKey(value, "baseExtents", gemo_outer_extents_data_*unit_scale, allocator);

			std::vector<SideWall> vScaledSideWalls = side_walls_vector_;
			FOREACH(itwall, vScaledSideWalls) {
				itwall->transf.trans *= unit_scale;
				itwall->vExtents *= unit_scale;
			}
			if (geom_inner_extents_data_.x > g_fEpsilon) {
				openravejson::SetJsonValueByKey(value, "innerSizeX", geom_inner_extents_data_.x*unit_scale, allocator);
			}
			if (geom_inner_extents_data_.y > g_fEpsilon) {
				openravejson::SetJsonValueByKey(value, "innerSizeY", geom_inner_extents_data_.y*unit_scale, allocator);
			}
			if (geom_inner_extents_data_.z > g_fEpsilon) {
				openravejson::SetJsonValueByKey(value, "innerSizeZ", geom_inner_extents_data_.z*unit_scale, allocator);
			}
			openravejson::SetJsonValueByKey(value, "sideWalls", vScaledSideWalls, allocator);
			break;
		}
		case GT_Sphere:
			openravejson::SetJsonValueByKey(value, "type", "sphere", allocator);
			openravejson::SetJsonValueByKey(value, "radius", gemo_outer_extents_data_.x*unit_scale, allocator);
			break;

		case GT_Cylinder:
			openravejson::SetJsonValueByKey(value, "type", "cylinder", allocator);
			openravejson::SetJsonValueByKey(value, "radius", gemo_outer_extents_data_.x*unit_scale, allocator);
			openravejson::SetJsonValueByKey(value, "height", gemo_outer_extents_data_.y*unit_scale, allocator);
			break;

		case GT_TriMesh:
			openravejson::SetJsonValueByKey(value, "type", "trimesh", allocator);
			openravejson::SetJsonValueByKey(value, "mesh", mesh_collision_, allocator);
			break;

		default:
			break;
		}

		openravejson::SetJsonValueByKey(value, "transparency", transparency_, allocator);
		openravejson::SetJsonValueByKey(value, "visible", is_visible_, allocator);
		openravejson::SetJsonValueByKey(value, "diffuseColor", diffuse_color_vec_, allocator);
		openravejson::SetJsonValueByKey(value, "ambientColor", ambient_color_vec_, allocator);
		openravejson::SetJsonValueByKey(value, "modifiable", is_modifiable_, allocator);
	}

	void KinBody::GeometryInfo::DeserializeJSON(const rapidjson::Value &value, const dReal unit_scale)
	{
		openravejson::LoadJsonValueByKey(value, "name", name_);
		openravejson::LoadJsonValueByKey(value, "transform", transform_);

		transform_.trans *= unit_scale;

		std::string typestr;
		openravejson::LoadJsonValueByKey(value, "type", typestr);

		if (typestr == "box") {
			type_ = GT_Box;
			openravejson::LoadJsonValueByKey(value, "halfExtents", gemo_outer_extents_data_);
			gemo_outer_extents_data_ *= unit_scale;
		}
		else if (typestr == "container") {
			type_ = GT_Container;
			openravejson::LoadJsonValueByKey(value, "outerExtents", gemo_outer_extents_data_);
			openravejson::LoadJsonValueByKey(value, "innerExtents", geom_inner_extents_data_);

			geom_bottom_cross_data_ = Vector();
			openravejson::LoadJsonValueByKey(value, "bottomCross", geom_bottom_cross_data_);

			geom_bottom_data_ = Vector();
			openravejson::LoadJsonValueByKey(value, "bottom", geom_bottom_data_);

			gemo_outer_extents_data_ *= unit_scale;
			geom_inner_extents_data_ *= unit_scale;
			geom_bottom_cross_data_ *= unit_scale;
			geom_bottom_data_ *= unit_scale;
		}
		else if (typestr == "cage") {
			type_ = GT_Cage;
			openravejson::LoadJsonValueByKey(value, "baseExtents", gemo_outer_extents_data_);
			gemo_outer_extents_data_ *= unit_scale;

			geom_inner_extents_data_ = Vector();
			openravejson::LoadJsonValueByKey(value, "innerSizeX", geom_inner_extents_data_.x);
			openravejson::LoadJsonValueByKey(value, "innerSizeY", geom_inner_extents_data_.y);
			openravejson::LoadJsonValueByKey(value, "innerSizeZ", geom_inner_extents_data_.z);
			geom_inner_extents_data_ *= unit_scale;

			openravejson::LoadJsonValueByKey(value, "sideWalls", side_walls_vector_);
			for (auto& itsidewall : side_walls_vector_) {
				itsidewall.transf.trans *= unit_scale;
				itsidewall.vExtents *= unit_scale;
			}
		}
		else if (typestr == "sphere") {
			type_ = GT_Sphere;
			openravejson::LoadJsonValueByKey(value, "radius", gemo_outer_extents_data_.x);

			gemo_outer_extents_data_ *= unit_scale;
		}
		else if (typestr == "cylinder") {
			type_ = GT_Cylinder;
			openravejson::LoadJsonValueByKey(value, "radius", gemo_outer_extents_data_.x);
			openravejson::LoadJsonValueByKey(value, "height", gemo_outer_extents_data_.y);

			gemo_outer_extents_data_.x *= unit_scale;
			gemo_outer_extents_data_.y *= unit_scale;
		}
		else if (typestr == "trimesh" || typestr == "mesh") {
			type_ = GT_TriMesh;
			openravejson::LoadJsonValueByKey(value, "mesh", mesh_collision_);

			for (auto& itvertex : mesh_collision_.vertices) {
				itvertex *= unit_scale;
			}
		}
		else {
			throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported geometry type \"%s\"", typestr, ORE_InvalidArguments);
		}

		openravejson::LoadJsonValueByKey(value, "transparency", transparency_);
		openravejson::LoadJsonValueByKey(value, "visible", is_visible_);
		openravejson::LoadJsonValueByKey(value, "diffuseColor", diffuse_color_vec_);
		openravejson::LoadJsonValueByKey(value, "ambientColor", ambient_color_vec_);
		openravejson::LoadJsonValueByKey(value, "modifiable", is_modifiable_);
	}

	AABB KinBody::GeometryInfo::ComputeAABB(const Transform& geometry_world) const
	{
		AABB ab;
		TransformMatrix tglobal = geometry_world * transform_;

		switch (type_) 
		{
		case GT_None:
			ab.extents.x = 0;
			ab.extents.y = 0;
			ab.extents.z = 0;
			break;
		case GT_Box: // origin of box is at the center
			ab.extents.x = RaveFabs(tglobal.m[0])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[1])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[2])*gemo_outer_extents_data_.z;
			ab.extents.y = RaveFabs(tglobal.m[4])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[5])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[6])*gemo_outer_extents_data_.z;
			ab.extents.z = RaveFabs(tglobal.m[8])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[9])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[10])*gemo_outer_extents_data_.z;
			ab.pos = tglobal.trans;
			break;
		case GT_Container: // origin of container is at the bottom
			ab.extents.x = 0.5*(RaveFabs(tglobal.m[0])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[1])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[2])*gemo_outer_extents_data_.z);
			ab.extents.y = 0.5*(RaveFabs(tglobal.m[4])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[5])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[6])*gemo_outer_extents_data_.z);
			ab.extents.z = 0.5*(RaveFabs(tglobal.m[8])*gemo_outer_extents_data_.x 
				+ RaveFabs(tglobal.m[9])*gemo_outer_extents_data_.y 
				+ RaveFabs(tglobal.m[10])*gemo_outer_extents_data_.z);
			ab.pos = tglobal.trans + Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10])*(0.5*gemo_outer_extents_data_.z);

			if (geom_bottom_data_.x > 0 && geom_bottom_data_.y > 0 && geom_bottom_data_.z > 0)
			{
				// Container with bottom
				Vector vcontainerdir = Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10]);
				ab.pos += vcontainerdir * geom_bottom_data_.z; // take into account the bottom of the container

				Vector vbottompos = tglobal.trans + vcontainerdir * (0.5*geom_bottom_data_.z);
				Vector vbottomextents;
				vbottomextents.x = 0.5*(RaveFabs(tglobal.m[0])*geom_bottom_data_.x 
					+ RaveFabs(tglobal.m[1])*geom_bottom_data_.y 
					+ RaveFabs(tglobal.m[2])*geom_bottom_data_.z);
				vbottomextents.y = 0.5*(RaveFabs(tglobal.m[4])*geom_bottom_data_.x 
					+ RaveFabs(tglobal.m[5])*geom_bottom_data_.y 
					+ RaveFabs(tglobal.m[6])*geom_bottom_data_.z);
				vbottomextents.z = 0.5*(RaveFabs(tglobal.m[8])*geom_bottom_data_.x 
					+ RaveFabs(tglobal.m[9])*geom_bottom_data_.y 
					+ RaveFabs(tglobal.m[10])*geom_bottom_data_.z);
				Vector vmin = ab.pos - ab.extents;
				Vector vmax = ab.pos + ab.extents;
				Vector vbottommin = vbottompos - vbottomextents;
				Vector vbottommax = vbottompos + vbottomextents;
				if (vmin.x > vbottommin.x) {
					vmin.x = vbottommin.x;
				}
				if (vmin.y > vbottommin.y) {
					vmin.y = vbottommin.y;
				}
				if (vmin.z > vbottommin.z) {
					vmin.z = vbottommin.z;
				}
				if (vmax.x < vbottommax.x) {
					vmax.x = vbottommax.x;
				}
				if (vmax.y < vbottommax.y) {
					vmax.y = vbottommax.y;
				}
				if (vmax.z < vbottommax.z) {
					vmax.z = vbottommax.z;
				}
				ab.pos = 0.5 * (vmin + vmax);
				ab.extents = vmax - ab.pos;
			}
			break;
		case GT_Sphere:
			ab.extents.x = ab.extents.y = ab.extents.z = gemo_outer_extents_data_[0];
			ab.pos = tglobal.trans;
			break;
		case GT_Cylinder:
			ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*gemo_outer_extents_data_.y
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[2] * tglobal.m[2]))*gemo_outer_extents_data_.x;
			ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*gemo_outer_extents_data_.y
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[6] * tglobal.m[6]))*gemo_outer_extents_data_.x;
			ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*gemo_outer_extents_data_.y
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[10] * tglobal.m[10]))*gemo_outer_extents_data_.x;
			ab.pos = tglobal.trans; //+(dReal)0.5*gemo_outer_extents_data_.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
			break;
		case GT_Cage: {
			// have to return the entire volume, even the inner region since a lot of code use the bounding box to compute cropping and other functions
			const Vector& vCageBaseExtents = gemo_outer_extents_data_;
			const Vector& vCageForceInnerFull = geom_inner_extents_data_;

			Vector vmin, vmax;
			vmin.x = -vCageBaseExtents.x;
			vmin.y = -vCageBaseExtents.y;
			vmax.x = vCageBaseExtents.x;
			vmax.y = vCageBaseExtents.y;
			vmax.z = vCageBaseExtents.z * 2;
			for (size_t i = 0; i < side_walls_vector_.size(); ++i) {
				const GeometryInfo::SideWall &s = side_walls_vector_[i];
				TransformMatrix sidewallmat = s.transf;
				Vector vselocal = s.vExtents;
				Vector vsegeom;
				vsegeom.x = RaveFabs(sidewallmat.m[0])*vselocal.x 
					+ RaveFabs(sidewallmat.m[1])*vselocal.y 
					+ RaveFabs(sidewallmat.m[2])*vselocal.z;
				vsegeom.y = RaveFabs(sidewallmat.m[4])*vselocal.x 
					+ RaveFabs(sidewallmat.m[5])*vselocal.y 
					+ RaveFabs(sidewallmat.m[6])*vselocal.z;
				vsegeom.z = RaveFabs(sidewallmat.m[8])*vselocal.x 
					+ RaveFabs(sidewallmat.m[9])*vselocal.y 
					+ RaveFabs(sidewallmat.m[10])*vselocal.z;

				Vector vcenterpos = s.transf.trans + Vector(sidewallmat.m[2], sidewallmat.m[6], sidewallmat.m[10])*(vselocal.z);
				Vector vsidemin = vcenterpos - vsegeom;
				Vector vsidemax = vcenterpos + vsegeom;

				if (vmin.x > vsidemin.x) {
					vmin.x = vsidemin.x;
				}
				if (vmin.y > vsidemin.y) {
					vmin.y = vsidemin.y;
				}
				if (vmin.z > vsidemin.z) {
					vmin.z = vsidemin.z;
				}
				if (vmax.x < vsidemax.x) {
					vmax.x = vsidemax.x;
				}
				if (vmax.y < vsidemax.y) {
					vmax.y = vsidemax.y;
				}
				if (vmax.z < vsidemax.z) {
					vmax.z = vsidemax.z;
				}
			}

			if (vCageForceInnerFull.x > 0) {
				if (vmin.x > -0.5*vCageForceInnerFull.x) {
					vmin.x = -0.5*vCageForceInnerFull.x;
				}
				if (vmax.x < 0.5*vCageForceInnerFull.x) {
					vmax.x = 0.5*vCageForceInnerFull.x;
				}
			}
			if (vCageForceInnerFull.y > 0) {
				if (vmin.y > -0.5*vCageForceInnerFull.y) {
					vmin.y = -0.5*vCageForceInnerFull.y;
				}
				if (vmax.y < 0.5*vCageForceInnerFull.y) {
					vmax.y = 0.5*vCageForceInnerFull.y;
				}
			}
			if (vCageForceInnerFull.z > 0) {
				if (vmax.z < vCageBaseExtents.z * 2 + vCageForceInnerFull.z) {
					vmax.z = vCageBaseExtents.z * 2 + vCageForceInnerFull.z;
				}
			}

			// now that vmin and vmax are in geom space, transform them
			Vector vgeomextents = 0.5*(vmax - vmin);

			ab.extents.x = RaveFabs(tglobal.m[0])*vgeomextents.x + RaveFabs(tglobal.m[1])*vgeomextents.y + RaveFabs(tglobal.m[2])*vgeomextents.z;
			ab.extents.y = RaveFabs(tglobal.m[4])*vgeomextents.x + RaveFabs(tglobal.m[5])*vgeomextents.y + RaveFabs(tglobal.m[6])*vgeomextents.z;
			ab.extents.z = RaveFabs(tglobal.m[8])*vgeomextents.x + RaveFabs(tglobal.m[9])*vgeomextents.y + RaveFabs(tglobal.m[10])*vgeomextents.z;
			ab.pos = tglobal * (0.5*(vmin + vmax));
			break;
		}
		case GT_TriMesh: {
			// Cage: init collision mesh?
			// just use mesh_collision_
			if (mesh_collision_.vertices.size() > 0) {
				Vector vmin, vmax; vmin = vmax = tglobal * mesh_collision_.vertices.at(0);
				for (auto itv : mesh_collision_.vertices) {
					Vector v = tglobal * itv;
					if (vmin.x > v.x) {
						vmin.x = v.x;
					}
					if (vmin.y > v.y) {
						vmin.y = v.y;
					}
					if (vmin.z > v.z) {
						vmin.z = v.z;
					}
					if (vmax.x < v.x) {
						vmax.x = v.x;
					}
					if (vmax.y < v.y) {
						vmax.y = v.y;
					}
					if (vmax.z < v.z) {
						vmax.z = v.z;
					}
				}
				ab.extents = (dReal)0.5*(vmax - vmin);
				ab.pos = (dReal)0.5*(vmax + vmin);
			}
			else {
				ab.pos = tglobal.trans;
			}
			break;
		}
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(("unknown geometry type %d"), type_, ORE_InvalidArguments);
		}

		return ab;
	}

	KinBody::Link::Geometry::Geometry(KinBody::LinkPtr parent, const KinBody::GeometryInfo& info)
		: parent_(parent), info_(info)
	{
	}

	bool KinBody::Link::Geometry::InitCollisionMesh(float fTessellation)
	{
		return info_.InitCollisionMesh(fTessellation);
	}

	bool KinBody::Link::Geometry::ComputeInnerEmptyVolume(Transform& inner_empty_volume, Vector& inner_empty_extents) const
	{
		return info_.ComputeInnerEmptyVolume(inner_empty_volume, inner_empty_extents);
	}

	AABB KinBody::Link::Geometry::ComputeAABB(const Transform& t) const
	{
		return info_.ComputeAABB(t);
	}

	void KinBody::Link::Geometry::serialize(std::ostream& o, int options) const
	{
		SerializeRound(o, info_.transform_);
		o << info_.type_ << " ";
		SerializeRound3(o, info_.render_scale_vec_);
		if (info_.type_ == GT_TriMesh) {
			info_.mesh_collision_.serialize(o, options);
		}
		else {
			SerializeRound3(o, info_.gemo_outer_extents_data_);
			if (info_.type_ == GT_Cage) {
				SerializeRound3(o, info_.geom_inner_extents_data_);
				for (size_t iwall = 0; iwall < info_.side_walls_vector_.size(); ++iwall) {
					const GeometryInfo::SideWall &s = info_.side_walls_vector_[iwall];
					SerializeRound(o, s.transf);
					SerializeRound3(o, s.vExtents);
					o << (uint32_t)s.type;
				}
			}
			else if (info_.type_ == GT_Container) {
				SerializeRound3(o, info_.geom_inner_extents_data_);
				SerializeRound3(o, info_.geom_bottom_cross_data_);
				SerializeRound3(o, info_.geom_bottom_data_);
			}
		}
	}

	void KinBody::Link::Geometry::SetCollisionMesh(const TriMesh& mesh)
	{
		OPENRAVE_ASSERT_FORMAT0(info_.is_modifiable_, "geometry cannot be modified", ORE_Failed);
		LinkPtr parent(parent_);
		info_.mesh_collision_ = mesh;
		parent->_Update();
	}

	bool KinBody::Link::Geometry::SetVisible(bool visible)
	{
		if (info_.is_visible_ != visible) {
			info_.is_visible_ = visible;
			LinkPtr parent(parent_);
			parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
			return true;
		}
		return false;
	}

	void KinBody::Link::Geometry::SetTransparency(float f)
	{
		LinkPtr parent(parent_);
		info_.transparency_ = f;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
	}

	void KinBody::Link::Geometry::SetDiffuseColor(const RaveVector<float>& color)
	{
		LinkPtr parent(parent_);
		info_.diffuse_color_vec_ = color;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
	}

	void KinBody::Link::Geometry::SetAmbientColor(const RaveVector<float>& color)
	{
		LinkPtr parent(parent_);
		info_.ambient_color_vec_ = color;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
	}

	/*
	 * Ray-box intersection using IEEE numerical properties to ensure that the
	 * test is both robust and efficient, as described in:
	 *
	 *      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
	 *      "An Efficient and Robust Ray-Box Intersection Algorithm"
	 *      Journal of graphics tools, 10(1):49-54, 2005
	 *
	 */
	 //static bool RayAABBIntersect(const Ray &r, float t0, float t1) const
	 //{
	 //    dReal tmin, tmax, tymin, tymax, tzmin, tzmax;
	 //    tmin = (parameters[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
	 //    tmax = (parameters[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
	 //    tymin = (parameters[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
	 //    tymax = (parameters[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
	 //    if ( (tmin > tymax) || (tymin > tmax) )
	 //        return false;
	 //    if (tymin > tmin)
	 //        tmin = tymin;
	 //    if (tymax < tmax)
	 //        tmax = tymax;
	 //    tzmin = (parameters[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
	 //    tzmax = (parameters[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
	 //    if ( (tmin > tzmax) || (tzmin > tmax) )
	 //        return false;
	 //    if (tzmin > tmin)
	 //        tmin = tzmin;
	 //    if (tzmax < tmax)
	 //        tmax = tzmax;
	 //    return ( (tmin < t1) && (tmax > t0) );
	 //}

	bool KinBody::Link::Geometry::ValidateContactNormal(const Vector& _position, Vector& _normal) const
	{
		Transform tinv = info_.transform_.inverse();
		Vector position = tinv * _position;
		Vector normal = tinv.rotate(_normal);
		const dReal feps = 0.00005f;
		switch (info_.type_) {
		case GT_Box: {
			// transform position in +x+y+z octant
			Vector tposition = position, tnormal = normal;
			if (tposition.x < 0) {
				tposition.x = -tposition.x;
				tnormal.x = -tnormal.x;
			}
			if (tposition.y < 0) {
				tposition.y = -tposition.y;
				tnormal.y = -tnormal.y;
			}
			if (tposition.z < 0) {
				tposition.z = -tposition.z;
				tnormal.z = -tnormal.z;
			}
			// find the normal to the surface depending on the region the position is in
			dReal xaxis = -info_.gemo_outer_extents_data_.z*tposition.y + info_.gemo_outer_extents_data_.y*tposition.z;
			dReal yaxis = -info_.gemo_outer_extents_data_.x*tposition.z + info_.gemo_outer_extents_data_.z*tposition.x;
			dReal zaxis = -info_.gemo_outer_extents_data_.y*tposition.x + info_.gemo_outer_extents_data_.x*tposition.y;
			dReal penetration = 0;
			if ((zaxis < feps) && (yaxis > -feps)) { // x-plane
				if (RaveFabs(tnormal.x) > RaveFabs(penetration)) {
					penetration = tnormal.x;
				}
			}
			if ((zaxis > -feps) && (xaxis < feps)) { // y-plane
				if (RaveFabs(tnormal.y) > RaveFabs(penetration)) {
					penetration = tnormal.y;
				}
			}
			if ((yaxis < feps) && (xaxis > -feps)) { // z-plane
				if (RaveFabs(tnormal.z) > RaveFabs(penetration)) {
					penetration = tnormal.z;
				}
			}
			if (penetration < -feps) {
				_normal = -_normal;
				return true;
			}
			break;
		}
		case GT_Cylinder: { // z-axis
			dReal fInsideCircle = position.x*position.x + position.y*position.y - info_.gemo_outer_extents_data_.x*info_.gemo_outer_extents_data_.x;
			dReal fInsideHeight = 2.0f*RaveFabs(position.z) - info_.gemo_outer_extents_data_.y;
			if ((fInsideCircle < -feps) && (fInsideHeight > -feps) && (normal.z*position.z < 0)) {
				_normal = -_normal;
				return true;
			}
			if ((fInsideCircle > -feps) && (fInsideHeight < -feps) && (normal.x*position.x + normal.y*position.y < 0)) {
				_normal = -_normal;
				return true;
			}
			break;
		}
		case GT_Sphere:
			if (normal.dot3(position) < 0) {
				_normal = -_normal;
				return true;
			}
			break;
		case GT_None:
		default:
			break;
		}
		return false;
	}

	void KinBody::Link::Geometry::SetRenderFilename(const std::string& renderfilename)
	{
		LinkPtr parent(parent_);
		info_.render_file_name_ = renderfilename;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
	}

	void KinBody::Link::Geometry::SetName(const std::string& name)
	{
		LinkPtr parent(parent_);
		info_.name_ = name;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
	}

	uint8_t KinBody::Link::Geometry::GetSideWallExists() const
	{
		uint8_t mask = 0;
		for (size_t i = 0; i < info_.side_walls_vector_.size(); ++i) {
			mask |= 1 << info_.side_walls_vector_[i].type;
		}
		return mask;
	}
}
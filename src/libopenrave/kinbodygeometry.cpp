﻿// -*- coding: utf-8 -*-
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
		_vDiffuseColor = Vector(1, 1, 1);
		_type = GT_None;
		_fTransparency = 0;
		_vRenderScale = _vCollisionScale = Vector(1, 1, 1);
		_bVisible = true;
		_bModifiable = true;
	}

	bool KinBody::GeometryInfo::InitCollisionMesh(float fTessellation)
	{
		if (_type == GT_TriMesh || _type == GT_None) {
			return true;
		}

		// is clear() better since it releases the memory?
		_meshcollision.indices.resize(0);
		_meshcollision.vertices.resize(0);

		if (fTessellation < 0.01f) {
			fTessellation = 0.01f;
		}
		// start tesselating
		switch (_type) {
		case GT_Sphere: {
			// log_2 (1+ tess)
			GenerateSphereTriangulation(_meshcollision, 3 + (int)(logf(fTessellation) / logf(2.0f)));
			dReal fRadius = GetSphereRadius();
			for(auto& it: _meshcollision.vertices) 
			{
				it *= fRadius;
			}
			break;
		}
		case GT_Box: {
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
			_meshcollision.vertices.resize(8);
			std::copy(&v[0], &v[8], _meshcollision.vertices.begin());
			_meshcollision.indices.resize(nindices);
			std::copy(&indices[0], &indices[nindices], _meshcollision.indices.begin());
			break;
		}
		case GT_Cylinder: {
			// cylinder is on z axis
			dReal rad = GetCylinderRadius(), len = GetCylinderHeight()*0.5f;
			int numverts = (int)(fTessellation*48.0f) + 3;
			dReal dtheta = 2 * PI / (dReal)numverts;
			_meshcollision.vertices.push_back(Vector(0, 0, len));
			_meshcollision.vertices.push_back(Vector(0, 0, -len));
			_meshcollision.vertices.push_back(Vector(rad, 0, len));
			_meshcollision.vertices.push_back(Vector(rad, 0, -len));
			for (int i = 0; i < numverts + 1; ++i) {
				dReal s = rad * RaveSin(dtheta * (dReal)i);
				dReal c = rad * RaveCos(dtheta * (dReal)i);
				int off = (int)_meshcollision.vertices.size();
				_meshcollision.vertices.push_back(Vector(c, s, len));
				_meshcollision.vertices.push_back(Vector(c, s, -len));

				_meshcollision.indices.push_back(0);       _meshcollision.indices.push_back(off - 2);       _meshcollision.indices.push_back(off);
				_meshcollision.indices.push_back(1);       _meshcollision.indices.push_back(off + 1);       _meshcollision.indices.push_back(off - 1);
				_meshcollision.indices.push_back(off - 2);   _meshcollision.indices.push_back(off - 1);         _meshcollision.indices.push_back(off);
				_meshcollision.indices.push_back(off);   _meshcollision.indices.push_back(off - 1);         _meshcollision.indices.push_back(off + 1);
			}
			break;
		}
		case GT_Cage: {
			const Vector& vCageBaseExtents = _vGeomData;
			for (size_t i = 0; i < _vSideWalls.size(); ++i) {
				const SideWall &s = _vSideWalls[i];
				const size_t vBase = _meshcollision.vertices.size();
				AppendBoxTriangulation(Vector(0, 0, s.vExtents[2]), s.vExtents, _meshcollision);

				for (size_t j = 0; j < 8; ++j) {
					_meshcollision.vertices[vBase + j] = s.transf * _meshcollision.vertices[vBase + j];
				}
			}
			// finally add the base
			AppendBoxTriangulation(Vector(0, 0, vCageBaseExtents.z), vCageBaseExtents, _meshcollision);
			break;
		}
		case GT_Container: {
			const Vector& outerextents = _vGeomData;
			const Vector& innerextents = _vGeomData2;
			const Vector& bottomcross = _vGeomData3;
			const Vector& bottom = _vGeomData4;
			dReal zoffset = 0;
			if (bottom[2] > 0) {
				if (bottom[0] > 0 && bottom[1] > 0) {
					zoffset = bottom[2];
				}
			}
			// +x wall
			AppendBoxTriangulation(Vector((outerextents[0] + innerextents[0]) / 4., 0, outerextents[2] / 2. + zoffset), Vector((outerextents[0] - innerextents[0]) / 4., outerextents[1] / 2., outerextents[2] / 2.), _meshcollision);
			// -x wall
			AppendBoxTriangulation(Vector(-(outerextents[0] + innerextents[0]) / 4., 0, outerextents[2] / 2. + zoffset), Vector((outerextents[0] - innerextents[0]) / 4., outerextents[1] / 2., outerextents[2] / 2.), _meshcollision);
			// +y wall
			AppendBoxTriangulation(Vector(0, (outerextents[1] + innerextents[1]) / 4., outerextents[2] / 2. + zoffset), Vector(outerextents[0] / 2., (outerextents[1] - innerextents[1]) / 4., outerextents[2] / 2.), _meshcollision);
			// -y wall
			AppendBoxTriangulation(Vector(0, -(outerextents[1] + innerextents[1]) / 4., outerextents[2] / 2. + zoffset), Vector(outerextents[0] / 2., (outerextents[1] - innerextents[1]) / 4., outerextents[2] / 2.), _meshcollision);
			// bottom
			if (outerextents[2] - innerextents[2] >= 1e-6) { // small epsilon error can make thin triangles appear, so test with a reasonable threshold
				AppendBoxTriangulation(Vector(0, 0, (outerextents[2] - innerextents[2]) / 2. + zoffset), Vector(outerextents[0] / 2., outerextents[1] / 2., (outerextents[2] - innerextents[2]) / 2), _meshcollision);
			}
			// cross
			if (bottomcross[2] > 0) {
				if (bottomcross[0] > 0) {
					AppendBoxTriangulation(Vector(0, 0, bottomcross[2] / 2 + outerextents[2] - innerextents[2] + zoffset), Vector(bottomcross[0] / 2, innerextents[1] / 2, bottomcross[2] / 2), _meshcollision);
				}
				if (bottomcross[1] > 0) {
					AppendBoxTriangulation(Vector(0, 0, bottomcross[2] / 2 + outerextents[2] - innerextents[2] + zoffset), Vector(innerextents[0] / 2, bottomcross[1] / 2, bottomcross[2] / 2), _meshcollision);
				}
			}
			// bottom
			if (bottom[2] > 0) {
				if (bottom[0] > 0 && bottom[1] > 0) {
					AppendBoxTriangulation(Vector(0, 0, bottom[2] / 2), Vector(bottom[0] / 2., bottom[1] / 2., bottom[2] / 2.), _meshcollision);
				}
			}
			break;
		}
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(("unrecognized geom type %d!"), _type, ORE_InvalidArguments);
		}

		return true;
	}

	bool KinBody::GeometryInfo::ComputeInnerEmptyVolume(
		Transform& tInnerEmptyVolume,
		Vector& abInnerEmptyExtents) const
	{
		switch (_type) {
		case GT_Cage: {
			Vector vwallmin, vwallmax;
			vwallmax.z = vwallmin.z = _vGeomData.z * 2;

			// initialize to the base extents if there is no wall
			vwallmin.x = -_vGeomData.x;
			vwallmin.y = -_vGeomData.y;
			vwallmax.x = _vGeomData.x;
			vwallmax.y = _vGeomData.y;
			int sideWallExtents = 0;

			for(auto& itwall: _vSideWalls) {
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

			// if _vGeomData2 is greater than 0, force inner region wherever possible.
			// The only thing that will prevent _vGeomData2's inner region is a wall present.
			// Should not use base to restrict _vGeomData2
			if (_vGeomData2.x > 0) {
				if (sideWallExtents & (1 << GeometryInfo::SWT_NX)) {
					if (vwallmin.x < -0.5*_vGeomData2.x) {
						vwallmin.x = -0.5*_vGeomData2.x;
					}
				}
				else {
					// no wall defined on NX
					vwallmin.x = -0.5*_vGeomData2.x;
				}

				if (sideWallExtents & (1 << GeometryInfo::SWT_PX)) {
					if (vwallmax.x > 0.5*_vGeomData2.x) {
						vwallmax.x = 0.5*_vGeomData2.x;
					}
				}
				else {
					// no wall defined on NX
					vwallmax.x = 0.5*_vGeomData2.x;
				}
			}

			if (_vGeomData2.y > 0) {
				if (sideWallExtents & (1 << GeometryInfo::SWT_NY)) {
					if (vwallmin.y < -0.5*_vGeomData2.y) {
						vwallmin.y = -0.5*_vGeomData2.y;
					}
				}
				else {
					vwallmin.y = -0.5*_vGeomData2.y;
				}

				if (sideWallExtents & (1 << GeometryInfo::SWT_PY)) {
					if (vwallmax.y > 0.5*_vGeomData2.y) {
						vwallmax.y = 0.5*_vGeomData2.y;
					}
				}
				else {
					vwallmax.y = 0.5*_vGeomData2.y;
				}
			}

			// the top has no constraints, so use the max of walls and force inner region
			if (vwallmax.z < _vGeomData.z * 2 + _vGeomData2.z) {
				vwallmax.z = _vGeomData.z * 2 + _vGeomData2.z;
			}

			abInnerEmptyExtents = 0.5*(vwallmax - vwallmin);
			tInnerEmptyVolume = _t;
			tInnerEmptyVolume.trans += tInnerEmptyVolume.rotate(0.5*(vwallmax + vwallmin));
			return true;
		}
		case GT_Container: {
			Transform tempty;
			// full outer extents - full inner extents + inner extents = _vGeomData.z - 0.5*_vGeomData2.z
			tempty.trans.z = _vGeomData.z - 0.5 * _vGeomData2.z;
			if (_vGeomData4.x > 0 && _vGeomData4.y > 0 && _vGeomData4.z > 0) {
				// if _vGeomData4 is valid, need to shift the empty region up.
				tempty.trans.z += _vGeomData4.z;
			}
			tInnerEmptyVolume = _t * tempty;
			abInnerEmptyExtents = 0.5*_vGeomData2;
			return true;
		}
		default:
			return false;
		}
	}

inline void SaveJsonValue(rapidjson::Value& v, const KinBody::GeometryInfo::SideWall& t,
	rapidjson::Document::AllocatorType& alloc) {
    v.SetObject();
    openravejson::SetJsonValueByKey(v, "transform", t.transf, alloc);
    openravejson::SetJsonValueByKey(v, "halfExtents", t.vExtents, alloc);
    openravejson::SetJsonValueByKey(v, "type", (int)t.type, alloc);
}

inline void LoadJsonValue(const rapidjson::Value& v, KinBody::GeometryInfo::SideWall& t) {
    if(v.IsObject()) {
        openravejson::LoadJsonValueByKey(v, "transform", t.transf);
        openravejson::LoadJsonValueByKey(v, "halfExtents", t.vExtents);
        int type = 0;
        openravejson::LoadJsonValueByKey(v, "type", type);
        t.type = (KinBody::GeometryInfo::SideWallType)type;
    } else {
        throw openravejson::OpenRAVEJSONException("Cannot convert json type " + openravejson::GetJsonTypeName(v) + " to OpenRAVE::Geometry::SideWall");
    }
}

void KinBody::GeometryInfo::SerializeJSON(rapidjson::Value& value, 
	rapidjson::Document::AllocatorType& allocator, const dReal fUnitScale, int options) const
{
    // RAVE_SERIALIZEJSON_ADDMEMBER(allocator, "sid", sid);
    openravejson::SetJsonValueByKey(value, "name", _name, allocator);

    Transform tscaled = _t;
    tscaled.trans *= fUnitScale;
    openravejson::SetJsonValueByKey(value, "transform", tscaled, allocator);

    switch(_type) {
    case GT_Box:
        openravejson::SetJsonValueByKey(value, "type", "box", allocator);
        openravejson::SetJsonValueByKey(value, "halfExtents", _vGeomData*fUnitScale, allocator);
        break;

    case GT_Container:
        openravejson::SetJsonValueByKey(value, "type", "container", allocator);
        openravejson::SetJsonValueByKey(value, "outerExtents", _vGeomData*fUnitScale, allocator);
        openravejson::SetJsonValueByKey(value, "innerExtents", _vGeomData2*fUnitScale, allocator);
        openravejson::SetJsonValueByKey(value, "bottomCross", _vGeomData3*fUnitScale, allocator);
        openravejson::SetJsonValueByKey(value, "bottom", _vGeomData4*fUnitScale, allocator);
        break;

    case GT_Cage: {
        openravejson::SetJsonValueByKey(value, "type", "cage", allocator);
        openravejson::SetJsonValueByKey(value, "baseExtents", _vGeomData*fUnitScale, allocator);

        std::vector<SideWall> vScaledSideWalls = _vSideWalls;
        FOREACH(itwall, vScaledSideWalls) {
            itwall->transf.trans *= fUnitScale;
            itwall->vExtents *= fUnitScale;
        }
        if( _vGeomData2.x > g_fEpsilon ) {
            openravejson::SetJsonValueByKey(value, "innerSizeX", _vGeomData2.x*fUnitScale, allocator);
        }
        if( _vGeomData2.y > g_fEpsilon ) {
            openravejson::SetJsonValueByKey(value, "innerSizeY", _vGeomData2.y*fUnitScale, allocator);
        }
        if( _vGeomData2.z > g_fEpsilon ) {
            openravejson::SetJsonValueByKey(value, "innerSizeZ", _vGeomData2.z*fUnitScale, allocator);
        }
        openravejson::SetJsonValueByKey(value, "sideWalls", vScaledSideWalls, allocator);
        break;
    }
    case GT_Sphere:
        openravejson::SetJsonValueByKey(value, "type", "sphere", allocator);
        openravejson::SetJsonValueByKey(value, "radius", _vGeomData.x*fUnitScale, allocator);
        break;


    case GT_Cylinder:
        openravejson::SetJsonValueByKey(value, "type", "cylinder", allocator);
        openravejson::SetJsonValueByKey(value, "radius", _vGeomData.x*fUnitScale, allocator);
        openravejson::SetJsonValueByKey(value, "height", _vGeomData.y*fUnitScale, allocator);
        break;

    case GT_TriMesh:
        openravejson::SetJsonValueByKey(value, "type", "trimesh", allocator);
        openravejson::SetJsonValueByKey(value, "mesh", _meshcollision, allocator);
			break;

		default:
			break;
		}

    openravejson::SetJsonValueByKey(value, "transparency", _fTransparency, allocator);
    openravejson::SetJsonValueByKey(value, "visible", _bVisible, allocator);
    openravejson::SetJsonValueByKey(value, "diffuseColor", _vDiffuseColor, allocator);
    openravejson::SetJsonValueByKey(value, "ambientColor", _vAmbientColor, allocator);
    openravejson::SetJsonValueByKey(value, "modifiable", _bModifiable, allocator);
	}


	void KinBody::GeometryInfo::DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale)
	{
    openravejson::LoadJsonValueByKey(value, "name", _name);
    openravejson::LoadJsonValueByKey(value, "transform", _t);

		_t.trans *= fUnitScale;

		std::string typestr;
    openravejson::LoadJsonValueByKey(value, "type", typestr);

		if (typestr == "box") {
			_type = GT_Box;
        openravejson::LoadJsonValueByKey(value, "halfExtents", _vGeomData);
			_vGeomData *= fUnitScale;
		}
		else if (typestr == "container") {
			_type = GT_Container;
        openravejson::LoadJsonValueByKey(value, "outerExtents", _vGeomData);
        openravejson::LoadJsonValueByKey(value, "innerExtents", _vGeomData2);

			_vGeomData3 = Vector();
        openravejson::LoadJsonValueByKey(value, "bottomCross", _vGeomData3);

			_vGeomData4 = Vector();
        openravejson::LoadJsonValueByKey(value, "bottom", _vGeomData4);

			_vGeomData *= fUnitScale;
			_vGeomData2 *= fUnitScale;
			_vGeomData3 *= fUnitScale;
			_vGeomData4 *= fUnitScale;
		}
		else if (typestr == "cage") {
			_type = GT_Cage;
        openravejson::LoadJsonValueByKey(value, "baseExtents", _vGeomData);
			_vGeomData *= fUnitScale;

			_vGeomData2 = Vector();
        openravejson::LoadJsonValueByKey(value, "innerSizeX", _vGeomData2.x);
        openravejson::LoadJsonValueByKey(value, "innerSizeY", _vGeomData2.y);
        openravejson::LoadJsonValueByKey(value, "innerSizeZ", _vGeomData2.z);
			_vGeomData2 *= fUnitScale;

        openravejson::LoadJsonValueByKey(value, "sideWalls", _vSideWalls);
			for(auto& itsidewall: _vSideWalls) {
				itsidewall.transf.trans *= fUnitScale;
				itsidewall.vExtents *= fUnitScale;
			}
		}
		else if (typestr == "sphere") {
			_type = GT_Sphere;
        openravejson::LoadJsonValueByKey(value, "radius", _vGeomData.x);

			_vGeomData *= fUnitScale;
		}
		else if (typestr == "cylinder") {
			_type = GT_Cylinder;
        openravejson::LoadJsonValueByKey(value, "radius", _vGeomData.x);
        openravejson::LoadJsonValueByKey(value, "height", _vGeomData.y);

			_vGeomData.x *= fUnitScale;
			_vGeomData.y *= fUnitScale;

		}
		else if (typestr == "trimesh" || typestr == "mesh") {
			_type = GT_TriMesh;
        openravejson::LoadJsonValueByKey(value, "mesh", _meshcollision);

			for(auto& itvertex: _meshcollision.vertices) {
				itvertex *= fUnitScale;
			}
		}
		else {
			throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported geometry type \"%s\"", typestr, ORE_InvalidArguments);
		}

    openravejson::LoadJsonValueByKey(value, "transparency", _fTransparency);
    openravejson::LoadJsonValueByKey(value, "visible", _bVisible);
    openravejson::LoadJsonValueByKey(value, "diffuseColor", _vDiffuseColor);
    openravejson::LoadJsonValueByKey(value, "ambientColor", _vAmbientColor);
    openravejson::LoadJsonValueByKey(value, "modifiable", _bModifiable);
	}

	AABB KinBody::GeometryInfo::ComputeAABB(const Transform& tGeometryWorld) const
	{
		AABB ab;
		TransformMatrix tglobal = tGeometryWorld * _t;

		switch (_type) {
		case GT_None:
			ab.extents.x = 0;
			ab.extents.y = 0;
			ab.extents.z = 0;
			break;
		case GT_Box: // origin of box is at the center
			ab.extents.x = RaveFabs(tglobal.m[0])*_vGeomData.x + RaveFabs(tglobal.m[1])*_vGeomData.y + RaveFabs(tglobal.m[2])*_vGeomData.z;
			ab.extents.y = RaveFabs(tglobal.m[4])*_vGeomData.x + RaveFabs(tglobal.m[5])*_vGeomData.y + RaveFabs(tglobal.m[6])*_vGeomData.z;
			ab.extents.z = RaveFabs(tglobal.m[8])*_vGeomData.x + RaveFabs(tglobal.m[9])*_vGeomData.y + RaveFabs(tglobal.m[10])*_vGeomData.z;
			ab.pos = tglobal.trans;
			break;
		case GT_Container: // origin of container is at the bottom
			ab.extents.x = 0.5*(RaveFabs(tglobal.m[0])*_vGeomData.x + RaveFabs(tglobal.m[1])*_vGeomData.y + RaveFabs(tglobal.m[2])*_vGeomData.z);
			ab.extents.y = 0.5*(RaveFabs(tglobal.m[4])*_vGeomData.x + RaveFabs(tglobal.m[5])*_vGeomData.y + RaveFabs(tglobal.m[6])*_vGeomData.z);
			ab.extents.z = 0.5*(RaveFabs(tglobal.m[8])*_vGeomData.x + RaveFabs(tglobal.m[9])*_vGeomData.y + RaveFabs(tglobal.m[10])*_vGeomData.z);
			ab.pos = tglobal.trans + Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10])*(0.5*_vGeomData.z);

			if (_vGeomData4.x > 0 && _vGeomData4.y > 0 && _vGeomData4.z > 0) {
				// Container with bottom
				Vector vcontainerdir = Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10]);
				ab.pos += vcontainerdir * _vGeomData4.z; // take into account the bottom of the container

				Vector vbottompos = tglobal.trans + vcontainerdir * (0.5*_vGeomData4.z);
				Vector vbottomextents;
				vbottomextents.x = 0.5*(RaveFabs(tglobal.m[0])*_vGeomData4.x + RaveFabs(tglobal.m[1])*_vGeomData4.y + RaveFabs(tglobal.m[2])*_vGeomData4.z);
				vbottomextents.y = 0.5*(RaveFabs(tglobal.m[4])*_vGeomData4.x + RaveFabs(tglobal.m[5])*_vGeomData4.y + RaveFabs(tglobal.m[6])*_vGeomData4.z);
				vbottomextents.z = 0.5*(RaveFabs(tglobal.m[8])*_vGeomData4.x + RaveFabs(tglobal.m[9])*_vGeomData4.y + RaveFabs(tglobal.m[10])*_vGeomData4.z);
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
			ab.extents.x = ab.extents.y = ab.extents.z = _vGeomData[0];
			ab.pos = tglobal.trans;
			break;
		case GT_Cylinder:
			ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*_vGeomData.y 
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[2] * tglobal.m[2]))*_vGeomData.x;
			ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*_vGeomData.y 
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[6] * tglobal.m[6]))*_vGeomData.x;
			ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*_vGeomData.y 
				+ RaveSqrt(std::max(dReal(0), 1 - tglobal.m[10] * tglobal.m[10]))*_vGeomData.x;
			ab.pos = tglobal.trans; //+(dReal)0.5*_vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
			break;
		case GT_Cage: {
			// have to return the entire volume, even the inner region since a lot of code use the bounding box to compute cropping and other functions
			const Vector& vCageBaseExtents = _vGeomData;
			const Vector& vCageForceInnerFull = _vGeomData2;

			Vector vmin, vmax;
			vmin.x = -vCageBaseExtents.x;
			vmin.y = -vCageBaseExtents.y;
			vmax.x = vCageBaseExtents.x;
			vmax.y = vCageBaseExtents.y;
			vmax.z = vCageBaseExtents.z * 2;
			for (size_t i = 0; i < _vSideWalls.size(); ++i) {
				const GeometryInfo::SideWall &s = _vSideWalls[i];
				TransformMatrix sidewallmat = s.transf;
				Vector vselocal = s.vExtents;
				Vector vsegeom;
				vsegeom.x = RaveFabs(sidewallmat.m[0])*vselocal.x + RaveFabs(sidewallmat.m[1])*vselocal.y + RaveFabs(sidewallmat.m[2])*vselocal.z;
				vsegeom.y = RaveFabs(sidewallmat.m[4])*vselocal.x + RaveFabs(sidewallmat.m[5])*vselocal.y + RaveFabs(sidewallmat.m[6])*vselocal.z;
				vsegeom.z = RaveFabs(sidewallmat.m[8])*vselocal.x + RaveFabs(sidewallmat.m[9])*vselocal.y + RaveFabs(sidewallmat.m[10])*vselocal.z;

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
			// just use _meshcollision
			if (_meshcollision.vertices.size() > 0) {
				Vector vmin, vmax; vmin = vmax = tglobal * _meshcollision.vertices.at(0);
				for(auto itv: _meshcollision.vertices) {
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
			throw OPENRAVE_EXCEPTION_FORMAT(("unknown geometry type %d"), _type, ORE_InvalidArguments);
		}

		return ab;
	}


	KinBody::Link::Geometry::Geometry(KinBody::LinkPtr parent, const KinBody::GeometryInfo& info) : _parent(parent), _info(info)
	{
	}

	bool KinBody::Link::Geometry::InitCollisionMesh(float fTessellation)
	{
		return _info.InitCollisionMesh(fTessellation);
	}

	bool KinBody::Link::Geometry::ComputeInnerEmptyVolume(Transform& tInnerEmptyVolume, Vector& abInnerEmptyExtents) const
	{
		return _info.ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents);
	}

	AABB KinBody::Link::Geometry::ComputeAABB(const Transform& t) const
	{
		return _info.ComputeAABB(t);
	}

	void KinBody::Link::Geometry::serialize(std::ostream& o, int options) const
	{
		SerializeRound(o, _info._t);
		o << _info._type << " ";
		SerializeRound3(o, _info._vRenderScale);
		if (_info._type == GT_TriMesh) {
			_info._meshcollision.serialize(o, options);
		}
		else {
			SerializeRound3(o, _info._vGeomData);
			if (_info._type == GT_Cage) {
				SerializeRound3(o, _info._vGeomData2);
				for (size_t iwall = 0; iwall < _info._vSideWalls.size(); ++iwall) {
					const GeometryInfo::SideWall &s = _info._vSideWalls[iwall];
					SerializeRound(o, s.transf);
					SerializeRound3(o, s.vExtents);
					o << (uint32_t)s.type;
				}
			}
			else if (_info._type == GT_Container) {
				SerializeRound3(o, _info._vGeomData2);
				SerializeRound3(o, _info._vGeomData3);
				SerializeRound3(o, _info._vGeomData4);
			}
		}
	}

	void KinBody::Link::Geometry::SetCollisionMesh(const TriMesh& mesh)
	{
		OPENRAVE_ASSERT_FORMAT0(_info._bModifiable, "geometry cannot be modified", ORE_Failed);
		LinkPtr parent(_parent);
		_info._meshcollision = mesh;
		parent->_Update();
	}

	bool KinBody::Link::Geometry::SetVisible(bool visible)
	{
		if (_info._bVisible != visible) {
			_info._bVisible = visible;
			LinkPtr parent(_parent);
			parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
			return true;
		}
		return false;
	}

	void KinBody::Link::Geometry::SetTransparency(float f)
	{
		LinkPtr parent(_parent);
		_info._fTransparency = f;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
	}

	void KinBody::Link::Geometry::SetDiffuseColor(const RaveVector<float>& color)
	{
		LinkPtr parent(_parent);
		_info._vDiffuseColor = color;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
	}

	void KinBody::Link::Geometry::SetAmbientColor(const RaveVector<float>& color)
	{
		LinkPtr parent(_parent);
		_info._vAmbientColor = color;
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
		Transform tinv = _info._t.inverse();
		Vector position = tinv * _position;
		Vector normal = tinv.rotate(_normal);
		const dReal feps = 0.00005f;
		switch (_info._type) {
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
			dReal xaxis = -_info._vGeomData.z*tposition.y + _info._vGeomData.y*tposition.z;
			dReal yaxis = -_info._vGeomData.x*tposition.z + _info._vGeomData.z*tposition.x;
			dReal zaxis = -_info._vGeomData.y*tposition.x + _info._vGeomData.x*tposition.y;
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
			dReal fInsideCircle = position.x*position.x + position.y*position.y - _info._vGeomData.x*_info._vGeomData.x;
			dReal fInsideHeight = 2.0f*RaveFabs(position.z) - _info._vGeomData.y;
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
		LinkPtr parent(_parent);
		_info._filenamerender = renderfilename;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
	}

	void KinBody::Link::Geometry::SetName(const std::string& name)
	{
		LinkPtr parent(_parent);
		_info._name = name;
		parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);

	}

	uint8_t KinBody::Link::Geometry::GetSideWallExists() const
	{
		uint8_t mask = 0;
		for (size_t i = 0; i < _info._vSideWalls.size(); ++i) {
			mask |= 1 << _info._vSideWalls[i].type;
		}
		return mask;
	}

}

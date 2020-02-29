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
#include "libopenrave.h"
#include <openrave/tri_mesh.h>

namespace OpenRAVE
{
	void TriMesh::ApplyTransform(const Transform& t)
	{
		FOREACH(it, vertices) {
			*it = t * *it;
		}
	}

	void TriMesh::ApplyTransform(const TransformMatrix& t)
	{
		FOREACH(it, vertices) {
			*it = t * *it;
		}
	}

	void TriMesh::Append(const TriMesh& mesh)
	{
		int offset = (int)vertices.size();
		vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
		if (indices.capacity() < indices.size() + mesh.indices.size()) {
			indices.reserve(indices.size() + mesh.indices.size());
		}
		FOREACHC(it, mesh.indices) {
			indices.push_back(*it + offset);
		}
	}

	void TriMesh::Append(const TriMesh& mesh, const Transform& trans)
	{
		int offset = (int)vertices.size();
		vertices.resize(vertices.size() + mesh.vertices.size());
		for (size_t i = 0; i < mesh.vertices.size(); ++i) {
			vertices[i + offset] = trans * mesh.vertices[i];
		}
		if (indices.capacity() < indices.size() + mesh.indices.size()) {
			indices.reserve(indices.size() + mesh.indices.size());
		}
		FOREACHC(it, mesh.indices) {
			indices.push_back(*it + offset);
		}
	}

	AABB TriMesh::ComputeAABB() const
	{
		AABB ab;
		if (vertices.size() == 0) {
			return ab;
		}
		Vector vmin, vmax;
		vmin = vmax = vertices.at(0);
		FOREACHC(itv, vertices) {
			Vector v = *itv;
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
		return ab;
	}

	void TriMesh::serialize(std::ostream& o, int options) const
	{
		o << vertices.size() << " ";
		FOREACHC(it, vertices) {
			SerializeRound3(o, *it);
		}
		o << indices.size() << " ";
		FOREACHC(it, indices) {
			o << *it << " ";
		}
	}

	std::ostream& operator<<(std::ostream& O, const TriMesh& trimesh)
	{
		trimesh.serialize(O, 0);
		return O;
	}

	std::istream& operator>>(std::istream& I, TriMesh& trimesh)
	{
		trimesh.vertices.resize(0);
		trimesh.indices.resize(0);
		size_t s = 0;
		I >> s;
		if (!I) {
			return I;
		}
		trimesh.vertices.resize(s);
		FOREACH(it, trimesh.vertices) {
			I >> it->x >> it->y >> it->z;
		}
		I >> s;
		if (!I) {
			return I;
		}
		trimesh.indices.resize(s);
		FOREACH(it, trimesh.indices) {
			I >> *it;
		}
		return I;
	}

}
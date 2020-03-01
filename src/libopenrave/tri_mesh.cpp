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
#include <openrave/tri_mesh.h>
#include <openrave/numerical.h>
#include <openrave/openrave_macros.h>
#include <map>

namespace OpenRAVE
{
	void TriMesh::ApplyTransform(const Transform& t)
	{
		for(auto& it: vertices)
		{
			it = t * it;
		}
	}

	void TriMesh::ApplyTransform(const TransformMatrix& t)
	{
		for(auto& it: vertices) 
		{
			it = t * it;
		}
	}

	void TriMesh::Append(const TriMesh& mesh)
	{
		int offset = (int)vertices.size();
		vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
		if (indices.capacity() < indices.size() + mesh.indices.size()) 
		{
			indices.reserve(indices.size() + mesh.indices.size());
		}
		for(auto it: mesh.indices)
		{
			indices.push_back(it + offset);
		}
	}

	void TriMesh::Append(const TriMesh& mesh, const Transform& trans)
	{
		int offset = (int)vertices.size();
		vertices.resize(vertices.size() + mesh.vertices.size());
		for (size_t i = 0; i < mesh.vertices.size(); ++i) 
		{
			vertices[i + offset] = trans * mesh.vertices[i];
		}
		if (indices.capacity() < indices.size() + mesh.indices.size())
		{
			indices.reserve(indices.size() + mesh.indices.size());
		}
		for(auto it: mesh.indices)
		{
			indices.push_back(it + offset);
		}
	}

	AABB TriMesh::ComputeAABB() const
	{
		AABB ab;
		if (vertices.size() == 0) 
		{
			return ab;
		}
		Vector vmin, vmax;
		vmin = vmax = vertices.at(0);
		for(auto itv: vertices)
		{
			Vector v = itv;
			if (vmin.x > v.x) 
			{
				vmin.x = v.x;
			}
			if (vmin.y > v.y) 
			{
				vmin.y = v.y;
			}
			if (vmin.z > v.z) 
			{
				vmin.z = v.z;
			}
			if (vmax.x < v.x) 
			{
				vmax.x = v.x;
			}
			if (vmax.y < v.y)
			{
				vmax.y = v.y;
			}
			if (vmax.z < v.z)
			{
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
		for(auto it: vertices) 
		{
			SerializeRound3(o, it);
		}
		o << indices.size() << " ";
		for(auto it: indices) 
		{
			o << it << " ";
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
		if (!I) 
		{
			return I;
		}
		trimesh.vertices.resize(s);
		for(auto it: trimesh.vertices) 
		{
			I >> it.x >> it.y >> it.z;
		}
		I >> s;
		if (!I)
		{
			return I;
		}
		trimesh.indices.resize(s);
		for(auto it: trimesh.indices)
		{
			I >> it;
		}
		return I;
	}


	// generate a sphere triangulation starting with an icosahedron
	// all triangles are oriented counter clockwise
	void GenerateSphereTriangulation(TriMesh& tri, int levels)
	{
		TriMesh temp, temp2;

		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));
		temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
		temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
		temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
		temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
		temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));

		const int nindices = 60;
		int indices[nindices] = {
			0, 1, 2,
			1, 3, 4,
			3, 5, 6,
			2, 4, 7,
			5, 6, 8,
			2, 7, 9,
			0, 5, 8,
			7, 9, 10,
			0, 1, 5,
			7, 10, 11,
			1, 3, 5,
			6, 10, 11,
			3, 6, 11,
			9, 10, 8,
			3, 4, 11,
			6, 8, 10,
			4, 7, 11,
			1, 2, 4,
			0, 8, 9,
			0, 2, 9
		};

		Vector v[3];

		// make sure oriented CCW
		for (int i = 0; i < nindices; i += 3)
		{
			v[0] = temp.vertices[indices[i]];
			v[1] = temp.vertices[indices[i + 1]];
			v[2] = temp.vertices[indices[i + 2]];
			if (v[0].dot3((v[1] - v[0]).cross(v[2] - v[0])) < 0)
			{
				std::swap(indices[i], indices[i + 1]);
			}
		}

		temp.indices.resize(nindices);
		std::copy(&indices[0], &indices[nindices], temp.indices.begin());

		TriMesh* pcur = &temp;
		TriMesh* pnew = &temp2;
		while (levels-- > 0) {

			pnew->vertices.resize(0);
			pnew->vertices.reserve(2 * pcur->vertices.size());
			pnew->vertices.insert(pnew->vertices.end(), pcur->vertices.begin(), pcur->vertices.end());
			pnew->indices.resize(0);
			pnew->indices.reserve(4 * pcur->indices.size());

			std::map< uint64_t, int > mapnewinds;
			std::map< uint64_t, int >::iterator it;

			for (size_t i = 0; i < pcur->indices.size(); i += 3) 
			{
				// for ever tri, create 3 new vertices and 4 new triangles.
				v[0] = pcur->vertices[pcur->indices[i]];
				v[1] = pcur->vertices[pcur->indices[i + 1]];
				v[2] = pcur->vertices[pcur->indices[i + 2]];

				int inds[3];
				for (int j = 0; j < 3; ++j) {
					uint64_t key = ((uint64_t)pcur->indices[i + j] << 32) | (uint64_t)pcur->indices[i + ((j + 1) % 3)];
					it = mapnewinds.find(key);

					if (it == mapnewinds.end()) {
						inds[j] = mapnewinds[key] = mapnewinds[(key << 32) | (key >> 32)] = (int)pnew->vertices.size();
						pnew->vertices.push_back((v[j] + v[(j + 1) % 3]).normalize3());
					}
					else {
						inds[j] = it->second;
					}
				}

				pnew->indices.push_back(pcur->indices[i]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[2]);
				pnew->indices.push_back(inds[0]);    pnew->indices.push_back(pcur->indices[i + 1]);    pnew->indices.push_back(inds[1]);
				pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[1]);
				pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[1]);    pnew->indices.push_back(pcur->indices[i + 2]);
			}

			std::swap(pnew, pcur);
		}

		tri = *pcur;
	}

	/// \param ex half extents
	void AppendBoxTriangulation(const Vector& pos, const Vector& ex, TriMesh& tri)
	{
		// trivial
		Vector v[8] = { Vector(ex.x, ex.y, ex.z) + pos,
						Vector(ex.x, ex.y, -ex.z) + pos,
						Vector(ex.x, -ex.y, ex.z) + pos,
						Vector(ex.x, -ex.y, -ex.z) + pos,
						Vector(-ex.x, ex.y, ex.z) + pos,
						Vector(-ex.x, ex.y, -ex.z) + pos,
						Vector(-ex.x, -ex.y, ex.z) + pos,
						Vector(-ex.x, -ex.y, -ex.z) + pos };
		int vertexoffset = (int)tri.vertices.size();
		const int nindices = 36;
		int indices[nindices] = {
			0 + vertexoffset, 2 + vertexoffset, 1 + vertexoffset,
			1 + vertexoffset, 2 + vertexoffset, 3 + vertexoffset,
			4 + vertexoffset, 5 + vertexoffset, 6 + vertexoffset,
			5 + vertexoffset, 7 + vertexoffset, 6 + vertexoffset,
			0 + vertexoffset, 1 + vertexoffset, 4 + vertexoffset,
			1 + vertexoffset, 5 + vertexoffset, 4 + vertexoffset,
			2 + vertexoffset, 6 + vertexoffset, 3 + vertexoffset,
			3 + vertexoffset, 6 + vertexoffset, 7 + vertexoffset,
			0 + vertexoffset, 4 + vertexoffset, 2 + vertexoffset,
			2 + vertexoffset, 4 + vertexoffset, 6 + vertexoffset,
			1 + vertexoffset, 3 + vertexoffset, 5 + vertexoffset,
			3 + vertexoffset, 7 + vertexoffset, 5 + vertexoffset
		};
		tri.vertices.insert(tri.vertices.end(), &v[0], &v[8]);
		tri.indices.insert(tri.indices.end(), &indices[0], &indices[nindices]);
	}

}
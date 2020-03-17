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
#ifndef OPENRAVE_TRI_MESH_H_
#define OPENRAVE_TRI_MESH_H_

#include <openrave/numerical.h>

namespace OpenRAVE
{

	/// \brief User data for trimesh geometries. Vertices are defined in counter-clockwise order for outward pointing faces.
	class OPENRAVE_API TriMesh
	{
	public:
		std::vector<Vector> vertices;
		std::vector<int32_t> indices;

		void ApplyTransform(const Transform& t);
		void ApplyTransform(const TransformMatrix& t);

		/// append another TriMesh to this tri mesh
		void Append(const TriMesh& mesh);
		void Append(const TriMesh& mesh, const Transform& trans);

		AABB ComputeAABB() const;
		void serialize(std::ostream& o, int options = 0) const;

		friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const TriMesh &trimesh);
		friend OPENRAVE_API std::istream& operator>>(std::istream& I, TriMesh& trimesh);
	};

	OPENRAVE_API void GenerateSphereTriangulation(TriMesh& tri, int levels);
	OPENRAVE_API void AppendBoxTriangulation(const Vector& pos, const Vector& ex, TriMesh& tri);

	OPENRAVE_API std::ostream& operator<<(std::ostream& O, const TriMesh& trimesh);
	OPENRAVE_API std::istream& operator>>(std::istream& I, TriMesh& trimesh);
}


#endif // OPENRAVE_TRI_MESH_H_
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
#ifndef OPENRAVE_IK_PARAMETERIZATION_H_
#define OPENRAVE_IK_PARAMETERIZATION_H_

#include <openrave/config.h>
#include <rapidjson/document.h>
#include <map>
#include <openrave/type.h>
#include <openrave/openrave_exception.h>
#include <openrave/numerical.h>
#include <openrave/configuration_specification.h>


namespace OpenRAVE
{
	/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
	enum IkParameterizationType
	{
		IKP_None = 0,
		IKP_Transform6D = 0x67000001,     //!< end effector reaches desired 6D transformation
		IKP_Rotation3D = 0x34000002,     //!< end effector reaches desired 3D rotation
		IKP_Translation3D = 0x33000003,     //!< end effector origin reaches desired 3D translation
		IKP_Direction3D = 0x23000004,     //!< direction on end effector coordinate system reaches desired direction
		IKP_Ray4D = 0x46000005,     //!< ray on end effector coordinate system reaches desired global ray
		IKP_Lookat3D = 0x23000006,     //!< direction on end effector coordinate system points to desired 3D position
		IKP_TranslationDirection5D = 0x56000007,     //!< end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
		IKP_TranslationXY2D = 0x22000008,     //!< 2D translation along XY plane
		IKP_TranslationXYOrientation3D = 0x33000009,     //!< 2D translation along XY plane and 1D rotation around Z axis. The offset of the rotation is measured starting at +X, so at +X is it 0, at +Y it is pi/2.
		IKP_TranslationLocalGlobal6D = 0x3600000a,     //!< local point on end effector origin reaches desired 3D global point

		IKP_TranslationXAxisAngle4D = 0x4400000b, //!< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with x-axis  like a cone, angle is from 0-pi. Axes defined in the manipulator base link's coordinate system)
		IKP_TranslationYAxisAngle4D = 0x4400000c, //!< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with y-axis  like a cone, angle is from 0-pi. Axes defined in the manipulator base link's coordinate system)
		IKP_TranslationZAxisAngle4D = 0x4400000d, //!< end effector origin reaches desired 3D translation, manipulator direction makes a specific angle with z-axis like a cone, angle is from 0-pi. Axes are defined in the manipulator base link's coordinate system.

		IKP_TranslationXAxisAngleZNorm4D = 0x4400000e, //!< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to z-axis and be rotated at a certain angle starting from the x-axis (defined in the manipulator base link's coordinate system)
		IKP_TranslationYAxisAngleXNorm4D = 0x4400000f, //!< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to x-axis and be rotated at a certain angle starting from the y-axis (defined in the manipulator base link's coordinate system)
		IKP_TranslationZAxisAngleYNorm4D = 0x44000010, //!< end effector origin reaches desired 3D translation, manipulator direction needs to be orthogonal to y-axis and be rotated at a certain angle starting from the z-axis (defined in the manipulator base link's coordinate system)

		IKP_NumberOfParameterizations = 16,     //!< number of parameterizations (does not count IKP_None)

		IKP_VelocityDataBit = 0x00008000, //!< bit is set if the data represents the time-derivate velocity of an IkParameterization
		IKP_Transform6DVelocity = IKP_Transform6D | IKP_VelocityDataBit,
		IKP_Rotation3DVelocity = IKP_Rotation3D | IKP_VelocityDataBit,
		IKP_Translation3DVelocity = IKP_Translation3D | IKP_VelocityDataBit,
		IKP_Direction3DVelocity = IKP_Direction3D | IKP_VelocityDataBit,
		IKP_Ray4DVelocity = IKP_Ray4D | IKP_VelocityDataBit,
		IKP_Lookat3DVelocity = IKP_Lookat3D | IKP_VelocityDataBit,
		IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D | IKP_VelocityDataBit,
		IKP_TranslationXY2DVelocity = IKP_TranslationXY2D | IKP_VelocityDataBit,
		IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D | IKP_VelocityDataBit,
		IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D | IKP_VelocityDataBit,
		IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D | IKP_VelocityDataBit,
		IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D | IKP_VelocityDataBit,
		IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D | IKP_VelocityDataBit,
		IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D | IKP_VelocityDataBit,
		IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D | IKP_VelocityDataBit,
		IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D | IKP_VelocityDataBit,

		IKP_UniqueIdMask = 0x0000ffff, //!< the mask for the unique ids
		IKP_CustomDataBit = 0x00010000, //!< bit is set if the ikparameterization contains custom data, this is only used when serializing the ik parameterizations
	};

	/// \brief returns a string of the ik parameterization type names
	///
	/// \param[in] alllowercase If 1, sets all characters to lower case. Otherwise can include upper case in order to match \ref IkParameterizationType definition.
	OPENRAVE_API const std::map<IkParameterizationType, std::string>& RaveGetIkParameterizationMap(int alllowercase = 0);

	/// \brief returns the IkParameterizationType given the unique id detmerined b IKP_UniqueIdMask
	OPENRAVE_API IkParameterizationType RaveGetIkTypeFromUniqueId(int uniqueid);

	/** \brief Parameterization of basic primitives for querying inverse-kinematics solutions.

	Holds the parameterization of a geometric primitive useful for autonomous manipulation scenarios like:
	6D pose, 3D translation, 3D rotation, 3D look at direction, and ray look at direction.
 */
	class OPENRAVE_API IkParameterization
	{
	public:
		IkParameterization() : type_(IKP_None)
		{
		}
		/// \brief sets a 6D transform parameterization
		IkParameterization(const Transform &t)
		{
			SetTransform6D(t);
		}
		/// \brief sets a ray parameterization
		IkParameterization(const RAY &r)
		{
			SetRay4D(r);
		}
		/// \brief set a custom parameterization using a transform as the source of the data. Not all types are supported with this method.
		IkParameterization(const Transform &t, IkParameterizationType type)
		{
			type_ = type;
			switch (type_)
			{
			case IKP_Transform6D: SetTransform6D(t); break;
			case IKP_Rotation3D: SetRotation3D(t.rot); break;
			case IKP_Translation3D: SetTranslation3D(t.trans); break;
			case IKP_Lookat3D: SetLookat3D(t.trans); break;
			default:
				throw OpenRAVEException(str(boost::format("IkParameterization constructor does not support type 0x%x") % type_));
			}
		}

		inline IkParameterizationType GetType() const
		{
			return type_;
		}

		/// \brief returns a string version of \ref GetType
		inline const std::string& GetName() const;

		/// \brief Returns the minimum degree of freedoms required for the IK type. Does \b not count custom data.
		static int GetDOF(IkParameterizationType type)
		{
			return (type >> 28) & 0xf;
		}
		/// \brief Returns the minimum degree of freedoms required for the IK type. Does \b not count custom data.
		inline int GetDOF() const
		{
			return (type_ >> 28) & 0xf;
		}

		/// \brief Returns the number of values used to represent the parameterization ( >= dof ). Does \b not count custom data.
		static int GetNumberOfValues(IkParameterizationType type)
		{
			return (type >> 24) & 0xf;
		}

		/// \brief returns a string of the ik parameterization type names
		///
		/// \param[in] alllowercase If 1, sets all characters to lower case. Otherwise can include upper case in order to match \ref IkParameterizationType definition.
		static const std::map<IkParameterizationType, std::string>& GetIkParameterizationMap(int alllowercase = 0);

		/// \brief returns the IkParameterizationType given the unique id detmerined b IKP_UniqueIdMask
		static IkParameterizationType GetIkTypeFromUniqueId(int uniqueid);

		/// \brief Returns the number of values used to represent the parameterization ( >= dof ). Does \b not count custom data.
		inline int GetNumberOfValues() const
		{
			return (type_ >> 24) & 0xf;
		}

		inline void SetTransform6D(const Transform& t)
		{
			type_ = IKP_Transform6D; transform_ = t;
		}

		inline void SetTransform6DVelocity(const Transform& t)
		{
			type_ = IKP_Transform6DVelocity; transform_ = t;
		}

		inline void SetRotation3D(const Vector& quaternion) 
		{
			type_ = IKP_Rotation3D; transform_.rot = quaternion;
		}

		inline void SetTranslation3D(const Vector& trans) 
		{
			type_ = IKP_Translation3D; transform_.trans = trans;
		}

		inline void SetDirection3D(const Vector& dir) 
		{
			type_ = IKP_Direction3D; transform_.rot = dir;
		}

		inline void SetRay4D(const RAY& ray) 
		{
			type_ = IKP_Ray4D; transform_.trans = ray.pos; transform_.rot = ray.dir;
		}

		inline void SetLookat3D(const Vector& trans) 
		{
			type_ = IKP_Lookat3D; transform_.trans = trans;
		}

		/// \brief the ray direction is not used for IK, however it is needed in order to compute the error
		inline void SetLookat3D(const RAY& ray) 
		{
			type_ = IKP_Lookat3D; transform_.trans = ray.pos; transform_.rot = ray.dir;
		}

		inline void SetTranslationDirection5D(const RAY& ray) 
		{
			type_ = IKP_TranslationDirection5D; transform_.trans = ray.pos; transform_.rot = ray.dir;
		}

		inline void SetTranslationXY2D(const Vector& trans)
		{
			type_ = IKP_TranslationXY2D; 
			transform_.trans.x = trans.x; 
			transform_.trans.y = trans.y; 
			transform_.trans.z = 0; 
			transform_.trans.w = 0;
		}

		inline void SetTranslationXYOrientation3D(const Vector& trans)
		{
			type_ = IKP_TranslationXYOrientation3D; 
			transform_.trans.x = trans.x; 
			transform_.trans.y = trans.y; 
			transform_.trans.z = trans.z; 
			transform_.trans.w = 0;
		}

		inline void SetTranslationLocalGlobal6D(const Vector& localtrans, const Vector& trans)
		{
			type_ = IKP_TranslationLocalGlobal6D; 
			transform_.rot.x = localtrans.x; 
			transform_.rot.y = localtrans.y; 
			transform_.rot.z = localtrans.z; 
			transform_.rot.w = 0; 
			transform_.trans.x = trans.x; 
			transform_.trans.y = trans.y; 
			transform_.trans.z = trans.z; 
			transform_.trans.w = 0;
		}

		inline void SetTranslationXAxisAngle4D(const Vector& trans, dReal angle) 
		{
			type_ = IKP_TranslationXAxisAngle4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline void SetTranslationYAxisAngle4D(const Vector& trans, dReal angle) 
		{
			type_ = IKP_TranslationYAxisAngle4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline void SetTranslationZAxisAngle4D(const Vector& trans, dReal angle)
		{
			type_ = IKP_TranslationZAxisAngle4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline void SetTranslationXAxisAngleZNorm4D(const Vector& trans, dReal angle) 
		{
			type_ = IKP_TranslationXAxisAngleZNorm4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline void SetTranslationYAxisAngleXNorm4D(const Vector& trans, dReal angle) 
		{
			type_ = IKP_TranslationYAxisAngleXNorm4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline void SetTranslationZAxisAngleYNorm4D(const Vector& trans, dReal angle) 
		{
			type_ = IKP_TranslationZAxisAngleYNorm4D;
			transform_.trans = trans;
			transform_.rot.x = angle;
		}

		inline const Transform& GetTransform6D() const 
		{
			return transform_;
		}

		inline const Vector& GetRotation3D() const 
		{
			return transform_.rot;
		}

		inline const Vector& GetTranslation3D() const 
		{
			return transform_.trans;
		}

		inline const Vector& GetDirection3D() const 
		{
			return transform_.rot;
		}

		inline const RAY GetRay4D() const 
		{
			return RAY(transform_.trans, transform_.rot);
		}

		inline const Vector& GetLookat3D() const
		{
			return transform_.trans;
		}

		inline const Vector& GetLookat3DDirection() const 
		{
			return transform_.rot;
		}

		inline const RAY GetTranslationDirection5D() const 
		{
			return RAY(transform_.trans, transform_.rot);
		}

		inline const Vector& GetTranslationXY2D() const 
		{
			return transform_.trans;
		}

		inline const Vector& GetTranslationXYOrientation3D() const 
		{
			return transform_.trans;
		}

		inline std::pair<Vector, Vector> GetTranslationLocalGlobal6D() const
		{
			return std::make_pair(transform_.rot, transform_.trans);
		}

		inline std::pair<Vector, dReal> GetTranslationXAxisAngle4D() const
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		inline std::pair<Vector, dReal> GetTranslationYAxisAngle4D() const 
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		inline std::pair<Vector, dReal> GetTranslationZAxisAngle4D() const 
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		inline std::pair<Vector, dReal> GetTranslationXAxisAngleZNorm4D() const 
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		inline std::pair<Vector, dReal> GetTranslationYAxisAngleXNorm4D() const 
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		inline std::pair<Vector, dReal> GetTranslationZAxisAngleYNorm4D() const 
		{
			return std::make_pair(transform_.trans, transform_.rot.x);
		}

		/// \brief Computes the distance squared between two IK parmaeterizations.
		inline dReal ComputeDistanceSqr(const IkParameterization& ikparam) const
		{
			const dReal anglemult = 0.4;     // this is a hack that should be removed....
			BOOST_ASSERT(type_ == ikparam.GetType());
			switch (type_) 
			{
			case IKP_Transform6D: 
			{
				Transform t0 = GetTransform6D(), t1 = ikparam.GetTransform6D();
				dReal fcos = RaveFabs(t0.rot.dot(t1.rot));
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return (t0.trans - t1.trans).lengthsqr3() + anglemult * facos*facos;
			}
			case IKP_Rotation3D:
			{
				dReal fcos = RaveFabs(GetRotation3D().dot(ikparam.GetRotation3D()));
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Translation3D:
				return (GetTranslation3D() - ikparam.GetTranslation3D()).lengthsqr3();
			case IKP_Direction3D:
			{
				dReal fcos = GetDirection3D().dot(ikparam.GetDirection3D());
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Ray4D: 
			{
				Vector pos0 = GetRay4D().pos - GetRay4D().dir*GetRay4D().dir.dot(GetRay4D().pos);
				Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
				dReal fcos = GetRay4D().dir.dot(ikparam.GetRay4D().dir);
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return (pos0 - pos1).lengthsqr3() + anglemult * facos*facos;
			}
			case IKP_Lookat3D:
			{
				Vector v = GetLookat3D() - ikparam.GetLookat3D();
				dReal s = v.dot3(ikparam.GetLookat3DDirection());
				if (s >= -1) 
				{     // ikparam's lookat is always 1 beyond the origin, this is just the convention for testing...
					v -= s * ikparam.GetLookat3DDirection();
				}
				return v.lengthsqr3();
			}
			case IKP_TranslationDirection5D:
			{
				dReal fcos = GetTranslationDirection5D().dir.dot(ikparam.GetTranslationDirection5D().dir);
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return (GetTranslationDirection5D().pos - ikparam.GetTranslationDirection5D().pos).lengthsqr3() + anglemult * facos*facos;
			}
			case IKP_TranslationXY2D: 
			{
				return (GetTranslationXY2D() - ikparam.GetTranslationXY2D()).lengthsqr2();
			}
			case IKP_TranslationXYOrientation3D: 
			{
				Vector v0 = GetTranslationXYOrientation3D();
				Vector v1 = ikparam.GetTranslationXYOrientation3D();
				dReal anglediff = v0.z - v1.z;
				if (anglediff < dReal(-PI))
				{
					anglediff += dReal(2 * PI);
					while (anglediff < dReal(-PI))
						anglediff += dReal(2 * PI);
				}
				else if (anglediff > dReal(PI)) 
				{
					anglediff -= dReal(2 * PI);
					while (anglediff > dReal(PI))
						anglediff -= dReal(2 * PI);
				}
				return (v0 - v1).lengthsqr2() + anglemult * anglediff*anglediff;
			}
			case IKP_TranslationLocalGlobal6D:
			{
				std::pair<Vector, Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
				return (p0.first - p1.first).lengthsqr3() + (p0.second - p1.second).lengthsqr3();
			}
			case IKP_TranslationXAxisAngle4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (p0.first - p1.first).lengthsqr3() + (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationYAxisAngle4D:
			{
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (p0.first - p1.first).lengthsqr3() + (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationZAxisAngle4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (p0.first - p1.first).lengthsqr3() + (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationXAxisAngleZNorm4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return (p0.first - p1.first).lengthsqr3() + anglediff * anglediff;
			}
			case IKP_TranslationYAxisAngleXNorm4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return (p0.first - p1.first).lengthsqr3() + anglediff * anglediff;
			}
			case IKP_TranslationZAxisAngleYNorm4D:
			{
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return (p0.first - p1.first).lengthsqr3() + anglediff * anglediff;
			}
			default:
				BOOST_ASSERT(0);
			}
			return 1e30;
		}

		/// \brief Computes the translational distance squared between two IK parmaeterizations.
		inline dReal ComputeTransDistanceSqr(const IkParameterization& ikparam) const
		{
			BOOST_ASSERT(type_ == ikparam.GetType());
			switch (type_) 
			{
			case IKP_Transform6D:
			{
				return (GetTransform6D().trans - ikparam.GetTransform6D().trans).lengthsqr3();
			}
			case IKP_Translation3D:
				return (GetTranslation3D() - ikparam.GetTranslation3D()).lengthsqr3();
			case IKP_Ray4D: 
			{
				Vector pos0 = GetRay4D().pos - GetRay4D().dir*GetRay4D().dir.dot(GetRay4D().pos);
				Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
				return (pos0 - pos1).lengthsqr3();
			}
			case IKP_TranslationDirection5D:
			{
				return (GetTranslationDirection5D().pos - ikparam.GetTranslationDirection5D().pos).lengthsqr3();
			}
			case IKP_TranslationXY2D:
			{
				return (GetTranslationXY2D() - ikparam.GetTranslationXY2D()).lengthsqr2();
			}
			case IKP_TranslationXYOrientation3D: 
			{
				Vector v0 = GetTranslationXYOrientation3D();
				Vector v1 = ikparam.GetTranslationXYOrientation3D();
				return (v0 - v1).lengthsqr2();
			}
			case IKP_TranslationLocalGlobal6D: 
			{
				std::pair<Vector, Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationXAxisAngle4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationYAxisAngle4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationZAxisAngle4D:
			{
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationXAxisAngleZNorm4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationYAxisAngleXNorm4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			case IKP_TranslationZAxisAngleYNorm4D: 
			{
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
				return (p0.first - p1.first).lengthsqr3();
			}
			default:
				throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", type_, ORE_InvalidArguments);
			}
			return 1e30;
		}

		/// \brief Computes the rotational distance squared between two IK parmaeterizations.
		inline dReal ComputeRotDistanceSqr(const IkParameterization& ikparam) const
		{
			BOOST_ASSERT(type_ == ikparam.GetType());
			switch (type_) 
			{
			case IKP_Transform6D: 
			{
				Transform t0 = GetTransform6D(), t1 = ikparam.GetTransform6D();
				dReal fcos = RaveFabs(t0.rot.dot(t1.rot));
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Rotation3D: 
			{
				dReal fcos = RaveFabs(GetRotation3D().dot(ikparam.GetRotation3D()));
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Direction3D: 
			{
				dReal fcos = GetDirection3D().dot(ikparam.GetDirection3D());
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Ray4D: 
			{
				dReal fcos = GetRay4D().dir.dot(ikparam.GetRay4D().dir);
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_Lookat3D: 
			{
				Vector v = GetLookat3D() - ikparam.GetLookat3D();
				dReal s = v.dot3(ikparam.GetLookat3DDirection());
				if (s >= -1) 
				{     // ikparam's lookat is always 1 beyond the origin, this is just the convention for testing...
					v -= s * ikparam.GetLookat3DDirection();
				}
				return v.lengthsqr3();
			}
			case IKP_TranslationDirection5D: 
			{
				dReal fcos = GetTranslationDirection5D().dir.dot(ikparam.GetTranslationDirection5D().dir);
				dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
				return facos * facos;
			}
			case IKP_TranslationXYOrientation3D:
			{
				Vector v0 = GetTranslationXYOrientation3D();
				Vector v1 = ikparam.GetTranslationXYOrientation3D();
				dReal anglediff = v0.z - v1.z;
				if (anglediff < dReal(-PI)) {
					anglediff += dReal(2 * PI);
					while (anglediff < dReal(-PI))
						anglediff += dReal(2 * PI);
				}
				else if (anglediff > dReal(PI)) {
					anglediff -= dReal(2 * PI);
					while (anglediff > dReal(PI))
						anglediff -= dReal(2 * PI);
				}
				return anglediff * anglediff;
			}
			case IKP_TranslationLocalGlobal6D: {
				std::pair<Vector, Vector> p0 = GetTranslationLocalGlobal6D(), p1 = ikparam.GetTranslationLocalGlobal6D();
				return (p0.second - p1.second).lengthsqr3();
			}
			case IKP_TranslationXAxisAngle4D: {
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngle4D(), p1 = ikparam.GetTranslationXAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationYAxisAngle4D: {
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngle4D(), p1 = ikparam.GetTranslationYAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationZAxisAngle4D: {
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngle4D(), p1 = ikparam.GetTranslationZAxisAngle4D();
				// dot product with axis is always in [0,pi]
				dReal angle0 = RaveFabs(NormalizeCircularAnglePrivate(p0.second, -PI, PI));
				dReal angle1 = RaveFabs(NormalizeCircularAnglePrivate(p1.second, -PI, PI));
				return (angle0 - angle1)*(angle0 - angle1);
			}
			case IKP_TranslationXAxisAngleZNorm4D: {
				std::pair<Vector, dReal> p0 = GetTranslationXAxisAngleZNorm4D(), p1 = ikparam.GetTranslationXAxisAngleZNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return anglediff * anglediff;
			}
			case IKP_TranslationYAxisAngleXNorm4D: {
				std::pair<Vector, dReal> p0 = GetTranslationYAxisAngleXNorm4D(), p1 = ikparam.GetTranslationYAxisAngleXNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return anglediff * anglediff;
			}
			case IKP_TranslationZAxisAngleYNorm4D: {
				std::pair<Vector, dReal> p0 = GetTranslationZAxisAngleYNorm4D(), p1 = ikparam.GetTranslationZAxisAngleYNorm4D();
				dReal anglediff = NormalizeCircularAnglePrivate(p0.second - p1.second, -PI, PI);
				return anglediff * anglediff;
			}
			default:
				throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", type_, ORE_InvalidArguments);
			}
			return 1e30;
		}

		/// \brief fills the iterator with the serialized values of the ikparameterization.
		///
		/// The container the iterator points to needs to have \ref GetNumberOfValues() available.
		/// Does not support custom data
		/// Don't normalize quaternions since it could hold velocity data.
		inline void GetValues(std::vector<dReal>::iterator itvalues) const
		{
			switch (type_ & ~IKP_VelocityDataBit) {
			case IKP_Transform6D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				*itvalues++ = transform_.rot.w;
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_Rotation3D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				*itvalues++ = transform_.rot.w;
				break;
			case IKP_Translation3D:
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_Direction3D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				break;
			case IKP_Ray4D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_Lookat3D:
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_TranslationDirection5D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_TranslationXY2D:
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				break;
			case IKP_TranslationXYOrientation3D:
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_TranslationLocalGlobal6D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.rot.y;
				*itvalues++ = transform_.rot.z;
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			case IKP_TranslationXAxisAngle4D:
			case IKP_TranslationYAxisAngle4D:
			case IKP_TranslationZAxisAngle4D:
			case IKP_TranslationXAxisAngleZNorm4D:
			case IKP_TranslationYAxisAngleXNorm4D:
			case IKP_TranslationZAxisAngleYNorm4D:
				*itvalues++ = transform_.rot.x;
				*itvalues++ = transform_.trans.x;
				*itvalues++ = transform_.trans.y;
				*itvalues++ = transform_.trans.z;
				break;
			default:
				throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", type_, ORE_InvalidArguments);
			}
		}

		/// \brief sets a serialized set of values for the IkParameterization
		///
		/// Function does not handle custom data. Don't normalize quaternions since it could hold velocity data.
		inline void SetValues(std::vector<dReal>::const_iterator itvalues, IkParameterizationType iktype)
		{
			type_ = iktype;
			switch (type_ & ~IKP_VelocityDataBit) {
			case IKP_Transform6D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				transform_.rot.w = *itvalues++;
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_Rotation3D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				transform_.rot.w = *itvalues++;
				break;
			case IKP_Translation3D:
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_Direction3D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				break;
			case IKP_Ray4D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_Lookat3D:
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_TranslationDirection5D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_TranslationXY2D:
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				break;
			case IKP_TranslationXYOrientation3D:
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_TranslationLocalGlobal6D:
				transform_.rot.x = *itvalues++;
				transform_.rot.y = *itvalues++;
				transform_.rot.z = *itvalues++;
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			case IKP_TranslationXAxisAngle4D:
			case IKP_TranslationYAxisAngle4D:
			case IKP_TranslationZAxisAngle4D:
			case IKP_TranslationXAxisAngleZNorm4D:
			case IKP_TranslationYAxisAngleXNorm4D:
			case IKP_TranslationZAxisAngleYNorm4D:
				transform_.rot.x = *itvalues++;
				transform_.trans.x = *itvalues++;
				transform_.trans.y = *itvalues++;
				transform_.trans.z = *itvalues++;
				break;
			default:
				throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", type_, ORE_InvalidArguments);
			}
		}

		inline void Set(std::vector<dReal>::const_iterator itvalues, IkParameterizationType iktype) 
		{
			SetValues(itvalues, iktype);
		}

		/** \brief sets named custom data in the ik parameterization

			The custom data is serialized along with the rest of the parameters and can also be part of a configuration specification under the "ikparam_values" anotation.
			The custom data name can have meta-tags for the type of transformation the data undergos when \ref MultiplyTransform is called. For example, if the user wants to have an extra 3 values that represent "direction", then the direction has to be rotated along with all the data or coordinate systems can get lost. The anotations are specified by putting:

			\b transform_=%s_

			somewhere in the string. The %s can be: \b direction, \b point, \b quat, \b ikparam

			If \b ikparam, the first value is expected to be the unique id of the ik type (GetType()&IKP_UniqueIdMask). The other values can be computed from \ref IkParameterization::GetValues

			\param name Describes the type of data, cannot contain spaces or new lines.
			\param values the values representing the data
			\throw OpenRAVEException throws if the name is invalid
		 */
		inline void SetCustomValues(const std::string& name, const std::vector<dReal>& values)
		{
			OPENRAVE_ASSERT_OP_FORMAT0(name.size(), > , 0, "name is empty", ORE_InvalidArguments);
			OPENRAVE_ASSERT_OP_FORMAT0(std::count_if(name.begin(), name.end(), _IsValidCharInName), == , (int)name.size(), "name has invalid characters", ORE_InvalidArguments);
			custom_data_map_[name] = values;
		}

		/// \brief sets named custom data in the ik parameterization (\see SetCustomValues)
		inline void SetCustomValue(const std::string& name, dReal value)
		{
			OPENRAVE_ASSERT_OP_FORMAT0(name.size(), > , 0, "name is empty", ORE_InvalidArguments);
			OPENRAVE_ASSERT_OP_FORMAT0(std::count_if(name.begin(), name.end(), _IsValidCharInName), == , (int)name.size(), "name has invalid characters", ORE_InvalidArguments);
			custom_data_map_[name].resize(1);
			custom_data_map_[name][0] = value;
		}

		/// \brief gets custom data if it exists, returns false if it doesn't
		inline bool GetCustomValues(const std::string& name, std::vector<dReal>& values) const
		{
			std::map<std::string, std::vector<dReal> >::const_iterator it = custom_data_map_.find(name);
			if (it == custom_data_map_.end()) 
			{
				return false;
			}
			values = it->second;
			return true;
		}

		/// \brief returns the first element of a custom value. If custom_data_map_ does not have 'name' and is not > 0, then will return defaultValue
		inline dReal GetCustomValue(const std::string& name, dReal defaultValue) const
		{
			std::map<std::string, std::vector<dReal> >::const_iterator it = custom_data_map_.find(name);
			if (it != custom_data_map_.end() && it->second.size() > 0) 
			{
				return it->second[0];
			}
			return defaultValue;
		}

		/// \brief returns a const reference of the custom data key/value pairs
		inline const std::map<std::string, std::vector<dReal> >& GetCustomDataMap() const
		{
			return custom_data_map_;
		}

		/// \brief clears custom data
		///
		/// \param name if name is empty, will clear all the data, otherwise will clear only the custom data with that name
		/// \return number of elements erased
		inline size_t ClearCustomValues(const std::string& name = std::string())
		{
			if (name.size() > 0) 
			{
				return custom_data_map_.erase(name) > 0;
			}
			else 
			{
				size_t num = custom_data_map_.size();
				custom_data_map_.clear();
				return num;
			}
		}

		static ConfigurationSpecification GetConfigurationSpecification(IkParameterizationType iktype, 
			const std::string& interpolation = "", const std::string& robotname = "", const std::string& manipname = "");

		inline ConfigurationSpecification GetConfigurationSpecification(const std::string& interpolation = "", 
			const std::string& robotname = "", const std::string& manipname = "") const
		{
			return GetConfigurationSpecification(GetType(), interpolation, robotname, manipname);
		}

		/// \brief in-place left-transform into a new coordinate system. Equivalent to t * ikparam
		inline IkParameterization& MultiplyTransform(const Transform& t) {
			switch (GetType()) {
			case IKP_Transform6D:
				transform_ = t * transform_;
				break;
			case IKP_Transform6DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				transform_.rot = quatMultiply(t.rot, transform_.rot);
				break;
			case IKP_Rotation3D:
			case IKP_Rotation3DVelocity:
				transform_.rot = quatMultiply(t.rot, transform_.rot);
				break;
			case IKP_Translation3D:
				transform_.trans = t * transform_.trans;
				break;
			case IKP_Translation3DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				break;
			case IKP_Direction3D:
			case IKP_Direction3DVelocity:
				transform_.rot = t.rotate(transform_.rot);
				break;
			case IKP_Ray4D:
				transform_.trans = t * transform_.trans;
				transform_.rot = t.rotate(transform_.rot);
				break;
			case IKP_Ray4DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				transform_.rot = t.rotate(transform_.rot);
				break;
			case IKP_Lookat3D:
				SetLookat3D(RAY(t*GetLookat3D(), t.rotate(GetLookat3DDirection())));
				break;
			case IKP_TranslationDirection5D:
				transform_.trans = t * transform_.trans;
				transform_.rot = t.rotate(transform_.rot);
				break;
			case IKP_TranslationDirection5DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				transform_.rot = t.rotate(transform_.rot);
				break;
			case IKP_TranslationXY2D:
				SetTranslationXY2D(t*GetTranslationXY2D());
				break;
			case IKP_TranslationXY2DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				break;
			case IKP_TranslationXYOrientation3D: {
				Vector v = GetTranslationXYOrientation3D();
				Vector voldtrans(v.x, v.y, 0);
				Vector vnewtrans = t * voldtrans;
				dReal zangle = -normalizeAxisRotation(Vector(0, 0, 1), t.rot).first;
				SetTranslationXYOrientation3D(Vector(vnewtrans.x, vnewtrans.y, v.z + zangle));
				break;
			}
			case IKP_TranslationXYOrientation3DVelocity: {
				Vector v = GetTranslationXYOrientation3D();
				Vector voldtrans(v.x, v.y, 0);
				transform_.trans = t.rotate(voldtrans);
				transform_.trans.z = quatRotate(t.rot, Vector(0, 0, v.z)).z;
				break;
			}
			case IKP_TranslationLocalGlobal6D:
				transform_.trans = t * transform_.trans;
				break;
			case IKP_TranslationLocalGlobal6DVelocity:
				transform_.trans = t.rotate(transform_.trans);
				break;
			case IKP_TranslationXAxisAngle4D: {
				transform_.trans = t * transform_.trans;
				// do not support rotations
				break;
			}
			case IKP_TranslationXAxisAngle4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// do not support rotations
				break;
			}
			case IKP_TranslationYAxisAngle4D: {
				transform_.trans = t * transform_.trans;
				// do not support rotations
				break;
			}
			case IKP_TranslationYAxisAngle4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// do not support rotations
				break;
			}
			case IKP_TranslationZAxisAngle4D: {
				transform_.trans = t * transform_.trans;
				// do not support rotations
				break;
			}
			case IKP_TranslationZAxisAngle4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// do not support rotations
				break;
			}

			case IKP_TranslationXAxisAngleZNorm4D: {
				transform_.trans = t * transform_.trans;
				// only support rotation along z-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(0, 0, 1), t.rot).first;
				break;
			}
			case IKP_TranslationXAxisAngleZNorm4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// only support rotation along z-axis
				transform_.rot.x = quatRotate(t.rot, Vector(0, 0, transform_.rot.x)).z;
				break;
			}
			case IKP_TranslationYAxisAngleXNorm4D: {
				transform_.trans = t * transform_.trans;
				// only support rotation along x-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(1, 0, 0), t.rot).first;
				break;
			}
			case IKP_TranslationYAxisAngleXNorm4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// only support rotation along x-axis
				transform_.rot.x = quatRotate(t.rot, Vector(transform_.rot.x, 0, 0)).x;
				break;
			}
			case IKP_TranslationZAxisAngleYNorm4D: {
				transform_.trans = t * transform_.trans;
				// only support rotation along y-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(0, 1, 0), t.rot).first;
				break;
			}
			case IKP_TranslationZAxisAngleYNorm4DVelocity: {
				transform_.trans = t.rotate(transform_.trans);
				// only support rotation along y-axis
				transform_.rot.x = quatRotate(t.rot, Vector(0, transform_.rot.x, 0)).y;
				break;
			}
			default:
				throw OpenRAVEException(str(boost::format("parameterization 0x%x does not support left-transform") % GetType()));
			}
			for (std::map<std::string, std::vector<dReal> >::iterator it = custom_data_map_.begin(); it != custom_data_map_.end(); ++it) {
				_MultiplyTransform(t, it->first, it->second);
			}
			return *this;
		}

		/** \brief in-place right-transform into a new coordinate system. Equivalent to ikparam*t

			Note that depending on the ikparam type, some information from the passed in transform can get lost or misinterpreted. For example
			Translation3D types do not have a rotation, so assume identity.

			For ik types that have 3D directions stored, the transformation is the following:
			\code
			quatRotate(quatMultiply(quatRotateDirection(Vector(0,0,1),direction), t.rot), Vector(0,0,1))
			\endcode
			Basically it is how the local z axis gets transformed and converting that back to world coordinates.
		 */
		inline IkParameterization& MultiplyTransformRight(const Transform& t) {
			switch (GetType()) {
			case IKP_Transform6D:
				transform_ *= t;
				break;
				//        case IKP_Transform6DVelocity:
				//            transform_.trans = t.rotate(transform_.trans);
				//            transform_.rot = quatMultiply(t.rot,transform_.rot);
				//            break;
			case IKP_Rotation3D:
				//        case IKP_Rotation3DVelocity:
				transform_.rot = quatMultiply(transform_.rot, t.rot);
				break;
			case IKP_Translation3D:
				transform_.trans = transform_.trans + t.trans;
				break;
				//        case IKP_Translation3DVelocity:
				//            transform_.trans = t.rotate(transform_.trans);
				//            break;
			case IKP_Direction3D:
				//        case IKP_Direction3DVelocity:
				transform_.rot = quatRotate(quatMultiply(quatRotateDirection(Vector(0, 0, 1), transform_.rot), t.rot), Vector(0, 0, 1));
				break;
				//        case IKP_Ray4D:
				//            transform_.trans = t * transform_.trans;
				//            transform_.rot = t.rotate(transform_.rot);
				//            break;
				//        case IKP_Ray4DVelocity:
				//            transform_.trans = t.rotate(transform_.trans);
				//            transform_.rot = t.rotate(transform_.rot);
				//            break;
			case IKP_Lookat3D:
				SetLookat3D(GetLookat3D() + t.trans);
				break;
			case IKP_TranslationDirection5D: {
				Vector qorig = quatRotateDirection(Vector(0, 0, 1), transform_.rot);
				Vector q = quatMultiply(qorig, t.rot);
				transform_.trans += quatRotate(qorig, t.trans);
				transform_.rot = quatRotate(q, Vector(0, 0, 1));
				break;
			}
											 //        case IKP_TranslationDirection5DVelocity:
											 //            transform_.trans = t.rotate(transform_.trans);
											 //            transform_.rot = t.rotate(transform_.rot);
											 //            break;
			case IKP_TranslationXY2D:
				SetTranslationXY2D(GetTranslationXY2D() + t.trans);
				break;
				//        case IKP_TranslationXY2DVelocity:
				//            transform_.trans = t.rotate(transform_.trans);
				//            break;
			case IKP_TranslationXYOrientation3D: {
				Vector v = GetTranslationXYOrientation3D();
				Vector voldtrans(v.x, v.y, 0);
				Vector q = quatFromAxisAngle(Vector(0, 0, 1), v.z);
				Vector vnewtrans = voldtrans + quatRotate(q, t.trans);
				dReal zangle = -normalizeAxisRotation(Vector(0, 0, 1), t.rot).first;
				SetTranslationXYOrientation3D(Vector(vnewtrans.x, vnewtrans.y, v.z + zangle));
				break;
			}
												 //        case IKP_TranslationXYOrientation3DVelocity: {
												 //            Vector v = GetTranslationXYOrientation3D();
												 //            Vector voldtrans(v.x,v.y,0);
												 //            transform_.trans = t.rotate(voldtrans);
												 //            transform_.trans.z = quatRotate(t.rot,Vector(0,0,v.z)).z;
												 //            break;
												 //        }
			case IKP_TranslationLocalGlobal6D:
				transform_.trans = transform_.trans + t.trans;
				break;
				//        case IKP_TranslationLocalGlobal6DVelocity:
				//            transform_.trans = t.rotate(transform_.trans);
				//            break;
				//        case IKP_TranslationXAxisAngle4D: {
				//            transform_.trans = t*transform_.trans;
				//            // do not support rotations
				//            break;
				//        }
				//        case IKP_TranslationXAxisAngle4DVelocity: {
				//            transform_.trans = t.rotate(transform_.trans);
				//            // do not support rotations
				//            break;
				//        }
				//        case IKP_TranslationYAxisAngle4D: {
				//            transform_.trans = t*transform_.trans;
				//            // do not support rotations
				//            break;
				//        }
				//        case IKP_TranslationYAxisAngle4DVelocity: {
				//            transform_.trans = t.rotate(transform_.trans);
				//            // do not support rotations
				//            break;
				//        }
				//        case IKP_TranslationZAxisAngle4D: {
				//            transform_.trans = t*transform_.trans;
				//            // do not support rotations
				//            break;
				//        }
				//        case IKP_TranslationZAxisAngle4DVelocity: {
				//            transform_.trans = t.rotate(transform_.trans);
				//            // do not support rotations
				//            break;
				//        }
			case IKP_TranslationXAxisAngleZNorm4D: {
				Vector q = quatFromAxisAngle(Vector(0, 0, 1), transform_.rot.x);
				transform_.trans += quatRotate(q, t.trans);
				// only support rotation along z-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(0, 0, 1), t.rot).first;
				break;
			}
												   //        case IKP_TranslationXAxisAngleZNorm4DVelocity: {
												   //            transform_.trans = t.rotate(transform_.trans);
												   //            // only support rotation along z-axis
												   //            transform_.rot.x = quatRotate(t.rot,Vector(0,0,transform_.rot.x)).z;
												   //            break;
												   //        }
			case IKP_TranslationYAxisAngleXNorm4D: {
				Vector q = quatFromAxisAngle(Vector(1, 0, 0), transform_.rot.x);
				transform_.trans += quatRotate(q, t.trans);
				// only support rotation along x-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(1, 0, 0), t.rot).first;
				break;
			}
												   //        case IKP_TranslationYAxisAngleXNorm4DVelocity: {
												   //            transform_.trans = t.rotate(transform_.trans);
												   //            // only support rotation along x-axis
												   //            transform_.rot.x = quatRotate(t.rot,Vector(transform_.rot.x,0,0)).x;
												   //            break;
												   //        }
			case IKP_TranslationZAxisAngleYNorm4D: {
				Vector q = quatFromAxisAngle(Vector(0, 1, 0), transform_.rot.x);
				transform_.trans += quatRotate(q, t.trans);
				// only support rotation along y-axis
				transform_.rot.x -= normalizeAxisRotation(Vector(0, 1, 0), t.rot).first;
				break;
			}
												   //        case IKP_TranslationZAxisAngleYNorm4DVelocity: {
												   //            transform_.trans = t.rotate(transform_.trans);
												   //            // only support rotation along y-axis
												   //            transform_.rot.x = quatRotate(t.rot,Vector(0,transform_.rot.x,0)).y;
												   //            break;
												   //        }
			default:
				throw OpenRAVEException(str(boost::format("parameterization 0x%x does not support right-transforms") % GetType()));
			}
			for (std::map<std::string, std::vector<dReal> >::iterator it = custom_data_map_.begin(); it != custom_data_map_.end(); ++it) {
				_MultiplyTransformRight(t, it->first, it->second);
			}
			return *this;
		}

		inline IkParameterization operator*(const Transform& t) const {
			IkParameterization iknew(*this);
			iknew.MultiplyTransformRight(t);
			return iknew;
		}

		inline void Swap(IkParameterization& r)
		{
			std::swap(transform_, r.transform_);
			std::swap(type_, r.type_);
			custom_data_map_.swap(r.custom_data_map_);
		}

		void SerializeJSON(rapidjson::Value& rIkParameterization, 
			rapidjson::Document::AllocatorType& alloc, dReal fUnitScale) const;

		void DeserializeJSON(const rapidjson::Value& rIkParameterization, dReal fUnitScale);

	protected:
		inline static bool _IsValidCharInName(char c) 
		{
			return c < 0 || c >= 33;
		}

		inline static void _MultiplyTransform(const Transform& t, const std::string& name, std::vector<dReal>& values)
		{
			size_t startoffset = name.find("_transform=");
			if (startoffset != std::string::npos) {
				size_t endoffset = name.find("_", startoffset + 11);
				std::string transformtype;
				if (endoffset == std::string::npos) {
					transformtype = name.substr(startoffset + 11);
				}
				else {
					transformtype = name.substr(startoffset + 11, endoffset - startoffset - 11);
				}
				if (transformtype == "direction") {
					if (values.size() < 3) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2]);
					v = t.rotate(v);
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
				}
				else if (transformtype == "point") {
					if (values.size() < 3) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2]);
					v = t * v;
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
				}
				else if (transformtype == "quat") {
					if (values.size() < 4) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 4", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2], values[3]);
					v = quatMultiply(t.rot, v);
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2]; values[3] = v[3];
				}
				else if (transformtype == "ikparam") {
					IkParameterizationType newiktype = RaveGetIkTypeFromUniqueId(static_cast<int>(values.at(0) + 0.5));
					IkParameterization newikparam;
					OPENRAVE_ASSERT_OP_FORMAT0(IkParameterization::GetNumberOfValues(newiktype) + 1, == , (int)values.size(), "expected values not equal", ORE_InvalidState);
					newikparam.SetValues(values.begin() + 1, newiktype);
					newikparam.MultiplyTransform(t);
					newikparam.GetValues(values.begin() + 1);
				}
				else {
					throw OPENRAVE_EXCEPTION_FORMAT("IkParameterization custom data '%s' does not have a valid transform", name, ORE_InvalidState);
				}
			}
		}

		inline static void _MultiplyTransformRight(const Transform& t, const std::string& name, std::vector<dReal>& values)
		{
			size_t startoffset = name.find("_transform=");
			if (startoffset != std::string::npos) {
				size_t endoffset = name.find("_", startoffset + 11);
				std::string transformtype;
				if (endoffset == std::string::npos) {
					transformtype = name.substr(startoffset + 11);
				}
				else {
					transformtype = name.substr(startoffset + 11, endoffset - startoffset - 11);
				}
				if (transformtype == "direction") {
					if (values.size() < 3) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2]);
					v = quatRotate(quatMultiply(quatRotateDirection(Vector(0, 0, 1), v), t.rot), Vector(0, 0, 1));
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
				}
				else if (transformtype == "point") {
					if (values.size() < 3) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 3", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2]);
					v += t.trans;
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2];
				}
				else if (transformtype == "quat") {
					if (values.size() < 4) {
						throw OPENRAVE_EXCEPTION_FORMAT0("Vector size < 4", ORE_InvalidArguments);
					}
					Vector v(values[0], values[1], values[2], values[3]);
					v = quatMultiply(v, t.rot);
					values[0] = v[0]; values[1] = v[1]; values[2] = v[2]; values[3] = v[3];
				}
				else if (transformtype == "ikparam") {
					IkParameterizationType newiktype = RaveGetIkTypeFromUniqueId(static_cast<int>(values.at(0) + 0.5));
					IkParameterization newikparam;
					OPENRAVE_ASSERT_OP_FORMAT0(IkParameterization::GetNumberOfValues(newiktype) + 1, == , (int)values.size(), "expected values not equal", ORE_InvalidState);
					newikparam.SetValues(values.begin() + 1, newiktype);
					newikparam.MultiplyTransformRight(t);
					newikparam.GetValues(values.begin() + 1);
				}
				else {
					throw OPENRAVE_EXCEPTION_FORMAT("IkParameterization custom data '%s' does not have a valid transform", name, ORE_InvalidState);
				}
			}
		}

		Transform transform_;
		IkParameterizationType type_;
		std::map<std::string, std::vector<dReal> > custom_data_map_;

		friend IkParameterization operator* (const Transform &t, const IkParameterization &ikparam);
		friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
		friend OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);
	};

	inline IkParameterization operator* (const Transform &t, const IkParameterization &ikparam)
	{
		IkParameterization local;
		switch (ikparam.GetType()) {
		case IKP_Transform6D:
			local.SetTransform6D(t * ikparam.GetTransform6D());
			break;
		case IKP_Rotation3D:
			local.SetRotation3D(quatMultiply(t.rot, ikparam.GetRotation3D()));
			break;
		case IKP_Translation3D:
			local.SetTranslation3D(t*ikparam.GetTranslation3D());
			break;
		case IKP_Direction3D:
			local.SetDirection3D(t.rotate(ikparam.GetDirection3D()));
			break;
		case IKP_Ray4D:
			local.SetRay4D(RAY(t*ikparam.GetRay4D().pos, t.rotate(ikparam.GetRay4D().dir)));
			break;
		case IKP_Lookat3D:
			local.SetLookat3D(RAY(t*ikparam.GetLookat3D(), t.rotate(ikparam.GetLookat3DDirection())));
			break;
		case IKP_TranslationDirection5D:
			local.SetTranslationDirection5D(RAY(t*ikparam.GetTranslationDirection5D().pos, t.rotate(ikparam.GetTranslationDirection5D().dir)));
			break;
		case IKP_TranslationXY2D:
			local.SetTranslationXY2D(t*ikparam.GetTranslationXY2D());
			break;
		case IKP_TranslationXYOrientation3D: {
			Vector v = ikparam.GetTranslationXYOrientation3D();
			Vector voldtrans(v.x, v.y, 0);
			Vector vnewtrans = t * voldtrans;
			dReal zangle = -normalizeAxisRotation(Vector(0, 0, 1), t.rot).first;
			local.SetTranslationXYOrientation3D(Vector(vnewtrans.x, vnewtrans.y, v.z + zangle));
			break;
		}
		case IKP_TranslationLocalGlobal6D:
			local.SetTranslationLocalGlobal6D(ikparam.GetTranslationLocalGlobal6D().first, t*ikparam.GetTranslationLocalGlobal6D().second);
			break;
		case IKP_TranslationXAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationXAxisAngle4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationXAxisAngle4D(t*p.first, p.second);
			break;
		}
		case IKP_TranslationYAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationYAxisAngle4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationYAxisAngle4D(t*p.first, p.second);
			break;
		}
		case IKP_TranslationZAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationZAxisAngle4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationZAxisAngle4D(t*p.first, p.second);
			break;
		}
		case IKP_TranslationXAxisAngleZNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationXAxisAngleZNorm4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationXAxisAngleZNorm4D(t*p.first, p.second);
			break;
		}
		case IKP_TranslationYAxisAngleXNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationYAxisAngleXNorm4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationYAxisAngleXNorm4D(t*p.first, p.second);
			break;
		}
		case IKP_TranslationZAxisAngleYNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationZAxisAngleYNorm4D();
			// don't change the angle since don't know the exact direction it is pointing at
			local.SetTranslationZAxisAngleYNorm4D(t*p.first, p.second);
			break;
		}
		default:
			// internal MultiplyTransform supports more types
			return IkParameterization(ikparam).MultiplyTransform(t);
		}
		local.custom_data_map_ = ikparam.custom_data_map_;
		for (std::map<std::string, std::vector<dReal> >::iterator it = local.custom_data_map_.begin();
			it != local.custom_data_map_.end(); ++it)
		{
			IkParameterization::_MultiplyTransform(t, it->first, it->second);
		}
		return local;
	}

	OPENRAVE_API std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam);
	OPENRAVE_API std::istream& operator>>(std::istream& I, IkParameterization& ikparam);

	// define inline functions
	const std::string& IkParameterization::GetName() const
	{
		std::map<IkParameterizationType, std::string>::const_iterator it = RaveGetIkParameterizationMap().find(type_);
		if (it != RaveGetIkParameterizationMap().end())
		{
			return it->second;
		}
		throw OpenRAVEException(str(boost::format("IkParameterization iktype 0x%x not supported") % type_));
	}
}

#endif // OPENRAVE_IK_PARAMETERIZATION_H_
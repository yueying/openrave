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
#include <openrave/ik_parameterization.h>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/openrave_exception.h>

namespace OpenRAVE
{
	const std::map<IkParameterizationType, std::string>& RaveGetIkParameterizationMap(int alllowercase)
	{
		return IkParameterization::GetIkParameterizationMap(alllowercase);
	}

	IkParameterizationType RaveGetIkTypeFromUniqueId(int uniqueid)
	{
		return IkParameterization::GetIkTypeFromUniqueId(uniqueid);
	}

	IkParameterizationType IkParameterization::GetIkTypeFromUniqueId(int uniqueid)
	{
		uniqueid &= IKP_UniqueIdMask;
		FOREACHC(it, RaveGlobal::instance()->GetIkParameterizationMap()) 
		{
			if ((it->first & (IKP_UniqueIdMask&~IKP_VelocityDataBit)) == (uniqueid&(IKP_UniqueIdMask&~IKP_VelocityDataBit)))
			{
				return static_cast<IkParameterizationType>(it->first | (uniqueid&IKP_VelocityDataBit));
			}
		}
		throw OPENRAVE_EXCEPTION_FORMAT(_("no ik exists of unique id 0x%x"), uniqueid, ORE_InvalidArguments);
	}

	ConfigurationSpecification IkParameterization::GetConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation, const std::string& robotname, const std::string& manipname)
	{
		ConfigurationSpecification spec;
		spec._vgroups.resize(1);
		spec._vgroups[0].offset = 0;
		spec._vgroups[0].dof = IkParameterization::GetNumberOfValues(iktype);
		spec._vgroups[0].name = str(boost::format("ikparam_values %d") % iktype);
		if (robotname.size() > 0) {
			spec._vgroups[0].name += robotname;
			spec._vgroups[0].name += " ";
			if (manipname.size() > 0) {
				spec._vgroups[0].name += manipname;
			}
		}
		spec._vgroups[0].interpolation = interpolation;

		// remove any trailing whitespace from missing robot or manipulator names
		boost::algorithm::trim(spec._vgroups[0].name);
		return spec;
	}

	std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam)
	{
		int type = ikparam._type;
		BOOST_ASSERT(!(type & IKP_CustomDataBit));
		if (ikparam._mapCustomData.size() > 0) {
			type |= IKP_CustomDataBit;
		}
		O << type << " ";
		switch (ikparam._type & ~IKP_VelocityDataBit) {
		case IKP_Transform6D:
			O << ikparam.GetTransform6D();
			break;
		case IKP_Rotation3D:
			O << ikparam.GetRotation3D();
			break;
		case IKP_Translation3D: {
			Vector v = ikparam.GetTranslation3D();
			O << v.x << " " << v.y << " " << v.z << " ";
			break;
		}
		case IKP_Direction3D: {
			Vector v = ikparam.GetDirection3D();
			O << v.x << " " << v.y << " " << v.z << " ";
			break;
		}
		case IKP_Ray4D: {
			RAY r = ikparam.GetRay4D();
			O << r.dir.x << " " << r.dir.y << " " << r.dir.z << " " << r.pos.x << " " << r.pos.y << " " << r.pos.z << " ";
			break;
		}
		case IKP_Lookat3D: {
			Vector v = ikparam.GetLookat3D();
			O << v.x << " " << v.y << " " << v.z << " ";
			break;
		}
		case IKP_TranslationDirection5D: {
			RAY r = ikparam.GetTranslationDirection5D();
			O << r.dir.x << " " << r.dir.y << " " << r.dir.z << " " << r.pos.x << " " << r.pos.y << " " << r.pos.z << " ";
			break;
		}
		case IKP_TranslationXY2D: {
			Vector v = ikparam.GetTranslationXY2D();
			O << v.x << " " << v.y << " ";
			break;
		}
		case IKP_TranslationXYOrientation3D: {
			Vector v = ikparam.GetTranslationXYOrientation3D();
			O << v.x << " " << v.y << " " << v.z << " ";
			break;
		}
		case IKP_TranslationLocalGlobal6D: {
			std::pair<Vector, Vector> p = ikparam.GetTranslationLocalGlobal6D();
			O << p.first.x << " " << p.first.y << " " << p.first.z << " " << p.second.x << " " << p.second.y << " " << p.second.z << " ";
			break;
		}
		case IKP_TranslationXAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationXAxisAngle4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		case IKP_TranslationYAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationYAxisAngle4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		case IKP_TranslationZAxisAngle4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationZAxisAngle4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		case IKP_TranslationXAxisAngleZNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationXAxisAngleZNorm4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		case IKP_TranslationYAxisAngleXNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationYAxisAngleXNorm4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		case IKP_TranslationZAxisAngleYNorm4D: {
			std::pair<Vector, dReal> p = ikparam.GetTranslationZAxisAngleYNorm4D();
			O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
			break;
		}
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(_("does not support parameterization 0x%x"), ikparam.GetType(), ORE_InvalidArguments);
		}
		if (ikparam._mapCustomData.size() > 0) {
			O << ikparam._mapCustomData.size() << " ";
			FOREACHC(it, ikparam._mapCustomData) {
				O << it->first << " " << it->second.size() << " ";
				FOREACHC(itvalue, it->second) {
					O << *itvalue << " ";
				}
			}
		}
		return O;
	}

	std::istream& operator>>(std::istream& I, IkParameterization& ikparam)
	{
		int type = IKP_None;
		I >> type;
		ikparam._type = static_cast<IkParameterizationType>(type&~IKP_CustomDataBit);
		switch (ikparam._type) {
		case IKP_Transform6D: {
			Transform t; I >> t;
			ikparam.SetTransform6D(t);
			break;
		}
		case IKP_Transform6DVelocity:
			I >> ikparam._transform;
			break;
		case IKP_Rotation3D: { Vector v; I >> v; ikparam.SetRotation3D(v); break; }
		case IKP_Rotation3DVelocity:
			I >> ikparam._transform.rot;
			break;
		case IKP_Translation3D: {
			Vector v;
			I >> v.x >> v.y >> v.z;
			ikparam.SetTranslation3D(v);
			break;
		}
		case IKP_Lookat3DVelocity:
		case IKP_Translation3DVelocity:
		case IKP_TranslationXYOrientation3DVelocity:
			I >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
			break;
		case IKP_Direction3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetDirection3D(v); break; }
		case IKP_Direction3DVelocity:
			I >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z;
			break;
		case IKP_Ray4D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetRay4D(r); break; }
		case IKP_Ray4DVelocity:
		case IKP_TranslationDirection5DVelocity:
			I >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z;
			break;
		case IKP_Lookat3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetLookat3D(v); break; }
		case IKP_TranslationDirection5D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetTranslationDirection5D(r); break; }
		case IKP_TranslationXY2D: { Vector v; I >> v.y >> v.y; ikparam.SetTranslationXY2D(v); break; }
		case IKP_TranslationXY2DVelocity:
			I >> ikparam._transform.trans.x >> ikparam._transform.trans.y;
			break;
		case IKP_TranslationXYOrientation3D: { Vector v; I >> v.y >> v.y >> v.z; ikparam.SetTranslationXYOrientation3D(v); break; }
		case IKP_TranslationLocalGlobal6D: { Vector localtrans, trans; I >> localtrans.x >> localtrans.y >> localtrans.z >> trans.x >> trans.y >> trans.z; ikparam.SetTranslationLocalGlobal6D(localtrans, trans); break; }
		case IKP_TranslationLocalGlobal6DVelocity:
			I >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
			break;
		case IKP_TranslationXAxisAngle4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationXAxisAngle4D(trans, angle);
			break;
		}
		case IKP_TranslationYAxisAngle4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationYAxisAngle4D(trans, angle);
			break;
		}
		case IKP_TranslationZAxisAngle4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationZAxisAngle4D(trans, angle);
			break;
		}
		case IKP_TranslationXAxisAngleZNorm4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationXAxisAngleZNorm4D(trans, angle);
			break;
		}
		case IKP_TranslationYAxisAngleXNorm4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationYAxisAngleXNorm4D(trans, angle);
			break;
		}
		case IKP_TranslationZAxisAngleYNorm4D: {
			Vector trans; dReal angle = 0;
			I >> angle >> trans.x >> trans.y >> trans.z;
			ikparam.SetTranslationZAxisAngleYNorm4D(trans, angle);
			break;
		}
		case IKP_TranslationXAxisAngle4DVelocity:
		case IKP_TranslationYAxisAngle4DVelocity:
		case IKP_TranslationZAxisAngle4DVelocity:
		case IKP_TranslationXAxisAngleZNorm4DVelocity:
		case IKP_TranslationYAxisAngleXNorm4DVelocity:
		case IKP_TranslationZAxisAngleYNorm4DVelocity:
			I >> ikparam._transform.rot.x >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
			break;
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(_("does not support parameterization 0x%x"), ikparam.GetType(), ORE_InvalidArguments);
		}
		ikparam._mapCustomData.clear();
		if (type & IKP_CustomDataBit) {
			size_t numcustom = 0, numvalues = 0;
			std::string name;
			I >> numcustom;
			if (!I) {
				return I;
			}
			for (size_t i = 0; i < numcustom; ++i) {
				I >> name >> numvalues;
				if (!I) {
					return I;
				}
				std::vector<dReal>& v = ikparam._mapCustomData[name];
				v.resize(numvalues);
				for (size_t j = 0; j < v.size(); ++j) {
					I >> v[j];
				}
			}
		}
		return I;
	}

	const std::map<IkParameterizationType, std::string>& IkParameterization::GetIkParameterizationMap(int alllowercase)
	{
		return RaveGlobal::instance()->GetIkParameterizationMap(alllowercase);
	}

}
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
#include <openrave/rave_global.h>
#include <openrave/openravejson.h>

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
		throw OPENRAVE_EXCEPTION_FORMAT(_tr("no ik exists of unique id 0x%x"), uniqueid, ORE_InvalidArguments);
	}

	ConfigurationSpecification IkParameterization::GetConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation, const std::string& robotname, const std::string& manipname)
	{
		ConfigurationSpecification spec;
		spec.groups_vector_.resize(1);
		spec.groups_vector_[0].offset = 0;
		spec.groups_vector_[0].dof = IkParameterization::GetNumberOfValues(iktype);
		spec.groups_vector_[0].name = str(boost::format("ikparam_values %d") % iktype);
		if (robotname.size() > 0) {
			spec.groups_vector_[0].name += robotname;
			spec.groups_vector_[0].name += " ";
			if (manipname.size() > 0) {
				spec.groups_vector_[0].name += manipname;
			}
		}
		spec.groups_vector_[0].interpolation = interpolation;

		// remove any trailing whitespace from missing robot or manipulator names
		boost::algorithm::trim(spec.groups_vector_[0].name);
		return spec;
	}

	std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam)
	{
		int type = ikparam.type_;
		BOOST_ASSERT(!(type & IKP_CustomDataBit));
		if (ikparam.custom_data_map_.size() > 0) {
			type |= IKP_CustomDataBit;
		}
		O << type << " ";
		switch (ikparam.type_ & ~IKP_VelocityDataBit) {
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
			throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support parameterization 0x%x"), ikparam.GetType(), ORE_InvalidArguments);
		}
		if (ikparam.custom_data_map_.size() > 0) {
			O << ikparam.custom_data_map_.size() << " ";
			FOREACHC(it, ikparam.custom_data_map_) {
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
		ikparam.type_ = static_cast<IkParameterizationType>(type&~IKP_CustomDataBit);
		switch (ikparam.type_) {
		case IKP_Transform6D: {
			Transform t; I >> t;
			ikparam.SetTransform6D(t);
			break;
		}
		case IKP_Transform6DVelocity:
			I >> ikparam.transform_;
			break;
		case IKP_Rotation3D: { Vector v; I >> v; ikparam.SetRotation3D(v); break; }
		case IKP_Rotation3DVelocity:
			I >> ikparam.transform_.rot;
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
			I >> ikparam.transform_.trans.x >> ikparam.transform_.trans.y >> ikparam.transform_.trans.z;
			break;
		case IKP_Direction3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetDirection3D(v); break; }
		case IKP_Direction3DVelocity:
			I >> ikparam.transform_.rot.x >> ikparam.transform_.rot.y >> ikparam.transform_.rot.z;
			break;
		case IKP_Ray4D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetRay4D(r); break; }
		case IKP_Ray4DVelocity:
		case IKP_TranslationDirection5DVelocity:
			I >> ikparam.transform_.trans.x >> ikparam.transform_.trans.y >> ikparam.transform_.trans.z >> ikparam.transform_.rot.x >> ikparam.transform_.rot.y >> ikparam.transform_.rot.z;
			break;
		case IKP_Lookat3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetLookat3D(v); break; }
		case IKP_TranslationDirection5D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetTranslationDirection5D(r); break; }
		case IKP_TranslationXY2D: { Vector v; I >> v.y >> v.y; ikparam.SetTranslationXY2D(v); break; }
		case IKP_TranslationXY2DVelocity:
			I >> ikparam.transform_.trans.x >> ikparam.transform_.trans.y;
			break;
		case IKP_TranslationXYOrientation3D: { Vector v; I >> v.y >> v.y >> v.z; ikparam.SetTranslationXYOrientation3D(v); break; }
		case IKP_TranslationLocalGlobal6D: { Vector localtrans, trans; I >> localtrans.x >> localtrans.y >> localtrans.z >> trans.x >> trans.y >> trans.z; ikparam.SetTranslationLocalGlobal6D(localtrans, trans); break; }
		case IKP_TranslationLocalGlobal6DVelocity:
			I >> ikparam.transform_.rot.x >> ikparam.transform_.rot.y >> ikparam.transform_.rot.z >> ikparam.transform_.trans.x >> ikparam.transform_.trans.y >> ikparam.transform_.trans.z;
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
			I >> ikparam.transform_.rot.x >> ikparam.transform_.trans.x >> ikparam.transform_.trans.y >> ikparam.transform_.trans.z;
			break;
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support parameterization 0x%x"), ikparam.GetType(), ORE_InvalidArguments);
		}
		ikparam.custom_data_map_.clear();
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
				std::vector<dReal>& v = ikparam.custom_data_map_[name];
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


	void IkParameterization::SerializeJSON(rapidjson::Value& rIkParameterization,
		rapidjson::Document::AllocatorType& alloc, dReal fUnitScale) const
	{
		rIkParameterization.SetObject();
		openravejson::SetJsonValueByKey(rIkParameterization, "type", GetName(), alloc);
		switch (type_) {
		case IKP_Transform6D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_Rotation3D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			break;
		case IKP_Translation3D:
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_Direction3D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			break;
		case IKP_Ray4D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_Lookat3D:
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_TranslationDirection5D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_TranslationXY2D:
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_TranslationXYOrientation3D:
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_TranslationLocalGlobal6D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		case IKP_TranslationXAxisAngle4D:
		case IKP_TranslationYAxisAngle4D:
		case IKP_TranslationZAxisAngle4D:
		case IKP_TranslationXAxisAngleZNorm4D:
		case IKP_TranslationYAxisAngleXNorm4D:
		case IKP_TranslationZAxisAngleYNorm4D:
			openravejson::SetJsonValueByKey(rIkParameterization, "rotate", transform_.rot, alloc);
			openravejson::SetJsonValueByKey(rIkParameterization, "translate", transform_.trans*fUnitScale, alloc);
			break;
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support parameterization %s"), GetName(), ORE_InvalidArguments);
		}
		if (custom_data_map_.size() > 0) {
			// TODO have to scale custom_data_map_ by fUnitScale
			openravejson::SetJsonValueByKey(rIkParameterization, "customData", custom_data_map_, alloc);
		}
	}

	void IkParameterization::DeserializeJSON(const rapidjson::Value& rIkParameterization, dReal fUnitScale)
	{
		if (!rIkParameterization.IsObject()) {
			throw openravejson::OpenRAVEJSONException("Cannot load value of non-object to IkParameterization.", openravejson::ORJE_InvalidArguments);
		}
		type_ = IKP_None;
		if (rIkParameterization.HasMember("type")) {
			const char* ptype = rIkParameterization["type"].GetString();
			if (!!ptype) {
				const std::map<IkParameterizationType, std::string>::const_iterator itend = RaveGetIkParameterizationMap().end();
				for (std::map<IkParameterizationType, std::string>::const_iterator it = RaveGetIkParameterizationMap().begin(); it != itend; ++it) {
					if (strcmp(ptype, it->second.c_str()) == 0) {
						type_ = it->first;
						break;
					}
				}
			}
		}
		switch (type_) {
		case IKP_Transform6D:
		case IKP_Transform6DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_Rotation3D:
		case IKP_Rotation3DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			break;
		case IKP_Translation3D:
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_Translation3DVelocity:
		case IKP_TranslationXYOrientation3DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_Direction3D:
		case IKP_Direction3DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			break;
		case IKP_Ray4D:
		case IKP_Ray4DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_TranslationDirection5D:
		case IKP_TranslationDirection5DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_Lookat3D:
		case IKP_Lookat3DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_TranslationXY2D:
		case IKP_TranslationXY2DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_TranslationXYOrientation3D:
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_TranslationLocalGlobal6D:
		case IKP_TranslationLocalGlobal6DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		case IKP_TranslationXAxisAngle4D:
		case IKP_TranslationXAxisAngle4DVelocity:
		case IKP_TranslationYAxisAngle4D:
		case IKP_TranslationYAxisAngle4DVelocity:
		case IKP_TranslationZAxisAngle4D:
		case IKP_TranslationZAxisAngle4DVelocity:
		case IKP_TranslationXAxisAngleZNorm4D:
		case IKP_TranslationXAxisAngleZNorm4DVelocity:
		case IKP_TranslationYAxisAngleXNorm4D:
		case IKP_TranslationYAxisAngleXNorm4DVelocity:
		case IKP_TranslationZAxisAngleYNorm4D:
		case IKP_TranslationZAxisAngleYNorm4DVelocity:
			openravejson::LoadJsonValueByKey(rIkParameterization, "rotate", transform_.rot);
			openravejson::LoadJsonValueByKey(rIkParameterization, "translate", transform_.trans);
			break;
		default:
			throw OPENRAVE_EXCEPTION_FORMAT(_tr("does not support parameterization 0x%x"), type_, ORE_InvalidArguments);
		}
		transform_.trans *= fUnitScale;

		custom_data_map_.clear();
		openravejson::LoadJsonValueByKey(rIkParameterization, "customData", custom_data_map_);
		// TODO have to scale custom_data_map_ by fUnitScale
	}


}
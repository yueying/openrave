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
#include <openrave/electric_motor_actuator_info.h>
#include <openrave/openravejson.h>

namespace OpenRAVE
{

	ElectricMotorActuatorInfo::ElectricMotorActuatorInfo()
	{
		gear_ratio = 0;
		assigned_power_rating = 0;
		max_speed = 0;
		no_load_speed = 0;
		stall_torque = 0;
		max_instantaneous_torque = 0;
		nominal_torque = 0;
		rotor_inertia = 0;
		torque_constant = 0;
		nominal_voltage = 0;
		speed_constant = 0;
		starting_current = 0;
		terminal_resistance = 0;
		coloumb_friction = 0;
		viscous_friction = 0;
	}


	void ElectricMotorActuatorInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
	{
		openravejson::SetJsonValueByKey(value, "modelType", model_type, allocator);
		openravejson::SetJsonValueByKey(value, "assignedPowerRating", assigned_power_rating, allocator);
		openravejson::SetJsonValueByKey(value, "maxSpeed", max_speed, allocator);
		openravejson::SetJsonValueByKey(value, "noLoadSpeed", no_load_speed, allocator);
		openravejson::SetJsonValueByKey(value, "stallTorque", stall_torque, allocator);
		openravejson::SetJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque, allocator);
		openravejson::SetJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points, allocator);
		openravejson::SetJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points, allocator);
		openravejson::SetJsonValueByKey(value, "nominalTorque", nominal_torque, allocator);
		openravejson::SetJsonValueByKey(value, "rotorInertia", rotor_inertia, allocator);
		openravejson::SetJsonValueByKey(value, "torqueConstant", torque_constant, allocator);
		openravejson::SetJsonValueByKey(value, "nominalVoltage", nominal_voltage, allocator);
		openravejson::SetJsonValueByKey(value, "speedConstant", speed_constant, allocator);
		openravejson::SetJsonValueByKey(value, "startingCurrent", starting_current, allocator);
		openravejson::SetJsonValueByKey(value, "terminalResistance", terminal_resistance, allocator);
		openravejson::SetJsonValueByKey(value, "gearRatio", gear_ratio, allocator);
		openravejson::SetJsonValueByKey(value, "coloumbFriction", coloumb_friction, allocator);
		openravejson::SetJsonValueByKey(value, "viscousFriction", viscous_friction, allocator);
	}

	void ElectricMotorActuatorInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale)
	{
		openravejson::LoadJsonValueByKey(value, "modelType", model_type);
		openravejson::LoadJsonValueByKey(value, "assignedPowerRating", assigned_power_rating);
		openravejson::LoadJsonValueByKey(value, "maxSpeed", max_speed);
		openravejson::LoadJsonValueByKey(value, "noLoadSpeed", no_load_speed);
		openravejson::LoadJsonValueByKey(value, "stallTorque", stall_torque);
		openravejson::LoadJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque);
		openravejson::LoadJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points);
		openravejson::LoadJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points);
		openravejson::LoadJsonValueByKey(value, "nominalTorque", nominal_torque);
		openravejson::LoadJsonValueByKey(value, "rotorInertia", rotor_inertia);
		openravejson::LoadJsonValueByKey(value, "torqueConstant", torque_constant);
		openravejson::LoadJsonValueByKey(value, "nominalVoltage", nominal_voltage);
		openravejson::LoadJsonValueByKey(value, "speedConstant", speed_constant);
		openravejson::LoadJsonValueByKey(value, "startingCurrent", starting_current);
		openravejson::LoadJsonValueByKey(value, "terminalResistance", terminal_resistance);
		openravejson::LoadJsonValueByKey(value, "gearRatio", gear_ratio);
		openravejson::LoadJsonValueByKey(value, "coloumbFriction", coloumb_friction);
		openravejson::LoadJsonValueByKey(value, "viscousFriction", viscous_friction);
	}

}
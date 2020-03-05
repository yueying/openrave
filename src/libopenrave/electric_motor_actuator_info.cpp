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


	void ElectricMotorActuatorInfo::SerializeJSON(rapidjson::Value& value,
		rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
	{
		SetJsonValueByKey(value, "modelType", model_type, allocator);
		SetJsonValueByKey(value, "assignedPowerRating", assigned_power_rating, allocator);
		SetJsonValueByKey(value, "maxSpeed", max_speed, allocator);
		SetJsonValueByKey(value, "noLoadSpeed", no_load_speed, allocator);
		SetJsonValueByKey(value, "stallTorque", stall_torque, allocator);
		SetJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque, allocator);
		SetJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points, allocator);
		SetJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points, allocator);
		SetJsonValueByKey(value, "nominalTorque", nominal_torque, allocator);
		SetJsonValueByKey(value, "rotorInertia", rotor_inertia, allocator);
		SetJsonValueByKey(value, "torqueConstant", torque_constant, allocator);
		SetJsonValueByKey(value, "nominalVoltage", nominal_voltage, allocator);
		SetJsonValueByKey(value, "speedConstant", speed_constant, allocator);
		SetJsonValueByKey(value, "startingCurrent", starting_current, allocator);
		SetJsonValueByKey(value, "terminalResistance", terminal_resistance, allocator);
		SetJsonValueByKey(value, "gearRatio", gear_ratio, allocator);
		SetJsonValueByKey(value, "coloumbFriction", coloumb_friction, allocator);
		SetJsonValueByKey(value, "viscousFriction", viscous_friction, allocator);
	}

	void ElectricMotorActuatorInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale)
	{
		LoadJsonValueByKey(value, "modelType", model_type);
		LoadJsonValueByKey(value, "assignedPowerRating", assigned_power_rating);
		LoadJsonValueByKey(value, "maxSpeed", max_speed);
		LoadJsonValueByKey(value, "noLoadSpeed", no_load_speed);
		LoadJsonValueByKey(value, "stallTorque", stall_torque);
		LoadJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque);
		LoadJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points);
		LoadJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points);
		LoadJsonValueByKey(value, "nominalTorque", nominal_torque);
		LoadJsonValueByKey(value, "rotorInertia", rotor_inertia);
		LoadJsonValueByKey(value, "torqueConstant", torque_constant);
		LoadJsonValueByKey(value, "nominalVoltage", nominal_voltage);
		LoadJsonValueByKey(value, "speedConstant", speed_constant);
		LoadJsonValueByKey(value, "startingCurrent", starting_current);
		LoadJsonValueByKey(value, "terminalResistance", terminal_resistance);
		LoadJsonValueByKey(value, "gearRatio", gear_ratio);
		LoadJsonValueByKey(value, "coloumbFriction", coloumb_friction);
		LoadJsonValueByKey(value, "viscousFriction", viscous_friction);
	}

}
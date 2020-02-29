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
}
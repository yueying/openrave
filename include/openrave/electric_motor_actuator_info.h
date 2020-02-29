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
#ifndef OPENRAVE_ELECTRIC_MOTOR_ACTUATOR_INFO_H_
#define OPENRAVE_ELECTRIC_MOTOR_ACTUATOR_INFO_H_

#include <openrave/numerical.h>

namespace OpenRAVE
{
	/// \brief holds parameters for an electric motor
	///
	/// all speed is in revolutions/second
	class OPENRAVE_API ElectricMotorActuatorInfo
	{
	public:
		ElectricMotorActuatorInfo();

		std::string model_type; ///< the type of actuator it is. Usually the motor model name is ok, but can include other info like gear box, etc
		//@{ from motor data sheet
		dReal assigned_power_rating; ///< the nominal power the electric motor can safely produce. Units are **Mass * Distance² * Time-³**
		dReal max_speed; ///< the maximum speed of the motor **Time-¹**
		dReal no_load_speed; ///< specifies the speed of the motor powered by the nominal voltage when the motor provides zero torque. Units are **Time-¹**.
		dReal stall_torque; ///< the maximum torque achievable by the motor at the nominal voltage. This torque is achieved at zero velocity (stall). Units are **Mass * Distance * Time-²**.
		dReal max_instantaneous_torque; ///< the maximum instantenous torque achievable by the motor when voltage <= nominal voltage. Motor going between nominal_torque and max_instantaneous_torque can overheat, so should not be driven at it for a long time. Units are **Mass * Distance * Time-²**.
		std::vector<std::pair<dReal, dReal> > nominal_speed_torque_points; ///< the speed and torque achievable when the motor is powered by the nominal voltage. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the stall torque directly (ideal). Should be ordered from increasing speed.
		std::vector<std::pair<dReal, dReal> > max_speed_torque_points; ///< the speed and torque achievable when the motor is powered by the max voltage/current. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the max_instantaneous_torque directly (ideal). Should be ordered from increasing speed.
		dReal nominal_torque; ///< the maximum torque the motor can provide continuously without overheating. Units are **Mass * Distance * Time-²**.
		dReal rotor_inertia; ///< the inertia of the rotating element about the axis of rotation. Units are **Mass * Distance²**.
		dReal torque_constant; ///< specifies the proportion relating current to torque. Units are **Mass * Distance * Time-¹ * Charge-¹**.
		dReal nominal_voltage; ///< the nominal voltage the electric motor can safely produce. Units are **Mass * Distance² * Time-² * Charge**.
		dReal speed_constant; ///< the constant of proportionality relating speed to voltage. Units are **Mass-¹ * Distance-² * Time * Charge-¹**.
		dReal starting_current; ///< specifies the current through the motor at zero velocity, equal to the nominal voltage divided by the terminal resistance. Also called the stall current.  Units are **Time-¹ * Charge**.
		dReal terminal_resistance; ///< the resistance of the motor windings. Units are **Mass * Distance² * Time-¹ * Charge-²**.
		//@}

		//@{ depending on gear box
		dReal gear_ratio; ///< specifies the ratio between the input speed of the transmission (the speed of the motor shaft) and the output speed of the transmission.
		dReal coloumb_friction; ///< static coloumb friction on each joint after the gear box. Units are **Mass * Distance * Time-²**.
		dReal viscous_friction; ///< viscous friction on each joint after the gear box. Units are **Mass * Distance * Time-²**.
		//@}
	};

	typedef std::shared_ptr<ElectricMotorActuatorInfo> ElectricMotorActuatorInfoPtr;
}

#endif // OPENRAVE_ELECTRIC_MOTOR_ACTUATOR_INFO_H_
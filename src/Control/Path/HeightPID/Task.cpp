//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Marcus Frølich                                                   *
//***************************************************************************

//***************************************************************************
// Pro-active PID controller
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// multiple #defines and the function initiateValues(...)
#include "../../../Plan/Generator_DubinsPath/Task.hpp"

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
namespace Path
{
namespace HeightPID
{
using DUNE_NAMESPACES;

//! Vector for System Mapping.
typedef std::vector<uint32_t> Systems;
//! Vector for Entity Mapping.
typedef std::vector<uint32_t> Entities;

// Tuning parameters used in the PID-controller
struct Tuning
{
	double K ;
	double Td;
	double Ti;
};

struct Arguments
{
	// Tuning values for approach phase
	Tuning appr;
	// Tuning values for descend phase
	Tuning desc;
	// To use anti windup, or not to use anti windup? That question is now obsolete I believe...
	bool AntiWindup;
	// Maximum value for the integral in PID
	double antiwindup_max;
	// Minimum climb rate
	double min_c_r;
	// Maximum climb rate
	double max_c_r;
	// Lookahead time for the proactive part of the PID
	double lookahead;
	// Time before hitting net to send brake
	double breakTimer;
};

struct Task: public Tasks::Periodic
{
	Arguments m_args;
	IMC::DesiredVerticalRate m_vrate;

	//! List of systems and entities allowed to pass EstimatedState.
	Systems m_state_filtered_sys;
	Entities m_state_filtered_ent;

	double net_hor, R;
	double WP[2][5], circles[2][4];
	double x, altitude, ground_speed;
	double land_lat, land_lon, land_heading;
	double alt_des;
	bool init_done;

	double prev_time;
	double dt;

	double prev_error;
	double integral;
	double derivative;

	int curWP;
	bool curWPFirst;

	bool plan_dyn_received;

	double height_offset;

	double climb_rate_lowpass;

	Task(const std::string& name, Tasks::Context& ctx):
		Tasks::Periodic(name, ctx),
		ground_speed(0.0),

		alt_des(100),
		init_done(false),

		prev_time(0.0),
		dt(0.0),

		prev_error(0),
		integral(0),
		derivative(0.0),

		curWP(0),
		curWPFirst(true),

		plan_dyn_received(false),

		height_offset(0.0),

		climb_rate_lowpass(0.0)
	{
		paramActive(Tasks::Parameter::SCOPE_MANEUVER,
				Tasks::Parameter::VISIBILITY_USER);

		param("Approach -- PID Gain Prop", m_args.appr.K)
				.defaultValue("0.55")
				.description("Proportional gain, approach phase");

		param("Approach -- PID Gain Int", m_args.appr.Ti)
				.defaultValue("0.30")
				.description("Integral gain, approach phase");

		param("Approach -- PID Gain Der", m_args.appr.Td)
				.defaultValue("0.60")
				.description("Derivative gain, approach phase");

		param("Descend -- PID Gain Prop", m_args.desc.K)
				.defaultValue("0.55")
				.description("Proportional gain, descend/FA phase");

		param("Descend -- PID Gain Int", m_args.desc.Ti)
				.defaultValue("0.30")
				.description("Integral gain, descend/FA phase");

		param("Descend -- PID Gain Der", m_args.desc.Td)
				.defaultValue("0.60")
				.description("Derivative gain, descend/FA phase");

		param("PID Anti-Windup", m_args.AntiWindup)
				.defaultValue("True")
				.description("Anti-windup");

		param("Anti Windup Max Climb Rate", m_args.antiwindup_max)
			.units(Units::MeterPerSecond)
			.minimumValue("0.0")
			.maximumValue("10.0")
			.defaultValue("1.5")
			.description("Maximum climb rate allowed for the integrator in PID");

		param("Climb Rate Min", m_args.min_c_r)
				.units(Units::MeterPerSecond)
				.minimumValue("-5.0")
				.maximumValue("0.0")
				.defaultValue("-3.0")
				.description("Minimum climb rate (negative)");

		param("Climb Rate Max", m_args.max_c_r)
				.units(Units::MeterPerSecond)
				.minimumValue("0.0")
				.maximumValue("5")
				.defaultValue("1.5")
				.description("Maximum climb rate");

		param("PID Lookahead Time", m_args.lookahead)
				.units(Units::Second)
				.minimumValue("0.0")
				.maximumValue("10.0")
				.defaultValue("0.5")
				.description("PID Lookahead Time");

		param("Brake Timer", m_args.breakTimer)
				.units(Units::Second)
				.minimumValue("0.0")
				.maximumValue("10.0")
				.defaultValue("1.0")
				.description("Time before hitting net to send brake cmd");

		bind<IMC::EstimatedState>(this);
		bind<IMC::ControlLoops>(this);
		bind<IMC::DesiredPath>(this);
		bind<IMC::PlanGeneration>(this);
	}

	void
	onActivation(void)
	{
		// DELETE THIS CONTROLLOOP?
		// Activate vertical rate controller.
		IMC::ControlLoops cloops;
		cloops.enable = IMC::ControlLoops::CL_ENABLE;
		cloops.mask = IMC::CL_VERTICAL_RATE;
		dispatch(cloops);

		// Not necessary to set the frequency if the dispatch function is inside PID() instead of task()
		double freq = 8;
		setFrequency(freq);

		inf("Activated");
	}

	void
	onDeactivation(void)
	{
//		// Deactivate vertical rate controller.
//		IMC::ControlLoops cloops;
//		cloops.enable = IMC::ControlLoops::CL_DISABLE;
//		cloops.mask = IMC::CL_VERTICAL_RATE;
//		dispatch(cloops);

//		init_done = false;
//		net_hor = R = 0;
//		std::memset(WP, 0, sizeof(WP));
//		std::memset(circles, 0, sizeof(circles));
//		land_lat = land_lon = land_heading = 0;
//		integral = prev_error = 0;
//
//		curWPFirst = true;
//		curWP = 0;

		inf("Deactivated");
	}

	void
	consume(const IMC::ControlLoops* c_loops)
	{
		if (!isActive())
			return;

		if ((c_loops->enable == IMC::ControlLoops::CL_ENABLE) &&
				(c_loops->mask & IMC::CL_ALTITUDE))
		{
			// Activate vertical rate controller.
			IMC::ControlLoops cloops;
			cloops.enable = IMC::ControlLoops::CL_ENABLE;
			cloops.mask = IMC::CL_VERTICAL_RATE;
			dispatch(cloops);
		}
	}

	void
	consume(const IMC::DesiredPath* dp)
	{
		// May be changed with PathControlState.path_ref
		// Keeps track of what the current waypoint is
		if(resolveEntity(dp->getSourceEntity()) == "Goto Maneuver"){
			if(curWPFirst && std::abs(dp->end_z-WP[yy][WP_OLD_HEIGHT_CTRL]) < 0.01){
				// Need to start on the waypoint where this height controller is activated
				curWP = WP_OLD_HEIGHT_CTRL;
				curWPFirst = false;
			} else if(plan_dyn_received){
				plan_dyn_received = false;
				curWP--;
			} else if(std::abs(dp->end_z-WP[yy][curWP+1]) < 0.01){
				curWP++;
			}

			if (!isActive())
				return;

			inf("Current WP = %i",curWP);
		}
	}

	void
	consume(const IMC::PlanGeneration* pg)
	{
		if(pg->plan_id == "land_dynamic"){
			plan_dyn_received = true;
			Delay::wait(0.5);
			// Retrieves all the necessary values defining the current (landing) path
			while(!initiateValues(net_hor, R, WP, circles, land_lat, land_lon, land_heading))
				Delay::wait(0.1);
			inf("Received new land_heading = %f",Angles::degrees(land_heading));

			// Test for dynamic height
			height_offset = 0.5;
			inf("Height offset %f meters", height_offset);
		}else if(pg->plan_id == "land"){
			init_done = false;
			curWPFirst = true;
			curWP = 0;
			integral = prev_error = 0;

			// Retrieves all the necessary values defining the current (landing) path
			while(!initiateValues(net_hor, R, WP, circles, land_lat, land_lon, land_heading))
				Delay::wait(0.1);

			init_done = true;
			inf("Init is done, and PID may start");
		}
	}

	void
	onEntityResolution(void)
	{
		spew("Entity resolution.");
	}

	void
	consume(const IMC::EstimatedState* state)
	{
		// Filter EstimatedState by systems and entities.
		bool matched = true;
		if (m_state_filtered_sys.size() > 0)
		{
			matched = false;
			std::vector<uint32_t>::iterator itr_sys = m_state_filtered_sys.begin();
			std::vector<uint32_t>::iterator itr_ent = m_state_filtered_ent.begin();
			for (; itr_sys != m_state_filtered_sys.end(); ++itr_sys)
			{
				if ((*itr_sys == state->getSource() || *itr_sys == (unsigned int)UINT_MAX) &&
						(*itr_ent == state->getSourceEntity() || *itr_ent == (unsigned int)UINT_MAX))
					matched = true;
				++itr_ent;
			}
		}
		// This system and entity are not listed to be passed.
		if (!matched)
		{
			trace("EstimatedState rejected (from system '%s' and entity '%s')",
					resolveSystemId(state->getSource()),
					resolveEntity(state->getSourceEntity()).c_str());
			return;
		}

		altitude = state->height - state->z;
		ground_speed = sqrt(state->vx*state->vx + state->vy*state->vy);

		double cur_lat = state->lat;
		double cur_lon = state->lon;
		double off_x   = state->x;
		double off_y   = state->y;
		Coordinates::WGS84::displace(off_x,off_y,&cur_lat,&cur_lon);

		// Dispatch of GpsFix calculated from EstimatedState
		// is needed to easily plot the position with higher frequency than 1Hz
		IMC::GpsFix gps_fix;
		gps_fix.lat = cur_lat;
		gps_fix.lon	= cur_lon;
		gps_fix.height = altitude;
		dispatch(gps_fix);

		// Calculation of bearing and range from current position to the net
		double bearing, range;
		Coordinates::WGS84::getNEBearingAndRange(cur_lat,cur_lon,land_lat,land_lon,&bearing,&range);

		// Need to ensure negative distance to net if the UAV is "behind"/approaching
		if(land_heading < -M_PI/2)
			x = (bearing > land_heading+3*M_PI/2 || bearing < land_heading+M_PI/2) ? -range : range;
		else if(land_heading > -M_PI/2 && land_heading < M_PI/2)
			x = (bearing > land_heading-M_PI/2 && bearing < land_heading+M_PI/2) ? -range : range;
		else // (land_heading > M_PI/2)
			x = (bearing > land_heading-M_PI/2 || bearing < land_heading-3*M_PI/2) ? -range : range;

		// Calculates the desired altitude based on distance and lookahead time
		//alt_des	= func_alt_des(x+ground_speed*m_args.lookahead);
		alt_des	= func_alt_des(x) + height_offset;
		IMC::DesiredZ d_z;
		d_z.value = alt_des;
		d_z.z_units = IMC::Z_HEIGHT;
		dispatch(d_z);
		//inf("alt_des = %f",alt_des);

		// SENDING BRAKE
		if(curWP >= 4 && (-x/ground_speed) < m_args.breakTimer){
			IMC::Brake brake;
			brake.op = IMC::Brake::OP_START;
			dispatch(brake);
			inf("Brake sent");
		}

		if (!isActive())
			return;

		if (!init_done)
			return;

		PID();
	}

	void PID()
	{
		// Calculation of delta time (dt)
		double now = Clock().get();
		if(prev_time == 0)
			dt = 0;
		else
			dt = (now - prev_time);
		prev_time = now;

		if(dt < 0.001)
			return;

		double error                	= alt_des - altitude; // "altitude" needs to be relative to bottom of net

		if(prev_error != 0)
			derivative       			= (error - prev_error)/dt;

		// Used because the UAV reacts much more aggressively to positive climb rates
		double scale_factor				= 0.5;
		if(error > 0) integral			= integral + scale_factor*error*dt;
		else integral              		= integral + error*dt;

		// Anti windup
		if(curWP <= 1 && m_args.appr.Ti != 0){
			double anti_windup_max 		= m_args.antiwindup_max/(m_args.appr.K*m_args.appr.Ti);
			// Limits the impact of the integral term
			integral					= trimValue(integral,-anti_windup_max,anti_windup_max);
		} else if (m_args.desc.Ti != 0){
			double anti_windup_max 		= m_args.antiwindup_max/(m_args.desc.K*m_args.desc.Ti);
			// Limits the impact of the integral term
			integral					= trimValue(integral,-anti_windup_max,anti_windup_max);
		}

		// Proactive part of the PID
		double climb_rate_ratio_demanded 	= func_climb_rate_ratio_des(x+ground_speed*m_args.lookahead);
		double climb_rate_demanded_path		= ground_speed*climb_rate_ratio_demanded;

//		// TEST!!!!!!!!!!!!!!!!
//		m_vrate.value = climb_rate_demanded_path;
//		dispatch(m_vrate);

//
		double climb_rate_demanded  	= climb_rate_demanded_path + m_args.desc.K*(error + m_args.desc.Ti*integral + m_args.desc.Td*derivative);
		if(curWP <= 1) // Uses different tuning values when in approach phase
			climb_rate_demanded  		= climb_rate_demanded_path + m_args.appr.K*(error + m_args.appr.Ti*integral + m_args.appr.Td*derivative);
		prev_error              		= error;

		double climb_rate_demanded_sat;
		if(curWP <= 1 && error > 2) // Amplifies max_c_r if error > 2, meaning UAV is way down under
			climb_rate_demanded_sat 	= trimValue(climb_rate_demanded,m_args.min_c_r,0.5*(error-2)*m_args.max_c_r);
		else if(curWP <= 1)
			climb_rate_demanded_sat 	= trimValue(climb_rate_demanded,m_args.min_c_r,m_args.max_c_r);
		else // Extra tight upper bound on climb_rate_demanded_sat when descending
			climb_rate_demanded_sat 	= trimValue(climb_rate_demanded,m_args.min_c_r,climb_rate_demanded_path/1.0);

//		if(m_args.AntiWindup && m_args.Ti > 0 && (climb_rate_demanded_sat-climb_rate_demanded) != 0){
//			//if(std::abs(climb_rate_demanded - integral*m_args.Ti*m_args.K) <= std::abs(climb_rate_demanded_sat)){
//			//	integral 				-= 0.2*(climb_rate_demanded_sat - climb_rate_demanded + integral*m_args.Ti*m_args.K)/(m_args.K*m_args.Ti);
//			//} else
//			if(climb_rate_demanded_sat > 0 && integral > 0.8*climb_rate_demanded_sat/(m_args.K*m_args.Ti)){
//				integral = 0.8*climb_rate_demanded_sat/(m_args.K*m_args.Ti);
//			} else if(climb_rate_demanded_sat < 0 && integral < 0.8*climb_rate_demanded_sat/(m_args.K*m_args.Ti)){
//				integral = 0.8*climb_rate_demanded_sat/(m_args.K*m_args.Ti);
//			}
//		}

		double lp_degree = 0.9;
		if(climb_rate_lowpass - climb_rate_demanded_sat > 1.0)
			climb_rate_lowpass = 0.5*lp_degree*climb_rate_lowpass + (1-0.5*lp_degree)*climb_rate_demanded_sat;
		else
			climb_rate_lowpass = lp_degree*climb_rate_lowpass + (1-lp_degree)*climb_rate_demanded_sat;

		// Return climb_rate_demanded_sat
		m_vrate.value = climb_rate_lowpass;

		// Dispatch could be done in the periodic task
		dispatch(m_vrate);

//		if(curWP <= 1)
//			inf("m_vrate PID/integral_effect/der_effect/error: %f/%f/%f/%f", climb_rate_demanded_sat, integral*m_args.appr.Ti*m_args.appr.K, m_args.appr.K*m_args.appr.Td*derivative, error);
//		else
//			inf("m_vrate PID/integral_effect/der_effect/error: %f/%f/%f/%f", climb_rate_demanded_sat, integral*m_args.desc.Ti*m_args.desc.K, m_args.desc.K*m_args.desc.Td*derivative, error);
	}

	// Calculates desired altitude based on values from [Plan.Generator_DubinsPath]
	double func_alt_des(double in_x){
		double alt;
		if (curWP == 0){
			alt  		= WP[yy][0];
		} else if (in_x < circles[0][xx1]){
			alt  		= WP[yy][1];
		} else if (in_x <= circles[0][xx2]){
			alt  		= sqrt(R*R-(in_x-circles[0][xxc])*(in_x-circles[0][xxc])) + circles[0][yyc];
		} else if (in_x < circles[1][xx1]){
			double h    = WP[yy][2]-WP[yy][1];
			double vert = WP[xx][2]-WP[xx][1];
			alt  		= (h/vert)*(in_x-(WP[xx][1]-net_hor)) + WP[yy][1];
		} else if (in_x <= circles[1][xx2]){
			alt  		= -sqrt(R*R-(in_x-circles[1][xxc])*(in_x-circles[1][xxc])) + circles[1][yyc];
		} else{
			double h    = WP[yy][3]-WP[yy][2];
			double vert = WP[xx][3]-WP[xx][2];
			if(vert != 0){
				alt  	= (h/vert)*(in_x-(WP[xx][2]-net_hor)) + WP[yy][2];
			} else{
				alt 	= 100;
				inf("Waypoint error (vert==0), alt set to 100");
			}
		}

		return alt;
	}

	// Calculates desired climb rate ratio based on values from [Plan.Generator_DubinsPath]
	double func_climb_rate_ratio_des(double in_x){
		double climb_rate_ratio;

		if (in_x < circles[0][xx1] || curWP == 0)
			climb_rate_ratio = (WP[yy][1]-WP[yy][0])/(WP[xx][1]-WP[xx][0]);
//			climb_angle = atan2(WP[yy][1]-WP[yy][0],WP[xx][1]-WP[xx][0]);
		else if (in_x <= circles[0][xx2])
			climb_rate_ratio = -(in_x-circles[0][xxc])/sqrt(R*R-(in_x-circles[0][xxc])*(in_x-circles[0][xxc]));
		else if (in_x < circles[1][xx1])
			climb_rate_ratio = (WP[yy][2]-WP[yy][1])/(WP[xx][2]-WP[xx][1]);
//			climb_angle = atan2(WP[yy][2]-WP[yy][1],WP[xx][2]-WP[xx][1]);
		else if (in_x <= circles[1][xx2])
			climb_rate_ratio = (in_x-circles[1][xxc])/sqrt(R*R-(in_x-circles[1][xxc])*(in_x-circles[1][xxc]));
		else
			climb_rate_ratio = (WP[yy][3]-WP[yy][2])/(WP[xx][3]-WP[xx][2]);
//			climb_angle = atan2(WP[yy][3]-WP[yy][2],WP[xx][3]-WP[xx][2]);

		return climb_rate_ratio;
	}


	void
	task(void)
	{
		if (!isActive())
			return;

		// Dispatched in PID()
		//dispatch(m_vrate);
		//inf("m_vrate PID:\t%f", m_vrate.value);
	}
};
}
}
}

DUNE_TASK

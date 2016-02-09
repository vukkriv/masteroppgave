//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: José Pinto   					       						    *
// Edit:   Marcus Frølich                                                   *
//***************************************************************************

// ISO C++ 98 headers.
#include <stdexcept>

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Coordinates/WGS84.hpp>

using DUNE_NAMESPACES;

#include <cmath>
#include "Task.hpp"

double net_hor, R_dubins;
double WP[2][5], circles[2][4];
double land_lat, land_lon, land_heading;
bool init_done = false;
bool ignore_evasive = false;
IMC::MessageList<IMC::Maneuver> maneuvers;

namespace Plan
{
//! This task accepts and processes messages of type PlanGeneration,
//! resulting in the generation of corresponding plans (marcusf: only land).
//! The idea is simple: According to the plan identifier and an
//! optional set of parameters, this task generates a
//! PlanSpecification that is sent to the plan database and
//! (optionally) is also executed immediately (through a PlanControl
//! message).
//!
//! @author José Pinto
//!
//! Updated for use with net landing in Masters thesis
//! @author Marcus Frølich
namespace Generator_DubinsPath
{

struct Task: public DUNE::Tasks::Task
{
	//! Stores the last estimated state;
	IMC::EstimatedState* m_estate;

	//! Class constructor
	Task(const std::string& name, Tasks::Context& ctx):
		DUNE::Tasks::Task(name, ctx),
		m_estate(NULL)
	{

		bind<IMC::EstimatedState>(this);
		bind<IMC::PlanGeneration>(this);
	}

	//! Frees memory associated with stored messages.
	void
	onResourceRelease(void)
	{
		Memory::clear(m_estate);
	}

	void
	consume(const IMC::EstimatedState* estate)
	{
		m_estate = new IMC::EstimatedState(*estate);
	}

	//! Handles a PlanGeneration message. According to the 'plan_id' may result
	//! in the generation of different plans:
	void
	consume(const IMC::PlanGeneration* msg)
	{
		if (msg->op != IMC::PlanGeneration::OP_REQUEST)
			return;

		TupleList tlist(msg->params, "=", ";", true);

		// try to generate corresponding plan
		IMC::PlanSpecification spec;

		// if generation is not possible, send an error back.
		if (!generate(msg->plan_id, tlist, spec))
		{
			IMC::PlanGeneration response;
			response.cmd = msg->cmd;
			response.op = msg->IMC::PlanGeneration::OP_ERROR;
			response.plan_id = msg->plan_id;
			response.params = "error=Given plan id was not understood.";

			inf(DTR("Unable to generate plan using template %s"), msg->plan_id.c_str());
			dispatch(response);

			return;
		}

		if (msg->cmd == IMC::PlanGeneration::CMD_GENERATE || msg->cmd == IMC::PlanGeneration::CMD_EXECUTE)
		{
			IMC::PlanDB pdb;
			pdb.op = IMC::PlanDB::DBOP_SET;
			pdb.type = IMC::PlanDB::DBT_REQUEST;
			pdb.plan_id = spec.plan_id;
			pdb.arg.set(spec);
			pdb.request_id = 0;

			dispatch(pdb);
		}

		if (msg->cmd == IMC::PlanGeneration::CMD_EXECUTE)
		{
			IMC::PlanControl pcontrol;
			pcontrol.arg.set(spec);
			pcontrol.info = DTR("Plan generated automatically");

			if (tlist.get("calibrate") != "false")
				pcontrol.flags = IMC::PlanControl::FLG_CALIBRATE;

			if (tlist.get("ignore_errors") == "true")
				pcontrol.flags |= IMC::PlanControl::FLG_IGNORE_ERRORS;

			pcontrol.plan_id = spec.plan_id;
			pcontrol.request_id = 0;
			pcontrol.type = IMC::PlanControl::PC_REQUEST;
			pcontrol.op = IMC::PlanControl::PC_START;
			dispatch(pcontrol);

			inf("Plan starts automatically");
		}

		IMC::PlanGeneration response;
		response.cmd = msg->cmd;
		response.op = msg->IMC::PlanGeneration::OP_SUCCESS;
		response.plan_id = msg->plan_id;
		response.params = msg->params;
		dispatch(response);
	}

	void
	onMain(void)
	{
		while (!stopping())
		{
			waitForMessages(1.0);
		}
	}

	//! This (utility) method generates a PlanSpecification
	//! consisting in the given maneuver sequence.
	//! @param[in] plan_id The name of the plan to be generated
	//! @param[in] maneuvers A vector with maneuvers (order of the resulting plan will correspond
	//! to the order of this vector).
	//! @param[out] result The resulting PlanSpecification will be stored here.
	void
	sequentialPlan(std::string plan_id, const IMC::MessageList<IMC::Maneuver>* maneuvers, IMC::PlanSpecification& result)
	{

// marcusf START: Maneuver specifications to use custom height controller "Height Controller PID" -------------

		IMC::MessageList<IMC::Message> setEntityParameters;

		IMC::SetEntityParameters* sep = new IMC::SetEntityParameters();
		sep->name = "Path Control";
			IMC::MessageList<IMC::EntityParameter> entityParameters;
			IMC::EntityParameter* ep = new IMC::EntityParameter();
			ep->name = "Use controller";
			ep->value = "true";
			entityParameters.push_back(*ep);
			delete ep;
		sep->params = entityParameters;
		setEntityParameters.push_back(*sep);
		delete sep;

		sep = new IMC::SetEntityParameters();
		sep->name = "Height Controller PID";
			entityParameters.clear();
			ep = new IMC::EntityParameter();

			ep->name = "Active";
			if(plan_id == "land" || plan_id == "land_dynamic")
				ep->value = "true";
			else if(plan_id == "evasive")
				ep->value = "false";

			entityParameters.push_back(*ep);
			delete ep;
		sep->params = entityParameters;
		setEntityParameters.push_back(*sep);
		delete sep;

		sep = new IMC::SetEntityParameters();
		sep->name = "Height Control";
			entityParameters.clear();
			ep = new IMC::EntityParameter();

			ep->name = "Active";
			if(plan_id == "land" || plan_id == "land_dynamic")
				ep->value = "false";
			else if(plan_id == "evasive")
				ep->value = "true";

			entityParameters.push_back(*ep);
			delete ep;
		sep->params = entityParameters;
		setEntityParameters.push_back(*sep);
		delete sep;

		sep = new IMC::SetEntityParameters();
		sep->name = "Evasive";
			entityParameters.clear();
			ep = new IMC::EntityParameter();

			ep->name = "Active";
			if(plan_id == "land" || plan_id == "land_dynamic")
				ep->value = "true";
			else if(plan_id == "evasive")
				ep->value = "false";

			entityParameters.push_back(*ep);
			delete ep;
		sep->params = entityParameters;
		setEntityParameters.push_back(*sep);
		delete sep;

		sep = new IMC::SetEntityParameters();
		sep->name = "Autopilot";
			entityParameters.clear();
			ep = new IMC::EntityParameter();
			ep->name = "Ardupilot Tracker";
			ep->value = "false";
			entityParameters.push_back(*ep);
			delete ep;

			ep = new IMC::EntityParameter();
			ep->name = "Formation Flight";
			ep->value = "false";
			entityParameters.push_back(*ep);
			delete ep;
		sep->params = entityParameters;
		setEntityParameters.push_back(*sep);
		delete sep;

// BEGINING (2): Maneuver specifications to use the standard "Height Controller"

		IMC::MessageList<IMC::Message> setEntityParameters2;

				IMC::SetEntityParameters* sep2 = new IMC::SetEntityParameters();
				sep2->name = "Path Control";
					IMC::MessageList<IMC::EntityParameter> entityParameters2;
					IMC::EntityParameter* ep2 = new IMC::EntityParameter();
					ep2->name = "Use controller";
					ep2->value = "true";
					entityParameters2.push_back(*ep2);
					delete ep2;
				sep2->params = entityParameters2;
				setEntityParameters2.push_back(*sep2);
				delete sep2;

				sep2 = new IMC::SetEntityParameters();
				sep2->name = "Height Controller PID";
					entityParameters2.clear();
					ep2 = new IMC::EntityParameter();

					ep2->name = "Active";
					ep2->value = "false";

					entityParameters2.push_back(*ep2);
					delete ep2;
				sep2->params = entityParameters2;
				setEntityParameters2.push_back(*sep2);
				delete sep2;

				sep2 = new IMC::SetEntityParameters();
				sep2->name = "Height Control";
					entityParameters2.clear();
					ep2 = new IMC::EntityParameter();

					ep2->name = "Active";
					ep2->value = "true";

					entityParameters2.push_back(*ep2);
					delete ep2;
				sep2->params = entityParameters2;
				setEntityParameters2.push_back(*sep2);
				delete sep2;

				sep2 = new IMC::SetEntityParameters();
				sep2->name = "Evasive";
					entityParameters2.clear();
					ep2 = new IMC::EntityParameter();

					ep2->name = "Active";
					ep2->value = "true";

					entityParameters2.push_back(*ep2);
					delete ep2;
				sep2->params = entityParameters2;
				setEntityParameters2.push_back(*sep2);
				delete sep2;

				sep2 = new IMC::SetEntityParameters();
				sep2->name = "Autopilot";
					entityParameters2.clear();
					ep2 = new IMC::EntityParameter();
					ep2->name = "Ardupilot Tracker";
					ep2->value = "false";
					entityParameters2.push_back(*ep2);
					delete ep2;

					ep2 = new IMC::EntityParameter();
					ep2->name = "Formation Flight";
					ep2->value = "false";
					entityParameters2.push_back(*ep2);
					delete ep2;
				sep2->params = entityParameters2;
				setEntityParameters2.push_back(*sep2);
				delete sep2;

// END (2):

// marcusf END ---------------------------------------------------------------------------------------------------


		IMC::PlanManeuver last_man;

		IMC::MessageList<IMC::Maneuver>::const_iterator itr;
		unsigned i = 0;
		for (itr = maneuvers->begin(); itr != maneuvers->end(); itr++, i++)
		{
			if (*itr == NULL)
				continue;

			IMC::PlanManeuver man_spec;

			man_spec.data.set(*(*itr));
			man_spec.maneuver_id = String::str(i + 1);

			if(plan_id == "land" || plan_id == "land_dynamic" || plan_id == "evasive"){
				// Set the parameters specified above, with standard height controller to the WP_OLD_HEIGHT_CTRL first waypoints
				if(plan_id == "land" && i < WP_OLD_HEIGHT_CTRL)
					man_spec.start_actions = setEntityParameters2;
				else if(plan_id == "land_dynamic" && i+(5-maneuvers->size()) < WP_OLD_HEIGHT_CTRL)
					man_spec.start_actions = setEntityParameters2;
				else
					man_spec.start_actions = setEntityParameters;
			}

			if (itr == maneuvers->begin())
			{
				// no transitions.
			}
			else
			{
				IMC::PlanTransition trans;
				trans.conditions = "maneuverIsDone";
				trans.dest_man = man_spec.maneuver_id;
				trans.source_man = last_man.maneuver_id;

				result.transitions.push_back(trans);
			}

			result.maneuvers.push_back(man_spec);

			last_man = man_spec;
		}

		result.plan_id = plan_id;
		result.start_man_id = "1";
	}

	//! This method parses a string and a list of parameters and
	//! eventually generates a corresponding plan.
	//! @param[in] plan_id the string to be parsed (command).
	//! @param[in] params a tuple list with parameters to be used in
	//! the generation.
	//! @param[out] result where to store the resulting plan
	//! specification.
	//! @returns true if a plan was actually generated or false if
	//! result wasn't touched.degrees
	bool
	generate(const std::string& plan_id, TupleList& params, IMC::PlanSpecification& result)
	{
		result.plan_id = plan_id;
		//result.description = DTR("Plan generated automatically by DUNE.");

		inf(DTR("generating plan from '%s' template..."), plan_id.c_str());


// LANDING START -------------------------------------------------------------------------------------

		if (plan_id == "land")
		{

			init_done = false;

			inf("plan_id 'land' received");


			land_lat 	        	= Angles::radians(params.get("land_lat", 0.0));
			land_lon 	        	= Angles::radians(params.get("land_lon", 0.0));
			land_heading 			= Angles::radians(params.get("land_heading", 0.0));
			double net_height 		= params.get("net_height", 0.0);
			double min_turn_radius 	        = params.get("min_turn_radius", 0.0);
			double attack_angle 	        = Angles::radians(params.get("attack_angle", 0.0));
			double descend_angle 	        = Angles::radians(params.get("descend_angle", 0.0));
			if(attack_angle > descend_angle){
				inf("error: attack_angle greater then descend_angle");
				attack_angle		= descend_angle;
				inf("attack_angle sat equal to descend_angle (%f)", Angles::degrees(attack_angle));
			}

			inf("\n\nLan_lat:\t%f\nLand_lon:\t%f\nLand_heading:\t%f\n",Angles::degrees(land_lat),Angles::degrees(land_lon),Angles::degrees(land_heading));

			double dist_behind 		= params.get("dist_behind", 0.0);
			double dist_infront 	        = params.get("dist_infront", 0.0);
			double speed12	 		= params.get("speed12", 0.0);
			double speed345	 		= params.get("speed345", 0.0);
			double ground_level		= params.get("ground_level", 0.0); // Should be same as init value in JSBSim to avoid crash

	                int z_unit			= (params.get("z_unit") == "height") ? IMC::Z_HEIGHT : IMC::Z_ALTITUDE;

			ignore_evasive			= (params.get("ignore_evasive") == "true") ? true : false;

			R_dubins   			= 20*min_turn_radius; // Global (Dubins path radius)

			double height			= (m_estate != NULL) ? m_estate->height - m_estate->z : 100;
			double min_height		= MIN_DESCEND_HEIGHT + -dist_infront*tan(attack_angle) + ground_level;
			double original_height	        = height;

			if(height < min_height){
				height			= min_height;
			}

//			inf("land_heading\t%f",	Angles::degrees(land_heading));
//			inf("net_height\t%f",	net_height);
//			inf("min_turn_radius\t%f",min_turn_radius);
//			inf("attack_angle\t%f",	Angles::degrees(attack_angle));
//			inf("descend_angle\t%f",Angles::degrees(descend_angle));
//			inf("dist_behind\t%f",	dist_behind);
//			inf("dist_infront\t%f",	dist_infront);
//			inf("ground_level\t%f",	ground_level);


			// wp0 is behind the net, and just something to aim at. Will hopefully land in the net before reaching this wp
			double wp0_meter[3] = {dist_behind,0,net_height - dist_behind*tan(attack_angle)};

			// wp1 is in the middle of the net
			double wp1_meter[3] = {0,0,net_height};

			// wp2 is after descending from original UAV hight
			double wp2_meter[3] = {dist_infront,0,net_height - dist_infront*tan(attack_angle)};

			// wp3 is a function of descend_angle and hight difference from wp2 to ensure the
			// waypoint is far enough away from wp2 to make the UAV able to descend to
			// the height of wp2
			double wp3_meter[3] = {wp2_meter[0] - (height-wp2_meter[2])/tan(descend_angle) + ground_level/tan(descend_angle),0,height};

			// wp4 is an "adjustment waypoint" making sure the UAV is in line when reaching wp3
			double wp4_meter[3] = {wp3_meter[0] - 2*min_turn_radius,0,wp3_meter[2]};

			// Calculations from relative meter and heading from net, into waypoints in rad

			// wp0
			double wp0_lat  = land_lat;
			double wp0_lon  = land_lon;
			double wp0_h	= wp0_meter[2] + ground_level;
			Coordinates::WGS84::displace(wp0_meter[xx]*cos(land_heading) + wp0_meter[yy]*sin(land_heading),
					wp0_meter[xx]*sin(land_heading) + wp0_meter[yy]*cos(land_heading),
					&wp0_lat,&wp0_lon);

			// wp1
			double wp1_lat  = land_lat;
			double wp1_lon  = land_lon;
			double wp1_h	= wp1_meter[2] + ground_level;
			Coordinates::WGS84::displace(wp1_meter[xx]*cos(land_heading) + wp1_meter[yy]*sin(land_heading),
					wp1_meter[xx]*sin(land_heading) + wp1_meter[yy]*cos(land_heading),
					&wp1_lat,&wp1_lon);

			// wp2
			double wp2_lat  = land_lat;
			double wp2_lon  = land_lon;
			double wp2_h	= wp2_meter[2] + ground_level;
			Coordinates::WGS84::displace(wp2_meter[xx]*cos(land_heading) + wp2_meter[yy]*sin(land_heading),
					wp2_meter[xx]*sin(land_heading) + wp2_meter[yy]*cos(land_heading),
					&wp2_lat,&wp2_lon);

			// wp3
			double wp3_lat  = land_lat;
			double wp3_lon  = land_lon;
			double wp3_h	= wp3_meter[2];
			Coordinates::WGS84::displace(wp3_meter[xx]*cos(land_heading) + wp3_meter[yy]*sin(land_heading),
					wp3_meter[xx]*sin(land_heading) + wp3_meter[yy]*cos(land_heading),
					&wp3_lat,&wp3_lon);

			// wp4
			double wp4_lat  = land_lat;
			double wp4_lon  = land_lon;
			double wp4_h	= wp4_meter[2];
			Coordinates::WGS84::displace(wp4_meter[xx]*cos(land_heading) + wp4_meter[yy]*sin(land_heading),
					wp4_meter[xx]*sin(land_heading) + wp4_meter[yy]*cos(land_heading),
					&wp4_lat,&wp4_lon);

			// If UAV starts bellow minimum height needed, it may need additional distance to ascend
			if(original_height < min_height){
				double cur_lat = m_estate->lat;
				double cur_lon = m_estate->lon;
				Coordinates::WGS84::displace(m_estate->x,m_estate->y,&cur_lat,&cur_lon);

				double dist_wp4	= Coordinates::WGS84::distance(cur_lat, cur_lon, 0, wp4_lat, wp4_lon, 0);
				double extra_distance = 0.35*(min_height-original_height)/tan(descend_angle) - 0.3*dist_wp4;
				inf(" ");
				inf("dist_wp4 = %f",dist_wp4);
				inf("extra_distance = %f",extra_distance);
				inf("min_height-original_height = %f",min_height-original_height);
				inf("0.35*(min_height-original_height)/tan(descend_angle) = %f",0.35*(min_height-original_height)/tan(descend_angle));
				inf("0.3*dist_wp0 = %f",0.3*dist_wp4);
				inf(" ");
				if(extra_distance > 0){
					min_turn_radius	+= extra_distance;
					wp4_meter[0] = wp3_meter[0] - 2*min_turn_radius;
					wp4_lat  = land_lat;
					wp4_lon  = land_lon;
					Coordinates::WGS84::displace(wp4_meter[xx]*cos(land_heading) + wp4_meter[yy]*sin(land_heading),
							wp4_meter[xx]*sin(land_heading) + wp4_meter[yy]*cos(land_heading),
							&wp4_lat,&wp4_lon);
				}
			}

			if (land_lat != 0 && land_lon != 0)
			{
				maneuvers.clear();

				// 4
				IMC::Goto* go_near 	= new IMC::Goto();
				go_near->lat 		= wp4_lat;
				go_near->lon 		= wp4_lon;
				go_near->z   		= wp4_h;
				go_near->z_units 	= z_unit;
				go_near->speed 		= speed12;
				go_near->speed_units= IMC::SUNITS_METERS_PS;
				maneuvers.push_back(*go_near);
				inf("\n\nWP4:\nlat = %f \nlon = %f\nheight = %f \n", Angles::degrees(go_near->lat), Angles::degrees(go_near->lon), go_near->z);
				delete go_near;


				// 3
				go_near 		= new IMC::Goto();
				go_near->lat 		= wp3_lat;
				go_near->lon 		= wp3_lon;
				go_near->z   		= wp3_h;
				go_near->z_units 	= z_unit;
				go_near->speed 		= speed12;
				go_near->speed_units= IMC::SUNITS_METERS_PS;
				maneuvers.push_back(*go_near);
				inf("\n\nWP3:\nlat = %f \nlon = %f\nheight = %f \n", Angles::degrees(go_near->lat), Angles::degrees(go_near->lon), go_near->z);
				delete go_near;

				// 2
				go_near 		= new IMC::Goto();
				go_near->lat 		= wp2_lat;
				go_near->lon 		= wp2_lon;
				go_near->z   		= wp2_h;
				go_near->z_units 	= z_unit;
				go_near->speed 		= speed345;
				go_near->speed_units= IMC::SUNITS_METERS_PS;
				maneuvers.push_back(*go_near);
				inf("\n\nWP2:\nlat = %f \nlon = %f\nheight = %f \n", Angles::degrees(go_near->lat), Angles::degrees(go_near->lon), go_near->z);
				delete go_near;

				// 1
				go_near 		= new IMC::Goto();
				go_near->lat 		= wp1_lat;
				go_near->lon 		= wp1_lon;
				go_near->z   		= wp1_h;
				go_near->z_units 	= z_unit;
				go_near->speed 		= speed345;
				go_near->speed_units= IMC::SUNITS_METERS_PS;
				maneuvers.push_back(*go_near);
				inf("\n\nWP1:\nlat = %f \nlon = %f\nheight = %f \n", Angles::degrees(go_near->lat), Angles::degrees(go_near->lon), go_near->z);
				delete go_near;

				// 0
				go_near 		= new IMC::Goto();
				go_near->lat 		= wp0_lat;
				go_near->lon 		= wp0_lon;
				go_near->z   		= wp0_h;
				go_near->z_units 	= z_unit;
				go_near->speed 		= speed345;
				go_near->speed_units= IMC::SUNITS_METERS_PS;
				maneuvers.push_back(*go_near);
				inf("\n\nWP0:\nlat = %f \nlon = %f\nheight = %f \n", Angles::degrees(go_near->lat), Angles::degrees(go_near->lon), go_near->z);
				delete go_near;


				sequentialPlan(plan_id, &maneuvers, result);

		// DUBINS START ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

				// Global. Setting of waypoints (WPs)
				WP [xx][0] = 0;
				WP [xx][1] = wp3_meter[0] - wp4_meter[0];
				WP [xx][2] = wp2_meter[0] - wp4_meter[0];
				WP [xx][3] = wp1_meter[0] - wp4_meter[0];
				WP [xx][4] = wp0_meter[0] - wp4_meter[0];

				WP [yy][0] = wp4_h;
				WP [yy][1] = wp3_h;
				WP [yy][2] = wp2_h;
				WP [yy][3] = wp1_h;
				WP [yy][4] = wp0_h;


//				for(int i=0; i<5; i++)
//					inf("WPxx\t%f",WP[xx][i]);
//				for(int i=0; i<5; i++)
//					inf("WPyy\t%f",WP[yy][i]);

				net_hor    = WP[xx][3]; 		 // Global

				for (int i=0; i<2; i++){
					double a 	   	= sqrt(pow(WP[xx][i]-WP[xx][i+2],2) + pow(WP[yy][i]-WP[yy][i+2],2));
					double b           	= sqrt(pow(WP[xx][i]-WP[xx][i+1],2) + pow(WP[yy][i]-WP[yy][i+1],2));
					double c           	= sqrt(pow(WP[xx][i+1]-WP[xx][i+2],2) + pow(WP[yy][i+1]-WP[yy][i+2],2));
					double cos_2_alpha 	= (-a*a+b*b+c*c)/(2*b*c);
					double alpha            = acos(cos_2_alpha)/2;

					// Distance from WP to switching between circle and straight line
					double R_tangent   	= R_dubins/tan(alpha);
					// inf("Distance from WP %i to switching between circle and straight line = %f",i+1,R_tangent);

					// Angle of straight lines
					double theta1      	= atan2(WP[yy][i+1]-WP[yy][i], WP[xx][i+1]-WP[xx][i]);
					double theta2      	= atan2(WP[yy][i+2]-WP[yy][i+1], WP[xx][i+2]-WP[xx][i+1]);
					// inf("theta1=%f, theta2=%f",theta1,theta2);

					// Positions of switching between circle and straight line
					double x1          	= WP[xx][i+1] - R_tangent*cos(theta1);
					double y1          	= WP[yy][i+1] - R_tangent*sin(theta1);

					double x2          	= WP[xx][i+1] + R_tangent*cos(theta2);
					double y2          	= WP[yy][i+1] + R_tangent*sin(theta2);

					// Calculation of circle center position
					double circ_c      	= sqrt(pow((x1-x2),2) + pow((y1-y2),2));
					double cosAlpha    	= (circ_c*circ_c)/(2*R_dubins*circ_c);

					double u1	   	= (x2-x1)/circ_c; 	// Unit vector
					double u2	   	= (y2-y1)/circ_c; 	// Unit vector
					double pu1	   	= u2; 		 	// Perpendicular vector to unit vector
					double pu2	   	= -u1;		 	// Perpendicular vector to unit vector

					double intersectX1 	= x1 + u1 * (R_dubins*cosAlpha) + pu1 * (R_dubins*sqrt(1-cosAlpha*cosAlpha));
					double intersectY1 	= y1 + u2 * (R_dubins*cosAlpha) + pu2 * (R_dubins*sqrt(1-cosAlpha*cosAlpha));
					double intersectX2 	= x1 + u1 * (R_dubins*cosAlpha) - pu1 * (R_dubins*sqrt(1-cosAlpha*cosAlpha));
					double intersectY2 	= y1 + u2 * (R_dubins*cosAlpha) - pu2 * (R_dubins*sqrt(1-cosAlpha*cosAlpha));

					double xc;
					double yc;

					if (pow(WP[xx][i+1]-intersectX1, 2) + pow(WP[yy][i+1]-intersectY1, 2) > R_dubins*R_dubins){
						xc = intersectX1;
						yc = intersectY1;
					}else{
						xc = intersectX2;
						yc = intersectY2;
					}

					// Horizontal position compensated for horizontal position of net
					x1 = x1 - net_hor;
					x2 = x2 - net_hor;
					xc = xc - net_hor;

					// Circle information to be used by other functions
					circles[i][xx1] = x1;
					circles[i][xx2] = x2;
					circles[i][xxc] = xc;
					circles[i][yyc] = yc;
				}

				init_done = true;

		// DUBINS END ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


			} else{
				inf("WARNING: land_lat and/or land_lon == 0");
			}

			return true;
		}

// LANDING END ---------------------------------------------------------------------------------------


// LANDING DYNAMIC START -------------------------------------------------------------------------------------

		if (plan_id == "land_dynamic")
		{

			inf("plan_id 'land_dynamic' received");

			if(!init_done){
				inf("plan_id 'land' was not received before 'land_dynamic'");
				return false;
			}

			init_done = false;

//			double new_land_lat    	= Angles::radians(params.get("land_lat", 0.0));
//			double new_land_lon    	= Angles::radians(params.get("land_lon", 0.0));
			double new_heading		= Angles::radians(params.get("land_heading", 0.0));
			double displace_N		= params.get("displace_N", 0.0);
			double displace_E		= params.get("displace_E", 0.0);
			double displace_Up		= params.get("displace_Up", 0.0);
			int curWP			= params.get("curWP", 0);

			//inf("\n\nLan_lat:\t%f\nLand_lon:\t%f\nLand_heading:\t%f\n",Angles::degrees(land_lat),Angles::degrees(land_lon),Angles::degrees(new_heading));
			//inf("\n\nCurWP:\t\t%i\nN:\t\t%f\nE:\t\t%f\nLand_heading:\t%f\n",curWP,displace_N,displace_E,Angles::degrees(new_heading));

			IMC::MessageList<IMC::Maneuver> maneuvers_dynamic;

			double cur_lat;
			double cur_lon;

			// Iterates over all maneuvers generated when plan_id: 'land' was issued
			// Rotates and displaces the WPs
			int i = 0;
			IMC::MessageList<IMC::Maneuver>::const_iterator it;
			for(it = maneuvers.begin(); it != maneuvers.end(); it++, i++){
				if(i < curWP){
					// If the UAV has already past some WPs, those maneuvers are ignored
					continue;
				} else if(i == curWP){
					// In front of the WP currently being chased, an additional WP needs to be placed
					// This is to ensure the ILOS leads the UAV in the correct heading towards the currently chased WP,
					// and not just a straight line from current position

					cur_lat = m_estate->lat;
					cur_lon = m_estate->lon;
					Coordinates::WGS84::displace(m_estate->x,m_estate->y,&cur_lat,&cur_lon);

					// Calculation of bearing and range from current position to the net
					double bearing, range;
					Coordinates::WGS84::getNEBearingAndRange(cur_lat,cur_lon,land_lat,land_lon,&bearing,&range);

					IMC::Maneuver* man = (*it);
					IMC::Goto* go_near = (IMC::Goto *) man;

					// Maybe add something like if( std::abs(bearing-new_heading) > Angles::radians(20) )

					if(curWP == 0){
						// If the UAV is chasing the first WP, it can continue in a straight line towards it

						double cog = atan2(m_estate->vy,m_estate->vx);

						double dist = 20;
						double tmp_lat = cur_lat;
						double tmp_lon = cur_lon;

						Coordinates::WGS84::displace(dist*cos(cog) + 0*sin(cog),
								dist*sin(cog) + 0*cos(cog),
								&tmp_lat,&tmp_lon);

						go_near->lat = tmp_lat;
						go_near->lon = tmp_lon;
						go_near->z += displace_Up;

						maneuvers_dynamic.push_back(*go_near);
					} else{
						// A WP will be placed close to the UAV, but on the new line behind the net

						// Make dist depend on distance to net, distance to next waypoint and heading change
						double dist = -(range-100);

						double tmp_lat = land_lat;
						double tmp_lon = land_lon;

						Coordinates::WGS84::displace(dist*cos(new_heading) + 0*sin(new_heading),
								dist*sin(new_heading) + 0*cos(new_heading),
								&tmp_lat,&tmp_lon);

						Coordinates::WGS84::displace(displace_N,displace_E,&tmp_lat,&tmp_lon);

						go_near->lat = tmp_lat;
						go_near->lon = tmp_lon;
						go_near->z += displace_Up;

						maneuvers_dynamic.push_back(*go_near);
					}
				}

				// All WPs are rotated and displaced

				double tmp_lat = land_lat;
				double tmp_lon = land_lon;

				Coordinates::WGS84::displace((WP[xx][i]-net_hor)*cos(new_heading) + 0*sin(new_heading),
						(WP[xx][i]-net_hor)*sin(new_heading) + 0*cos(new_heading),
						&tmp_lat,&tmp_lon);

				Coordinates::WGS84::displace(displace_N,displace_E,&tmp_lat,&tmp_lon);

				IMC::Maneuver* man = (*it);
				IMC::Goto* go_near = (IMC::Goto *) man;

				go_near->lat = tmp_lat;
				go_near->lon = tmp_lon;
				go_near->z += displace_Up;

				maneuvers_dynamic.push_back(*go_near);
			}

			// Sets the global variables land_lat, land_lon and land_heading
			Coordinates::WGS84::displace(displace_N,displace_E,&land_lat,&land_lon);
			land_heading = new_heading;

			sequentialPlan(plan_id, &maneuvers_dynamic, result);

			init_done = true;

			return true;
		}

// LANDING DYNAMIC END ---------------------------------------------------------------------------------------


// EVASIVE START -------------------------------------------------------------------------------------

		if (plan_id == "evasive")
		{

			inf("plan_id 'evasive' received");

			if(ignore_evasive){
				inf("Ignore evasive is TRUE, continues with current plan");
				return false;
			}

			double cur_lat = m_estate->lat;
			double cur_lon = m_estate->lon;
			Coordinates::WGS84::displace(m_estate->x,m_estate->y,&cur_lat,&cur_lon);

			double wp0_meter[3] = {-100,-200,WP[yy][0]};
			double wp1_meter[3] = {-500,0,WP[yy][0]};

			// Calculations of relative meter and heading from net, into waypoints in rad

			// wp0
			double wp0_lat  = cur_lat;
			double wp0_lon  = cur_lon;
			double wp0_h	= wp0_meter[2];
			Coordinates::WGS84::displace(wp0_meter[xx]*cos(land_heading) + wp0_meter[yy]*sin(land_heading),
					wp0_meter[xx]*sin(land_heading) - wp0_meter[yy]*cos(land_heading),
					&wp0_lat,&wp0_lon);

			// wp1
			double wp1_lat  = cur_lat;
			double wp1_lon  = cur_lon;
			double wp1_h	= wp1_meter[2];
			Coordinates::WGS84::displace(wp1_meter[xx]*cos(land_heading) + wp1_meter[yy]*sin(land_heading),
					wp1_meter[xx]*sin(land_heading) - wp1_meter[yy]*cos(land_heading),
					&wp1_lat,&wp1_lon);


			IMC::MessageList<IMC::Maneuver> maneuvers_evasive;

			// 0
			IMC::Goto* go_near 	= new IMC::Goto();
			go_near->lat 		= wp0_lat;
			go_near->lon 		= wp0_lon;
			go_near->z   		= wp0_h;
			go_near->z_units 	= IMC::Z_HEIGHT;
			go_near->speed 		= 25;
			go_near->speed_units= IMC::SUNITS_METERS_PS;
			maneuvers_evasive.push_back(*go_near);
			delete go_near;

			// 1
			go_near 		= new IMC::Goto();
			go_near->lat 		= wp1_lat;
			go_near->lon 		= wp1_lon;
			go_near->z   		= wp1_h;
			go_near->z_units 	= IMC::Z_HEIGHT;
			go_near->speed 		= 18;
			go_near->speed_units= IMC::SUNITS_METERS_PS;
			maneuvers_evasive.push_back(*go_near);
			delete go_near;

			// 1.1
			// This loiter is meant to trigger an abort from Evasive
			IMC::Loiter* lo_near    = new IMC::Loiter();
			lo_near->lat 		= wp1_lat;
			lo_near->lon 		= wp1_lon;
			lo_near->z   		= wp1_h;
			lo_near->z_units 	= IMC::Z_HEIGHT;
			lo_near->speed 		= 18;
			lo_near->speed_units= IMC::SUNITS_METERS_PS;
			lo_near->type 		= IMC::Loiter::LT_CIRCULAR;
			lo_near->direction	= IMC::Loiter::LD_CLOCKW;
			lo_near->radius		= 160;
//			lo_near->setSourceEntity(this->getEntityState());
			lo_near->setSubId(10);
			maneuvers_evasive.push_back(*lo_near);
			delete lo_near;


			// Sends new DesiredPath to lateral and height controller to ensure it reacts immediately, otherwise almost a second delay
			IMC::DesiredPath d_path;
			d_path.end_lat = wp0_lat;
			d_path.end_lon = wp0_lon;
			d_path.end_z = wp0_h;
			d_path.end_z_units = IMC::Z_HEIGHT;
			d_path.speed = 18;
			d_path.speed_units = IMC::SUNITS_METERS_PS;
			d_path.flags = IMC::DesiredPath::FL_DIRECT;
			dispatch(d_path);
			inf("Temp. desiredPath sent");


			sequentialPlan(plan_id, &maneuvers_evasive, result);

			init_done = false;

			return true;
		}

// EVASIVE END ---------------------------------------------------------------------------------------

		// in the case the template is not understood, returns false
		return false;
	}
};
}
}

// A function for HeightPIDnew to get the values defining the landing path
bool initiateValues(double &p_net_hor, double &p_R, double p_WP[2][5], double p_circles[2][4],
		double &p_land_lat, double &p_land_lon, double &p_land_heading){

	if(init_done == false)
		return false;

	p_net_hor = net_hor;
	p_R 	  = R_dubins;

	for(int i=0; i<5; i++)
		p_WP[xx][i] = WP[xx][i];
	for(int i=0; i<5; i++)
		p_WP[yy][i] = WP[yy][i];
	for(int i=0; i<4; i++)
		p_circles[0][i] = circles[0][i];
	for(int i=0; i<4; i++)
		p_circles[1][i] = circles[1][i];

	p_land_lat 		= land_lat;
	p_land_lon 		= land_lon;
	p_land_heading 	        = land_heading;

	return true;
} 	

DUNE_TASK

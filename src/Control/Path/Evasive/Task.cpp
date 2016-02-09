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
// Author: Marcus Frølich                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Number of sequential errors before evasive
#define NUM_SIDESLIP 5
#define NUM_ALTITUDE 5
#define NUM_CROSS_TRACK 5

namespace Control
{
namespace Path
{
namespace Evasive
{
using DUNE_NAMESPACES;

struct Arguments
{
	double vert_error_tolerance;
	double hor_error_tolerance;
	double sideslip_error_tolerance;
};

struct Task: public Tasks::Periodic
{
	Arguments m_args;

	double desiredZ;
	double altitude;
	double crossTrackError;
	double psi; // Vehicle heading
	double cog; // Course over ground

	double sideslipErrorArr[NUM_SIDESLIP-1];
	bool isSideslipError;

	double altitudeErrorArr[NUM_ALTITUDE-1];
	bool isAltitudeError;

	double crossTrackErrorArr[NUM_CROSS_TRACK-1];
	bool isCrossTrackError;

	int curWP;
	bool curWPFirst;

	bool plan_dyn_received;

	//! Constructor.
	//! @param[in] name task name.
	//! @param[in] ctx context.
	Task(const std::string& name, Tasks::Context& ctx):
		Tasks::Periodic(name, ctx),
		desiredZ(0.0),
		altitude(0.0),
		crossTrackError(0.0),
		psi(0.0),
		cog(0.0),

		isSideslipError(false),
		isAltitudeError(false),
		isCrossTrackError(false),

		curWP(0),
		curWPFirst(true),

		plan_dyn_received(false)
	{
		paramActive(Tasks::Parameter::SCOPE_MANEUVER,
				Tasks::Parameter::VISIBILITY_USER);

		param("Vertical error tolerance", m_args.vert_error_tolerance)
			.units(Units::Meter)
			.minimumValue("0.0")
			.maximumValue("5.0")
			.defaultValue("1.0")
			.description("Vertical error tolerance");

		param("Horizontal error tolerance", m_args.hor_error_tolerance)
			.units(Units::Meter)
			.minimumValue("0.0")
			.maximumValue("5.0")
			.defaultValue("1.0")
			.description("Horizontal error tolerance");

		param("Sideslip error tolerance", m_args.sideslip_error_tolerance)
			.units(Units::Degree)
			.minimumValue("0.0")
			.maximumValue("90.0")
			.defaultValue("45.0")
			.description("COG/Heading error tolerance");


		bind<IMC::DesiredZ>(this);
		bind<IMC::EstimatedState>(this);
		//bind<IMC::FuelLevel>(this);
		bind<IMC::PathControlState>(this);
		bind<IMC::DesiredPath>(this);
		bind<IMC::PlanGeneration>(this);
	}

	void
	consume(const IMC::DesiredZ* dz)
	{
		if(resolveEntity(dz->getSourceEntity()) == "Height Controller PID"){
			desiredZ = dz->value;
			isAltitudeError = checkAltitudeError();
		}

	}

	void
	consume(const IMC::EstimatedState* es)
	{
		altitude = es->height - es->z;
		psi = es->psi;
		cog = atan2(es->vy,es->vx);
		isSideslipError = checkSideslipError();
	}

//	void
//	consume(const IMC::FuelLevel* fl)
//	{
//		// Set fuel level
//	}

	void
	consume(const IMC::PathControlState* pcs)
	{
		crossTrackError = pcs->y;
		isCrossTrackError = checkCrossTrackError();
	}

	void
	consume(const IMC::DesiredPath* dp)
	{
		// dp->getSubId() does not work, so this is always false
		if(resolveEntity(dp->getSourceEntity()) == "Loiter Maneuver" && dp->getSubId() == 10){
			// If loiter, abort should be sent to use standard built-in ArduPlane controllers
			IMC::Abort abort;
			abort.setDestination(getSystemId());
			dispatch(abort);
			inf("Abort is dispatched, as 'loiter' was received");
		}
		// Keeps track of what the current waypoint is
		else if(resolveEntity(dp->getSourceEntity()) == "Goto Maneuver"){
			// Need to do a check if this is the first time *dp is consumed during this landing
			if(curWPFirst){
				curWP = 0;
				curWPFirst = false;
			} else if(plan_dyn_received){
				plan_dyn_received = false;
				curWP--;
			} else{
				curWP++;
				if(curWP == 4){
					inf(" ");
					for(int i=0; i<NUM_ALTITUDE-1; i++){
						inf("altErr(%i): %f",i,altitudeErrorArr[i]);
					}
					for(int i=0; i<NUM_CROSS_TRACK-1; i++){
						inf("crossTrackErr(%i): %f",i,crossTrackErrorArr[i]);
					}
					for(int i=0; i<NUM_SIDESLIP-1; i++){
						inf("sideslipErr(%i): %f",i,Angles::degrees(sideslipErrorArr[i]));
					}
					inf(" ");
				}
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
		}else if(pg->plan_id == "land"){

			std::memset(sideslipErrorArr, 0, sizeof(sideslipErrorArr));
			isSideslipError = false;

			std::memset(altitudeErrorArr, 0, sizeof(altitudeErrorArr));
			isAltitudeError = false;

			std::memset(crossTrackErrorArr, 0, sizeof(crossTrackErrorArr));
			isCrossTrackError = false;

			curWPFirst = true;
			curWP = 0;
		}
	}

	void
	onActivation(void)
	{
		setFrequency(8);

		inf("Activated");
	}

	void
	onDeactivation(void)
	{
		inf("Deactivated");
	}

	void
	onEntityResolution(void)
	{
		spew("Entity resolution.");
	}

	bool
	checkSideslipError()
	{
		// Initialize return value to true (= need evasive)
		bool ret = true;
		if(std::abs(Angles::normalizeRadian(cog-psi)) < Angles::radians(m_args.sideslip_error_tolerance)){
			// Set return value to false if current error is small (= no need for evasive)
			ret = false;
		}else{
			for(int i=0; i<NUM_SIDESLIP-1; i++){
				if(sideslipErrorArr[i] < Angles::radians(m_args.sideslip_error_tolerance)){
					// Set return value to false if one of the older errors are small
					ret = false;
					break;
				}
			}
		}
		// Add newest error to array while the oldest is set free
		for(int i=NUM_SIDESLIP-2; i>0; i--){
			sideslipErrorArr[i] = sideslipErrorArr[i-1];
		}
		sideslipErrorArr[0] = std::abs(Angles::normalizeRadian(cog-psi));

		return ret;
	}

	bool
	checkAltitudeError()
	{
		// Initialize return value to true (= need evasive)
		bool ret = true;
		if(std::abs(desiredZ-altitude) < m_args.vert_error_tolerance){
			// Set return value to false if current error is small (= no need for evasive)
			ret = false;
		}else{
			for(int i=0; i<NUM_ALTITUDE-1; i++){
				if(altitudeErrorArr[i] < m_args.vert_error_tolerance){
					// Set return value to false if one of the older errors are small
					ret = false;
					break;
				}
			}
		}
		// Add newest error to array while the oldest is set free
		for(int i=NUM_ALTITUDE-2; i>0; i--){
			altitudeErrorArr[i] = altitudeErrorArr[i-1];
		}
		altitudeErrorArr[0] = std::abs(desiredZ-altitude);

		return ret;
	}

	bool
	checkCrossTrackError()
	{
		// Initialize return value to true (= need evasive)
		bool ret = true;
		if(std::abs(crossTrackError) < m_args.hor_error_tolerance){
			// Set return value to false if current error is small (= no need for evasive)
			ret = false;
		}else{
			for(int i=0; i<NUM_CROSS_TRACK-1; i++){
				if(crossTrackErrorArr[i] < m_args.hor_error_tolerance){
					// Set return value to false if one of the older errors are small
					ret = false;
					break;
				}
			}
		}
		// Add newest error to array while the oldest is set free
		for(int i=NUM_CROSS_TRACK-2; i>0; i--){
			crossTrackErrorArr[i] = crossTrackErrorArr[i-1];
		}
		crossTrackErrorArr[0] = std::abs(crossTrackError);

		return ret;
	}

	void
	task(void)
	{
		if (!isActive())
			return;

		// Include evasive if GPS (RTK) quality is too low (like cycle slip). See how Skulstad used AR-ratio (page 50 (74)).

		// Ignore evasive if fuel low and already tried for some time (last resort)

		// curWP == 3 means the UAV is in FA in front of net.
		// When curWP changes to 4, it is too late. Better just continue aiming for the net.
		// How far away from the actual WP that curWP changes is controlled by "Time Of Arrival Factor" in [General].
		if(curWP == 3){

			if(isAltitudeError){
				inf("\n\nEvasive (current altitude error = %f)\n",desiredZ-altitude);
				for(int i=0; i<NUM_ALTITUDE-1; i++){
					inf("altErr(%i): %f",i,altitudeErrorArr[i]);
				}
				sendEvasive();
			}else if(isCrossTrackError){
				inf("\n\nEvasive (current cross track error = %f)\n",crossTrackError);
				for(int i=0; i<NUM_CROSS_TRACK-1; i++){
					inf("crossTrackErr(%i): %f",i,crossTrackErrorArr[i]);
				}
				sendEvasive();
			}else if(isSideslipError){
				inf("\n\nEvasive (current course over ground vs heading error = %f)\n",Angles::degrees(Angles::normalizeRadian(cog-psi)));
				for(int i=0; i<NUM_SIDESLIP-1; i++){
					inf("sideslipErr(%i): %f",i,Angles::degrees(sideslipErrorArr[i]));
				}
				sendEvasive();
			}
		}
	}

	void
	sendEvasive()
	{
		// NB! If the setting ignore_evasive == true, the evasive plan will be dispatched but not generated
		IMC::PlanGeneration pg;
		pg.cmd = IMC::PlanGeneration::CMD_EXECUTE;
		pg.op = IMC::PlanGeneration::OP_REQUEST;
		pg.plan_id = "evasive";

		// Request for evasive plan will (hopefully) be consumed by [Plan.Generator_DubinsPath]
		dispatch(pg);
	}
};
}
}
}

DUNE_TASK

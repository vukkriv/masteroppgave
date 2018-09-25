
//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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
// Author: Kristoffer Gryte                                              *
//***************************************************************************

// This task is to be used in tuning of the Longitudinal path controller, by feeding it
// sinusodal references around the setpoint defined by the path.
// The sinus amplitude and frequency is controlled by the user through Neptus
// Consumes 
// Dispatches desiredZ and desired vertical rate


// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
	namespace Path
	{
		namespace LongRef
		{
			using DUNE_NAMESPACES;

			struct Arguments
			{
				bool use_controller; //Flag to enable controller
				double lower_lim_gammarate;
				struct Perturbation
				{
					// offset from the nominal (path specific) value
					double offset;
					// amplitude of the oscillation
					double amplitude;
					// frequency of the oscillation
					double frequency;
					// wheter the oscillation is sine or square
					bool is_square;
				} height_osc, spd_osc;
			};

			struct Task: public DUNE::Tasks::Periodic
			{
				Arguments m_args;
				IMC::DesiredVerticalRate m_vrate;
				IMC::DesiredZ m_zref;
				IMC::DesiredSpeed m_dspeed;
				double m_height0;
				double m_dspeed0;

				Task(const std::string& name, Tasks::Context& ctx):
					DUNE::Tasks::Periodic(name, ctx),
					m_height0(0),
					m_dspeed0(0)
				{
					param("Height ref -- frequency", m_args.height_osc.frequency)
					.defaultValue("0.0")              
					.units(Units::RadianPerSecond)
					.description("Frequency of the reference sinusodal in height");

					param("Height ref -- amplitude", m_args.height_osc.amplitude)
					.defaultValue("0.0")              
					.units(Units::Meter)
					.description("Amplitude of the reference sinusodal in height");

					param("Height ref -- offset", m_args.height_osc.offset)
					.defaultValue("200.0")              
					.units(Units::Meter)
					.description("Offset from the path-spcified height");

					param("Height ref -- square", m_args.height_osc.is_square)
					.defaultValue("false")              
					.description("If true, the height reference will be a square wave");

					param("Speed ref -- frequency", m_args.spd_osc.frequency)
					.defaultValue("0.0")              
					.units(Units::RadianPerSecond)
					.description("Frequency of the reference sinusodal in speed");

					param("Speed ref -- amplitude", m_args.spd_osc.amplitude)
					.defaultValue("0.0")              
					.units(Units::MeterPerSecond)
					.description("Amplitude of the reference sinusodal in speed");

					param("Speed ref -- offset", m_args.spd_osc.offset)
					.defaultValue("0.0")              
					.units(Units::MeterPerSecond)
					.description("Offset from the path-spcified speed");

					param("Speed ref -- square", m_args.spd_osc.is_square)
					.defaultValue("false")              
					.description("If true, the speed reference will be a square wave");

					param("Use controller", m_args.use_controller)
					.visibility(Tasks::Parameter::VISIBILITY_USER)
					.scope(Tasks::Parameter::SCOPE_MANEUVER)
					.defaultValue("false")
					.description("Use this controller for maneuver");

					bind<IMC::DesiredPath>(this);
					bind<IMC::DesiredSpeed>(this);

				}


				void
					onUpdateParameters(void)
					{
						inf("parameters updated!");
						Periodic::onUpdateParameters();
					}

				//! Reserve entity identifiers.
				void
					onEntityReservation(void)
					{
					}

				//! Resolve entity names.
				void
					onEntityResolution(void)
					{
					}

				//! Acquire resources.
				void
					onResourceAcquisition(void)
					{
					}

				//! Initialize resources.
				void
					onResourceInitialization(void)
					{
					}

				//! Release resources.
				void
					onResourceRelease(void)
					{
					}


				void
					consume(const IMC::DesiredPath* dpath)
					{
						m_height0 = dpath->end_z;
					}

				void
					consume(const IMC::DesiredSpeed* d_speed)
					{
						m_dspeed0 = d_speed->value;
					}

				void
					task(void)
					{

						if (!m_args.use_controller)
							return;

						double time = Clock::get();
						double h_osc = sin(m_args.height_osc.frequency*time);
						double h_rate_scale = 1;
						if (m_args.height_osc.is_square)
						{
							int8_t h_sgn = (h_osc > 0)? 1:-1;
							h_rate_scale = h_sgn*((h_osc == 0)? std::numeric_limits<double>::max():0.0);//-inf,0,inf
							h_osc = h_sgn;
						}
						//assume flat path
						m_zref.value =  m_height0 + m_args.height_osc.amplitude*h_osc + m_args.height_osc.offset;

						//height derivative
						m_vrate.value = h_rate_scale*m_args.height_osc.amplitude*m_args.height_osc.frequency*cos(m_args.height_osc.frequency*time);

						double s_osc = sin(m_args.spd_osc.frequency*time);
						if (m_args.spd_osc.is_square)
							s_osc = (s_osc > 0)? 1:-1;

						m_dspeed.value =  m_dspeed0 + m_args.spd_osc.amplitude*s_osc + m_args.spd_osc.offset;

						dispatch(m_vrate);
						m_zref.z_units=Z_HEIGHT;
						dispatch(m_zref);
						dispatch(m_dspeed);
					}
			};
		}
	}
}

DUNE_TASK

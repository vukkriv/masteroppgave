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
// Author: João Fortuna                                                     *
//***************************************************************************
// Paper submitted to MSC2015 and awaiting review:                          *
// "Cascaded Line-of-Sight Path-Following and                               *
// Sliding Mode Controllers for Fixed-Wing UAVs"                            *
// João Fortuna and Thor I. Fossen                                          *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace HeightGlideslope
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
    	  //Flag to enable controller
    	  bool use_controller;
          double k_vr;
          double phi_h;

      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredVerticalRate m_vrate;
        IMC::DesiredZ  zref;

        double m_airspeed;
        double glideslope_range;
        double glideslope_bearing;
        double glideslope_angle;
        double glideslope_start_z;
        bool glideslope_up;
        bool glideslope_down;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          glideslope_range(0.0),
          glideslope_bearing(0.0),
          glideslope_angle(0.0),
          glideslope_start_z(0.0),
          glideslope_up(0),
          glideslope_down(0)
        {
        	param("Height bandwidth", m_args.phi_h)
        	          .units(Units::Meter)
        	          .defaultValue("20")
        	          .description("Limit distance above and bellow desired height from which maximum control is used");

            param("Vertical Rate maximum gain", m_args.k_vr)
               .defaultValue("0.15")
               .description("Vertical Rate maximum gain for control");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");

          bind<IMC::IndicatedSpeed>(this);
          //bind<IMC::DesiredPath>(this);
          //bind<IMC::DesiredZ>(this);
       }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate height and height-rate controller
          enableControlLoops(IMC::CL_ALTITUDE);
          enableControlLoops(IMC::CL_VERTICAL_RATE);
        }

//        void
//        cosume(const IMC::DesiredPath* dpath){
//        	//Calculate glide-slope parameters
//
//        	inf("Fikk desired path!");
//        	Coordinates::WGS84::getNEBearingAndRange(dpath->start_lat,dpath->start_lon,dpath->end_lat,dpath->end_lon,&glideslope_bearing,&glideslope_range);
//
//        	if(dpath->end_z > dpath->start_z){ //Glide-slope up
//        		glideslope_angle = atan2(dpath->end_z-dpath->start_z, glideslope_range);
//        		glideslope_up = true;
//        		glideslope_down = false;
//
//        	}
//        	else{								//Glide-slope-down
//        		glideslope_angle = atan2(dpath->end_z-dpath->start_z, glideslope_range);
//        		glideslope_up = false;
//        		glideslope_down = false;
//        	}
//
//        	glideslope_start_z = dpath->start_z;
//        	zref.z_units = dpath->end_z_units;
//        }

        bool
        hasSpecificZControl(void) const
        {
          return true;
        }

        void
        consume(const IMC::IndicatedSpeed* airspeed)
        {
          m_airspeed = airspeed->value;
        }


        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
            if (!m_args.use_controller)
              return;


            //
            glideslope_angle = atan2(ts.end.z -ts.start.z,ts.track_length);
            inf("glideslope angle is: %f",glideslope_angle);

            if(ts.end.z < ts.start.z){//Glide-slope up
            	zref.value = (tan(glideslope_angle)*(ts.track_length - ts.range)) + ts.start.z; //Current desired z
            	zref.value = trimValue(zref.value,ts.start.z,tan(glideslope_angle)*(ts.track_length));
            	inf("Zref is: %f and glideslope up",zref.value);
            }
            else{ //Glide-slope down
            	zref.value = abs(tan(glideslope_angle)*(ts.range - ts.track_length)) + ts.start.z; //Current desired z
            	zref.value = trimValue(zref.value,tan(glideslope_angle)*(ts.track_length),ts.start.z);
            	inf("Zref is: %f and glideslope down",zref.value);
            }

        	double delta_h = zref.value - (state.height - state.z);
        	delta_h = delta_h *-1;
		    double delta_h_phi = (delta_h / m_args.phi_h);
		    double trimmed_d_h_phi = trimValue(delta_h_phi,-1,1);
		    m_vrate.value = m_args.k_vr * m_airspeed * trimmed_d_h_phi;

        	dispatch(m_vrate);
        	dispatch(zref);
        }
      };
    }
  }
}

DUNE_TASK

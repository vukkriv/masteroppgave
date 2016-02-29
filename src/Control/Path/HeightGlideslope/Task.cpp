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
// Author: Sigurd Olav Nevstad                                              *
//***************************************************************************
// Based on Paper: A Guidance and Control Law Design for Precision          *
// Automatic Take-off and Landing of Fixed-Wing UAVs                        *
// http://arc.aiaa.org/doi/abs/10.2514/6.2012-4674                          *
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
    	  bool use_controller; //Flag to enable controller
          double k_vr;
          double phi_h;
          double k_ph_down;
          double k_ph_up;
          double k_ih_down;
          double k_ih_up;
          double k_r_up;
          double k_r_down;
          double kp;
          double h_dot_i;
          double h_dot_p;

      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredVerticalRate m_vrate;
        IMC::DesiredZ  zref;

        //! LOS m_integrator
        double m_integrator;
        Delta m_last_step;

        double m_airspeed;
        double glideslope_range;
        double glideslope_bearing;
        double glideslope_angle;
        double glideslope_start_z;
        bool glideslope_up;
        double los_angle;
        bool glideslope_down;
        double desired_z_last;
        bool first_waypoint;
        double start_time;
        bool m_first_run;
        double last_end_z;
        bool m_shifting_waypoint;
        double state_z_shifting;
        double last_glideslope_angle;



        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          glideslope_range(0.0),
          glideslope_bearing(0.0),
          glideslope_angle(0.0),
          m_integrator(0.0),
          glideslope_start_z(0.0),
          glideslope_up(0),
          glideslope_down(0),
          desired_z_last(0.0),
          m_first_run(true),
          los_angle(0.0),
          start_time(99999),
          first_waypoint(true),
          m_shifting_waypoint(false)

        {
        	param("Height bandwidth", m_args.phi_h)
        			.units(Units::Meter)
        			.defaultValue("20")
        			.description("Limit distance above and bellow desired height from which maximum control is used");

            param("Vertical Rate maximum gain", m_args.k_vr)
               .defaultValue("0.15")
               .description("Vertical Rate maximum gain for control");

            param("LOS Proportional gain up", m_args.k_ph_up)
               .defaultValue("0.9")
               .description("LOS Proportional gain for control");

            param("LOS Proportional gain down", m_args.k_ph_down)
               .defaultValue("0.9")
               .description("LOS Proportional gain for control");

            param("LOS Integral gain up", m_args.k_ih_up)
               .defaultValue("0.1")
               .description("LOS Integral gain for control");

            param("LOS Integral gain down", m_args.k_ih_down)
               .defaultValue("0.1")
               .description("LOS Integral gain for control");

            param("LOS Radius up", m_args.k_r_up)
               .defaultValue("16.0")
               .description("Approach distance gain up");

            param("LOS Radius down", m_args.k_r_down)
                           .defaultValue("16.0")
                           .description("Approach distance gain up");

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
          first_waypoint = true; // A new path arrived. Tracking to first waypoint.
        }


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

            //Waypoint- handling
            double start_z = ts.start.z;
            double end_z = ts.end.z;


            //Handle tracking to first waypoint: Need height offset in start.z for the first waypoint, W.I.P
            if(start_z == last_end_z){
            	first_waypoint = false;
            }

           if(first_waypoint){
           	start_z = state.height - start_z;
           }

           double Vg = sqrt( (state.vx*state.vx) + (state.vy*state.vy) + (state.vz*state.vz) ); // Ground speed


            // Calculate glide-slope angle
            glideslope_angle = atan2((abs(end_z) -abs(start_z)),ts.track_length); //Negative for decent


            //Calculate Z_ref based along-track along the glideslope. Endpoint is trimmed in order so Z_ref always is between the waypoints
            if(abs(start_z) < abs(end_z)){//Glide-slope upwards
            	zref.value = tan(glideslope_angle)*(ts.track_length - ts.range) + abs(start_z); //Current desired z
            	zref.value = trimValue(zref.value,abs(start_z),tan(glideslope_angle)*(ts.track_length) + abs(start_z));
            }
            else{ //Glide-slope downwards
            	zref.value = tan(glideslope_angle)*(ts.track_length - ts.range) + abs(start_z); //Current desired z
            	zref.value = trimValue(zref.value,tan(glideslope_angle)*(ts.track_length)+ abs(start_z),abs(start_z));
            }

            if (m_first_run){
            	desired_z_last = zref.value;
                m_first_run = false;
            }

            //LP-Filter for desired z. Prevents jump in Z_ref in transition to tracking a new waypoint!
            double lp_degree = 0.97;
            desired_z_last = lp_degree*desired_z_last + (1-lp_degree)*zref.value;
            last_glideslope_angle =0.99*last_glideslope_angle + (1-0.99)*glideslope_angle;
            zref.value = desired_z_last;
            glideslope_angle = last_glideslope_angle;


            double h_error = (zref.value - (state.height - state.z))*cos(glideslope_angle); // H_error w.r.t flight-path

            inf("H_error: %f",h_error);

            //Integrator
            double timestep = m_last_step.getDelta();
            m_integrator = m_integrator + timestep*h_error;
            m_integrator = trimValue(m_integrator,-2,2); //Anti wind-up at 2 degrees


            //double h_app = Vg*m_args.k_hv; //LOS Approach distance
          //  double h_error_trimmed = trimValue(abs(h_error),0,m_args.k_r); //Force the look-ahead distance to be within a circle with radius m_args.k_r
          //  double h_app = sqrt(m_args.k_r*m_args.k_r - h_error_trimmed*h_error_trimmed);


            //Calculate look-ahead distance based on glide-slope up or down.
            if(glideslope_angle > 0){
            	double h_error_trimmed = trimValue(abs(h_error),0.0,m_args.k_r_up-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            	double h_app = sqrt(m_args.k_r_up*m_args.k_r_up - h_error_trimmed*h_error_trimmed);
            	los_angle = atan2(m_args.k_ph_up*h_error + m_args.k_ih_up*m_integrator,h_app); //Calculate LOS-angle glideslope up
            }
            else{ //Glideslope down or straight-line
            	double h_error_trimmed = trimValue(abs(h_error),0.0,m_args.k_r_down-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            	double h_app = sqrt(m_args.k_r_down*m_args.k_r_down - h_error_trimmed*h_error_trimmed);
            	los_angle = atan2(m_args.k_ph_down*h_error + m_args.k_ih_down*m_integrator,h_app); //Calculate LOS-angle glideslope down
            }
            //Limit los_angle based on saturation limit for climb-rate.
            los_angle = trimValue(los_angle,-0.2,0.2);
            inf("Los_angle: %f",los_angle*(180/3.14159265));

            double gamma_cmd = glideslope_angle + los_angle; //Commanded flight path angle
            double h_dot_desired = Vg*sin(gamma_cmd);		 //Convert commanded flight path angle to demanded vertical-rate.

            m_vrate.value = h_dot_desired;

            //Testing constant-rate
            //h_dot_desired = Vg*sin(2*(M_PI/180));
            //m_vrate.value= h_dot_desired;

        	dispatch(m_vrate);
        	zref.z_units=Z_HEIGHT;
        	dispatch(zref);

        	last_end_z = end_z;
        	last_glideslope_angle = glideslope_angle;

        }
      };
    }
  }
}

DUNE_TASK

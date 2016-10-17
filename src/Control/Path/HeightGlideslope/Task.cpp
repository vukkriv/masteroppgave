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
        bool use_refmodel;
        double k_vr;
        double phi_h;
        double k_ph_down;
        double k_ih_down;
        double k_r_down;

        double k_ph_up;
        double k_ih_up;
        double k_r_up;

        double k_ph_line;
        double k_ih_line;
        double k_r_line;

        double kp;
        double h_dot_i;
        double h_dot_p;

        double Tref_z;
        double zeta_z;
        double Tref_gamma;
        double zeta_gamma;

        double los_min_deg;
        double los_max_deg;
      };

      class ReferenceModel
      {
      public:
        ReferenceModel():
          A(Matrix(3,3, 0.0)),
          B(3,1, 0.0),
          I(3),
          x(3,1, 0.0),
          T(1.0),
          w(1/T),
          zeta(1.0)
      {
          /*
           * A = [0    1               0;
           *      0    0               1;
           *      -w^3 -(2*zeta+1)*w^2 -(2*zeta+1)*w]
           */
          A(0,1)=1;
          A(1,2)=1;
          A(2,0)=-w*-w*-w;
          A(2,1)=-(2*zeta+1)*w*w;
          A(2,2)=-(2*zeta+1)*w;

          /*
           * B = [0
           *      0;
           *      w^3]
           */
          B(2,0)=w*w*w;
          I(3);
      }
        void updateRefmodel()
        {
          A(0,1)=1;
          A(1,2)=1;
          A(2,0)=-w*-w*-w;
          A(2,1)=-(2*zeta+1)*w*w;
          A(2,2)=-(2*zeta+1)*w;
          B(2,0)=w*w*w;
          I(3);
        }
        void setTimeconstant(double Tconst)
        {
          T = Tconst;
          w = 1/Tconst;
          updateRefmodel();
        }

        void setDampeningRatio(double zeta_z)
        {
          zeta = zeta_z;
          updateRefmodel();
        }


      public:
        Matrix A;
        Matrix B;
        Matrix I;
        Matrix x;
        double T; //Time-constant
        double w; //natural frequency
        double zeta; //Relative damping ratio
      };

      struct waypoint
      {
        double x;
        double y;
        double z;
      };

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredVerticalRate m_vrate;
        IMC::DesiredZ  zref;
        IMC::DesiredLinearState zref_nofilter; //Used as DesiredZ for non-filtered height reference, for live plotting in Neptus
        IMC::ControlParcel m_parcel_los;
        waypoint last_end_wp;
        Delta m_last_step;

        double m_airspeed;
        double glideslope_range;
        double glideslope_bearing;
        double glideslope_angle;
        double m_integrator;
        double glideslope_start_z;
        bool glideslope_up;
        bool glideslope_down;
        double desired_z_last;
        bool m_first_run;
        double los_angle;
        double start_time;
        double last_end_z;
        bool last_loitering;
        double last_start_z;
        bool first_waypoint;
        bool m_shifting_waypoint;
        double state_z_shifting;
        bool last_WP_loiter;
        double last_loiter_z;
        ReferenceModel m_refmodel_z;
        ReferenceModel m_refmodel_gamma;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          glideslope_range(0.0),
          glideslope_bearing(0.0),
          glideslope_angle(1.0),
          m_integrator(0.0),
          glideslope_start_z(0.0),
          glideslope_up(0),
          glideslope_down(0),
          desired_z_last(0.0),
          m_first_run(true),
          los_angle(1.0),
          start_time(99999),
          last_start_z(9999),
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

          param("LOS Integral gain up", m_args.k_ih_up)
          .defaultValue("0.1")
          .description("LOS Integral gain for control");

          param("LOS Radius up", m_args.k_r_up)
          .defaultValue("14.0")
          .description("Approach distance gain up");

          param("LOS Proportional gain down", m_args.k_ph_down)
          .defaultValue("0.9")
          .description("LOS Proportional gain for control");

          param("LOS Integral gain down", m_args.k_ih_down)
          .defaultValue("0.1")
          .description("LOS Integral gain for control");

          param("LOS Radius down", m_args.k_r_down)
          .defaultValue("14.0")
          .description("Approach distance gain up");

          param("LOS Proportional gain line", m_args.k_ph_line)
          .defaultValue("1.4")
          .description("LOS Proportional gain for control");

          param("LOS Integral gain line", m_args.k_ih_line)
          .defaultValue("0.02")
          .description("LOS Integral gain for control");

          param("LOS Radius line", m_args.k_r_line)
          .defaultValue("25.0")
          .description("Approach distance gain up");

          param("Time constant refmodelZ", m_args.Tref_z)
          .defaultValue("1.0")
          .description("Time constant for reference model for desired Z");

          param("Dampening ratio refmodelZ", m_args.zeta_z)
          .defaultValue("1.0")
          .description("Dampening ratio for reference model for desired Z");

          param("Time constant refmodelGamma", m_args.Tref_gamma)
          .defaultValue("1.0")
          .description("Time constant for reference model for gamma");

          param("Dampening ratio refmodelGamma", m_args.zeta_gamma)
          .defaultValue("1.0")
          .description("Dampening ratio for reference model for gamma");

          param("Minimum LOS angle", m_args.los_min_deg)
          .defaultValue("-7.0")              
          .units(Units::Degree)
          .description("Longitudinal line of sight angle is saturated to this value");

          param("Maximum LOS angle", m_args.los_max_deg)
          .defaultValue("7.0")              
          .units(Units::Degree)
          .description("Longitudinal line of sight angle is saturated to this value");

          param("Use reference model", m_args.use_refmodel)
          .defaultValue("true")
          .description("Flag to use reference model");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");

          bind<IMC::IndicatedSpeed>(this);

        }

        
        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          m_first_run = true; //Updates parameters in a later if-statement
          
          //m_refmodel_z.setTimeconstant(m_args.Tref_z);
          //m_refmodel_gamma.setTimeconstant(m_args.Tref_gamma);
          //m_refmodel_z.setDampeningRatio(m_args.zeta_z);
          //m_refmodel_gamma.setDampeningRatio(m_args.zeta_gamma);
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

        void
        onPathDeactivation(void)
        {
          if (!m_args.use_controller){
            // Deactivate controller.
            disableControlLoops(IMC::CL_ALTITUDE);
            disableControlLoops(IMC::CL_VERTICAL_RATE);
          }
        }
        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void)state;
          (void)ts;

          if (!m_args.use_controller){
            disableControlLoops(IMC::CL_ALTITUDE);
            disableControlLoops(IMC::CL_VERTICAL_RATE);
          }
          else{
            // Activate controller
            enableControlLoops(IMC::CL_ALTITUDE);
            enableControlLoops(IMC::CL_VERTICAL_RATE);
          }

          //Check if tracking to first waypoint
          if(last_start_z != ts.start.z){
            if(last_end_z == ts.start.z){
              first_waypoint = false;
            }
            else{
              first_waypoint = true;
            }
          }

          if(ts.start.z == last_end_z){
            first_waypoint = false;
          }

          last_end_z = ts.end.z;
          last_start_z = ts.start.z;
          m_shifting_waypoint = true;
          if(last_loitering){
            last_WP_loiter = true;
          }
          else{
            last_WP_loiter = false;
          }
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

          if(first_waypoint){
            start_z = state.height - start_z;
          }

          double Vg = sqrt( (state.vx*state.vx) + (state.vy*state.vy) + (state.vz*state.vz) ); // Ground speed


          // Calculate glide-slope angle
          glideslope_angle = atan2((std::abs(end_z) -std::abs(start_z)),ts.track_length); //Negative for decent
          double glideslope_angle_nofilter = glideslope_angle;

          if(last_WP_loiter){
            start_z = last_loiter_z;
          }

          //Calculate Z_ref based along-track along the glideslope. Endpoint is trimmed in order so Z_ref always is between the waypoints
          if(std::abs(start_z) < std::abs(end_z)){//Glide-slope upwards
            zref.value = (tan(glideslope_angle)*(ts.track_length - ts.range)) + std::abs(start_z); //Current desired z
            zref.value = trimValue(zref.value,std::abs(start_z),tan(glideslope_angle)*(ts.track_length) + std::abs(start_z));
          }
          else{ //Glide-slope downwards
            zref.value = (tan(glideslope_angle)*(ts.track_length - ts.range)) + std::abs(start_z); //Current desired z
            zref.value = trimValue(zref.value,tan(glideslope_angle)*(ts.track_length)+ std::abs(start_z),std::abs(start_z));
          }
          if (m_first_run){
            m_refmodel_z.x(0,0)     = (state.height - state.z);
            m_refmodel_gamma.x(0,0) = glideslope_angle;
            m_refmodel_z.setTimeconstant(m_args.Tref_z);
            m_refmodel_gamma.setTimeconstant(m_args.Tref_gamma);
            m_refmodel_z.setDampeningRatio(m_args.zeta_z);
            m_refmodel_gamma.setDampeningRatio(m_args.zeta_gamma);
            m_first_run = false;
          }

          if(ts.loitering){
            zref.value = ts.loiter.center.z;
            glideslope_angle = 0.0;
            last_loiter_z = ts.loiter.center.z;
            debug("Loiter-z: %f",ts.loiter.center.z);
            debug("end-z : %f",ts.end.z);
            debug("start-z : %f",ts.start.z);
          }
          else{
            debug("end-z : %f",ts.end.z);
            debug("start-z : %f",ts.start.z);
          }

          //****************************************************
          // Reference model for desired Z and flight-path angle
          //****************************************************
          zref_nofilter.z = zref.value;
          if(m_args.use_refmodel)
          {
            debug("Z-ref before filter: %f",zref.value);
            m_refmodel_z.x = (m_refmodel_z.I + (ts.delta*m_refmodel_z.A))*m_refmodel_z.x + (ts.delta*m_refmodel_z.B) * zref.value;
            zref.value = m_refmodel_z.x(0,0);

            debug("glideslope before filter: %f",glideslope_angle);

            m_refmodel_gamma.x = (m_refmodel_gamma.I + (ts.delta*m_refmodel_gamma.A))*m_refmodel_gamma.x + (ts.delta*m_refmodel_gamma.B) * glideslope_angle;
            glideslope_angle = m_refmodel_gamma.x(0,0);
          }


          //Calculate height error along glideslope
          double h_error = (zref.value - (state.height - state.z))*cos(glideslope_angle);
          spew("H_error: %f",h_error);

          //Integrator
          double timestep = m_last_step.getDelta();
          m_integrator = m_integrator + timestep*h_error;
          m_integrator = trimValue(m_integrator,-2,2); //Anti wind-up at 2 meter


          //Calculate look-ahead distance based on glide-slope up, down or straight line
          if(glideslope_angle_nofilter > 0){ //Glideslope up
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_up-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_up*m_args.k_r_up - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_up*h_error + m_args.k_ih_up*m_integrator,h_app); //Calculate LOS-angle glideslope up
            m_parcel_los.p  =m_args.k_ph_up*h_error;
            m_parcel_los.i = m_args.k_ih_up*m_integrator;
            spew("Glideslope UP! %f",glideslope_angle);
          }
          else if(glideslope_angle_nofilter < 0){ //Glideslope down
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_down-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_down*m_args.k_r_down - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_down*h_error + m_args.k_ih_down*m_integrator,h_app); //Calculate LOS-angle glideslope down
            m_parcel_los.p  =m_args.k_ph_down*h_error;
            m_parcel_los.i = m_args.k_ih_down*m_integrator;

            spew("Glideslope DOWN! %f",glideslope_angle);
          }
          else{//Straight line
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_line-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_line*m_args.k_r_line - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_line*h_error + m_args.k_ih_line*m_integrator,h_app); //Calculate LOS-angle straight line
            m_parcel_los.p  =m_args.k_ph_line*h_error;
            m_parcel_los.i = m_args.k_ih_line*m_integrator;
            spew("Glideslope LINE ! %f",glideslope_angle);
          }


          los_angle = trimValue(los_angle,Angles::radians(m_args.los_min_deg),Angles::radians(m_args.los_max_deg));
          debug("Los_angle: %f",los_angle*(180/3.14159265));

          double gamma_cmd = glideslope_angle + los_angle; //Commanded flight path angle
          double h_dot_desired = Vg*sin(gamma_cmd);        //Convert commanded flight path angle to demanded vertical-rate.

          m_vrate.value = h_dot_desired;

          //Testing constant-rate
          //h_dot_desired = Vg*sin(2*(M_PI/180));
          //m_vrate.value= h_dot_desired;

          dispatch(m_vrate);
          zref.z_units=Z_HEIGHT;
          dispatch(zref);
          dispatch(zref_nofilter);
          dispatch(m_parcel_los);

          last_end_wp.x = ts.end.x;
          last_end_wp.y = ts.end.y;
          last_end_wp.z = ts.end.z;
          last_loitering = ts.loitering;
        }
      };
    }
  }
}

DUNE_TASK

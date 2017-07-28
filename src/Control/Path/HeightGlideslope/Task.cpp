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
        bool use_ratelim;
        double k_ph_down;
        double k_ih_down;
        double k_dh_down;
        double k_r_down;

        double k_ph_up;
        double k_ih_up;
        double k_dh_up;
        double k_r_up;

        double k_ph_line;
        double k_ih_line;
        double k_dh_line;
        double k_r_line;

        double Tref_z_ramp;
        double zeta_z_ramp;
        double Tref_z_step;
        double zeta_z_step;
        double Tref_gamma;
        double zeta_gamma;

        double los_min_deg;
        double los_max_deg;
        double upper_lim_zrate;
        double lower_lim_zrate;
        double upper_lim_gammarate;
        double lower_lim_gammarate;
      };

      class ReferenceModel
      {
      public:
        ReferenceModel():
          A(Matrix(3,3, 0.0)),
          B(3,1, 0.0),
          C(1,3,0.0),
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
          B(2,0)=1;
          C(0,0)=w*w*w;
          C(0,1)=(2*zeta+1)*w;
          C(0,2)=0;
          I(3);
      }
        void updateRefmodel()
        {
          A(0,1)=1;
          A(1,2)=1;
          A(2,0)=-w*-w*-w;
          A(2,1)=-(2*zeta+1)*w*w;
          A(2,2)=-(2*zeta+1)*w;
          B(2,0)=1;
          C(0,0)=w*w*w;
          C(0,1)=(2*zeta+1)*w;
          C(0,2)=0;
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

        void updateFilter(double Tref_z, double zeta_z)
        {
          double w_new = 1/Tref_z;
          Matrix x_old = x;
          //Need to calculate the new x vector because of the change to the C matrix (y_new = C_new*x_new = C_old*x_old = y_old)

          x(0,0) = C(0,0)/(w_new*w_new*w_new)*x_old(0,0);
          x(1,0) = C(0,1)/((2*zeta_z + 1)*w_new)*x_old(1,0);
          x(2,0) = 0.0;

          setTimeconstant(Tref_z);
          setDampeningRatio(zeta_z);
        }


      public:
        Matrix A;
        Matrix B;
        Matrix C;
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
        IMC::DesiredZ m_zref;
        IMC::DesiredLinearState ref_nofilter; //Used for non-filtered height and glideslope_angle reference filtered and gldeslope_angle, for live plotting in Neptus
        IMC::ControlParcel m_parcel_los;
        IMC::RelativeState m_hdiff;
        waypoint m_last_end_wp;
        Delta m_last_step;

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
        double m_last_end_z;
        bool m_last_loitering;
        double m_last_start_z;
        bool first_waypoint;
        bool m_shifting_waypoint;
        double state_z_shifting;
        bool m_last_WP_loiter;
        double m_last_loiter_z;
        double m_prev_unfiltered_height;
        double m_prev_z;
        double m_prev_gamma;
        bool m_last_filter_ramp;
        ReferenceModel m_refmodel_z;
        ReferenceModel m_refmodel_gamma;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
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
          m_last_start_z(9999),
          first_waypoint(true),
          m_shifting_waypoint(false),
          m_prev_unfiltered_height(0),
          m_prev_z(0.0),
          m_prev_gamma(0.0),
          m_last_filter_ramp(false)

        {
          param("LOS Proportional gain up", m_args.k_ph_up)
          .defaultValue("0.9")
          .description("LOS Proportional gain for control");

          param("LOS Integral gain up", m_args.k_ih_up)
          .defaultValue("0.1")
          .description("LOS Integral gain for control");

          param("LOS Derivative gain up", m_args.k_dh_up)
          .defaultValue("0.0")
          .description("LOS Derivative gain for control");

          param("LOS Radius up", m_args.k_r_up)
          .defaultValue("14.0")
          .description("Approach distance gain up");

          param("LOS Proportional gain down", m_args.k_ph_down)
          .defaultValue("0.9")
          .description("LOS Proportional gain for control");

          param("LOS Integral gain down", m_args.k_ih_down)
          .defaultValue("0.1")
          .description("LOS Integral gain for control");

          param("LOS Derivative gain down", m_args.k_dh_down)
          .defaultValue("0.0")
          .description("LOS Derivative gain for control");

          param("LOS Radius down", m_args.k_r_down)
          .defaultValue("14.0")
          .description("Approach distance gain up");

          param("LOS Proportional gain line", m_args.k_ph_line)
          .defaultValue("1.4")
          .description("LOS Proportional gain for control");

          param("LOS Derivative gain line", m_args.k_dh_line)
          .defaultValue("0.0")
          .description("LOS Derivative gain for control");

          param("LOS Integral gain line", m_args.k_ih_line)
          .defaultValue("0.02")
          .description("LOS Integral gain for control");

          param("LOS Radius line", m_args.k_r_line)
          .defaultValue("25.0")
          .description("Approach distance gain up");

          param("Time constant refmodelZ (ramp)", m_args.Tref_z_ramp)
          .defaultValue("1.0")
          .description("Time constant for reference model for desired Z ramp");

          param("Dampening ratio refmodelZ (ramp)", m_args.zeta_z_ramp)
          .defaultValue("1.0")
          .description("Dampening ratio for reference model for desired Z ramp");

          param("Time constant refmodelZ (step)", m_args.Tref_z_step)
          .defaultValue("1.0")
          .description("Time constant for reference model for desired Z step");

          param("Dampening ratio refmodelZ (step)", m_args.zeta_z_step)
          .defaultValue("1.0")
          .description("Dampening ratio for reference model for desired Z step");

          param("Time constant refmodelGamma", m_args.Tref_gamma)
          .defaultValue("1.0")
          .description("Time constant for reference model for gamma");

          param("Dampening ratio refmodelGamma", m_args.zeta_gamma)
          .defaultValue("1.0")
          .description("Dampening ratio for reference model for gamma");

          param("Minimum LOS angle", m_args.los_min_deg)
          .defaultValue("-10.0")              
          .units(Units::Degree)
          .description("Longitudinal line of sight angle is saturated to this value");

          param("Maximum LOS angle", m_args.los_max_deg)
          .defaultValue("10.0")              
          .units(Units::Degree)
          .description("Longitudinal line of sight angle is saturated to this value");

          param("Maximum z rate", m_args.upper_lim_zrate)
          .defaultValue("15.0")              
          .units(Units::MeterPerSecond)
          .description("When rate limited, z rate is saturated at this value");

          param("Minimum z rate", m_args.lower_lim_zrate)
          .defaultValue("-20.0")              
          .units(Units::MeterPerSecond)
          .description("When rate limited, z rate is saturated at this value");

          param("Maximum gamma rate", m_args.upper_lim_gammarate)
          .defaultValue("15.0")              
          .units(Units::DegreePerSecond)
          .description("When rate limited, gamma rate is saturated at this value");

          param("Minimum gamma rate", m_args.lower_lim_gammarate)
          .defaultValue("-15.0")              
          .units(Units::DegreePerSecond)
          .description("When rate limited, gamma rate is saturated at this value");

          param("Use reference model", m_args.use_refmodel)
          .defaultValue("false")
          .description("Flag to use reference model");

          param("Use rate limiter", m_args.use_ratelim)
          .defaultValue("true")
          .description("Flag to use rate limiter (when ref model is not used)");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");

        }

        
        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          if (m_last_filter_ramp)
          {
            m_refmodel_z.updateFilter(m_args.Tref_z_ramp, m_args.zeta_z_ramp);
          }
          else if (!m_last_filter_ramp)
          {
            m_refmodel_z.updateFilter(m_args.Tref_z_step, m_args.zeta_z_step);
          }

          m_refmodel_gamma.updateFilter(m_args.Tref_gamma, m_args.zeta_gamma);

          if (paramChanged(m_args.use_controller) && !m_args.use_controller)
          { //controller should no longer be used
            disableControlLoops(IMC::CL_ALTITUDE | IMC::CL_VERTICAL_RATE);
          }

        }
        

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate height and height-rate controller
          enableControlLoops(IMC::CL_ALTITUDE | IMC::CL_VERTICAL_RATE);
          first_waypoint = true; // A new path arrived. Tracking to first waypoint.
        }

        void
        onPathDeactivation(void)
        {

        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void)state;

          if (m_args.use_controller)
          {
            // Activate controller
            enableControlLoops(IMC::CL_ALTITUDE | IMC::CL_VERTICAL_RATE);
          }

          //Check if tracking to first waypoint
          if(m_last_start_z != ts.start.z)
          {
            if(m_last_end_z == ts.start.z)
              first_waypoint = false;
            else
              first_waypoint = true;
          }

          if(m_last_end_z == ts.start.z)
            first_waypoint = false;

          m_last_end_z = ts.end.z;
          m_last_start_z = ts.start.z;
          m_shifting_waypoint = true;
          if(m_last_loitering)
            m_last_WP_loiter = true;
          else
            m_last_WP_loiter = false;
        }

        bool
        hasSpecificZControl(void) const
        {
          return true;
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          if (!m_args.use_controller)
            return;

          //Waypoint- handling
          double start_z = ts.start.z;
          double end_z = ts.end.z;

          if(first_waypoint)
          {
            start_z = state.height - start_z;
            m_prev_gamma = atan2((std::abs(end_z) -std::abs(start_z)),ts.track_length); //Negative for decent
          }

          double speed_g = ts.speed; // Ground speed 

          // Calculate glide-slope angle
          glideslope_angle = atan2((std::abs(end_z) -std::abs(start_z)),ts.track_length); //Negative for decent
          double glideslope_angle_nofilter = glideslope_angle;

          if(m_last_WP_loiter)
            start_z = m_last_loiter_z;


          //Calculate Z_ref based along-track along the glideslope. Endpoint is trimmed in order so Z_ref always is between the waypoints
          if(std::abs(start_z) < std::abs(end_z)){//Glide-slope upwards
            m_zref.value = (tan(glideslope_angle)*(ts.track_length - ts.range)) + std::abs(start_z); //Current desired z
            m_zref.value = trimValue(m_zref.value,std::abs(start_z),tan(glideslope_angle)*(ts.track_length) + std::abs(start_z));
          }
          else{ //Glide-slope downwards
            m_zref.value = (tan(glideslope_angle)*(ts.track_length - ts.range)) + std::abs(start_z); //Current desired z
            m_zref.value = trimValue(m_zref.value,tan(glideslope_angle)*(ts.track_length)+ std::abs(start_z),std::abs(start_z));
          }
          if (m_first_run){
            // Avoid large jumps in the desired height when 
            // going to first WP (since initial x(0,0) = 0)
            // or when updating filter parameters

            // initialize model so that z = C*x is true, assuming x(1,0) = 0
            m_refmodel_z.x(0,0) = (state.height - state.z)/m_refmodel_z.C(0,0);
            m_refmodel_z.x(1,0) = 0.0;
            m_refmodel_z.x(2,0) = 0.0;

            // initialize model so that z = C*x is true, assuming x(1,0) = 0
            m_refmodel_gamma.x(0,0) = glideslope_angle/m_refmodel_gamma.C(0,0);
            m_refmodel_gamma.x(1,0) = 0.0;
            m_refmodel_gamma.x(2,0) = 0.0;

            m_first_run = false;
            m_prev_z = (state.height - state.z);
            m_prev_gamma = glideslope_angle;
          }

          if(ts.loitering){
            m_zref.value = ts.loiter.center.z;
            glideslope_angle = 0.0;
            m_last_loiter_z = ts.loiter.center.z;
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
          ref_nofilter.z = m_zref.value;
          ref_nofilter.ay = glideslope_angle;

          if((m_args.use_refmodel) && (ts.delta < 10))
          {
            double height_ref_derivative = (m_zref.value - m_prev_unfiltered_height)/ts.delta;
            spew("Height ref derivative: %f", height_ref_derivative);
            spew("Last filter ramp?: %d", m_last_filter_ramp);

            ////Checking what state the unfiltered reference is in and filtering accordingly
            //if (height_ref_derivative <= 0.0001 && height_ref_derivative >= -0.0001 && m_last_filter_ramp)
            //{
              ////Step
              //spew("Filter: Step");
              //m_refmodel_z.updateFilter(m_args.Tref_z_step, m_args.zeta_z_step);
              //m_last_filter_ramp = false;
            //}
            //else if ((height_ref_derivative > 0.0001 || height_ref_derivative < -0.0001) && !m_last_filter_ramp)
            //{
              ////Ramp
              spew("Filter: Ramp");
              //m_refmodel_z.updateFilter(m_args.Tref_z_ramp, m_args.zeta_z_ramp);
              m_last_filter_ramp = true;
            //}

            debug("Z-ref before filter: %f",m_zref.value);
            m_refmodel_z.x = (m_refmodel_z.I + (ts.delta*m_refmodel_z.A))*m_refmodel_z.x + (ts.delta*m_refmodel_z.B) * m_zref.value;
            m_zref.value = m_refmodel_z.C(0,0)*m_refmodel_z.x(0,0) + m_refmodel_z.C(0,1)*m_refmodel_z.x(1,0) + m_refmodel_z.C(0,2)*m_refmodel_z.x(2,0);

            debug("glideslope before filter: %f",glideslope_angle);

            m_refmodel_gamma.x = (m_refmodel_gamma.I + (ts.delta*m_refmodel_gamma.A))*m_refmodel_gamma.x + (ts.delta*m_refmodel_gamma.B) * glideslope_angle;
            glideslope_angle = m_refmodel_gamma.C(0,0)*m_refmodel_gamma.x(0,0); //Uses C = [omega^3 0 0] because of unfiltered bahaviour is only a series of steps
          }
          else if ((m_args.use_ratelim) && (ts.delta < 10))
          {
            //height rate limiter
              double rate = (m_zref.value - m_prev_z);
              double upper_lim = ts.delta*m_args.upper_lim_zrate;
              double lower_lim = ts.delta*m_args.lower_lim_zrate;
              spew("Z upper %f, lower %f, rate %f, delta_t %f", upper_lim, lower_lim, rate, ts.delta);

              //m_zref.value = trimValue(m_zref.value, m_prev_unfiltered_height
              if (rate > upper_lim) 
              {
                m_zref.value = m_prev_z + ts.delta*upper_lim;
                debug("Limiting z rate to upper lim: zref = %f", m_zref.value);
              }
              else if (rate < lower_lim)
              {
                m_zref.value = m_prev_z + ts.delta*lower_lim;
                debug("Limiting z rate to lower lim: zref = %f", m_zref.value);
              } 
              //else //unmodified reference
              
            //gamma rate limiter
              rate = (glideslope_angle - m_prev_gamma);
              upper_lim = ts.delta*Angles::radians(m_args.upper_lim_gammarate);
              lower_lim = ts.delta*Angles::radians(m_args.lower_lim_gammarate);
              spew("Gamma upper %f, lower %f, rate %f, delta_t %f", upper_lim, lower_lim, rate, ts.delta);

              //m_zref.value = trimValue(m_zref.value, m_prev_unfiltered_height
              if (rate > upper_lim) 
              {
                glideslope_angle = m_prev_gamma + ts.delta*upper_lim;
                debug("Limiting gamma rate to upper lim: gammaref = %f", glideslope_angle);
              }
              else if (rate < lower_lim)
              {
                glideslope_angle = m_prev_gamma + ts.delta*lower_lim;
                debug("Limiting gamma rate to lower lim: gammaref = %f", glideslope_angle);
              } 
              //else //unmodified reference
          }

          ref_nofilter.vy = glideslope_angle; //store filtered value, for plot
          m_prev_z = m_zref.value;
          m_prev_gamma = glideslope_angle;

          //Calculate height error along glideslope
          double h_error = (m_zref.value - (state.height - state.z))*cos(glideslope_angle);
          spew("H_error: %f",h_error);

          //Integrator
          double timestep = m_last_step.getDelta();
          m_integrator = m_integrator + timestep*h_error;
          m_integrator = trimValue(m_integrator,-2,2); //Anti wind-up at 2 meter

          //Derivative term
          double h_dot = state.u*sin(state.theta) - state.v*sin(state.phi)*cos(state.theta) - state.w*cos(state.phi)*cos(state.theta);


          //Calculate look-ahead distance based on glide-slope up, down or straight line
          if(glideslope_angle_nofilter > 0){ //Glideslope up
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_up-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_up*m_args.k_r_up - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_up*h_error + m_args.k_ih_up*m_integrator + m_args.k_dh_up*h_dot,h_app); //Calculate LOS-angle glideslope up
            m_parcel_los.p = m_args.k_ph_up*h_error;
            m_parcel_los.i = m_args.k_ih_up*m_integrator;
            spew("Glideslope UP! %f",glideslope_angle);
          }
          else if(glideslope_angle_nofilter < 0){ //Glideslope down
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_down-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_down*m_args.k_r_down - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_down*h_error + m_args.k_ih_down*m_integrator + m_args.k_dh_down*h_dot,h_app); //Calculate LOS-angle glideslope down
            m_parcel_los.p = m_args.k_ph_down*h_error;
            m_parcel_los.i = m_args.k_ih_down*m_integrator;
            spew("Glideslope DOWN! %f",glideslope_angle);
          }
          else{//Straight line
            double h_error_trimmed = trimValue(std::abs(h_error),0.0,m_args.k_r_line-0.5); //Force the look-ahead distance to be within a circle with radius m_args.k_r
            double h_app = sqrt(m_args.k_r_line*m_args.k_r_line - h_error_trimmed*h_error_trimmed);
            m_parcel_los.a = h_app;
            los_angle = atan2(m_args.k_ph_line*h_error + m_args.k_ih_line*m_integrator + m_args.k_dh_line*h_dot,h_app); //Calculate LOS-angle straight line
            m_parcel_los.p = m_args.k_ph_line*h_error;
            m_parcel_los.i = m_args.k_ih_line*m_integrator;
            spew("Glideslope LINE ! %f",glideslope_angle);
          }


          los_angle = trimValue(los_angle,Angles::radians(m_args.los_min_deg),Angles::radians(m_args.los_max_deg));
          m_parcel_los.d = los_angle; // dispatch los angle as the derivative los parcel, for plotting
          debug("Los_angle: %f",Angles::degrees(los_angle));

          double gamma_cmd = glideslope_angle + los_angle; //Commanded flight path angle
          double h_dot_desired = speed_g*sin(gamma_cmd);        //Convert commanded flight path angle to demanded vertical-rate.

          m_vrate.value = h_dot_desired;

          //Testing constant-rate
          //h_dot_desired = speed_g*sin(2*(M_PI/180));
          //m_vrate.value= h_dot_desired;

          m_hdiff.err_z = (state.height - state.z) - m_zref.value;

          m_prev_unfiltered_height = ref_nofilter.z;

          dispatch(m_vrate);
          m_zref.z_units=Z_HEIGHT;
          dispatch(m_zref);
          dispatch(ref_nofilter);
          dispatch(m_parcel_los);
          dispatch(m_hdiff);

          m_last_end_wp.x = ts.end.x;
          m_last_end_wp.y = ts.end.y;
          m_last_end_wp.z = ts.end.z;
          m_last_loitering = ts.loitering;
        }
      };
    }
  }
}

DUNE_TASK

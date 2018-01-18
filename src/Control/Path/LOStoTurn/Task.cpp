//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace LOStoTurn
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Maximum bank angle - Defined by aircraft structural, navigation
        //! or control constraints
        double max_bank;
        //! Radius of acceptance for line-of-sight
        double acc_radius;
        //! Look ahead time for line-of-sight
        double lookahead;
        //! Sliding surface gain
        double rho;
        //! Course error gain
        double lambda;
        //! K_d gain
        double k_d;
        //! Sliding surface bandwidth
        double bandwidth;
        //! Estimated aircraft roll time constant
        double roll_tc;
        //! Flag to enable controller
        bool use_controller;
        //! Flag to enable Z controller
        bool has_Z_crtl;
        //! Flag to enable controller
        bool use_filter_roll;
        //! Flag to enable rate feed forward
        bool use_rate_ff;
        //! Ardupilot roll time constant
        double roll_ap_tc;
        //! Roll filter coefficient
        double rll_smooth_fact;
        //! Lower limit for roll error to enable smoothing 
        double smooth_lim;
        //! Upper limit for filter integrator time
        double delta_t_lim;
        //! Choose how the lookahead distance should be selected: 1; lookahead time 2; radius of acceptance, 3; speed-dependant radius of acceptance
        int lookahead_type;
      };

      static const std::string c_parcel_names[] = {DTR_RT(""), DTR_RT("Extra1"), DTR_RT("Extra2")};

      enum Parcel {
        PC_NORMAL = 0,
        PC_EXTRA1 = 1,
        PC_EXTRA2 = 2
      };

      static const int NUM_PARCELS = 3;

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredRoll m_bank;
        IMC::ControlParcel m_parcels[NUM_PARCELS];
        double m_airspeed;
        double m_bank_lim;
        double m_lookahead, m_lookahead_sq;
        double m_W_x, m_W_y;
        double m_bank_prev;
        double m_bank_dot;
        //! Time difference in seconds used in Eulers method
        Time::Delta m_delta;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          m_bank_lim(0.0),
          m_lookahead(0.0),
          m_lookahead_sq(0.0),
          m_W_x(0.0),
          m_W_y(0.0),
          m_bank_prev(0.0),
          m_bank_dot(0.0)
        {
          param("Radius of acceptance", m_args.acc_radius)
          .defaultValue("30.0")
          .units(Units::Meter)
          .description("Radius of acceptance for finding lookahead");

          param("Lookahead time", m_args.lookahead)
          .defaultValue("3.0")
          .description("Lookahead time in seconds");

          param("Rho", m_args.rho)
          .defaultValue("4.0")
          .description("Sliding surface gain");

          param("Lambda", m_args.lambda)
          .defaultValue("2.0")
          .description("Course error gain");

          param("Kd", m_args.k_d)
          .defaultValue("3.0")
          .description("Kd gain");

          param("Bandwidth", m_args.bandwidth)
          .defaultValue("5.0")
          .description("Sliding surface bandwidth");

          param("Roll Time Const", m_args.roll_tc)
          .defaultValue("0.1")
          .description("Estimated aircraft roll time constant");

          param("Maximum Bank", m_args.max_bank)
          .units(Units::Degree)
          .minimumValue("5")
          .maximumValue("90")
          .defaultValue("30")
          .description("Limit for absolute value of output bank angle reference");

          param("Use controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Use this controller for maneuver");

          param("Use external Z control", m_args.has_Z_crtl)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("True indicates that you use a spesific Z controller, i.e. PathController will not send Z references");

          param("Smoothen roll reference", m_args.use_filter_roll)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable roll reference filter for maneuver, to avoid large steps to low-level control");

          param("Feed forward roll rate", m_args.use_rate_ff)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable roll rate feed forward, by counter acting AP rate calculation");

          param("Ardupilot roll time constant", m_args.roll_ap_tc)
          .defaultValue("0.2")
          .maximumValue("100.0")
          .minimumValue("0.000000000001")
          .description("Ardupilot roll time constant, RLL2SRV_TCONST");

          param("Roll smoothing factor", m_args.rll_smooth_fact)
          .defaultValue("0.8")
          .maximumValue("10.0")
          .minimumValue("0.0")
          .description("Smoothing factor on the roll reference");

          param("Roll smoothing limit", m_args.smooth_lim)
          .defaultValue("20")
          .maximumValue("90")
          .minimumValue("0.0")
          .description("If the absolute roll error is less than this, smoothing is not used");

          param("Max filter integration time", m_args.delta_t_lim)
          .defaultValue("0.5")
          .maximumValue("2")
          .minimumValue("0.0")
          .description("Limit the integration time in the roll filter to this value, to avoid overflow e.g. when the entity has been inactive.");

          param("Lookahead type", m_args.lookahead_type)
          .minimumValue("1")
          .defaultValue("1")
          .maximumValue("3")
          .description("Choose how the lookahead distance is calculated: 1; lookahead time, 2; radius of acceptance, 3; speed-dependant radius of acceptance");

          bind<IMC::IndicatedSpeed>(this);
          bind<IMC::EstimatedStreamVelocity>(this);

          m_delta.reset();
       }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          m_bank_lim = Angles::radians(m_args.max_bank);
          m_lookahead = m_args.lookahead;
          m_lookahead_sq = m_lookahead * m_lookahead;

          if (paramChanged(m_args.use_controller) && !m_args.use_controller)
          { //controller should no longer be used
            disableControlLoops(IMC::CL_ROLL);
          }
        }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate bank (roll) controller.
          enableControlLoops(IMC::CL_ROLL);
        }

        void
        onPathDeactivation(void)
        {

        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void)state;
          (void)ts;

          if (m_args.use_controller){
            enableControlLoops(IMC::CL_ROLL);
          }
        }

        void
        consume(const IMC::IndicatedSpeed* airspeed)
        {
          m_airspeed = airspeed->value;
        }

        void
        consume(const IMC::EstimatedStreamVelocity* wind)
        {
          m_W_x = wind->x;
          m_W_y = wind->y;
        }
        
        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();

          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(this->getEntityLabel() + c_parcel_names[i] + " Parcel"));
        }

        virtual bool
        hasSpecificZControl(void) const
        {
          if(m_args.has_Z_crtl)           
            return true;
          else
            return false;
        }


        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          spew("Step start");

          debug("hasSpecificZControl: %i",hasSpecificZControl());

          if (!m_args.use_controller)
            return;

          // Check if airspeed is larger than zero
          if (m_airspeed <= 0)
          {
            war("No waypoint tracking control update: Airspeed <= 0!");
            return;
          }

          IMC::ReportedState log_state;

          double speed_g = Math::norm(Math::norm(state.vx,state.vy),state.vz);//ts.speed;
          double g_speed_sq = speed_g * speed_g;

          double chi = ts.course;
          double phi = state.phi;

          double chi_p = ts.track_bearing; //path angle
          double chi_p_dot = 0; // path derivative

          m_parcels[PC_EXTRA1].p = speed_g;
          m_parcels[PC_EXTRA1].d = chi;
          m_parcels[PC_EXTRA1].i = chi_p;
          m_parcels[PC_EXTRA1].a = chi_p_dot;

          /* log_state.roll = chi_dot; */

          double y_e = ts.track_pos.y; //does not account for path angle
          double x_e = ts.track_pos.x; //does not account for path angle
          double y_e_sq = y_e * y_e;
          /* double y_dot = ts.track_vel.y; */ //does not account for vertical speed
          double y_dot   = speed_g*sin ( chi         ) ;
          double y_e_dot = speed_g*sin ( chi - chi_p ) ;

          m_parcels[PC_EXTRA2].p = y_e;
          m_parcels[PC_EXTRA2].d = ts.track_pos.y;
          m_parcels[PC_EXTRA2].i = y_e_dot;
          m_parcels[PC_EXTRA2].a = ts.track_vel.y;

          double lookahead_dist, lookahead_dist_sq, y_trimmed;
          // LOS
          switch (m_args.lookahead_type) {
            default:
            case 1:
              // lokahead time
              
              lookahead_dist = m_lookahead*speed_g;
              lookahead_dist_sq = m_lookahead_sq*g_speed_sq;
              break;
            case 2:
              //radius of acceptance
              y_trimmed = trimValue(std::abs(y_e),0.0,m_args.acc_radius*0.9); //Force the look-ahead distance to be within a circle with radius acc_radius
              lookahead_dist = sqrt(m_args.acc_radius*m_args.acc_radius - y_trimmed*y_trimmed);
              lookahead_dist_sq = lookahead_dist*lookahead_dist;
              break;
            case 3:
              //speed-depentant radius of acceptance
              y_trimmed = trimValue(std::abs(y_e),0.0,m_args.lookahead*speed_g*0.9); //Force the look-ahead distance to be within a circle with radius acc_radius
              lookahead_dist = sqrt((m_args.lookahead*speed_g)*(m_args.lookahead*speed_g) - y_trimmed*y_trimmed);
              lookahead_dist_sq = lookahead_dist*lookahead_dist;
              break;
          }
          log_state.depth = lookahead_dist;

          double chi_d = -std::atan(y_e/lookahead_dist) + chi_p;
          
          //desired cross track error speed
          double y_e_dot_d = speed_g*sin ( chi_d - chi_p ) ;
          double chi_d_dot = -(lookahead_dist/(lookahead_dist_sq + y_e_sq)) * y_e_dot_d + (y_e/(lookahead_dist_sq + y_e_sq)) * lookahead_dist_dot + chi_p_dot;
          double chi_tilde = Angles::normalizeRadian(chi_d - chi);


          /* double chi_err = chi - chi_d; */
          /* /1* double chi_err_dot = chi_dot - chi_d_dot; *1/ */
          /* if ((x - ts.track_length > 20) && (chi_err < 0.1)) */
          /* {// we are following the LOS, but have passed the WP */
          /*   signalError(DTR("passedWP divergence error")); */
          /* } */
          /* log_state.pitch = chi_err; */
          /* /1* log_state.yaw = chi_err_dot; *1/ */

          m_bank.value = atan(speed_g/(Math::c_gravity*cos(chi - state.psi))*(m_args.k_chi*chi_tilde + chi_d_dot));
          m_parcels[PC_NORMAL].p = chi_d_dot;
          m_parcels[PC_NORMAL].d = cos(chi-state.psi);
          m_parcels[PC_NORMAL].i = y_e_dot_d;
          m_parcels[PC_NORMAL].a = chi_tilde;

          /* m_parcels[PC_EXTRA3].p = chi_d; */
          /* m_parcels[PC_EXTRA3].d = chi_p; */
          /* m_parcels[PC_EXTRA3].i = */ 
          /* m_parcels[PC_EXTRA3].a = chi_tilde; */
          for (int i = 0; i < NUM_PARCELS; ++i) {
            dispatch(m_parcels[i]);
          }

          double delta_t = trimValue(m_delta.getDelta(),0.0, m_args.delta_t_lim); //Avoid large delta if e.g. module has been inactive
          
          if ((m_args.use_filter_roll) && (std::abs(Math::Angles::minSignedAngle(m_bank.value, m_bank_prev)) > Math::Angles::radians(m_args.smooth_lim)))
          {
            // Filter roll reference, to avoid large steps for low-level controller
            // Only enable when roll error is large
            spew("Using roll filter, since use_filter_roll is %d and roll error is %f", m_args.use_filter_roll, std::abs(Math::Angles::minSignedAngle(m_bank.value, m_bank_prev)));
            
            m_bank_dot = -m_args.rll_smooth_fact*m_bank_prev + m_bank.value;
            spew("m_bank_dot: %f \t dt: %f",m_bank_dot,delta_t);
            if (m_args.use_rate_ff) 
            {
              // inverse AP calculation of desired rates
              // Not done with SITL testing
              m_bank.value = state.phi - m_bank_dot*m_args.roll_ap_tc/0.01f;
            }
            else
            {
              m_bank.value = m_bank_prev + m_bank_dot*delta_t; //Eulers method
            }
            spew("DesiredBank vs FilteredBank in roll filter: %f", Math::Angles::degrees(Math::Angles::minSignedAngle(m_bank.value, m_bank_prev)));
          }

          
          // Output - Bank angle command, constrained
          m_bank.value = trimValue(m_bank.value, -m_bank_lim, m_bank_lim);
          m_bank_prev = m_bank.value;

          // Send to bus
          dispatch(m_bank);
          spew("DesiredBank = %3.2f deg", Math::Angles::degrees(m_bank.value));
          //for plotting internal signals in tuning
          dispatch(log_state);
        }
      };
    }
  }
}

DUNE_TASK

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
// Input: Desired speed and desired path-angle/vertical-rate				*
// Output: Desired pitch and throttle										*
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace Longitudinal
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        bool use_controller; //Flag to enable controller
        double k_thr_p;
        double k_thr_i;
        double k_gamma_p;
        double k_thr_ph;
        double trim_pitch;
        double trim_throttle;
        double thr_max;
        double thr_min;
        double pitch_max_deg;
        double pitch_min_deg;
        // Input filtering
        std::string dz_src;

      };

      static const std::string c_parcel_names[] = {DTR_RT("Throttle"), DTR_RT("Pitch")};

      enum Parcel {
        PC_THR = 0,
        PC_PTCH = 1
      };

      static const int NUM_PARCELS = 2;

      struct Task: public DUNE::Control::PathController
      {
        Arguments m_args;
        IMC::DesiredThrottle m_throttle;
        IMC::DesiredPitch m_pitch;
        //! Parcel array
        IMC::ControlParcel m_parcels[NUM_PARCELS];

        double m_airspeed;
        double m_dspeed;
        double m_dvrate;
        double m_thr_i;
        double m_dz;
        double m_h_err;
        bool   m_h_err_feedforward;
        Delta m_last_step;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_airspeed(0.0),
          m_dspeed(18.0),
          m_dvrate(0.0),
          m_thr_i(0.0),
          m_dz(0.0),
          m_h_err(0.0),
          m_h_err_feedforward(false)

        {
          param("Use controller", m_args.use_controller)
			              .visibility(Tasks::Parameter::VISIBILITY_USER)
			              .scope(Tasks::Parameter::SCOPE_MANEUVER)
			              .defaultValue("false")
			              .description("Use this controller for maneuver");

          param("Throttle Integrator gain", m_args.k_thr_i)
          .defaultValue("1.0")
          .description("Throttle Integrator gain");

          param("Throttle Proportional gain", m_args.k_thr_p)
          .defaultValue("10.0")
          .description("Throttle Proportional gain");

          param("Throttle Proportional height gain", m_args.k_thr_ph)
          .defaultValue("2.0")
          .description("Throttle Proportional height gain");

          param("Gamma Proportional gain", m_args.k_gamma_p)
          .defaultValue("5.0")
          .description("Gamma Proportional gain");

          param("Trim pitch", m_args.trim_pitch)
          .defaultValue("2.6585")
          .description("Trim Pitch for level flight");

          param("Trim throttle", m_args.trim_throttle)
          .defaultValue("44.0")
          .description("Trim throttle for level flight");

          param("Minimum pitch", m_args.pitch_min_deg)
          .defaultValue("-10.0")              
          .units(Units::Degree)
          .description("Desired pitch is saturated to this value");

          param("Maximum pitch", m_args.pitch_max_deg)
          .defaultValue("15.0")
          .units(Units::Degree)
          .description("Desired pitch is saturated to this value");

          param("Minimum throttle", m_args.thr_min)
          .defaultValue("-30.0")              
          .units(Units::Percentage)
          .description("Throttle integrator is limited to this percentage");

          param("Maximum throttle", m_args.thr_max)
          .defaultValue("30.0")              
          .units(Units::Percentage)
          .description("Throttle integrator is limited to this percentage");

          param("Desired Vertical Rate source", m_args.dz_src)
          .defaultValue("Glideslope Height Controller")
          .description("Entity allowed to set DesiredZ and DesiredVerticalRate.");

          bind<IMC::IndicatedSpeed>(this);
          bind<IMC::DesiredVerticalRate>(this);
          bind<IMC::DesiredSpeed>(this);
          bind<IMC::DesiredZ>(this);

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();


          if (paramChanged(m_args.use_controller) && !m_args.use_controller)
          { //controller should no longer be used
            disableControlLoops(IMC::CL_THROTTLE | IMC::CL_PITCH);
          }

        }

        void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
            return;
          // Activate controller
          enableControlLoops(IMC::CL_THROTTLE | IMC::CL_PITCH);
        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          (void)state;
          (void)ts;

          if (m_args.use_controller){
            // Activate controller
            enableControlLoops(IMC::CL_THROTTLE | IMC::CL_PITCH);
          }
        }

        void
        onPathDeactivation(void)
        {

        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();

          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(this->getEntityLabel() + c_parcel_names[i] + " Parcel"));
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
        consume(const IMC::DesiredSpeed* d_speed)
        {
          m_dspeed = d_speed->value;
        }

        void
        consume(const IMC::DesiredZ* d_z)
        {
          if(!(d_z->getSourceEntity() == resolveEntity(m_args.dz_src)))
            return;


          m_h_err_feedforward = true;

          m_dz = d_z->value;
        }

        void
        consume(const IMC::DesiredVerticalRate* d_vrate)
        {
          if(!(d_vrate->getSourceEntity() == resolveEntity(m_args.dz_src)))
            return;

          m_dvrate = d_vrate->value;
        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {
          if (!m_args.use_controller)
            return;

          //double speed_g = ts.speed; // Ground speed 
          double speed_g = sqrt(state.vx*state.vx+state.vy*state.vy+state.vz*state.vz);//ground speed
          double h_dot = state.u*sin(state.theta) - state.v*sin(state.phi)*cos(state.theta) - state.w*cos(state.phi)*cos(state.theta);
          double gamma_now = asin(h_dot/speed_g);

          double glideslope_angle = atan2((std::abs(ts.end.z) -std::abs(ts.start.z)),ts.track_length); //Negative for decent

          // Assumes no wind
          double alpha_now = gamma_now - state.theta;

          double gamma_desired = asin(m_dvrate/speed_g);
          double gamma_error = gamma_now - gamma_desired;
          double V_error =  m_dspeed - m_airspeed;

          if(m_h_err_feedforward){
            m_h_err = (m_dz - (state.height - state.z))*std::cos(glideslope_angle);
            m_h_err = trimValue(m_h_err,-2,2);
          }
          else
          {
            m_h_err = 0.0;
          }


          //Throttle integrator
          double timestep = m_last_step.getDelta();
          m_thr_i = m_thr_i + timestep*V_error;
          m_thr_i = trimValue(m_thr_i,m_args.thr_min,m_args.thr_max); //Throttle anti wind-up at 

          //Calculate desired throttle and pitch
          double throttle_desired = m_args.k_thr_p*V_error + m_args.k_thr_i*m_thr_i + m_h_err*m_args.k_thr_ph + m_args.trim_throttle;
          double pitch_desired = gamma_desired + Angles::radians(m_args.trim_pitch)-gamma_error*m_args.k_gamma_p; //Backstepping,pitch_desired = gamma_desired + alpha_0
          pitch_desired = trimValue(pitch_desired,Angles::radians(m_args.pitch_min_deg),Angles::radians(m_args.pitch_max_deg));
          m_throttle.value = trimValue(throttle_desired, 0, 100);
          m_pitch.value = pitch_desired;

          m_parcels[PC_THR].p = m_args.k_thr_p*V_error;
          m_parcels[PC_THR].i = m_args.k_thr_i*m_thr_i;
          m_parcels[PC_THR].d = m_h_err*m_args.k_thr_ph ;
          m_parcels[PC_PTCH].p = gamma_error*m_args.k_gamma_p; 
          m_parcels[PC_PTCH].i = gamma_desired; //Abuse of notation

          spew("pitch desired: %f \t alpha_0: %f",m_pitch.value,Angles::degrees(alpha_now));

          dispatch(m_throttle);
          dispatch(m_pitch);
          for (int i = 0; i < NUM_PARCELS; ++i) {
            dispatch(m_parcels[i]);
          }
        }
      };
    }
  }
}

DUNE_TASK

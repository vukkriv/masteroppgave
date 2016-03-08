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
// Author: Jornel van den hoorn                                             *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

#include <vector>
#include <queue>

namespace Control
{
  namespace NetCatch
  {
    namespace Velocity
    {
      using DUNE_NAMESPACES;

      //! %Task arguments.
      struct Arguments
      {
        std::string m_copter_id;

        //! Enable this velocity controller or not
        bool use_controller;
        //! Disable Z flag, this will utilize new rate controller on some targets
        bool disable_Z;

        bool disable_path_control;

        //!velocity Controller parameters
        Matrix Kp_path;
        Matrix Ki_path;
        Matrix Kd_path;

        //!velocity Controller parameters
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;

        double max_norm_F;

        //! Frequency of controller
        double m_freq;
      };

      static const std::string c_parcel_names[] = { "PID-X",   "PID-Y",   "PID-Z",
                                                    "ERROR-X", "ERROR-Y", "ERROR-Z",};
      enum Parcel
      {
        PC_PID_X = 0,
        PC_PID_Y = 1,
        PC_PID_Z = 2,
        PC_ERROR_X = 3,
        PC_ERROR_Y = 4,
        PC_ERROR_Z = 5
      };

      static const int NUM_PARCELS = 6;

      struct Task: public DUNE::Control::PeriodicUAVAutopilot
      {
        //! Controllable loops.
        static const uint32_t c_controllable = IMC::CL_SPEED;
        //! Required loops.
        static const uint32_t c_required = IMC::CL_FORCE;

        //! Task arguments.
        Arguments m_args;

        //! Last messages received
        IMC::DesiredVelocity m_v_des;
        IMC::DesiredHeading m_desired_heading;
        IMC::EstimatedLocalState m_est_l_state;
        IMC::Acceleration m_a_est;

        IMC::ControlParcel m_parcels[NUM_PARCELS];

        Matrix m_v_int_value;

        uint64_t m_time_end;
        uint64_t m_time_diff;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          PeriodicUAVAutopilot(name, ctx, c_controllable, c_required),
          m_v_int_value(3,1,0.0),
          m_time_end(0.0),
          m_time_diff(0.0)
        {
          param("Velocity Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("true")
          .description("Enable Velocity Controller");

          param("Frequency", m_args.m_freq)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Controller frequency");

          param("Kp Path Velocity Control", m_args.Kp_path)
          .defaultValue("1.0,1.0,1.0").visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Position Controller tuning parameter Kp");

          param("Ki Path Velocity Control", m_args.Ki_path)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Ki");

          param("Kd Path Velocity Control", m_args.Kd_path)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Kd");

          param("Kp Velocity Control", m_args.Kp)
          .defaultValue("1.0,1.0,1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Position Controller tuning parameter Kp");

          param("Ki Velocity Control", m_args.Ki)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Ki");

          param("Kd Velocity Control", m_args.Kd)
          .defaultValue("0.0,0.0,0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Velocity Controller tuning parameter Kd");

          param("Maximum Normalized Force", m_args.max_norm_F)
          .defaultValue("5.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Maximum Normalized Force of the Vehicle");

          param("Disable Z flag", m_args.disable_Z)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable Z flag. In turn, this will utilize new rate controller on some targets");

          param("Disable Path Control", m_args.disable_path_control)
          .defaultValue("false").visibility(
          Tasks::Parameter::VISIBILITY_USER).description("Choose whether to control velocity in the current path frame");

          // Bind incoming IMC messages
          bind<IMC::DesiredVelocity>(this);
          bind<IMC::DesiredHeading>(this);
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::Acceleration>(this);
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Force Parcel"));
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          inf("Current frequency: %f",getFrequency());
          setFrequency(m_args.m_freq);
          inf("Frequency changed to : %f",getFrequency());
        }



        void
        consume(const IMC::DesiredVelocity* msg)
        {
          m_v_des = *msg;
        }

        void
        consume(const IMC::DesiredHeading* msg)
        {
          spew("Got DesiredHeading from system '%s' and entity '%s'.",
                resolveSystemId(msg->getSource()),
                resolveEntity(msg->getSourceEntity()).c_str());
          m_desired_heading = *msg;
        }


        void
        consume(const IMC::EstimatedLocalState* msg)
        {
          /*inf("Got EstimatedLocalState \nfrom '%s' at '%s'",
                 resolveEntity(msg->getSourceEntity()).c_str(),
                 resolveSystemId(msg->getSource()));
           */

          if (getSystemId() == msg->getSource())
          {

            //inf("-->consume copter est loc state");
            m_est_l_state = *msg;
          }
        }

        void
        consume(const IMC::Acceleration* msg)
        {
          m_a_est = *msg;
        }

        //! Control velocity in NED frame
        Matrix
        vel_con(Matrix v_est, Matrix a_est, Matrix v_des)
        {
          static double startPrint = 0;
          if (Clock::get() - startPrint > 1)
          {
            spew("v_est: [%f,%f,%f]", v_est(0),v_est(1),v_est(2));
            spew("v_des: [%f,%f,%f]", v_des(0),v_des(1),v_des(2));
            startPrint = Clock::get();
          }

          Matrix e_v_est = v_des-v_est;
          Matrix e_a_est = -a_est;

          m_v_int_value = m_v_int_value + e_v_est*m_time_diff;

          Matrix F_des = Matrix(3,1,0.0);
          Matrix p = Matrix(3,1,0.0);
          Matrix i = Matrix(3,1,0.0);
          Matrix d = Matrix(3,1,0.0);
          if (m_args.disable_path_control)
          {
            p = m_args.Kp*e_v_est;
            i = m_args.Ki*m_v_int_value;
            d = m_args.Kd*e_a_est;
            F_des = p + i + d;
          }
          else
          {
            Matrix F_des_path = Matrix(3, 1, 0.0);
            Matrix Rpn = Rzyx(0,0,m_desired_heading.value);
            e_v_est = Rpn*e_v_est;
            e_a_est = Rpn*e_a_est;
            m_v_int_value = Rpn*m_v_int_value;

            p = m_args.Kp_path*e_v_est;
            i = m_args.Ki_path*m_v_int_value;
            d = m_args.Kd_path*e_a_est;
            F_des_path = p + i + d;

            F_des = transpose(Rpn)*F_des_path;
          }

          IMC::ControlParcel parcel_pid_x = m_parcels[PC_PID_X];
          IMC::ControlParcel parcel_pid_y = m_parcels[PC_PID_Y];
          IMC::ControlParcel parcel_pid_z = m_parcels[PC_PID_Z];
          parcel_pid_x.p = p(0);
          parcel_pid_y.p = p(1);
          parcel_pid_z.p = p(2);

          parcel_pid_x.d = d(0);
          parcel_pid_y.d = d(1);
          parcel_pid_z.d = d(2);

          parcel_pid_x.i = i(0);
          parcel_pid_y.i = i(1);
          parcel_pid_z.i = i(2);

          dispatch(parcel_pid_x);
          dispatch(parcel_pid_y);
          dispatch(parcel_pid_z);

          IMC::ControlParcel errors_x = m_parcels[PC_ERROR_X];
          IMC::ControlParcel errors_y = m_parcels[PC_ERROR_Y];
          IMC::ControlParcel errors_z = m_parcels[PC_ERROR_Z];

          errors_x.p = e_v_est(0);
          errors_y.p = e_v_est(1);
          errors_z.p = e_v_est(2);

          errors_x.d = e_a_est(0);
          errors_y.d = e_a_est(1);
          errors_z.d = e_a_est(2);

          errors_x.i = m_v_int_value(0);
          errors_y.i = m_v_int_value(1);
          errors_z.i = m_v_int_value(2);

          dispatch(errors_x);
          dispatch(errors_y);
          dispatch(errors_z);


          if (F_des.norm_2() > m_args.max_norm_F)
          {
            F_des = sqrt(pow(m_args.max_norm_F,2)) * F_des/F_des.norm_2();
          }
          return F_des;
        }

        //! Dispatch desired force
        void
        sendDesiredForce(Matrix F_des)
        {
          IMC::DesiredControl desired_force;

          desired_force.x =  F_des(0);
          desired_force.y =  F_des(1);
          desired_force.z =  F_des(2);

          if (m_args.disable_Z)
          {
            desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y;
          }
          else
          {
            desired_force.flags = IMC::DesiredControl::FL_X | IMC::DesiredControl::FL_Y | IMC::DesiredControl::FL_Z;
          }

          desired_force.setSourceEntity(getEntityId());
          dispatch(desired_force);
        }

        virtual void
        reset(void)
        {
          m_time_end = Clock::getMsec();
          m_time_diff = 0.0;
          m_v_int_value = Matrix(3,1,0.0);
        }

        //! Main loop.
        void
        task(void)
        {
          if(!m_args.use_controller || !isActive())
            return;

          m_time_diff = Clock::getMsec() - m_time_end;
          m_time_end = Clock::getMsec();

          Matrix v_est = Matrix(3,1,0);
          v_est(0) = m_est_l_state.vx;
          v_est(1) = m_est_l_state.vy;
          v_est(2) = m_est_l_state.vz;

          Matrix a_est = Matrix(3,1,0);
          a_est(0) = m_a_est.x;
          a_est(1) = m_a_est.y;
          a_est(2) = m_a_est.z;

          Matrix v_des = Matrix(3,1,0);
          v_des(0) = m_v_des.u;
          v_des(1) = m_v_des.v;
          v_des(2) = m_v_des.w;

          Matrix F_des   = vel_con(v_est,a_est,v_des);

          sendDesiredForce(F_des);

          //spew("Frequency: %1.1f", 1000.0/m_time_diff);
        }

        //! @return  Rotation matrix.
        Matrix Rzyx(double phi, double theta, double psi) const
        {
          double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(phi)), ( sin(psi)*sin(phi))+(cos(psi)*  cos(phi)*sin(theta)),
              sin(psi)*cos(theta), ( cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
              -sin(theta), 			 cos(theta)*sin(phi), 								  cos(theta)*cos(phi)};
          return Matrix(R_en_elements,3,3);
        }

      };
    }
  }
}

DUNE_TASK

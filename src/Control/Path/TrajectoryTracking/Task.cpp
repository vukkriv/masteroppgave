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
// Author: RecepCetin                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Path
  {
    namespace TrajectoryTracking
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        bool use_controller;
        double max_speed;
        double refmodel_max_speed;
        double refmodel_omega_n;
        double refmodel_xi;
        double ref_down;
        bool delayed_feedback;
        double Kp;
      };

      struct Task: public DUNE::Control::PathController
      {
        IMC::TranslationalSetpoint m_setpoint;
        //! Task arguments.
        Arguments m_args;
        //! Reference model state
        Matrix m_refmodel_x;
        //! Reference model trans.matrix
        Matrix m_refmodel_A;
        //! Reference model input matrix
        Matrix m_refmodel_B;
        //! Desired speed profile
        double m_desired_speed;
        //! Current autopilot mode
        IMC::AutopilotMode m_autopilot_mode;
        Matrix m_R_bn;
        Matrix m_R_nb;
        bool m_eulerangles;


        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_desired_speed(0)
        {

          param("Velocity Controller", m_args.use_controller)
                                       .visibility(Tasks::Parameter::VISIBILITY_USER)
                                       .scope(Tasks::Parameter::SCOPE_MANEUVER)
                                       .defaultValue("false")
                                       .description("Enable Velocoty Controller");


          param("Max Speed", m_args.max_speed)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .description("Max speed of the vehicle");

          param("Reference Model - Max Speed", m_args.refmodel_max_speed)
          .defaultValue("3.0");

          param("Down reference - height", m_args.ref_down)
          .defaultValue("-10.0");

          param("Delayed Feedback controller", m_args.delayed_feedback)
          .defaultValue("False");

          param("Reference Model - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.1")
          .description("Natural frequency for the speed reference model");

          param("Reference Model - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .description("Relative Damping Factorof the speed reference model");

          param("Velocity Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("0.1")
          .description("P-Gain of the velocity controller");

          bind<IMC::EulerAngles>(this);
          m_eulerangles = false;

        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          // update desired speed to max speed
          m_desired_speed = m_args.refmodel_max_speed;

        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
        }


        //! Consumer for EulerAngles message
        void
        consume(const IMC::EulerAngles* eangles)
        {
          m_eulerangles = true;
          //eangles->

        }

        //! Consumer for DesiredSpeed message.
        //! @param dspeed message to consume.
        void
        consume(const IMC::DesiredSpeed* dspeed)
        {
          // overloaded.
          // Update desired speed
          if (dspeed->value < m_args.refmodel_max_speed)
          {
            m_desired_speed = dspeed->value;
          }
          else
          {
            m_desired_speed = m_args.refmodel_max_speed;
            debug("Trying to set a speed above maximum speed. ");
          }

          PathController::consume(dspeed);
        }

        void
        initRefmodel(const IMC::EstimatedState& state)
        {
          // Convencience matrix
          double ones[] = {1.0, 1.0, 1.0};

          Matrix eye = Matrix(ones, 3);

          // Restart refmodel
          m_refmodel_x = Matrix(6, 1, 0.0);
          m_refmodel_x(0) = state.x;
          m_refmodel_x(1) = state.y;
          m_refmodel_x(2) = state.z;

          m_refmodel_x(3) = state.u;
          m_refmodel_x(4) = state.v;
          m_refmodel_x(5) = state.w;


          //          m_vel = Matrix(2,1, 0.0);
          //          m_vel(0) = m_refmodel_x(3);
          //          m_vel(1) = m_refmodel_x(4);
          //          m_vel(2) = m_refmodel_x(5);



          // Set model
          Matrix A_12 = eye;
          Matrix A_11 = Matrix(3,3, 0.0);

          Matrix A_21 = -pow(m_args.refmodel_omega_n, 2)*eye;
          Matrix A_22 = -2*m_args.refmodel_omega_n * m_args.refmodel_xi * eye;

          Matrix A_1 = A_11.horzCat(A_12);
          Matrix A_2 = A_21.horzCat(A_22);

          m_refmodel_A = A_1.vertCat(A_2);

          m_refmodel_B = Matrix(3,3, 0.0).vertCat(eye) * pow(m_args.refmodel_omega_n,2);

        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          (void)ts;


          // Print end coordinates
          debug("End coordinates: %f, %f", ts.end.x, ts.end.y);

          // Restart ref model
          initRefmodel(state);

        }

        virtual void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
          {
            requestDeactivation();
            return;
          }
          // Activate velocity controller.
          enableControlLoops(IMC::CL_SPEED);

          // Activate height controller
          enableControlLoops(IMC::CL_ALTITUDE);
          inf("Vel-control activated.");

        }

        void
        step(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          if (!m_args.use_controller)
            return;


          /*
           *
           * Calculates input (setpoint)
           * NB: There is a serious bug in height calculation.
           */
          Matrix x_d = Matrix(3,1, 0.0);
          x_d(0) = ts.end.x;
          x_d(1) = ts.end.y;
          x_d(2) = -ts.end.z;


          // Update reference
          m_refmodel_x += ts.delta * (m_refmodel_A * m_refmodel_x + m_refmodel_B * x_d);


          // Saturate velocity
          Matrix vel = m_refmodel_x.get(3,5,0,0);


          if( vel.norm_2() > m_args.refmodel_max_speed )
          {
            vel = m_args.refmodel_max_speed * vel / vel.norm_2();

            m_refmodel_x.put(3,0,vel);
          }

          debug("Vel norm: %f", m_refmodel_x.get(3,5,0,0).norm_2());

          // Move at a rate towards the target

          // Head straight to target

          vel(0) = m_refmodel_x(3) - m_args.Kp * (state.x - m_refmodel_x(0));
          vel(1) = m_refmodel_x(4) - m_args.Kp * (state.y - m_refmodel_x(1));
          vel(2) = m_refmodel_x(5) - m_args.Kp * (state.z - m_refmodel_x(2));

          if( vel.norm_2() > m_args.max_speed )
          {
            vel = m_args.max_speed * vel / vel.norm_2();
          }

          //
          //
          //
          //          // Rotate from body to ned
          //          m_R_bn = Rbn(state.phi, state.theta, state.psi);
          //          //m_R_nb = transpose(m_R_bn);
          //
          //          Matrix temp;
          //          temp = m_R_bn.multiply(m_vel);
          //          m_refmodel_x(3) = temp(0);
          //          m_refmodel_x(4) = temp(1);
          //          m_refmodel_x(5) = temp(2);


          m_setpoint.x = m_refmodel_x(0);
          m_setpoint.y = m_refmodel_x(1);
          m_setpoint.z = m_refmodel_x(2);
          m_setpoint.u = vel(0);
          m_setpoint.v = vel(1);
          m_setpoint.w = vel(2);


          if (m_args.delayed_feedback)
          {
            //m_setpoint.x += Gd*l;
          }



          // Todo: Add seperate altitude controller.
          ///m_setpoint.w = 0;
          //m_setpoint.z = m_args.ref_down;

          m_setpoint.flags = IMC::TranslationalSetpoint::FL_SURGE | IMC::TranslationalSetpoint::FL_SWAY | IMC::TranslationalSetpoint::FL_HEAVE |
              IMC::TranslationalSetpoint::FL_X | IMC::TranslationalSetpoint::FL_Y | IMC::TranslationalSetpoint::FL_Z;

          dispatch(m_setpoint);
       //   debug("dispatched m_setpoint -> TranslationalSetpoint \n.");

          debug("[x, y, z] = [%f, %f, %f] \n",m_setpoint.x,m_setpoint.y,m_setpoint.z);
          debug("[u, v, w] = [%f, %f, %f] \n",m_setpoint.u,m_setpoint.v,m_setpoint.w);


        }

        //!  Rotation matrix from Body to NED

        //! @return  Rotation matrix.
        Matrix Rbn(double phi, double theta, double psi) const
        {
          double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(psi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
              sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
              -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};
          return Matrix(R_en_elements,3,3);
        }
        //! Print matrix (for debuging)
        void
        prntMatrix(Matrix m){

          for(int i = 0; i<m.rows(); i++ ){
            for(int j = 0; j<m.columns();j++){
              printf("%1.15f ", m.element(i,j));
            }
            printf("\n");
          }
        }

      };
    }
  }
}

DUNE_TASK

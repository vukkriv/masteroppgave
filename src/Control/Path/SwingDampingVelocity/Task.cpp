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
    namespace SwingDampingVelocity
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        bool use_controller;
        double max_speed;
        bool use_refmodel;
        double refmodel_max_speed;
        double refmodel_omega_n;
        double refmodel_xi;
        double Kp;
        bool use_altitude;
        bool disable_heave;
        double pL;  // wire length
        bool use_delayed_feedback;
        double Gd; // Delayed feedback control parameter
        double tau_n; // Delayed feedback control parameter
        bool static_delayed_feedback;
        int EulerAngle_samplesize;
        int offset;
        bool use_inputshaper_zv;
        int inputZV_time_offset;
      };

      struct Task: public DUNE::Control::PathController
      {
        double m_last_estate_phi;
        double m_last_estate_theta;
        double m_last_estate_psi;
        IMC::TranslationalSetpoint m_translational_setpoint;
        std::vector<double> m_phi;
        std::vector<double> m_theta;
        std::vector<double> m_timestamp;
        Matrix m_load_pos_ned; // load position in NED frame
        // Last LoadAngles
        Matrix m_load_z_vect;
        Matrix m_delayed_pos;
        Matrix m_delayed_vel;
        Matrix m_static_ref;
        double m_delayed_phi_LN; // phi on load, given in NED
        double m_delayed_theta_LN; //theta on load, given in NED
        double m_delayed_timestamp_LN;
        double m_delayed_phi_LN_dot;
        double m_delayed_theta_LN_dot;
        bool m_delayed_euler_data;
        double m_Td;
        double m_tau_d;
        double m_input_A1;
        double m_input_A2;
        double m_input_t2;
        Matrix m_last_refmodel_pos_t2;
        Matrix m_last_refmodel_vel_t2;
        Matrix m_input_pos;
        Matrix m_input_vel;
        Matrix m_temp_pos;
        Matrix m_temp_vel;
        std::vector<double> m_last_refmodel_t2_x;
        std::vector<double> m_last_refmodel_t2_y;
        std::vector<double> m_last_refmodel_t2_z;
        std::vector<double> m_last_refmodel_t2_u;
        std::vector<double> m_last_refmodel_t2_v;
        std::vector<double> m_last_refmodel_t2_w;
        std::vector<double> m_timetracker_t2;
        IMC::DesiredVelocity m_velocity;
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
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Control::PathController(name, ctx),
          m_desired_speed(0)
        {
          param("Velocity Controller", m_args.use_controller)
                              .visibility(Tasks::Parameter::VISIBILITY_USER)
                              .scope(Tasks::Parameter::SCOPE_MANEUVER)
                              .defaultValue("false")
                              .description("Enable Velocity Controller");

          param("Max Speed", m_args.max_speed)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max speed of the vehicle");

          param("Use Reference Model", m_args.use_refmodel)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable Reference Model.");

          param("Reference Model - Max Speed", m_args.refmodel_max_speed)
          .defaultValue("3.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max speed of the reference model.");

          param("Reference Model - Natural Frequency",m_args.refmodel_omega_n)
          .units(Units::RadianPerSecond)
          .defaultValue("0.1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Natural frequency for the speed reference model");

          param("Reference Model - Relative Damping", m_args.refmodel_xi)
          .units(Units::None)
          .defaultValue("0.9")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Relative Damping Factor of the speed reference model");

          param("Velocity Controller - Kp", m_args.Kp)
          .units(Units::None)
          .defaultValue("0.1")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("P-Gain of the velocity controller");

          param("Use Altitude", m_args.use_altitude)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether altitude is controlled or not (set to 0 if not).");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          param("Suspended wire length - pL", m_args.pL)
          .defaultValue("0.7")
          .units(Units::Meter)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Suspended wire length");

          param("Use Delayed Feedback", m_args.use_delayed_feedback)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether delayed feedback control is on (set to 0 if not).");

          param("Delayed Controller - Gd", m_args.Gd)
          .units(Units::None)
          .defaultValue("0.325")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Optimized gain of the delayed feedback controller");

          param("Delayed Controller - tau_n", m_args.tau_n)
          .units(Units::None)
          .defaultValue("0.325")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Optimized gain of the delayed feedback controller");

          param("Delayed Feedback - static", m_args.static_delayed_feedback)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether static delayed feedback control is on(set to 0 if not).");

          param("EulerAngle_samplesize", m_args.EulerAngle_samplesize)
          .defaultValue("50")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Size of the remembered last samples of Euler Angles.");

          param("Offset - Delayed tau", m_args.offset)
          .defaultValue("0")
          .units(Units::Millisecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Offsett for delayed tau");

          param("Input Shaper - ZV", m_args.use_inputshaper_zv)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether input shaper is on(set to 0 if not).");

          param("InputZV - Offset", m_args.inputZV_time_offset)
          .defaultValue("0")
          .units(Units::Millisecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Offsett for inputshaper time impulse");

          bind<IMC::EulerAngles>(this);
        }

        void
        onUpdateParameters(void)
        {
          PathController::onUpdateParameters();

          // update desired speed to max speed
          m_desired_speed = m_args.refmodel_max_speed;
          // update wire length.
          m_load_z_vect = Matrix(3,1,0);
          m_load_z_vect(2) = m_args.pL;

          double pm_L = 2;
          double pd =  0.01;
          double pg = 9.81;
          double omega_n = sqrt(pg/m_args.pL);
          double xi = pd/(2*omega_n*pm_L);
          double omega_d = omega_n*sqrt(1 - pow(xi,2));
          m_Td = 2*Math::c_pi/omega_d;


          //iput shaper values
          double input_K = exp(-(xi*Math::c_pi)/sqrt(1-pow(xi,2)));
          m_input_A1 = 1/(1 + input_K);
          m_input_A2 = input_K/(1 + input_K);
          m_input_t2 = m_Td/2;

        }

        void
        onEntityReservation(void)
        {
          PathController::onEntityReservation();
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

        //! Consume EulerAngles and save the rotated and delayed angles for vel controller
        void
        consume(const IMC::EulerAngles* eangles)
        {
          trace("Euler angle consume started.../n");
          // Load position in NED frame.
          m_load_pos_ned = Rzyx(m_last_estate_phi,m_last_estate_theta,m_last_estate_psi) *
              Rx(eangles->phi)*Ry(eangles->theta) * m_load_z_vect;

          // Insert new element at the end
          m_phi.push_back (atan(m_load_pos_ned(1)/m_load_pos_ned(2)));
          m_theta.push_back (atan(m_load_pos_ned(0)/m_load_pos_ned(2)));
          m_timestamp.push_back (eangles->getTimeStamp());

          // Delayed time
          m_tau_d = m_args.tau_n * m_Td;

          // Delete oldest element in vector (first one), after reaching m_tau_d seconds + some offsett
          if ( (m_timestamp.back() - m_timestamp.front()) >= (m_tau_d + (m_args.offset / 1000.0)) )
          {
            m_delayed_euler_data =  true;
            // most likely the closest
            m_delayed_phi_LN = m_phi.front();
            m_delayed_theta_LN = m_theta.front();
            m_delayed_timestamp_LN = m_timestamp.front();



            m_phi.erase(m_phi.begin());
            m_theta.erase(m_theta.begin());
            m_timestamp.erase(m_timestamp.begin());

            /*            // Check if the sample before was better
            if ( abs((m_timestamp.back() - m_timestamp.front()) - (m_tau_d + (m_args.offset / 1000.0))) < abs(m_delayed_timestamp_LN - (m_tau_d + (m_args.offset / 1000.0))))
            {
              m_delayed_phi_LN = m_phi.front();
              m_delayed_theta_LN = m_theta.front();
              m_delayed_timestamp_LN = m_timestamp.front();

              m_phi.erase(m_phi.begin());
              m_theta.erase(m_theta.begin());
              m_timestamp.erase(m_timestamp.begin());
            }*/

            // save the derivative
            m_delayed_phi_LN_dot = (m_phi.at(1)-m_delayed_phi_LN) / (m_timestamp.at(1)-m_delayed_timestamp_LN);
            m_delayed_theta_LN_dot = (m_theta.at(1)-m_delayed_theta_LN) / (m_timestamp.at(1)-m_delayed_timestamp_LN);

            // sanity check on derivative
            if (m_delayed_phi_LN_dot > 10.0)
              m_delayed_phi_LN_dot = 10.0;
            if (m_delayed_theta_LN_dot > 10.0)
              m_delayed_theta_LN_dot = 10.0;

            trace("Euler angle consume finished whitout error.../n");
          }

          // If no new angles consumed for a long time and restarts, reset the vectors
          if (m_timestamp.back() - m_timestamp.front() > 2.0)
          {
            inf("No new Euler Angles");
            m_phi.resize(0);
            m_theta.resize(0);
            m_timestamp.resize(0);
          }

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

          m_refmodel_x(3) = state.vx;
          m_refmodel_x(4) = state.vy;
          m_refmodel_x(5) = state.vz;

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
        void
        initDelayedFeedback()
        {
          // Restart pos model
          m_delayed_pos = Matrix(3, 1, 0.0);
          // Restart vel model
          m_delayed_vel = Matrix(3, 1, 0.0);
          m_static_ref = Matrix(3,1,0.0);
          m_delayed_euler_data =  false;
        }

        void
        initInputShaperZV()
        {
          m_input_pos =  Matrix(3,1,0.0);
          m_input_vel =  Matrix(3,1,0.0);
          m_last_refmodel_pos_t2 = Matrix(3,1,0.0);
          m_last_refmodel_vel_t2 =Matrix(3,1,0.0);
        }

        virtual void
        onPathStartup(const IMC::EstimatedState& state, const TrackingState& ts)
        {

          //(void)ts;


          // Print end coordinates
          debug("End coordinates: [%f, %f, %f]", ts.end.x, ts.end.y, ts.end.z);

          // Restart ref model
          initRefmodel(state);

          // Restart delayed feedback parameter
          initDelayedFeedback();

          // Restart input shaper parameters
          initInputShaperZV();


        }

        virtual void
        onPathActivation(void)
        {
          if (!m_args.use_controller)
          {
            debug("Path activated, but not active: Requesting deactivation");
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

          // Save the most current attitude
          m_last_estate_phi = state.phi;
          m_last_estate_theta = state.theta;
          m_last_estate_psi = state.psi;

          if (!m_args.use_controller)
            return;

          // Get current position
          Matrix x = Matrix(3, 1, 0.0);
          x(0) = state.x;
          x(1) = state.y;
          x(2) = state.z;
          trace("x:\t [%1.2f, %1.2f, %1.2f]",
              state.x, state.y, state.z);

          // Get target position
          Matrix x_d = Matrix(3, 1, 0.0);
          x_d(0) = ts.end.x;
          x_d(1) = ts.end.y;
          x_d(2) = -ts.end.z; // NEU?!
          trace("x_d:\t [%1.2f, %1.2f, %1.2f]",
              x_d(0), x_d(1), x_d(2));

          Matrix vel = Matrix(3,1, 0.0);

          if (m_args.use_refmodel)
          {
            // Update reference
            m_refmodel_x += ts.delta * (m_refmodel_A * m_refmodel_x + m_refmodel_B * x_d);




            // Saturate reference velocity
            vel = m_refmodel_x.get(3,5,0,0);

            // Set heave to 0 if not controlling altitude
            if (!m_args.use_altitude)
            {
              vel(2) = 0;
            }

            if( vel.norm_2() > m_args.refmodel_max_speed )
            {
              vel = m_args.refmodel_max_speed * vel / vel.norm_2();
              m_refmodel_x.put(3,0,vel);
            }
            spew("Vel norm: %f", m_refmodel_x.get(3,5,0,0).norm_2());

            // Print reference pos and vel
            trace("x_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(0), m_refmodel_x(1), m_refmodel_x(2));
            trace("v_r:\t [%1.2f, %1.2f, %1.2f]",
                m_refmodel_x(3), m_refmodel_x(4), m_refmodel_x(5));

            // Head straight to reference
            //vel(0) = m_refmodel_x(3) - m_args.Kp * (state.x - m_refmodel_x(0));
            //vel(1) = m_refmodel_x(4) - m_args.Kp * (state.y - m_refmodel_x(1));
            //vel(2) = m_refmodel_x(5) - m_args.Kp * (state.z - m_refmodel_x(2));

            // input shaping controller - convolution hack!
            if (m_args.use_inputshaper_zv)
            {
              m_input_pos =  Matrix(3,1,0.0);
              m_input_vel =  Matrix(3,1,0.0);
              m_last_refmodel_pos_t2 = Matrix(3,1,0.0);
              m_last_refmodel_vel_t2 = Matrix(3,1,0.0);

              // Dont do anything on the Z reference
              m_input_pos(0) =  m_refmodel_x(0) * m_input_A1;
              m_input_pos(1) =  m_refmodel_x(1) * m_input_A1;
              m_input_vel(0) = m_refmodel_x(3) * m_input_A1;
              m_input_vel(1) = m_refmodel_x(4) * m_input_A1;


              // Save the lastest reference model values
              m_last_refmodel_t2_x.push_back (m_input_pos(0));
              m_last_refmodel_t2_y.push_back (m_input_pos(1));
              m_last_refmodel_t2_z.push_back (m_input_pos(2));
              m_last_refmodel_t2_u.push_back (m_input_vel(0));
              m_last_refmodel_t2_v.push_back (m_input_vel(1));
              m_last_refmodel_t2_w.push_back (m_input_vel(2));
              m_timetracker_t2.push_back(state.getTimeStamp());


              // Save the new ref mdoel values ones until t2
              if (m_timetracker_t2.back() - m_timetracker_t2.front() >= m_input_t2 + (m_args.inputZV_time_offset/1000.0))
              {
                // The closest value of refmodel for t2 seconds ago
                m_last_refmodel_pos_t2(0) = m_last_refmodel_t2_x.front();
                m_last_refmodel_pos_t2(1) = m_last_refmodel_t2_y.front();
                m_last_refmodel_pos_t2(2) = m_last_refmodel_t2_z.front();
                m_last_refmodel_vel_t2(0) = m_last_refmodel_t2_u.front();
                m_last_refmodel_vel_t2(1) = m_last_refmodel_t2_v.front();
                m_last_refmodel_vel_t2(2) = m_last_refmodel_t2_w.front();


                // Add A2 to convulution using the ref model values at t2 -> now-t2
                //m_input_pos = m_input_pos + (m_last_refmodel_pos_t2 * m_input_A2);
                // m_input_vel = m_input_vel + (m_last_refmodel_vel_t2 * m_input_A2);

                inf("m_timetracker_t2.back() - m_timetracker_t2.front() %f \n",m_timetracker_t2.back() - m_timetracker_t2.front());
                m_input_pos(0) = m_input_pos(0) + m_last_refmodel_pos_t2(0) * m_input_A2;
                m_input_pos(1) = m_input_pos(1) + m_last_refmodel_pos_t2(1) * m_input_A2;

                m_input_vel(0) = m_input_vel(0) + m_last_refmodel_vel_t2(0) * m_input_A2;
                m_input_vel(1) = m_input_vel(1) + m_last_refmodel_vel_t2(1) * m_input_A2;


                // Delete the oldest parameter
                m_last_refmodel_t2_x.erase(m_last_refmodel_t2_x.begin());
                m_last_refmodel_t2_y.erase(m_last_refmodel_t2_y.begin());
                m_last_refmodel_t2_z.erase(m_last_refmodel_t2_z.begin());
                m_last_refmodel_t2_u.erase(m_last_refmodel_t2_u.begin());
                m_last_refmodel_t2_v.erase(m_last_refmodel_t2_v.begin());
                m_last_refmodel_t2_w.erase(m_last_refmodel_t2_w.begin());
                m_timetracker_t2.erase(m_timetracker_t2.begin());
              }

              m_input_pos(2) = m_refmodel_x(2);
              m_input_vel(2) = m_refmodel_x(5);



              // m_refmodel_x.put(0,0,m_input_pos);
              // m_refmodel_x.put(3,0,m_input_vel);

              spew("m_input_A1: %f \n", m_input_A1);
              spew("m_input_A2: %f \n", m_input_A2);
              spew("m_input_t2: %f \n", m_input_t2);

            }




            vel = m_refmodel_x.get(3,5,0,0) + m_args.Kp * (m_refmodel_x.get(0,2,0,0) - x);

            if (m_args.use_inputshaper_zv)
            {
              m_temp_pos = Matrix(3,1,0.0);
              m_temp_pos = m_input_pos.get(0,2,0,0);
              m_temp_vel = Matrix(3,1,0.0);
              m_temp_vel = m_input_vel.get(0,2,0,0);
              vel = m_temp_vel + m_args.Kp * (m_temp_pos - x);
            }


            if (m_args.use_delayed_feedback)
            {

              m_tau_d = m_args.tau_n * m_Td;


              // If last EulerAngle_samplesize of Euler Angles are stored
              if (m_delayed_euler_data)
              {

                m_delayed_pos(0) = m_args.Gd*m_args.pL*sin(m_delayed_theta_LN);
                m_delayed_pos(1) = m_args.Gd*m_args.pL*sin(m_delayed_phi_LN);

                m_delayed_vel(0) = m_args.Gd*m_args.pL*cos(m_delayed_theta_LN)*m_delayed_theta_LN_dot;
                m_delayed_vel(1) = m_args.Gd*m_args.pL*cos(m_delayed_phi_LN)*m_delayed_phi_LN_dot;

                // delayed feedback control
                vel = (m_refmodel_x.get(3,5,0,0) + m_delayed_vel) + m_args.Kp * ((m_refmodel_x.get(0,2,0,0) + m_delayed_pos) - x);


                // If vehicle is standing still
                if (m_args.static_delayed_feedback)
                {
                  m_static_ref(0) = ts.start.x;
                  m_static_ref(1) = ts.start.y;
                  m_static_ref(2) = ts.start.z;
                  vel = m_delayed_vel + m_args.Kp * ((m_static_ref + m_delayed_pos) - x);
                }
              }
              spew("m_tau_d: %f, \n ",m_tau_d);
              spew("m_phi_LN: %f, \n m_theta_LN: %f, \n",m_delayed_phi_LN,m_delayed_theta_LN);
              spew("m_phi_LN_dot: %f, \n m_theta_LN_fot: %f, \n",m_delayed_phi_LN_dot,m_delayed_theta_LN_dot);
              spew("m_delayed_pos(0): %f, m_delayed_pos(1): %f, \n",m_delayed_pos(0),m_delayed_pos(1));
              spew("m_delayed_vel(0): %f, m_delayed_vel(1): %f, \n",m_delayed_vel(0),m_delayed_vel(1));
            }
          }
          else
          {
            // Head straight to target
            //vel(0) = - m_args.Kp * (state.x - x_d(0));
            //vel(1) = - m_args.Kp * (state.y - x_d(1));
            //vel(2) = - m_args.Kp * (state.z - x_d(2));
            vel = m_args.Kp * (x_d - x);
          }

          // Set heave to 0 if not controlling altitude
          if (!m_args.use_altitude)
          {
            vel(2) = 0;
          }

          // Saturate velocity
          if( vel.norm_2() > m_args.max_speed )
          {
            vel = m_args.max_speed * vel / vel.norm_2();
          }

          m_velocity.u = vel(0);
          m_velocity.v = vel(1);
          m_velocity.w = vel(2);

          // Print desired velocity
          trace("v_d:\t [%1.2f, %1.2f, %1.2f]",
              m_velocity.u, m_velocity.v, m_velocity.w);

          // Todo: Add seperate altitude controller.

          m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY | IMC::DesiredVelocity::FL_HEAVE;

          if(m_args.disable_heave)
            m_velocity.flags = IMC::DesiredVelocity::FL_SURGE | IMC::DesiredVelocity::FL_SWAY;


          // Kristian Test: Dispatch linear setpoint for logging
          /*
                      IMC::TranslationalSetpoint setpoint;
                      setpoint.x = m_refmodel_x(0);
                      setpoint.y = m_refmodel_x(1);
                      setpoint.z = m_refmodel_x(2);
                      setpoint.u = m_refmodel_x(3);
                      setpoint.v = m_refmodel_x(4);
                      setpoint.w = m_refmodel_x(5);
                      dispatch(setpoint);
           */

          dispatch(m_velocity);
          spew("Sent vel data.");
        }
        //! @return  Rotation matrix.
        Matrix Rzyx(double phi, double theta, double psi) const
        {
          double R_en_elements[] = {cos(psi)*cos(theta), (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(psi)), (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)) ,
              sin(psi)*cos(theta), (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)), (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)),
              -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)};
          return Matrix(R_en_elements,3,3);
        }
        Matrix Rx(double phi) const{
          double Rx_elements[] = {1.0, 0.0, 0.0, 0.0, cos(phi), -sin(phi), 0.0, sin(phi), cos(phi)};
          return Matrix(Rx_elements,3,3);
        }
        Matrix Ry(double theta) const{
          double Ry_elements[] = {cos(theta), 0.0, sin(theta), 0.0, 1.0, 0.0, -sin(theta), 0.0, cos(theta)};
          return Matrix(Ry_elements,3,3);
        }
      };
    }
  }
}

DUNE_TASK

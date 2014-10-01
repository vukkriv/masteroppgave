//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian                                                         *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "MulticopterModel.hpp"

namespace Simulators
{
  namespace Multicopter
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      double mass;
      double hover_throttle; // 0.45;
      double k; // pitch/roll motor gain
      double l; // Length from centre of multicopter to motor
      double b; // Yaw coefficient

      bool linear_actuator_dynamics;

      std::string frame; // Frame configuratin
      std::string configuration;


      Math::Matrix inertia;
      Math::Matrix ldrag;

      bool tuning_mode;

    };

    struct Task : public Tasks::Periodic
    {
      //! Simulation vehicle.
      MulticopterModel* m_model;
      //! Simulated position (X,Y,Z).
      IMC::SimulatedState m_sstate;
      IMC::EstimatedState m_estate;
      //! Accelleration
      IMC::Acceleration m_acc;
      //! Start time.
      double m_start_time;
      //! Last time update was ran
      double m_last_update;
      //! Set Servo positions
      Matrix m_servo_speed;
      //! Vehicle position
      Matrix m_position;
      //! Vehicle velocity vector
      Matrix m_velocity;
      //! Task arguments.
      Arguments m_args;
      //! Set true if landed
      bool m_on_ground;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx),
        m_model(NULL),
        m_start_time(Clock::get()),
        m_last_update(Clock::get()),
        m_servo_speed(8, 1, 0.0), // max 8 servos.
        m_position(6, 1, 0.0),
        m_velocity(6, 1, 0.0),
        m_on_ground(true)
      {
        // init positions

        // at trondheim
        m_sstate.lat = Angles::radians(63.45515289);
        m_sstate.lon = Angles::radians(10.91983723);

        // USA
        m_sstate.lat = Angles::radians(37.61);
        m_sstate.lon = Angles::radians(-122.38);
        m_sstate.height = 0;

        param("Mass", m_args.mass)
        .defaultValue("3.0")
        .units(Units::Kilogram)
        .description("Mass of the vehicle");

        param("Hover Throttle", m_args.hover_throttle)
        .defaultValue("0.5")
        .description("Throttle to keep the copter floating");

        param("k", m_args.k)
        .defaultValue("3")
        .description("Pitch/roll motor coefficient");

        param("l", m_args.l)
        .defaultValue("0.25")
        .description("Length from centre to motor");

        param("b", m_args.b)
        .defaultValue("1")
        .description("Yaw motor coefficient");

        param("Linear Actuator Dynamics", m_args.linear_actuator_dynamics)
        .defaultValue("False")
        .description("Use linear simplified actuator dynamics");

        param("Frame Type", m_args.frame)
        .values("quad,hex")
        .defaultValue("quad")
        .description("Sets frame type. (quad/hex)");

        param("Configuration", m_args.configuration)
        .values("+,x")
        .defaultValue("+")
        .description("Set frame configuration, +/x");

        param("Inertia", m_args.inertia)
        .defaultValue("")
        .description("Inertia of the vehicle (3 elements of main diagonal)");

        param("Linear Drag", m_args.ldrag)
        .defaultValue("")
        .description("Linear drag of the vehicle (6 elements of main diagonal)");

        param("Tuning Mode", m_args.tuning_mode)
        .defaultValue("False")
        .description("Set to true to lock the copter in position, and only respect changes to roll and yaw");


        bind<IMC::SetPWM>(this);

        // Set OK status
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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
        // Run inits!

        double inertia[3];
        for (int i = 0; i < 3; i++)
        {
          inertia[i] = m_args.inertia(i);
        }

        double ldrag[6];
        for (int i = 0; i < 6; i++)
        {
          ldrag[i] = m_args.ldrag(i);
        }



        Simulators::Multicopter::MulticopterModelParameters par;
        par.mass = m_args.mass;
        par.hover_throttle = m_args.hover_throttle;
        par.k = m_args.k;
        par.l = m_args.l;
        par.b = m_args.b;
        par.linear_actuator_dynamics = m_args.linear_actuator_dynamics;
        par.frame = (m_args.frame == "quad") ? Frame_quad : Frame_hexa;
        par.configuration = (m_args.configuration == "+") ? Configuration_plus : Configuration_x;
        par.inertia = Matrix(inertia, 3);
        par.ldrag = Matrix(ldrag, 6);


        if (par.frame == Frame_quad)
          inf("Got quad");
        if (par.frame == Frame_hexa)
          inf("Got hexacopter");
        if (par.configuration == Configuration_plus)
          inf("Got plus");
        if (par.configuration == Configuration_x)
          inf("Got X");

        inf("Starting with: k: %f", m_args.k);

        m_model = new MulticopterModel(par);

        inf(DTR("Multicopter simulation started."));
        if(m_args.tuning_mode)
        {
          inf("Starting in Tuning Mode");
        }
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
        // Release model
        Memory::clear(m_model);
      }



      Matrix
      matrixJ(float roll, float pitch, float yaw)
      {
        double cx_elements[9] = { 1, 0, 0,
                                  0, std::cos(roll), std::sin(roll),
                                  0, -std::sin(roll), std::cos(roll) };

        double cy_elements[9] = { std::cos(pitch), 0, -std::sin(pitch),
                                  0, 1, 0,
                                  std::sin(pitch), 0, std::cos(pitch) };

        double cz_elements[9] = { std::cos(yaw), std::sin(yaw), 0,
                                  -std::sin(yaw), std::cos(yaw), 0,
                                  0, 0, 1 };

        Matrix J1 = transpose(Matrix(cz_elements, 3, 3)) * transpose(Matrix(cy_elements, 3, 3)) * transpose(Matrix(cx_elements, 3, 3));

        double j2_elements[9] = { 1, std::sin(roll) * std::tan(pitch), std::cos(roll) * std::tan(pitch),
                                  0, std::cos(roll), -std::sin(roll),
                                  0, std::sin(roll) / std::cos(pitch), std::cos(roll) / std::cos(pitch) };

        J1.vertCat(Matrix(3, 3, 0.0));
        Matrix cols456 = Matrix(3, 3, 0.0);
        cols456.vertCat(Matrix(j2_elements, 3, 3));
        J1.horzCat(cols456);

        return J1;
      }

      void
      consume(const IMC::SetPWM* msg)
      {
        if (resolveEntity(msg->getSourceEntity()) == "Sitl Layer")
        {
          int id = 0;
          bool got_relevant_message = true;

          // This implements the strange mapping APM has to motors.
          // Also remember channels are 1-indexed.
          // [1:1, 2:2, 3:3, 4:4, 5:7, 6:8, 7:10, 8:11]

          /*
        // Note: Intentionally skipping chan. 5,6 and 9.
        m_servo_speed(0) = msg->chan1;
        m_servo_speed(1) = msg->chan2;
        m_servo_speed(2) = msg->chan3;
        m_servo_speed(3) = msg->chan4;
        m_servo_speed(4) = msg->chan7;
        m_servo_speed(5) = msg->chan8;
        m_servo_speed(6) = msg->chan10;
        m_servo_speed(7) = msg->chan11;
           *
           */

          switch (msg->id)
          {
          default:
            got_relevant_message = false;
            break;
          case 1:
          case 2:
          case 3:
          case 4:
            id = msg->id;
            break;
          case 7:
            id = 5;
            break;
          case 8:
            id = 6;
            break;
          case 10:
            id = 7;
            break;
          case 11:
            id = 8;
            break;
          }

          if (got_relevant_message)
          {
            // this is zero-indexed.
            m_servo_speed(id - 1) = (msg->duty_cycle - 1000) / 1000.0;
          }
        }
        else
        {
          trace(DTR("Got a SetPWM message from another source. Ignoring."));
        }
      }

      void
      task(void)
      {
        bool was_on_ground = m_on_ground;

        // Test: Spew some rc motors!
        Matrix moments = m_model->computeTau(m_servo_speed);
        try{

          debug("Moments: %f, %f; %f", moments.element(3), moments.element(4), moments.element(5));
        }
        catch(DUNE::Math::Matrix::Error& ex)
        {
          inf("nope.. %d, %s", moments.size(), ex.what());
        }

        // compute the timestep
        double timestep = Clock::get() - m_last_update;
        m_last_update = Clock::get();

        if (timestep > 2*(1/this->getFrequency()) )
        {
          debug("Warning: Missed time. Should be %f, was %f", 1/this->getFrequency(), timestep);
        }

        // Find the derivative of the position in the earth fixed frame
        Matrix dposition(6, 1, 0.0);
        dposition = matrixJ(m_position(3), m_position(4), m_position(5)) * m_velocity;

        // Integrate using Euler method
        m_position += timestep * dposition;

        Matrix accel;
        accel = m_model->stepInv(m_servo_speed, m_velocity, m_position);
        trace("Resulting z-acc: %f", accel(2));

        // TODO: Add more sophisticated ground behaviour
        // Remember, we use a NED-convention, so z is negative when we are in the air
        spew("Moving at z: %f, height: %f", m_velocity(2), m_position(2));
        /*
        if (m_position(2) > 0 && !m_on_ground)
        {
          trace(DTR("On ground!"));
          // Set position on ground
          m_position(2) = 0;


          m_on_ground = true;

          // If we are moving downwards, positive z in the ned-frame
          fp32_t vx, vy, vz;
          BodyFixedFrame::toInertialFrame(m_position(3), m_position(4), m_position(5),
                                          m_velocity(0), m_velocity(1), m_velocity(2),
                                          &vx, &vy, &vz);

          if (vz > 0)
          {
            // create simple bouncing-feature
            //vz = -0.3 * vz;

            trace(DTR("Bounce! (disabled. )"));

            // Update body-velocities
            BodyFixedFrame::toBodyFrame(m_position(3), m_position(4), m_position(5),
                                        vx, vy, vz,
                                        &m_velocity(0), &m_velocity(1), &m_velocity(2));



          }
        }
        */

        // Okei. The idea is.
        // If a negative z is detected, set on ground. if we get accell upwards, disable ground

        if ( !was_on_ground && m_position(2) > 0 ) {
          m_on_ground = true;
          inf("On Ground!");
        }

        if ( was_on_ground && accel(2) < 0 )
        {
          inf("Liftoff!");
          m_on_ground = false;
        }

        if (m_on_ground)
        {
          spew("I am on ground, setting stuff to zero.");
          // Set stuff to zero
          m_position(2) = 0;
          m_position(3) = 0;
          m_position(4) = 0;

          // rates
          m_velocity = Matrix(6,1,0.0);

          // accel
          accel = Matrix(6,1,0.0);
        }





        // Compute velocity in the vehicle frame that will be used in the next iteration
        m_velocity += timestep * accel;

        // If tuning mode, lock position
        if(m_args.tuning_mode)
        {
          m_position(0) = 0;
          m_position(1) = 0;
          m_position(2) = -10;
          m_position(4) = 0;

          m_velocity(0) = 0;
          m_velocity(1) = 0;
          m_velocity(2) = 0;
          m_velocity(4) = 0;
        }

        // Fill position.
        double sim_time = Clock::get() - m_start_time;
        (void)sim_time;
        m_sstate.x = m_position(0); // + sim_time * m_args.wx;
        m_sstate.y = m_position(1); // + sim_time * m_args.wy;
        m_sstate.z = m_position(2);

        // Fill attitude.
        m_sstate.phi = m_position(3);
        m_sstate.theta = m_position(4);
        //m_position(5) = Angles::normalizeRadian(m_position(5));
        m_sstate.psi = Angles::normalizeRadian(m_position(5));

        // Fill linear velocity.
        m_sstate.u = m_velocity(0);
        m_sstate.v = m_velocity(1);
        m_sstate.w = m_velocity(2);

        // Fill angular velocity.
        m_sstate.p = m_velocity(3);
        m_sstate.q = m_velocity(4);
        m_sstate.r = m_velocity(5);

        // Fill acceleration
        // Remember, accel is body derived acceleration.
        // Needs to be converted to inertia differentiated acceleration, represented in body
        // Matrix Rnb = matrixRnb(m_sstate.phi, m_sstate.theta, m_sstate.psi);

        Matrix ddp_b = (skew(m_velocity.get(3,5,0,0)) * m_velocity.get(0,2,0,0) + accel.get(0,2,0,0));
        m_acc.x = ddp_b(0);
        m_acc.y = ddp_b(1);
        m_acc.z = ddp_b(2);

        dispatch(m_acc);
        dispatch(m_sstate);
      }
    };
  }
}

DUNE_TASK

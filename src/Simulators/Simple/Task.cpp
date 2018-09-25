//***************************************************************************
// Copyright 2007-2018 Norwegian University of Science and Technology (NTNU)*
// Centre for Autonomous Marine Operations and Systems (AMOS)               *
// Department of Engineering Cybernetics (ITK)                              *
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
// Author: Jon-Håkon Bøe Røli                                               *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.


namespace Simulators
{
  namespace Simple
  {
    using DUNE_NAMESPACES;

    //! %Simulator type
    enum SIM_TYPE
    {
      SINGLE = 0,
      DOUBLE
    };

    struct Arguments
    {
      //! Mass
      double mass;

      //! Maximum speed
      double max_speed;

      //! Simulator type
      std::string type;

      //! Inital position (LLH)
      double lat, lon, height;

      //! Initial position (NED)
      Math::Matrix pos_ned;

      //! Use DesiredLinearState instead of DesiredVelocity
      bool trans_setpoint;

      //! Input is acceleration
      bool force_input_is_accel;

      //! Wind drag
      double wind_drag;

      //! Wind force
      Matrix wind_speed;
    };

    struct Task : public Tasks::Periodic
    {
      //! Vehicle mass
      double m_mass;
      //! Vehicle maximum speed
      double m_max_speed;
      //! Simulated position (X,Y,Z).
      IMC::SimulatedState m_sstate;
      IMC::EstimatedState m_estate;
      //! Accelleration
      IMC::Acceleration m_acc;
      //! Start time.
      double m_start_time;
      //! Last time update was ran
      double m_last_update;
      //! Vehicle position
      Matrix m_position;
      //! Vehicle velocity vector
      Matrix m_velocity;
      //! Task arguments.
      Arguments m_args;
      //! Set true if landed
      bool m_on_ground;
      //! Simulator type
      SIM_TYPE m_type;
      //! Desired velocity
      Matrix m_desired_velocity;
      //! Desired force
      Matrix m_desired_force;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx),
        m_mass(1.0),
        m_max_speed(-1.0),
        m_start_time(Clock::get()),
        m_last_update(Clock::get()),
        m_position(3, 1, 0.0),
        m_velocity(3, 1, 0.0),
        m_on_ground(false),
        m_type(SINGLE),
        m_desired_velocity(3, 1, 0.0),
        m_desired_force(3, 1, 0.0)
      {
        // init positions

        // at trondheim
        //m_sstate.lat = Angles::radians(63.45515289);
        //m_sstate.lon = Angles::radians(10.91983723);

        // USA
        //m_sstate.lat = Angles::radians(37.61);
        //m_sstate.lon = Angles::radians(-122.38);
        //m_sstate.height = 0;

        param("Mass", m_args.mass)
        .defaultValue("4.0")
        .units(Units::Kilogram)
        .description("Mass of the vehicle");

        param("Maximum Speed", m_args.max_speed)
        .defaultValue("-1.0")
        .units(Units::MeterPerSecond)
        .description("Maximum speed of the vehicle");

        param("Type", m_args.type)
        .defaultValue("Single")
        .values("Single, Double")
        .description("Simulator Type - Single or Double integrator");

        param("Latitude", m_args.lat)
        .defaultValue("63.45515289")
        .units(Units::Degree)
        .description("Initial latitude (deg) of the vehicle");

        param("Longitude", m_args.lon)
        .defaultValue("10.91983723")
        .units(Units::Degree)
        .description("Initial longitude (deg) of the vehicle");

        param("Height", m_args.height)
        .defaultValue("120.0")
        .units(Units::Meter)
        .description("Initial height (m) above the WGS-84 ellipsoid");

        param("Initial Position", m_args.pos_ned)
        .defaultValue("0.0, 0.0, 0.0")
        .units(Units::Meter)
        .description("Initial NED position of the vehicle");

        param("Use Translational Setpoint", m_args.trans_setpoint)
        .defaultValue("false")
        .description("Use DesiredLinearState instead of DesiredVelocity for velocity control.");

        param("Input Force Is Acceleration", m_args.force_input_is_accel)
        .defaultValue("true");

        param("Wind Speed", m_args.wind_speed)
        .defaultValue("1,0,0");

        param("Wind Drag", m_args.wind_drag)
        .defaultValue("0.05");


        bind<IMC::DesiredControl>(this);
        bind<IMC::DesiredVelocity>(this);
        bind<DesiredLinearState>(this);

        // Set OK status
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Check type
        if (!m_args.type.compare("Double"))
        {
          m_type = DOUBLE;
        }
        else
        {
          m_type = SINGLE;
        }
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

        // Set mass
        m_mass = m_args.mass;

        // Set max speed
        m_max_speed = m_args.max_speed;

        // Set LLH
        m_sstate.lat = Angles::radians(m_args.lat);
        m_sstate.lon = Angles::radians(m_args.lon);
        m_sstate.height = m_args.height;

        // Set NED
        m_position = m_args.pos_ned;

        inf(DTR("Multicopter simulation started."));
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
      }


      void
      consume(const IMC::DesiredVelocity* msg)
      {
        if (m_type == SINGLE && !m_args.trans_setpoint)
        {
          m_desired_velocity(0) = msg->u;
          m_desired_velocity(1) = msg->v;
          m_desired_velocity(2) = msg->w;
        }
        spew("Received Desired Velocity [N,E,D]: [%f,%f,%f]", m_desired_velocity(0), m_desired_velocity(1), m_desired_velocity(2));
      }

      void
      consume(const IMC::DesiredLinearState* msg)
      {
        if (m_type == SINGLE && m_args.trans_setpoint)
        {
          m_desired_velocity(0) = msg->vx;
          m_desired_velocity(1) = msg->vy;
          m_desired_velocity(2) = msg->vz;
        }
        spew("Received Translational Setpoint [N,E,D]: [%f,%f,%f]", m_desired_velocity(0), m_desired_velocity(1), m_desired_velocity(2));
      }

      void
      consume(const IMC::DesiredControl* msg)
      {
        if (m_type == DOUBLE)
        {
          m_desired_force(0) = msg->x;
          m_desired_force(1) = msg->y;
          m_desired_force(2) = msg->z;

          if (m_args.force_input_is_accel)
            m_desired_force = m_desired_force * m_args.mass;
        }


      }

      void
      task(void)
      {
        bool was_on_ground = m_on_ground;

        // Compute the timestep
        double timestep = Clock::get() - m_last_update;
        m_last_update = Clock::get();

        if (timestep > 2*(1/this->getFrequency()) )
        {
          debug("Warning: Missed time. Should be %f, was %f", 1/this->getFrequency(), timestep);
        }

        // Integrate velocity to get new position
        Matrix dposition(3, 1, 0.0);
        dposition = m_velocity;
        // Integrate using Euler method
        m_position += timestep * dposition;
        spew("New position set: [%f,%f,%f]", m_position(0), m_position(1), m_position(2));

        switch (m_type)
        {
          case SINGLE:
            // Simply assume desired velocity is achieved
            m_velocity = m_desired_velocity;
            if (m_max_speed > 0 && m_velocity.norm_2() > m_max_speed)
            {
              m_velocity = m_velocity/m_velocity.norm_2()*m_max_speed;
            }
            spew("New velocity set: [%f,%f,%f]", m_velocity(0), m_velocity(1), m_velocity(2));
            break;
          case DOUBLE:
            // Integrate desired acceleration to get new velocity
            Matrix dvelocity(3, 1, 0.0);
            dvelocity = m_desired_force/m_mass - m_args.wind_drag * (m_velocity - m_args.wind_speed) / m_mass;
            // Integrate using Euler method
            m_velocity += timestep * dvelocity;

            // Fill acceleration
            m_acc.x = dvelocity(0);
            m_acc.y = dvelocity(1);
            m_acc.z = dvelocity(2) - 9.81;
            break;
        }

        // TODO: Add more sophisticated ground behaviour
        // Remember, we use a NED-convention, so z is negative when we are in the air
        spew("Moving at z: %f, height: %f", m_velocity(2), m_position(2));

        if (m_position(2) > 0 && !m_on_ground)
        {
          trace(DTR("On ground!"));
          // Set position on ground
          m_position(2) = 0;

          m_on_ground = true;

          // If we are moving downwards, positive z in the ned-frame
          if (m_velocity(2) > 0)
          {
            // create simple bouncing-feature
            //m_velocity(2) = -0.3 * m_velocity(2);

            trace(DTR("Bounce! (disabled. )"));
          }
        }

        // Okei. The idea is.
        // If a negative z is detected, set on ground. if we get accell upwards, disable ground
        if ( !was_on_ground && m_position(2) > 0 )
        {
          m_on_ground = true;
          inf("On Ground!");
        }

        if ( was_on_ground && (m_desired_force(2) < 0 || m_desired_velocity(2) < 0) )
        {
          inf("Liftoff!");
          m_on_ground = false;
        }

        if (m_on_ground)
        {
          spew("I am on ground, setting stuff to zero.");
          // Set stuff to zero
          m_position(2) = 0;

          // rates
          m_velocity = Matrix(3,1,0.0);
        }



        // Fill position.
        m_sstate.x = m_position(0);
        m_sstate.y = m_position(1);
        m_sstate.z = m_position(2);

        // Fill linear velocity.
        m_sstate.u = m_velocity(0);
        m_sstate.v = m_velocity(1);
        m_sstate.w = m_velocity(2);

        dispatch(m_sstate);
        dispatchEstimatedState();
      }

      void
      dispatchEstimatedState(void)
      {
        m_estate.lat = m_sstate.lat;
        m_estate.lon = m_sstate.lon;
        m_estate.height = m_sstate.height;
        m_estate.alt = -m_sstate.z;

        m_estate.x = m_sstate.x;
        m_estate.y = m_sstate.y;
        m_estate.z = m_sstate.z;

        m_estate.u = m_sstate.u;
        m_estate.v = m_sstate.v;
        m_estate.w = m_sstate.w;

        m_estate.vx = m_sstate.u;
        m_estate.vy = m_sstate.v;
        m_estate.vz = m_sstate.w;

        Matrix groundSpeed = Matrix(3,1, 0.0);
        groundSpeed(0) = m_estate.vx;
        groundSpeed(1) = m_estate.vy;
        groundSpeed(2) = m_estate.vz;

        Matrix windSpeed = Matrix(3,1, 0.0);
        windSpeed(0) = 0;
        windSpeed(1) = 0;
        windSpeed(2) = 0;

        Matrix airSpeed = groundSpeed - windSpeed;

        IMC::IndicatedSpeed ias;
        IMC::TrueSpeed gs;

        ias.value = airSpeed.norm_2();
        gs.value  = groundSpeed.norm_2();

        dispatch(ias);
        dispatch(gs);

        dispatch(m_estate);

        if (m_type == DOUBLE)
          dispatch(m_acc);

        spew("Estimated State [N,E,D] [vN,vE,vD]: [%f,%f,%f] [%f,%f,%f]", m_estate.x, m_estate.y, m_estate.z, m_estate.u, m_estate.v, m_estate.w);
      }

    };
  }
}

DUNE_TASK

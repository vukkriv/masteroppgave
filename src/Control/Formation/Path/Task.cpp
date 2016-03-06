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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Formation
  {
    namespace Path
    {
      using DUNE_NAMESPACES;

      struct ControlArguments
      {
        bool tunedirect;
        bool use_controller;
        double Kp;
        double Ki;
        double Kd;
        double omega;
        double xi;
        double T;
      };

      struct ReferenceSimArguments
      {
        ControlArguments c_heading;
        ControlArguments c_surge;
        double max_speed;
        double max_acc;

        //! Model parameters
        double heading_T;
        double heading_K;
        double surge_m;
        double surge_d;
      };

      struct Arguments
      {
        bool use_controller;
        bool use_altitude;
        bool disable_heave;
        double max_integral;
        bool reset_to_state;
        bool reset_integral_on_path_activation;
        bool enable_refsim;
        float print_frequency;
        ReferenceSimArguments refsim;
      };

      static const std::string c_parcel_names[] = { "PID-SURGE", "PID-HEADING", "ERROR-SURGE", "ERROR-HEADING"};

      enum Parcel
      {
        PC_PID_SURGE = 0,
        PC_PID_HEADING = 1,
        PC_ERROR_SURGE = 2,
        PC_ERROR_HEADING = 3
      };

      static const int NUM_PARCELS = 4;

      enum RefState
      {
        R_SURGE   = 0,
        R_HEADING = 1
      };

      class ReferenceSimulator
      {
      public:

        ReferenceSimulator() :
          A(3, 3, 0.0),
          B(3, 2, 0.0),
          H1(2, 3, 0.0),
          H2(2, 3, 0.0),
          x_des(3, 1, 0.0),
          x_dot_des(3, 1, 0.0),
          x_ref(3, 1, 0.0),
          Kp(2, 2, 0.0),
          Ki(2, 2, 0.0),
          Kd(2, 2, 0.0)
        {

        }

        double
        getDesAcc(void)
        {
          return x_dot_des(R_SURGE);
        }

        double
        getDesSpeed(void)
        {
          return x_des(R_SURGE);
        }

        double
        getDesHeading(void)
        {
          return x_des(R_HEADING);
        }

        Matrix
        getDesXdot(Matrix& u)
        {
          return A * x_des + B * u;
        }

        Matrix
        getError(void)
        {
          return H1 * (x_ref - x_des);
        }

        Matrix
        getDError(void)
        {
          return H2 * (x_ref - x_des);
        }

        void
        setPID(double Kp_in, double Ki_in, double Kd_in, int id)
        {
          this->Kp(id, id) = Kp_in;
          this->Ki(id, id) = Ki_in;
          this->Kd(id, id) = Kd_in;
        }

        void
        setRefSpeed(double& speed)
        {
          x_ref(R_SURGE) = speed;
        }

        void
        setRefHeading(double& heading)
        {
          x_ref(R_HEADING) = heading;
        }

        void
        setRefdHeading(double& heading)
        {
          x_ref(R_HEADING + 1) = heading;
        }

        void
        setDesSpeed(double& speed)
        {
          x_des(R_SURGE) = speed;
        }

        void
        setDesAcc(double& acc)
        {
          x_dot_des(R_SURGE)= acc;
        }

        void
        setDesHeading(double& heading)
        {
          x_des(R_HEADING) = heading;
        }

        void
        setDesdHeading(double& heading)
        {
          x_des(R_HEADING + 1) = heading;
        }

      public:
        Matrix A;
        Matrix B;
        Matrix C;
        Matrix H1;
        Matrix H2;
        Matrix x_des;
        Matrix x_dot_des;
        Matrix x_ref;
        Matrix Kp;
        Matrix Ki;
        Matrix Kd;
      };

      class DesiredReference
      {
      public:
        DesiredReference() :
          x(3, 1, 0.0)
        {
        }

        Matrix
        getSpeed(void)
        {
          return x(R_SURGE);
        }

        Matrix
        getHeading(void)
        {
          return x(R_HEADING);
        }

        void
        setDesiredReference(Matrix& x_new)
        {
          x = x_new;
        }

      public:

        Matrix x;
      };

      struct Task : public DUNE::Control::PathFormationController
      {
        //! Task arguments
        Arguments m_args;

        //! Reference Model
        ReferenceSimulator m_refsim;

        //! Current integrator value
        Matrix m_integrator_value;

        //! The current reference
        DesiredReference m_desired;

        //IMC::DesiredVelocity m_desired_velocity;
        //use this one with a internal entity --> centroid body
        IMC::DesiredLinearState m_desired_linear;
        //IMC::DesiredSpeed 	 m_desired_speed;
        IMC::DesiredHeading m_desired_heading;
        //! Last Estimated Local State received
        IMC::EstimatedLocalState m_elstate;

        //! Timestamp of previous step
        double m_timestamp_prev_step;

        //! Desired speed profile
        double m_desired_speed;

        //double m_Amatrix[9];
        //double m_Bmatrix[6];

        //! Parcel array
        IMC::ControlParcel m_parcels[NUM_PARCELS];

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          PathFormationController(name, ctx),
          m_integrator_value(2,1, 0.0),
          m_timestamp_prev_step(0.0),
          m_desired_speed(0)
        {
          param("Formation Path Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Formation Path Controller.");

          param("Reference - Enable",m_args.enable_refsim)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable reference simulator output");

          param("Reference - Max Speed", m_args.refsim.max_speed)
          .defaultValue("1.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max speed of the reference simulator.");

          param("Reference - Max Acceleration", m_args.refsim.max_acc)
          .defaultValue("1.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max acceleration of the reference simulator.");


          param("Reference - Enable surge", m_args.refsim.c_surge.use_controller)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable surge reference simulator output.");

          param("Reference - Enable heading", m_args.refsim.c_heading.use_controller)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable heading reference simulator output.");

          param("Reference - Max Integral", m_args.max_integral)
          .defaultValue("20")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max integral value");

          param("Reference - Heading T", m_args.refsim.heading_T)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading model time constant.");

          param("Reference - Heading K", m_args.refsim.heading_K)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading model rudder gain.");

          param("Reference - Surge m", m_args.refsim.surge_m)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge model mass.");

          param("Reference - Surge d", m_args.refsim.surge_d)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge drag constant.");

          param("Reference - Heading Control Tune Direct", m_args.refsim.c_heading.tunedirect)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller setting PID parameters directly.");

          param("Reference - Surge Control Tune Direct", m_args.refsim.c_surge.tunedirect)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller setting PID parameters directly.");

          param("Reference - Heading Control Bandwidth", m_args.refsim.c_heading.omega)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller bandwidth.");

          param("Reference - Heading Control Damping", m_args.refsim.c_heading.xi)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller damping");

          param("Reference - Heading Control Kp", m_args.refsim.c_heading.Kp)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller gain.");

          param("Reference - Heading Control Ki", m_args.refsim.c_heading.Ki)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller integral gain.");

          param("Reference - Heading Control Kd", m_args.refsim.c_heading.Kd)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller derivative gain.");

          param("Reference - Surge Control Kp", m_args.refsim.c_surge.Kp)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller gain.");

          param("Reference - Surge Control Ki", m_args.refsim.c_surge.Ki)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller integral gain.");

          param("Reference - Surge Control Timeconstant", m_args.refsim.c_surge.T)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller damping");

          param("Use Altitude", m_args.use_altitude)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether altitude is controlled or not (set to 0 if not).");

          param("Disable Heave flag", m_args.disable_heave)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Choose whether to disable heave flag. In turn, this will utilize new rate controller on some targets");

          param("Reset integrator on path activation", m_args.reset_integral_on_path_activation)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Choose whether to reset the integrator in a path activation. ");

          param("Reset to position on path startup", m_args.reset_to_state)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Set to reset to state rather than previous ref_pos on change");

          param("Print Frequency", m_args.print_frequency)
          .defaultValue("0.0")
          .units(Units::Second)
          .description("Frequency of pos.data prints. Zero => Print on every update.");
        }

        //! Consumer for DesiredSpeed message.
        //! @param dspeed message to consume.
        void
        consume(const IMC::DesiredSpeed* dspeed)
        {
          // overloaded.
          // Update desired speed
          if (dspeed->value < m_args.refsim.max_speed)
          {
            m_desired_speed = dspeed->value;
          }
          else
          {
            m_desired_speed = m_args.refsim.max_speed;
            debug("Trying to set a speed above maximum speed. ");
          }
          m_refsim.x_ref = Matrix(3,1,0.0);
          m_refsim.setRefSpeed(m_desired_speed);
          debug("New surge reference: [%f] m/s",m_refsim.x_ref(R_SURGE));

          PathFormationController::consume(dspeed);
        }
        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          PathFormationController::onUpdateParameters();

          // update desired speed to max speed
          m_desired_speed = m_args.refsim.max_speed;

          debug("Setting new surge tuning parameters");
          double Kp_s = m_args.refsim.c_surge.Kp;
          double Ki_s = m_args.refsim.c_surge.Ki;
          double Kd_s = m_args.refsim.c_surge.Kd;
          if (!m_args.refsim.c_surge.tunedirect)
          {
            //do same thing as below
            /*
            Kp_s =
            Ki_s =
            Kd_s =
            */
          }
          m_refsim.setPID(Kp_s,Ki_s,Kd_s,static_cast<int>(R_SURGE));

          debug("Setting new heading tuning parameters");
          double Kp_h = m_args.refsim.c_heading.Kp;
          double Ki_h = m_args.refsim.c_heading.Ki;
          double Kd_h = m_args.refsim.c_heading.Kd;
          if (!m_args.refsim.c_heading.tunedirect)
          {
            Kp_h = (m_args.refsim.heading_T/m_args.refsim.heading_T)*sqrt(m_args.refsim.c_heading.omega);
            Ki_h = m_args.refsim.c_heading.omega/10;
            Kd_h = (1-2*m_args.refsim.c_heading.xi*m_args.refsim.c_heading.omega*m_args.refsim.heading_T)/m_args.refsim.heading_K;
          }
          m_refsim.setPID(Kp_h,Ki_h,Kd_h,static_cast<int>(R_HEADING));
          debug("New parameters set");
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          debug("Reserve entities");
          PathFormationController::onEntityReservation();
          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Parcel"));
          debug("Entities reserved");
        }

        void
        consume(const IMC::EstimatedLocalState* elstate)
        {
          //should be of type centroid and be local
          if (this->sourceFilter(elstate) || resolveEntity(elstate->getSourceEntity()).c_str() != m_state_entity)
            return;
          m_elstate = *elstate;
        }

        virtual void
        onPathStartup(const IMC::EstimatedLocalState& state, const TrackingState& ts)
        {
          // Print end coordinates
          debug("End coordinates: [%f, %f, %f]", ts.end.x, ts.end.y, ts.end.z);

          // Restart Reference Simulator
          initRefSim(state);

          // Reset integral
          // If switch, and always if more than x seconds since last step.
          if (m_args.reset_integral_on_path_activation || Clock::get() - m_timestamp_prev_step > 4.0)
            m_integrator_value = Matrix(2, 1, 0.0);

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
          inf("Formation path controller activated.");
        }

        void
        initRefSim(const IMC::EstimatedLocalState& state)
        {
          debug("Initialize reference simulator");
          // Restart refmodel
          m_desired.x = Matrix(3,1,0.0);
          if (m_args.reset_to_state || Clock::get() - m_timestamp_prev_step > 2.0)
          {
            m_refsim.x_ref    = Matrix(3, 1, 0.0);
            m_refsim.x_ref(0) = m_desired_speed;
            m_refsim.x_ref(1) = state.psi;
            m_refsim.x_ref(2) = state.r;

            //set the desired output to the same as the initial reference to avoid steps
            m_refsim.x_des = Matrix(3, 1, 0.0);
            m_refsim.x_des(0) = state.u;
            m_refsim.x_des(1) = state.psi;
            m_refsim.x_des(2) = state.r;
          }
          debug("Set H1 and H2 matrices");
          //H1 access [surge heading]' from m_refsim.x_#
          m_refsim.H1 = Matrix(2,3,0.0);
          m_refsim.H1(0,0) = 1;
          m_refsim.H1(1,1) = 1;
          //H2 access [0 heading_rate]' from m_refsim.x_# --> x_dot
          m_refsim.H2 = Matrix(2,3,0.0);
          m_refsim.H2(1,2) = 1;

          debug("Set reference simulator matrices");
          //set model, A matrix is dynamic and must updated each step
          setMatrixA();
          setMatrixB();
          debug("Reference simulator initialized");
        }

        void
        stepRefSim(const IMC::EstimatedLocalState& state, const TrackingState& ts)
        {
          spew("Step refsim");
          //get target speed and heading (these or the reference states should be logged somehow)

          //update system matrix with the current states
          updateMatrixA();

          spew("Get integrator value");
          Matrix e     = m_refsim.getError();
          Matrix e_dot = m_refsim.getDError();

          m_integrator_value += ts.delta*e;
          // Constrain
          if (m_integrator_value.norm_2() > m_args.max_integral)
            m_integrator_value = m_args.max_integral * m_integrator_value / m_integrator_value.norm_2();

          spew("Calculate u");

          Matrix u_p = m_refsim.Kp*e;
          Matrix u_i = m_refsim.Ki*m_integrator_value;
          Matrix u_d = m_refsim.Kd*e_dot;
          Matrix u   = u_p + u_i + u_d;
          spew("Set x_dot_des");
          m_refsim.x_dot_des = m_refsim.getDesXdot(u);
          // Integrate (Euler)
          spew("Integrate Euler");
          m_refsim.x_des += ts.delta * (m_refsim.x_dot_des);

          m_refsim.x_des(R_HEADING) = Angles::normalizeRadian(m_refsim.x_des(R_HEADING));
          spew("Step refsim done.");

          IMC::ControlParcel parcel_s = m_parcels[PC_PID_SURGE];
          parcel_s.p = u_p(0);
          parcel_s.d = u_d(0);
          parcel_s.i = u_i(0);
          dispatch(parcel_s);
          IMC::ControlParcel errors_s = m_parcels[PC_ERROR_SURGE];
          errors_s.p = e(0);
          errors_s.d = e_dot(0);
          errors_s.i = m_integrator_value(0);
          dispatch(errors_s);

          IMC::ControlParcel parcel_h = m_parcels[PC_PID_HEADING];
          parcel_h.p = u_p(1);
          parcel_h.d = u_d(1);
          parcel_h.i = u_i(1);
          dispatch(parcel_h);
          IMC::ControlParcel errors_h = m_parcels[PC_ERROR_HEADING];
          errors_h.p = e(1);
          errors_h.d = e_dot(1);
          errors_h.i = m_integrator_value(1);
          dispatch(errors_h);
        }

        void
        updateReferenceSim(const IMC::EstimatedLocalState& state, const TrackingState& ts, double now)
        {
          double ref_heading = ts.los_angle;
          m_refsim.setRefHeading(ref_heading);

          stepRefSim(state, ts);

          if (!m_args.enable_refsim)
          {
            m_refsim.x_des      = m_refsim.x_ref;
            m_refsim.x_dot_des  = Matrix(3,1,0.0);
          }

          if (!m_args.refsim.c_heading.use_controller)
          {
            m_refsim.x_des(R_HEADING)     = m_refsim.x_ref(R_HEADING);
            m_refsim.x_des(R_HEADING+1)     = m_refsim.x_ref(R_HEADING+1);
            m_refsim.x_dot_des(R_HEADING) = 0;
            m_refsim.x_dot_des(R_HEADING+1) = 0;
          }
          if (!m_args.refsim.c_surge.use_controller)
          {
            m_refsim.x_des(R_SURGE)     = m_refsim.x_ref(R_SURGE);
            m_refsim.x_dot_des(R_SURGE) = 0;
          }
            // Set reference from reference model
        	m_desired.setDesiredReference(m_refsim.x_des);
        }



        void
        step(const IMC::EstimatedLocalState& state, const TrackingState& ts)
        {
          if (!m_args.use_controller)
            return;

          double now = Clock::get();

          updateReferenceSim(state, ts, now);

          //Desired acceleration and velocity in body
          m_desired_linear.vx = m_refsim.getDesSpeed();
          m_desired_linear.vy = 0;
          m_desired_linear.vz = 0;
          //Desired acceleration in body
          m_desired_linear.ax = m_refsim.getDesAcc();
          m_desired_linear.ay = 0;
          m_desired_linear.az = 0;

          dispatch(m_desired_linear);
          //Desired heading
          m_desired_heading.value = m_refsim.getDesHeading();
          dispatch(m_desired_heading);
          //
          m_timestamp_prev_step = Clock::get();

          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            trace("Surge:\nRef: [%f]   \nDes: [%f]",m_refsim.x_ref(R_SURGE),  m_refsim.x_des(R_SURGE));
            trace("Heading:\nRef: [%f] \nDes: [%f]",m_refsim.x_ref(R_HEADING),m_refsim.x_des(R_HEADING));
            last_print = now;
          }
        }

        void
        setMatrixA()
        {
          /*
          m_Amatrix[1] = 0;
          m_Amatrix[2] = 0;
          m_Amatrix[3] = 0;
          m_Amatrix[4] = 0;
          m_Amatrix[5] = 1;
          m_Amatrix[6] = 0;
          m_Amatrix[7] = 0;
          m_Amatrix[8] = -1/m_args.refsim.heading_T;
          */
          m_refsim.A = Matrix(3,3,0.0);
          m_refsim.A(1,2) = 1;
          m_refsim.A(2,2) = -(double)1 / m_args.refsim.heading_T;
          updateMatrixA();
        }

        void
        setMatrixB()
        {
        	/*
        	m_Bmatrix[0] = 1/m_args.refsim.surge_m;
            m_Bmatrix[1] = 0;
            m_Bmatrix[2] = 0;
            m_Bmatrix[3] = 0;
            m_Bmatrix[4] = 0;
            m_Bmatrix[5] = m_args.refsim.heading_K;
            m_refsim.B = Matrix(m_Bmatrix, 2, 3);
            */
          m_refsim.B = Matrix(3,2,0.0);
        	m_refsim.B(0,0) = (double)1/m_args.refsim.surge_m;
        	m_refsim.B(2,1) = m_args.refsim.heading_K;
        }

        void
        updateMatrixA()
        {
          spew("Update A matrix");
        	m_refsim.A(0,0) = -(m_args.refsim.surge_d/m_args.refsim.surge_m)*abs(m_refsim.getDesSpeed()); //quadratic drag term
        	//m_Amatrix[0] = -(m_args.refsim.surge_d/m_args.refsim.surge_m)*abs(m_refsim.getDesSpeed()); //quadratic drag term
            //m_refsim.A = Matrix(m_Amatrix, 3, 3);
        	spew("A matrix updated");
        }


        //! @return  Rotation matrix.
        Matrix
        Rzyx(double phi, double theta, double psi) const
        {
          double R_en_elements[] =
            { cos(psi) * cos(theta), (-sin(psi) * cos(phi))
                + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi))
                + (cos(psi) * cos(phi) * sin(theta)), sin(psi) * cos(theta), (cos(
                psi) * cos(phi)) + (sin(phi) * sin(theta) * sin(psi)), (-cos(psi)
                * sin(phi)) + (sin(theta) * sin(psi) * cos(phi)), -sin(theta),
                cos(theta) * sin(phi), cos(theta) * cos(phi) };
          return Matrix(R_en_elements, 3, 3);
        }
      };
    }
  }
}

DUNE_TASK

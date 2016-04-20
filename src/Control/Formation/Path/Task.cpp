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

// USER headers.
#include <USER/DUNE.hpp>

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

      struct LosArguments
      {
        bool use_LOSangle;
        bool use_bearingHeading;
        double delta_y;
        double delta_z;
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
        LosArguments los;
      };

      struct LosControl
      {
        //Current vehicle position
        Matrix p;

        //Start WP position
        Matrix startWP;

        //Next WP position
        Matrix endWP;

        //Current along- and cross-track error
        Matrix eps;

        //Current path course (alpha) and pitching angle (theta)
        double alpha;
        double theta;

        //Current desired path course (xi_d) and pitching angle (theta_d)
        double xi_d;
        double theta_d;

        //Current reference path velocity
        Matrix v_p;

        //Current desired NED LOS velocity
        Matrix v_n_los;

        //Current reference body velocity
        Matrix v_ref;
        //Current reference body acceleration
        Matrix a_ref;

        // if receiving first waypoint
        bool first_WP;

        double last_start_z;
        double last_end_z;
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

      static const std::string c_desired_names[] = {"Reference","Desired"};
      enum DesiredEntites
      {
        D_REFERENCE = 0,
        D_DESIRED = 1
      };
      static const int NUM_DESIRED = 2;

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

        //! LOS
        LosControl m_los;

        //! The current reference
        DesiredReference m_desired;

        //! Localization origin (WGS-84)
        fp64_t m_ref_lat, m_ref_lon;
        fp32_t m_ref_hae;
        bool m_ref_valid;

        //! Current integrator value
        Matrix m_integrator_value;

        //! Reference and desired linear state, the reference message only sent for logging
        IMC::DesiredLinearState m_desired_linear[NUM_DESIRED];
        //! Reference and desired heading, the reference message only sent for logging
        IMC::DesiredHeading m_desired_heading[NUM_DESIRED];
        //! Last Estimated Local State received
        IMC::EstimatedLocalState m_elstate;
        //! Current desired Z
        IMC::DesiredZ m_dz;

        //! Timestamp of previous step
        double m_timestamp_prev_step;

        //! Desired speed profile
        double m_reference_speed;



        //double m_Amatrix[9];
        //double m_Bmatrix[6];

        //! Parcel array
        IMC::ControlParcel m_parcels[NUM_PARCELS];

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          PathFormationController(name, ctx),
          m_ref_lat(0.0),
          m_ref_lon(0.0),
          m_ref_hae(0.0),
          m_ref_valid(false),
          m_integrator_value(2,1, 0.0),
          m_timestamp_prev_step(0.0),
          m_reference_speed(0)
        {
          param("Formation Path Controller", m_args.use_controller)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Formation Path Controller.");

          param("LOS -- Lookahead NE", m_args.los.delta_y)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("1.0")
          .description("Lookahead distance for LOS control in North-East plane.");

          param("LOS -- Lookahead Down", m_args.los.delta_z)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("1.0")
          .description("Lookahead distance for LOS height control.");

          param("LOS -- Use LOS Heading", m_args.los.use_LOSangle)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable LOS heading from current position to next WP as reference heading.");

          param("LOS -- Use Bearing Heading", m_args.los.use_bearingHeading)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable bearing heading from last WP to next WP as reference heading.");

          param("RefSim--Enable",m_args.enable_refsim)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable reference simulator output");

          param("RefSim--Max Speed", m_args.refsim.max_speed)
          .defaultValue("1.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max speed of the reference simulator.");

          param("RefSim--Max Acceleration", m_args.refsim.max_acc)
          .defaultValue("1.5")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          //.scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Max acceleration of the reference simulator.");

          param("RefSim--Enable surge", m_args.refsim.c_surge.use_controller)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable surge reference simulator output.");

          param("RefSim--Enable heading", m_args.refsim.c_heading.use_controller)
          .defaultValue("True")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .description("Enable heading reference simulator output.");

          param("RefSim--Max Integral", m_args.max_integral)
          .defaultValue("20")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Max integral value");

          param("RefSim--Heading T", m_args.refsim.heading_T)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading model time constant.");

          param("RefSim--Heading K", m_args.refsim.heading_K)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading model rudder gain.");

          param("RefSim--Surge m", m_args.refsim.surge_m)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge model mass.");

          param("RefSim--Surge d", m_args.refsim.surge_d)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge drag constant.");

          param("RefSim--Heading Control Tune Direct", m_args.refsim.c_heading.tunedirect)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller setting PID parameters directly.");

          param("RefSim--Surge Control Tune Direct", m_args.refsim.c_surge.tunedirect)
          .defaultValue("False")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller setting PID parameters directly.");

          param("RefSim--Heading Control Bandwidth", m_args.refsim.c_heading.omega)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller bandwidth.");

          param("RefSim--Heading Control Damping", m_args.refsim.c_heading.xi)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller damping");

          param("RefSim--Heading Control Kp", m_args.refsim.c_heading.Kp)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller gain.");

          param("RefSim--Heading Control Ki", m_args.refsim.c_heading.Ki)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller integral gain.");

          param("RefSim--Heading Control Kd", m_args.refsim.c_heading.Kd)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator heading controller derivative gain.");

          param("RefSim--Surge Control Kp", m_args.refsim.c_surge.Kp)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller gain.");

          param("RefSim--Surge Control Ki", m_args.refsim.c_surge.Ki)
          .defaultValue("1.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Reference simulator surge controller integral gain.");

          param("RefSim--Surge Control Timeconstant", m_args.refsim.c_surge.T)
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
          .description("Choose whether to reset the integrator in a path activation. ");

          param("Reset to position on path startup", m_args.reset_to_state)
          .defaultValue("false")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Set to reset to state rather than previous ref_pos on change");

          param("Print Frequency", m_args.print_frequency)
          .defaultValue("0.0")
          .units(Units::Second)
          .description("Frequency of pos.data prints. Zero => Print on every update.");

          bind<IMC::DesiredZ>(this);
          bind<IMC::DesiredSpeed>(this);
        }

        //! Consumer for DesiredSpeed message.
        //! @param dspeed message to consume.
        void
        consume(const IMC::DesiredSpeed* dspeed)
        {
          // overloaded.
          // Update desired speed
          trace("DesiredSpeed= %f",dspeed->value);
          if (dspeed->value < m_args.refsim.max_speed)
          {
            m_reference_speed = dspeed->value;
          }
          else
          {
            m_reference_speed = m_args.refsim.max_speed;
            war("Trying to set a speed above maximum speed, setting to max (%f)",m_args.refsim.max_speed);
          }

          //PathFormationController::consume(dspeed);
        }

        void
        consume(const IMC::DesiredZ* zref)
        {
          trace("Got Desired Z. ");
          if (zref->z_units == IMC::Z_ALTITUDE || zref->z_units == IMC::Z_HEIGHT)
          {
            m_dz = *zref;
          }
        }

        virtual bool
        hasSpecificZControl(void) const
        {
          return false;
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          PathFormationController::onUpdateParameters();

          // update desired speed to max speed
          m_reference_speed = m_args.refsim.max_speed;

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

          if (paramChanged(m_args.los.delta_y) || paramChanged(m_args.los.delta_z))
          {
            if (m_args.los.delta_y == 0)
            {
              war("LOS -- Lookahead NE set to zero, setting to default value [1.0]");
              m_args.los.delta_y = 1.0;
            }
            if (m_args.los.delta_z == 0)
            {
              war("LOS -- Lookahead Down set to zero, setting to default value [1.0]");
              m_args.los.delta_z = 1.0;
            }
          }
          debug("Parameters set");
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          debug("Reserve entities");
          PathFormationController::onEntityReservation();
          for (unsigned i = 0; i < NUM_PARCELS; ++i)
            m_parcels[i].setSourceEntity(reserveEntity(c_parcel_names[i] + " Parcel " +  this->getEntityLabel()));
          for (unsigned i = 0; i < NUM_DESIRED; ++i)
          {
            m_desired_linear[i].setSourceEntity(reserveEntity(c_desired_names[i] + " Linear " + this->getEntityLabel()));
            debug("Entity label '%s' for DesiredLinearState reserved",resolveEntity(m_desired_linear[i].getSourceEntity()).c_str());
            m_desired_heading[i].setSourceEntity(reserveEntity(c_desired_names[i] + " Heading " + this->getEntityLabel()));
            debug("Entity label '%s' for DesiredHeading reserved",resolveEntity(m_desired_heading[i].getSourceEntity()).c_str());
          }
          debug("Entities reserved");
        }

        virtual void
        onPathStartup(const IMC::EstimatedLocalState& state, const TrackingState& ts)
        {
          // Print end coordinates
          debug("End coordinates: [%f, %f, %f]", ts.end.x, ts.end.y, ts.end.z);

          // Restart Reference Simulator
          initRefSim(state);

          // Restart LOS control
          initLOS(state,ts);

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

          m_los.first_WP  = true; // A new path arrived. Tracking to first waypoint.
        }

        void
        initLOS(const IMC::EstimatedLocalState& el, const TrackingState& ts)
        {
          (void) el;
          (void) ts;

          m_los.p       = Matrix(3,1,0.0);

          m_los.eps     = Matrix(3,1,0.0);

          m_los.v_p    = Matrix(3,1,0.0);
          m_los.v_n_los= Matrix(3,1,0.0);

          m_los.v_ref   = Matrix(3,1,0.0);
          m_los.a_ref   = Matrix(3,1,0.0);

          m_los.startWP = Matrix(3,1,0.0);
          m_los.endWP   = Matrix(3,1,0.0);
          debug("LOS initialized");
        }

        void
        initRefSim(const IMC::EstimatedLocalState& el)
        {
          debug("Initialize reference simulator");
          // Restart refmodel
          m_desired.x = Matrix(3,1,0.0);
          if (m_args.reset_to_state || Clock::get() - m_timestamp_prev_step > 2.0)
          {
            m_refsim.x_ref    = Matrix(3, 1, 0.0);
            m_refsim.x_ref(0) = m_reference_speed;
            m_refsim.x_ref(1) = el.state->psi;
            m_refsim.x_ref(2) = el.state->r;

            //set the desired output to the same as the initial reference to avoid steps
            m_refsim.x_des = Matrix(3, 1, 0.0);
            m_refsim.x_des(0) = el.state->u;
            m_refsim.x_des(1) = el.state->psi;
            m_refsim.x_des(2) = el.state->r;
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
          (void)state;
          //get target speed and heading (these or the reference states should be logged somehow)

          spew("ts.delta=%f",ts.delta);
          //update system matrix with the current states
          updateMatrixA();

          Matrix e     = m_refsim.getError();
          Matrix e_dot = m_refsim.getDError();

          //Normalize angle error
          e(R_HEADING) = Angles::normalizeRadian(e(R_HEADING));

          m_integrator_value += ts.delta*e;
          // Constrain
          if (m_integrator_value.norm_2() > m_args.max_integral)
            m_integrator_value = m_args.max_integral * m_integrator_value / m_integrator_value.norm_2();


          Matrix u_p = m_refsim.Kp*e;
          Matrix u_i = m_refsim.Ki*m_integrator_value;
          Matrix u_d = m_refsim.Kd*e_dot;
          Matrix u   = u_p + u_i + u_d;

          m_refsim.x_dot_des = m_refsim.getDesXdot(u);
          // Integrate (Euler)
          m_refsim.x_des += ts.delta * (m_refsim.x_dot_des);
          if (m_refsim.x_des.norm_2() > 100)
            war("m_refsim.x_des.norm_2()=%f",m_refsim.x_des.norm_2());
          m_refsim.x_des(R_HEADING) = Angles::normalizeRadian(m_refsim.x_des(R_HEADING));

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
          (void)now;
          
          spew("stepRefSim");
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
        setReferences(const IMC::EstimatedLocalState& el, const TrackingState& ts)
        {

          if(m_los.last_start_z != ts.start.z){
            if(m_los.last_end_z == ts.start.z){
              m_los.first_WP = false;
            }
            else{
              m_los.first_WP = true;
            }
          }

          //Waypoint- handling
          double start_z = ts.start.z;
          double end_z = ts.end.z;


          if(m_los.first_WP){
            start_z = el.state->height - start_z;
          }

          float desiredZstart = el.state->z;
          float desiredZend   = el.state->z;
          // Z ref handling
          if (m_dz.z_units == IMC::Z_HEIGHT)
          {
            desiredZstart = el.state->height - start_z;
            desiredZend   = el.state->height - end_z;
          }
          else if(m_dz.z_units == IMC::Z_ALTITUDE)
          {
            desiredZstart = el.state->z + el.state->alt - start_z;
            desiredZend   = el.state->z + el.state->alt - end_z;
          }
          else
          {
            war("DesiredZ not received");
          }

          //global reference point and these value might change during maneuver
          spew("setReferences: set new states");
          m_los.p(0) = el.state->x;
          m_los.p(1) = el.state->y;
          m_los.p(2) = el.state->z;
          m_los.startWP(0) = ts.start.x;
          m_los.startWP(1) = ts.start.y;
          m_los.startWP(2) = desiredZstart;
          m_los.endWP(0)   = ts.end.x;
          m_los.endWP(1)   = ts.end.y;
          m_los.endWP(2)   = desiredZend;

          spew("setReferences: set new states done");

          Matrix deltaWP = m_los.endWP - m_los.startWP;
          double deltaWP_NE = deltaWP.get(0, 1, 0, 0).norm_2();

          spew("setReferences: calc alpha and theta");
          m_los.alpha  =  atan2(deltaWP(1), deltaWP(0));
          m_los.theta  = -atan2(deltaWP_NE, deltaWP(2)) + Angles::radians(90);

          m_los.eps = transpose(RNedPath())*(m_los.p - m_los.startWP);

          spew("setReferences: calc psi_los and theta_los");
          double e_y = m_los.eps(1);
          double e_z = m_los.eps(2);

          double psi_los     = atan(-e_y/m_args.los.delta_y);
          double theta_los   = atan(-e_z/m_args.los.delta_z);

          spew("setReferences: calc psi_los and theta_los done");
          m_los.xi_d     = m_los.alpha + psi_los;
          m_los.theta_d  = m_los.theta + theta_los;



          // set reference heading
          double ref_heading = 0.0;
          if(m_args.los.use_LOSangle)
            ref_heading = ts.los_angle;
          else if(m_args.los.use_bearingHeading)
            ref_heading = ts.track_bearing;
          else
            ref_heading = m_los.xi_d;

          //save and dispatch references
          m_refsim.setRefHeading(ref_heading);
          m_desired_heading[D_REFERENCE].value = m_refsim.x_ref(R_HEADING);
          dispatch(m_desired_heading[D_REFERENCE]);
          spew("setReferences: reference heading dispatched");

          m_los.v_p(0)  = m_reference_speed;
          m_los.v_n_los = Rzyx(0,-m_los.theta_d,m_los.xi_d)*m_los.v_p;

          spew("setReferences: rotate body XY to NED");
          // Ned reference velocity
          Matrix v_NE = Rzyx(0,0,el.state->psi)*m_los.v_p;

          spew("setReferences: rotate to NED");
          // Reference velocity in NED
          Matrix v_n = Matrix(3,1,0.0);
          v_n(0) = v_NE(0);
          v_n(1) = v_NE(1);
          v_n(2) = m_los.v_n_los(2);

          spew("setReferences: rotate back to body");
          //Rotate back to body
          m_los.v_ref = transpose(RNedCentroid())*v_n;

          spew("setReferences: desired body velocity calculated");
          if (!m_args.use_altitude)
          {
            m_los.v_ref(2) = 0;
            m_los.a_ref(2) = 0;
          }


          m_refsim.setRefSpeed(m_los.v_ref(0));
          m_desired_linear[D_REFERENCE].vx = m_los.v_ref(0);
          m_desired_linear[D_REFERENCE].vy = m_los.v_ref(1);
          m_desired_linear[D_REFERENCE].vz = m_los.v_ref(2);
          if (m_args.disable_heave)
          {
            m_desired_linear[D_REFERENCE].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY
                                                | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY;
          }
          else
          {
            m_desired_linear[D_REFERENCE].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY | IMC::DesiredLinearState::FL_VZ
                                                | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY | IMC::DesiredLinearState::FL_VZ;
          }
          dispatch(m_desired_linear[D_REFERENCE]);
          spew("setReferences: reference linear state dispatched");

          double now = Clock::get();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            trace("setReferences: alpha=[%f],theta=[%f]",m_los.alpha,m_los.theta);
            trace("setReferences: xi_d=[%f],theta_d=[%f]",m_los.xi_d,m_los.theta_d);
            trace("setReferences: v_n=[%f,%f,%f]",v_n(0),v_n(1),v_n(2));
            trace("setReferences: v_n_los=[%f,%f,%f]",m_los.v_n_los(0),m_los.v_n_los(1),m_los.v_n_los(2));
            trace("setReferences: eps=[%f,%f,%f]",m_los.eps(0),m_los.eps(1),m_los.eps(2));
            trace("setReferences: delta=[%f,%f]",m_args.los.delta_y,m_args.los.delta_z);
            trace("setReferences: ts.start.z=[%f]",ts.start.z);
            trace("setReferences: ts.end.z=[%f]",ts.end.z);
            trace("setReferences: desiredZstart=[%f]",desiredZstart);
            trace("setReferences: desiredZend=[%f]",desiredZend);
            trace("setReferences: z=[%f]",el.state->z);
            trace("setReferences: height=[%f]",el.state->height);
            trace("WPstart=[%f,%f,%f]",m_los.startWP(0),m_los.startWP(1),m_los.startWP(2));
            trace("WPend=[%f,%f,%f]",m_los.endWP(0),m_los.endWP(1),m_los.endWP(2));
            trace("m_los.first_WP=%d",m_los.first_WP);

            last_print = now;
          }

          m_los.last_end_z   = end_z;
          m_los.last_start_z = start_z;
        }

        void
        sendDesired()
        {
          //Desired acceleration and velocity in body
          m_desired_linear[D_DESIRED].vx = m_refsim.getDesSpeed();
          m_desired_linear[D_DESIRED].vy = m_los.v_ref(1);
          m_desired_linear[D_DESIRED].vz = m_los.v_ref(2);
          //Desired acceleration in body
          m_desired_linear[D_DESIRED].ax = m_refsim.getDesAcc();
          m_desired_linear[D_DESIRED].ay = m_los.a_ref(1);
          m_desired_linear[D_DESIRED].az = m_los.a_ref(2);
          if (m_args.disable_heave)
          {
            m_desired_linear[D_DESIRED].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY
                                              | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY;
          }
          else
          {
            m_desired_linear[D_DESIRED].flags = IMC::DesiredLinearState::FL_VX | IMC::DesiredLinearState::FL_VY | IMC::DesiredLinearState::FL_VZ
                                              | IMC::DesiredLinearState::FL_AX | IMC::DesiredLinearState::FL_AY | IMC::DesiredLinearState::FL_AZ;
          }
          dispatch(m_desired_linear[D_DESIRED]);
          //Desired heading
          m_desired_heading[D_DESIRED].value = m_refsim.getDesHeading();
          dispatch(m_desired_heading[D_DESIRED]);
        }

        void
        step(const IMC::EstimatedLocalState& state, const TrackingState& ts)
        {
          if (!m_args.use_controller)
          {
            trace("Controller not enabled");
            return;
          }
          m_elstate = state;

          double now = Clock::get();


          spew("setReferences");
          setReferences(state,ts);

          spew("updateReferenceSim");
          updateReferenceSim(state, ts, now);

          spew("sendDesired");
          sendDesired();

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

        //! R^(n)_(centroid)
        Matrix
        RNedPath() const
        {
          return Rzyx(0,-m_los.theta,m_los.alpha);
        }

        //! R^(n)_(centroid)
        Matrix
        RNedCentroid() const
        {
          return Rzyx(m_elstate.state->phi,m_elstate.state->theta,m_elstate.state->psi);
        }

        //! @return  Rotation yaw matrix.
        Matrix
        Rz(double psi) const
        {
          double R_en_elements[] =
            { cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1 };
          return Matrix(R_en_elements, 3, 3);
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

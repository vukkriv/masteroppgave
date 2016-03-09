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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace UAV
  {
    namespace Copter
    {
      namespace RCFollowReference
      {
        using DUNE_NAMESPACES;

        struct Arguments
        {
          double max_speed_xy;
          double max_speed_z;
          double lookahead_time;
          bool auto_enable;
          std::string ctrl_entity_name;
        };



        enum PwmChannel {
          CH_X = 1,
          CH_Y = 0,
          CH_Z = 2,
          CH_TUNE = 5
        };

        struct MaxAccelerationContainer
        {

          public:
            MaxAccelerationContainer()
            {
              time_last_check = Clock::get();
              time_last_received = Clock::get();
              last_check_reference = 0;
              max_acceleration = 3.0;
              entity_id = 0;
            };

            double time_last_check;
            double time_last_received;
            unsigned int last_check_reference;
            double max_acceleration;
            unsigned int entity_id;
        };

        struct Task: public DUNE::Tasks::Task
        {

          //! Task arguments
          Arguments m_args;
          //! Test time
          double m_time_last_dispatch;
          //! Is running
          bool m_fr_is_running;
          //! Last received estate
          IMC::EstimatedState m_estate;

          //! current output
          double m_lat, m_lon, m_hae, m_dspeed;

          //! Vehicle position at last calculation
          double m_vehicle_last_lat, m_vehicle_last_lon, m_vehicle_last_hae, m_vehicle_last_speed;

          //! PWM inputs
          unsigned int m_pwm_inputs[8];

          //! Last received plan control state.
          IMC::PlanControlState m_pcs;

          //! Last received autopilot mode
          IMC::AutopilotMode m_apmode;

          //! Last sent reference
          IMC::Reference m_ref;

          //! Container for maximum acceleration
          MaxAccelerationContainer m_max_acc;
          //!
          //! Constructor.
          //! @param[in] name task name.
          //! @param[in] ctx context.
          Task(const std::string& name, Tasks::Context& ctx):
            DUNE::Tasks::Task(name, ctx),
            m_fr_is_running(false),
            m_lat(0.0), m_lon(0.0), m_hae(0.0), m_dspeed(0.0),
            m_vehicle_last_lat(0.0), m_vehicle_last_lon(0.0), m_vehicle_last_hae(0.0), m_vehicle_last_speed(0.0)
          {

            param("Max Speed - XY", m_args.max_speed_xy)
            .defaultValue("5")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::MeterPerSecond);

            param("Max Speed - Z", m_args.max_speed_z)
            .defaultValue("2")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::MeterPerSecond);

            param("Lookahead Time", m_args.lookahead_time)
            .defaultValue("8")
            .visibility(Parameter::VISIBILITY_USER)
            .units(Units::Second);

            param("Automatic Enable", m_args.auto_enable)
            .defaultValue("false")
            .visibility(Parameter::VISIBILITY_USER)
            .description("Automatically enable if entering guided and still in service mode. ");

            param("Ctrl entity name", m_args.ctrl_entity_name)
            .defaultValue("Simple Acceleration Path Controller");


            bind<IMC::PWM>(this);
            bind<IMC::PlanControlState>(this);
            bind<IMC::RemoteActions>(this);
            bind<IMC::EstimatedState>(this);
            bind<IMC::AutopilotMode>(this);
            bind<IMC::EntityParameters>(this);

            m_time_last_dispatch = Clock::get();

            for (int i = 0; i < 8; i++)
            {
              m_pwm_inputs[i] = 1500;
            }
          }


          double
          pwmToValueDeadband(double val_min, double val_max, unsigned int rc_min, unsigned int rc_max, int inverse, double deadbandPercent, unsigned int rc_in)
          {
            unsigned int rc_trim = (rc_max + rc_min)/2;
            unsigned int dead_zone = (rc_max - rc_min) * deadbandPercent;

            unsigned int rc_trim_high = rc_trim + dead_zone;
            unsigned int rc_trim_low  = rc_trim - dead_zone;

            // prevent div by 0
            if ((rc_trim_low - rc_min) == 0 || (rc_max - rc_trim_high) == 0)
                return 0;

            // Sanity checks
            if ( rc_in > rc_max )
              rc_in = rc_max;
            if ( rc_in < rc_min )
              rc_in = rc_min;


            int reverse_mul = 1;
            if( inverse == 1 )
              reverse_mul = -1;


            if(rc_in > rc_trim_high) {
                return reverse_mul * ((double)val_max * (rc_in - (int)rc_trim_high)) / (rc_max  - rc_trim_high);
            }else if(rc_in < rc_trim_low) {
                return reverse_mul * ((double)val_min * ((int)rc_trim_low - rc_in)) / (rc_trim_low - rc_min);
            }else
                return 0.0;

          }


          void
          consume(const IMC::AutopilotMode* apmode)
          {

            if (apmode->mode == "GUIDED" && m_apmode.mode == "LOITER")
            {
              // If we are also in service, check if we should start.
              if (m_pcs.state == IMC::PlanControlState::PCS_READY
                  && m_args.auto_enable)
              {
                inf("Starting Follow Reference based on mode switch. ");
                generatePlan();
              }
            }

            // Check if disable
            if (apmode->mode != "GUIDED" && m_fr_is_running)
            {
              m_ref.flags |= IMC::Reference::FLAG_MANDONE;
              dispatch(m_ref);
            }

            m_apmode = *apmode;
          }

          void
          consume(const IMC::PlanControlState* pcs)
          {
            m_pcs = *pcs;

            if (pcs->plan_id == "RCFollowReference" && pcs->state == IMC::PlanControlState::PCS_EXECUTING)
              m_fr_is_running = true;
            else
              m_fr_is_running = false;
          }

          void
          consume(const IMC::PWM* pwm)
          {
            spew("Got pwm: %d, %d", pwm->id, pwm->duty_cycle);



            if (pwm->id > 0 && pwm->id <=8 )
            {
              m_pwm_inputs[pwm->id - 1] = pwm->duty_cycle;
            }

            if (pwm->id == 8 && m_fr_is_running)
            {
              double now = Clock::get();
              if (now - m_time_last_dispatch > 1)
              {
                m_time_last_dispatch = now;
                calculateReference();
              }
            }

            //inf("Channel: %d, value: %f", pwm->id, pwmToValueDeadband(-4, 4, 900, 2100, 0.1, pwm->duty_cycle));

          }

          void
          consume(const IMC::EstimatedState* msg)
          {
            m_estate = *msg;
          }

          void
          consume(const IMC::RemoteActions* ra)
          {
            inf("GOt remote actin. ");
            inf("%s", ra->actions.c_str());

            if(strcmp("piksiResetIARs=1", ra->actions.c_str()) == 0)
            {
              inf("Success. Generate plan. ");
              generatePlan();
            }
          }

          void
          consume(const IMC::EntityParameters* msg)
          {
            if (msg->getSource() != getSystemId())
              return;

            if (msg->getSourceEntity() != m_max_acc.entity_id)
              return;

            IMC::MessageList<IMC::EntityParameter>::const_iterator itr = msg->params.begin();
            for ( ; itr != msg->params.end(); ++itr)
            {
              if( (*itr)->name == "Ref - Max Acceleration" && !(*itr)->value.empty())
              {
                // Parse the string
                std::istringstream ss((*itr)->value);

                double max_acc;
                ss >> max_acc;

                if (!ss.fail())
                {
                  m_max_acc.max_acceleration = max_acc;
                  m_max_acc.time_last_received = Clock::get();
                  debug("Set max acc to: %.3f", max_acc);
                }
              }

            }
          }
          void
          calculateReference(void)
          {
            // Calculate the velocity inputs
            Matrix vel = Matrix(3,1, 0.0);


            vel(0) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 900, 2100, 1, 0.1, m_pwm_inputs[CH_X]);
            vel(1) = pwmToValueDeadband(-m_args.max_speed_xy, m_args.max_speed_xy, 900, 2100, 0, 0.1, m_pwm_inputs[CH_Y]);
            vel(2) = pwmToValueDeadband( m_args.max_speed_z, -m_args.max_speed_z,  900, 2100, 0, 0.1, m_pwm_inputs[CH_Z]);

            // Rotate velocity
            double u,v,w;
            BodyFixedFrame::toInertialFrame(0.0, 0.0, (double)m_estate.psi,
                                        vel(0), vel(1), vel(2),
                                        &u, &v, &w);

            vel(0) = u;
            vel(1) = v;
            vel(2) = w;

            // Calculate position offset
            Matrix pos_offset = vel * m_args.lookahead_time;

            // Increase pos offset in z to get more "punch"
            pos_offset(2) *= 3;

            // We cannot set a reference to lower alt than -1 (to be able to land).
            if( m_estate.alt - pos_offset(2) < -1 )
            {
              pos_offset(2) = m_estate.alt + 1;
              vel(2) = pos_offset(2)/m_args.lookahead_time;
              trace("Limiting velz. ");
            }

            trace("z offset: %.2f, alt: %.2f, resulting alt: %.2f", pos_offset(2), m_estate.alt, m_estate.alt - pos_offset(2));

            // Also handle initialization.
            if ((vel.norm_2() > 0.5) || WGS84::distance(m_lat, m_lon, (float)m_hae, m_estate.lat, m_estate.lon, m_estate.height) > 1000)
            {
              // Calculate lat lons;
              double lat = m_estate.lat;
              double lon = m_estate.lon;
              double hae = m_estate.height;
              WGS84::displace(m_estate.x + pos_offset(0), m_estate.y + pos_offset(1), m_estate.z + pos_offset(2),
                              &lat, &lon, &hae);


              m_lat = lat; m_lon = lon; m_hae = hae;
              m_dspeed = vel.norm_2();


              // Store this position
              m_vehicle_last_lat = m_estate.lat;
              m_vehicle_last_lon = m_estate.lon;
              m_vehicle_last_hae = m_estate.height;
              WGS84::displace(m_estate.x, m_estate.y, m_estate.z,
                                            &m_vehicle_last_lat, &m_vehicle_last_lon, &m_vehicle_last_hae);

              m_vehicle_last_speed = m_dspeed;

            }
            else {
              m_lat = m_vehicle_last_lat;
              m_lon = m_vehicle_last_lon;
              m_hae = m_vehicle_last_hae;

              // Allow a minimum speed to stop
              double minSpeed = 2.0;
              m_dspeed = m_vehicle_last_speed < minSpeed ? minSpeed : m_vehicle_last_speed;
            }

            IMC::Reference ref;
            ref.flags = IMC::Reference::FLAG_LOCATION | IMC::Reference::FLAG_Z | IMC::Reference::FLAG_SPEED;
            ref.lat = m_lat;
            ref.lon = m_lon;

            IMC::DesiredZ dz;
            dz.z_units = IMC::Z_HEIGHT;
            dz.value = m_hae;

            ref.z.set(dz);

            IMC::DesiredSpeed ds;
            ds.speed_units = IMC::SUNITS_METERS_PS;
            ds.value       = m_dspeed;

            ref.speed.set(ds);

            ref.radius = 0.0;


            m_ref = ref;


            dispatch(ref);

          }



          void
          generatePlan(void)
          {
            // Create plan set request
            IMC::PlanDB plan_db;
            plan_db.type = IMC::PlanDB::DBT_REQUEST;
            plan_db.op = IMC::PlanDB::DBOP_SET;
            plan_db.plan_id = "RCFollowReference";
            plan_db.request_id = 0;

            // Create plan specification
            IMC::PlanSpecification plan_spec;
            plan_spec.plan_id = plan_db.plan_id;
            plan_spec.start_man_id = 1;
            plan_spec.description = "Plan activating FollowReference";

            // Create plan maneuver
            IMC::PlanManeuver man_spec;
            man_spec.maneuver_id = 1;
            /*
             // Create custom maneuver (not supported?!)
             IMC::CustomManeuver c_man;
             c_man.name = "formationManeuver";
             */

            // Create some maneuver
            //IMC::Goto c_man;

            // Create a follow reference maneuver
            IMC::FollowReference c_man;
            c_man.loiter_radius = 0.0;
            c_man.control_ent = getEntityId();
            c_man.control_src = getSystemId();
            c_man.timeout    = 2.0;


            man_spec.data.set(c_man);

            // Create start actions
            IMC::SetEntityParameters eparam_start;
            eparam_start.name = "Simple Acceleration Path Controller";


            IMC::EntityParameter param_t;
            param_t.name = "Acceleration Controller";
            param_t.value = "true";
            eparam_start.params.push_back(param_t);


            man_spec.start_actions.push_back(eparam_start);

            // Create end actions
            IMC::SetEntityParameters eparam_stop;
            eparam_start.name = "Simple Acceleration Path Controller";
            IMC::EntityParameter param_f;
            param_f.name = "Acceleration Controller";
            param_f.value = "false";

            eparam_start.params.push_back(param_f);

            man_spec.end_actions.push_back(eparam_stop);

            plan_spec.maneuvers.push_back(man_spec);

            plan_db.arg.set(plan_spec);

            // Send set plan request
            dispatch(plan_db);

            // Create and send plan start request
            IMC::PlanControl plan_ctrl;
            plan_ctrl.type = IMC::PlanControl::PC_REQUEST;
            plan_ctrl.op = IMC::PlanControl::PC_START;
            plan_ctrl.plan_id = plan_spec.plan_id;
            plan_ctrl.request_id = 0;
            plan_ctrl.arg.set(plan_spec);
            dispatch(plan_ctrl);
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
            try
            {
              m_max_acc.entity_id = resolveEntity(m_args.ctrl_entity_name);
            }
            catch (...)
            {
              err("Unable to get entity id. ");
              m_max_acc.entity_id = 0;
            }
          }

          //! Acquire resources.
          void
          onResourceAcquisition(void)
          {
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
          queryMaxAcc(void)
          {
            IMC::QueryEntityParameters ereq;

            ereq.name = m_args.ctrl_entity_name;
            ereq.setDestinationEntity(m_max_acc.entity_id);



            dispatch(ereq);
          }

          //! Main loop.
          void
          onMain(void)
          {
            while (!stopping())
            {
              // Check max acc every 5 second

              double now = Clock::get();
              if( now - m_max_acc.time_last_check > 5.0 )
              {
                queryMaxAcc();
                m_max_acc.time_last_check = now;
                trace("Query acc. ");
              }


              waitForMessages(1.0);
            }
          }
        };
      }
    }
  }
}

DUNE_TASK

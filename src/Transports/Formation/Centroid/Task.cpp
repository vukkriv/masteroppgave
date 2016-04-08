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

namespace Transports
{
  namespace Formation
  {
    namespace Centroid
    {
      using DUNE_NAMESPACES;

      struct Arguments
      {
        //! Send Formation Centroid
        bool use_task;

        //! Vehicle list
        //std::vector<std::string> formation_systems;

        float print_frequency;

        std::vector<std::string> desired_heading_entity_labels;
      };

      //static const std::string c_entity_names[] = {DTR_RT("Centroid")};
      //static const int NUM_ENTITIES = 1;

      class Vehicles
      {
      public:
        Vehicles() :
            m_i(0), m_N(0)
        {

        }
        //! Vehicle formation number
        unsigned int m_i;
        //! Number of agents
        unsigned int m_N;
        //! Vehicle IDs
        std::vector<uint32_t> m_uav_ID;

        std::vector<bool> connected;

        bool
        setVehicleList(const IMC::CoordConfig* config, DUNE::Tasks::Task *task)
        {
          const IMC::MessageList<IMC::VehicleFormationParticipant>* part = &config->participants;
          IMC::MessageList<IMC::VehicleFormationParticipant>::const_iterator itr;

          // Extract vehicle IDs
          m_uav_ID.clear();

          if (static_cast<unsigned int>(part->size()) == 0)
          {
            m_uav_ID.push_back(task->getSystemId());
            m_N = 1;
            m_i = 0;
          }
          else
          {
            m_N = static_cast<unsigned int>(part->size());
            bool found_self = false;
            unsigned int uav = 0;
            for (itr = part->begin(); itr != part->end(); itr++)
            {
              //all participants
              m_uav_ID.push_back((*itr)->vid);

              if ((*itr)->vid == task->getSystemId())
              {
                m_i = uav; // Set my formation id
                found_self = true;
              }
              uav += 1;
            }
            if (!found_self)
              return false;
          }
          connected = std::vector<bool>(m_N, false);
          return true;
        }

        bool
        allConnected()
        {
          bool allConn = true;
          for (std::vector<bool>::iterator it = connected.begin(); it != connected.end(); ++it)
            allConn *= *it;
          return allConn;
        }

        int
        getVehicle(unsigned int id)
        {
          int vehicle = -1;
          for (unsigned int uav = 0; uav < m_N; uav++)
          {
            if (m_uav_ID[uav] == id)
            {
              vehicle = uav;
              break;
            }
          }
          if (vehicle != -1)
            connected[vehicle] = true;
          return vehicle;
        }

      };

      class Centroid
      {
      public:
        Centroid() :
          m_N(1),
          v_b(Matrix(3, 1, 0.0)),
          a_b(Matrix(3, 1, 0.0)),
          v_n(Matrix(3, 1, 0.0)),
          a_n(Matrix(3, 1, 0.0)),
          omega(Matrix(3, 1, 0.0)),
          omega_i(Matrix(3, 1, 0.0)),
          alpha(Matrix(3, 1, 0.0))
        {
        }
        int m_N;

        Matrix v_b;
        Matrix a_b;
        Matrix v_n;
        Matrix a_n;
        Matrix omega;
        Matrix omega_i;
        Matrix alpha;

        IMC::EstimatedLocalState els;
        IMC::EstimatedState m_es;
        IMC::Acceleration m_acc;

        void
        calculateCentroid(std::vector<IMC::EstimatedLocalState> &el_v,
            int noAgents)
        {
          this->m_N = noAgents;

          m_es.x = 0;
          m_es.y = 0;
          m_es.z = 0;

          v_n *= 0;
          a_n *= 0;
          for (int i = 0; i < noAgents; i++)
          {
            double factor = (double)1 / double(noAgents);
            m_es.x += factor * (el_v[i].state->x);
            m_es.y += factor * (el_v[i].state->y);
            m_es.z += factor * (el_v[i].state->z);

            v_n(0) += factor * (el_v[i].state->vx);
            v_n(1) += factor * (el_v[i].state->vy);
            v_n(2) += factor * (el_v[i].state->vz);

            omega_i(0) = el_v[i].state->p;
            omega_i(1) = el_v[i].state->q;
            omega_i(2) = el_v[i].state->r;

            alpha = skew(omega_i) * v_n;
            //Acceleration is BODY acceleration differentiated in NED frame from Ardupilot interface
            a_n(0) += factor * (el_v[i].acc->x + alpha(0));
            a_n(1) += factor * (el_v[i].acc->y + alpha(1));
            a_n(2) += factor * (el_v[i].acc->z + alpha(2));
          };

          m_es.vx = v_n(0);
          m_es.vy = v_n(1);
          m_es.vz = v_n(2);

          /*
           let the internal localstate acc be in body
           state.ax = a_n(0);
           state.ay = a_n(1);
           state.az = a_n(2);
           */

          m_es.phi = 0;
          m_es.theta = 0;
          m_es.psi = this->centroidHeading(el_v);

          m_es.p = 0;
          m_es.q = 0;
          m_es.r = 0;  //TODO: find expression for this
          omega(2) = m_es.r;

          v_b = getBodyVelocity();
          m_es.u = v_b(0);
          m_es.v = v_b(1);
          m_es.w = v_b(2);

          a_b = getBodyAcceleration();
          m_acc.x = a_b(0);
          m_acc.y = a_b(1);
          m_acc.z = a_b(2);

          //all vehicles should have the same reference point
          m_es.lat     = el_v[0].state->lat;
          m_es.lon     = el_v[0].state->lon;
          m_es.height  = el_v[0].state->height;

          els.state.set(m_es);
          els.acc.set(m_acc);
        }
      private:

        fp32_t
        centroidHeading(std::vector<IMC::EstimatedLocalState> &el_v)
        {
          if (this->m_N > 1)
          {
            Matrix p = Matrix(2, this->m_N, 0);
            for (int i = 0; i < this->m_N; i++)
            {
              p(0, i) = el_v[i].state->x;
              p(1, i) = el_v[i].state->y;
            }
            Matrix p_diff = p.column(1) - p.column(0);

            return Angles::normalizeRadian(-std::atan2(p_diff(0), p_diff(1)));
          }
          else
          {
            return el_v[0].state->psi;
          }
        }

        Matrix
        getBodyVelocity()
        {
          Matrix Rcn = transpose(Rz(m_es.psi));
          return Rcn * v_n;
        }

        Matrix
        getBodyAcceleration()
        {
          Matrix Rcn = transpose(Rz(m_es.psi));
          return Rcn * (a_n - skew(omega) * v_n);
        }

        //! @return  Rotation yaw matrix.
        Matrix
        Rz(double psi) const
        {
          double R_en_elements[] =
            { cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1 };
          return Matrix(R_en_elements, 3, 3);
        }
      };

      struct Task : public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;

        //! SourceIDs of all vehicles in formation
        Vehicles m_vehicles;

        //! State of centroid
        Centroid m_centroid;

        IMC::EstimatedLocalState m_last_elstate;

        std::vector<IMC::EstimatedLocalState> states;

        bool m_configured;

        //! Last received desired heading from master agent
        IMC::DesiredHeading m_desired_heading;
        double m_curr_desired_heading;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx) :
            DUNE::Tasks::Task(name, ctx),
            m_configured(false)
        {
          param("Formation Centroid", m_args.use_task)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
//          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("true")
          .description("Enable Formation Centroid EstimatedLocalState.");
/*
          param("Vehicle List", m_args.formation_systems)
          .defaultValue("")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("System name list of the formation vehicles.");
*/
          param("Print Frequency", m_args.print_frequency)
          .defaultValue("0.0")
          .units(Units::Second)
          .description("Frequency of pos.data prints. Zero => Print on every update.");

          param("Desired Heading Entity Labels", m_args.desired_heading_entity_labels)
          .defaultValue("Desired Heading")
          .description("Entity labels for the DesiredHeading message");

          bind<IMC::CoordConfig>(this);
          bind<IMC::EstimatedLocalState>(this);
          bind<IMC::DesiredHeading>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          /*
          debug("Starting update of parameters.");
          if (paramChanged(m_args.formation_systems))
          {
            inf("New Formation vehicles' list.");
            m_vehicles.setVehicleList(m_args.formation_systems, this);
            states = std::vector<IMC::EstimatedLocalState>(m_vehicles.m_N);
          }
          */
        }

        //! Reserve entity identifiers.
        void
        onEntityReservation(void)
        {
          //Reserve entities for Centroid, might expand to set entites for vehicle C1...Cn
          //however, if only using one, all should come from the overall entity name
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
        consume(const IMC::CoordConfig* config)
        {
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got CoordConfig from '%s'", resolveSystemId(config->getSource()));
            last_print = now;
          }

          if (config->update || !m_configured)
          {
            //CoordConfig contains new formation data
            debug("New Formation vehicles' list.");
            bool found_self = m_vehicles.setVehicleList(config, this);
            if (!found_self)
            {
              war("Vehicle is no longer a part of the formation");
              m_configured = false;
              return;
            }
            states = std::vector<IMC::EstimatedLocalState>(m_vehicles.m_N);
            if (!m_configured)
            {
              m_configured = true;
              debug("Configured with IMC::CoordConfig");
            }
          }
        }

        void
        consume(const IMC::EstimatedLocalState* msg)
        {
          if (!m_configured)
          {
            spew("Not configured!");
            return;
          }
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got EstimatedLocalState from system '%s'.",resolveSystemId(msg->getSource()));
            if (msg->getSource() == this->getSystemId())
              spew("Entity '%s'", resolveEntity(msg->getSourceEntity()).c_str());
            last_print = now;
            if (!m_configured)
              spew("Not configured");
          }

          if (!m_configured)
            return;

          int vh_id = m_vehicles.getVehicle(msg->getSource());
          spew("Vehicle[%d]", vh_id);
          if (vh_id != -1)
          {
            states[vh_id] = *msg;
            spew("Pos x: [%f]", states[vh_id].state->x);
          }
          else
          {
            spew("Unknown vehicle");
            return;
          }
          /*
           for (unsigned int i=0; i < m_vehicles.m_N; i++)
           {
           spew("msgSource=%d",msg->getSource());
           spew("m_uav_ID[%d]=%d",i,m_vehicles.m_uav_ID[i]);
           }
           */
          //calculate centroid (if received from all and the current message is from itself)
          if (m_vehicles.allConnected()
              && msg->getSource() == this->getSystemId())
          {
            //spew("m_centroid.v_n.size()=%d",m_centroid.);
            m_centroid.calculateCentroid(states, m_vehicles.m_N);
            if (m_vehicles.m_N == 1)
            {
              //TODO:
              //with heading control, this would not be necessary
              //then the current vehicle heading should be the same as
              //the virtual centroid heading
              m_centroid.els.state->psi = m_curr_desired_heading;
            }
            spew(
                "Dispatching centroid EstimatedLocalState, heading: %f [deg], noAgents=[%d]",
                Angles::degrees(m_centroid.els.state->psi), m_centroid.m_N);
            //set sourceEntity here, make sure NOT to send the centroid message inbetween vehicles
            m_centroid.els.ots = msg->getTimeStamp();
            dispatch(m_centroid.els);
          }
        }

        void
        consume(const IMC::DesiredHeading* msg)
        {
          double now = Clock::getSinceEpoch();
          static double last_print;
          if (!m_args.print_frequency || !last_print
              || (now - last_print) > 1.0 / m_args.print_frequency)
          {
            spew("Got DesiredHeading from '%s'", resolveSystemId(msg->getSource()));
            spew("DesiredHeading [%f]",msg->value);
            last_print = now;
          }
          //Desired heading should only come from master only
          //if receiving local message, it should have the correct entity (desired vs reference)
          if (   msg->getSource() == this->getSystemId()
              && !isDesiredHeading(msg->getSourceEntity())      )
            return;

          m_desired_heading = *msg;
          m_curr_desired_heading = m_desired_heading.value;
        }

        //! Main loop.
        void
        onMain(void)
        {
          while (!stopping())
          {
            waitForMessages(1.0);
          }
        }

        bool
        isDesiredHeading(uint8_t msgSourceEntity)
        {
          std::string msgEntity = resolveEntity(msgSourceEntity).c_str();
          for (std::vector<std::string>::iterator it = m_args.desired_heading_entity_labels.begin(); it != m_args.desired_heading_entity_labels.end(); ++it)
            if (*it == msgEntity)
              return true;
          return false;
        }
      };
    }
  }
}

DUNE_TASK

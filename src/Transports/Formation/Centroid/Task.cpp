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
        std::vector<std::string> formation_systems;
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

        void
        setVehicleList(std::vector<std::string> list, DUNE::Tasks::Task *task)
        {
          // Extract vehicle IDs
          m_uav_ID.clear();
          if (list.empty())
          {
            m_uav_ID.push_back(task->getSystemId());
            m_N = 1;
            m_i = 0;
          }
          else
          {
            m_N = list.size();
            bool found_self = false;
            for (unsigned int uav = 0; uav < m_N; uav++)
            {
              m_uav_ID.push_back(task->resolveSystemName(list[uav]));
              if (m_uav_ID[uav] == task->getSystemId())
              {
                m_i = uav; // Set my formation id
                found_self = true;
              }
            }
            if (!found_self)
              throw DUNE::Exception(
                  "Vehicle not found in formation vehicle list!");
          }
          connected = std::vector<bool>(m_N, false);
        }

        bool
        allConnected()
        {
          bool allConn = true;
          for (std::vector<bool>::iterator it = connected.begin();
              it != connected.end(); ++it)
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

        IMC::EstimatedLocalState state;

        void
        calculateCentroid(std::vector<IMC::EstimatedLocalState> &states,
            int noAgents)
        {
          this->m_N = noAgents;

          state.x = 0;
          state.y = 0;
          state.z = 0;
          v_n *= 0;
          a_n *= 0;
          for (int i = 0; i < noAgents; i++)
          {
            double factor = (double)1 / double(noAgents);
            state.x += factor * (states[i].x);
            state.y += factor * (states[i].y);
            state.z += factor * (states[i].z);

            v_n(0) += factor * (states[i].vx);
            v_n(1) += factor * (states[i].vy);
            v_n(2) += factor * (states[i].vz);

            omega_i(0) = states[i].p;
            omega_i(1) = states[i].q;
            omega_i(2) = states[i].r;

            alpha = skew(omega_i) * v_n;
            //Acceleration is BODY acceleration differentiated in NED frame from Ardupilot interface
            a_n(0) += factor * (states[i].ax + alpha(0));
            a_n(1) += factor * (states[i].ay + alpha(1));
            a_n(2) += factor * (states[i].az + alpha(2));
          };

          state.vx = v_n(0);
          state.vy = v_n(1);
          state.vz = v_n(2);

          /*
           let the internal localstate acc be in body
           state.ax = a_n(0);
           state.ay = a_n(1);
           state.az = a_n(2);
           */

          state.phi = 0;
          state.theta = 0;
          state.psi = this->centroidHeading(states);

          state.p = 0;
          state.q = 0;
          state.r = 0;  //TODO: find expression for this
          omega(2) = state.r;

          v_b = getBodyVelocity();
          state.u = v_b(0);
          state.v = v_b(1);
          state.w = v_b(2);
          a_b = getBodyAcceleration();
          state.ax = a_b(0);
          state.ay = a_b(1);
          state.az = a_b(2);

          //all vehicles should have the same reference point
          state.lat     = states[0].lat;
          state.lon     = states[0].lon;
          state.height  = states[0].height;
        }
      private:

        fp32_t
        centroidHeading(std::vector<IMC::EstimatedLocalState> &states)
        {
          if (this->m_N > 1)
          {
            Matrix p = Matrix(2, this->m_N, 0);
            for (int i = 0; i < this->m_N; i++)
            {
              p(0, i) = states[i].x;
              p(1, i) = states[i].y;
            }
            Matrix p_diff = p.column(1) - p.column(0);

            return Angles::normalizeRadian(-std::atan2(p_diff(0), p_diff(1)));
          }
          else
          {
            return states[1].psi;
          }
        }

        Matrix
        getBodyVelocity()
        {
          Matrix Rcn = transpose(Rz(state.psi));
          return Rcn * v_n;
        }

        Matrix
        getBodyAcceleration()
        {
          Matrix Rcn = transpose(Rz(state.psi));
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

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx) :
            DUNE::Tasks::Task(name, ctx)
        {
          param("Formation Centroid", m_args.use_task)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .scope(Tasks::Parameter::SCOPE_MANEUVER)
          .defaultValue("false")
          .description("Enable Formation Centroid EstimatedLocalState.");

          param("Vehicle List", m_args.formation_systems)
          .defaultValue("")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("System name list of the formation vehicles.");

          bind<IMC::EstimatedLocalState>(this);
        }

        //! Update internal state with new parameter values.
        void
        onUpdateParameters(void)
        {
          debug("Starting update of parameters.");
          if (paramChanged(m_args.formation_systems))
          {
            inf("New Formation vehicles' list.");
            m_vehicles.setVehicleList(m_args.formation_systems, this);
            states = std::vector<IMC::EstimatedLocalState>(m_vehicles.m_N);
          }
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
        consume(const IMC::EstimatedLocalState* msg)
        {
          std::string vh_name = resolveSystemId(msg->getSource());
          spew("Got EstimatedLocalState \nfrom '%s'", vh_name.c_str());

          if (msg->getSource() == this->getSystemId())
            spew("Entity '%s'", resolveEntity(msg->getSourceEntity()).c_str());
          int vh_id = m_vehicles.getVehicle(msg->getSource());
          spew("Vehicle[%d]", vh_id);
          if (vh_id != -1)
          {
            states[vh_id] = *msg;
            spew("Pos x: [%f]", states[vh_id].x);
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
            m_centroid.calculateCentroid(states, m_vehicles.m_N);
            spew(
                "Dispatching centroid EstimatedLocalState, heading: %f [deg], noAgents=[%d]",
                Angles::degrees(m_centroid.state.psi), m_centroid.m_N);
            //set sourceEntity here, make sure NOT to send the centroid message inbetween vehicles
            m_centroid.state.ots = msg->getTimeStamp();
            dispatch(m_centroid.state);
          }
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
      };
    }
  }
}

DUNE_TASK

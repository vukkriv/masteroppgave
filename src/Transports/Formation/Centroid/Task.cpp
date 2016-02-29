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
        Vehicles():
          m_i(0),
          m_N(0)
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
           connected = std::vector<bool> (m_N,false);
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
        Centroid()
        {
        }
        IMC::EstimatedLocalState state;
        int m_N;

        void
        calculateCentroid(std::vector<IMC::EstimatedLocalState> states, int noAgents)
        {                    
          this->m_N = noAgents;
          for (int i=0; i < noAgents; i++)
          {
              state.x  +=  1/noAgents*(states[i].x);
              state.y  +=  1/noAgents*(states[i].y);
              state.z  +=  1/noAgents*(states[i].z);
              state.vx +=  1/noAgents*(states[i].vx);
              state.vy +=  1/noAgents*(states[i].vy);
              state.vz +=  1/noAgents*(states[i].vz);
          };

          state.phi   = 0;
          state.theta = 0;          
          state.psi   = this->centroidHeading(states);

          state.p   = 0;
          state.q   = 0;          
          state.r   = 0;          
          
          Matrix v_b = getBodyVelocity();
          state.u = v_b(0);
          state.v = v_b(1);
          state.w = v_b(2);            
          Matrix a_b = getBodyAcceleration();
          state.ax = a_b(0);
          state.ay = a_b(1);
          state.az = a_b(2);                      
        }
        private:
        
        fp32_t 
        centroidHeading(std::vector<IMC::EstimatedLocalState> states)
        {
          Matrix p = Matrix(2, this->m_N, 0);
          for (int i=0; i < this->m_N; i++)
          {
            p(0,i) = states[i].x;
            p(1,i) = states[i].y; 
          }
          return atan2(p(0,1)-p(0,0),p(1,1)-p(1,0));
        }

        Matrix
        getBodyVelocity()
        {
          return Matrix(3,1,0);
        }

        Matrix
        getBodyAcceleration()
        {
          return Matrix(3,1,0);          
        }        

      };

      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments
        Arguments m_args;

        //! SourceIDs of all vehicles in formation
        Vehicles m_vehicles;

        //! State of centroid
        Centroid m_centroid;

        std::vector<IMC::EstimatedLocalState> states;

        //! Constructor.
        //! @param[in] name task name.
        //! @param[in] ctx context.
        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx)
        {
         param("Formation Centroid", m_args.use_task).visibility(
              Tasks::Parameter::VISIBILITY_USER).scope(
              Tasks::Parameter::SCOPE_MANEUVER).defaultValue("false").description(
              "Enable Formation Centroid EstimatedLocalState.");

         param("Vehicle List", m_args.formation_systems).defaultValue("").visibility(
              Tasks::Parameter::VISIBILITY_USER).description(
              "System name list of the formation vehicles.");

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
            m_vehicles.setVehicleList(m_args.formation_systems,this);
            states = std::vector<IMC::EstimatedLocalState> (m_vehicles.m_N);
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
          trace("Got EstimatedLocalState \nfrom '%s' at '%s'",
          resolveEntity(msg->getSourceEntity()).c_str(),
          resolveSystemId(msg->getSource()));

          //fitler on sourceEntity, do not consume message which was sent from this task, by
          if (msg->getSourceEntity() == this->getEntityId())
            return;

          std::string vh_name = resolveSystemId(msg->getSource());          
          int vh_id = m_vehicles.getVehicle(msg->getSource());
          spew("Vehicle[%d]",vh_id);
          if (vh_id != -1)           
            states[vh_id] = *msg;
          else
            spew("Unknown vehicle");
/*
          for (unsigned int i=0; i < m_vehicles.m_N; i++)
          {
            spew("msgSource=%d",msg->getSource());
            spew("m_uav_ID[%d]=%d",i,m_vehicles.m_uav_ID[i]);
          }
*/
          //calculate centroid (if received from all)
          if (m_vehicles.allConnected())
          {
            spew("All vehicles connected");
            m_centroid.calculateCentroid(states,m_vehicles.m_N);
            spew("Dispatching centroid EstimatedLocalState, heading: %f [deg]",Angles::degrees(m_centroid.state.psi));
            //set sourceEntity here, make sure NOT to send the centroid message inbetween vehicles

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

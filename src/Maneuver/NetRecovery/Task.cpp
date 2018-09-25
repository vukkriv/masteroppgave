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
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Maneuver
{
  namespace NetRecovery
  {
    using DUNE_NAMESPACES;

    struct Vehicles
    {
      int no_vehicles;
      std::string aircraft;
      std::vector<std::string> copters;
    };

    enum Vehicle
    {
      FIXEDWING = 0, COPTER_MASTER, COPTER_SLAVE, INVALID = -1
    };


    struct Task: public DUNE::Maneuvers::Maneuver
    {
      Vehicles m_vehicles;
      Vehicle m_vehicle;
      int m_currWP;
      bool m_near_WP;
      IMC::NetRecovery m_maneuver;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Maneuvers::Maneuver(name, ctx),
        m_currWP(0),
        m_near_WP(false)
      {
        bindToManeuver<Task, IMC::NetRecovery>();
        bind<IMC::NetRecoveryState>(this);
        bind<IMC::PathControlState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      void
      consume(const IMC::NetRecovery* maneuver)
      {
      	inf("NetRecovery maneuver received");
        m_maneuver = *maneuver;
        saveVehicles(m_maneuver.aircraft,m_maneuver.multicopters.c_str());
        m_vehicle = getVehicle(resolveSystemId(m_maneuver.getSource()));
        debug("Vehicle: %d",(int)m_vehicle);

        if (m_vehicle != FIXEDWING)
        {
          //First disable all
          setControl(0);
          // Enable control loops
          setControl(IMC::CL_PATH);
        }

        switch(m_vehicle)
        {
          case FIXEDWING:
            m_currWP = 1;
            sendFixedWingPath();
            break;
          case COPTER_MASTER:
            sendCopterPath();
            break;
          case COPTER_SLAVE:
            //should not handle NetRecovery
            war("NetRecovery maneuver is enabled on the slave copter");
            break;
          case INVALID:
            war("NetRecovery maneuver is enabled on a non-supported vehicle");
            break;
        }
      }

      void
      consume(const IMC::NetRecoveryState* state)
      {
    	  spew("NetRecoveryState received");
    	  if (state->flags == IMC::NetRecoveryState::NR_STOP)
    	  {
          if (m_vehicle==COPTER_MASTER)
          {
            stopManeuver();
          }
    	  }
    	  else
    	  {
          //status from NetRecovery path controller received
    	  }
      }
      void
      consume(const IMC::PathControlState* pcs)
      {
        if (m_vehicle==FIXEDWING)
        {
          signalProgress(pcs->eta);

          if (pcs->flags & IMC::PathControlState::FL_NEAR)
          {
            debug("PathControlState FL_NEAR at WP %d",m_currWP);
            if ((m_currWP == 2) & (pcs->end_lat == m_maneuver.end_lat) & (pcs->end_lon == m_maneuver.end_lon) )
            {
              stopManeuver();
            }
            else if((m_currWP == 1) & (pcs->end_lat == m_maneuver.start_lat) & (pcs->end_lon == m_maneuver.start_lon) )
            { 
              m_currWP += 1;
              sendFixedWingPath();
            }
          }

        }        
      }      
      void 
      stopManeuver()
      {
        inf("NetRecoveryState STOP, maneuver is done");
        signalCompletion();
        if (m_vehicle != FIXEDWING)
        {
          setControl(0);
        }
      }

      void 
      sendCopterPath()
      {
        IMC::DesiredNetRecoveryPath d_path;
        d_path.start_lat = m_maneuver.start_lat;
        d_path.start_lon = m_maneuver.start_lon;
        d_path.end_lat = m_maneuver.end_lat;
        d_path.end_lon = m_maneuver.end_lon;
        d_path.z = m_maneuver.z;
        d_path.lbox_height = m_maneuver.lbox_height;
        d_path.lbox_width = m_maneuver.lbox_width;
        d_path.max_acc  = m_maneuver.max_acc;
        d_path.speed = m_maneuver.speed;
        d_path.z_off = m_maneuver.z_off;
        d_path.z_units = m_maneuver.z_units;
        d_path.aircraft = m_maneuver.aircraft;
        d_path.multicopters = m_maneuver.multicopters;

        dispatch(d_path);
        inf("DesiredNetRecoveryPath dispatched");
      }

      void
      sendFixedWingPath()
      {
        IMC::DesiredPath path;
        path.speed = m_maneuver.speed;
        path.speed_units = IMC::SUNITS_METERS_PS;
        path.end_z_units = m_maneuver.z_units;
        path.end_z = m_maneuver.z + m_maneuver.z_off;
        switch (m_currWP)
        {
          case 1:
            path.end_lat = m_maneuver.start_lat;
            path.end_lon = m_maneuver.start_lon;
            debug("WP 1");
            break;
          case 2:
            path.end_lat = m_maneuver.end_lat;
            path.end_lon = m_maneuver.end_lon;
            debug("WP 2");
            break;
        }
        dispatch(path);
        inf("DesiredPath WP %d dispatched to fixed-wing",m_currWP);
      }

      void
      saveVehicles(std::string aircraft, std::string copters)
      {
        m_vehicles.aircraft = aircraft;
        m_vehicles.copters  = std::vector<std::string>();
        std::stringstream lineStream(copters);
        std::string copter;
        while (std::getline(lineStream, copter, ','))
        {
          m_vehicles.copters.push_back(copter);
        }

        debug("Aircraft: %s", m_vehicles.aircraft.c_str());
        for (unsigned int i = 0; i < m_vehicles.copters.size(); i++)
        {
          debug("Multicopter[%d]: %s", i, m_vehicles.copters[i].c_str());
        }
      }

      Vehicle
      getVehicle(std::string src_entity)
      {
        if (src_entity == m_vehicles.aircraft)
        {
          return FIXEDWING;
        }
        for (unsigned int i = 0; i < m_vehicles.copters.size(); i++)
        {
          if (src_entity == m_vehicles.copters[i])
          {
            if (i == 0)
              return COPTER_MASTER;
            else
              return COPTER_SLAVE;
          }
        }
        return INVALID;
      }
    };
  }
}

DUNE_TASK

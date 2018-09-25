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
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// LOCAL headers
#include "UM100Hal.hpp"



//#include "pool.h"
//#include "artls.h"
//#include "smac_options.h"



namespace Sensors
{
  namespace UM100SPI
  {
    using DUNE_NAMESPACES;

    static const unsigned int c_max_num_tags = 10;

    struct Arguments {
      // File name
      std::string device;
      // Role
      std::string role;
      // Initial address book
      std::vector<int> addressbook;
      // If coordinator, expect to hear from these tags
      std::vector<unsigned int> expected_tags;
    };


    struct Task: public DUNE::Tasks::Task
    {
      // Task arguments
      Arguments m_args;
      // Device
      UM100Hal* m_radio;
      // Options
      UM100_options m_options;
      // Role
      Role m_role;

      // Address book map
      // sender id <-> output ID
      std::map<unsigned int, unsigned int> m_device_addressbook;

      // The stateful beacons
      std::map<unsigned int, Entities::StatefulEntity*> m_beacon_entities;

      // Array of outgoing messages
      IMC::BeaconDistance m_bdistance[c_max_num_tags];

      // Map of timestamps of last messages
      std::map<unsigned int, double> m_timestamp_prev_measurement;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_radio(NULL),
        m_role(UNKNOWN)
      {
        param("Device", m_args.device)
        .defaultValue("/dev/ppdev1.0");

        param("Role", m_args.role)
        .defaultValue("tag")
        .values("Tag,Coordinator,Base")
        .description("Set the role of the device. ");

        param("Address Book", m_args.addressbook)
        .defaultValue("2008, 3007, 3202, 2106, 2923");

        param("Expected Connected Tags", m_args.expected_tags)
        .defaultValue("0,1,2,3,4");

        param("UM100 - Loc Rate", m_options.loc_rate)
        .defaultValue("0")
        .description("Use 0 for dynamic. ");

        param("UM100 - Slot Method", m_options.slotMethod_nb)
        .defaultValue("1")
        .description("Slot Method. Default 1");

        param("UM100 - Slot Method Increment", m_options.slotMethod_increment)
        .defaultValue("0")
        .description("Slot method increment. Default 0");

        param("UM100 - Antenna Offset", m_options.antenna_offset)
        .defaultValue("0.105")
        .units(Units::Meter)
        .description("Antenna Offset. ");

        // Hardcode datalength in bits for ext for now.
        m_options.extDataLenInBits = UWBDATA_EXTDATA_SIZE_IN_BITS_BASE;

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.role))
        {
          if (m_role != UNKNOWN)
            war("Not able to change role run-time. Ignoring. ");
          else
          {
            if (m_args.role == "Tag")
              m_role = TAG;
            else if (m_args.role == "Base")
              m_role = BASE;
            else if (m_args.role == "Coordinator")
              m_role = COORDINATOR;
            else
            {
              err("Invalid role supplied. Assuming tag. ");
              m_role = TAG;
            }
          }
        }
        if (paramChanged(m_args.addressbook))
        {
          if (m_device_addressbook.size() != 0)
            err("Unable to update default addressbook. Ignoring. ");
          else
          {
            for (unsigned int i = 0; i < m_args.addressbook.size(); i++)
            {
              m_device_addressbook[m_args.addressbook[i]] = i;
            }
          }

          for (std::map<unsigned, unsigned>::iterator it = m_device_addressbook.begin(); it != m_device_addressbook.end(); it++)
          {
            inf("Found: %d, with own id: %d", it->first, it->second);
          }
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        for( unsigned int i = 0; i < c_max_num_tags; ++i)
        {
          bool expected = false;
          for (std::vector<unsigned>::iterator it = m_args.expected_tags.begin(); it != m_args.expected_tags.end(); ++it)
          {
            if (i == *it)
            {
              expected = true;
              break;
            }
          }

          std::ostringstream ostr;
          ostr << "BeSpoon-" << i;

          unsigned int new_entity;

          if (expected)
          {
            m_beacon_entities[i] = reserveEntity<Entities::StatefulEntity>(ostr.str());
            new_entity = m_beacon_entities[i]->getId();
          }
          else
          {
            new_entity = reserveEntity(ostr.str());
          }

          m_bdistance[i].setSourceEntity(new_entity);
        }
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
        if (m_radio == NULL)
          m_radio = new UM100Hal(this, m_args.device, m_role, m_options);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        m_radio->initialize();
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_radio != NULL)
        {
          m_radio->close();
          delete m_radio;
          m_radio = NULL;
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        int timeout_ms = 500;

        unsigned int events_mask = 0;

        unsigned int n_consequtive_timeouts = 0;

        while (!stopping())
        {
          consumeMessages();

          PollResult pres = m_radio->poll(timeout_ms);

          if (pres == P_EVENT)
          {
            // Check which events got fired.
            events_mask = m_radio->parse_events();

            // Restart counter
            n_consequtive_timeouts = 0;

            if (events_mask & DATA_READY)
            {
              spew("DATA_READY");
              read_dispatch_new_data();
            }
            if (events_mask & NETWORK_MANAGEMENT)
            {
              spew("Handle network");
              m_radio->handle_network();
            }
            if (events_mask & HANGUP)
            {
              err("Driver issued hangup. Ignoring. ");
            }
            if (events_mask == EVENT_NONE)
            {
              spew("No event, boring. ");
            }
          }
          else if (pres == P_ERROR)
          {
            war("Got polling error. ");
          }
          else {
            // timeout
            war("Poll timeout. ");

            n_consequtive_timeouts++;

            if (n_consequtive_timeouts > 2)
            {
              throw RestartNeeded("Too many timeouts, restarting task. ", 0);
            }
          }

          // Check timeouts of states of the reserved beacons
          double now = Clock::getSinceEpoch();
          for (std::map<unsigned int, Entities::StatefulEntity*>::iterator it = m_beacon_entities.begin(); it != m_beacon_entities.end(); it++)
          {
            unsigned int id = it->first;
            Entities::StatefulEntity* entity = it->second;

            // TODO: Parameterize timeout
            if (now - m_bdistance[id].getTimeStamp() > 2.0)
            {
              entity->setState(EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
            }

          }

        }
      }

      void
      read_dispatch_new_data(void)
      {
        // Gets all new available measurements
        std::vector<Measurement> res = m_radio->read_measurements();

        // spew("Number of measurements: %zu", res.size());

        // Loop to process
        for (std::vector<Measurement>::iterator it = res.begin(); it != res.end(); it++)
        {
          std::vector<Measurement>::iterator m = it;
          unsigned int src = m->src;
          bool deviceInAddreessBook = false;

          // if cannot find the source in the map.
          if (m_device_addressbook.find(src) ==  m_device_addressbook.end())
          {
            // Try to add
            if (m_device_addressbook.size() < c_max_num_tags)
            {
              // Add
              unsigned int newId = m_device_addressbook.size();
              m_device_addressbook[src] = newId;
              deviceInAddreessBook = true;
              inf("Added new device: %d at id %d", src,m_device_addressbook[src] );
            }
            else
            {
              // We are full, do not use the address book
              deviceInAddreessBook = false;
            }
          }
          else
          {
            deviceInAddreessBook = true;
          }

          if (deviceInAddreessBook)
          {
            unsigned int id = m_device_addressbook[src];

            m_bdistance[id].dist = m->distance;
            m_bdistance[id].sender = m->src;
            m_bdistance[id].receiver = m->dst;
            m_bdistance[id].rssi = m->RSSI;
            m_bdistance[id].dqf = m->quality_factor;
            m_bdistance[id].time = m->time * 1E9;
            m_bdistance[id].dlt = m->delta_time * 1E9;

            m_beacon_entities[id]->setState(EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

            dispatch(m_bdistance[id]);
          }

          IMC::BeaconDistance bd;

          bd.dist = m->distance;
          bd.sender = m->src;
          bd.receiver = m->dst;
          bd.rssi = m->RSSI;
          bd.dqf = m->quality_factor;
          bd.time = m->time * 1E9;
          bd.dlt = m->delta_time * 1E9;

          dispatch(bd);
        }
      }
    };
  }
}

DUNE_TASK

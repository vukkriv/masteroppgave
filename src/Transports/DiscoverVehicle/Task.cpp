//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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
  namespace DiscoverVehicle
  {
    using DUNE_NAMESPACES;
    //! %Task arguments.
    struct Arguments
    {
      //! Entity label for transport layer
      std::string layer_entlab;

      //! Service type
      std::string service;

      //! Vehicle list
      std::vector<std::string> vehicles;
    };

    struct Destination
    {
      bool connected;
      std::string vehicle;
      std::string ip;
      std::string port;
      std::string ip_port;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;

      unsigned int m_N;
      std::vector<Destination> m_destinations;
      Destination m_curr_parsed;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_N(0)
      {
        param("Transport Entity Label", m_args.layer_entlab)
        .defaultValue("")
        .description("Entity label for transport layer");

        param("Service type", m_args.service)
        .defaultValue("")
        .description("Transport layer service type");

        param("Vehicle List", m_args.vehicles)
        .defaultValue("")
        .description("Vehicles in transport layer (comma separated list with vehicle names)");

        bind<IMC::Announce>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        m_N = static_cast<unsigned int>(m_args.vehicles.size())-1;
        m_destinations = std::vector<Destination>(m_N);
        m_destinations.clear();
        for (unsigned int i=0; i < m_N + 1; i++)
        {
          if ( strcmp(m_args.vehicles[i].c_str(),resolveSystemId(this->getSystemId())) != 0 )
          {
            Destination dest;
            dest.connected = false;
            dest.vehicle = m_args.vehicles[i];
            dest.ip = "";
            dest.port = "";
            dest.ip_port = "";
            m_destinations.push_back(dest);
            trace("intervehicle node '%s' added to desired destinations",m_args.vehicles[i].c_str());
          }
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
      consume(const IMC::Announce* msg)
      {
        // check if local message
        if (this->getSystemId() == msg->getSource())
          return;

        spew("Got Announce from system '%s'.",
        resolveSystemId(msg->getSource()));

        // consider checking if connection is lost and stop sending as well ?
        if (allConnected())
        {
          spew("All vehicle discovered and connected");
          return;
        }

        // parse service string
        if (parseServices(msg->services))
        {
          //if desired service found,
          //and if vehicle is a desired destination,
          //set ip and port
          for(std::vector<Destination>::iterator it = m_destinations.begin(); it != m_destinations.end(); ++it)
          {
              if (it->vehicle == resolveSystemId(msg->getSource()) && !it->connected)
              {
                it->connected = true;
                it->ip = m_curr_parsed.ip;
                it->port = m_curr_parsed.port;
                it->ip_port = m_curr_parsed.ip_port;
                inf("new intervehicle node within range '%s' / %s / %s",it->vehicle.c_str(),m_curr_parsed.port.c_str(),m_curr_parsed.ip.c_str());
                // set Static Destination in UDP transport layer
                updateDestinations();
              }
          }
        }

      }

      bool
      parseServices(std::string services)
      {
        std::string token;
        while(token != services){
          token = services.substr(0,services.find_first_of(";"));
          services = services.substr(services.find_first_of(";") + 1);
          //spew("parseServiceElem: %s",token.c_str());
          if (parseServiceElem(token.c_str()))
            return true;
        }
        return false;
      }

      bool
      parseServiceElem(const char* service_elem)
      {
        // Check if we should read this announcement.
        char service [128] = {0};

        sscanf (service_elem,"%[^:]://%*[^:]:%*[^/]/",service);
        if (strcmp(service,m_args.service.c_str())!=0)
          return false;

        spew("Announce has desired service: %s",service);

        char ip [128] = {0};
        char port [128] = {0};
        sscanf (service_elem,"%*[^:]://%[^:]:%[^/]/",ip,port);
        if (strlen(ip)==0 || strlen(port)==0)
          return false;
        //sscanf (msg->service.c_str(),"%[^:]://%[^:]:%[^/]/",service,ip,port);

        char ip_port [128] = {0};
        sscanf (service_elem,"%*[^/]//%[^/]/",ip_port);
        if (strlen(ip_port)==0)
          return false;

        m_curr_parsed.ip       = ip;
        m_curr_parsed.port     = port;
        m_curr_parsed.ip_port = ip_port;
        return true;
      }

      void
      updateDestinations()
      {
        const char* separator = "";
        std::stringstream os;

        for(std::vector<Destination>::iterator it = m_destinations.begin(); it != m_destinations.end(); ++it)
        {
            if (it->connected)
            {
              os << separator << it->ip_port;
              separator = ",";
            }
        }
        trace("%s : Static Destinations = %s",m_args.layer_entlab.c_str(),os.str().c_str());
        // then set the Static Destination in the desired Transport layer
        setStaticDestinations(os.str());
      }

      void
      setStaticDestinations(std::string dest)
      {
        IMC::SetEntityParameters* static_dest = new IMC::SetEntityParameters();
        static_dest->name = m_args.layer_entlab;
          IMC::MessageList<IMC::EntityParameter> entityParameters;
          IMC::EntityParameter* ep = new IMC::EntityParameter();
          ep->name = "Static Destinations";
          ep->value = dest;
          entityParameters.push_back(*ep);
          delete ep;
        static_dest->params = entityParameters;
        dispatch(static_dest);
      }

      bool
      allConnected()
      {
        for(std::vector<Destination>::iterator it = m_destinations.begin(); it != m_destinations.end(); ++it)
        {
            if (!it->connected)
              return false;
        }
        return true;
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

DUNE_TASK

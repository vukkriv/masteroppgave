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

// Library headers
#include <vector>
#include <map>

namespace Monitors
{
  namespace Ping
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Timeout
      double timeout;

      //! Number of timeouts to delete entry
      int n_timeouts_before_delete;
    };

    class RunningStat
    {
    public:
      RunningStat() : m_n(0),m_oldM(0.0),m_newM(0.0), m_oldS(0.0),m_newS(0.0) {}

      void clear()
      {
        m_n = 0;
      }

      void push(double x)
      {
        m_n++;

        // See Knuth TAOCP vol 2, 3rd edition, page 232
        if (m_n == 1)
        {
          m_newM = x;

          m_oldM = m_newM;
          m_oldS = 0.0;
        }
        else
        {
          m_newM = m_oldM + (x - m_oldM)/m_n;
          m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

          // set up for next iteration
          m_oldM = m_newM;
          m_oldS = m_newS;
        }
      }

      int numDataValues() const
      {
        return m_n;
      }

      double mean() const
      {
        return (m_n > 0) ? m_newM : 0.0;
      }

      double variance() const
      {
        return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
      }

      double standardDeviation() const
      {
        return sqrt( variance() );
      }

    private:
      int m_n;
      double m_oldM, m_newM, m_oldS, m_newS;
    };

    class PingRequest
    {
    public:
      PingRequest(): id(0), got_response(false), time_of_dispatch(0.0), time_of_response(0.0), ping(0.0) {};
      unsigned int id;
      bool got_response;
      double time_of_dispatch;
      double time_of_response;
      double ping;
    };
    typedef std::vector<PingRequest> Requests;

    class AddressBookEntry
    {
    public:
      AddressBookEntry(): id(0), nextRequestId(0), name(""),ping_avg(0.0),ping_stddev(0.0),timeOfNextRequest(0.0),num_timeouts(0) {};
      unsigned int id;
      unsigned int nextRequestId;
      std::string name;
      double ping_avg;
      double ping_stddev;
      RunningStat statistics;
      double timeOfNextRequest;
      Requests requests;
      int num_timeouts;
    };

    typedef std::map<unsigned int, AddressBookEntry> AddressBook;

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments
      Arguments m_args;

      //! Address book
      AddressBook m_addressBook;

      //! Random number generator
      Math::Random::Generator* m_prndgen;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_prndgen(NULL)
      {
        m_prndgen = Math::Random::Factory::create("drand48");


        param("Timeout", m_args.timeout)
        .defaultValue("5");

        param("N Timeouts to Delete", m_args.n_timeouts_before_delete)
        .defaultValue("5");

        bind<IMC::Ping>(this);
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
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }



      void
      consume(const IMC::Ping *msg)
      {
        if (!sourceIsKnown(msg))
          addNewAddressBookEntry(msg);

        // From this point, only handle messages directed to this system
        if (msg->getDestination() != getSystemId())
          return;

        switch(msg->type)
        {
          case IMC::Ping::PT_REQUEST:
          {
            IMC::Ping response = generateResponse(msg);
            dispatch(response, DF_LOOP_BACK);
            spew("Got requests, id: %d ", msg->ping_id);
            break;
          }
          case IMC::Ping::PT_RESPONSE:
            handleResponseAndDispatchAck(msg);
            spew("Got response, id: %d", msg->ping_id);
            break;
          case IMC::Ping::PT_ACK:
            // NOP. Consider storing time as another measurement.
            break;
          case IMC::Ping::PT_TIMEOUT:
            // NOP
            break;
        }
      }

      void
      handleResponseAndDispatchAck(const IMC::Ping* response)
      {
        double now = Clock::get();
        // Loop active requests
        try
        {
          AddressBookEntry* entry = &m_addressBook.at(response->getSource());

          for (Requests::iterator it = entry->requests.begin(); it != entry->requests.end(); ++it)
          {
            if (it->id == response->ping_id)
            {
              // Found the right ID
              // Calculate the round-trip time
              double rtt = now - it->time_of_dispatch;

              // Calculate and store ping
              it->ping = rtt/2.0;

              // Store time of response
              it->time_of_response = now;

              // Generate and dispatch ack
              IMC::Ping ack;
              ack.type = IMC::Ping::PT_ACK;
              ack.setDestination(response->getSource());

              ack.ping_id = it->id;
              ack.time = it->ping;

              dispatch(ack, DF_LOOP_BACK);

              trace("Ping from %s is %f ms. Avg: %f, stddev: %f", entry->name.c_str(), it->ping*1000.0, entry->ping_avg*1000, entry->ping_stddev*1000);
            }
          }


        }
        catch (std::out_of_range &ex)
        {
          err("Trying to handle response from unknown source: %s; %s", resolveSystemId(response->getSource()), ex.what());
        }
      }

      IMC::Ping
      generateResponse(const IMC::Ping* request)
      {
        IMC::Ping response;
        response.ping_id = request->ping_id;
        response.type = IMC::Ping::PT_RESPONSE;
        response.time = request->time;

        response.setDestination(request->getSource());

        return response;
      }

      bool
      sourceIsKnown(const IMC::Ping* msg)
      {
        if (m_addressBook.count(msg->getSource()) == 1)
            return true;

        return false;
      }

      // Add entry to address book.
      // Assumes entry is new. Otherwise, will overwrite entry.
      void
      addNewAddressBookEntry(const IMC::Ping* msg)
      {
        AddressBookEntry entry;

        entry.id    = msg->getSource();
        entry.name  = resolveSystemId(msg->getSource());
        entry.nextRequestId = 0;
        entry.num_timeouts = 0;
        entry.timeOfNextRequest = Clock::get();

        m_addressBook[msg->getSource()] = entry;

        inf("Adding new entry: %s", entry.name.c_str());
      }

      void
      cleanRequestsUpdateStatistics(void)
      {
        double now = Clock::get();

        // Loop over all request entries, clean completed requests and update statistics
        for (AddressBook::iterator entry = m_addressBook.begin(); entry != m_addressBook.end(); )
        {
          bool deletedEntry = false;

          // In the following, mind that erasing while iterating requires special attention
          for (Requests::iterator request = entry->second.requests.begin(); request != entry->second.requests.end(); )
          {
            bool deletedRequest = false;

            if (request->time_of_response > 0)
            {
              // Update statistics
              entry->second.statistics.push(request->ping);

              entry->second.ping_avg = entry->second.statistics.mean();
              entry->second.ping_stddev = entry->second.statistics.standardDeviation();

              // Delete request
              request = entry->second.requests.erase(request);
              deletedRequest = true;

            }

            else if (now - request->time_of_dispatch > m_args.timeout)
            {
              // Handle timeout
              entry->second.num_timeouts++;

              // Delete request
              request = entry->second.requests.erase(request);
              deletedRequest = true;

              if (entry->second.num_timeouts > m_args.n_timeouts_before_delete)
              {
                inf("Deleted entry for %s due to to many timeouts. ", entry->second.name.c_str());

                // Special handling for in-loop erase with C11 maps.
                AddressBook::iterator entry_next = entry;

                if (entry == m_addressBook.end())
                  entry_next = m_addressBook.end();
                else
                  entry_next++;

                m_addressBook.erase(entry);
                entry = entry_next;
                deletedEntry = true;


                // Need to break the inner loop.
                break;
              }

            }

            if (!deletedRequest)
              ++request;
          }

          if (!deletedEntry) // Due to the way C11 handles maps, iterate anyway.
            ++entry;
        }
      }

      void
      sendNewRequests(void)
      {

        double now = Clock::get();

        // Loop entries, create requests
        for (AddressBook::iterator entry = m_addressBook.begin(); entry != m_addressBook.end(); ++entry)
        {
          if (entry->second.timeOfNextRequest < now)
          {
            IMC::Ping request;
            request.type = IMC::Ping::PT_REQUEST;
            request.setDestination(entry->second.id);
            request.ping_id = entry->second.nextRequestId;
            request.time = entry->second.statistics.mean(); // Fill with mean just to have something.

            // Store the request
            PingRequest req;
            req.id = request.ping_id;
            req.time_of_dispatch = now;

            //entry->second.requests.push_back(req);

            // test;
            AddressBookEntry *tmp = &m_addressBook.at(entry->first);
            tmp->requests.push_back(req);

            // Dispatch request
            dispatch(request, DF_LOOP_BACK);
            spew("Dispatched request. ");

            // Set next time of request and update next id.
            // For now, a random time between .1 and 5 seconds.
            entry->second.nextRequestId++;
            entry->second.timeOfNextRequest = now + m_prndgen->uniform(0.1, 5.0);

          }
        }
      }

      void
      printBook(void)
      {
        // Loop entries, create requests
        for (AddressBook::iterator entry = m_addressBook.begin(); entry != m_addressBook.end(); ++entry)
        {
          inf("%s, timeouts: %d:", entry->second.name.c_str(), entry->second.num_timeouts);
          for (unsigned int i = 0; i < entry->second.requests.size(); i++)
          {
            std::cout << entry->second.requests[i].id;
          }
          std::cout << std::endl;
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        // Kick-start the thing by adding self
        IMC::Ping starter;
        starter.type = IMC::Ping::PT_TIMEOUT;

        starter.setSource(getSystemId());
        starter.setDestination(getSystemId());

        addNewAddressBookEntry(&starter);

        while (!stopping())
        {
          waitForMessages(1.0);
          cleanRequestsUpdateStatistics();
          sendNewRequests();
        }
      }
    };
  }
}

DUNE_TASK

//***************************************************************************
// Copyright 2007-2017 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
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

// UM100 headers
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stddef.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/reboot.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <signal.h>

#define CONFIG_RTLS_PROTOCOL
#define CONFIG_RTLS_FULL_LOC
#define ARM


extern "C" {
  #include <osal_type.h>
  //#include <osal_trace.h>
  #include <osal_time.h>
  #include <osal_stdlib.h>
  #include <pinpointer_common_def.h>
  #include <pinpointer_protocol_def.h>
  #include <pinpointer_drv_api.h> // Probly dont need.
  #include <pinpointer_drv_api_protocol.h>

  #include <ranging_data_depayload.h>

}

#define ioctl_w(fd, request, data) _ioctl_w(fd, request, data, #request)

//#include "pool.h"
//#include "artls.h"
//#include "smac_options.h"



namespace Sensors
{
  namespace UM100SPI
  {
    using DUNE_NAMESPACES;

    static const size_t c_trame_buff_max_size = sizeof(rng_protocol_header_t)
                                                + sizeof(rng_protocol_entry_t)*MAX_FRAME_PER_SUPERFRAME;

    struct Arguments {
      // File name
      std::string device;
    };

    struct Task: public DUNE::Tasks::Task
    {
      // Task arguments
      Arguments m_args;
      // File descriptor
      struct pollfd m_pollfds;
      // Input buffer
      rng_protocol_header_t* m_trame_buff;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Device", m_args.device)
        .defaultValue("/dev/ppdev1.0");

        // TODO: Move to resource acq/release. (Release with null check)
        m_trame_buff = (rng_protocol_header_t*) malloc(c_trame_buff_max_size);
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

        m_pollfds.fd = open(m_args.device.c_str(), O_RDWR);

        if (m_pollfds.fd < 0)
        {
          throw RestartNeeded("Unable to open device. ", 5.0);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        // Init protocol
        if(ioctl_w(m_pollfds.fd, PP_IOC_OPEN, NULL) < 0)
          throw RestartNeeded("Unable to open protocol. ", 5.0);

        // TODO:
        // Set caps: PP_IOC_S_PARM

        // TODO:
        // Set Frame Info: PP_IOC_S_SFRAMEINF

        // TODO:
        // Init devices?

        // TODO:
        // Set moving

        // TODO:
        // Init depayload: ranging_data_depayload_init

        // TODO:
        // Set EXT_DATA_SIZE: cmd_type = ARTLS_SET_EXT_DATA_SIZE, PP_IOC_S_ARTLS

        // Init frames and start protocol
        // These possibly needs to be set.
        start_param_t startTime;
        startTime.mega_frame = 0;
        startTime.hyper_frame = 0;
        startTime.super_frame = 0;
        if(ioctl_w(m_pollfds.fd, PP_IOC_START_PROTOCOL, &startTime) < 0)
          throw RestartNeeded("Unable to start protocol. ", 5.0);

        // TODO:
        // Set Bind Coord
        // cmd_type = ARTLS_SET_PREFERED_REF_BASE_DOWN;
        // PP_IOC_S_ARTLS

        // TODO:
        // Set threshold coeffs
        // cmd_type = ARTLS_SET_THRESHOLD_COEFF_DOWN
        // PP_IOC_S_ARTLS

        // TODO:
        // Set Antenna offset
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if(ioctl_w(m_pollfds.fd,PP_IOC_STOP_PROTOCOL,NULL) < 0)
          war("Unable to stop protocol. ");
        if(ioctl_w(m_pollfds.fd,PP_IOC_CLOSE,NULL) < 0)
          war("Unable to close connection. ");

        close(m_pollfds.fd);
      }

      //! Read and parse data
      void read_parse_data()
      {
        size_t cur_size=read( m_pollfds.fd,
                              m_trame_buff,
                              c_trame_buff_max_size);

        if(cur_size > 0)
        {
          unsigned char* data=(unsigned char*) m_trame_buff;
          data+=sizeof(rng_protocol_header_t);
          for(int rng_idx=0; rng_idx < m_trame_buff->nb_entry; rng_idx++)
          {
            type_rdv_t rdvtype= (type_rdv_t) 0;
            type_trame_t ttype=TT_UNKNOWN;
            rng_protocol_entry_t* curEntry = (rng_protocol_entry_t*) data;
            rng_protocol_entry_basic_t* rng=NULL;
            beacon_protocol_entry_basic_t* b=NULL;
            beacon1_protocol_entry_basic_t* b1=NULL;
            beacon2_protocol_entry_basic_t* b2=NULL;
            rdv_protocol_entry_basic_t* rdv=NULL;
            OSAL_u16 entrySize=0;
            if(curEntry==NULL)
            {
              printf("SOMETHING STRANGE.CHECK CODE\n");
              continue;
            }
            if(ranging_data_depayload_getLocData(curEntry,
                     OSAL_false,\
                     &ttype,
                     &rdvtype,
                     &entrySize,
                     &b,
                     &b1,
                     &b2,
                     &rng,
                     &rdv,
                     NULL,
                     NULL,
                     NULL,
                     NULL,
                     NULL,
                     NULL,
                     NULL)!=0)
            {
              printf("ranging_data_depayload_getLocData failure!!\n");
              fflush(stdout);
              break;
            }
            data +=entrySize;
            if((b == NULL)&&(b1 == NULL)&&(b2 == NULL)&&(rng == NULL)&&(rdv == NULL))
              continue;//redundant with previous test?
          }
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        int timeout_ms = 500;

        m_pollfds.events = POLLIN|POLLPRI|POLLHUP;
        while (!stopping())
        {
          consumeMessages();

          if(poll(&m_pollfds, 1, timeout_ms) < 0)
            err("poll error: %s\n", strerror(errno));

          // TODO: Do time management?

          // Check events.
          if(m_pollfds.revents & POLLIN)
          {
            read_parse_data();
          }
          if(m_pollfds.revents & POLLPRI)
          {
            // TODO: Handle ARTLS

          }
          // Driver hangup, close,
          if(m_pollfds.revents & (POLLHUP|POLLERR))
          {
            // TODO: Handle (if needed?)
          }



        }
      }

      int _ioctl_w(int fd, int request, void* data, const char* ioctl_name)
      {
        int status = ioctl(fd, request, data);
        if(status < 0)
          war("ioctl(%s) failed with ret %d: %s\n", ioctl_name, status, strerror(errno));
        return status;
      }

    };
  }
}

DUNE_TASK

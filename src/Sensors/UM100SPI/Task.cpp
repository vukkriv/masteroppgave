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

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#endif


#define SFI_RATE_NORMAL 1
#define SFI_RATE_FAST 2

// Chose rate
#define SFI_RATE_USE_FAST

extern "C" {
  #include <osal_type.h>
  //#include <osal_trace.h>
  #include <osal_time.h>
  #include <osal_stdlib.h>
  #include <pinpointer_common_def.h>
  #include <pinpointer_protocol_def.h>
  #include <pinpointer_drv_api.h>
  #include <pinpointer_drv_api_protocol.h>
  #include <ranging_data_depayload.h>
  #include <pool.h>
  #include <artls.h>

#ifdef SFI_RATE_USE_FAST
  #include <uavlab_sfi_fast.h>
#else
  #include "default_sfi_normal_rate.h"
#endif
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
      // Role
      std::string role;
    };

    struct UM100_options
    {
      // Capabilitites
      uint32_t capabilitites;
      // Superframe info
      superframe_info_t sfi;
      // Loc rate. Default 0
      uint8_t loc_rate;
      // Slot method. Default 1
      uint8_t slotMethod_nb;
      // Slot method increment. Default 0
      uint8_t slotMethod_increment;
      // External data length in bits.
      uint32_t extDataLenInBits;
      // Antenna offset
      uint16_t antenna_offset;
    };

    enum Role
    {
      UNKNOWN = 0,
      TAG = 1,
      BASE = 2,
      COORDINATOR = 3,
    };

    struct Task: public DUNE::Tasks::Task
    {
      // Task arguments
      Arguments m_args;
      // File descriptor
      struct pollfd m_pollfds;
      // Input buffer
      rng_protocol_header_t* m_trame_buff;
      // Options
      UM100_options m_options;
      // Role
      Role m_role;

      // UM100 States
      // Last response from kernel driver
      artls_up_t m_artls_out;
      // Pointer to data to send down
      artls_down_t* m_partls_in;
      // Pool to keep track of kernel driver input requests.
      pool_t* m_artls_down_pool;
      // Number of devices to use
      OSAL_u32 m_maxdev_nb;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_role(UNKNOWN),
        m_maxdev_nb(32)
      {
        param("Device", m_args.device)
        .defaultValue("/dev/ppdev1.0");

        param("Role", m_args.role)
        .defaultValue("tag")
        .values("Tag,Coordinator,Base")
        .description("Set the role of the device. ");

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
        .defaultValue("0")
        .description("Antenna Offset. ");

        // TODO: Move to resource acq/release. (Release with null check)
        m_trame_buff = (rng_protocol_header_t*) malloc(c_trame_buff_max_size);

        m_partls_in = NULL;

        // Initialize pool of ARTLS commands
        m_artls_down_pool = allocate_pool(sizeof(artls_down_t), 50);

        // Hardcode datalength in bits for ext for now.
        m_options.extDataLenInBits = UWBDATA_EXTDATA_SIZE_IN_BITS_BASE;

        // Initialize
        m_pollfds.fd = -1;
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

        // Set caps: PP_IOC_S_PARM
        struct dev_param params;
        params.caps = getCapabilities();
        if(ioctl_w(m_pollfds.fd, PP_IOC_S_PARM, &params) < 0)
          throw RestartNeeded("Unable to set capabilitites. ", 5.0);

        // Set Frame Info: PP_IOC_S_SFRAMEINF
        set_superframe_schema(getCapabilities(), &m_options.sfi);
        int status = ioctl_w(m_pollfds.fd, PP_IOC_S_SFRAMEINF, &m_options.sfi);
        if(status < 0)
        {
          if(getCapabilities() & DEVCAPS_UWB_SLAVE)
            war("assume --no_superframe_info\n");
          else
            throw RestartNeeded("Unable to set superframe info. ", 5.0);
        }

        // TODO: Check/fix memory leaks on ARTLS.H/C
        // Init devices?
        // NB: This allocates internal data/state in the artls.c file.
        init_devices(&m_maxdev_nb,
                     &m_options.sfi,
                     m_options.loc_rate,
                     m_options.slotMethod_nb,
                     m_options.slotMethod_increment);

        // TODO:
        // Set moving
        war("For now, not setting moving bit. ");


        // Init depayload: ranging_data_depayload_init
        if(ranging_data_depayload_init(m_options.extDataLenInBits)!=0)
          err("ranging_data_depayload_init error");

        // Set EXT_DATA_SIZE: cmd_type = ARTLS_SET_EXT_DATA_SIZE, PP_IOC_S_ARTLS
        m_partls_in = (artls_down_t*)pool_get_buffer(m_artls_down_pool);
        if(m_partls_in)
        {
          m_partls_in->cmd_type = ARTLS_SET_EXT_DATA_SIZE;
          m_partls_in->cmd_tracking = AT_NO_TRACKING;
          m_partls_in->user_data = 0;
          m_partls_in->sf_start = 0;
          m_partls_in->sf_period = 0;
          m_partls_in->payload.extDataLenInBits = m_options.extDataLenInBits;
          ioctl_w(m_pollfds.fd, PP_IOC_S_ARTLS, m_partls_in);
          pool_release_buffer(m_artls_down_pool, m_partls_in);
        }
        else
          war("no more artls_in");


        // Init frames and start protocol
        // These possibly needs to be set.
        start_param_t startTime;
        startTime.mega_frame = 0;
        startTime.hyper_frame = 0;
        startTime.super_frame = 0;
        if(ioctl_w(m_pollfds.fd, PP_IOC_START_PROTOCOL, &startTime) < 0)
          throw RestartNeeded("Unable to start protocol. ", 5.0);

        // TODO:
        // Set Bind Coord. IGNORE FOR NOW AS DEFAULT 0.
        // cmd_type = ARTLS_SET_PREFERED_REF_BASE_DOWN;
        // PP_IOC_S_ARTLS

        // TODO:
        // Set threshold coeffs. IGNORE FOR NOW AS DEFAULT 0
        // cmd_type = ARTLS_SET_THRESHOLD_COEFF_DOWN
        // PP_IOC_S_ARTLS

        // TODO:
        // Set Antenna offset
        uint16_t antenna_offset = (int16_t)(m_options.antenna_offset * 3333); // Convert m to ps
        if(antenna_offset != 0)
        {
          m_partls_in = (artls_down_t*)pool_get_buffer(m_artls_down_pool);
          if(m_partls_in)
          {
            int i;
            m_partls_in->cmd_type = ARTLS_SET_ANTENNA_OFFSET_DOWN;
            m_partls_in->cmd_tracking = AT_NO_TRACKING;
            m_partls_in->user_data = 0;
            m_partls_in->sf_start = 0;
            m_partls_in->sf_period = 0;
            for(i = 0; i < RFPATH_NB; i++)
              m_partls_in->payload.antenna_offset[i] = antenna_offset;
            ioctl_w(m_pollfds.fd, PP_IOC_S_ARTLS, m_partls_in);
            pool_release_buffer(m_artls_down_pool, m_partls_in);
          }
          else
            war("no more artls_in");
        }

        inf("UM100 Initialized successfully. ");
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_pollfds.fd >= 0)
        {
          if(ioctl_w(m_pollfds.fd,PP_IOC_STOP_PROTOCOL,NULL) < 0)
            war("Unable to stop protocol. ");
          if(ioctl_w(m_pollfds.fd,PP_IOC_CLOSE,NULL) < 0)
            war("Unable to close connection. ");

          close(m_pollfds.fd);
        }
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
            loc_toa_t* ltoa=NULL;
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
                     &ltoa,
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
            inf("I think I got some data!");
            if (rng != NULL)
              inf("src_hash: %d", rng->src_hash);

            data +=entrySize;
            if((b == NULL)&&(b1 == NULL)&&(b2 == NULL)&&(rng == NULL)&&(rdv == NULL))
              continue;//redundant with previous test?

            // Calculate data
            double dist = calculate_distance(m_trame_buff,curEntry,rng, ltoa);
            inf("Distance: %f", dist);
          }
        }
      }

#define C_VELOCITY    299792458.0f  // in m/s
      static double
      calculate_distance(
          rng_protocol_header_t* header,
          rng_protocol_entry_t * data,
          rng_protocol_entry_basic_t* rng,
          loc_toa_t* ltoa
          )
      {
        bool with_id = true;
        const char* sep;

        double d = 0;
        OSAL_s32 s32toa_le=0;
        int precision = 2;
        char* units = "m";


        if(ltoa == NULL) return 0;//not configured to sho this
        s32toa_le=(OSAL_s32) SWITCH_ENDIANESS_32BITS(ltoa->toa_le);
        d = s32toa_le*1e-12*C_VELOCITY;

        if(rng)
        {
          // Divide ranging by 2 if we are ref_base
          if(header->receiver_hash == rng->dst_hash)
            d /= 2;
        }
        return d;
      }

      void
      handleARTLS()
      {
#ifndef __x86_64__
        int status = ioctl_w(m_pollfds.fd, PP_IOC_G_ARTLS, &m_artls_out);
        if(status >= 0)
        {

          if(m_artls_out.notif_type == ARTLS_CMD_STATUS_UP)
          {
            m_partls_in = (artls_down_t*)m_artls_out.payload.cmd_status.user_data;

            if(m_partls_in->cmd_type == ARTLS_SIMPLE_PAIRING_REPLY_DOWN)
            {
              simple_pairing_request_finished(&m_partls_in->payload.spr, m_artls_out.payload.cmd_status.status);
            }
            pool_release_buffer(m_artls_down_pool, m_partls_in);
          }

          if(m_artls_out.notif_type == ARTLS_PAIRING_REQUEST_UP)
          {
            m_partls_in = (artls_down_t*)pool_get_buffer(m_artls_down_pool);
            if(m_partls_in)
            {
              m_partls_in->cmd_type = ARTLS_SIMPLE_PAIRING_REPLY_DOWN;
              m_partls_in->cmd_tracking = AT_NOTIFY_END;
              m_partls_in->sf_start = 0;
              m_partls_in->sf_period = 0;

              m_partls_in->user_data = (OSAL_u32)m_partls_in; // Keep track of the command

              // NB: REMOVED PAIRING_SEND;
              if(simple_pairing_request(&m_artls_out.payload.pairing_req, &m_partls_in->payload.spr) == OSAL_OK)
                ioctl_w(m_pollfds.fd, PP_IOC_S_ARTLS, m_partls_in);
              else
                pool_release_buffer(m_artls_down_pool, m_partls_in);
            }
            else
              war("no more artls_in");
          }

          if(m_artls_out.notif_type == ARTLS_DISSOCIATION_REQUEST_UP)
          {
            inf("Got request to stop. Ignoring.");
          }
        }
#else
          war("Not implemented on 64bit.");
#endif
      }

      //! Main loop.
      void
      onMain(void)
      {
        int timeout_ms = 500;
        time_t curr_time, last_time; // Last time we call device_tiemout_tick() (should be less than 1 second ago)

        // Initialize timer
        if(time(&last_time) == (time_t)(-1))
          err("Can't get clock time: %s\n", strerror(errno));

        m_pollfds.events = POLLIN|POLLPRI|POLLHUP;
        while (!stopping())
        {
          consumeMessages();

          if(poll(&m_pollfds, 1, timeout_ms) < 0)
            err("poll error: %s\n", strerror(errno));

          // Time management
          time(&curr_time);
          if(last_time > curr_time)
            war("Clock going backward of %ld sec\n", last_time - curr_time);
          else if( (curr_time - last_time) >= ARTLS_TICK_DURATION)
          {
            device_timeout_tick(curr_time - last_time);
            last_time = curr_time;
          }

          // Check events.
          if(m_pollfds.revents & POLLIN)
          {
            read_parse_data();
          }
          if(m_pollfds.revents & POLLPRI)
          {
            handleARTLS();

          }
          // Driver hangup, close,
          if(m_pollfds.revents & (POLLHUP|POLLERR))
          {
            // TODO: Handle (if needed?)
            err("Got request to hang up. Ignoring. ");
          }
        }
      }

      bool
      isMoving()
      {
        if (m_role == COORDINATOR)
          return true;
        else
          return false;
      }

      uint32_t
      getCapabilities(void)
      {
        switch(m_role)
        {
          case TAG:
            return DEVCAPS_AGILE|DEVCAPS_UWB_SLAVE;
          case BASE:
            return 0;
          case COORDINATOR:
            return DEVCAPS_UWB_MASTER|DEVCAPS_ROOT|DEVCAPS_AGILE;
          default:
            return 0;
        }
      }


      static void set_superframe_schema(uint32_t caps, superframe_info_t *sfi)
      {
        if(!sfi)
          return;

        sfi->phs = DEFAULT_SFI_PHS;
        sfi->psn = DEFAULT_SFI_PSN;
        sfi->pss = DEFAULT_SFI_PSS;
        sfi->beacon_distance = DEFAULT_SFI_SLOT_B2 - DEFAULT_SFI_SLOT_B1;

        if(caps & DEVCAPS_UWB_SLAVE)
        {
          memcpy(sfi->frames_desc, Tag_Slot_Desc, nTag_Slot_Desc*sizeof(frame_info_t));
          sfi->nb_frames = nTag_Slot_Desc;
        }
        else if(caps & DEVCAPS_ROOT)
        {
          memcpy(sfi->frames_desc, Coord_Slot_Desc, nCoord_Slot_Desc*sizeof(frame_info_t));
          sfi->nb_frames = nCoord_Slot_Desc;
        }
        else
        {
          memcpy(sfi->frames_desc, Base_Slot_Desc, nBase_Slot_Desc*sizeof(frame_info_t));
          sfi->nb_frames = nBase_Slot_Desc;
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

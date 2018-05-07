/*
 * UM100Hal.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: krisklau
 */

#include <Sensors/UM100SPI/UM100Hal.hpp>




#define C_VELOCITY    299792458.0f  // in m/s

namespace Sensors
{
  namespace UM100SPI
  {

    UM100Hal::UM100Hal(Task* task, std::string device, Role role, UM100_options options):
      m_task(task),
      m_device(device),
      m_role(role),
      m_options(options),
      m_maxdev_nb(32)
    {
        m_task->inf("HAL constructed.");

        // TODO: Move to resource acq/release. (Release with null check)
        m_trame_buff = (rng_protocol_header_t*) malloc(c_trame_buff_max_size);

        m_partls_in = NULL;

        // Initialize pool of ARTLS commands
        m_artls_down_pool = allocate_pool(sizeof(artls_down_t), 50);

        // Init timers
        time(&curr_time);
        time(&last_time);
        m_time_first_message = -1;
        m_time_previous_message = -1;
    }

    UM100Hal::~UM100Hal()
    {
      free(m_trame_buff);
      release_pool(m_artls_down_pool);
    }

    // Uses available data to calculate distance
    static double
    calculate_distance(
        rng_protocol_header_t* header,
        rng_protocol_entry_t * data,
        rng_protocol_entry_basic_t* rng,
        loc_toa_t* ltoa
        )
    {
      (void) data;
      double d = 0;
      OSAL_s32 s32toa_le=0;

      if(ltoa == NULL)
        return 0;//not configured to show this

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

    // Use data to calculate toa
    static unsigned int
    calculate_toa(
        rng_protocol_header_t* header,
        rng_protocol_entry_t * data,
        loc_toa_t* ltoa)
    {
      (void) header;
      (void) data;
      return SWITCH_ENDIANESS_32BITS(ltoa->toa_le);
    }

    // Calculate link quality
    static unsigned int
    calculate_link_quality(
        rng_protocol_header_t* header,
        rng_protocol_entry_t * data)
    {
      (void) header;
      return data->link_quality;
    }


    // Returns a default superframe schema.
    static void
    set_superframe_schema(uint32_t caps, superframe_info_t *sfi)
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



    bool
    UM100Hal::initialize(void)
    {

      m_task->inf("Trying to open: %s", m_device.c_str());
      m_pollfds.fd = open(m_device.c_str(), O_RDWR);

      if (m_pollfds.fd < 0)
      {
        throw RestartNeeded("Unable to open device. ", 5.0);
      }

      // Initialize timer
      if(time(&last_time) == (time_t)(-1))
        m_task->err("Can't get clock time: %s\n", strerror(errno));

      // Set poll events.
      m_pollfds.events = POLLIN|POLLPRI|POLLHUP;

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
          m_task->war("assume --no_superframe_info\n");
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
      m_task->war("For now, not setting moving bit. ");


      // Init depayload: ranging_data_depayload_init
      if(ranging_data_depayload_init(m_options.extDataLenInBits)!=0)
        m_task->err("ranging_data_depayload_init error");

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
        m_task->war("no more artls_in");


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
      uint16_t antenna_offset = (int16_t)(m_options.antenna_offset * 3333.0); // Convert m to ps
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
          m_task->war("no more artls_in");
      }

      m_task->inf("UM100 Initialized successfully. ");
      return true;
    }

    bool
    UM100Hal::close(void)
    {
      if (m_pollfds.fd >= 0)
      {
        if(ioctl_w(m_pollfds.fd,PP_IOC_STOP_PROTOCOL,NULL) < 0)
          m_task->war("Unable to stop protocol. ");
        if(ioctl_w(m_pollfds.fd,PP_IOC_CLOSE,NULL) < 0)
          m_task->war("Unable to close connection. ");

        ::close(m_pollfds.fd);
      }
      return true;
    }

    PollResult
    UM100Hal::poll(double timeout_ms)
    {
      int res = ::poll(&m_pollfds, 1, timeout_ms);
      if(res < 0)
      {
        m_task->err("poll error: %s\n", strerror(errno));
        return P_ERROR;
      }
      if (res > 0)
        return P_EVENT;

      return P_TIMEOUT;
    }
    unsigned int
    UM100Hal::parse_events()
    {
      // Time management
      time(&curr_time);
      if(last_time > curr_time)
        m_task->war("Clock going backward of %ld sec\n", last_time - curr_time);
      else if( (curr_time - last_time) >= ARTLS_TICK_DURATION)
      {
        device_timeout_tick(curr_time - last_time);
        last_time = curr_time;
      }

      unsigned int ret = EVENT_NONE;

      // Check events.
      if(m_pollfds.revents & POLLIN)
      {
        ret |= DATA_READY;
      }
      if(m_pollfds.revents & POLLPRI)
      {
        ret |= NETWORK_MANAGEMENT;

      }
      // Driver hangup, close,
      if(m_pollfds.revents & (POLLHUP|POLLERR))
      {
        ret |= HANGUP;
      }
      return ret;
    }

    uint32_t
    UM100Hal::getCapabilities(void)
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

    bool
    UM100Hal::isMoving()
    {
      if (m_role == COORDINATOR)
        return true;
      else
        return false;
    }



    bool
    UM100Hal::read_single_from_buffer(rng_protocol_header_t* header, unsigned char* buffer, OSAL_u16* entrySize, Measurement& m)
    {
      type_rdv_t rdvtype = (type_rdv_t) 0;
      type_trame_t ttype =TT_UNKNOWN;
      rng_protocol_entry_t* curEntry = (rng_protocol_entry_t*) buffer;
      rng_protocol_entry_basic_t* rng=NULL;
      beacon_protocol_entry_basic_t* b=NULL;
      beacon1_protocol_entry_basic_t* b1=NULL;
      beacon2_protocol_entry_basic_t* b2=NULL;
      rdv_protocol_entry_basic_t* rdv=NULL;
      loc_toa_t* ltoa=NULL;

      if(curEntry==NULL)
      {
        m_task->war("SOMETHING STRANGE.CHECK CODE\n");
        return false;
      }
      if(ranging_data_depayload_getLocData(curEntry,
               OSAL_false,\
               &ttype,
               &rdvtype,
               entrySize,
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
        m_task->err("ranging_data_depayload_getLocData failure!!\n");
        return false;
      }

      if((b == NULL)&&(b1 == NULL)&&(b2 == NULL)&&(rng == NULL)&&(rdv == NULL))
        return false;

      if (rng == NULL || ltoa == NULL)
        return false;

      // Calculate data
      m.src = SWITCH_ENDIANESS_32BITS(rng->src_hash);
      m.dst = SWITCH_ENDIANESS_32BITS(rng->dst_hash);
      m.distance = calculate_distance(header,curEntry,rng, ltoa);
      m.time = calculate_toa(header, curEntry, ltoa);
      m.quality_factor = calculate_link_quality(header, curEntry);
      m.delta_time = -1;

      m_task->spew("Src: %x, Dst: %x, Distance: %.3fm, Link Quality: %d",
                   m.src,
                   m.dst,
                   m.distance,
                   m.quality_factor);

      return true;
    }

    std::vector<Measurement>
    UM100Hal::read_measurements()
    {
      std::vector<Measurement> result;

      double now = Clock::get();

      size_t cur_size = read( m_pollfds.fd,
        m_trame_buff,
        c_trame_buff_max_size);

      if(cur_size > 0)
      {


        unsigned char* data=(unsigned char*) m_trame_buff;
        OSAL_u16 entrySize=0;
        data+=sizeof(rng_protocol_header_t);
        for(int rng_idx=0; rng_idx < m_trame_buff->nb_entry; rng_idx++)
        {
          Measurement m;
          bool valid_data = read_single_from_buffer(m_trame_buff, data, &entrySize, m);

          if (valid_data)
          {
            if (m_time_first_message < 0)
              m_time_first_message = now;

            double diff = 0;
            if (m_time_previous_message > 0)
              diff = now - m_time_previous_message;

            //m.time = now - m_time_first_message;
            m.delta_time = diff;

            result.push_back(m);
          }

          // Increment data pointer
          data +=entrySize;
        }
      }
      else
      {
        m_task->err("Failed to read, got size: %zu", cur_size);
      }

      if (result.size() > 1)
        m_time_previous_message = now;

      return result;
    }



    int UM100Hal::_ioctl_w(int fd, int request, void* data, const char* ioctl_name)
    {
      int status = ioctl(fd, request, data);
      if(status < 0)
        m_task->war("ioctl(%s) failed with ret %d: %s\n", ioctl_name, status, strerror(errno));
      return status;
    }



    void
    UM100Hal::handle_network()
    {
// To avoid BeSpoon SDK hardcode of 32bit pointer warnings.
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
            m_task->war("no more artls_in");
        }

        if(m_artls_out.notif_type == ARTLS_DISSOCIATION_REQUEST_UP)
        {
          m_task->inf("Got request to stop. Ignoring.");
        }
      }
#else
        m_task->war("Not implemented on 64bit.");
#endif
    }

  } /* namespace UM100SPI */
} /* namespace Sensors */

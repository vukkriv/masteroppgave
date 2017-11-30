/*
 * UM100Hal.hpp
 *
 *  Created on: Nov 30, 2017
 *      Author: krisklau
 */

#ifndef USER_SRC_SENSORS_UM100SPI_UM100HAL_HPP_
#define USER_SRC_SENSORS_UM100SPI_UM100HAL_HPP_

// DUNE headers
#include <DUNE/DUNE.hpp>

// stdlib headers
#include <vector>

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



namespace Sensors
{
  namespace UM100SPI
  {
    using DUNE_NAMESPACES;

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

    enum PollResult
    {
      P_ERROR = -1,
      P_TIMEOUT = 0,
      P_EVENT = 1
    };

    enum EventType
    {
      EVENT_NONE          = 0x0000,
      DATA_READY          = 0x0001,
      NETWORK_MANAGEMENT  = 0x0002,
      HANGUP              = 0x0004
    };

    struct Measurement
    {
      // Distance (in meters)
      double distance;
      // Signal strength
      double RSSI;
      // Source of transmission (ID)
      int src;
      // Destination of transmision (ID)
      int dst;
      // Quality factor of the range measurement, as set by UM100
      int quality_factor;
      // Timestamp
      double time;
      // Relative time.
      double delta_time;
    };

    static const size_t c_trame_buff_max_size = sizeof(rng_protocol_header_t)
                                                + sizeof(rng_protocol_entry_t)*MAX_FRAME_PER_SUPERFRAME;

    class UM100Hal
    {
    public:
      UM100Hal(Task* task, std::string device, Role role, UM100_options options):
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
      }

      ~UM100Hal()
      {
        free(m_trame_buff);
        release_pool(m_artls_down_pool);
      }

      // Initialize the UM100
      bool initialize();

      // Closes the connection to the driver and releases the file handle.
      bool close();

      // Polls the file handle. Returns PollResult based on which event triggered.
      // (Event/timeout/error)
      PollResult poll(double timeout_ms);

      // Returns a mask of events
      unsigned int parse_events();

      // Handles network tasks. Should be called after an NETWORK_MANAGEMENT event.
      void handle_network(void);

      // Returns new available data. Should be called after an DATA_READY event.
      std::vector<Measurement> read_measurements();


    private:
      uint32_t getCapabilities(void);
      int _ioctl_w(int fd, int request, void* data, const char* ioctl_name);
      bool isMoving();
      bool read_single_from_buffer(rng_protocol_header_t* header, unsigned char* buffer, OSAL_u16* read_size, Measurement& m);


      // Task
      Task* m_task;
      // Device
      std::string m_device;
      // Role
      Role m_role;
      // Options
      UM100_options m_options;
      // File descriptor
      struct pollfd m_pollfds;
      // Input buffer
      rng_protocol_header_t* m_trame_buff;
      // UM100 States
      // Timing info
      time_t curr_time, last_time;
      // Last response from kernel driver
      artls_up_t m_artls_out;
      // Pointer to data to send down
      artls_down_t* m_partls_in;
      // Pool to keep track of kernel driver input requests.
      pool_t* m_artls_down_pool;
      // Number of devices to use
      OSAL_u32 m_maxdev_nb;
    };

  } /* namespace UM100SPI */
} /* namespace Sensors */

#endif /* USER_SRC_SENSORS_UM100SPI_UM100HAL_HPP_ */

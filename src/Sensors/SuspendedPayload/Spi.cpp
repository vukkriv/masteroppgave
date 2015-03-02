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
// Author: Kristian Klausen                                                 *
//***************************************************************************

// Local Headers
#include "Spi.hpp"



// ISO C++ 98 headers.
#include <stdexcept>
#include <cstdlib>

// DUNE headers.
#include <DUNE/Exceptions.hpp>

// POSIX headers.
#if defined(DUNE_SYS_HAS_SYS_TYPES_H)
#  include <sys/types.h>
#endif

#if defined(DUNE_SYS_HAS_SYS_STAT_H)
#  include <sys/stat.h>
#endif

#include <stdint.h>

#if defined(DUNE_SYS_HAS_SYS_IOCTL_H)
#  include <sys/ioctl.h>
#endif

#if defined(DUNE_SYS_HAS_FCNTL_H)
#  include <fcntl.h>
#endif

#if defined(DUNE_SYS_HAS_UNISTD_H)
#  include <unistd.h>
#endif

// Linux headers.
#if defined(DUNE_SYS_HAS_LINUX_SPI_SPIDEV_H)
#  include <linux/spi/spidev.h>
#  define DUNE_SYS_HAS_LINUX_SPI_DEV 1
#endif




namespace Sensors
{
  namespace SuspendedPayload
  {



    Spi::Spi(const std::string& spi_dev)
    {
#if defined(DUNE_SYS_HAS_LINUX_SPI_DEV)
      if ((m_fd = open(spi_dev.c_str(), O_RDWR)) == -1)
        throw Error("opening device", DUNE::System::Error::getLastMessage());
#else
      //(void)spi_dev;
      throw DUNE::NotImplemented("SPI");
#endif
    }

    Spi::~Spi()
    {
#if defined(DUNE_SYS_HAS_LINUX_SPI_DEV)
      close(m_fd);
#endif
    }


    int
    Spi::transfer(SpiFrame* frames, unsigned int n)
    {

#if defined(DUNE_SYS_HAS_LINUX_SPI_DEV)
      if (n == 0 || n > c_max_frames)
      {
        throw Error("Exceeding maximum frames: ", "");
      }
      // Alternatively, have a member variable allocated in class constructor
      struct spi_ioc_transfer* xfer = (spi_ioc_transfer*) std::malloc(n * sizeof( spi_ioc_transfer ));
      memset(xfer, 0, n * sizeof( spi_ioc_transfer ));

      for (unsigned int k = 0; k < n; ++k)
      {

        xfer[k].rx_buf =  (unsigned long) frames[k].rx_buf;
        xfer[k].tx_buf =  (unsigned long) frames[k].tx_buf;

        xfer[k].len    = frames[k].len;
        xfer[k].delay_usecs = frames[k].delay_us;
      }

      // As we cannot have variable n, do it like this:
      int status;
      if (n == 1)
        status = ioctl(m_fd, SPI_IOC_MESSAGE(1), xfer);
      else if (n == 2)
        status = ioctl(m_fd, SPI_IOC_MESSAGE(2), xfer);
      else if (n == 3)
        status = ioctl(m_fd, SPI_IOC_MESSAGE(3), xfer);
      else
        throw Error("Invalid number of frames.", "");

      if (status < 0)
      {
        throw Error("SPI transfer error", strerror(errno));
      }


      free(xfer);

      return status;
#else
      (void)frames;
      (void)n;
#endif
    }

    void
    Spi::setSpeed(uint32_t speed_hz)
    {
#if defined(DUNE_SYS_HAS_LINUX_SPI_DEV)
      ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
#else
      (void) speed_hz;
#endif
    }

    void
    Spi::setMode(SPI_MODE mode)
    {
#if defined(DUNE_SYS_HAS_LINUX_SPI_DEV)
      uint32_t newmode = 0;
      switch(mode)
      {
        default:
          newmode = SPI_MODE_0;
          break;
        case MODE_0:
          newmode = SPI_MODE_0;
          break;
        case MODE_1:
          newmode = SPI_MODE_1;
          break;
        case MODE_2:
          newmode = SPI_MODE_2;
          break;
        case MODE_3:
          newmode = SPI_MODE_3;
          break;
      }
     ioctl(m_fd, SPI_IOC_WR_MODE, &newmode);
#else
     (void)mode;
#endif
    }

  } /* namespace MTS360 */
} /* namespace Sensors */

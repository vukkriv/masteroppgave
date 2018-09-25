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

// C Headers
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

// ISO C++ 98 headers.
#include <cerrno>
#include <iostream>

// DUNE headers.
#include <DUNE/System/Error.hpp>
#include <DUNE/Streams/Terminal.hpp>
#include <DUNE/Utils/String.hpp>
#include <DUNE/Hardware/GPIO.hpp>
#include <DUNE/Time/Utils.hpp>
#include <USER/Hardware/IrqGPIO.hpp>



namespace DUNE
{
  namespace Hardware
  {
    using System::Error;
    using Utils::String;


    IrqGPIO::IrqGPIO(unsigned int number, Edge edge):
      m_gpio(number),
      m_number(number),
      m_edge(edge)
    {
#if defined(DUNE_OS_LINUX)

      // Setup paths
      std::string prefix = std::string("/sys/class/gpio/gpio") +
                           String::str(m_number);
      m_file_val = prefix + std::string("/value");
      m_file_edge = prefix + std::string("/edge");


      // Setup GPIO
      m_gpio.setDirection(GPIO::GPIO_DIR_INPUT);

      // Set edge
      setEdge(m_edge);

      // Open value for polling
      m_handle = open(m_file_val.c_str(), O_RDONLY | O_NONBLOCK);
      if (m_handle == -1)
        throw Error(errno, "unable to open gpio value file", m_number);


      // Lacking implementation.
#else
      throw Error("unimplemented feature", "DUNE::Hardware::IrqGPIO");
#endif
    }

    IrqGPIO::~IrqGPIO()
    {
#if defined(DUNE_OS_LINUX)
      close(m_handle);
#endif
    }

    void
    IrqGPIO::setEdge(Edge edge)
    {
#if defined(DUNE_OS_LINUX)
      if (edge == GPIO_EDGE_FALLING)
        writeToFile(m_file_edge, "falling");
      else
        writeToFile(m_file_edge, "rising");
#endif
    }

    bool
    IrqGPIO::poll(double timeout)
    {
#if defined(DUNE_OS_POSIX)

      int rv = 0;

      pollfd fdlist[1];
      fdlist[0].fd = m_handle;
      fdlist[0].events = POLLPRI;

      // TODO: Consider using timeval/timespec
      if (timeout < 0.0)
        rv = ::poll(fdlist, 1, -1 );
      else
        rv = ::poll(fdlist, 1, (unsigned int) (timeout * 1000.0));

      // Read to clear interrupt
      if (rv > 0)
      {
        char buf[3];

        if (read(m_handle, buf, 2) == -1)
        {
          throw Error("reading handle", Error::getLastMessage());
        }

        // Set seek
        if (lseek(m_handle, 0, SEEK_SET) == -1)
        {
          throw Error("set seek of handle", Error::getLastMessage());
          return false;
        }
      }

      if (rv == -1)
      {
        //! Workaround for when we are interrupted by a signal.
        if (errno == EINTR)
          return false;
        else
          throw Error("polling handle", Error::getLastMessage());
      }

      return rv > 0;
#endif
    }


#if defined(DUNE_OS_LINUX)
    void
    IrqGPIO::writeToFile(const std::string& file, int value)
    {
      writeToFile(file, String::str(value));
    }

    void
    IrqGPIO::writeToFile(const std::string& file, const std::string& value)
    {
      std::FILE* fd = std::fopen(file.c_str(), "w");
      if (fd == 0)
        throw Error(errno, "unable to export GPIO", value);
      std::fputs(value.c_str(), fd);
      std::fclose(fd);
    }

#endif

  }
}

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

// C Headers
#include <stdio.h>
#include <fcntl.h>

// ISO C++ 98 headers.
#include <cerrno>

// DUNE headers.
#include <DUNE/System/Error.hpp>
#include <DUNE/Streams/Terminal.hpp>
#include <DUNE/Utils/String.hpp>
#include <DUNE/Hardware/GPIO.hpp>
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

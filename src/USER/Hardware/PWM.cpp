
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
// Author: Kristoffer Gryte                                                 *
//***************************************************************************

// ISO C++ 98 headers.
#include <cerrno>
#include <sstream>
#include <iomanip>

// DUNE headers.
#include <DUNE/System/Error.hpp>
#include <DUNE/Streams/Terminal.hpp>
#include <DUNE/Utils/String.hpp>
#include <USER/Hardware/PWM.hpp>

namespace DUNE
{
  namespace Hardware
  {
    using DUNE::System::Error;
    using DUNE::Utils::String;

    PWM::PWM(unsigned int number, std::string chip_path):
      m_number(number),
      m_chip_path(chip_path)
    {
      // Linux 2.6 implementation.
#if defined(DUNE_OS_LINUX)
      writeToFile(m_chip_path + "export", m_number);

      std::string prefix = m_chip_path + 
                           std::string("pwm") +
                           String::str(m_number);
      m_file_duty = prefix + std::string("/duty_cycle");
      m_file_period = prefix + std::string("/period");
      m_file_enable = prefix + std::string("/enable");

      enable();
      // Lacking implementation.
#else
      throw Error("unimplemented feature", "DUNE::Hardware::PWM");
#endif
    }

    PWM::~PWM(void)
    {
      // Linux 2.6 implementation.
#if defined(DUNE_OS_LINUX)
      try
      {
        writeToFile(m_chip_path + "unexport", m_number);
      }
      catch (std::exception& e)
      {
        DUNE_ERR("DUNE::Hardware::PWM", e.what());
      }
#endif
    }

    void
    PWM::setPeriod(int period)
    {
      //! TODO: check that value is reasonable?
#if defined(DUNE_OS_LINUX)
        writeToFile(m_file_period, period);
#endif
      m_period = period;
    }

    void
    PWM::setValue(int value)
    {
      //! TODO: check that value makes sense, i.e. is smaller than period
#if defined(DUNE_OS_LINUX)
      writeToFile(m_file_duty, value);
#else
      throw Error("unimplemented feature", "DUNE::Hardware::PWM");
#endif
      m_duty_cycle = value;
    }

    void
    PWM::enable(void)
    {
#if defined(DUNE_OS_LINUX)
      writeToFile(m_file_enable, 1);
#else
      throw Error("unimplemented feature", "DUNE::Hardware::PWM");
#endif
    }

    void
    PWM::disable(void)
    {
#if defined(DUNE_OS_LINUX)
      writeToFile(m_file_enable, 0);
#else
      throw Error("unimplemented feature", "DUNE::Hardware::PWM");
#endif
    }

#if defined(DUNE_OS_LINUX)
    void
    PWM::writeToFile(const std::string& file, int value)
    {
      writeToFile(file, String::str(value));
    }

    void
    PWM::writeToFile(const std::string& file, const std::string& value)
    {
      std::FILE* fd = std::fopen(file.c_str(), "w");
      if (fd == 0)
        throw Error(errno, "unable to export PWM " + file, value);
      std::fputs(value.c_str(), fd);
      std::fclose(fd);
    }

#endif
  }
}

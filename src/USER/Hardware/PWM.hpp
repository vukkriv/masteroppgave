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

#ifndef DUNE_HARDWARE_PWM_HPP_INCLUDED_
#define DUNE_HARDWARE_PWM_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>

// DUNE headers.
#include <DUNE/Config.hpp>

namespace DUNE
{
  namespace Hardware
  {
    // Export symbol.
    class DUNE_DLL_SYM PWM;

    class PWM
    {
    public:
      
      PWM(void)
      {
        PWM(0);
      }

      //! Initialize PWM.
      //! @param[in] number PWM number.
      PWM(unsigned int number)
      {
        // number should be 0 or 1
        PWM(number, std::string("/sys/class/pwm/pwmchip0/"));
        //PWM(number, std::string("/sys/class/pwm/pwmchip2"));
        //PWM(number, std::string("/sys/class/pwm/pwmchip4"));
      }

      //! Initialize PWM.
      //! @param[in] number PWM number.
      //! @param[in] PWM chip path
      PWM(unsigned int number, std::string chip_path);

      //! Default destructor.
      ~PWM(void);

      //! Set PWM duty cycle.
      //! @param[in] duty cycle in nano seconds
      void
      setValue(int value);

      //! Set PWM period
      //! @param[in] period in nano seconds
      void
      setPeriod(int period);

      void
      enable(void);

      void
      disable(void);
      ////! Get PWM value.
      ////! @return pwm duty cycle
      //bool
      //getValue(void);

    private:
      //! PWM number.
      unsigned int m_number;
      //! i.e. /sys/class/pwm/pwmchip0
      std::string m_chip_path;

      double m_period;
      double m_duty_cycle;
#if defined(DUNE_OS_LINUX)
      //! Path to PWM duty cycle (ns) file.
      std::string m_file_duty;
      //! Path to PWM period_ns file
      std::string m_file_period;
      //! Path to PWM enable file
      std::string m_file_enable;

      static void
      writeToFile(const std::string& file, int value);

      static void
      writeToFile(const std::string& file, const std::string& value);
#endif
    };
  }
}

#endif

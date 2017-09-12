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

#ifndef DUNE_HARDWARE_IRQGPIO_HPP_INCLUDED_
#define DUNE_HARDWARE_IRQGPIO_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>

// DUNE headers.
#include <DUNE/Config.hpp>

namespace DUNE
{
  namespace Hardware
  {
    // Export symbol.
    class DUNE_DLL_SYM IRQGPIO;

    class IrqGPIO
    {
    public:
      enum Edge {
        //! Trigger interrupt on rising edge
        GPIO_EDGE_RISING,
        //! Trigger interrupt on falling edge (default)
        GPIO_EDGE_FALLING
      };

      //! Initialize IrqGPIO.
      //! @param[in] number GPIO number.
      //! @param[in] Edge to trigger interrupt
      IrqGPIO(unsigned int number, Edge edge = GPIO_EDGE_FALLING);

      //! Get GPIO value.
      //! @return pin value (false = off, true = on).
      bool
      getValue(void) { return m_gpio.getValue(); };

    private:
      //! GPIO Handle
      Hardware::GPIO m_gpio;
      //! GPIO number.
      unsigned int m_number;
      //! GPIO direction.
      Edge m_edge;

      //! Set edge
      void
      setEdge(Edge edge);


#if defined(DUNE_OS_LINUX)
      //! Path to GPIO edge file.
      std::string m_file_edge;
      //! Path to GPIO value file.
      std::string m_file_val;

      static void
      writeToFile(const std::string& file, int value);

      static void
      writeToFile(const std::string& file, const std::string& value);
#endif
    };

  }
}


#endif

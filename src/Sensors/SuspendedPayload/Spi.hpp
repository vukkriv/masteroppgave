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

#ifndef SENSORS_MTS360_SPI_HPP_INCLUDED_
#define SENSORS_MTS360_SPI_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <stdexcept>

// DUNE headers.
#include <DUNE/Config.hpp>
#include <DUNE/Utils/String.hpp>
#include <DUNE/System/Error.hpp>


namespace Sensors
{
  namespace SuspendedPayload
  {

    enum SPI_MODE
    {
      MODE_0 = 0, // CPOL = 0, CPHA  = 0
      MODE_1,     // CPOL = 0, CPHA  = 1
      MODE_2,     // CPOL = 1, CPHA  = 0
      MODE_3     // CPOL = 1, CPHA  = 1
    };

    struct SpiFrame {
      uint8_t* rx_buf;
      uint8_t* tx_buf;
      unsigned int len;
      unsigned int delay_us;
    };

    class Spi
    {
    public:
      class Error: public std::runtime_error
      {
      public:
        Error(std::string op, std::string msg):
          std::runtime_error("SPI error (" + op + "): " + msg)
        { }
      };

      //! Spi constructor
      Spi(const std::string& spi_dev);


      //! Spi destructor
      ~Spi(void);


      //! Do a SPI transfer
      //! txdata: Buffer containing tx data
      //! rxdata: Allocated buffer for return data. (may be the same as tx data)
      //! len: Length in bytes of transfer size.
      int
      transfer(const uint8_t* txdata, uint8_t* rxdata, uint8_t len);

      //! Do a two-framed transfer, with a optional delay between frames.
      int
      transfer(const uint8_t* f1_txdata, uint8_t* f1_rxdata, uint8_t f1_len, int delay_us, const uint8_t* f2_txdata, uint8_t* f2_rxdata, int f2_len);

      //! Transfer a number of frames
      int
      transfer(SpiFrame* frames, unsigned int n);

      //! Set speed
      void
      setSpeed(uint32_t speed_hz);

      //! Set mode
      void
      setMode(SPI_MODE mode);

    private:
      int m_fd;
      static const uint8_t c_max_frames = 3;



    };

  } /* namespace MTS360 */
} /* namespace Sensors */

#endif

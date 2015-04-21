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

#include "MTS360.hpp"

#define MTS360_INIT 0xAA
#define MTS360_READ 0xFF

namespace Sensors
{
  namespace SuspendedPayload
  {
    MTS360::MTS360(std::string device):
        m_device(device)
    {
      // Set speed and mode
      m_device.setSpeed(100000);
      m_device.setMode(MODE_1);

      // initialize buffers
      memset(m_tx_buf, 0, sizeof m_tx_buf);
      memset(m_rx_buf, 0, sizeof m_rx_buf);

      // Setup TX buffer
      // Two bytes initialization, two (four) bytes read
      m_tx_buf[0] = MTS360_INIT;
      m_tx_buf[1] = MTS360_READ;

      m_tx_buf[2] = MTS360_READ;
      m_tx_buf[3] = MTS360_READ;
      m_tx_buf[4] = MTS360_READ;
      m_tx_buf[5] = MTS360_READ;

      // Setup frames
      m_frames[0].rx_buf = m_rx_buf;
      m_frames[0].tx_buf = m_tx_buf;
      m_frames[0].len    = 2;
      m_frames[0].delay_us = 140;

      m_frames[1].rx_buf = &m_rx_buf[2];
      m_frames[1].tx_buf = &m_tx_buf[2];
      m_frames[1].len    = 4;
      m_frames[1].delay_us = 100;
    }

    float
    MTS360::read(void)
    {

      m_device.transfer(m_frames, 2);

      float angle = 0;
      uint16_t aMax = 0xFFFF - 0.1* 0xFFFF;
      uint16_t aMin = 0.1 * 0xFFFF;

      //printf("Got: %x%x - %x%x", m_rx_buf[2], m_rx_buf[3], m_rx_buf[4], m_rx_buf[5]);


      angle = (float)((m_rx_buf[2] << 8) | m_rx_buf[3]);

      angle -= aMin;

      angle = ( angle / (aMax - aMin) ) * 360.0;

      return angle;

    }

  } /* namespace MTS360 */
} /* namespace Sensors */

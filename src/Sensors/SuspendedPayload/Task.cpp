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

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Include Sensor headers
#include "MTS360.hpp"


namespace Sensors
{
  namespace SuspendedPayload
  {
    using DUNE_NAMESPACES;

    //! %Task arguments.
    struct Arguments
    {
      //! Device address
      std::string spi_device_1;
      std::string spi_device_2;

    };

    class Error: public std::runtime_error
    {
    public:
      Error(std::string op, std::string msg):
        std::runtime_error("SPI bus error (" + op + "): " + msg)
      { }
    };

    struct Task: public Tasks::Periodic
    {
      //! Arguments
      Arguments m_args;

      // Member variables
      int m_fd_1;
      int m_fd_2;

      MTS360* m_sensor_1;
      MTS360* m_sensor_2;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        Periodic(name, ctx),
        m_fd_1(0),
        m_fd_2(0),
        m_sensor_1(NULL),
        m_sensor_2(NULL)
      {

        param("SPI Device", m_args.spi_device_1)
        .defaultValue("/dev/spidev1.0")
        .description("Path to SPIdev device. ");

        param("SPI Device 2", m_args.spi_device_2)
        .defaultValue("/dev/spidev1.1")
        .description("Path to SPIdev device. ");

      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        // Todo: Add try/catch blocks
        m_sensor_1 = new MTS360(m_args.spi_device_1);
        m_sensor_2 = new MTS360(m_args.spi_device_2);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_sensor_1);
        Memory::clear(m_sensor_2);
      }

      //! Do reading
      void
      doRead(void)
      {

        float a1 = m_sensor_1->read() - 180.0;
        float a2 = m_sensor_2->read() - 180.0;

        debug("Angle: %f, %f \n", a1, a2);

        IMC::EulerAngles angles;

        angles.phi = -(a1 * Math::c_pi) / 180.0;
        angles.theta = (a2 * Math::c_pi) / 180.0 ;

        dispatch(angles);

      }

      //! Main loop.
      void
      task(void)
      {


        doRead();

      }
    };
  }
}

DUNE_TASK

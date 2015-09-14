//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Kristian Klausen                                                 *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Library headers
extern "C"
{
  #include <phidget21.h>
}



namespace Sensors
{
  namespace PhidgetBridge
  {
    using DUNE_NAMESPACES;



    //! %Task arguments.
    struct Arguments
    {
      //! Communications timeout
      uint8_t comm_timeout;
      //! Desired data-rate (ms)
      int data_rate_ms;
      //! Gain setting
      int gain;



    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Processing state of incoming sbp messages
      CPhidgetBridgeHandle m_bridge;
      //! Gain setting
      CPhidgetBridge_Gain m_gain;
      //! Sensor connected
      bool m_connected;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_gain(PHIDGET_BRIDGE_GAIN_1),
        m_connected(false)
      {


        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for base and local communication.");

        param("Data-rate", m_args.data_rate_ms)
        .defaultValue("10")
        .units(Units::Millisecond)
        .description("Sensor datarate in ms");

        param("Gain", m_args.gain)
        .defaultValue("1")
        .values("1,8,16,32,64,128")
        .description("Sensor Gain. ");


        // Init bridge interface
        CPhidgetBridge_create(&m_bridge);


        // Populate callback nodes.
        CPhidget_set_OnAttach_Handler((CPhidgetHandle)m_bridge, ph_attachHandler, (void*)this);
        CPhidget_set_OnDetach_Handler((CPhidgetHandle)m_bridge, ph_detachHandler, (void*)this);
        CPhidget_set_OnError_Handler((CPhidgetHandle)m_bridge, ph_errorHandler, (void*)this);

        CPhidgetBridge_set_OnBridgeData_Handler(m_bridge, ph_dataHandler, (void*)this);


      }





      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        switch (m_args.gain)
        {
          default:
            m_gain = PHIDGET_BRIDGE_GAIN_1;
            trace("Set gain to 1");
            break;
          case 1:
            m_gain = PHIDGET_BRIDGE_GAIN_1;
            trace("Set gain to %d", m_args.gain);
            break;
          case 8:
            m_gain = PHIDGET_BRIDGE_GAIN_8;
            trace("Set gain to %d", m_args.gain);
            break;
          case 16:
            m_gain = PHIDGET_BRIDGE_GAIN_16;
            trace("Set gain to %d", m_args.gain);
            break;
          case 32:
            m_gain = PHIDGET_BRIDGE_GAIN_32;
            trace("Set gain to %d", m_args.gain);
            break;
          case 64:
            m_gain = PHIDGET_BRIDGE_GAIN_64;
            trace("Set gain to %d", m_args.gain);
            break;
          case 128:
            m_gain = PHIDGET_BRIDGE_GAIN_128;
            trace("Set gain to %d", m_args.gain);
            break;
        }



      }



      void
      handleBridgeData(int index, double val)
      {
        trace("New handler data: (%d) %f", index, val);
      }

      void
      handleAttachment(CPhidgetBridgeHandle bridge)
      {

        m_connected = true;

        inf("Attached a phidget bridge.");

        CPhidgetBridge_setEnabled(bridge, 0, PTRUE);
        CPhidgetBridge_setEnabled(bridge, 1, PFALSE);
        CPhidgetBridge_setEnabled(bridge, 2, PFALSE);
        CPhidgetBridge_setEnabled(bridge, 3, PFALSE);

        CPhidgetBridge_setGain(bridge, 0, m_gain);
        CPhidgetBridge_setGain(bridge, 1, PHIDGET_BRIDGE_GAIN_1);
        CPhidgetBridge_setGain(bridge, 2, PHIDGET_BRIDGE_GAIN_1);
        CPhidgetBridge_setGain(bridge, 3, PHIDGET_BRIDGE_GAIN_1);
        CPhidgetBridge_setDataRate(bridge, m_args.data_rate_ms);
      }

      void
      handleDetachment()
      {
        inf("Detached.");
        m_connected = false;
      }

      void
      handleError(int errorCode, const char* errorString)
      {
        err("Phidget error (%d): %s", errorCode, errorString);
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



      }


      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        CPhidget_open((CPhidgetHandle)m_bridge, -1);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {

        if (m_connected)
        {
          CPhidget_close((CPhidgetHandle)m_bridge);
          CPhidget_delete((CPhidgetHandle)m_bridge);

          m_connected = false;
        }
      }



      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {

          // Wait for messages
          waitForMessages(1);
        }
      }

      // Forwards to task-handler
      static int
      ph_attachHandler(  CPhidgetHandle phid, void *userPtr)
      {
        CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
        Sensors::PhidgetBridge::Task* task = (Sensors::PhidgetBridge::Task*) (userPtr);

        task->handleAttachment(bridge);
        return 0;
      }

      static int
      ph_detachHandler(  CPhidgetHandle phid, void *userPtr)
      {

        CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
        (void) bridge;

        Sensors::PhidgetBridge::Task* task = (Sensors::PhidgetBridge::Task*) userPtr;
        task->handleDetachment();

        return 0;
      }

      static int
      ph_errorHandler(  CPhidgetHandle phid, void *userPtr, int errorCode, const char *errorString)
      {

        CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
        (void) bridge;

        Sensors::PhidgetBridge::Task* task = (Sensors::PhidgetBridge::Task*) userPtr;
        task->handleError(errorCode, errorString);

        return 0;
      }

      static int
      ph_dataHandler( CPhidgetBridgeHandle phid, void *userPtr, int index, double val)
      {

        CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
        (void)bridge;

        Sensors::PhidgetBridge::Task* task = (Sensors::PhidgetBridge::Task*) userPtr;
        task->handleBridgeData(index, val);

        return 0;
      }


    };
  }
}

DUNE_TASK

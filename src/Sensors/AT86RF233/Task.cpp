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
#include "common.h"
#include "edc.h"
#include "sbp.h"

#define SBP_PREAMBLE 0x55
namespace Sensors
{
  namespace AT86RF233
  {
    using DUNE_NAMESPACES;


    //! %Task arguments.
    struct Arguments
    {
      //! Communications timeout
      uint8_t comm_timeout;
      //! TCP Port - Local Sensor
      uint16_t TCP_port;
      //! TCP Address - Local Sensor
      Address TCP_addr;
    };


    typedef struct {
      u32 dist;      // distance in cm?
      u8 dqf;        // distance quality factor
      u16 reflector; //2 bytes address of reflector
      u16 sender;    //2 bytes address of sender
      u32 time;      // time in ns or us?
    } AT_Measurement;


    typedef struct {
         uint32_t dist;      // distance in cm?
         char dqf;        // distance quality factor
         uint16_t reflector; //2 bytes address of reflector
         uint16_t sender;    //2 bytes address of sender
         uint32_t time;      // time in ns or us?
       } AT_Measurement2;


    struct Task: public DUNE::Tasks::Task
    {

      //! Task arguments.
      Arguments m_args;
      //! Timestamp on prevoius local packet
      double m_last_pkt_time;
      uint8_t m_buf[512];
      //! TCP socket - Sensor
      TCPSocket* m_TCP_sock;
      Address m_TCP_addr;
      uint16_t m_TCP_port;
      //! Processing state of incomming sbp messages
      sbp_state_t m_sbp_state;


      bool m_error_missing_data;



      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_last_pkt_time(0),
        m_TCP_sock(NULL),
        m_TCP_port(0),
        m_error_missing_data(false)
      {

        param("TCP - Port", m_args.TCP_port)
        				    .defaultValue("9999")
        				    .description("Port for connection to Atmel device");

        param("TCP - Address", m_args.TCP_addr)
        .defaultValue("127.0.0.1")
        .description("Address for connection to Atmel device");


        param("Communication Timeout", m_args.comm_timeout)
        .defaultValue("5")
        .units(Units::Second)
        .description("Timeout for communication.");

        // Init piksi interface
        sbp_state_init(&m_sbp_state);

        // Register task as context
        sbp_state_set_io_context(&m_sbp_state, (void*) this);



      }

      ///////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////
      // START SBP
      /////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////

      /** Register a callback for a message type.
       * Register a callback that is called when a message
       * with type msg_type is received.
       *
       * \param msg_type Message type associated with callback
       * \param cb       Pointer to message callback function
       * \param context  Pointer to context for callback function
       * \param node     Statically allocated #sbp_msg_callbacks_node_t struct
       * \return `SBP_OK` (0) if successful, `SBP_CALLBACK_ERROR` if callback was
       *         already registered for that message type.
       */
      s8 sbp_register_callback(sbp_state_t *s, u16 msg_type, sbp_msg_callback_t cb, void *context,
          sbp_msg_callbacks_node_t *node)
      {
        /* Check our callback function pointer isn't NULL. */
        if (cb == 0)
          return SBP_NULL_ERROR;

        /* Check our callback node pointer isn't NULL. */
        if (node == 0)
          return SBP_NULL_ERROR;

        /* Check if callback was already registered for this type. */
        if (sbp_find_callback(s, msg_type) != 0)
          return SBP_CALLBACK_ERROR;

        /* Fill in our new sbp_msg_callback_node_t. */
        node->msg_type = msg_type;
        node->cb = cb;
        node->context = context;
        /* The next pointer is set to NULL, i.e. this
         * will be the new end of the linked list.
         */
        node->next = 0;

        /* If our linked list is empty then just
         * add the new node to the start.
         */
        if (s->sbp_msg_callbacks_head == 0) {
          s->sbp_msg_callbacks_head = node;
          return SBP_OK;
        }

        /* Find the tail of our linked list and
         * add our new node to the end.
         */
        sbp_msg_callbacks_node_t *p = s->sbp_msg_callbacks_head;
        while (p->next)
          p = p->next;

        p->next = node;

        return SBP_OK;
      }

      /** Clear all registered callbacks.
       * This is probably only useful for testing but who knows!
       */
      void sbp_clear_callbacks(sbp_state_t *s)
      {
        /* Reset the head of the callbacks list to NULL. */
        s->sbp_msg_callbacks_head = 0;
      }

      /** Find the callback function associated with a message type.
       * Searches through the list of registered callbacks to find the callback
       * associated with the passed message type.
       *
       * \param msg_type Message type to find callback for
       * \return Pointer to callback node (#sbp_msg_callbacks_node_t) or `NULL` if
       *         callback not found for that message type.
       */
      sbp_msg_callbacks_node_t* sbp_find_callback(sbp_state_t *s, u16 msg_type)
      {
        /* If our list is empty, return NULL. */
        if (!s->sbp_msg_callbacks_head)
          return 0;

        /* Traverse the linked list and return the callback
         * function pointer if we find a node with a matching
         * message id.
         */
        sbp_msg_callbacks_node_t *p = s->sbp_msg_callbacks_head;
        do
          if (p->msg_type == msg_type)
            return p;

        while ((p = p->next));

        /* Didn't find a matching callback, return NULL. */
        return 0;
      }


      /** Initialize an #sbp_state_t struct before use.
       * This resets the entire state, including all callbacks.
       * Remember to use this function to initialize the state before calling
       * sbp_process() for the first time.
       *
       * \param s State structure
       */
      void sbp_state_init(sbp_state_t *s)
      {
        s->state = sbp_state_t::WAITING;

        /* Set the IO context pointer, passed to read and write functions, to NULL. */
        s->io_context = 0;

        /* Clear the callbacks, if any, currently in s */
        sbp_clear_callbacks(s);
      }


      /** Set a context to pass to all function pointer calls made by sbp functions
       * This helper function sets a void* context pointer in sbp_state.
       * Whenever `sbp_process` calls the `read` function pointer, it passes this context.
       * Whenever `sbp_send_message` calls the `write` function pointer, it passes this context.
       * This allows C++ code to get a pointer to an object inside these functions.
       */
      void sbp_state_set_io_context(sbp_state_t *s, void *context)
      {
        s->io_context = context;
      }

      /** Read and process SBP messages.
       * Reads bytes from an input source using the provided `read` function, decodes
       * the SBP framing and performs a CRC check on the message.
       *
       * When an SBP message is successfully received then the list of callbacks is
       * searched for a callback corresponding to the received message type. If a
       * callback is found then it is called with the ID of the sender, the message
       * length and the message payload data buffer as arguments.
       *
       * \note sbp_process will always call `read` with n > 0
       *       (aka it will attempt to always read something)
       *
       * The supplied `read` function must have the prototype:
       *
       * ~~~
       * u32 read(u8 *buff, u32 n, void* context)
       * ~~~
       *
       * where `n` is the number of bytes requested and `buff` is the buffer into
       * which to write the received data, and `context` is the arbitrary pointer
       * set by `sbp_state_set_io_context`.
       * The function should return the number of
       * bytes successfully written into `buff` which may be between 0 and `n`
       * inclusive, but must never be greater than `n`.
       *
       * Note that `sbp_process` may not read all available bytes from the `read`
       * function so the caller should loop until all bytes available from the input
       * source have been consumed.
       *
       * \param s State structure
       * \param read Function pointer to a function that reads `n` bytes from the
       *             input source into `buff` and returns the number of bytes
       *             successfully read.
       * \return `SBP_OK` (0) if successful but no complete message yet,
       *         `SBP_OK_CALLBACK_EXECUTED` (1) if message decoded and callback executed,
       *         `SBP_OK_CALLBACK_UNDEFINED` (2) if message decoded with no associated
       *         callback, and `SBP_CRC_ERROR` (-2) if a CRC error
       *         has occurred. Thus can check for >0 to ensure good processing.
       */


      s8 sbp_process(sbp_state_t *s, u32 (*read)(u8 *buff, u32 n, void *context))
      {
        u8 temp;
        u16 crc;

        switch (s->state) {
          case sbp_state_t::WAITING:
            if ((*read)(&temp, 1, s->io_context) == 1)
              if (temp == SBP_PREAMBLE) {
                s->n_read = 0;
                s->state = sbp_state_t::GET_TYPE;
              }
            break;

          case sbp_state_t::GET_TYPE:
            s->n_read += (*read)((u8*)&(s->msg_type) + s->n_read,
                2-s->n_read, s->io_context);
            if (s->n_read >= 2) {
              /* Swap bytes to little endian. */
              s->n_read = 0;
              s->state = sbp_state_t::GET_SENDER;
            }
            break;

          case sbp_state_t::GET_SENDER:
            s->n_read += (*read)((u8*)&(s->sender_id) + s->n_read,
                2-s->n_read, s->io_context);
            if (s->n_read >= 2) {
              /* Swap bytes to little endian. */
              s->state = sbp_state_t::GET_LEN;
            }
            break;

          case sbp_state_t::GET_LEN:
            if ((*read)(&(s->msg_len), 1, s->io_context) == 1) {
              s->n_read = 0;
              s->state = sbp_state_t::GET_MSG;
            }
            break;

          case sbp_state_t::GET_MSG:
            /* Not received whole message yet, try and read some more. */
            s->n_read += (*read)(
                &(s->msg_buff[s->n_read]),
                s->msg_len - s->n_read,
                s->io_context
            );
            if (s->msg_len - s->n_read <= 0) {
              s->n_read = 0;
              s->state = sbp_state_t::GET_CRC;
            }
            break;

          case sbp_state_t::GET_CRC:
            s->n_read += (*read)((u8*)&(s->crc) + s->n_read,
                2-s->n_read, s->io_context);
            if (s->n_read >= 2) {
              s->state = sbp_state_t::WAITING;

              /* Swap bytes to little endian. */
              crc = crc16_ccitt((u8*)&(s->msg_type), 2, 0);
              crc = crc16_ccitt((u8*)&(s->sender_id), 2, crc);
              crc = crc16_ccitt(&(s->msg_len), 1, crc);
              crc = crc16_ccitt(s->msg_buff, s->msg_len, crc);
              if (s->crc == crc) {

                /* Message complete, process it. */
                // Check message type
                if(s->msg_type == 2) { /* user-defined */
                  // Call our only callback function

                  //AT_Measurement* msg = (AT_Measurement*) s->msg_buff;
                  AT_Measurement* msg = new AT_Measurement();


                  u32* tmp = (u32*) &s->msg_buff[0];
                  msg->dist = *tmp;

                  u8* tmp2 = (u8*) &s->msg_buff[4];
                  msg->dqf = *tmp2;

                  u16* tmp3 = (u16*) &s->msg_buff[5];
                  msg->sender = *tmp3;

                  u16* tmp4 = (u16*) &s->msg_buff[7];
                  msg->reflector = *tmp4;

                  u32* tmp5 = (u32*) &s->msg_buff[9];
                  msg->time = *tmp5;

                  handleATMEasurementMessage(*msg);


                  return SBP_OK_CALLBACK_EXECUTED;
                }


                sbp_msg_callbacks_node_t* node = sbp_find_callback(s, s->msg_type);
                if (node) {
                  (*node->cb)(s->sender_id, s->msg_len, s->msg_buff, node->context);
                  return SBP_OK_CALLBACK_EXECUTED;
                } else {
                  return SBP_OK_CALLBACK_UNDEFINED;
                }
              } else
                return SBP_CRC_ERROR;
            }
            break;

          default:
            s->state = sbp_state_t::WAITING;
            break;
        }

        return SBP_OK;
      }

      /*      u32 my_read(u8 *buff, u32 n, void *context)
      {
    	  for (u32 i=0; i<n; i++)
    	  {
    		  if (uart_has_data())
    			  buff[i] = receiveData();
    		  else
    			  break;
    	  }
    	  return i;
      }*/

      /////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////////
      // END SBP
      ////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////


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

      void
      openConnection(void)
      {
        try
        {
          m_TCP_sock = new TCPSocket;
          m_TCP_sock->connect(m_TCP_addr, m_TCP_port);
          inf(DTR("Atmel interface initialized"));
        }
        catch (...)
        {
          Memory::clear(m_TCP_sock);
          war(DTR("Local Connection failed, retrying..."));
          setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_COM_ERROR);
        }
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
        m_TCP_addr = m_args.TCP_addr;
        m_TCP_port = m_args.TCP_port;

        openConnection();
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
        Memory::clear(m_TCP_sock);

      }

      bool
      poll(double timeout)
      {
        if (m_TCP_sock != NULL)
          return Poll::poll(*m_TCP_sock, timeout);

        return false;
      }

      int
      sendData(uint8_t* bfr, int size)
      {
        if (m_TCP_sock)
        {
          trace("Sending something");
          return m_TCP_sock->write((char*)bfr, size);
        }
        return 0;
      }



      static u32
      receiveData(u8* buf, u32 blen, void* context)
      {
        Task* task = (Task*)(context);
        if (task->m_TCP_sock)
        {
          try
          {
            return task->m_TCP_sock->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            task->err("%s", e.what());
            task->war(DTR("Connection lost locally, retrying..."));
            Memory::clear(task->m_TCP_sock);

            task->m_TCP_sock = new Network::TCPSocket;
            task->m_TCP_sock->connect(task->m_TCP_addr, task->m_TCP_port);
            return 0;
          }
        }
        return 0;
      }

      /*      int
      receiveData(uint8_t* buf, size_t blen)
      {
        if (m_TCP_sock)
        {
          try
          {
            return m_TCP_sock->read(buf, blen);
          }
          catch (std::runtime_error& e)
          {
            err("%s", e.what());
            war(DTR("Connection lost, retrying..."));
            Memory::clear(m_TCP_sock);

            m_TCP_sock = new Network::TCPSocket;
            m_TCP_sock->connect(m_TCP_addr, m_TCP_port);
            return 0;
          }
        }
        return 0;
      }*/

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          // Handle data
          if (m_TCP_sock)
          {
            handleInputData();
          }
          else
          {
            Time::Delay::wait(0.5);
            openConnection();
          }

          if (!m_error_missing_data)
          {

            if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
            {
              setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
            }
          }

          // Handle IMC messages from bus
          consumeMessages();
        }
      }

      void
      handleATMEasurementMessage(AT_Measurement msg)
      {

    	  //double tstamp = Clock::getSinceEpoch();

    	  IMC::BeaconDistance bcnDistance;
    	  bcnDistance.dist = msg.dist;
    	  bcnDistance.dqf = msg.dqf;
    	  bcnDistance.receiver = msg.reflector;
    	  bcnDistance.sender = msg.sender;
    	  bcnDistance.time = msg.time;
    	  dispatch(bcnDistance);

      }



      void
      handleInputData(void)
      {
        double now = Clock::get();
        int counter = 0;


        while (poll(0.01) && counter < 100)
        {

          // START HERE SBP!!!!!
          counter++;

          now = Clock::get();
          int result = sbp_process(&m_sbp_state, receiveData);
          m_last_pkt_time = now;

          switch(result)
          {
            case SBP_OK:
            case SBP_OK_CALLBACK_EXECUTED:
              break;
            case SBP_OK_CALLBACK_UNDEFINED:
              debug("Unknown message. (NB: may be heartbeat).");
              break;
            case SBP_CRC_ERROR:
              debug("Received message CRC error.");
              break;
            default:
              spew("Unknown result from sbp process.");

              // END HERE SPB




              /*         counter++;

          int n = receiveData(m_buf, sizeof(m_buf));

          if (n < 0)
          {
            debug("Receive error");
            break;
          }

          now = Clock::get();


          // Do something with the data.


          inf("Got Something!");

          m_last_pkt_time = now;*/
          }

          // SBP!


          if (now - m_last_pkt_time >= m_args.comm_timeout)
          {

            if (!m_error_missing_data)
            {
              setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
              m_error_missing_data = true;
            }
          }
          else
            m_error_missing_data = false;
        }
      }
    };
  }
}

DUNE_TASK

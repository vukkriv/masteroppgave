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
// Author: Jostein B. Moe                                                   *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
  namespace NetCatchTransport
  {
    using DUNE_NAMESPACES;

    //! %Input type
    enum INPUT_TYPE
    {
      RTK = 0,
      ESTATE
    };

    struct Arguments
    {
      //! Input type
      std::string type;
      //! Reference latitude
      double ref_lat;
      //! Reference longitude
      double ref_lon;
      //! Reference height (above elipsoid)
      double ref_hae;
      //! RtkFix Jump Threshold
      double rtk_jump_threshold;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Sensor Type
      INPUT_TYPE m_type;

      IMC::EstimatedLocalState m_state;

	    //! Localization origin (WGS-84)
	    fp64_t m_ref_lat, m_ref_lon;
	    fp32_t m_ref_hae;
      bool m_ref_valid;
      //! Previous RtkFix received
      IMC::GpsFixRtk m_prev_RtkFix;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
          m_type(RTK),
		  m_ref_lat(0.0),
		  m_ref_lon(0.0),
		  m_ref_hae(0.0),
		  m_ref_valid(false)
      {
          param("Input Type", m_args.type)
          .defaultValue("RTK")
          .values("RTK,EstimatedState")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Input Type - RTK or EstimatedState");

          param("Latitude", m_args.ref_lat)
          .defaultValue("-999.0")
          .units(Units::Degree)
          .description("Reference Latitude");

          param("Longitude", m_args.ref_lon)
          .defaultValue("0.0")
          .units(Units::Degree)
          .description("Reference Longitude");

          param("Height", m_args.ref_hae)
          .defaultValue("0.0")
          .units(Units::Meter)
          .description("Reference Height (above elipsoid)");

          param("RTK Jump Threshold", m_args.rtk_jump_threshold)
          .defaultValue("5.0")
          .units(Units::MeterPerSecond)
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Threshold for distance between two consecutive RTK Fixes");

          // Bind to incoming IMC messages
          bind<IMC::GpsFixRtk>(this);
          bind<IMC::EstimatedState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
          // Check input type
          if (m_args.type == "EstimatedState")
          {
            m_type = ESTATE;
            inf("Entered ESTATE mode");
          }
          else
          {
            m_type = RTK;
            inf("Entered RTK mode");
          }

          if (paramChanged(m_args.ref_lat) || paramChanged(m_args.ref_lon) || paramChanged(m_args.ref_hae))
          {
            if (m_args.ref_lat == -999) // Reference not set; return
              return;

            inf("New reference position.");

            // Check validity
            if (std::abs(m_args.ref_lat) > 90)
              throw DUNE::Exception("Unvalid reference latitude!");
            if (std::abs(m_args.ref_lon) > 180)
              throw DUNE::Exception("Unvalid reference longitude!");

            m_ref_lat = Angles::radians(m_args.ref_lat);
            m_ref_lon = Angles::radians(m_args.ref_lon);
            m_ref_hae = m_args.ref_hae;
            m_ref_valid = true;
            inf("Ref. LLH set from ini: [Lat = %f, Lon = %f, Height = %.1f]",
                Angles::degrees(m_ref_lat), Angles::degrees(m_ref_lon), m_ref_hae);
          }
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
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
       consume(const IMC::GpsFixRtk* msg)
       {
         spew("Got RTK Fix");

         if (m_type == RTK)
         {
           switch (msg->type)
           {
             case IMC::GpsFixRtk::RTK_NONE:
               break;
             case IMC::GpsFixRtk::RTK_OBS:
               break;
             case IMC::GpsFixRtk::RTK_FLOAT:
               break;
             case IMC::GpsFixRtk::RTK_FIXED:
               // Check for jump in fix
               if (m_prev_RtkFix.type == IMC::GpsFixRtk::RTK_FIXED)
               {
                 Matrix diff_ned(3,1,0);
                 diff_ned(0) = msg->n - m_prev_RtkFix.n;
                 diff_ned(1) = msg->e - m_prev_RtkFix.e;
                 diff_ned(2) = msg->d - m_prev_RtkFix.d;
                 double diff_dist = diff_ned.norm_2();
                 double diff_time = (msg->tow - m_prev_RtkFix.tow)/1E3;
                 if (diff_time < 0)
                 {
                   war("Received old RtkFix! Diff = %1.1f s",
                       diff_time);
                   return;
                 }

                 double change = diff_dist/diff_time;
                 if (change > m_args.rtk_jump_threshold)
                 {
                   err("Jump in RTK Fix above threshold (%1.1f): %1.1f m/s \nJumped %1.1f m in %1.1f s",
                       m_args.rtk_jump_threshold, change, diff_dist, diff_time);
                   IMC::Abort abort_msg;
                   abort_msg.setDestination(this->getSystemId());
                   dispatch(abort_msg);
                   debug("Abort sent.");
                 }
                 else if (change > m_args.rtk_jump_threshold*0.75)
                 {
                   war("Jump in RTK Fix close to threshold (%1.1f): %1.1f m/s",
                       m_args.rtk_jump_threshold, change);
                 }
               }

               // Set time stamp
               m_state.ots = msg->getTimeStamp();
               // Set position
               m_state.x = msg->n;
               m_state.y = msg->e;
               m_state.z = msg->d;
               // Set velocity
               m_state.vx = msg->v_n;
               m_state.vy = msg->v_e;
               m_state.vz = msg->v_d;

               dispatch(m_state);
               spew("Sent Estimated Local State");
           }

           m_prev_RtkFix = *msg;
         }
       }

      void
      consume(const IMC::EstimatedState* msg)
      {
  	    //Message should be from this vehicle
	    if ( msg->getSource() != getSystemId() )
		  return;

	    spew("Got Estimated State from system '%s' and entity '%s'.",
		   resolveSystemId(msg->getSource()),
		   resolveEntity(msg->getSourceEntity()).c_str());

        if (m_type == ESTATE)
        {
          if (!m_ref_valid)
          {
            war("Ignored ESTATE; valid reference LLH not set!");
            return;
          }
          // Set time stamp
          m_state.ots = msg->getTimeStamp();
          // Set displacement of agent reference from main reference LLH
          WGS84::displacement(m_ref_lat, m_ref_lon, m_ref_hae,
                              msg->lat, msg->lon, msg->height,
                              &m_state.x, &m_state.y, &m_state.z);
          // Add displacement from agent reference
          m_state.x += msg->x;
          m_state.y += msg->y;
          m_state.z += msg->z;
          // Set velocity
          m_state.vx = msg->vx;
          m_state.vy = msg->vy;
          m_state.vz = msg->vz;

          m_state.source = IMC::EstimatedLocalState::SRC_GPS;
          m_state.ref = IMC::EstimatedLocalState::REF_FIXED;

          if (m_ref_valid)
          {
        	  m_state.lat 	 = m_ref_lat;
        	  m_state.lon 	 = m_ref_lon;
        	  m_state.height = m_ref_hae;
          }
          spew("ES: Height = %1.1f, z = %1.1f, New Z = %1.1f",
              msg->height, msg->z,m_state.z);

          // Keep source entity and source ID
          //m_state.setSource(msg->getSource());
          //m_state.setSourceEntity(msg->getSourceEntity());
          //dispatch(m_state, DF_KEEP_SRC_EID);

          dispatch(m_state);

          spew("Sent Estimated Local State");
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK

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
  namespace LocalStateTransport
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
        double ref_lat;
        //! Reference longitude
        double ref_lon;
        //! Reference height (above elipsoid)
        double ref_hae;

        bool use_static_ref;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;

      //! Localization origin (WGS-84)
      fp64_t m_ref_lat, m_ref_lon;
      fp32_t m_ref_hae;
      bool m_ref_valid;
      
      IMC::EstimatedLocalState m_state;    


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
          param("Latitude", m_args.ref_lat)
          .defaultValue("-999.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .units(Units::Degree)
          .description("Reference Latitude");

          param("Longitude", m_args.ref_lon)
          .defaultValue("0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .units(Units::Degree)
          .description("Reference Longitude");

          param("Height", m_args.ref_hae)
          .defaultValue("0.0")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .units(Units::Meter)
          .description("Reference Height (above elipsoid)");

          param("Use Static Reference", m_args.use_static_ref)
          .defaultValue("true")
          .visibility(Tasks::Parameter::VISIBILITY_USER)
          .description("Static reference LLH is set from config file");

          // Bind to incoming IMC messages
          bind<IMC::EstimatedState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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
      consume(const IMC::EstimatedState* msg)
      {
  	    //Message should be from this vehicle
	     if ( msg->getSource() != getSystemId() )
		    return;

	     spew("Got Estimated State from system '%s' and entity '%s'.",
		   resolveSystemId(msg->getSource()),
		   resolveEntity(msg->getSourceEntity()).c_str());

		if (m_args.use_static_ref)
		{
		 m_state.lat    = m_ref_lat;
		 m_state.lon    = m_ref_lon;
		 m_state.height = m_ref_hae;
		 WGS84::displacement(m_ref_lat, m_ref_lon, m_ref_hae,
							 msg->lat, msg->lon, msg->height,
							 &m_state.x, &m_state.y, &m_state.z);
		 // Add displacement from agent reference
		 m_state.x += msg->x;
		 m_state.y += msg->y;
		 m_state.z += msg->z;

		 m_state.ref    = IMC::EstimatedLocalState::REF_FIXED;
		 spew("Fixed reference on EstimatedState");
		}
		else
		{
		 m_state.lat    = msg->lat;
		 m_state.lon    = msg->lon;
		 m_state.height = msg->height;
		 m_state.x = msg->x;
		 m_state.y = msg->y;
		 m_state.z = msg->z;
		 m_state.ref = IMC::EstimatedLocalState::REF_MOVING;
		}

        m_state.u = msg->u;
        m_state.v = msg->v;
        m_state.w = msg->w;

        m_state.phi   = msg->phi;
        m_state.theta = msg->theta;
        m_state.psi   = msg->psi;

        m_state.p = msg->p;
        m_state.q = msg->q;
        m_state.r = msg->r;

        // Set time stamp
        m_state.ots = msg->getTimeStamp();

        // Set velocity
        m_state.vx = msg->vx;
        m_state.vy = msg->vy;
        m_state.vz = msg->vz;

        //the following are not neccesary anymore?
        m_state.source = IMC::EstimatedLocalState::SRC_GPS;
        m_state.ref = IMC::EstimatedLocalState::REF_FIXED;

        // Keep source entity and source ID
        //m_state.setSource(msg->getSource());
        //m_state.setSourceEntity(msg->getSourceEntity());
        //dispatch(m_state, DF_KEEP_SRC_EID);
        dispatch(m_state);
        spew("Sent Estimated Local State");        
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

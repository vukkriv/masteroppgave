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
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;

      //! Localization origin (WGS-84)
      fp64_t m_ref_lat, m_ref_lon;
      fp32_t m_ref_hae;
      bool m_ref_valid;
      bool m_use_fallback;

      bool m_configured;
      bool m_warning_given;

      IMC::EstimatedLocalState m_elstate;
      IMC::EstimatedState m_estate;
      IMC::Acceleration m_acc;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_use_fallback(true),
        m_configured(false),
        m_warning_given(false)
      {
        // Bind to incoming IMC messages
        bind<IMC::EstimatedState>(this);
        bind<IMC::Acceleration>(this);
        bind<IMC::CoordConfig>(this);

        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_ACTIVATING);
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
      consume(const IMC::CoordConfig* config)
      {
        m_configured = true;
        m_use_fallback = config->use_fallback;
        if (m_use_fallback)
        {
          if (config->lat == -999) // Reference not set; return
            return;

          // Check validity
          if (std::abs(config->lat) > 90)
            throw DUNE::Exception("Unvalid reference latitude!");
          if (std::abs(config->lon) > 180)
            throw DUNE::Exception("Unvalid reference longitude!");

          m_ref_lat = config->lat;
          m_ref_lon = config->lon;
          m_ref_hae = config->height;
          m_ref_valid = true;
          debug("Ref. LLH set from ini: [Lat = %f, Lon = %f, Height = %.1f]",
              Angles::degrees(m_ref_lat), Angles::degrees(m_ref_lon), m_ref_hae);
        }
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        //Message should be from this vehicle
        if ( msg->getSource() != getSystemId() )
        return;

        if (!m_ref_valid && m_use_fallback)
        {
          if(!m_warning_given)
          {
            war("Ignored EstimatedState, valid LLH reference is not set!");
            m_warning_given = true;
            m_configured = false;
            setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
          }
         return;
        }

        if (m_warning_given && m_configured)
        {
          m_warning_given = false;
          war("Valid LLH reference is set, starting producing EstimatedLocalState");
        }
        spew("Got Estimated State from system '%s' and entity '%s'.",
                resolveSystemId(msg->getSource()),
                resolveEntity(msg->getSourceEntity()).c_str());

        if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        if (m_use_fallback)
        {
          m_estate.lat    = m_ref_lat;
          m_estate.lon    = m_ref_lon;
          m_estate.height = m_ref_hae;

          WGS84::displacement(m_ref_lat, m_ref_lon, m_ref_hae,
              msg->lat, msg->lon, msg->height,
              &m_estate.x, &m_estate.y, &m_estate.z);

          // Add displacement from agent reference
          m_estate.x += msg->x;
          m_estate.y += msg->y;
          m_estate.z += msg->z;
        }
        else
        {
          m_estate.lat    = msg->lat;
          m_estate.lon    = msg->lon;
          m_estate.height = msg->height;
          m_estate.x = msg->x;
          m_estate.y = msg->y;
          m_estate.z = msg->z;
        }

        m_estate.vx = msg->vx;
        m_estate.vy = msg->vy;
        m_estate.vz = msg->vz;

        m_estate.u = msg->u;
        m_estate.v = msg->v;
        m_estate.w = msg->w;

        m_estate.phi   = msg->phi;
        m_estate.theta = msg->theta;
        m_estate.psi   = msg->psi;

        m_estate.p = msg->p;
        m_estate.q = msg->q;
        m_estate.r = msg->r;

        // Set time stamp
        m_elstate.ots = msg->getTimeStamp();
        m_elstate.state.set(m_estate);
        m_elstate.acc.set(m_acc);

        dispatch(m_elstate);
        spew("Sent Estimated Local State");
      }

      void
      consume(const IMC::Acceleration* msg)
      {
        // Convert to NED and store.
        BodyFixedFrame::toInertialFrame(m_estate.phi, m_estate.theta, m_estate.psi,
                                        msg->x, msg->y, msg->z,
                                        &m_acc.x, &m_acc.y, &m_acc.z);
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

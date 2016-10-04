//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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
// This task lets the user set controller mode, either on external or maneuver
// basis.

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO headers
#include <vector>
#include <map>
#include <set>

namespace Supervisors
{
  namespace ControlProfile
  {
    using DUNE_NAMESPACES;

    const std::string c_controller_profiles = "Normal,High Gain,Cruise";
    const std::string c_controller_profile_flags = "No Integral,Land,Evasive,Passive";

    struct Arguments
    {
      //! True to let maneuver override mode. If enabled, trumps rc input.
      bool maneuver_override;
      std::string maneuver_profile;
      std::string maneuver_profile_flags;

      //! True to enable rc input. Is superseeded by maneuver_override.
      bool rc_enable;
      int rc_channel;
      unsigned int rc_threshold;
      std::string rc_profile_on_high;
      std::string rc_profile_flags;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task Arguments
      Arguments m_args;

      //! Vector of the controller profile strings;
      std::vector<std::string> m_cprofiles;

      //! String of accumulated flag combinations
      std::string m_cprofile_flag_combinations;

      //! Map of flag combinations vs bitvalues
      std::map<std::string, uint8_t> m_cprofile_flag_map;

      //! Is currently rc triggered
      bool m_rc_is_triggered;

      //! Current cmode id
      unsigned int m_cprofile_id;


      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_rc_is_triggered(false),
        m_cprofile_id(0)
      {
        // Setup cprofiles
        Utils::String::split(c_controller_profiles, ",", m_cprofiles);

        // Initialize the flag combinations and map
        std::vector<std::string> flags;
        Utils::String::split(c_controller_profile_flags, ",", flags);

        // Temporary map to store oposite lookup on flag map
        std::map<uint8_t, bool> tmp_flag_map_is_added;

        size_t nFlags = flags.size();

        // First, add the None flag
        m_cprofile_flag_combinations = "None";
        m_cprofile_flag_map["None"] = 0;
        tmp_flag_map_is_added[0] = true;



        // Add all individual flags
        for (size_t i = 0; i < flags.size(); ++i)
        {
          uint8_t flagCombination = (0x01 << (i));
          m_cprofile_flag_map[flags[i]] = flagCombination;
          m_cprofile_flag_combinations += "," + flags[i];

          tmp_flag_map_is_added[flagCombination] = true;
        }

        // Then, loop over all possible values
        for (size_t i = 0; i < std::pow(2, nFlags); ++i)
        {
          if (tmp_flag_map_is_added.find(i) == tmp_flag_map_is_added.end())
          {
            std::string value;
            for (size_t flag = 0; flag < nFlags; ++flag)
            {
              if (i & (0x01 << flag))
              {
                if (value.size() > 0)
                  value += "-";
                value += flags[flag];

              }
            }
            m_cprofile_flag_map[value] = i;
            m_cprofile_flag_combinations += "," + value;
            tmp_flag_map_is_added[i] = true;
          }
        }

        param("Maneuver Override", m_args.maneuver_override)
        .defaultValue("false")
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("Set true to let this maneuver override the mode. ");

        param("Maneuver Profile", m_args.maneuver_profile)
        .defaultValue(m_cprofiles[0])
        .values(c_controller_profiles)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("The profile to override if Maneuver Override is true");

        param("Maneuver Profile Flags", m_args.maneuver_profile_flags)
        .defaultValue("None")
        .values(m_cprofile_flag_combinations)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .description("The flags to set if Maneuver Override is true");

        param("RC - Enable", m_args.rc_enable)
        .defaultValue("False")
        .visibility(Parameter::VISIBILITY_USER)
        .description("Set to enable RC override. ");

        param("RC - Profile On High", m_args.rc_profile_on_high)
        .defaultValue(m_cprofiles[1])
        .values(c_controller_profiles)
        .visibility(Parameter::VISIBILITY_USER)
        .description("Mode to switch to on High");

        param("RC - Profile Flags on High", m_args.rc_profile_flags)
        .defaultValue("None")
        .values(m_cprofile_flag_combinations)
        .visibility(Tasks::Parameter::VISIBILITY_USER)
        .description("The flags to set if rc switch is high.");

        param("RC - Channel", m_args.rc_channel)
        .minimumValue("0")
        .defaultValue("6")
        .maximumValue("12")
        .description("Channel to survey");

        param("RC - Threshold", m_args.rc_threshold)
        .minimumValue("800")
        .defaultValue("1600")
        .maximumValue("2200")
        .description("Threshold for high signal. ");

        bind<IMC::PWM>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        // Simply send normal based on changed parameter.

        if(paramChanged(m_args.maneuver_override))
        {
          if (m_args.maneuver_override)
          {
            sendNewProfile(resolveControlType(m_args.maneuver_profile), resolveFlags(m_args.maneuver_profile_flags));
            inf("Did maneuver override to control mode: %s", m_args.maneuver_profile.c_str());
          }
          else
          {
            sendNewProfile(IMC::ControlProfile::CPP_NORMAL);
          }
        }


        if(paramChanged(m_args.rc_enable))
        {
          if (!m_args.rc_enable)
          {
            sendNewProfile(IMC::ControlProfile::CPP_NORMAL);
          }
        }
      }

      void
      sendNewProfile(IMC::ControlProfile::ProfileEnum profile, uint16_t flags = 0x0000)
      {
        IMC::ControlProfile cprofile;
        cprofile.op = IMC::ControlProfile::CPO_REQUEST;
        cprofile.op_id = getNextCmodeId();

        cprofile.profile = profile;
        cprofile.flags = flags;

        dispatch(cprofile);
      }

      IMC::ControlProfile::ProfileEnum
      resolveControlType(std::string type)
      {
        for(unsigned int i = 0; i < m_cprofiles.size(); i++)
        {
          if (type==m_cprofiles[i])
            return (IMC::ControlProfile::ProfileEnum) i;
        }
        return IMC::ControlProfile::CPP_NORMAL;
      }

      uint8_t
      resolveFlags(std::string flagString)
      {
        // Use the map to lookup
        if (m_cprofile_flag_map.find(flagString) != m_cprofile_flag_map.end())
        {
          return m_cprofile_flag_map[flagString];
        }
        return 0x00;
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
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      consume(const IMC::PWM* msg)
      {
        if (msg->id != m_args.rc_channel)
          return;

        bool was_triggered = m_rc_is_triggered;

        if (!m_args.maneuver_override && m_args.rc_enable)
        {
          m_rc_is_triggered = msg->duty_cycle > m_args.rc_threshold ? true : false;
        }

        if (was_triggered != m_rc_is_triggered)
        {
          if (m_rc_is_triggered)
          {
            sendNewProfile(resolveControlType(m_args.rc_profile_on_high), resolveFlags(m_args.rc_profile_flags));
          }
          else
          {
            sendNewProfile(IMC::ControlProfile::CPP_NORMAL);
          }
        }
      }

      unsigned int
      getNextCmodeId(void)
      {
        return ++m_cprofile_id;
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

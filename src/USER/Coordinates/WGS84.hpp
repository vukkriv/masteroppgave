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
// These are the standard conversion routines as described in: J. Zhu,      *
// "Conversion of Earth-centered Earth-fixed coordinates to geodetic        *
// coordinates," IEEE Transactions on Aerospace and Electronic              *
// Systems, IEEE Transactions on, vol. 30, pp. 957-961, 1994.               *
//                                                                          *
// A summary of the implementation can be found in:                         *
// - http://en.wikipedia.org/wiki/Geodetic_system                           *
//***************************************************************************

#ifndef USER_COORDINATES_WGS84_HPP_INCLUDED_
#define USER_COORDINATES_WGS84_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <cassert>
#include <cmath>
#include <cstddef>

// DUNE headers.
#include <DUNE/Config.hpp>

namespace DUNE
{
  namespace Coordinates
  {
    // Export DLL Symbol.
    class DUNE_DLL_SYM WGS84_Accurate;



    //! Routines for WGS-84 coordinate manipulation.
    class WGS84_Accurate
    {
    public:
      //! Displace a geodetic coordinate in the NED frame
      //! according to given offsets.
      //!
      //! @param[in] n North offset (m).
      //! @param[in] e East offset (m).
      //! @param[in] d Down offset (m).
      //! @param[in,out] lat reference latitude on entry, displaced
      //!                latitude on exit (rad).
      //! @param[in,out] lon reference longitude entry, displaced
      //!                longitude on exit (rad).
      //! @param[in,out] hae reference height on entry, displaced
      //!                height on exit (rad).
      template <typename Ta, typename Tb, typename Tc, typename Td>
      static void
      displace(Ta n, Ta e, Tb d,
               Tc* lat, Tc* lon, Td* hae)
      {
        // Convert reference to ECEF coordinates
        double x;
        double y;
        double z;
        toECEF(*lat, *lon, *hae, &x, &y, &z);

        // Compute Geocentric latitude
        //double phi = std::atan2(z, std::sqrt(x * x + y * y));
        double N = computeRn(*lat);
        double p = std::sqrt(x * x + y * y);
        double phi = std::atan2(z,p*(1 - c_wgs84_e2 * N / (N + *hae)));

        // Compute all needed sine and cosine terms for conversion.
        double slon = std::sin(*lon);
        double clon = std::cos(*lon);
        double sphi = std::sin(phi);
        double cphi = std::cos(phi);

        // Obtain ECEF coordinates of displaced point
        // Note: some signs from standard ENU formula
        // are inverted - we are working with NED (= END) coordinates
        x += -slon * e - clon * sphi * n - clon * cphi * d;
        y += clon * e - slon * sphi * n - slon * cphi * d;
        z += cphi * n - sphi * d;

        // Convert back to WGS-84 coordinates.
        fromECEF(x, y, z, lat, lon, hae);
      }

      //! Displace a WGS-84 coordinate in the NED frame according to
      //! given offsets.
      //!
      //! @param[in] n North offset (m).
      //! @param[in] e East offset (m).
      //! @param[in,out] lat reference latitude on entry, displaced
      //!                latitude on exit (rad).
      //! @param[in,out] lon reference longitude entry, displaced
      //!                longitude on exit (rad).
      template <typename Ta, typename Tb>
      static inline void
      displace(Ta n, Ta e, Tb* lat, Tb* lon)
      {
        // Dummy variable
        double hae = 0.00;

        // Call the general method
        displace(n, e, 0.00, lat, lon, &hae);
      }



    private:
      //! Convert WGS-84 coordinates to ECEF (Earch Center Earth Fixed) coordinates.
      //!
      //! @param[in] lat WGS-84 latitude (rad).
      //! @param[in] lon WGS-84 longitude (rad).
      //! @param[in] hae WGS-84 coordinate height (m).
      //! @param[out] x storage for ECEF x coordinate (m).
      //! @param[out] y storage for ECEF y coordinate (m).
      //! @param[out] z storage for ECEF z coordinate (m).
      template <typename Ta, typename Tb, typename Tc>
      static void
      toECEF(Ta lat, Ta lon, Tb hae, Tc* x, Tc* y, Tc* z)
      {
        assert(x != 0);
        assert(y != 0);
        assert(z != 0);

        double cos_lat = std::cos(lat);
        double sin_lat = std::sin(lat);
        double cos_lon = std::cos(lon);
        double sin_lon = std::sin(lon);
        double rn = computeRn(lat);

        *x = (rn + hae) * cos_lat * cos_lon;
        *y = (rn + hae) * cos_lat * sin_lon;
        *z = (((1.0 - c_wgs84_e2) * rn) + hae) * sin_lat;
      }

      //! Convert ECEF (x,y,z) to WGS-84 (lat, lon, hae).
      //!
      //! @param[in] x ECEF x coordinate (m).
      //! @param[in] y ECEF y coordinate (m).
      //! @param[in] z ECEF z coordinate (m).
      //! @param[out] lat WGS-84 latitude (rad).
      //! @param[out] lon WGS-84 longitude (rad).
      //! @param[out] hae height above WGS-84 ellipsoid (m).
      template <typename Ta, typename Tb, typename Tc, typename Td>
      static void
      fromECEF(Ta x, Ta y, Tb z, Tc* lat, Tc* lon, Td* hae)
      {
        assert(lat != 0);
        assert(lon != 0);
        assert(hae != 0);

        double p = std::sqrt(x * x + y * y);
        *lon = std::atan2(y, x);
        *lat = std::atan2(z / p, 0.01);
        double n = computeRn(*lat);
        *hae = p / std::cos(*lat) - n;
        double old_hae = -1e-9;
        double num = z / p;

        while (std::fabs(*hae - old_hae) > 1e-4)
        {
          old_hae = *hae;
          double den = 1 - c_wgs84_e2 * n / (n + *hae);
          *lat = std::atan2(num, den);
          n = computeRn(*lat);
          *hae = p / std::cos(*lat) - n;
        }
      }

      //! Compute the radius of curvature in the prime vertical (Rn).
      //!
      //! @param[in] lat WGS-84 latitude (rad).
      //!
      //! @return radius of curvature in the prime vertical (rad).
      template <typename Type>
      static inline Type
      computeRn(Type lat)
      {
        double lat_sin = std::sin(lat);
        return c_wgs84_a / std::sqrt(1 - c_wgs84_e2 * (lat_sin * lat_sin));
      }
    };
  }
}

#endif

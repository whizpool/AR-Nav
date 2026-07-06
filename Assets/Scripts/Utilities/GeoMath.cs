namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    using UnityEngine;
    using System;

    /// <summary>
    /// Converts WGS84 geodetic coordinates to a local East-North-Up (ENU)
    /// coordinate system centered on a reference point.
    /// Unity mapping: X = East, Y = Up, Z = North.
    /// </summary>
    public static class GeoMath
    {
        #region Constants
        private const double EarthRadius = 6378137.0; // WGS84 semi-major axis (meters)
        #endregion

        #region Public Geodetic Conversion & Distance Math
        /// <summary>
        /// Returns a Unity Vector3 in meters relative to the reference origin.
        /// </summary>
        public static Vector3 GeoToLocal(
            double lat, double lon, double alt,
            double refLat, double refLon, double refAlt)
        {
            double dLat = (lat - refLat) * Mathf.Deg2Rad;
            double dLon = (lon - refLon) * Mathf.Deg2Rad;
            double refLatRad = refLat * Mathf.Deg2Rad;

            // Meters per degree at reference latitude
            double metersPerDegLat = EarthRadius * Mathf.Deg2Rad;
            double metersPerDegLon = EarthRadius * Math.Cos(refLatRad) * Mathf.Deg2Rad;

            float east  = (float)(dLon / Mathf.Deg2Rad * metersPerDegLon);  // X
            float up    = (float)(alt - refAlt);                              // Y
            float north = (float)(dLat / Mathf.Deg2Rad * metersPerDegLat);  // Z

            return new Vector3(east, up, north);
        }

        /// <summary>
        /// Haversine distance between two geo points, in meters.
        /// Used for proximity checks without needing full ENU conversion.
        /// </summary>
        public static double HaversineDistance(
            double lat1, double lon1, double lat2, double lon2)
        {
            double dLat = (lat2 - lat1) * Mathf.Deg2Rad;
            double dLon = (lon2 - lon1) * Mathf.Deg2Rad;
            double a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2)
                     + Math.Cos(lat1 * Mathf.Deg2Rad) * Math.Cos(lat2 * Mathf.Deg2Rad)
                     * Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            return EarthRadius * c;
        }
        #endregion
    }
}

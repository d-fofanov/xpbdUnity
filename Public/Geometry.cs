using UnityEngine;

namespace xpbdUnity
{
    public static class Geometry
    {
        public static float PointPlaneDistance(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
        {
            return Vector3.Dot(point - planePoint, planeNormal);
        }

        public static bool IsSphereIntersectingPlane(Vector3 spherePos, float sphereRadius,
            Vector3 planeNormal, Vector3 planePoint, out Vector3 secPoint, out float secRadius, out float depth)
        {
            var dist = PointPlaneDistance(spherePos, planeNormal, planePoint);
            var absDist = Mathf.Abs(dist);

            secPoint = spherePos - dist * planeNormal;
            secRadius = Mathf.Sqrt(Mathf.Max(0f, sphereRadius * sphereRadius - dist * dist));
            depth = sphereRadius - absDist;

            return absDist <= sphereRadius;
        }
    }
}
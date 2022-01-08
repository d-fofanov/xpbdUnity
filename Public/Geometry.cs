using System.Runtime.CompilerServices;
using UnityEngine;

namespace xpbdUnity
{
    public static class Geometry
    {
        public static float PointPlaneDistance(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
        {
            return Vector3.Dot(point - planePoint, planeNormal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ClipPointByPlaneInline(Vector3 point, Vector3 surfaceNormal, Vector3 surfacePoint)
        {
            var delta = surfacePoint - point;
            var dot = Vector3.Dot(delta, surfaceNormal);
            if (dot > 0f)
            {
                // place the point on the surface if it is behind it
                point += surfaceNormal * dot;
            }

            return point;
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

        /// <summary>
        /// Does Separating Axis Theorem intersection test for bodies with center symmetry
        /// </summary>
        /// <param name="deltaPos">Delta vector between bodies centers, from the first body to the second</param>
        /// <param name="directions">Directions to test</param>
        /// <param name="halfProjections">Halves of bodies' extents projected onto directions, mapped by indices, for the first body, then for the second one. Expected to be positive</param>
        /// <param name="collNormal">Out vector for collision's normal in terms of the first body</param>
        /// <param name="collDepth">Out float for collision's depth</param>
        /// <returns>True if collision detected</returns>
        public static bool SATIntersect(Vector3 deltaPos, Vector3[] directions, float[] halfProjections, out Vector3 collNormal,
            out float collDepth)
        {
            collNormal = Vector3.zero;
            collDepth = 0f;
            
            var dirCnt = directions.Length;
            if (dirCnt == 0 || halfProjections.Length != dirCnt * 2)
                return false;
            
            var minOverlap = float.MaxValue;
            for (int i=0; i<dirCnt; i++)
            {
                var dir = directions[i];
                if (dir == Vector3.zero)
                    break;
                
                var dp = Vector3.Dot(dir, deltaPos);
                var dpSign = Mathf.Sign(dp);
                dp *= dpSign;
                var b0Proj = halfProjections[i];
                var b1Proj = halfProjections[i + dirCnt];
                var penetrationDepth = b0Proj + b1Proj - dp;
                if (penetrationDepth <= 0f)
                    return false;

                if (penetrationDepth < minOverlap)
                {
                    minOverlap = penetrationDepth;
                    collNormal = -dir * dpSign;
                }
            }

            collDepth = minOverlap;
            return true;
        }
    }
}
using System.Runtime.CompilerServices;
using UnityEngine;

namespace xpbdUnity.Collision
{
    public class BoxCollider : BaseCollider
    {
        public Vector3 Size => _size;
        public Vector3 HalfSize => _halfSize;
        public override float Volume => _size.x * _size.y * _size.z;
        
        private Vector3 _size;
        private Vector3 _halfSize;
        
        public BoxCollider(Vector3 size, float mass)
        {
            _size = size;
            _halfSize = 0.5f * size;
            
            _invMass = 1f / mass;
            mass /= 12f;
            _invInertia = new Vector3(
                1f / (size.y * size.y + size.z * size.z) / mass,
                1f / (size.z * size.z + size.x * size.x) / mass,
                1f / (size.x * size.x + size.y * size.y) / mass);
        }

        public override bool Intersect(Pose atPose, BaseCollider withCollider, Pose otherPose, out Vector3 point, out Vector3 normal,
            out float shift)
        {
            if (withCollider is SphereCollider sphere)
                return IntersectWithSphere(atPose, sphere, otherPose, out point, out normal, out shift);
            
            if (withCollider is BoxCollider box)
                return IntersectWithBox(atPose, box, otherPose, out point, out normal, out shift);

            Debug.LogError($"Unsupported colliders pair: SphereCollider checks collision with {withCollider.GetType().Name}");
            point = normal = Vector3.zero;
            shift = 0f;
            return false;
        }

        internal bool IntersectWithSphere(Pose atPose, SphereCollider sphere, Pose otherPose, out Vector3 point, out Vector3 normal, out float shift)
        {
            var boxPos = atPose.Position;
            var boxRot = atPose.Rotation;
            var boxRight = boxRot.GetAxis0();
            var boxUp = boxRot.GetAxis1();
            var boxFwd = boxRot.GetAxis2();

            var fullExtent = boxRight * _halfSize.x + boxUp * _halfSize.y + boxFwd * _halfSize.z;
            var pMin = boxPos - fullExtent;
            var pMax = boxPos + fullExtent;

            point = Vector3.zero;
            normal = Vector3.zero;
            shift = 0f;

            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, boxRight, pMax,
                boxFwd, boxUp, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;
            
            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, -boxRight, pMin,
                boxFwd, boxUp, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;

            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, boxUp, pMax,
                boxRight, boxFwd, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;

            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, -boxUp, pMin,
                boxRight, boxFwd, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;

            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, boxFwd, pMax,
                boxRight, boxUp, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;
            
            if (IntersectSphereWithPlaneSegment(otherPose.Position, sphere.Radius, -boxFwd, pMin,
                boxRight, boxUp, pMax, pMin,
                ref point, ref normal, ref shift))
                return true;
            
            return shift > 0f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IntersectSphereWithPlaneSegment(Vector3 spherePos, float sphereRadius, Vector3 planeNormal,
            Vector3 planePoint, Vector3 sidePlane1Normal, Vector3 sidePlane2Normal, Vector3 sidePlanesPoint, Vector3 invSidePlanesPoint,
            ref Vector3 outPoint, ref Vector3 outNormal, ref float shift)
        {
            if (Geometry.IsSphereIntersectingPlane(spherePos, sphereRadius,
                planeNormal, planePoint, out var secPoint, out var secRadius, out var depthNormal))
            {
                var depth0 = secRadius - Geometry.PointPlaneDistance(secPoint, sidePlane1Normal, sidePlanesPoint);
                var depth1 = secRadius - Geometry.PointPlaneDistance(secPoint, -sidePlane1Normal, invSidePlanesPoint);
                var depth2 = secRadius - Geometry.PointPlaneDistance(secPoint, sidePlane2Normal, sidePlanesPoint);
                var depth3 = secRadius - Geometry.PointPlaneDistance(secPoint, -sidePlane2Normal, invSidePlanesPoint);
                
                // point either inside all the other planes, or it's no more than radius outside
                if (depth0 >= 0f && depth1 >= 0f && depth2 >= 0f && depth3 >= 0f)
                {
                    var isSupported = depth0 > secRadius && depth1 > secRadius && depth2 > secRadius && depth3 > secRadius;
                    if (isSupported)
                    {
                        outPoint = secPoint;
                        outNormal = -planeNormal;
                        shift = depthNormal;
                        return true;
                    }
                    
                    var depth = Mathf.Min(Mathf.Min(depth0, depth1), Mathf.Min(depth2, depth3));
                    if (depth >= shift && shift != 0f)
                        return false;

                    var minSecNormal = Vector3.zero;
                    if (depth == depth0)
                        minSecNormal = sidePlane1Normal;
                    else if (depth == depth1)
                        minSecNormal = -sidePlane1Normal;
                    else if (depth == depth2)
                        minSecNormal = sidePlane2Normal;
                    else if (depth == depth3)
                        minSecNormal = -sidePlane2Normal;
                    
                    shift = depth;
                    outPoint = secPoint - minSecNormal * (secRadius - 0.5f * depth);
                    outNormal = (outPoint - spherePos).normalized;
                }
            }

            return false;
        }

        private bool IntersectWithBox(Pose atPose, BoxCollider box, Pose otherPose, out Vector3 point, out Vector3 normal, out float shift)
        {
            throw new System.NotImplementedException();
        }

        public override bool IntersectWithFloor(Pose atPose, float floorLevel, out Vector3 point, out Vector3 normal, out float shift)
        {
            normal = Vector3.up;
            
            var boxPos = atPose.Position;
            var boxRot = atPose.Rotation;
            var boxRight = boxRot.GetAxis0().normalized;
            var boxUp = boxRot.GetAxis1().normalized;
            var boxFwd = boxRot.GetAxis2().normalized;
                
            var extRight = boxRight * _halfSize.x;
            var extUp = boxUp * _halfSize.y;
            var extFwd = boxFwd * _halfSize.z;

            if (boxPos.y > Mathf.Abs(extRight.y) + Mathf.Abs(extUp.y) + Mathf.Abs(extFwd.y))
            {
                point = Vector3.zero;
                shift = 0f;
                return false;
            }

            var p0 = boxPos + extRight + extUp + extFwd;
            var p1 = boxPos + extRight + extUp - extFwd;
            var p2 = boxPos + extRight - extUp + extFwd;
            var p3 = boxPos + extRight - extUp - extFwd;
            var p4 = boxPos - extRight + extUp + extFwd;
            var p5 = boxPos - extRight + extUp - extFwd;
            var p6 = boxPos - extRight - extUp + extFwd;
            var p7 = boxPos - extRight - extUp - extFwd;

            var s0 = Mathf.Max(0f, floorLevel - p0.y);
            var s1 = Mathf.Max(0f, floorLevel - p1.y);
            var s2 = Mathf.Max(0f, floorLevel - p2.y);
            var s3 = Mathf.Max(0f, floorLevel - p3.y);
            var s4 = Mathf.Max(0f, floorLevel - p4.y);
            var s5 = Mathf.Max(0f, floorLevel - p5.y);
            var s6 = Mathf.Max(0f, floorLevel - p6.y);
            var s7 = Mathf.Max(0f, floorLevel - p7.y);
            
            var totalShift = s0 + s1 + s2 + s3 + s4 + s5 + s6 + s7;
            point = p0 * s0 + p1 * s1 + p2 * s2 + p3 * s3 + p4 * s4 + p5 * s5 + p6 * s6 + p7 * s7;
            point *= 1f / totalShift;
            shift = Mathf.Max(Mathf.Max(Mathf.Max(s0, s1), Mathf.Max(s2, s3)),
                Mathf.Max(Mathf.Max(s4, s5), Mathf.Max(s6, s7)));

            return true;
        }
    }
}
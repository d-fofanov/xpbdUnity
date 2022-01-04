using System.Runtime.CompilerServices;
using UnityEngine;

namespace xpbdUnity.Collision
{
    public class BoxCollider : BaseCollider
    {
        public Vector3 Size => _size;
        public Vector3 HalfSize => _halfSize;
        public override float Volume => _size.x * _size.y * _size.z;
        public override Vector3 AABBSize => _size;
        
        private Vector3 _size;
        private Vector3 _halfSize;
        
        public BoxCollider(Vector3 size, float mass, Vector3? drag = null)
        {
            _size = size;
            _halfSize = 0.5f * size;

            _drag = drag ?? Vector3.zero;
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

        private static Vector3[] _directionsCache = new Vector3[6];
        private static float[] _halfProjectionsCache = new float[12];
        private bool IntersectWithBox(Pose atPose, BoxCollider box, Pose otherPose, out Vector3 point, out Vector3 normal, out float shift)
        {
            var box0HalfExtents = _halfSize;
            var box0Rot = atPose.Rotation;
            _directionsCache[0] = box0Rot.GetAxis0();
            _directionsCache[1] = box0Rot.GetAxis1();
            _directionsCache[2] = box0Rot.GetAxis2();

            var box1HalfExtents = box._halfSize;
            var box1Rot = otherPose.Rotation;
            _directionsCache[3] = box1Rot.GetAxis0();
            _directionsCache[4] = box1Rot.GetAxis1();
            _directionsCache[5] = box1Rot.GetAxis2();

            var box0Axis0HalfExt = _directionsCache[0] * box0HalfExtents.x;
            var box0Axis1HalfExt = _directionsCache[1] * box0HalfExtents.y;
            var box0Axis2HalfExt = _directionsCache[2] * box0HalfExtents.z;
            _halfProjectionsCache[0] = box0HalfExtents.x;
            _halfProjectionsCache[1] = box0HalfExtents.y;
            _halfProjectionsCache[2] = box0HalfExtents.z;
            _halfProjectionsCache[3] = ProjectBoxOnAxis(_directionsCache[3], box0Axis0HalfExt, box0Axis1HalfExt, box0Axis2HalfExt);
            _halfProjectionsCache[4] = ProjectBoxOnAxis(_directionsCache[4], box0Axis0HalfExt, box0Axis1HalfExt, box0Axis2HalfExt);
            _halfProjectionsCache[5] = ProjectBoxOnAxis(_directionsCache[5], box0Axis0HalfExt, box0Axis1HalfExt, box0Axis2HalfExt);
            
            var box1Axis0HalfExt = _directionsCache[3] * box1HalfExtents.x;
            var box1Axis1HalfExt = _directionsCache[4] * box1HalfExtents.y;
            var box1Axis2HalfExt = _directionsCache[5] * box1HalfExtents.z;
            _halfProjectionsCache[6] = ProjectBoxOnAxis(_directionsCache[0], box1Axis0HalfExt, box1Axis1HalfExt, box1Axis2HalfExt);
            _halfProjectionsCache[7] = ProjectBoxOnAxis(_directionsCache[1], box1Axis0HalfExt, box1Axis1HalfExt, box1Axis2HalfExt);
            _halfProjectionsCache[8] = ProjectBoxOnAxis(_directionsCache[2], box1Axis0HalfExt, box1Axis1HalfExt, box1Axis2HalfExt);
            _halfProjectionsCache[9] = box1HalfExtents.x;
            _halfProjectionsCache[10] = box1HalfExtents.y;
            _halfProjectionsCache[11] = box1HalfExtents.z;

            var deltaPos = otherPose.Position - atPose.Position;
            if (Geometry.SATIntersect(deltaPos, _directionsCache, _halfProjectionsCache, out var collNormal,
                    out var collDepth))
            {
                normal = collNormal;
                shift = collDepth;
                point = CalcCollisionPoint(collNormal,
                    atPose, box0HalfExtents, _directionsCache[0], _directionsCache[1], _directionsCache[2],
                    otherPose, box1HalfExtents, _directionsCache[3], _directionsCache[4], _directionsCache[5]
                );
                return true;
            }

            normal = point = Vector3.zero;
            shift = 0f;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private float ProjectBoxOnAxis(Vector3 axis, Vector3 axis0HalfExt, Vector3 axis1HalfExt, Vector3 axis2HalfExt)
        {
            var proj0 = Mathf.Abs(Vector3.Dot(axis, axis0HalfExt + axis1HalfExt + axis2HalfExt));
            var proj1 = Mathf.Abs(Vector3.Dot(axis, axis0HalfExt + axis1HalfExt - axis2HalfExt));
            var proj2 = Mathf.Abs(Vector3.Dot(axis, axis0HalfExt - axis1HalfExt + axis2HalfExt));
            var proj3 = Mathf.Abs(Vector3.Dot(axis, axis0HalfExt - axis1HalfExt - axis2HalfExt));

            return Mathf.Max(Mathf.Max(proj0, proj1), Mathf.Max(proj2, proj3));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Vector3 CalcCollisionPoint(Vector3 collNormal,
            Pose box0Pose, Vector3 box0HalfExtents, Vector3 box0Axis0, Vector3 box0Axis1, Vector3 box0Axis2,
            Pose box1Pose, Vector3 box1HalfExtents, Vector3 box1Axis0, Vector3 box1Axis1, Vector3 box1Axis2)
        {
            CalcCollisionAndClippingData(collNormal, box0Pose, box0HalfExtents, box0Axis0, box0Axis1, box0Axis2,
                out var box0PlaneNormal,
                out var box0PlanePoint0, out var box0PlanePoint1,
                out var box0PlanePoint2, out var box0PlanePoint3,
                out var box0ClipNormal0, out var box0ClipNormal1,
                out var box0ClipPointPositive, out var box0ClipPointNegative,
                out var box0PlaneNormalDot);
            
            CalcCollisionAndClippingData(-collNormal, box1Pose, box1HalfExtents, box1Axis0, box1Axis1, box1Axis2,
                out var box1PlaneNormal,
                out var box1PlanePoint0, out var box1PlanePoint1,
                out var box1PlanePoint2, out var box1PlanePoint3,
                out var box1ClipNormal0, out var box1ClipNormal1,
                out var box1ClipPointPositive, out var box1ClipPointNegative,
                out var box1PlaneNormalDot);
            
            bool box0IsReference = Mathf.Abs(box0PlaneNormalDot) > Mathf.Abs(box1PlaneNormalDot);
            if (box0IsReference)
            {
                ClipPointByPlanes(ref box1PlanePoint0, box0ClipNormal0, box0ClipNormal1, box0ClipPointPositive, box0ClipPointNegative);
                ClipPointByPlanes(ref box1PlanePoint1, box0ClipNormal0, box0ClipNormal1, box0ClipPointPositive, box0ClipPointNegative);
                ClipPointByPlanes(ref box1PlanePoint2, box0ClipNormal0, box0ClipNormal1, box0ClipPointPositive, box0ClipPointNegative);
                ClipPointByPlanes(ref box1PlanePoint3, box0ClipNormal0, box0ClipNormal1, box0ClipPointPositive, box0ClipPointNegative);
                var weight0 = Mathf.Max(0f, Vector3.Dot(box0PlanePoint0 - box1PlanePoint0, box0PlaneNormal));
                var weight1 = Mathf.Max(0f, Vector3.Dot(box0PlanePoint0 - box1PlanePoint1, box0PlaneNormal));
                var weight2 = Mathf.Max(0f, Vector3.Dot(box0PlanePoint0 - box1PlanePoint2, box0PlaneNormal));
                var weight3 = Mathf.Max(0f, Vector3.Dot(box0PlanePoint0 - box1PlanePoint3, box0PlaneNormal));

                var weightsSum = weight0 + weight1 + weight2 + weight3;
                if (weightsSum <= 0f)
                {
                    Debug.LogWarning("No points under the contact surface");
                    return box1PlanePoint0;
                }

                return (weight0 * box1PlanePoint0 + weight1 * box1PlanePoint1 +
                        weight2 * box1PlanePoint2 + weight3 * box1PlanePoint3) / weightsSum;
            }
            else
            {
                ClipPointByPlanes(ref box0PlanePoint0, box1ClipNormal0, box1ClipNormal1, box1ClipPointPositive, box1ClipPointNegative);
                ClipPointByPlanes(ref box0PlanePoint1, box1ClipNormal0, box1ClipNormal1, box1ClipPointPositive, box1ClipPointNegative);
                ClipPointByPlanes(ref box0PlanePoint2, box1ClipNormal0, box1ClipNormal1, box1ClipPointPositive, box1ClipPointNegative);
                ClipPointByPlanes(ref box0PlanePoint3, box1ClipNormal0, box1ClipNormal1, box1ClipPointPositive, box1ClipPointNegative);
                var weight0 = Mathf.Max(0f, Vector3.Dot(box1PlanePoint0 - box0PlanePoint0, box1PlaneNormal));
                var weight1 = Mathf.Max(0f, Vector3.Dot(box1PlanePoint0 - box0PlanePoint1, box1PlaneNormal));
                var weight2 = Mathf.Max(0f, Vector3.Dot(box1PlanePoint0 - box0PlanePoint2, box1PlaneNormal));
                var weight3 = Mathf.Max(0f, Vector3.Dot(box1PlanePoint0 - box0PlanePoint3, box1PlaneNormal));

                var weightsSum = weight0 + weight1 + weight2 + weight3;
                if (weightsSum <= 0f)
                {
                    Debug.LogWarning("No points under the contact surface");
                    return box0PlanePoint0;
                }

                return (weight0 * box0PlanePoint0 + weight1 * box0PlanePoint1 +
                        weight2 * box0PlanePoint2 + weight3 * box0PlanePoint3) / weightsSum;
            }
        }

        private void ClipPointByPlanes(ref Vector3 point,
            Vector3 clipNormal0, Vector3 clipNormal1, Vector3 clipPointPositive, Vector3 clipPointNegative)
        {
            // inverting normals to clip by the cube sides facing inward
            point = Geometry.ClipPointByPlaneInline(point, -clipNormal0, clipPointPositive);
            point = Geometry.ClipPointByPlaneInline(point, -clipNormal1, clipPointPositive);
            point = Geometry.ClipPointByPlaneInline(point, clipNormal0, clipPointNegative);
            point = Geometry.ClipPointByPlaneInline(point, clipNormal1, clipPointNegative);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void CalcCollisionAndClippingData(Vector3 collNormal,
            Pose boxPose, Vector3 boxHalfExtents, Vector3 boxAxis0, Vector3 boxAxis1, Vector3 boxAxis2,
            out Vector3 boxPlaneNormal, out Vector3 boxPlanePoint0, out Vector3 boxPlanePoint1, out Vector3 boxPlanePoint2, out Vector3 boxPlanePoint3,
            out Vector3 boxClipNormal0, out Vector3 boxClipNormal1, out Vector3 boxClipPointPositive, out Vector3 boxClipPointNegative,
            out float boxPlaneNormalDot)
        {
            var boxAxis0Dot = Vector3.Dot(collNormal, boxAxis0);
            var boxAxis1Dot = Vector3.Dot(collNormal, boxAxis1);
            var boxAxis2Dot = Vector3.Dot(collNormal, boxAxis2);
            if (Mathf.Abs(boxAxis0Dot) > Mathf.Abs(boxAxis1Dot) && Mathf.Abs(boxAxis0Dot) > Mathf.Abs(boxAxis2Dot))
            {
                boxPlaneNormalDot = boxAxis0Dot;
                var dotSign = -Mathf.Sign(boxAxis0Dot);
                boxPlaneNormal = boxAxis0 * dotSign;
                var boxSideCenter = boxPose.Position + boxHalfExtents.x * boxAxis0 * dotSign;
                boxPlanePoint0 = boxSideCenter + boxHalfExtents.y * boxAxis1 + boxHalfExtents.z * boxAxis2;
                boxPlanePoint1 = boxSideCenter + boxHalfExtents.y * boxAxis1 - boxHalfExtents.z * boxAxis2;
                boxPlanePoint2 = boxSideCenter - boxHalfExtents.y * boxAxis1 + boxHalfExtents.z * boxAxis2;
                boxPlanePoint3 = boxSideCenter - boxHalfExtents.y * boxAxis1 - boxHalfExtents.z * boxAxis2;

                boxClipNormal0 = boxAxis1;
                boxClipNormal1 = boxAxis2;
                boxClipPointPositive = boxPlanePoint0;
                boxClipPointNegative = boxPlanePoint3;
            }
            else if (Mathf.Abs(boxAxis1Dot) > Mathf.Abs(boxAxis0Dot) &&
                     Mathf.Abs(boxAxis1Dot) > Mathf.Abs(boxAxis2Dot))
            {
                boxPlaneNormalDot = boxAxis1Dot;
                var dotSign = -Mathf.Sign(boxAxis1Dot);
                boxPlaneNormal = boxAxis1 * dotSign;
                var boxSideCenter = boxPose.Position + boxHalfExtents.y * boxAxis1 * dotSign;
                boxPlanePoint0 = boxSideCenter + boxHalfExtents.x * boxAxis0 + boxHalfExtents.z * boxAxis2;
                boxPlanePoint1 = boxSideCenter + boxHalfExtents.x * boxAxis0 - boxHalfExtents.z * boxAxis2;
                boxPlanePoint2 = boxSideCenter - boxHalfExtents.x * boxAxis0 + boxHalfExtents.z * boxAxis2;
                boxPlanePoint3 = boxSideCenter - boxHalfExtents.x * boxAxis0 - boxHalfExtents.z * boxAxis2;

                boxClipNormal0 = boxAxis0;
                boxClipNormal1 = boxAxis2;
                boxClipPointPositive = boxPlanePoint0;
                boxClipPointNegative = boxPlanePoint3;
            }
            else
            {
                boxPlaneNormalDot = boxAxis2Dot;
                var dotSign = -Mathf.Sign(boxAxis2Dot);
                boxPlaneNormal = boxAxis2 * dotSign;
                var boxSideCenter = boxPose.Position + boxHalfExtents.z * boxAxis2 * dotSign;
                boxPlanePoint0 = boxSideCenter + boxHalfExtents.x * boxAxis0 + boxHalfExtents.y * boxAxis1;
                boxPlanePoint1 = boxSideCenter + boxHalfExtents.x * boxAxis0 - boxHalfExtents.y * boxAxis1;
                boxPlanePoint2 = boxSideCenter - boxHalfExtents.x * boxAxis0 + boxHalfExtents.y * boxAxis1;
                boxPlanePoint3 = boxSideCenter - boxHalfExtents.x * boxAxis0 - boxHalfExtents.y * boxAxis1;

                boxClipNormal0 = boxAxis0;
                boxClipNormal1 = boxAxis1;
                boxClipPointPositive = boxPlanePoint0;
                boxClipPointNegative = boxPlanePoint3;
            }
        }

        public override bool IntersectWithFloor(Pose atPose, float floorLevel, out Vector3 point, out Vector3 normal, out float shift)
        {
            normal = Vector3.up;
            
            var boxPos = atPose.Position;
            var boxRot = atPose.Rotation;
            var boxRight = boxRot.GetAxis0();
            var boxUp = boxRot.GetAxis1();
            var boxFwd = boxRot.GetAxis2();
                
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
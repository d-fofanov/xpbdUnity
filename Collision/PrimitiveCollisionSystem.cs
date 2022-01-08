using System.Collections.Generic;
using UnityEngine;

namespace xpbdUnity.Collision
{
    internal class PrimitiveCollisionSystem
    {
        internal readonly struct ContactInfo
        {
            public readonly Body Body0;
            public readonly Body Body1;
            public readonly Vector3 Point;
            public readonly Vector3 Normal;
            public readonly float Depth;
            public readonly Vector3 DeltaVDirection;
            public readonly float DeltaVMagnitude;
            public readonly float Friction;

            public ContactInfo(Body body0, Body body1, Vector3 point, Vector3 normal, float depth, Vector3 deltaVDirection, float deltaVMagnitude, float friction)
            {
                Body0 = body0;
                Body1 = body1;
                Point = point;
                Normal = normal;
                Depth = depth;
                DeltaVDirection = deltaVDirection;
                DeltaVMagnitude = deltaVMagnitude;
                Friction = friction;
            }
        }

        private float _globalFloorLevel = 0f;
        private FrictionProvider _frictionProvider;
        
        private List<Body> _bodies = new List<Body>();
        private List<ContactInfo> _contacts = new List<ContactInfo>();

        internal IReadOnlyList<ContactInfo> Contacts => _contacts;

        public PrimitiveCollisionSystem(float globalFloorLevel, FrictionProvider frictionProvider)
        {
            _globalFloorLevel = globalFloorLevel;
            _frictionProvider = frictionProvider;
        }

        internal void SetBodies(List<Body> bodies)
        {
            _bodies = bodies;
        }

        internal void ClearContacts()
        {
            _contacts.Clear();
        }

        internal void Collide(float dt)
        {
            for (int i=0; i<_bodies.Count; i++)
            {
                for (int j=i+1; j<_bodies.Count; j++)
                {
                    var b0 = _bodies[i];
                    var b1 = _bodies[j];

                    var p0 = b0.Pose;
                    var p1 = b1.Pose;
                    var c0 = b0.Collider;
                    var c1 = b1.Collider;

                    var bounds0 = new Bounds(p0.Position, b0.AABBExtents);
                    var bounds1 = new Bounds(p1.Position, b1.AABBExtents);
                    if (bounds0.Intersects(bounds1) &&
                        c0.Intersect(p0, c1, p1, out var point, out var normal, out var shift))
                    {
                        AddContact(b0, b1, point, normal, shift);
                        Body.ApplyBodyPairCorrection(b0, b1, normal * shift, 0f, dt, point, point);
                    }
                }
            }
            
            foreach (var body in _bodies)
            {
                var pose = body.Pose;
                var collider = body.Collider;

                var boundsMin = pose.Position - body.AABBExtents * 0.5f;
                if (boundsMin.y < _globalFloorLevel &&
                    collider.IntersectWithFloor(pose, _globalFloorLevel,
                        out var point, out var normal, out var shift))
                {
                    AddContact(body, null, point, normal, shift);
                    Body.ApplyBodyPairCorrection(body, null, normal * shift, 0f, dt, point);
                }
            }
        }

        private void AddContact(Body body0, Body body1, Vector3 point, Vector3 normal, float depth)
        {
            var pointVel0 = body0.GetVelocityAt(point);
            var pointVel1 = body1?.GetVelocityAt(point) ?? Vector3.zero;
            var proj0 = Vector3.Dot(pointVel0, normal);
            var proj1 = Vector3.Dot(pointVel1, normal);
            
            // simplified (pointVel0 - proj0 * normal) - (pointVel1 - proj1 * normal)
            // delta of velocities' parts tangential to normal
            var deltaV = pointVel0 - pointVel1 - (proj0 - proj1) * normal;
            var deltaVMagnitude = deltaV.magnitude;
            var deltaVDirection = Vector3.zero;
            if (deltaVMagnitude != 0f)
            {
                deltaVDirection = deltaV * (1f / deltaVMagnitude);
            }
            
            Debug.DrawLine(point, point + deltaV, Color.blue);
            var friction = _frictionProvider.CalculateFriction(body0, body1, point, normal, depth, deltaVDirection, deltaVMagnitude);
            _contacts.Add(new ContactInfo(body0, body1, point, normal, depth, deltaVDirection, deltaVMagnitude, friction));
        }
    }
}
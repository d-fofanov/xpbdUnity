using System.Collections.Generic;

namespace xpbdUnity.Collision
{
    internal class PrimitiveCollisionSystem
    {
        private float _globalFloorLevel = 0f;
        private List<Body> _bodies = new List<Body>();

        internal void SetBodies(List<Body> bodies)
        {
            _bodies = bodies;
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

                    if (c0.Intersect(p0, c1, p1, out var point, out var normal, out var shift))
                    {
                        Body.ApplyBodyPairCorrection(b0, b1, normal * shift, 0f, dt, point, point);
                    }
                }
            }
            
            foreach (var body in _bodies)
            {
                var pose = body.Pose;
                var collider = body.Collider;
                if (collider.IntersectWithFloor(pose, _globalFloorLevel,
                    out var point, out var normal, out var shift))
                {
                    Body.ApplyBodyPairCorrection(body, null, normal * shift, 0f, dt, point);
                }
            }
        }
    }
}
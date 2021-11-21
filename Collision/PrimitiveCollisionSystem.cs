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

            foreach (var b0 in _bodies)
            {
                foreach (var b1 in _bodies)
                {
                    if (b0 == b1)
                        continue;

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
        }
    }
}
using UnityEngine;

namespace xpbdUnity.Collision
{
    public class SphereCollider : BaseCollider
    {
        public float Radius => _radius;
        
        private float _radius;
        
        public SphereCollider(float radius, float mass)
        {
            _radius = radius;
            _invMass = 1f / mass;
            var invInertiaVal = 1f / (0.4f * mass * radius * radius);
            _invInertia = new Vector3(invInertiaVal, invInertiaVal, invInertiaVal);
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

        public override bool IntersectWithFloor(Pose atPose, float floorLevel, out Vector3 point, out Vector3 normal, out float shift)
        {
            var lowPos = atPose.Position;
            lowPos.y -= _radius;

            shift = floorLevel - lowPos.y;
            normal = Vector3.up;
            point = lowPos + normal * shift * 0.5f;

            return shift > 0f;
        }
        
        private bool IntersectWithSphere(Pose atPose, SphereCollider withSphere, Pose otherPose, out Vector3 point,
            out Vector3 normal, out float shift)
        {
            var dp = atPose.Position - otherPose.Position;
            var rSum = _radius + withSphere._radius;
            var distance = dp.magnitude;

            normal = distance > Mathf.Epsilon ? dp / distance : Vector3.up;
            shift = rSum - distance;
            point = atPose.Position + normal * (_radius - 0.5f * shift);

            if (shift > 0f)
            {
                Debug.Log($"Collision");
            }
            return shift > 0f;
        }

        private bool IntersectWithBox(Pose atPose, BoxCollider withBox, Pose otherPose, out Vector3 point,
            out Vector3 normal, out float shift)
        {
            var result = withBox.IntersectWithSphere(otherPose, this, atPose,
                out var boxPoint, out var boxNormal, out var boxShift);

            normal = -boxNormal;
            point = boxPoint;
            shift = boxShift;
            
            return result;
        }
    }
}
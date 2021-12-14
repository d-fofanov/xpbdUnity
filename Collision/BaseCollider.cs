using UnityEngine;

namespace xpbdUnity.Collision
{
    public abstract class BaseCollider
    {
        public enum Type
        {
            Box,
            Sphere
        }

        public Vector3 Drag => _drag;
        public float InvMass => _invMass;
        public Vector3 InvInertia => _invInertia;

        public abstract float Volume { get; }
        public abstract Vector3 AABBSize { get; }
        
        protected float _invMass;
        protected Vector3 _invInertia;
        protected Vector3 _drag;

        public abstract bool Intersect(Pose atPose, BaseCollider withCollider, Pose otherPose, out Vector3 point, out Vector3 normal, out float shift);
        public abstract bool IntersectWithFloor(Pose atPose, float floorLevel, out Vector3 point, out Vector3 normal, out float shift);
    }
}
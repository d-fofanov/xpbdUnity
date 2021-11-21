using UnityEngine;

namespace xpbdUnity
{
    public readonly struct Pose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public Pose(Vector3 pos, Quaternion rot)
        {
            Position = pos;
            Rotation = rot;
        }

        public Pose(Pose pose)
        {
            Position = pose.Position;
            Rotation = pose.Rotation;
        }

        public Vector3 Rotate(Vector3 v)
        {
            return Rotation * v;
        }
        
        public Vector3 InvRotate(Vector3 v)
        {
            return Quaternion.Inverse(Rotation) * v;
        }

        public Vector3 Transform(Vector3 v)
        {
            return Rotation * v + Position;
        }

        public Vector3 InvTransform(Vector3 v)
        {
            return InvRotate(v - Position);
        }

        public Pose TransformPose(in Pose pose)
        {
            return new Pose(Position + Rotate(pose.Position),
                Rotation * pose.Rotation);
        }
        
        // Mutations
        public Pose Translate(Vector3 deltaPos)
        {
            return new Pose(Position + deltaPos, Rotation);
        }
        
        public Pose SetRotation(Quaternion rot)
        {
            return new Pose(Position, rot);
        }
    }
}
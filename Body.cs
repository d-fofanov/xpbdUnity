using UnityEngine;

namespace xpbdUnity
{
    internal class Body
    {
        private const float MaxRotationPerSubstep = 0.5f;
        private Pose _pose;
        private Pose _prevPose;

        private Vector3 _vel;
        private Vector3 _omega;

        private float _invMass;
        private Vector3 _invInertia;

        public Pose Pose => _pose;
        public Vector3 Omega => _omega;

        public Body(in Pose pose)
        {
            _prevPose = _pose = pose;
            _omega = _vel = Vector3.zero;

            _invMass = 1f;
            _invInertia = Vector3.one;
        }

        // TODO move out into the collider
        public void SetBox(Vector3 size, float density = 1f)
        {
            var mass = size.x * size.y * size.z * density;
            _invMass = 1f / mass;
            mass /= 12f;
            _invInertia = new Vector3(
                1f / (size.y * size.y + size.z * size.z) / mass,
                1f / (size.z * size.z + size.x * size.x) / mass,
                1f / (size.x * size.x + size.y * size.y) / mass);
        }

        public void ApplyRotation(Vector3 rot, float scale = 1f)
        {

            // safety clamping. This happens very rarely if the solver
            // wants to turn the body by more than 30 degrees in the
            // orders of milliseconds

            var phi = rot.magnitude;
            if (phi * scale > MaxRotationPerSubstep)
                scale = MaxRotationPerSubstep / phi;

            var dq = new Quaternion(rot.x * scale, rot.y * scale, rot.z * scale, 0f);
            dq *= _pose.Rotation;

            var q = _pose.Rotation;
            var newRotation = new Quaternion(
                q.x + 0.5f * dq.x, q.y + 0.5f * dq.y, q.z + 0.5f * dq.z, q.w + 0.5f * dq.w
            );
            newRotation.Normalize();
            _pose = _pose.SetRotation(newRotation);
        }

        public void Integrate(float dt, Vector3 gravity)
        {
            _prevPose = _pose;
            _vel += gravity * dt;
            _pose = _pose.Translate(_vel * dt);
            ApplyRotation(_omega, dt);
        }

        public void Update(float dt)
        {
            _vel = _pose.Position - _prevPose.Position;
            _vel *= 1f / dt;

            var dq = _pose.Rotation * Quaternion.Inverse(_prevPose.Rotation);
            _omega = new Vector3(dq.x * 2f / dt, dq.y * 2f / dt, dq.z * 2f / dt);
            if (dq.w < 0f)
                _omega *= -1f;
        }

        public Vector3 GetVelocityAt(Vector3 pos)
        {
            return _vel - Vector3.Cross(pos - _pose.Position, _omega);
        }

        public float GetInverseMass(Vector3 normal, Vector3? pos)
        {
            var n = pos == null ? normal : Vector3.Cross(pos.Value - _pose.Position, normal);

            n = _pose.InvRotate(n);

            var w =
                n.x * n.x * _invInertia.x +
                n.y * n.y * _invInertia.y +
                n.z * n.z * _invInertia.z;

            if (pos != null)
                w += _invMass;

            return w;
        }

        public void ApplyCorrection(Vector3 corr, Vector3? pos = null, bool velocityLevel = false)
        {
            var dq = Vector3.zero;
            if (pos == null)
            {
                dq = corr;
            }
            else
            {
                if (velocityLevel)
                    _vel += corr * _invMass;
                else
                    _pose = _pose.Translate(corr * _invMass);

                dq = Vector3.Cross(pos.Value - _pose.Position, corr);
            }

            dq = _pose.InvRotate(dq);
            dq.Scale(_invInertia);

            dq = _pose.Rotate(dq);

            if (velocityLevel)
                _omega += dq;
            else
                ApplyRotation(dq);
        }

        public static void ApplyBodyPairCorrection(Body body0, Body body1, Vector3 corr, float compliance, float dt,
            Vector3? pos0 = null, Vector3? pos1 = null,
            bool velocityLevel = false)
        {
            var corrMagnitude = corr.magnitude;
            if (corrMagnitude == 0f)
                return;

            var normal = corr / corrMagnitude;

            var w0 = body0?.GetInverseMass(normal, pos0) ?? 0f;
            var w1 = body1?.GetInverseMass(normal, pos1) ?? 0f;

            var w = w0 + w1;
            if (w == 0f)
                return;

            var lambda = -corrMagnitude / (w + compliance / dt / dt);
            normal *= -lambda;

            if (body0 != null)
                body0.ApplyCorrection(normal, pos0, velocityLevel);
            if (body1 != null)
            {
                normal *= -1f;
                body1.ApplyCorrection(normal, pos1, velocityLevel);
            }
        }

        public static void LimitAngle(Body body0, Body body1, Vector3 n, Vector3 a, Vector3 b, float minAngle,
            float maxAngle, float compliance, float dt, float maxCorr = Mathf.PI)
        {
            // the key function to handle all angular joint limits
            var c = Vector3.Cross(a, b);

            var phi = Mathf.Asin(Vector3.Dot(c, n));
            if (Vector3.Dot(a, b) < 0f)
                phi = Mathf.PI - phi;

            if (phi > Mathf.PI)
                phi -= 2f * Mathf.PI;
            if (phi < -Mathf.PI)
                phi += 2f * Mathf.PI;

            if (phi < minAngle || phi > maxAngle)
            {
                phi = Mathf.Min(Mathf.Max(minAngle, phi), maxAngle);

                var q = Quaternion.AngleAxis(phi, n);

                var omega = Vector3.Cross(q * a, b);

                phi = omega.magnitude;
                if (phi > maxCorr)
                    omega *= maxCorr / phi;

                ApplyBodyPairCorrection(body0, body1, omega, compliance, dt);
            }
        }
    }
}
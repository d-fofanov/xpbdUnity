using UnityEngine;

namespace xpbdUnity
{
    internal class XJoint
    {
        private Body _body0;
        private Body _body1;
        private Pose _localPose0;
        private Pose _localPose1;
        private Pose _globalPose0;
        private Pose _globalPose1;

        private JointParams _params;

        public XJoint(Body body0, Body body1, JointParams parameters)
        {
            _body0 = body0;
            _body1 = body1;
            _params = parameters;

            _localPose0 = new Pose(_params.localPos0, _params.localRot0);
            _localPose1 = new Pose(_params.localPos1, _params.localRot1);

            _globalPose0 = _localPose0;
            _globalPose1 = _localPose1;
        }

        private void UpdateGlobalPoses()
        {
            _globalPose0 = _localPose0;
            if (_body0 != null)
                _globalPose0 = _body0.Pose.TransformPose(_globalPose0);

            _globalPose1 = _localPose1;
            if (_body1 != null)
                _globalPose1 = _body1.Pose.TransformPose(_globalPose1);
        }

        public void SolvePos(float dt)
        {

            UpdateGlobalPoses();

            // orientation

            if (_params.type == JointType.Fixed)
            {
                var q = _globalPose1.Rotation * Quaternion.Inverse(_globalPose0.Rotation);
                var omega = new Vector3(2f * q.x, 2f * q.y, 2f * q.z);
                if (q.w < 0f)
                    omega *= -1f;
                Body.ApplyBodyPairCorrection(_body0, _body1, omega, _params.compliance, dt);
            }

            if (_params.type == JointType.Hinge)
            {

                // align axes
                var a0 = _globalPose0.Rotation.GetAxis0();
                var a1 = _globalPose1.Rotation.GetAxis0();
                Body.ApplyBodyPairCorrection(_body0, _body1, Vector3.Cross(a0, a1), 0f, dt);

                // limits
                if (_params.hasSwingLimits)
                {
                    UpdateGlobalPoses();
                    var n = _globalPose0.Rotation.GetAxis0();
                    var b0 = _globalPose0.Rotation.GetAxis1();
                    var b1 = _globalPose1.Rotation.GetAxis1();
                    Body.LimitAngle(_body0, _body1, n, b0, b1,
                        _params.minSwingAngle, _params.maxSwingAngle, _params.swingLimitsCompliance, dt);
                }
            }

            if (_params.type == JointType.Spherical)
            {

                // swing limits
                if (_params.hasSwingLimits)
                {
                    UpdateGlobalPoses();
                    var a0 = _globalPose0.Rotation.GetAxis0();
                    var a1 = _globalPose1.Rotation.GetAxis0();
                    var n = Vector3.Cross(a0, a1).normalized;
                    Body.LimitAngle(_body0, _body1, n, a0, a1,
                        _params.minSwingAngle, _params.maxSwingAngle, _params.swingLimitsCompliance, dt);
                }

                // twist limits
                if (_params.hasTwistLimits)
                {
                    UpdateGlobalPoses();
                    var n0 = _globalPose0.Rotation.GetAxis0();
                    var n1 = _globalPose1.Rotation.GetAxis0();

                    var n = (n0 + n1).normalized;
                    var a0 = _globalPose0.Rotation.GetAxis1();
                    a0 = (a0 - n * Vector3.Dot(n, a0)).normalized;

                    var a1 = _globalPose1.Rotation.GetAxis1();
                    a1 = (a1 - n * Vector3.Dot(n, a1)).normalized;

                    // handling gimbal lock problem
                    var maxCorr = Vector3.Dot(n0, n1) > -0.5f ? 2f * Mathf.PI : dt;

                    Body.LimitAngle(_body0, _body1, n, a0, a1,
                        _params.minTwistAngle, _params.maxTwistAngle, _params.twistLimitCompliance, dt, maxCorr);
                }
            }

            // position

            // simple attachment

            UpdateGlobalPoses();
            Vector3 corr = Vector3.zero;
            if (_params.type == JointType.Distance)
            {
                var dp = _globalPose1.Position - _globalPose0.Position;
                var distance = dp.magnitude;
                if (distance > _params.distance)
                {
                    corr = dp / distance * (distance - _params.distance);
                }
            }
            else
            {
                corr = _globalPose1.Position - _globalPose0.Position;
            }
            Body.ApplyBodyPairCorrection(_body0, _body1, corr, _params.compliance, dt,
                _globalPose0.Position, _globalPose1.Position);
        }

        public void SolveVel(float dt)
        {

            // Gauss-Seidel vars us make damping unconditionally stable in a 
            // very simple way. We clamp the correction for each constraint
            // to the magnitude of the currect velocity making sure that
            // we never subtract more than there actually is.

            if (_params.rotDamping > 0f)
            {
                var omega = Vector3.zero;
                if (_body0 != null)
                    omega -= _body0.Omega;
                if (_body1 != null)
                    omega += _body1.Omega;

                omega *= Mathf.Min(1f, _params.rotDamping * dt);
                Body.ApplyBodyPairCorrection(_body0, _body1, omega, 0f, dt,
                    null, null, true);
            }

            if (_params.posDamping > 0f)
            {
                UpdateGlobalPoses();
                var vel = Vector3.zero;

                if (_body0 != null)
                    vel -= _body0.GetVelocityAt(_globalPose0.Position);
                if (_body1 != null)
                    vel += _body1.GetVelocityAt(_globalPose1.Position);

                vel *= Mathf.Min(1f, _params.posDamping * dt);
                Body.ApplyBodyPairCorrection(_body0, _body1, vel, 0f, dt,
                    _globalPose0.Position, _globalPose1.Position, true);
            }
        }
    }
}
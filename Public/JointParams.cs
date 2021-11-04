using System;
using UnityEngine;

namespace xpbdUnity
{
    [Serializable]
    public class JointParams
    {
        public Vector3 localPos0;
        public Quaternion localRot0;
        public Vector3 localPos1;
        public Quaternion localRot1;

        public JointType type;					
        public float compliance = 0f;
        public float rotDamping = 0f;
        public float posDamping = 0f;
        public bool hasSwingLimits = false;
        public float minSwingAngle = -2f * Mathf.PI;
        public float maxSwingAngle = 2f * Mathf.PI;
        public float swingLimitsCompliance = 0f;
        public bool hasTwistLimits = false;
        public float minTwistAngle = -2f * Mathf.PI;
        public float maxTwistAngle = 2f * Mathf.PI;
        public float twistLimitCompliance = 0f;
    }
}
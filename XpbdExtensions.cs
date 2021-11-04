using UnityEngine;

namespace xpbdUnity
{
    public static class XpbdExtensions
    {
        public static Vector3 GetAxis0(this Quaternion q)
        {
            var x2 = q.x * 2f;
            var w2 = q.w * 2f;
            return new Vector3(q.w * w2 - 1f + q.x * x2, q.z * w2 + q.y * x2, -q.y * w2 + q.z * x2);
        }

        public static Vector3 GetAxis1(this Quaternion q)
        {
            var y2 = q.y * 2f;
            var w2 = q.w * 2f;
            return new Vector3(-q.z * w2 + q.x * y2, q.w * w2 - 1f + q.y * y2, q.x * w2 + q.z * y2);
        }

        public static Vector3 GetAxis2(this Quaternion q)
        {
            var z2 = q.z * 2f;
            var w2 = q.w * 2f;
            return new Vector3(q.y * w2 + q.x * z2, -q.x * w2 + q.y * z2, q.w * w2 - 1f + q.z * z2);
        }
    }
}
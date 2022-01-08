using UnityEngine;

namespace xpbdUnity.Collision
{
    internal class FrictionProvider
    {
        internal Vector3 CalculateFriction(Body body0, Body body1, Vector3 point, Vector3 normal, float depth,
            Vector3 deltaV)
        {
            // TODO
            return -deltaV.normalized * 100f;
        }
    }
}
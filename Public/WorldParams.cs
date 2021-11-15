using System;
using UnityEngine;

namespace xpbdUnity
{
    [Serializable]
    public class WorldParams
    {
        public float timeStep = 0.02f;
        public float numSubsteps = 5;
        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    }
}
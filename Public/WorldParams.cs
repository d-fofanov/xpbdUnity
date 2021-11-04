using System;
using UnityEngine;

namespace xpbdUnity
{
    [Serializable]
    public class WorldParams
    {
        public float timeStep;
        public float numSubsteps;
        public Vector3 gravity;
    }
}
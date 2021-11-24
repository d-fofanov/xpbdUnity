using System;
using UnityEngine;

namespace xpbdUnity
{
    [DefaultExecutionOrder(-999)]
    public class XPBDHost : MonoBehaviour
    {
        [SerializeField] private WorldParams _params;

        private void Awake()
        {
            XPBDSingleWorld.Instance.Initialize(_params);
        }

        private void FixedUpdate()
        {
            XPBDSingleWorld.Instance.Simulate();
        }
    }
}
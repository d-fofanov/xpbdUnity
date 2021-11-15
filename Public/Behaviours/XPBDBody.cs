using UnityEngine;

namespace xpbdUnity
{
    public class XPBDBody : MonoBehaviour
    {
        [SerializeField] private float _mass = 1f;
        [SerializeField] private Vector3 _boxSize = Vector3.one;

        private Transform _transform;
        internal Body _body;

        private void Start()
        {
            _transform = transform;
            _body = new Body(new Pose(_transform.position, _transform.rotation), _boxSize, _mass);
            XPBDSingleWorld.Instance.AddBody(_body);
        }

        private void Update()
        {
            var pose = _body.Pose;
            _transform.position = pose.Position;
            _transform.rotation = pose.Rotation;
        }
    }
}
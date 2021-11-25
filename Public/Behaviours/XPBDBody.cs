using UnityEngine;
using xpbdUnity.Collision;

namespace xpbdUnity
{
    public class XPBDBody : MonoBehaviour
    {
        [SerializeField] private float _mass = 1f;
        [SerializeField] private Vector3 _drag = Vector3.zero;
        [SerializeField] private BaseCollider.Type _colliderType;
        [Tooltip("Full size for a box collider, radius in X component for sphere collider")]
        [SerializeField] private Vector3 _size = Vector3.one;

        public BaseCollider Collider => _body?.Collider;

        private Transform _transform;
        internal Body _body;

        public void AddForce(Vector3 force)
        {
            XPBDSingleWorld.Instance.AddForce(_body, force);
        }

        private void Start()
        {
            _transform = transform;
            var collider = _colliderType == BaseCollider.Type.Box
                ? (BaseCollider) new Collision.BoxCollider(_size, _mass, _drag)
                : (BaseCollider) new Collision.SphereCollider(_size.x, _mass, _drag);
            
            _body = new Body(new Pose(_transform.position, _transform.rotation), collider);
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
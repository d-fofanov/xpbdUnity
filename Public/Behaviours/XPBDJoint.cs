using UnityEngine;

namespace xpbdUnity
{
    [DefaultExecutionOrder(1)]
    public class XPBDJoint : MonoBehaviour
    {
        [SerializeField] private XPBDBody _body0;
        [SerializeField] private XPBDBody _body1;
        [SerializeField] private JointParams _params;

        private XJoint _joint;

        private void Start()
        {
            var b0 = _body0 == null ? null : _body0._body;
            var b1 = _body1 == null ? null : _body1._body;
            _joint = new XJoint(b0, b1, _params);
            
            XPBDSingleWorld.Instance.AddJoint(_joint);
        }
    }
}
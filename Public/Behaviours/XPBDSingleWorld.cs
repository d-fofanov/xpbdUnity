namespace xpbdUnity
{
    public class XPBDSingleWorld
    {
        public static XPBDSingleWorld Instance
        {
            get
            {
                if (_instance == null)
                    _instance = new XPBDSingleWorld();

                return _instance;
            }
        }

        private static XPBDSingleWorld _instance = null;

        private World _world = null;

        public void Initialize(WorldParams parameters)
        {
            _world = new World(parameters);
        }
        
        internal void AddBody(Body body)
        {
            _world.AddBody(body);
        }

        internal void AddJoint(XJoint joint)
        {
            _world.AddJoint(joint);
        }

        public void Simulate()
        {
            _world.Simulate();
        }
    }
}
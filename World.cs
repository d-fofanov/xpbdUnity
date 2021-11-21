using System.Collections.Generic;
using xpbdUnity.Collision;

namespace xpbdUnity
{
    internal class World
    {
        private WorldParams _params;
        private List<Body> _bodies = new List<Body>();
        private List<XJoint> _joints = new List<XJoint>();
        private PrimitiveCollisionSystem _collisionSystem = new PrimitiveCollisionSystem();

        public World(WorldParams @params)
        {
            _params = @params;
            _collisionSystem.SetBodies(_bodies);
        }

        internal void AddBody(Body body)
        {
            if (!_bodies.Contains(body))
                _bodies.Add(body);
        }

        internal void AddJoint(XJoint joint)
        {
            if (!_joints.Contains(joint))
                _joints.Add(joint);
        }

        internal void Simulate()
        {
            var dt = _params.timeStep / _params.numSubsteps;

            for (var i = 0; i < _params.numSubsteps; i++)
            {
                for (var j = 0; j < _bodies.Count; j++)
                    _bodies[j].Integrate(dt, _params.gravity);
                
                _collisionSystem.Collide(dt);

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolvePos(dt);
                
                _collisionSystem.Collide(dt);

                for (var j = 0; j < _bodies.Count; j++)
                    _bodies[j].Update(dt);

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolveVel(dt);
            }
        }
    }
}
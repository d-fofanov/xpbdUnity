using System.Collections.Generic;

namespace xpbdUnity
{
    internal class World
    {
        private WorldParams _params;
        private List<Body> _bodies = new List<Body>();
        private List<XJoint> _joints = new List<XJoint>();

        public World(WorldParams @params)
        {
            _params = @params;
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

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolvePos(dt);

                for (var j = 0; j < _bodies.Count; j++)
                    _bodies[j].Update(dt);

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolveVel(dt);
            }
        }
    }
}
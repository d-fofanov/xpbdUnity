using System.Collections.Generic;
using UnityEngine;
using xpbdUnity.Collision;

namespace xpbdUnity
{
    internal class World
    {
        private WorldParams _params;
        private List<Body> _bodies = new List<Body>();
        private List<Vector3> _forces = new List<Vector3>();
        private List<XJoint> _joints = new List<XJoint>();
        private PrimitiveCollisionSystem _collisionSystem;

        public World(WorldParams @params)
        {
            _params = @params;
            _collisionSystem = new PrimitiveCollisionSystem(_params.floorLevel, new FrictionProvider());
            _collisionSystem.SetBodies(_bodies);
        }

        internal void AddBody(Body body)
        {
            if (_bodies.Contains(body))
                return;
            
            _bodies.Add(body);
            _forces.Add(Vector3.zero);
        }

        internal void AddJoint(XJoint joint)
        {
            if (!_joints.Contains(joint))
                _joints.Add(joint);
        }

        internal void AddForce(Body body, Vector3 force)
        {
            // TODO remove linear search
            var index = _bodies.IndexOf(body);
            if (index == -1)
                return;

            _forces[index] += force;
        }

        internal void Simulate()
        {
            var dt = _params.timeStep / _params.numSubsteps;

            for (var i = 0; i < _params.numSubsteps; i++)
            {
                for (var j = 0; j < _bodies.Count; j++)
                {
                    _bodies[j].OnNextTick();
                }

                var contacts = _collisionSystem.Contacts;
                for (int j = 0; j < contacts.Count; j++)
                {
                    var contact = contacts[j];
                    contact.Body0.ApplyFrictionForce(dt, contact.Friction, contact.Normal, contact.Point, contact.DeltaV);
                    // TODO get actually applied from prev call and pass it into the second
                    contact.Body1?.ApplyFrictionForce(dt, -contact.Friction, -contact.Normal, contact.Point, -contact.DeltaV);
                }
                _collisionSystem.ClearContacts();
                
                for (var j = 0; j < _bodies.Count; j++)
                {
                    _bodies[j].ApplyGravity(dt, _params.gravity);
                    _bodies[j].ApplyForce(dt, _forces[j]);
                    _bodies[j].ApplyDrag(dt);
                    _bodies[j].Integrate(dt);
                }

                _collisionSystem.Collide(dt);

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolvePos(dt);

                _collisionSystem.Collide(dt);

                for (var j = 0; j < _bodies.Count; j++)
                    _bodies[j].Update(dt);

                for (var j = 0; j < _joints.Count; j++)
                    _joints[j].SolveVel(dt);
                
            }

            for (var j = 0; j < _bodies.Count; j++)
            {
                _forces[j] = Vector3.zero;
            }
        }
    }
}
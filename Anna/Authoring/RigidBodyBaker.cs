using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Psyshock.Anna.Authoring
{
    [DisableAutoCreation]
    public class RigidBodyBaker : Baker<UnityEngine.Rigidbody>
    {
        public override void Bake(UnityEngine.Rigidbody authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new RigidBody
            {
                inverseMass              = 1f / authoring.mass,
                velocity                 = new UnitySim.Velocity { linear = authoring.linearVelocity, angular = authoring.angularVelocity },
                coefficientOfFriction    = (half)0.3f,
                coefficientOfRestitution = (half)0.3f
            });
            AddBuffer<AddImpulse>(entity);
            if (authoring.isKinematic)
                AddComponent<KinematicCollisionTag>(entity);
        }
    }
}


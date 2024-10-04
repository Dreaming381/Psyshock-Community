using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.Psyshock.Anna
{
    public struct RigidBody : IComponentData
    {
        public UnitySim.Velocity velocity;
        public float             inverseMass;
        public half              coefficientOfFriction;
        public half              coefficientOfRestitution;
    }

    [InternalBufferCapacity(0)]
    public struct AddImpulse : IBufferElementData
    {
        internal float3 pointOrField;
        internal float3 pointImpulseOrZero;

        public AddImpulse(float3 fieldImpulse)
        {
            pointOrField       = fieldImpulse;
            pointImpulseOrZero = float3.zero;
        }

        public AddImpulse(float3 worldPoint, float3 impulse)
        {
            pointOrField       = worldPoint;
            pointImpulseOrZero = impulse;
        }
    }

    public partial struct RigidBodyCollisionLayer : ICollectionComponent
    {
        public CollisionLayer layer;

        public JobHandle TryDispose(JobHandle inputDeps) => inputDeps;  // Uses WorldUpdateAllocator
    }
}


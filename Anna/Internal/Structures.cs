using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Psyshock.Anna
{
    internal struct CapturedRigidBodyState
    {
        public UnitySim.Velocity         velocity;
        public UnitySim.MotionExpansion  motionExpansion;
        public RigidTransform            inertialPoseWorldTransform;
        public UnitySim.Mass             mass;
        public UnitySim.MotionStabilizer motionStabilizer;
        public float3                    gravity;
        public float                     angularExpansion;
        public int                       bucketIndex;
        public int                       numOtherSignificantBodiesInContact;
        public half                      coefficientOfFriction;
        public half                      coefficientOfRestitution;
        public half                      linearDamping;
        public half                      angularDamping;
    }

    struct SolveByteCodes
    {
        public const byte contactEnvironment = 0;
        public const byte contactKinematic   = 1;
        public const byte contactBody        = 2;
    }

    struct ContactStreamData
    {
        public int                                                   indexA;
        public int                                                   indexB;
        public UnitySim.ContactJacobianBodyParameters                bodyParameters;
        public StreamSpan<UnitySim.ContactJacobianContactParameters> contactParameters;
        public StreamSpan<float>                                     contactImpulses;
    }
}


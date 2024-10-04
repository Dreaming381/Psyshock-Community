using System.Collections.Generic;
using Latios.Psyshock.Authoring;
using Latios.Transforms.Authoring;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Psyshock.Anna.Authoring
{
    public class CollisionTagAuthoring : MonoBehaviour
    {
        public enum Mode
        {
            IncludeEnvironmentRecursively,
            IncludeKinematicRecursively,
            ExcludeRecursively,
            IncludeEnvironmentSelfOnly,
            IncludeKinematicSelfOnly,
            ExcludeSelfOnly,
        }

        public Mode mode;
    }

    [BakeDerivedTypes]
    public class CollisionTagAuthoringBaker : Baker<UnityEngine.Collider>
    {
        static List<UnityEngine.Collider> s_colliderCache = new List<UnityEngine.Collider>();
        static List<ColliderAuthoring>    s_compoundCache = new List<ColliderAuthoring>();

        [BakingType]
        struct RequestPrevious : IRequestPreviousTransform { }

        public override void Bake(UnityEngine.Collider authoring)
        {
            if (!ShouldBake(authoring))
                return;

            var  search        = authoring.gameObject;
            bool isEnvironment = false;
            bool isKinematic   = false;
            while (search != null)
            {
                var tag = GetComponentInParent<CollisionTagAuthoring>(search);
                if (tag == null)
                    break;

                if (tag.mode == CollisionTagAuthoring.Mode.IncludeEnvironmentSelfOnly)
                {
                    if (search == authoring.gameObject)
                    {
                        isEnvironment = true;
                        break;
                    }
                }
                else if (tag.mode == CollisionTagAuthoring.Mode.IncludeKinematicSelfOnly)
                {
                    if (search == authoring.gameObject)
                    {
                        isKinematic = true;
                        break;
                    }
                }
                else if (tag.mode == CollisionTagAuthoring.Mode.ExcludeSelfOnly)
                {
                    if (search != authoring.gameObject)
                    {
                        break;
                    }
                }
                else if (tag.mode == CollisionTagAuthoring.Mode.IncludeEnvironmentRecursively)
                {
                    isEnvironment = true;
                    break;
                }
                else if (tag.mode == CollisionTagAuthoring.Mode.IncludeKinematicRecursively)
                {
                    isKinematic = true;
                    break;
                }

                search = GetParent(search);
            }

            if (isEnvironment)
            {
                var entity = GetEntity(TransformUsageFlags.Renderable);
                AddComponent<EnvironmentCollisionTag>(entity);
            }
            else if (isKinematic)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent<KinematicCollisionTag>(entity);
                AddComponent<RequestPrevious>(      entity);
            }
        }

        // Todo: Expose as utility in Psyshock?
        bool ShouldBake(UnityEngine.Collider authoring)
        {
            if (!authoring.enabled)
                return false;

            s_colliderCache.Clear();
            GetComponents(s_colliderCache);
            if (s_colliderCache.Count > 1)
            {
                int enabledCount = 0;
                foreach (var c in s_colliderCache)
                    enabledCount += c.enabled ? 1 : 0;
                if (enabledCount > 1)
                    return false;
            }
            s_compoundCache.Clear();
            GetComponentsInParent(s_compoundCache);
            foreach (var compoundAuthoring in s_compoundCache)
            {
                if (compoundAuthoring.colliderType == AuthoringColliderTypes.None)
                    continue;
                if (!compoundAuthoring.enabled)
                    continue;
                if (compoundAuthoring.generateFromChildren)
                    return false;

                foreach (var child in compoundAuthoring.colliders)
                {
                    if (child.gameObject == authoring.gameObject)
                        return false;
                }
            }

            return true;
        }
    }
}


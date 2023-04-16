using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Xedrial.Physics.Math;

namespace Xedrial.Physics
{
    public static class PhysicsUtility
    {
        public static void AddBody(EntityManager entityManager, BodyDef bodyDef)
        {
            Debug.Assert(bodyDef.Entity != Entity.Null, "No Entity has been provided in the Body Definition.");
            
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsVelocity2D());
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsBody2D { Type = bodyDef.Type });
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsShape2D { Shape = bodyDef.Shape});
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsGravityFactor2D { Value = 1f });
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsTransformComponent
            {
                Value = new PhysicsTransform(bodyDef.Translation, bodyDef.Orientation)
            });
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsMass2D
            {
                InverseMass = bodyDef.Mass != 0f ? 1f / bodyDef.Mass : 0f,
                InverseInertia = bodyDef.Inertia != 0f ? 1f / bodyDef.Inertia : 0f
            });
            entityManager.AddComponentData(bodyDef.Entity, new PhysicsMaterial2D
            {
                Density = bodyDef.Density,
                Restitution = bodyDef.Restitution
            });
        }

        public static void AddBody(EntityCommandBuffer ecb, BodyDef bodyDef)
        {
            Debug.Assert(bodyDef.Entity != Entity.Null, "No Entity has been provided in the Body Definition.");
            
            ecb.AddComponent(bodyDef.Entity, new PhysicsVelocity2D());
            ecb.AddComponent(bodyDef.Entity, new PhysicsBody2D { Type = bodyDef.Type });
            ecb.AddComponent(bodyDef.Entity, new PhysicsShape2D { Shape = bodyDef.Shape});
            ecb.AddComponent(bodyDef.Entity, new PhysicsGravityFactor2D { Value = 1f });
            ecb.AddComponent(bodyDef.Entity, new PhysicsTransformComponent
            {
                Value = new PhysicsTransform(bodyDef.Translation, bodyDef.Orientation)
            });
            ecb.AddComponent(bodyDef.Entity, new PhysicsMass2D
            {
                InverseMass = bodyDef.Mass != 0f ? 1f / bodyDef.Mass : 0f,
                InverseInertia = bodyDef.Inertia != 0f ? 1f / bodyDef.Inertia : 0f
            });
            ecb.AddComponent(bodyDef.Entity, new PhysicsMaterial2D
            {
                Density = bodyDef.Density,
                Restitution = bodyDef.Restitution
            });
        }

        public static void ApplyImpulse(Entity entity, EntityManager entityManager, in float2 impulse,
            in float2 contactVector)
        {
            var pv = entityManager.GetComponentData<PhysicsVelocity2D>(entity);
            var pm = entityManager.GetComponentData<PhysicsMass2D>(entity);
            
            pv.Linear += pm.InverseMass * impulse;
            pv.Angular += pm.InverseInertia * PhysicsMath.cross(contactVector, impulse);
            
            entityManager.SetComponentData(entity, pv);
        }

        public static void ApplyImpulse(Entity entity, PhysicsMass2D pm, PhysicsVelocity2D pv, EntityCommandBuffer.ParallelWriter ecb,
            int index,in float2 impulse, in float2 contactVector)
        {
            pv.Linear += pm.InverseMass * impulse;
            pv.Angular += pm.InverseInertia * PhysicsMath.cross(contactVector, impulse);
            
            ecb.SetComponent(index, entity, pv);
        }

        public static void ApplyImpulse(
            Entity entity,
            EntityManager entityManager,
            EntityCommandBuffer.ParallelWriter ecb,
            int index,
            in float2 impulse,
            in float2 contactVector
            )
        {
            var pv = entityManager.GetComponentData<PhysicsVelocity2D>(entity);
            var pm = entityManager.GetComponentData<PhysicsMass2D>(entity);
            
            pv.Linear += pm.InverseMass * impulse;
            pv.Angular += pm.InverseInertia * PhysicsMath.cross(contactVector, impulse);
            
            ecb.SetComponent(index, entity, pv);
        }
    }
}
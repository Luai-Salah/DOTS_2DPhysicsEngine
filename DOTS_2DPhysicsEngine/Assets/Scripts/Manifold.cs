using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

using Xedrial.Physics.Math;

namespace Xedrial.Physics
{
    public struct Manifold
    {
        public int IndexA;
        public int IndexB;

        public int ContactCount;
        
        public float Penetration;
        public float2 Normal;
        
        public float2 Contact_1;
        public float2 Contact_2;

        public float DeltaTime;
        public float Gravity;

        public float MixedRestitution;
        public float MixedDynamicFriction;
        public float MixedStaticFriction;

        public void Solve(NativeArray<RigidBody2D> rbs)
        {
            switch (rbs[IndexA].Shape)
            {
                case Shape.Polygon:
                    break;
                case Shape.Circle:
                    var shapeA = new Circle
                    {
                        Position = rbs[IndexA].Transform.Translation,
                        Radius = 1f
                    };

                    if (rbs[IndexB].Shape == Shape.Circle)
                    {
                        var shapeB = new Circle
                        {
                            Position = rbs[IndexB].Transform.Translation,
                            Radius = 1f
                        };
                        
                        Collision.CircleToCircle(ref this, shapeA, shapeB);
                    }
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public void Initialize(NativeArray<RigidBody2D> rbs)
        {
            RigidBody2D a = rbs[IndexA];
            RigidBody2D b = rbs[IndexB];
            
            // Calculate average restitution
            MixedRestitution = math.min(a.Restitution, b.Restitution);

            // Calculate static and dynamic friction
            MixedStaticFriction = math.sqrt(a.StaticFriction * b.StaticFriction);
            MixedDynamicFriction = math.sqrt(a.DynamicFriction * b.DynamicFriction);

            for(int i = 0; i < ContactCount; ++i)
            {
                float2 contactPoint = i == 0 ? Contact_1 : Contact_2;
                
                // Calculate radii from COM to contact
                float2 ra = contactPoint - a.Transform.Translation;
                float2 rb = contactPoint - b.Transform.Translation;

                float2 rv = b.LinearVelocity + PhysicsMath.cross(b.AngularVelocity, rb) -
                            a.LinearVelocity - PhysicsMath.cross(a.AngularVelocity, ra);


                // Determine if we should perform a resting collision or not
                // The idea is if the only thing moving this object is gravity,
                // then the collision should be performed without any restitution
                if (math.lengthsq(rv) < math.lengthsq(DeltaTime * Gravity) + math.EPSILON)
                    MixedRestitution = 0.0f;
            }
        }

        public void ApplyImpulse(int index, NativeArray<RigidBody2D> rbs, EntityCommandBuffer.ParallelWriter entityCommandBuffer)
        {
            RigidBody2D a = rbs[IndexA];
            RigidBody2D b = rbs[IndexB];
            
            // Early out and positional correct if both objects have infinite mass
            if (PhysicsMath.Equal(a.InverseMass + b.InverseMass, 0))
            {
                InfiniteMassCorrection(new PhysicsVelocity2D
                {
                    Angular = a.AngularVelocity,
                    Linear = a.LinearVelocity
                }, new PhysicsVelocity2D
                {
                    Angular = b.AngularVelocity,
                    Linear = b.LinearVelocity
                }, index, entityCommandBuffer);
                return;
            }

            for (int i = 0; i < ContactCount; i++)
            {
                float2 contactPoint = i == 0 ? Contact_1 : Contact_2;
                
                // Calculate radii from COM to contact
                float2 ra = contactPoint - a.Transform.Translation;
                float2 rb = contactPoint - b.Transform.Translation;

                // Relative velocity
                float2 rv = b.LinearVelocity + PhysicsMath.cross(b.AngularVelocity, rb) -
                            a.LinearVelocity - PhysicsMath.cross(a.AngularVelocity, ra);

                // Relative velocity along the normal
                float contactVel = math.dot(rv, Normal);

                // Do not resolve if velocities are separating
                if(contactVel > 0)
                    return;

                float raCrossN = PhysicsMath.cross(ra, Normal);
                float rbCrossN = PhysicsMath.cross(rb, Normal);
                float invMassSum = a.InverseMass + b.InverseMass + raCrossN * raCrossN * a.InverseInertia 
                                   + rbCrossN * rbCrossN * b.InverseInertia;

                // Calculate impulse scalar
                float j = -(1.0f + MixedRestitution) * contactVel;
                j /= invMassSum;
                j /= ContactCount;

                // Apply impulse
                float2 impulse = Normal * j;
                PhysicsUtility.ApplyImpulse(a.Entity, new PhysicsMass2D
                {
                    InverseInertia = a.InverseInertia,
                    InverseMass = a.InverseMass
                }, new PhysicsVelocity2D
                {
                    Angular = a.AngularVelocity,
                    Linear = a.LinearVelocity
                }, entityCommandBuffer, index, -impulse, ra);
                
                PhysicsUtility.ApplyImpulse(b.Entity, new PhysicsMass2D
                {
                    InverseInertia = b.InverseInertia,
                    InverseMass = b.InverseMass
                }, new PhysicsVelocity2D
                {
                    Angular = b.AngularVelocity,
                    Linear = b.LinearVelocity
                }, entityCommandBuffer, index, impulse, rb);

                // Friction impulse
                rv = b.LinearVelocity + PhysicsMath.cross(b.AngularVelocity, rb) -
                     a.LinearVelocity - PhysicsMath.cross(a.AngularVelocity, ra);

                float2 t = rv - Normal * math.dot(rv, Normal);
                math.normalize(t);

                // j tangent magnitude
                float jt = -math.dot(rv, t);
                jt /= invMassSum;
                jt /= ContactCount;

                // Don't apply tiny friction impulses
                if(PhysicsMath.Equal(jt, 0.0f))
                    return;

                // Coulumb's law
                float2 tangentImpulse;
                if(math.abs( jt ) < j * MixedStaticFriction)
                    tangentImpulse = t * jt;
                else
                    tangentImpulse = t * -j * MixedDynamicFriction;

                // Apply friction impulse
                PhysicsUtility.ApplyImpulse(a.Entity, new PhysicsMass2D
                {
                    InverseInertia = a.InverseInertia,
                    InverseMass = a.InverseMass
                }, new PhysicsVelocity2D
                {
                    Angular = a.AngularVelocity,
                    Linear = a.LinearVelocity
                }, entityCommandBuffer, index, -tangentImpulse, ra);
                
                PhysicsUtility.ApplyImpulse(b.Entity, new PhysicsMass2D
                {
                    InverseInertia = b.InverseInertia,
                    InverseMass = b.InverseMass
                }, new PhysicsVelocity2D
                {
                    Angular = b.AngularVelocity,
                    Linear = b.LinearVelocity
                }, entityCommandBuffer, index, tangentImpulse, rb);
            }
        }

        public void PositionalCorrection(EntityManager em, int index, EntityCommandBuffer.ParallelWriter ecb)
        {
            float aIm = em.GetComponentData<PhysicsMass2D>(IndexA.Entity).InverseMass;
            float bIm = em.GetComponentData<PhysicsMass2D>(IndexB.Entity).InverseMass;
            
            var aTransform = em.GetComponentData<PhysicsTransformComponent>(IndexA.Entity);
            var bTransform = em.GetComponentData<PhysicsTransformComponent>(IndexB.Entity);
            
            const float kSlop = 0.05f; // Penetration allowance
            const float percent = 0.4f; // Penetration percentage to correct
            float2 correction = math.max(Penetration - kSlop, 0.0f ) / (aIm + bIm) * Normal * percent;
            aTransform.Value.Translation -= correction * aIm;
            bTransform.Value.Translation += correction * bIm;
            
            ecb.SetComponent(index, IndexA.Entity, aTransform);
            ecb.SetComponent(index, IndexB.Entity, bTransform);
        }

        private void InfiniteMassCorrection(
            PhysicsVelocity2D aVelocity,
            PhysicsVelocity2D bVelocity,
            int index,
            EntityCommandBuffer.ParallelWriter ecb
            )
        {
            aVelocity.Linear = float2.zero;
            bVelocity.Linear = float2.zero;
            
            ecb.SetComponent(index, IndexA.Entity, aVelocity);
            ecb.SetComponent(index, IndexB.Entity, bVelocity);
        }
    }
}
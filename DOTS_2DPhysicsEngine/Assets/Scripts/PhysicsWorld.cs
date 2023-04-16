using System;

using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using Xedrial.Physics.Math;

namespace Xedrial.Physics
{
    [BurstCompile]
    public struct NativeQueueToArrayJob<T> : IJob where T : struct
    {
        public NativeQueue<T> NativeQueue;
        public NativeArray<T> NativeArray;

        public void Execute()
        {
            int i = 0;
            while (NativeQueue.TryDequeue(out T outValue))
            {
                NativeArray[i] = outValue;
                i++;
            }
        }
    }
    
    public partial class PhysicsWorld : SystemBase, IDisposable
    {
        private EntityQuery m_Bodies;

        [BurstCompile]
        private struct DetectCollisionJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody2D> Bodies;
            [ReadOnly] public float DeltaTime;
            
            public NativeQueue<Manifold>.ParallelWriter Collisions;

            public void Execute(int index)
            {
                RigidBody2D a = Bodies[index];

                for (int i = index + 1; i < Bodies.Length; i++)
                {
                    RigidBody2D b = Bodies[i];
                    if (a.InverseMass == 0 && b.InverseMass == 0)
                        continue;

                    var m = new Manifold
                    {
                        IndexA = index,
                        IndexB = i,
                        Gravity = 9.81f,
                        DeltaTime = DeltaTime
                    };
                    
                    m.Solve(Bodies);
                    
                    if (m.ContactCount > 0)
                        Collisions.Enqueue(m);
                }
            }
        }
        
        [BurstCompile]
        private partial struct CreateRigidBodiesJob : IJobEntity
        {
            public NativeQueue<RigidBody2D>.ParallelWriter RigidBodiesQueue;

            private void Execute(
                Entity e,
                in PhysicsBody2D pb,
                in PhysicsShape2D ps,
                in PhysicsTransformComponent ptc,
                in PhysicsForce2D pFo,
                in PhysicsFriction2D pFr,
                in PhysicsMass2D pm,
                in PhysicsMaterial2D pMat,
                in PhysicsVelocity2D pv,
                in PhysicsGravityFactor2D pg
            )
            {
                var rb = new RigidBody2D
                {
                    Entity = e,
                    Type = pb.Type,
                    Shape = ps.Shape,
                    Transform = ptc.Value,
                    Density = pMat.Density,
                    Restitution = pMat.Restitution,
                    Force = pFo.Force,
                    Torque = pFo.Torque,
                    AngularVelocity = pv.Angular,
                    LinearVelocity = pv.Linear,
                    DynamicFriction = pFr.Dynamic,
                    StaticFriction = pFr.Static,
                    InverseInertia = pm.InverseInertia,
                    InverseMass = pm.InverseMass
                };

                RigidBodiesQueue.Enqueue(rb);
            }
        }
        
        [BurstCompile]
        private struct InitializeManifoldsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody2D> RigidBodies;
            public NativeArray<Manifold> Contacts;

            public void Execute(int index)
            {
                Contacts[index].Initialize(RigidBodies);
            }
        }
        
        [BurstCompile]
        private struct IntegrateForcesJob : IJobParallelFor
        {
            [ReadOnly] public float Gravity;
            [ReadOnly] public float HalfDeltaTime;
            public NativeArray<RigidBody2D> Bodies;
            public EntityCommandBuffer.ParallelWriter EntityCommandBuffer;

            public void Execute(int index)
            {
                RigidBody2D rb = Bodies[index];
                
                if (rb.InverseMass == 0f)
                    return;

                rb.LinearVelocity += (rb.Force * rb.InverseMass + Gravity) * HalfDeltaTime;
                rb.AngularVelocity += rb.Torque * rb.InverseInertia * HalfDeltaTime;

                Bodies[index] = rb;
            }
        }

        [BurstCompile]
        private struct ApplyImpulsesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<Manifold> Contacts;
            public EntityCommandBuffer.ParallelWriter EntityCommandBuffer;

            public void Execute(int index)
            {
                Contacts[index].ApplyImpulse(index, EntityCommandBuffer);
            }
        }
        
        [BurstCompile]
        private struct IntegrateVelocitiesJob : IJobParallelFor
        {
            public NativeArray<RigidBody2D> Bodies;
            public float Gravity;
            public float DeltaTime;

            public void Execute(int index)
            {
                RigidBody2D rb = Bodies[index];
                
                if (rb.InverseMass == 0f)
                    return;

                rb.Transform.Translation += rb.LinearVelocity * DeltaTime;
                rb.Transform.Rotation = float2x2.Rotate(PhysicsMath.angle(rb.Transform.Rotation) + rb.AngularVelocity * DeltaTime);
                
                // Integrate Forces
                float halfDeltaTime = DeltaTime / 2f;
                
                rb.LinearVelocity += (rb.Force * rb.InverseMass + Gravity) * halfDeltaTime;
                rb.AngularVelocity += rb.Torque * rb.InverseInertia * halfDeltaTime;

                Bodies[index] = rb;
            }
        }
        
        [BurstCompile]
        private struct PositionalCorrectionsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<Manifold> Contacts;
            public EntityManager EntityManager;
            public EntityCommandBuffer.ParallelWriter EntityCommandBuffer;

            public void Execute(int index)
            {
                Contacts[index].PositionalCorrection(EntityManager, index, EntityCommandBuffer);
            }
        }
        
        [BurstCompile]
        private struct ClearAllForcesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody2D> Bodies;
            public EntityCommandBuffer.ParallelWriter EntityCommandBuffer;

            public void Execute(int index)
            {
                RigidBody2D rb = Bodies[index];
                
                rb.Force = 0;
                rb.Torque = 0;
                EntityCommandBuffer.SetComponent(index, rb.Entity, new PhysicsForce2D());

                Bodies[index] = rb;
            }
        }

        public void Dispose() => m_Bodies.Dispose();
        protected override void OnCreate()
        {
            m_Bodies = GetEntityQuery(
                typeof(PhysicsBody2D),
                typeof(PhysicsMass2D)
            );
            
            PhysicsUtility.AddBody(EntityManager, new BodyDef
            {
                Density = 1,
                Entity = EntityManager.CreateEntity(),
                Inertia = 1,
                Mass = 1,
                Orientation = quaternion.identity,
                Shape = Shape.Circle,
                Type = BodyType.Dynamic,
                Restitution = 1,
                Translation = float3.zero
            });
        }

        protected override void OnUpdate()
        {
            var rigidBodiesQueue = new NativeQueue<RigidBody2D>(Allocator.TempJob);

            new CreateRigidBodiesJob
            {
                RigidBodiesQueue = rigidBodiesQueue.AsParallelWriter()
            }.ScheduleParallel().Complete();

            var rigidBodies = new NativeArray<RigidBody2D>(rigidBodiesQueue.Count, Allocator.TempJob);
            new NativeQueueToArrayJob<RigidBody2D>
            {
                NativeQueue = rigidBodiesQueue,
                NativeArray = rigidBodies
            }.Schedule().Complete();
            
            var contactsQueue = new NativeQueue<Manifold>(Allocator.TempJob);

            new DetectCollisionJob
            {
                Bodies = rigidBodies,
                Collisions = contactsQueue.AsParallelWriter(),
                DeltaTime = Time.DeltaTime
            }.Schedule(rigidBodies.Length, 10).Complete();
            
            var contacts = new NativeArray<Manifold>(contactsQueue.Count, Allocator.TempJob);

            new NativeQueueToArrayJob<Manifold>
            {
                NativeQueue = contactsQueue,
                NativeArray = contacts
            }.Schedule().Complete();

            var ecb = new EntityCommandBuffer(Allocator.TempJob);

            new IntegrateForcesJob
            {
                Bodies = rigidBodies,
                Gravity = 9.81f,
                HalfDeltaTime = Time.DeltaTime / 2f,
                EntityCommandBuffer = ecb.AsParallelWriter()
            }.Schedule(rigidBodies.Length, 10).Complete();
            
            ecb.Playback(EntityManager);
            ecb.Dispose();
            
            new InitializeManifoldsJob
            {
                Contacts = contacts,
                RigidBodies = rigidBodies
            }.Schedule(contacts.Length, 10).Complete();


            for (int i = 0; i < 10; i++)
            {
                ecb = new EntityCommandBuffer(Allocator.TempJob);

                new ApplyImpulsesJob
                {
                    Contacts = contacts,
                    EntityCommandBuffer = ecb.AsParallelWriter()
                }.Schedule(contacts.Length, 10).Complete();
                
                ecb.Playback(EntityManager);
                ecb.Dispose();
            }

            new IntegrateVelocitiesJob
            {
                Bodies = rigidBodies,
                DeltaTime = Time.DeltaTime,
                Gravity = 9.81f
            }.Schedule(rigidBodies.Length, 10).Complete();
            
            ecb = new EntityCommandBuffer(Allocator.TempJob);

            new PositionalCorrectionsJob
            {
                Contacts = contacts,
                EntityManager = EntityManager,
                EntityCommandBuffer = ecb.AsParallelWriter()
            }.Schedule(rigidBodies.Length, 10).Complete();
            
            ecb.Playback(EntityManager);
            ecb.Dispose();
            
            ecb = new EntityCommandBuffer(Allocator.TempJob);

            new ClearAllForcesJob()
            {
                Bodies = rigidBodies,
                EntityCommandBuffer = ecb.AsParallelWriter()
            }.Schedule(rigidBodies.Length, 10).Complete();
            
            ecb.Playback(EntityManager);
            ecb.Dispose();
        }
    }

    public struct RigidBody2D
    {
        public Entity Entity;

        public BodyType Type;
        public Shape Shape;

        public PhysicsTransform Transform;

        public float InverseMass;
        public float InverseInertia;
        
        public float2 LinearVelocity;
        public float AngularVelocity;
        
        public float2 Force;
        public float Torque;

        public float StaticFriction;
        public float DynamicFriction;

        public float Density;
        public float Restitution;
    }

    public struct Collision2D : IComponentData
    {
        public RigidBody2D A;
        public RigidBody2D B;
    }
}
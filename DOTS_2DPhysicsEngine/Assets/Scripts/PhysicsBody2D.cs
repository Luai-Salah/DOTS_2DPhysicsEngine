using Unity.Entities;
using Unity.Mathematics;
using Xedrial.Physics.Math;

namespace Xedrial.Physics
{
    public struct EntityPair
    {
        public Entity EntityA;
        public Entity EntityB;
    }
    
    public struct PhysicsVelocity2D : IComponentData
    {
        public float2 Linear;
        public float Angular;
    }
    
    public struct PhysicsForce2D : IComponentData
    {
        public float2 Force;
        public float Torque;
    }
    
    public struct PhysicsShape2D : IComponentData
    {
        public Shape Shape;
    }
    
    public struct PhysicsMass2D : IComponentData
    {
        public float InverseMass;
        public float InverseInertia;
    }

    public struct PhysicsGravityFactor2D : IComponentData
    {
        public float Value;
    }
    
    public struct PhysicsMaterial2D : IComponentData
    {
        public float Density;
        public float Restitution;
    }
    
    public struct PhysicsFriction2D : IComponentData
    {
        public float Static;
        public float Dynamic;
    }

    public struct PhysicsBody2D : IComponentData
    {
        public BodyType Type;
    }

    public struct PhysicsTransformComponent : IComponentData
    {
        public PhysicsTransform Value;
    }
}
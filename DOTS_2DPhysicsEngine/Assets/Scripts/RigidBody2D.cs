using Unity.Entities;
using Unity.Mathematics;
using Xedrial.Physics.Math;

namespace Xedrial.Physics
{
    public enum Shape { Polygon, Circle }

    public struct CircleShapeComponent : IComponentData
    {
        public Circle Value;
    }

    public struct PolygonShape
    {
    }

    public enum BodyType { Dynamic, Static }

    public struct BodyDef
    {
        public Entity Entity;
        
        public BodyType Type;
        public Shape Shape;
        
        public float3 Translation;
        public quaternion Orientation;

        public float Mass;
        public float Inertia;
        
        public float Density;
        public float Restitution;
    }
}
using System;

using Unity.Mathematics;
using UnityEngine;

namespace Xedrial.Physics
{
    [Serializable]
    public struct AABB
    {
        public float2 Center;
        public float2 Extent;

        public AABB(float2 min, float2 max)
        {
            Center = (max + min) / 2f;
            Extent = (max - min) / 2f;
        }

        public static bool operator&(AABB a, AABB b)
        {
            // Vector from A to B
            float2 n = a.Center - b.Center;
  
            // Calculate overlap on x axis
            float x_overlap = a.Extent.x + b.Extent.x - math.abs(n.x);
  
            // SAT test on x axis
            if (x_overlap <= 0) 
                return false;
            
            // Calculate overlap on y axis
            float y_overlap = a.Extent.y + b.Extent.y - math.abs(n.y);

            // SAT test on y axis
            return y_overlap > 0;
        }
    }
    
    public interface ICollidable
    {
        public AABB ComputeAABB();
    }
    
    [Serializable]
    public struct Circle : ICollidable
    {
        public float2 Position;
        public float Radius;
        
        public AABB ComputeAABB()
        {
            return new AABB
            {
                Center = Position,
                Extent = new Vector2(Radius, Radius)
            };
        }
    }

    public static class Collision
    {
        public static void CircleToCircle(ref Manifold manifold, in Circle a, in Circle b)
        {
            // Calculate translational vector, which is normal
            float2 normal = a.Position - b.Position;

            float dist_sqr = math.lengthsq(normal);
            float radius = a.Radius + b.Radius;

            // Not in contact
            if(dist_sqr >= radius * radius)
            {
                manifold.ContactCount = 0;
                return;
            }

            float distance = math.sqrt(dist_sqr);

            manifold.ContactCount = 1;

            if (distance == 0.0f)
            {
                manifold.Penetration = a.Radius;
                manifold.Normal = new float2(1, 0);
                manifold.Contact_1 = a.Position;
            }
            else
            {
                manifold.Penetration = radius - distance;
                manifold.Normal = normal / distance; // Faster than using Normalized since we already performed sqrt
                manifold.Contact_1 = manifold.Normal * a.Radius + a.Position;
            }
        }
    }
}

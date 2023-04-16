using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Xedrial.Physics
{
    public class Testing : MonoBehaviour
    {
        public Vector2 m_Center_A;
        public Vector2 m_Center_B;
        public Vector2 m_Extent_A;
        public Vector2 m_Extent_B;
        
        private void Update()
        {
            var a = new AABB
            {
                Center = m_Center_A,
                Extent = m_Extent_A
            };
            
            var b = new AABB
            {
                Center = m_Center_B,
                Extent = m_Extent_B
            };
            
            Debug.Log(a & b);
        }

        private void OnDrawGizmos()
        {
            Gizmos.DrawWireCube(m_Center_A, m_Extent_A * 2f);
            Gizmos.DrawWireCube(m_Center_B, m_Extent_B * 2f);
        }
    }
}

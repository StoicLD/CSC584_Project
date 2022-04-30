using System;
using System.Collections.Generic;
using UnityEngine;

namespace AI
{
    public class TexiRoute : MonoBehaviour
    {
        public List<Transform> path = new List<Transform>();

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            if (path.Count>0)
            {
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Gizmos.DrawLine(path[i].position,path[i+1].position);
                }
            }
            
        }
    }
}
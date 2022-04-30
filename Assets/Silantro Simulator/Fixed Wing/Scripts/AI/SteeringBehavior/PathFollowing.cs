using System.Collections.Generic;
using UnityEngine;

namespace AI
{
    public class PathFollowing : Steering
    {
        private List<Transform> path = new List<Transform>();

        private Transform character;

        private int currentIndex = 0;

        private float arriveRadius = 5f;

        public Transform Character
        {
            get => character;
            set => character = value;
        }

        public int CurrentIndex => currentIndex;

        public float ArriveRadius
        {
            get => arriveRadius;
            set => arriveRadius = value;
        }

        public List<Transform> Path => path;

        public void Init(Transform t)
        {
            this.character = t;
            Transform pathContainer = GameObject.Find("Takeoff Track A").transform;
            for (int i = 0; i < pathContainer.childCount; i++)
            {
                path.Add(pathContainer.GetChild(i));
            }
        }

        public float GetTotalDistance()
        {
            if (path.Count == 0)
            {
                return 0;
            }

            float dist = 0;
            for (int i = 0; i < path.Count - 1; i++)
            {
                dist += Vector3.Distance(path[i].position, path[i + 1].position);
            }

            return dist;
        }

        public float GetRestDistance()
        {
            if (path.Count == 0 || currentIndex == path.Count - 1)
            {
                return 0;
            }

            float dist = 0;
            for (int i = currentIndex; i < path.Count - 1; i++)
            {
                dist += Vector3.Distance(path[i].position, path[i + 1].position);
            }

            return dist;
        }

        // Only calculate speed for original interface
        public float CalculateSpeed()
        {
            if (Vector3.Distance(character.position, path[currentIndex].position) <= arriveRadius)
            {
                if (currentIndex == path.Count - 1)
                {
                    return 0;
                }
                currentIndex++;
            }
            return (GetRestDistance() / GetTotalDistance()) * 0.8f + 0.2f;
        }

        public override Vector3 CalculateSteering()
        {
            Vector3 steering = new Vector3();
            if (Vector3.Distance(character.position, path[currentIndex].position) <= arriveRadius)
            {
                if (currentIndex == path.Count - 1)
                {
                    return Vector3.zero;
                }
                currentIndex++;
            }
            //todo add coefficient
            steering = (path[currentIndex].position - character.position).normalized;
            return steering;
        }
    }
}
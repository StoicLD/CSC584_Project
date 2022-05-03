using System.Collections.Generic;
using UnityEngine;

namespace AI
{
    public class EasyPathFollowing
    {
        private List<Transform> path = new List<Transform>();

        private Transform character;

        private int currentIndex = 0;

        private float arriveRadius = 5f;
        private float maxAcceleration = 100;
        private float maxSpeed = 30;

        private float timeToTarget = 1;
        private float targetRadius = 10;
        private float slowRadius = 20;

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
            Transform pathContainer = GameObject.Find("path").transform;
            for (int i = 0; i < pathContainer.childCount; i++)
            {
                path.Add(pathContainer.GetChild(i));
            }
        }
        
        public Acceleration CalculateSteering(SteeringData cha)
        {
            Acceleration a = new Acceleration();
            a.angular = cha.Orientation;
            Vector3 targetPos = new Vector3();
            Vector3 targetVelocity = new Vector3();
            float targetSpeed = 0;
            
            if (path.Count>0 && currentIndex < path.Count)
            {
                targetPos = path[currentIndex].position;
                Vector3 dis = targetPos - character.position;
                Vector3 direction = dis.normalized;
                
                if (dis.magnitude > slowRadius)
                {
                    targetSpeed = maxSpeed;
                }
                else
                {
                    targetSpeed = maxSpeed * ((dis.magnitude - targetRadius) / (slowRadius - targetRadius));
                }
                
                targetVelocity = direction * targetSpeed;
                a.linear = targetVelocity - cha.Velocity;
                a.linear /= timeToTarget;

                a.angular = direction;
                
                if (a.linear.magnitude > maxAcceleration)
                {
                    a.linear.Normalize();
                    a.linear *= maxAcceleration;
                }
            }

            if ((character.position - targetPos).magnitude < 10)
            {
                currentIndex++;
            }
            return a;
        }
    }
}
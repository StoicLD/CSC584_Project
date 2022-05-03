using System;
using UnityEngine;

namespace AI
{
    public class EasyAI : MonoBehaviour
    {
        private EasyPathFollowing pathFollowing;

        private SteeringData data;
        
        public float angleSpeed = 0.00001f;
        public bool isRotate = true;

        private Vector3 lastOrientation;
        
        void Initialize()
        {
            data = new SteeringData();
            pathFollowing = new EasyPathFollowing();
            lastOrientation = transform.forward;
            pathFollowing.Init(transform);
            data.Position = transform.position;
        }

        private void Start()
        {
            Initialize();
        }
        public static float AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
        {
            return Mathf.Atan2(
                Vector3.Dot(n, Vector3.Cross(v1, v2)),
                Vector3.Dot(v1, v2)) * Mathf.Rad2Deg;
        }
        
        public Vector3 NormalizeEuler(Vector3 an)
        {
            Vector3 rotation = an;
            if (rotation.x < -180f) { rotation.x = rotation.x + 360f; }
            else if (rotation.x > 180f) { rotation.x = rotation.x - 360f; }

            if (rotation.y < -180f) { rotation.y = rotation.y + 360f; }
            else if (rotation.y > 180f) { rotation.y = rotation.y - 360f; }

            if (rotation.z < -180f) { rotation.z = rotation.z + 360f; }
            else if (rotation.z > 180f) { rotation.z = rotation.z - 360f; }
            
            return rotation;
        }

        public float CalculateAngle(float angle)
        {
            if (Mathf.Abs(angle) > 20)
            {
                return 80;
            }
            return 50;
        }
        
        private void Update()
        {
            data.Position = transform.position;
            data.UpdateData(pathFollowing.CalculateSteering(data));
            transform.position = data.Position;
            
            Vector3 vec = data.Orientation;
            Quaternion rotate = Quaternion.LookRotation(vec);
            Debug.LogError(NormalizeEuler(transform.eulerAngles).z);
            if (AngleSigned(vec, transform.forward, transform.up) < -1 && Mathf.Abs(NormalizeEuler(transform.eulerAngles).z) <= CalculateAngle(AngleSigned(vec, transform.forward, transform.up)))
            {
                transform.Rotate(new Vector3(0, 0, -200 * Time.deltaTime));
            }
            else if (AngleSigned(vec, transform.forward, transform.up) > 1 && Mathf.Abs(NormalizeEuler(transform.eulerAngles).z) <= CalculateAngle(AngleSigned(vec, transform.forward, transform.up)))
            {
                transform.Rotate(new Vector3(0, 0, 200 * Time.deltaTime));
            }
            else
            {
                if (NormalizeEuler(transform.eulerAngles).z > 5)
                {
                    transform.Rotate(new Vector3(0, 0, -50 * Time.deltaTime));
                }
                else if (NormalizeEuler(transform.eulerAngles).z < -5)
                {
                    transform.Rotate(new Vector3(0, 0, 50 * Time.deltaTime));
                }
            }
            
            if (Vector3.Angle(vec, transform.forward) < 0.1f)
            {
                isRotate = false;
            }
            else
            {
                isRotate = true;
            }
            if (isRotate)
            {
                transform.localRotation = Quaternion.Slerp(transform.localRotation, rotate, angleSpeed);
            }
        }
    }
}
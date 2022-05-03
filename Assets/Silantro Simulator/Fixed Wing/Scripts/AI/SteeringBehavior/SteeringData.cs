using UnityEngine;

namespace AI
{
    public class Acceleration
    {
        public Vector3 linear;
        public Vector3 angular;

        public Acceleration()
        {
            linear = new Vector3();
        }
    }

    public class SteeringData
    {
        private Vector3 position;
        private Vector3 velocity;
        private Vector3 orientation;
        public Vector3 Position
        {
            get => position;
            set => position = value;
        }

        public Vector3 Velocity
        {
            get => velocity;
            set => velocity = value;
        }

        public Vector3 Orientation
        {
            get => orientation;
            set => orientation = value;
        }

        public SteeringData()
        {
            position = new Vector3();
            velocity = new Vector3();
            orientation = new Vector3();
        }
        public void UpdateData(Acceleration a)
        {
            velocity += a.linear * Time.deltaTime;
            position += velocity * Time.deltaTime;
            orientation = a.angular;
        }
    }
}
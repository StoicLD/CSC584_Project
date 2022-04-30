using UnityEngine;

namespace AI
{
    public abstract class Steering
    {
        /// <summary>
        /// Calculate force or acceleration
        /// </summary>
        /// <returns>Force or acceleration</returns>
        /// todo update function
        public abstract Vector3 CalculateSteering();
    }
}
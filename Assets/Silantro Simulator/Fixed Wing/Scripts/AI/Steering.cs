using UnityEngine;

namespace AI
{
    abstract public class Steering
    {
        /// <summary>
        /// Calculate force or acceleration
        /// </summary>
        /// <returns>Force or acceleration</returns>
        /// todo update function
        abstract public Vector3 CalculateSteering();
    }
}
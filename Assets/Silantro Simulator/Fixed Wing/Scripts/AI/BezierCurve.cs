using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class BeizeCurve
{
    public List<Vector3> controlPoints;

    public BeizeCurve(List<Vector3> otherControlPoints)
    {
        controlPoints = new List<Vector3>();
        for (int i = 0; i < otherControlPoints.Count; i++)
        {           
            controlPoints.Add(new Vector3(otherControlPoints[i].x, otherControlPoints[i].y, otherControlPoints[i].z));
        }
    }

    public Vector3 GetCurrentPoint(float t)
    {
        t = Mathf.Clamp(t, 0.0f, 1.0f);
        var tmp = new List<Vector3>();

        for (int i = 0; i < controlPoints.Count; i++)
        {
            tmp.Add(new Vector3(controlPoints[i].x, controlPoints[i].y, controlPoints[i].z));
        }

        int count = controlPoints.Count - 1;

        while (count > 0)
        {
            for (int k = 0; k < count; k++)
            {
                tmp[k] = tmp[k] + t * (tmp[k + 1] - tmp[k]);
            }
            count--;
        }

        return tmp[0];
    }

    public float GetBin(int n, int i, float t)
    {
        int prefix = MathToolskit.GetPermutation(n, i);
        return (float)prefix * Mathf.Pow(t, i) * Mathf.Pow(1.0f - t, n - i);
    }


    public Vector3 GetCurrentDerivative(float t)
    {
        Vector3 result = new Vector3(0.0f, 0.0f, 0.0f);
        int n = controlPoints.Count;
        for (int i = 0; i < n - 1; i++)
        {
            result += GetBin(n - 1, i, t) * (controlPoints[i + 1] - controlPoints[i]);
        }
        return result * n;
    }
}

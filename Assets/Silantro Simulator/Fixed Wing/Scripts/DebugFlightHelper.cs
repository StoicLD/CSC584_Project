using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugFlightHelper : MonoBehaviour
{
    // Start is called before the first frame update
    public SilantroFlightComputer computer;
    public BeizeCurve curve;

    private const int drawPointsSize = 30;
    private List<Vector3> drawPoints = new List<Vector3>();
    private List<Vector3> drawPointsDirective = new List<Vector3>();


    void Start()
    {
        GetControlPoints();
    }

    void GetControlPoints()
    {
        if (computer.brain != null)
        {
            //List<Vector3> bControlPoints = computer.brain.akwardCurve.controlPoints;
            curve = computer.brain.akwardCurve;
            float delta = 1.0f / (float)drawPointsSize;
            drawPoints.Clear();
            drawPointsDirective.Clear();
            for (int i = 0; i < drawPointsSize; i++)
            {
                drawPoints.Add(curve.GetCurrentPoint(delta * (float)i));
                drawPointsDirective.Add(curve.GetCurrentDerivative(delta * (float)i));
                //Debug.Log(drawPoints[i]);
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        if(computer != null)
        {
            //float yawVec = computer.yawAngle;
            //float rollVec = computer.rollAngle;
            //float pitchVec = computer.pitchAngle;

            //Vector3 localAxisX = computer.transform.localPosition + new Vector3(10f, 0f, 0f);
            //Vector3 localAxisY = computer.transform.localPosition + new Vector3(0f, 10f, 0f);
            //Vector3 localAxisZ = computer.transform.localPosition + new Vector3(0f, 0f, 10f);

            Vector3 currPos = computer.brain.GetCurveStartPos();
            for (int i = 0; i < drawPoints.Count - 1; i++)
            {
                Debug.DrawLine(currPos + drawPoints[i], currPos + drawPoints[i + 1], Color.black);
                Debug.DrawLine(currPos + drawPoints[i], currPos + drawPoints[i] + drawPointsDirective[i].normalized, Color.red);
            }

            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisX), Color.red);
            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisY), Color.black);
            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisZ), Color.yellow);

            //Debug.Log("yaw: " + yawVec.ToString());
            //Debug.Log("roll: " + rollVec.ToString());
            //Debug.Log("pitch: " + pitchVec.ToString());
        }
    }
}

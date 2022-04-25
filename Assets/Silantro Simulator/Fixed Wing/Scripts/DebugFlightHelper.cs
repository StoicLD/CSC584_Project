using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugFlightHelper : MonoBehaviour
{
    // Start is called before the first frame update
    public SilantroFlightComputer computer;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        if(computer != null)
        {
            float yawVec = computer.yawAngle;
            float rollVec = computer.rollAngle;
            float pitchVec = computer.pitchAngle;
            
            Vector3 localAxisX = computer.transform.localPosition + new Vector3(10f, 0f, 0f);
            Vector3 localAxisY = computer.transform.localPosition + new Vector3(0f, 10f, 0f);
            Vector3 localAxisZ = computer.transform.localPosition + new Vector3(0f, 0f, 10f);

            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisX), Color.red);
            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisY), Color.black);
            //Debug.DrawRay(computer.transform.position, computer.transform.TransformDirection(localAxisZ), Color.yellow);

            //Debug.Log("yaw: " + yawVec.ToString());
            //Debug.Log("roll: " + rollVec.ToString());
            //Debug.Log("pitch: " + pitchVec.ToString());
        }
    }
}

using Oyedoyin;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using AI;

/// <summary>
/// 
/// Use:		 Handles the AI control section of the flight computer
/// </summary>




[Serializable]
public class SilantroBrain
{
    // ----------------------------------------- Selectibles
    public enum FlightState { Grounded, Taxi, Takeoff, Cruise, Loiter, Decent, Landing }
    public FlightState flightState = FlightState.Grounded;

    public enum TaxiState { Stationary, Moving, Holding}
    public TaxiState taxiState = TaxiState.Stationary;


    // ----------------------------------------- Connections
    public Rigidbody aircraft;
    public SilantroWaypointPlug tracker;
    //public SilantroWaypointPlug cruiseTracker;
    public Transform cruiseWayPoint;

    public SilantroController controller;
    public AnimationCurve steerCurve;
    public SilantroFlightComputer computer;



    // ----------------------------------------- Variables
    public float currentSpeed;
    public float takeoffSpeed = 100f;
    public float climboutPitchAngle = 8f;
    public float checkListTime = 2f;
    public float transitionTime = 5f;
    private float evaluateTime = 12f;
    public float takeoffHeading;
    public float inputCheckFactor;
    private float currentTestTime;



    // -------------------------------- Control Markers
    public bool flightInitiated;
    public bool checkedSurfaces;
    public bool groundChecklistComplete;
    public bool isTaxing;
    public bool checkingSurfaces;
    public bool clearedForTakeoff;
   

    // -------------------------------- Taxi
    public float maximumTaxiSpeed = 100f;
    public float recommendedTaxiSpeed = 8f;
    public float maximumTurnSpeed = 5f;
    public float maximumSteeringAngle = 30f;
    public float minimumSteeringAngle = 15f;
    public float steeringAngle;
    public float targetTaxiSpeed;

    [Range(0, 1)] public float steerSensitivity = 0.05f;
    [Range(0, 2)] public float brakeSensitivity = 0.85f;
    public float brakeInput;


    // -------------------------------- Cruise
    public float cruiseAltitude = 1000f;
    public float cruiseSpeed = 300f;
    public float cruiseHeading = -90f;
    public float cruiseClimbRate = 1200f;
    public float cruiseHeadingTurnFactor = 1.25f;


    public float cruiseAccerBankAngle = 30f;
    public float cruiseDeccerBankAngle = 15f;


    //---------------------------------- New
    // public Transform pathContainer;
    public BeizeCurve akwardCurve = null;
    public bool useAkwardCurve = true;
    private float akwardCurveTime = 0.0f;
    private Vector3 curveStartPostion = new Vector3();


    private List<Transform> path = new List<Transform>();
    private int path_cur = 0;
    private PathFollowing pathFollowing;

    private bool isTaxiModeNeeded = true;

    

    // Start is called before the first frame update

    private Matrix4x4 fromRotation(float rad, Vector3 axis) 
    {
        float x = axis[0];
        float y = axis[1];
        float z = axis[2];
        float len = Mathf.Sqrt(x*x + y*y + z*z);
        float s, c, t;
        Matrix4x4 ret = new Matrix4x4();
        ret[0] = 0;
        if (len < 1e-7) {
            return ret;
        }
        len = 1 / len;
        x *= len;
        y *= len;
        z *= len;
        s = Mathf.Sin(rad);
        c = Mathf.Cos(rad);
        t = 1 - c;
        // Perform rotation-specific matrix multiplication
        ret[0] = x * x * t + c;
        ret[1] = y * x * t + z * s;
        ret[2] = z * x * t - y * s;
        ret[3] = 0;
        ret[4] = x * y * t - z * s;
        ret[5] = y * y * t + c;
        ret[6] = z * y * t + x * s;
        ret[7] = 0;
        ret[8] = x * z * t + y * s;
        ret[9] = y * z * t - x * s;
        ret[10] = z * z * t + c;
        ret[11] = 0;
        ret[12] = 0;
        ret[13] = 0;
        ret[14] = 0;
        ret[15] = 1;
        return ret;
    }
    private void ApplyMat4toAxis(ref Matrix4x4 l, ref Vector3 a)
    {
        a = l.MultiplyPoint3x4(a);
        return;
    }

    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    public void InitializeBrain()
    {
        if(akwardCurve == null)
        {
            List<Vector3> bControlPoints = new List<Vector3>();
            bControlPoints.Add(new Vector3(0f, 0f, 0f));
            bControlPoints.Add(new Vector3(100f, 0f, 100f));
            bControlPoints.Add(new Vector3(200f, 0f, 0f));
            akwardCurve = new BeizeCurve(bControlPoints);
        }
        if(useAkwardCurve)
        {
            akwardCurveTime = 0.0f;
        }

        // AI customized path
        pathFollowing = new PathFollowing();
        pathFollowing.Init(controller.transform);

        // -------------------------
        aircraft = controller.aircraft;
        if(tracker != null) { tracker.aircraft = controller; tracker.InitializePlug(); }
        // customized path
        // pathContainer = GameObject.Find("Takeoff Track A").transform;
        // for (int i = 0; i < pathContainer.childCount; i++)
        // {
        //     path.Add(pathContainer.GetChild(i));
        // }
        // path_cur = 0;
        // ------------------------ Plot
        PlotSteerCurve();
    }


    public void test1()
    {
        cruiseHeading -= 2.5f;
        //cruiseHeading = -90f;
        Debug.Log("Test1" + cruiseHeading);
    }

    public void test2()
    {
        cruiseAltitude -= 100;
        Debug.Log("Test2  " + cruiseAltitude);
    }
    
    public void test3()
    {
        cruiseHeading += 2.5f;
        Debug.Log("Test3" + cruiseHeading);
    }

    public void test5()
    {
        cruiseAltitude += 100;
        Debug.Log("Test5  " + cruiseAltitude);
    }
    public void test4()
    {
        Vector3 temp = aircraft.velocity.normalized;
        float x =  Mathf.Sin(computer.yawAngle* Mathf.Deg2Rad);
        float z =  Mathf.Cos(computer.yawAngle* Mathf.Deg2Rad);
        
        Vector3 Model_Facing_Axis = new Vector3(
            Mathf.Cos(computer.pitchAngle* Mathf.Deg2Rad) * x,
            -Mathf.Sin(computer.pitchAngle* Mathf.Deg2Rad),
            Mathf.Cos(computer.pitchAngle* Mathf.Deg2Rad) * z
        );
        Vector3 Model_Side_Axis = new Vector3(
            Mathf.Cos(computer.rollAngle* Mathf.Deg2Rad) *z,
            -Mathf.Sin(computer.rollAngle* Mathf.Deg2Rad),
            Mathf.Cos(computer.rollAngle* Mathf.Deg2Rad) *-x
        );
        Vector3 Model_Up = Vector3.Cross(Model_Facing_Axis,Model_Side_Axis);
        Debug.Log(  " " + temp.ToString("F4"));
        Debug.Log( "SB: " + Model_Side_Axis.ToString("F4") + " " + Model_Up.ToString("F4"));
        temp = Vector3.ProjectOnPlane(temp, Model_Up);
        Debug.Log( "SB: " + Model_Facing_Axis.ToString("F4") + " " + temp.ToString("F4"));
        Debug.Log( "ANGLE ERROR?: " + Vector3.SignedAngle(temp, Model_Facing_Axis, Model_Up));

        //float rotangle = computer.rollAngle;
        //Matrix4x4 m1 = fromRotation(Mathf.Deg2Rad * rotangle, Model_Facing_Axis);
        //temp = m1.MultiplyPoint3x4(temp);
        //Vector3 rotAxis = Vector3.Cross(Model_Facing_Axis, temp);

        //rotangle = Vector3.Angle(Model_Facing_Axis, new Vector3(0,0,1));
        //Vector3 rotAxis = Vector3.Cross(Model_Facing_Axis, new Vector3(0,0,1));
        //m1 = fromRotation(Mathf.Deg2Rad * rotangle, rotAxis);
        //temp = m1.MultiplyPoint3x4(temp);

        //temp -= Model_Facing_Axis;
        //float yawError = (float)Math.Round(temp[0] - Model_Facing_Axis[0], 3) * 100;
        //Debug.Log(Model_Facing_Axis.ToString("F4") + "\n" + temp.ToString("F4") + "  " + aircraft.velocity);
        //Debug.Log( "YAW ERROR: " + temp.ToString("F4"));
       
    }
/*
"CR:" + computer.currentClimbRate +
        float sp = temp.magnitude;
        float x = Mathf.Cos(computer.pitchAngle* Mathf.Deg2Rad) * Mathf.Sin(computer.yawAngle* Mathf.Deg2Rad)  * sp;
        float z = Mathf.Cos(computer.pitchAngle* Mathf.Deg2Rad) * Mathf.Cos(computer.yawAngle* Mathf.Deg2Rad) *  sp;
        float y = -Mathf.Sin(computer.pitchAngle* Mathf.Deg2Rad) * sp;
*/
    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    public void UpdateBrain()
    {
        //---------------------------------------------OPERATION MODE
        switch (flightState)
        {
            case FlightState.Grounded: GroundMode(); break;
            case FlightState.Taxi: TaxiMode(); break;
            case FlightState.Takeoff: TakeoffMode();
                 break;
            case FlightState.Cruise: 
                if(useAkwardCurve)
                {
                    AkwardCruiseMode();
                }
                else
                {
                    CruiseMode();
                }
                break;
        }


        // ---------------------------------------------- COLLECT DATA
        currentSpeed = controller.core.currentSpeed;

        // ---------------------------------------------- SURFACE CHECK
        if (checkingSurfaces) { CheckControlSurfaces(inputCheckFactor); }
    }









    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    public void PlotSteerCurve()
    {
        // ------------------------- Plot Steer Curve
        steerCurve = new AnimationCurve();

        steerCurve.AddKey(new Keyframe(0.0f * maximumSteeringAngle, maximumTaxiSpeed));
        steerCurve.AddKey(new Keyframe(0.5f * maximumSteeringAngle, recommendedTaxiSpeed));
        steerCurve.AddKey(new Keyframe(0.8f * maximumSteeringAngle, maximumTurnSpeed));

#if UNITY_EDITOR
        for (int i = 0; i < steerCurve.keys.Length; i++)
        {
            AnimationUtility.SetKeyLeftTangentMode(steerCurve, i, AnimationUtility.TangentMode.Auto);
            AnimationUtility.SetKeyRightTangentMode(steerCurve, i, AnimationUtility.TangentMode.Auto);
        }
#endif
    }




    // -------------------------------------------GROUND MODE----------------------------------------------------------------------------------------------------------
    void GroundMode()
    {
        //------------------------- Start Flight Process
        if(flightInitiated && !controller.engineRunning) { controller.TurnOnEngines(); flightInitiated = false; }
        if(flightInitiated && controller.engineRunning) { flightInitiated = false; }


        if (!controller.engineRunning)
        {
            // Check States
            if (controller.lightState == SilantroController.LightState.On) { controller.input.TurnOffLights(); }
            if (controller.gearHelper && controller.gearHelper.brakeState == SilantroGearSystem.BrakeState.Disengaged) { controller.gearHelper.EngageBrakes(); }
        }
        else
        {
            // -------------------------------------------------------------------------------------- Check List
            if (!groundChecklistComplete)
            {
                //Debug.Log("Engine Start Complete, commencing ground checklist");
                computer.StartCoroutine(GroundCheckList());
            }
        }
    }

    bool flapSet;
    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    IEnumerator GroundCheckList()
    {
        // --------------------------- Lights
        controller.input.TurnOnLights();
        // flapSet = true;
        if (isTaxiModeNeeded)
        {
            // --------------------------- Actuators
            if (controller.canopyActuator && controller.canopyActuator.actuatorState == SilantroActuator.ActuatorState.Engaged) { controller.canopyActuator.DisengageActuator(); }
            if (controller.speedBrakeActuator && controller.speedBrakeActuator.actuatorState == SilantroActuator.ActuatorState.Engaged) { controller.speedBrakeActuator.DisengageActuator(); }
            if (controller.wingActuator && controller.wingActuator.actuatorState == SilantroActuator.ActuatorState.Engaged) { controller.wingActuator.DisengageActuator(); }

            // --------------------------- Flaps
            // yield return new WaitForSeconds(checkListTime);
            foreach (SilantroAerofoil foil in controller.wings)
            {
                if (foil.flapSetting != 1 && !flapSet && foil.flapAngleSetting == SilantroAerofoil.FlapAngleSetting.ThreeStep) { foil.SetFlaps(1, 1); }
                if (foil.flapSetting != 2 && !flapSet && foil.flapAngleSetting == SilantroAerofoil.FlapAngleSetting.FiveStep) { foil.SetFlaps(2, 1); }
            }
            flapSet = true;

            // --------------------------- Slats
            /* yield return new WaitForSeconds(checkListTime);
            foreach (SilantroAerofoil foil in controller.flightComputer.wingFoils)
            {
                if (foil.slatState == SilantroAerofoil.ControlState.Active) { foil.baseSlat = Mathf.MoveTowards(foil.baseSlat, controller.flightComputer.takeOffSlat, foil.slatActuationSpeed * Time.fixedDeltaTime); }
            }*/


            // --------------------------- Control Surfaces

            // yield return new WaitForSeconds(checkListTime);
            if (!checkingSurfaces && currentTestTime < 1f) { currentTestTime = evaluateTime; checkingSurfaces = true; }
            if (!checkedSurfaces)
            {
                float startRange = -1.0f; float endRange = 1.0f; float cycleRange = (endRange - startRange) / 2f;
                float offset = cycleRange + startRange;
                inputCheckFactor = offset + Mathf.Sin(Time.time * 5f) * cycleRange;
            }
        }
        // yield return new WaitForSeconds(evaluateTime);
        checkedSurfaces = true;checkingSurfaces = false;
        computer.processedPitch = 0f;
        computer.processedRoll = 0f;
        computer.processedYaw = 0f;
        computer.processedStabilizerTrim = 0f;
                
        groundChecklistComplete = true;

        // ---------------------------- Transition
        yield return new WaitForSeconds(transitionTime);
        if (isTaxiModeNeeded)
        {
            flightState = FlightState.Taxi;
        }
        else
        {
            flightState = FlightState.Takeoff;
        }
        // takeoffHeading = computer.currentHeading;
        if (controller.gearHelper != null) { controller.gearHelper.ReleaseBrakes(); } 
    }


    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    void CheckControlSurfaces(float controlInput)
    {
        if (checkingSurfaces)
        {
            currentTestTime -= Time.deltaTime;
            if(currentTestTime < 0) { currentTestTime = 0f; }

            //--------------------- Pitch
            if(currentTestTime < evaluateTime && currentTestTime > (0.75f*evaluateTime))
            {
                computer.processedPitch = controlInput;
                computer.processedRoll = 0f;
                computer.processedYaw = 0f;
                computer.processedStabilizerTrim = 0f;
            }
            //--------------------- Roll
            if (currentTestTime < (0.75f * evaluateTime) && currentTestTime > (0.50f * evaluateTime))
            {
                computer.processedPitch = 0f;
                computer.processedRoll = controlInput;
                computer.processedYaw = 0f;
                computer.processedStabilizerTrim = 0f;
            }
            //--------------------- Yaw
            if (currentTestTime < (0.50f * evaluateTime) && currentTestTime > (0.25f * evaluateTime))
            {
                computer.processedPitch = 0f;
                computer.processedRoll = 0f;
                computer.processedYaw = controlInput;
                computer.processedStabilizerTrim = 0f;
            }
            //--------------------- Trim
            if (currentTestTime < (0.25f * evaluateTime) && currentTestTime > (0.00f * evaluateTime))
            {
                computer.processedPitch = 0f;
                computer.processedRoll = 0f;
                computer.processedYaw = 0f;
                computer.processedStabilizerTrim = controlInput;
            }
        }
    }
    
    // -------------------------------------------TAXI----------------------------------------------------------------------------------------------------------
    void TaxiMode()
    {
        //Debug.Log("Taxiing!");
        // ------------------------------------- Clamp
        float thresholdSpeed = maximumTaxiSpeed;

        // ------------------------------------- States
        if (taxiState == TaxiState.Stationary)
        {
            if (controller.gearHelper != null && controller.gearHelper.brakeState == SilantroGearSystem.BrakeState.Disengaged && tracker.currentPoint <= tracker.track.pathPoints.Count)
            {
                
                taxiState = TaxiState.Moving;
                isTaxing = true; 
            }
        }
        if (taxiState == TaxiState.Moving)
        {
            // if (tracker.currentPoint > tracker.track.pathPoints.Count - 2 && controller.gearHelper != null && controller.gearHelper.brakeState == SilantroGearSystem.BrakeState.Disengaged)
            // {
            //     taxiState = TaxiState.Holding;
            //     targetTaxiSpeed = 0; brakeInput = 0f;
            //     if (controller.gearHelper != null) { controller.ToggleBrakeState(); }
            //     isTaxing = false;

            //     //--------------- Set Takeoff Parameters
            //     takeoffHeading = computer.currentHeading;
            // }
            if (pathFollowing.CurrentIndex >= pathFollowing.Path.Count - 1 && controller.gearHelper != null && controller.gearHelper.brakeState == SilantroGearSystem.BrakeState.Disengaged)
            {
                taxiState = TaxiState.Holding;
                targetTaxiSpeed = 0; 
                brakeInput = 0f;
                if (controller.gearHelper != null) { controller.gearHelper.EngageBrakes(); }
                isTaxing = false;
            
                //--------------- Set Takeoff Parameters
                takeoffHeading = computer.currentHeading;
            }
            else
            {
                targetTaxiSpeed = pathFollowing.CalculateSpeed() * maximumTaxiSpeed * 0.6f / 1.94384f;
                isTaxing = true;
            }
            // else
            // {
            //     if(tracker.currentPoint > tracker.track.pathPoints.Count - 6 && tracker.currentPoint < tracker.track.pathPoints.Count - 4) { targetTaxiSpeed = (0.9f*maximumTaxiSpeed)/ 1.94384f; } //96
            //     else if(tracker.currentPoint > tracker.track.pathPoints.Count - 5 && tracker.currentPoint < tracker.track.pathPoints.Count - 3) { targetTaxiSpeed = (0.5f * maximumTaxiSpeed) / 1.94384f; } //97
            //     else if (tracker.currentPoint > tracker.track.pathPoints.Count - 4 && tracker.currentPoint < tracker.track.pathPoints.Count - 2) { targetTaxiSpeed = (0.25f * maximumTaxiSpeed) / 1.94384f; } //98
            //     else if (tracker.currentPoint > tracker.track.pathPoints.Count - 3 && tracker.currentPoint < tracker.track.pathPoints.Count - 1) { targetTaxiSpeed = (0.1f * maximumTaxiSpeed) / 1.94384f; } //98
            //     else { targetTaxiSpeed = steerCurve.Evaluate(steeringAngle) / 1.94384f; }

            //     isTaxing = true;
            // }
        }
        if(taxiState == TaxiState.Holding)
        {
            // -------------------------- Perform function while on hold

            // -------------------------- Receive clearance
            if (clearedForTakeoff) { flightState = FlightState.Takeoff; }
        }


        



        if (isTaxing)
        {
            // ------------------------------------- Speed Control
            float speedError = (targetTaxiSpeed - currentSpeed) * 1.94384f;
            if(computer.autoThrottle == SilantroFlightComputer.ControlState.Off) { computer.autoThrottle = SilantroFlightComputer.ControlState.Active; }
            // if(speedError > 5 && tracker.currentPoint < tracker.track.pathPoints.Count - 2)
            if(speedError > 5 && pathFollowing.CurrentIndex < pathFollowing.Path.Count - 1)
            {
                // ---------------- Auto Throttle
                float presetSpeed = targetTaxiSpeed;
                computer.speedError = (presetSpeed - currentSpeed);
                computer.processedThrottle = computer.throttleSolver.CalculateOutput(computer.speedError, computer.timeStep);
                if (float.IsNaN(computer.processedThrottle) || float.IsInfinity(computer.processedThrottle)) { computer.processedThrottle = 0f; }
            }
            else { computer.processedThrottle = 0f; }



            // ------------------------------------- Point
            // tracker.UpdateTrack();


            // ------------------------------------- Steer
            float taxiSpeed = controller.transform.InverseTransformDirection(controller.aircraft.velocity).z;
            brakeInput = -1 *Mathf.Clamp((targetTaxiSpeed - currentSpeed) * brakeSensitivity, -1, 0);
            // Vector3 offsetTargetPos = tracker.target.position;
            Vector3 offsetTargetPos = pathFollowing.Path[pathFollowing.CurrentIndex].position; //tracker.target.position;
            Vector3 localTarget = controller.transform.InverseTransformPoint(offsetTargetPos);
            float targetAngle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
            float steer = Mathf.Clamp(targetAngle * steerSensitivity, -1, 1) * Mathf.Sign(taxiSpeed);
            steeringAngle = Mathf.Lerp(maximumSteeringAngle, minimumSteeringAngle, Mathf.Abs(taxiSpeed) * 0.015f) * steer;


            // ------------------------------------- Send
            if (controller.gearHelper != null)
            {
                controller.gearHelper.brakeInput = brakeInput;
                controller.gearHelper.currentSteerAngle = steeringAngle;
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------------------------------------------------
    public void TakeoffClearance()
    {
        if(flightState == FlightState.Taxi && taxiState == TaxiState.Holding)
        {
            if (!clearedForTakeoff) { clearedForTakeoff = true; }
            else { Debug.Log(controller.transform.name + " has been cleared for takeoff"); }
        }
        else 
        { 
            Debug.Log(controller.transform.name + " clearance invalid! Aircraft not in holding pattern"); 
        }
        computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
        Debug.Log("Clicking!");
    }

    // -------------------------------------------TAKEOFF----------------------------------------------------------------------------------------------------------
    void TakeoffMode()
    {
        if (controller.engineRunning)
        {
            //------Accelerate
            if (controller.gearHelper) { controller.gearHelper.ReleaseBrakes(); }
            computer.processedThrottle = Mathf.Lerp(computer.processedThrottle, 1.05f, Time.deltaTime);
            if (computer.processedThrottle > 1f && controller.input.boostState == SilantroInput.BoostState.Off) { controller.input.EngageBoost(); }

            // ------------------------------------- Send
            if (controller.gearHelper != null)
            {
                controller.gearHelper.brakeInput = brakeInput = 0f;
                controller.gearHelper.currentSteerAngle = steeringAngle = 0f;
            }


            #region Pitch Control
            if (computer.knotSpeed < takeoffSpeed)
            {
                computer.commandPitchRate = 0f;
                computer.pitchRateError = computer.pitchRate - computer.commandPitchRate;
                computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
            }
            else
            {
                // -------------------------------------------- Pitch Rate Required
                computer.pitchAngleError = computer.pitchAngle - (-1f * climboutPitchAngle);
                computer.pitchAngleSolver.minimum = -computer.climboutPitchRate; computer.pitchAngleSolver.maximum = computer.climboutPitchRate;
                computer.commandPitchRate = computer.pitchAngleSolver.CalculateOutput(computer.pitchAngleError, computer.timeStep);

                computer.pitchRateError = computer.pitchRate - computer.commandPitchRate;
                computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
            }
            #endregion Pitch Control

            #region Roll/Yaw Control
            computer.yawAngleError = takeoffHeading - computer.yawAngle;
            computer.yawAngleSolver.minimum = -computer.balanceYawRate; computer.yawAngleSolver.maximum = computer.balanceYawRate;
            computer.commandYawRate = computer.yawAngleSolver.CalculateOutput(computer.yawAngleError, computer.timeStep);
            computer.yawRateError = computer.commandYawRate - computer.yawRate;
            computer.processedYaw = computer.yawRateSolver.CalculateOutput(computer.yawRateError, computer.timeStep);

            computer.commandBankAngle = 0f;
            computer.rollAngleError = computer.rollAngle - (-1f * computer.commandBankAngle);
            computer.rollAngleSolver.minimum = -computer.balanceRollRate; computer.rollAngleSolver.maximum = computer.balanceRollRate;
            computer.commandRollRate = computer.rollAngleSolver.CalculateOutput(computer.rollAngleError, computer.timeStep);
            computer.rollRateError = computer.commandRollRate - computer.rollRate;
            computer.processedRoll = computer.rollRateSolver.CalculateOutput(computer.rollRateError, computer.timeStep);
            #endregion Roll/Yaw Control

            if (computer.knotSpeed > computer.maximumGearSpeed - 10f) { computer.StartCoroutine(PostTakeoffCheckList()); }
        }
    }





    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    IEnumerator PostTakeoffCheckList()
    {
        // --------------------------- Flaps
        yield return new WaitForSeconds(checkListTime);
        foreach (SilantroAerofoil foil in controller.wings) { if (foil.flapSetting != 0) { foil.SetFlaps(0, 1); } }

        // --------------------------- Gear
        if (computer.gearSolver)
        {
            yield return new WaitUntil(() => computer.knotSpeed > computer.maximumGearSpeed && computer.ftHeight > 50f);
            computer.gearSolver.DisengageActuator();
        }

        // ---------------------------- Transition
        yield return new WaitForSeconds(transitionTime);
        foreach (SilantroAerofoil foil in computer.wingFoils) { if (foil.slatState == SilantroAerofoil.ControlState.Active) { foil.baseSlat = Mathf.MoveTowards(foil.baseSlat, 0f, foil.slatActuationSpeed * Time.fixedDeltaTime); } }
        //Debug.Log("Now Cruising!!!");
        flightState = FlightState.Cruise;
    }







    // -------------------------------------------CRUISE----------------------------------------------------------------------------------------------------------
    void CruiseMode()
    {
        if (Mathf.Abs(cruiseSpeed - computer.knotSpeed) > 2) { computer.autoThrottle = SilantroFlightComputer.ControlState.Active; }
        else { computer.autoThrottle = SilantroFlightComputer.ControlState.Off; }

        // velocity of the rigidbody of the aircraft (direction of movement)
        Vector3 temp = aircraft.velocity.normalized;
        // yawAngle is the rotation value aspect to y axis(world coordinate
        float x = Mathf.Sin(computer.yawAngle * Mathf.Deg2Rad);
        float z = Mathf.Cos(computer.yawAngle * Mathf.Deg2Rad);

        //Vector3 Model_Facing_Axis = new Vector3(
        //    Mathf.Cos(computer.pitchAngle * Mathf.Deg2Rad) * x,
        //    -Mathf.Sin(computer.pitchAngle * Mathf.Deg2Rad),
        //    Mathf.Cos(computer.pitchAngle * Mathf.Deg2Rad) * z
        //);
        //Vector3 Model_Side_Axis = new Vector3(
        //    Mathf.Cos(computer.rollAngle * Mathf.Deg2Rad) * z,
        //    Mathf.Sin(computer.rollAngle * Mathf.Deg2Rad),
        //    Mathf.Cos(computer.rollAngle * Mathf.Deg2Rad) * -x
        //);
        //Vector3 Model_Up = Vector3.Cross(Model_Facing_Axis, Model_Side_Axis);

        var Model_Side_Axis = computer.transform.TransformDirection(new Vector3(1.0f, 0.0f, 0.0f));
        var Model_Up = computer.transform.TransformDirection(new Vector3(0.0f, 1.0f, 0.0f));
        var Model_Facing_Axis = computer.transform.TransformDirection(new Vector3(0.0f, 0.0f, 1.0f));       
        //Debug.Log("model facing:" + Model_Facing_Axis.normalized.ToString("F4"));
        //Debug.Log("local z:" + computer_local_z_axis.normalized.ToString("F4"));

        //Debug.Log("model side:" + Model_Side_Axis.normalized.ToString("F4"));
        //Debug.Log("local x:" + computer_local_x_axis.normalized.ToString("F4"));

        //Debug.Log("model up:" + Model_Up.normalized.ToString("F4"));
        //Debug.Log("local y:" + computer_local_y_axis.normalized.ToString("F4"));


        // ---------------- Auto Throttle
        float presetSpeed = cruiseSpeed / MathBase.toKnots;
        computer.speedError = (presetSpeed - currentSpeed);
        computer.processedThrottle = computer.throttleSolver.CalculateOutput(computer.speedError, computer.timeStep);
        if (float.IsNaN(computer.processedThrottle) || float.IsInfinity(computer.processedThrottle)) { computer.processedThrottle = 0f; }

        // ----------------- Boost e.g Piston Turbo or Turbine Reheat
        if (computer.processedThrottle > 1f && computer.speedError > 5f && controller.input.boostState == SilantroInput.BoostState.Off) { controller.input.EngageBoost(); }
        if (computer.processedThrottle < 1f && computer.speedError < 3f && controller.input.boostState == SilantroInput.BoostState.Active) { controller.input.DisEngageBoost(); }



        // ----------------- Boost Deceleration with Speedbrake
        /*
        if (computer.speedBrakeSolver != null && computer.overideSpeedbrake && computer.ftHeight > 500f)
        {
            if (computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Disengaged && computer.speedError < -30f) { computer.speedBrakeSolver.EngageActuator(); }
            if (computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Engaged && computer.speedError > -20f) { computer.speedBrakeSolver.DisengageActuator(); }
        }
        */

        // ---------------------------------------------------------------------------- Roll
        float presetHeading = cruiseHeading; if (presetHeading > 180) { presetHeading -= 360f; }
        computer.headingError = presetHeading - computer.currentHeading;
        // change max turning rate based on cruiseHeading
        //float currMaxTurnRate = computer.maximumTurnRate * Mathf.Clamp((Math.Abs(cruiseHeading) / 180.0f) + 0.5f, 0.8f, 2.0f) * cruiseHeadingTurnFactor;

        //computer.turnSolver.maximum = currMaxTurnRate;
        //computer.turnSolver.minimum = -currMaxTurnRate;
        computer.turnSolver.maximum = computer.maximumTurnRate;
        computer.turnSolver.minimum = -computer.maximumTurnRate;
        computer.commandTurnRate = computer.turnSolver.CalculateOutput(computer.headingError, computer.timeStep);


        // ---------------------------------------------- Calculate Required Bank
        float rollFactor = (computer.commandTurnRate * currentSpeed * MathBase.toKnots) / 1091f;
        computer.commandBankAngle = Mathf.Atan(rollFactor) * Mathf.Rad2Deg;
        float currMaxTurnBank = computer.maximumTurnBank;
        if (Math.Abs(computer.headingError) > computer.minimumAccerHeading)
        {
            currMaxTurnBank = computer.maximumAccerTurnBank;
        }
        else if (Math.Abs(computer.headingError) < computer.maximumDeccerHeading)
        {
            currMaxTurnBank = computer.maximumDeccerTurnBank;
        }

        if (computer.commandBankAngle > currMaxTurnBank) 
        { 
            computer.commandBankAngle = currMaxTurnBank; 
        }
        if (computer.commandBankAngle < -currMaxTurnBank) 
        { 
            computer.commandBankAngle = -currMaxTurnBank; 
        }

        // -------------------------------------------- Roll Rate Required
        // computer.rollAngle is the final roll angle that we want to achieve
        computer.rollAngleError = computer.rollAngle - (-1f * computer.commandBankAngle);
        //Debug.Log(computer.rollAngleError);
        computer.rollAngleSolver.minimum = -computer.balanceRollRate; computer.rollAngleSolver.maximum = computer.balanceRollRate;
        computer.commandRollRate = computer.rollAngleSolver.CalculateOutput(computer.rollAngleError, computer.timeStep);

        computer.rollRateError = computer.commandRollRate - computer.rollRate;
        computer.processedRoll = computer.rollRateSolver.CalculateOutput(computer.rollRateError, computer.timeStep);

        // -------------------------------------------- Yaw Rate Required
        
        temp = Vector3.ProjectOnPlane(temp, Model_Up);
        
        float yawError = (float)Math.Round(Vector3.SignedAngle(temp, Model_Facing_Axis, Model_Up), 3) * 1.5f;
        computer.yawAngleError = yawError;
        computer.commandYawRate = computer.yawRateSolver.CalculateOutput(computer.yawAngleError, computer.timeStep);

        computer.yawAngleError = computer.commandYawRate - computer.yawRate;
        computer.processedYaw = computer.yawRateSolver.CalculateOutput(computer.yawAngleError, computer.timeStep); 
        //Debug.Log(yawError + " " + computer.commandYawRate + " " + computer.yawRate + " " + computer.processedYaw);
        
        // --------------------------------------------------------------------------- Pitch
        // ------------------------------------------------ Altitude Hold
        computer.altitudeError = cruiseAltitude / MathBase.toFt - computer.currentAltitude;
        float presetClimbLimit = cruiseClimbRate / MathBase.toFtMin;
        computer.altitudeSolver.maximum = presetClimbLimit;
        computer.altitudeSolver.minimum = -presetClimbLimit;
        computer.commandClimbRate = computer.altitudeSolver.CalculateOutput(computer.altitudeError, computer.timeStep) * MathBase.toFtMin;

        // -------------------------------------------- Pitch Rate Required
        float presetClimbRate = computer.commandClimbRate / MathBase.toFtMin;
        computer.climbRateError = presetClimbRate - computer.currentClimbRate;
        computer.climbSolver.minimum = -computer.balancePitchRate * 0.2f; computer.climbSolver.maximum = computer.balancePitchRate * 0.5f;
        computer.commandPitchRate = computer.climbSolver.CalculateOutput(computer.climbRateError, computer.timeStep);

        computer.pitchRateError = computer.pitchRate - computer.commandPitchRate;
        computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
    }

    void AkwardCruiseMode()
    {
        // ignore all physical simulation
        if (useAkwardCurve)
        {
            aircraft.isKinematic = false;
            float currT = Mathf.Clamp(Time.deltaTime, 0.0f, 0.0333f);
            if(Math.Abs(akwardCurveTime) <= 0.001f || akwardCurveTime + currT > 1.0f)
            {
                curveStartPostion = aircraft.transform.position;
            }
            akwardCurveTime = 1.0f % (akwardCurveTime + currT);
            Vector3 currPos = akwardCurve.GetCurrentPoint(akwardCurveTime);
            Vector3 currDir = akwardCurve.GetCurrentDerivative(akwardCurveTime);
            // change postion
            aircraft.transform.position = currPos + curveStartPostion;

            // change direction
            //var Model_Up = computer.transform.TransformDirection(new Vector3(0.0f, 1.0f, 0.0f));
            //var targetDir = aircraft.transform.TransformDirection(currDir);
            //aircraft.transform.rotation = Quaternion.LookRotation(targetDir, Model_Up);
        }
        else
        {
            CruiseMode();
        }
    }
}

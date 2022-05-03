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
    public enum FlightState { Grounded, Taxi, Takeoff, Cruise, Loiter, Descent, Landing }
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
    public float recommendedTaxiSpeed = 12f;
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
    public float cruiseSpeed = 250f;
    public float cruiseHeading = -90f;
    public float cruiseClimbRate = 2200f / MathBase.toFtMin;
    public float cruiseHeadingTurnFactor = 1.25f;
    public float descentSpeed = 120f;
    public float landingSpeed = 50f;
    public float landingAltitude = 0f;


    public float cruiseAccerBankAngle = 30f;
    public float cruiseDeccerBankAngle = 15f;


    //---------------------------------- New
    // public Transform pathContainer;
    public List<Transform> path = new List<Transform>();
    public int path_currentIndex = 0;
    private int path_cur = 0;
    private PathFollowing pathFollowing;
    private int timer = 0;
    private Vector3 World_Up = new Vector3(0,1,0);
    public Transform landing_track;

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
        // AI customized path
        pathFollowing = new PathFollowing();
        pathFollowing.Init(controller.transform);
        landing_track = GameObject.Find("Landing Track A").transform;
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
        timer = 25;
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
        timer = 25;
        cruiseHeading += 2.5f;
        Debug.Log("Test3" + cruiseHeading);
    }

    public void test5()
    {
        cruiseAltitude += 100;
        Debug.Log("Test5  " + cruiseAltitude);
    }
    public void test4()  // test 1
    {
        Transform pathContainer = GameObject.Find("Flight Track C").transform;
        path.Clear();
        path_currentIndex = 0;
        for (int i = 0; i < pathContainer.childCount; i++)
        {
            path.Add(pathContainer.GetChild(i));
        }
        Debug.Log("Loaded!" + path.Count);
    }
    public void test6()  // test 2
    {
        Transform pathContainer = GameObject.Find("Flight Track B").transform;
        path.Clear();
        path_currentIndex = 0;
        for (int i = 0; i < pathContainer.childCount; i++)
        {
            path.Add(pathContainer.GetChild(i));
        }
        Debug.Log("Loaded!" + path.Count);
    }
    public void test7()  //test 3
    {
        Transform pathContainer = GameObject.Find("Flight Track E").transform;
        path.Clear();
        path_currentIndex = 0;
        for (int i = 0; i < pathContainer.childCount; i++)
        {
            path.Add(pathContainer.GetChild(i));
        }
        Debug.Log("Loaded!" + path.Count);
    }
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
            case FlightState.Cruise: CruiseMode(); break;
            case FlightState.Descent: CruiseMode();  break;
            case FlightState.Landing: LandingMode();  break;
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

        // yield return new WaitForSeconds(evaluateTime);
        checkedSurfaces = true;checkingSurfaces = false;
        computer.processedPitch = 0f;
        computer.processedRoll = 0f;
        computer.processedYaw = 0f;
        computer.processedStabilizerTrim = 0f;
                
        groundChecklistComplete = true;

        // ---------------------------- Transition
        yield return new WaitForSeconds(transitionTime);
        flightState = FlightState.Taxi;
        takeoffHeading = computer.currentHeading;
        flightState = FlightState.Takeoff;
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
                targetTaxiSpeed = pathFollowing.CalculateSpeed() * maximumTaxiSpeed * 1f / 1.94384f;
                isTaxing = true;
            }
        }
        if(taxiState == TaxiState.Holding)
        {
            if (clearedForTakeoff) { flightState = FlightState.Takeoff; }
        }

        if (isTaxing)
        {
            // ------------------------------------- Speed Control
            float speedError = (targetTaxiSpeed - currentSpeed) * 1.94384f;
            Debug.Log(speedError);
            if(computer.autoThrottle == SilantroFlightComputer.ControlState.Off) { computer.autoThrottle = SilantroFlightComputer.ControlState.Active; }
            // if(speedError > 5 && tracker.currentPoint < tracker.track.pathPoints.Count - 2)
            if( pathFollowing.CurrentIndex < pathFollowing.Path.Count - 1)
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
                computer.processedPitch = -0.4f;
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
        flightState = FlightState.Cruise;  cruiseAltitude = 1000f;  cruiseSpeed = 250f;
    }







    // -------------------------------------------CRUISE----------------------------------------------------------------------------------------------------------
    void CruiseMode()
    {
        if (Mathf.Abs(cruiseSpeed - computer.knotSpeed) > 2) { computer.autoThrottle = SilantroFlightComputer.ControlState.Active; }
        else { computer.autoThrottle = SilantroFlightComputer.ControlState.Off; }

        // velocity of the rigidbody of the aircraft (direction of movement)
        Vector3 temp = aircraft.velocity.normalized;
        // yawAngle is the rotation value aspect to y axis(world coordinate
        Vector3 D_Model_Facing = new Vector3(Mathf.Sin(computer.yawAngle*Mathf.Deg2Rad), 0, Mathf.Cos(computer.yawAngle*Mathf.Deg2Rad));

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
        /*
        Vector3 Destination = new Vector3(
                Mathf.Sin(cruiseHeading * Mathf.Deg2Rad) * 300f,
                cruiseAltitude/ MathBase.toFt - computer.transform.position[1],
                Mathf.Cos(cruiseHeading * Mathf.Deg2Rad) * 300f
                );
                */
        var Model_Side_Axis = computer.transform.TransformDirection(new Vector3(1.0f, 0.0f, 0.0f));
        var Model_Up = computer.transform.TransformDirection(new Vector3(0.0f, 1.0f, 0.0f));
        var Model_Facing_Axis = computer.transform.TransformDirection(new Vector3(0.0f, 0.0f, 1.0f));
        
        Vector3 Destination;
        if(path_currentIndex < path.Count)
            Destination = path[path_currentIndex].position - computer.transform.position;
        else
        {
            if (flightState != FlightState.Descent)
            {
                Debug.Log(Mathf.Abs(landing_track.GetChild(0).position[1] - computer.transform.position[1]));
                if(Mathf.Abs(landing_track.GetChild(0).position[1] - computer.transform.position[1]) > 1200)
                {
                    Vector3 intermediate =  D_Model_Facing*2000 + path[path_currentIndex - 1].position;
                    intermediate[1] = Mathf.Max(600f, (computer.transform.position[1])/3);
                    landing_track.GetChild(0).position = intermediate;
                    Debug.Log("intermediate!" + intermediate);
                    Debug.Log("intermediate!" + intermediate);
                    Debug.Log("intermediate!" + intermediate);
                }
                descentSpeed = 160f;
                flightState = FlightState.Descent;
                path.Clear();
                path_currentIndex = 0;
                for (int i = 0; i < landing_track.childCount; i++)
                {
                    path.Add(landing_track.GetChild(i));
                }
                Destination = path[path_currentIndex].position - computer.transform.position;
                flightState = FlightState.Descent;
                Debug.Log("Loaded!" + path.Count + " " + cruiseSpeed + flightState);
            }
            else
            {
                path.Clear();
                flightState = FlightState.Landing;
                Destination = new Vector3(
                Mathf.Sin(-180 * Mathf.Deg2Rad) * 200f,
                 - computer.transform.position[1],
                Mathf.Cos(-180 * Mathf.Deg2Rad) * 200f
                );
                if (computer.gearSolver)
                {
                    computer.gearSolver.EngageActuator();
                }
                if ( computer.speedBrakeSolver != null && computer.overideSpeedbrake )
                {
                    if (computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Disengaged) { computer.speedBrakeSolver.EngageActuator(); }
                }
            }
        }

        if(Destination.sqrMagnitude < 40000f)
        {
            descentSpeed = 140f;
            if(flightState != FlightState.Descent ) timer = 15;
            else
            {
                if (computer.gearSolver && currentSpeed < 100f)
                {
                    computer.gearSolver.EngageActuator();
                }
            }
            ++path_currentIndex;
            
            //if(path_currentIndex < path.Count)
            //{
            //    Destination = new Vector3(
            //    Mathf.Sin(computer.currentHeading * Mathf.Deg2Rad) * 300f,
            //    path[path_currentIndex].position[1] - computer.transform.position[1],
            //    Mathf.Cos(computer.currentHeading * Mathf.Deg2Rad) * 300f
            //    );
            //}
        }



        float yawAngleDiff = Vector3.SignedAngle( new Vector3(Destination[0], 0 , Destination[2]),D_Model_Facing, World_Up);
        // ---------------- Auto Throttle
        float presetSpeed;
        if (flightState != FlightState.Descent && flightState != FlightState.Landing)
            presetSpeed = cruiseSpeed / MathBase.toKnots;
        else
            presetSpeed = descentSpeed / MathBase.toKnots;
        float spError = (presetSpeed - currentSpeed);
        //Debug.Log(path_currentIndex + " " + Destination.sqrMagnitude + " " + spError + " " + presetSpeed + " " + descentSpeed);
        if(Mathf.Abs(spError) > 2f)
        {
            computer.speedError = spError;
            computer.processedThrottle = computer.throttleSolver.CalculateOutput(computer.speedError, computer.timeStep);
            if (float.IsNaN(computer.processedThrottle) || float.IsInfinity(computer.processedThrottle)) { computer.processedThrottle = 0f; }
        }
        else
            computer.processedThrottle = 0.84f;
        // ----------------- Boost e.g Piston Turbo or Turbine Reheat
        if (computer.processedThrottle > 1f && computer.speedError > 5f && controller.input.boostState == SilantroInput.BoostState.Off) { controller.input.EngageBoost(); }
        if (computer.processedThrottle < 1f && computer.speedError < 3f && controller.input.boostState == SilantroInput.BoostState.Active) { controller.input.DisEngageBoost(); }



        // ----------------- Boost Deceleration with Speedbrake
        
        if ( computer.speedBrakeSolver != null && computer.overideSpeedbrake)
        {
            if (computer.currentClimbRate < -2f && computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Engaged && computer.speedError > -25f) { computer.speedBrakeSolver.DisengageActuator();Debug.Log("111");}
            else if (computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Disengaged && computer.speedError < -30f) { computer.speedBrakeSolver.EngageActuator(); Debug.Log("222");}
        }
        

        // ---------------------------------------------------------------------------- Roll
        float presetHeading = cruiseHeading; if (presetHeading > 180) { presetHeading -= 360f; }
        computer.headingError = presetHeading - computer.currentHeading;
        // change max turning rate based on cruiseHeading
        //float currMaxTurnRate = computer.maximumTurnRate * Mathf.Clamp((Math.Abs(cruiseHeading) / 180.0f) + 0.5f, 0.8f, 2.0f) * cruiseHeadingTurnFactor;

        computer.turnSolver.maximum = computer.maximumTurnRate;
        computer.turnSolver.minimum = -computer.maximumTurnRate;
        computer.commandTurnRate = computer.turnSolver.CalculateOutput(computer.headingError, computer.timeStep);


        // ---------------------------------------------- Calculate Required Bank
        
        Vector3 rollHelper = Destination.normalized;
        rollHelper = Vector3.ProjectOnPlane(rollHelper, Model_Up);
        float rollFactor = (currentSpeed * MathBase.toKnots) / 150f;
        float rollError =  (float)Math.Round(Vector3.SignedAngle(rollHelper, Model_Facing_Axis, Model_Up), 4);
        
        float test2 = rollError;
        rollError *= Mathf.Abs(rollError) *Mathf.Abs(rollError) *rollFactor/20f;

        // ----w---------------------------------------- Roll Rate Required
        // computer.rollAngle is the final roll angle that we want to achieve
        if(Mathf.Abs(rollError) < 0.8f) rollError = 0f;
        else if (Mathf.Abs(rollError) > 89.9f) rollError = Mathf.Abs(yawAngleDiff)/yawAngleDiff * 89f;
        computer.rollAngleError = computer.rollAngle -rollError;  //computer.rollAngle - (-1f * computer.commandBankAngle);

        computer.rollAngleSolver.minimum = -computer.balanceRollRate; 
        computer.rollAngleSolver.maximum = computer.balanceRollRate;
        computer.commandRollRate = computer.rollAngleSolver.CalculateOutput(computer.rollAngleError, computer.timeStep);

        computer.rollRateError = computer.commandRollRate - computer.rollRate;
        computer.processedRoll = computer.rollRateSolver.CalculateOutput(computer.rollRateError, computer.timeStep);
        if(timer != 0) timer--;
        // -------------------------------------------- Yaw Rate Required
        float yawDiff = 0f;
        float pitchDiff = 0f;
        if(timer == 0)
        {
            Vector3 yawHelper = Destination.normalized;
            Vector3 pitchHelper = Vector3.ProjectOnPlane(yawHelper, Model_Side_Axis);
            yawHelper = Vector3.ProjectOnPlane(yawHelper, Model_Up);
            yawHelper = Vector3.ProjectOnPlane(yawHelper, Model_Facing_Axis);
            yawDiff =  Mathf.Max(yawHelper[0]/Model_Side_Axis[0], yawHelper[2]/Model_Side_Axis[2]);
            if (Mathf.Abs(yawDiff) > 1.5f || Mathf.Abs(yawDiff) < 0.01f) yawDiff = 0f;

            pitchHelper = Vector3.ProjectOnPlane(pitchHelper, Model_Facing_Axis);
            pitchDiff =Mathf.Max(pitchHelper[0]/Model_Up[0], pitchHelper[1]/Model_Up[1]);
        }
        
        computer.yawAngleError = 16 * yawDiff;
        computer.processedYaw = computer.yawRateSolver.CalculateOutput(computer.yawAngleError, computer.timeStep); 
        
        // --------------------------------------------------------------------------- Pitch
        // ------------------------------------------------ Altitude Hold
        //temp = Vector3.ProjectOnPlane(temp, Model_Side_Axis);
        //float yawError = (float)Math.Round(Vector3.SignedAngle(temp, Model_Facing_Axis, Model_Up), 3) * 2f;
        
        computer.pitchRateError =  pitchDiff * -7 - 0.01f;
        computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
        
        /*
        computer.altitudeError = cruiseAltitude / MathBase.toFt - computer.currentAltitude;

        computer.altitudeSolver.maximum = cruiseClimbRate;
        computer.altitudeSolver.minimum = -cruiseClimbRate;
        computer.commandClimbRate = computer.altitudeSolver.CalculateOutput(computer.altitudeError, computer.timeStep) * MathBase.toFtMin;

        // -------------------------------------------- Pitch Rate Required
        float presetClimbRate = computer.commandClimbRate / MathBase.toFtMin;
        computer.climbRateError = presetClimbRate - computer.currentClimbRate;
        computer.climbSolver.minimum = -computer.balancePitchRate * 0.2f; computer.climbSolver.maximum = computer.balancePitchRate * 0.5f;
        computer.commandPitchRate = computer.climbSolver.CalculateOutput(computer.climbRateError, computer.timeStep);
        
        computer.pitchRateError = computer.pitchRate - computer.commandPitchRate;
        computer.processedPitch = computer.pitchRateSolver.CalculateOutput(computer.pitchRateError, computer.timeStep);
        */
    }
    void LandingMode()
    {
        if (computer.gearSolver && computer.gearSolver.actuatorState == SilantroActuator.ActuatorState.Disengaged)
        {
            computer.gearSolver.EngageActuator();
        }
        computer.processedThrottle = 0.1f;

        // ----------------- Boost Deceleration with Speedbrake
        
        if ( computer.speedBrakeSolver != null && computer.overideSpeedbrake )
        {
            if (computer.speedBrakeSolver.actuatorState == SilantroActuator.ActuatorState.Disengaged) { computer.speedBrakeSolver.EngageActuator(); }
        }
        //if (controller.gearHelper && controller.gearHelper.brakeState == SilantroGearSystem.BrakeState.Disengaged) { controller.gearHelper.EngageBrakes(); }

        computer.rollRateError = - computer.rollRate;
        computer.processedRoll = computer.rollRateSolver.CalculateOutput(computer.rollRateError, computer.timeStep);
        
        
        foreach (SilantroAerofoil foil in controller.wings)
        {
            if (foil.flapSetting != 1 && !flapSet && foil.flapAngleSetting == SilantroAerofoil.FlapAngleSetting.ThreeStep) { foil.SetFlaps(1, 1); }
            if (foil.flapSetting != 2 && !flapSet && foil.flapAngleSetting == SilantroAerofoil.FlapAngleSetting.FiveStep) { foil.SetFlaps(2, 1); }
        }
        computer.processedPitch = -0.5f * (90f / currentSpeed);
        if(computer.transform.position[1] < 3.8f) computer.processedPitch = 0f;
        
        computer.yawAngleError = 0f;
        computer.processedYaw = computer.yawRateSolver.CalculateOutput(computer.yawAngleError, computer.timeStep); 
        if(computer.transform.position[1] < 4f && computer.currentSpeed < 100f)
        {
            float taxiSpeed = controller.transform.InverseTransformDirection(controller.aircraft.velocity).z;
            brakeInput = 0.1f;
            Vector3 offsetTargetPos = new Vector3(66,2,-1500);
            Vector3 localTarget = controller.transform.InverseTransformPoint(offsetTargetPos);
            float targetAngle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
            float steer = Mathf.Clamp(targetAngle * steerSensitivity, -1, 1) * Mathf.Sign(taxiSpeed);
            steeringAngle = Mathf.Lerp(maximumSteeringAngle, minimumSteeringAngle, Mathf.Abs(taxiSpeed) * 0.015f) * steer;

            if (controller.gearHelper != null)
            {
                controller.gearHelper.brakeInput = brakeInput;
                controller.gearHelper.currentSteerAngle = steeringAngle;
            }
        }
        if (computer.currentSpeed < 2f)
        {
            Debug.Log("Ready To take off Again!");
            flightState = FlightState.Grounded;
        }
    }

}


using System;
using UnityEngine;
using UnityEngine.InputSystem;

public enum VehicleState
{
    Idle,
    Accelerating,
    Cruising,
    Braking,
    Reversing,
    Airborne
}

public class PrometeoCarController : MonoBehaviour
{
    [Header("Vehicle Dynamics")]
    public VehicleState currentState = VehicleState.Idle;
    public Vector3 bodyMassCenter = new Vector3(0, -0.45f, 0);

    [Header("Engine")]
    public float idleRPM = 850f;
    public float redlineRPM = 6200f;
    public float engineInertia = 0.25f;
    public float engineDrag = 80f; // Nm of resistance
    public float peakTorque = 430f; // Nm
    public float stallRPM = 2100f; // Launch/Torque converter stall RPM
    public AnimationCurve torqueCurve = new AnimationCurve(
        new Keyframe(800, 0.70f), // Boosted V8 bottom end
        new Keyframe(1500, 0.80f),
        new Keyframe(2800, 0.95f),
        new Keyframe(3600, 1.00f),
        new Keyframe(5200, 0.88f),
        new Keyframe(6200, 0.72f)
    );
    public bool useRevLimiter = true;

    [Header("Transmission")]
    public bool automatic = true;
    public float finalDriveRatio = 3.73f;
    public float[] gearRatios = { 2.87f, 1.89f, 1.28f, 1.00f };
    public float reverseRatio = 2.90f;
    public float shiftUpRPM = 5800f;
    public float shiftDownRPM = 2400f; // Increased from 1800 to prevent lugging
    public float shiftTime = 0.45f; // Increased for realistic old-school auto feel

    [Header("Drivetrain Feel")]
    public float drivetrainElasticity = 0.12f; // Lowers jerk and adds "weight" feel
    public float lockRecoveryTime = 0.9f;

    [Header("Torque Converter")]
    public float converterMaxFactor = 1.75f;
    public float converterFadeSpeed = 18f;
    public int converterGear = 1;

    [Header("Automatic Behavior")]
    public float idleCreepTorque = 90f;
    public float creepMaxSpeed = 8f; // km/h

    [Header("Resistance")]
    public float rollingResistance = 12.5f;
    public float aerodynamicDrag = 0.32f;
    public float frontalArea = 2.05f;

    [Header("Steering")]
    public float maxSteeringAngle = 32f;
    public AnimationCurve steeringVsSpeed = new AnimationCurve(new Keyframe(0, 1f), new Keyframe(200, 0.2f));
    public float steeringSpeed = 0.5f;

    [Header("Brakes")]
    public float maxBrakeTorque = 4200f;
    public float handbrakeTorque = 6500f;
    [Range(0, 1)]
    public float brakeBias = 0.65f;

    [Header("Chassis Dynamics")]
    public float weightTransfer = 0.22f;
    public float antiRollStiffness = 6500f;

    [Header("Stability & Assists")]
    public float gripMultiplier = 1.15f;
    [Range(0, 1)]
    public float tractionControl = 0.15f;

    [Header("Wheels")]
    public GameObject frontLeftMesh;
    public WheelCollider frontLeftCollider;
    public GameObject frontRightMesh;
    public WheelCollider frontRightCollider;
    public GameObject rearLeftMesh;
    public WheelCollider rearLeftCollider;
    public GameObject rearRightMesh;
    public WheelCollider rearRightCollider;

    [Header("Inputs")]
    public InputActionAsset inputActions;
    public bool useExternalInput = false;
    public float externalSteering = 0f;
    public float externalAcceleration = 0f;

    [Header("Telemetry")]
    public VehicleTelemetry telemetry = new VehicleTelemetry();

    // Internal state
    Rigidbody rb;
    float steeringAxis;
    float throttleInput;
    float brakeInput;
    float steeringInput;
    bool handbrakePressed;
    
    InputAction steeringAction;
    InputAction throttleAction;
    InputAction brakeAction;
    InputAction handbrakeAction;
    InputAction gearShiftAction;

    Engine engine;
    Transmission transmission;
    Drivetrain drivetrain;

    void Awake()
    {
        if (telemetry == null)
        {
            telemetry = new VehicleTelemetry();
        }

        InitializePowertrain();
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = bodyMassCenter;

        if (inputActions != null)
        {
            steeringAction = inputActions.FindAction("Vehicle/Steering");
            throttleAction = inputActions.FindAction("Vehicle/Throttle");
            brakeAction = inputActions.FindAction("Vehicle/Brake");
            handbrakeAction = inputActions.FindAction("Vehicle/HandBrake");
            gearShiftAction = inputActions.FindAction("Vehicle/GearShift");
            
            steeringAction?.Enable();
            throttleAction?.Enable();
            brakeAction?.Enable();
            handbrakeAction?.Enable();
            gearShiftAction?.Enable();

            if (gearShiftAction != null)
            {
                gearShiftAction.performed += _ => ToggleGearMode();
            }
        }
    }

    void ToggleGearMode()
    {
        if (telemetry.carSpeed > 1f) return;

        GearMode currentMode = transmission.Mode;
        GearMode nextMode = currentMode;

        switch (currentMode)
        {
            case GearMode.Drive:
                nextMode = GearMode.Neutral;
                break;
            case GearMode.Neutral:
                nextMode = GearMode.Reverse;
                break;
            case GearMode.Reverse:
                nextMode = GearMode.Drive;
                break;
        }

        transmission.SetMode(nextMode);
    }

    void InitializePowertrain()
    {
        EngineConfig engineConfig = new EngineConfig
        {
            idleRPM = idleRPM,
            redlineRPM = redlineRPM,
            inertia = engineInertia,
            drag = engineDrag,
            peakTorque = peakTorque,
            torqueCurve = torqueCurve,
            useRevLimiter = useRevLimiter
        };

        TransmissionConfig transmissionConfig = new TransmissionConfig
        {
            gears = gearRatios,
            finalDrive = finalDriveRatio,
            shiftUpRPM = shiftUpRPM,
            shiftDownRPM = shiftDownRPM,
            shiftTime = shiftTime
        };

        DrivetrainConfig drivetrainConfig = new DrivetrainConfig
        {
            elasticity = drivetrainElasticity,
            lockRecoveryTime = lockRecoveryTime,
            converterMaxFactor = converterMaxFactor,
            converterFadeSpeed = converterFadeSpeed,
            converterGear = converterGear
        };

        engine = new Engine(engineConfig, telemetry);
        transmission = new Transmission(transmissionConfig, reverseRatio, telemetry);
        drivetrain = new Drivetrain(drivetrainConfig, telemetry);
    }

    void Update()
    {
        if (!useExternalInput)
        {
            if (steeringAction != null) steeringInput = steeringAction.ReadValue<float>();
            if (throttleAction != null) throttleInput = throttleAction.ReadValue<float>();
            if (brakeAction != null) brakeInput = brakeAction.ReadValue<float>();
            if (handbrakeAction != null) handbrakePressed = handbrakeAction.ReadValue<float>() > 0.5f;
        }

        AnimateWheelMeshes();
    }

    void FixedUpdate()
    {
        // Speed in km/h
        telemetry.carSpeed = rb.linearVelocity.magnitude * 3.6f;
        
        CalculateWheelTelemetry();
        UpdatePowertrain(Time.fixedDeltaTime);

        if (useExternalInput)
        {
            ApplyExternalInputs();
        }
        else
        {
            ApplyPlayerInputs();
        }

        ApplyResistance();
        ApplyAntiRollBar(frontLeftCollider, frontRightCollider);
        ApplyAntiRollBar(rearLeftCollider, rearRightCollider);
    }

    void CalculateWheelTelemetry()
    {
        // AWD telemetry: Average of all wheels
        float wheelMS_FL = GetWheelLinearSpeed(frontLeftCollider);
        float wheelMS_FR = GetWheelLinearSpeed(frontRightCollider);
        float wheelMS_RL = GetWheelLinearSpeed(rearLeftCollider);
        float wheelMS_RR = GetWheelLinearSpeed(rearRightCollider);

        float groundMS_FL = GetLongitudinalGroundSpeed(frontLeftCollider);
        float groundMS_FR = GetLongitudinalGroundSpeed(frontRightCollider);
        float groundMS_RL = GetLongitudinalGroundSpeed(rearLeftCollider);
        float groundMS_RR = GetLongitudinalGroundSpeed(rearRightCollider);

        float slipFL = CalculateSlipRatio(wheelMS_FL, groundMS_FL);
        float slipFR = CalculateSlipRatio(wheelMS_FR, groundMS_FR);
        float slipRL = CalculateSlipRatio(wheelMS_RL, groundMS_RL);
        float slipRR = CalculateSlipRatio(wheelMS_RR, groundMS_RR);

        // Telemetry update
        telemetry.wheelSpeed = ((wheelMS_FL + wheelMS_FR + wheelMS_RL + wheelMS_RR) / 4f) * 3.6f; // km/h
        telemetry.wheelSlip = (slipFL + slipFR + slipRL + slipRR) / 4f;
    }

    float GetWheelLinearSpeed(WheelCollider wc)
    {
        // rpm -> rad/s: rpm * 2π / 60
        float omega = wc.rpm * 2f * Mathf.PI / 60f;
        return omega * wc.radius; // m/s
    }

    float GetLongitudinalGroundSpeed(WheelCollider wc)
    {
        // Speed of the rigidbody at the specific wheel contact patch, projected forward
        Vector3 wheelVelocity = rb.GetPointVelocity(wc.transform.position);
        return Vector3.Dot(wheelVelocity, wc.transform.forward);
    }

    float CalculateSlipRatio(float wheelMS, float groundMS)
    {
        // SAE slip ratio with 1.0 m/s floor to prevent explode-at-zero artifacts
        float denom = Mathf.Max(1.0f, Mathf.Abs(groundMS));
        return (wheelMS - groundMS) / denom;
    }

    void UpdatePowertrain(float dt)
    {
        // AWD: Average RPM of all wheels
        float avgDrivenWheelRPM = (frontLeftCollider.rpm + frontRightCollider.rpm + rearLeftCollider.rpm + rearRightCollider.rpm) / 4f;
        float wheelRPMToEngine = Mathf.Abs(avgDrivenWheelRPM) * transmission.TotalRatio;
        float throttleForRPM = useExternalInput ? Mathf.Abs(externalAcceleration) : Mathf.Abs(throttleInput);

        drivetrain.UpdateLock(dt);

        float engineRPM = engine.UpdateRPM(wheelRPMToEngine, throttleForRPM, drivetrain.LockAmount, dt);

        if (automatic)
        {
            bool shiftStarted = transmission.Update(engineRPM, dt);
            if (shiftStarted)
            {
                drivetrain.OnShift();
            }
        }

    }

    void ApplyPlayerInputs()
    {
        // Speed Sensitive Steering
        float speedFactor = steeringVsSpeed.Evaluate(telemetry.carSpeed);
        steeringAxis = Mathf.MoveTowards(steeringAxis, steeringInput, Time.fixedDeltaTime * 10f * steeringSpeed);
        float finalAngle = steeringAxis * maxSteeringAngle * speedFactor;
        telemetry.steeringAngle = finalAngle;
        
        frontLeftCollider.steerAngle = finalAngle;
        frontRightCollider.steerAngle = finalAngle;

        // Brakes & Acceleration
        if (handbrakePressed)
        {
            ApplyBrakes(handbrakeTorque, 1.0f, true);
        }
        else if (brakeInput > 0.05f)
        {
            ApplyBrakes(maxBrakeTorque, brakeInput, false);
        }
        else
        {
            // Reset brakes when not braking
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = 0;
            rearRightCollider.brakeTorque = 0;
        }

        if (throttleInput > 0.05f)
        {
            CalculateAndApplyTorque(throttleInput);
        }
        else
        {
            CalculateAndApplyTorque(0);
        }
    }

    void ApplyExternalInputs()
    {
        float speedFactor = steeringVsSpeed.Evaluate(telemetry.carSpeed);
        steeringAxis = Mathf.MoveTowards(steeringAxis, externalSteering, Time.fixedDeltaTime * 10f * steeringSpeed);
        float finalAngle = steeringAxis * maxSteeringAngle * speedFactor;
        telemetry.steeringAngle = finalAngle;
        
        frontLeftCollider.steerAngle = finalAngle;
        frontRightCollider.steerAngle = finalAngle;

        if (externalAcceleration > 0.05f)
        {
            CalculateAndApplyTorque(externalAcceleration);
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = 0;
            rearRightCollider.brakeTorque = 0;
        }
        else if (externalAcceleration < -0.05f)
        {
            ApplyBrakes(maxBrakeTorque, Mathf.Abs(externalAcceleration), false);
            CalculateAndApplyTorque(0);
        }
        else
        {
            CalculateAndApplyTorque(0);
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = 0;
            rearRightCollider.brakeTorque = 0;
        }
    }

    void CalculateAndApplyTorque(float input)
    {
        float engineTorque = engine.GetTorque(input);

        if (transmission.Mode == GearMode.Reverse)
        {
            engineTorque = -engineTorque;
        }

        if (input > 0.05f)
        {
            // Low-speed torque protection (prevent engine bogging exploits)
            float rpmFactor = Mathf.InverseLerp(idleRPM * 0.5f, 2000f, telemetry.engineRPM);
            engineTorque *= rpmFactor;

            // Traction Control (TC)
            ApplyTractionControl(ref engineTorque);
        }
        else
        {
            // Engine Braking: resist rotation when throttle is zero
            float forwardVel = Vector3.Dot(rb.linearVelocity, transform.forward);
            float directionSign = Mathf.Sign(forwardVel);
            engineTorque *= directionSign;

            // Creep Torque logic (Automatic creep behavior)
            if (transmission.Mode == GearMode.Drive && !handbrakePressed && telemetry.carSpeed < creepMaxSpeed)
            {
                float creepFactor = 1f - Mathf.InverseLerp(0, creepMaxSpeed, telemetry.carSpeed);
                engineTorque += idleCreepTorque * creepFactor;
            }
            else if (transmission.Mode == GearMode.Reverse && !handbrakePressed && telemetry.carSpeed < creepMaxSpeed)
            {
                float creepFactor = 1f - Mathf.InverseLerp(0, creepMaxSpeed, telemetry.carSpeed);
                engineTorque -= idleCreepTorque * creepFactor;
            }
        }

        if (transmission.Mode == GearMode.Neutral) engineTorque = 0;

        telemetry.engineTorque = engineTorque;

        float wheelTorque = drivetrain.Update(
            engineTorque,
            transmission.TotalRatio,
            telemetry.carSpeed,
            transmission.CurrentGear,
            transmission.IsShifting,
            Time.fixedDeltaTime);

        // AWD: Apply torque to all 4 wheels
        frontLeftCollider.motorTorque = wheelTorque * 0.25f;
        frontRightCollider.motorTorque = wheelTorque * 0.25f;
        rearLeftCollider.motorTorque = wheelTorque * 0.25f;
        rearRightCollider.motorTorque = wheelTorque * 0.25f;
    }

    void ApplyTractionControl(ref float torque)
    {
        // Traction Control now uses the physics-based slip ratio calculated in CalculateWheelTelemetry
        float slipThreshold = 0.25f; // Lowered threshold for tighter V8 control

        if (telemetry.wheelSlip > slipThreshold)
        {
            // Simple proactive reduction based on how far over the threshold we are
            float reduction = Mathf.InverseLerp(slipThreshold, slipThreshold + 0.5f, telemetry.wheelSlip);
            torque *= (1f - (reduction * tractionControl));
        }
    }

    void ApplyBrakes(float force, float input, bool isHandbrake)
    {
        float totalBrake = force * input;

        if (isHandbrake)
        {
            // Handbrake locks rear wheels
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = totalBrake;
            rearRightCollider.brakeTorque = totalBrake;
        }
        else
        {
            // Regular braking (resistance)
            frontLeftCollider.brakeTorque = totalBrake * brakeBias;
            frontRightCollider.brakeTorque = totalBrake * brakeBias;
            rearLeftCollider.brakeTorque = totalBrake * (1f - brakeBias);
            rearRightCollider.brakeTorque = totalBrake * (1f - brakeBias);
        }
        
        frontLeftCollider.motorTorque = 0;
        frontRightCollider.motorTorque = 0;
        rearLeftCollider.motorTorque = 0;
        rearRightCollider.motorTorque = 0;
        telemetry.wheelTorque = 0f;
    }

    void ApplyAntiRollBar(WheelCollider left, WheelCollider right)
    {
        WheelHit hit;
        float travelL = 1.0f;
        float travelR = 1.0f;

        bool groundedL = left.GetGroundHit(out hit);
        if (groundedL) travelL = (-left.transform.InverseTransformPoint(hit.point).y - left.radius) / left.suspensionDistance;

        bool groundedR = right.GetGroundHit(out hit);
        if (groundedR) travelR = (-right.transform.InverseTransformPoint(hit.point).y - right.radius) / right.suspensionDistance;

        float antiRollForce = (travelL - travelR) * antiRollStiffness;

        if (groundedL) rb.AddForceAtPosition(left.transform.up * -antiRollForce, left.transform.position);
        if (groundedR) rb.AddForceAtPosition(right.transform.up * antiRollForce, right.transform.position);
    }

    void ApplyResistance()
    {
        // Aerodynamic Drag
        float velocity = rb.linearVelocity.magnitude;
        float dragForce = 0.5f * 1.225f * aerodynamicDrag * frontalArea * velocity * velocity;
        rb.AddForce(-rb.linearVelocity.normalized * dragForce);

        // Rolling Resistance (simplified physical model)
        float rrForce = rollingResistance * rb.mass * 0.015f; 
        rb.AddForce(-rb.linearVelocity.normalized * rrForce);
    }

    void AnimateWheelMeshes()
    {
        UpdateWheelMesh(frontLeftCollider, frontLeftMesh);
        UpdateWheelMesh(frontRightCollider, frontRightMesh);
        UpdateWheelMesh(rearLeftCollider, rearLeftMesh);
        UpdateWheelMesh(rearRightCollider, rearRightMesh);
    }

    void UpdateWheelMesh(WheelCollider collider, GameObject mesh)
    {
        if (collider == null || mesh == null) return;
        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        mesh.transform.position = pos;
        mesh.transform.rotation = rot;
    }
}

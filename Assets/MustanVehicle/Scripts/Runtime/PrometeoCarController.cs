using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

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
        new Keyframe(6200, 0.72f),
        new Keyframe(6600, 0.40f),
        new Keyframe(7200, 0.01f)
    );
    public bool useRevLimiter = true;

    [Header("Transmission")]
    public bool automatic = true;
    public float finalDriveRatio = 3.73f;
    public float[] gearRatios = { 2.87f, 1.89f, 1.28f, 1.00f, 0.82f, 0.67f };
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

    [Header("Launch / Clutch Slip")]
    public float launchTargetRPM = 2000f;
    public float launchSyncSpeed = 12f; // km/h
    [Range(0f, 1f)]
    public float launchClutchLock = 0.35f;
    [Range(0f, 1f)]
    public float launchThrottleThreshold = 0.1f;

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
    [FormerlySerializedAs("externalAcceleration")]
    public float externalThrottle = 0f;
    public float externalBrake = 0f;
    public bool externalHandbrake = false;
    public GearMode externalGearMode = GearMode.Drive;

    [Header("Telemetry")]
    public VehicleTelemetry telemetry = new VehicleTelemetry();

    // Internal state
    Rigidbody rb;
    float steeringAxis;
    float throttleInput;
    float brakeInput;
    float steeringInput;
    bool handbrakePressed;
    bool isBraking;

    float wheelMS_FL;
    float wheelMS_FR;
    float wheelMS_RL;
    float wheelMS_RR;
    float groundMS_FL;
    float groundMS_FR;
    float groundMS_RL;
    float groundMS_RR;
    
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

        // Cycle Reverse -> Hold(Neutral) -> Forward(Drive) when stopped
        switch (transmission.Mode)
        {
            case GearMode.Drive:
                transmission.SetMode(GearMode.Reverse);
                break;
            case GearMode.Reverse:
                transmission.SetMode(GearMode.Neutral);
                break;
            case GearMode.Neutral:
                transmission.SetMode(GearMode.Drive);
                break;
        }
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
        if (useExternalInput)
        {
            ApplyExternalGearMode();
        }
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
        wheelMS_FL = GetWheelLinearSpeed(frontLeftCollider);
        wheelMS_FR = GetWheelLinearSpeed(frontRightCollider);
        wheelMS_RL = GetWheelLinearSpeed(rearLeftCollider);
        wheelMS_RR = GetWheelLinearSpeed(rearRightCollider);

        groundMS_FL = GetLongitudinalGroundSpeed(frontLeftCollider);
        groundMS_FR = GetLongitudinalGroundSpeed(frontRightCollider);
        groundMS_RL = GetLongitudinalGroundSpeed(rearLeftCollider);
        groundMS_RR = GetLongitudinalGroundSpeed(rearRightCollider);

        float slipFL = CalculateSlipRatio(wheelMS_FL, groundMS_FL);
        float slipFR = CalculateSlipRatio(wheelMS_FR, groundMS_FR);
        float slipRL = CalculateSlipRatio(wheelMS_RL, groundMS_RL);
        float slipRR = CalculateSlipRatio(wheelMS_RR, groundMS_RR);

        float wheelKPH_FL = wheelMS_FL * 3.6f;
        float wheelKPH_FR = wheelMS_FR * 3.6f;
        float wheelKPH_RL = wheelMS_RL * 3.6f;
        float wheelKPH_RR = wheelMS_RR * 3.6f;

        // Telemetry update
        telemetry.wheelSpeed = (wheelKPH_FL + wheelKPH_FR + wheelKPH_RL + wheelKPH_RR) / 4f;
        telemetry.wheelSpeedFL = wheelKPH_FL;
        telemetry.wheelSpeedFR = wheelKPH_FR;
        telemetry.wheelSpeedRL = wheelKPH_RL;
        telemetry.wheelSpeedRR = wheelKPH_RR;
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
        float throttleForRPM = useExternalInput ? Mathf.Clamp01(externalThrottle) : Mathf.Abs(throttleInput);

        drivetrain.UpdateLock(dt);

        float rpmTarget = wheelRPMToEngine;
        float engineLockForRPM = drivetrain.LockAmount;

        bool launchActive = false;
        if (transmission.Mode == GearMode.Drive &&
            transmission.CurrentGear == 1 &&
            throttleForRPM > launchThrottleThreshold &&
            telemetry.carSpeed < launchSyncSpeed)
        {
            launchActive = true;
            float launchBlend = Mathf.InverseLerp(0f, launchSyncSpeed, telemetry.carSpeed);
            float launchTarget = Mathf.Max(wheelRPMToEngine, launchTargetRPM);
            rpmTarget = Mathf.Lerp(launchTarget, wheelRPMToEngine, launchBlend);
            engineLockForRPM = Mathf.Lerp(launchClutchLock, drivetrain.LockAmount, launchBlend);
        }

        bool drivelineLocked = transmission.Mode != GearMode.Neutral &&
            !transmission.IsShifting &&
            engineLockForRPM >= 0.99f &&
            drivetrain.LockAmount >= 0.99f;

        // When the driveline is locked, let wheel RPM drive engine RPM (no limiter clamp).
        float engineRPM = engine.UpdateRPM(rpmTarget, throttleForRPM, engineLockForRPM, dt, !drivelineLocked);

        if (automatic)
        {
            bool allowShift = !launchActive || transmission.IsShifting;
            bool shiftStarted = transmission.Update(engineRPM, dt, allowShift);
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
        isBraking = false;

        if (handbrakePressed)
        {
            ApplyBrakes(handbrakeTorque, 1.0f, true);
            isBraking = true;
        }
        else if (brakeInput > 0.05f)
        {
            ApplyBrakes(maxBrakeTorque, brakeInput, false);
            isBraking = true;
        }
        else
        {
            // Reset brakes when not braking
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = 0;
            rearRightCollider.brakeTorque = 0;
        }

        if (!isBraking && throttleInput > 0.05f)
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

        isBraking = false;
        handbrakePressed = externalHandbrake;

        float throttle = Mathf.Clamp01(externalThrottle);
        float brake = Mathf.Clamp01(externalBrake);

        if (handbrakePressed)
        {
            ApplyBrakes(handbrakeTorque, 1.0f, true);
            isBraking = true;
        }
        else if (brake > 0.05f)
        {
            ApplyBrakes(maxBrakeTorque, brake, false);
            isBraking = true;
        }
        else
        {
            frontLeftCollider.brakeTorque = 0;
            frontRightCollider.brakeTorque = 0;
            rearLeftCollider.brakeTorque = 0;
            rearRightCollider.brakeTorque = 0;
        }

        if (!isBraking && throttle > 0.05f)
        {
            CalculateAndApplyTorque(throttle);
        }
        else
        {
            CalculateAndApplyTorque(0);
        }
    }

    void ApplyExternalGearMode()
    {
        if (transmission == null)
        {
            InitializePowertrain();
            if (transmission == null) return;
        }

        if (telemetry.carSpeed > 1f) return;
        if (transmission.Mode != externalGearMode)
        {
            transmission.SetMode(externalGearMode);
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
            if (!isBraking && transmission.Mode == GearMode.Drive && !handbrakePressed && telemetry.carSpeed < creepMaxSpeed)
            {
                float creepFactor = 1f - Mathf.InverseLerp(0, creepMaxSpeed, telemetry.carSpeed);
                engineTorque += idleCreepTorque * creepFactor;
            }
            else if (!isBraking && transmission.Mode == GearMode.Reverse && !handbrakePressed && telemetry.carSpeed < creepMaxSpeed)
            {
                float creepFactor = 1f - Mathf.InverseLerp(0, creepMaxSpeed, telemetry.carSpeed);
                engineTorque -= idleCreepTorque * creepFactor;
            }
        }

        if (transmission.Mode == GearMode.Neutral) engineTorque = 0;

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

    float ApplyAbsToBrakeTorque(float baseTorque, float wheelMS, float groundMS)
    {
        const float absSlipThreshold = 0.2f;
        const float absSlipRange = 0.4f;
        const float absMinFactor = 0.25f;

        float groundAbs = Mathf.Abs(groundMS);
        if (groundAbs < 1.0f)
        {
            return baseTorque;
        }

        float slip = (groundAbs - Mathf.Abs(wheelMS)) / groundAbs;
        if (slip <= absSlipThreshold)
        {
            return baseTorque;
        }

        float t = Mathf.InverseLerp(absSlipThreshold, absSlipThreshold + absSlipRange, slip);
        float factor = Mathf.Lerp(1f, absMinFactor, t);
        return baseTorque * factor;
    }

    void ApplyBrakeWeightTransfer(float input)
    {
        if (rb == null || input <= 0f) return;

        float speedFactor = Mathf.Clamp01(telemetry.carSpeed / 5f);
        float transferForce = rb.mass * Physics.gravity.magnitude * weightTransfer * input * speedFactor;

        Vector3 frontPos = (frontLeftCollider.transform.position + frontRightCollider.transform.position) * 0.5f;
        Vector3 rearPos = (rearLeftCollider.transform.position + rearRightCollider.transform.position) * 0.5f;

        rb.AddForceAtPosition(-transform.up * transferForce, frontPos);
        rb.AddForceAtPosition(transform.up * transferForce, rearPos);
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
            float frontBrake = totalBrake * brakeBias;
            float rearBrake = totalBrake * (1f - brakeBias);

            frontLeftCollider.brakeTorque = ApplyAbsToBrakeTorque(frontBrake, wheelMS_FL, groundMS_FL);
            frontRightCollider.brakeTorque = ApplyAbsToBrakeTorque(frontBrake, wheelMS_FR, groundMS_FR);
            rearLeftCollider.brakeTorque = ApplyAbsToBrakeTorque(rearBrake, wheelMS_RL, groundMS_RL);
            rearRightCollider.brakeTorque = ApplyAbsToBrakeTorque(rearBrake, wheelMS_RR, groundMS_RR);

            ApplyBrakeWeightTransfer(input);
        }
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

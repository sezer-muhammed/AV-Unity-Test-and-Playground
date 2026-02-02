/*
MESSAGE FROM CREATOR: This script was coded by Mena. You can use it in your games either these are commercial or
personal projects. You can even add or remove functions as you wish. However, you cannot sell copies of this
script by itself, since it is originally distributed as a free product.
I wish you the best for your project. Good luck!

P.S: If you need more cars, you can check my other vehicle assets on the Unity Asset Store, perhaps you could find
something useful for your game. Best regards, Mena.
*/

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PrometeoCarController : MonoBehaviour
{

    //CAR SETUP

      [Space(20)]
      //[Header("CAR SETUP")]
      [Space(10)]
      [Range(20, 220)]
      public int maxSpeed = 90; //The maximum speed that the car can reach in km/h.
      [Range(10, 120)]
      public int maxReverseSpeed = 45; //The maximum speed that the car can reach while going on reverse in km/h.
      [Range(1, 10)]
      public int accelerationMultiplier = 2; // How fast the car can accelerate. 1 is a slow acceleration and 10 is the fastest.
      [Space(10)]
      [Range(10, 45)]
      public int maxSteeringAngle = 27; // The maximum angle that the tires can reach while rotating the steering wheel.
      [Range(0.1f, 1f)]
      public float steeringSpeed = 0.5f; // How fast the steering wheel turns.
      [Space(10)]
      [Range(100, 600)]
      public int brakeForce = 350; // The strength of the wheel brakes.
      [Range(1, 10)]
      public int decelerationMultiplier = 2; // How fast the car decelerates when the user is not using the throttle.
      [Range(1, 10)]
      public int handbrakeDriftMultiplier = 2; // Reduced from 5 to 2 to increase grip during handbrake
      [Range(1, 10)]
      public float tireGripMultiplier = 2.0f; // New multiplier to increase overall friction
      [Space(10)]
      public Vector3 bodyMassCenter; // This is a vector that contains the center of mass of the car. I recommend to set this value
                                    // in the points x = 0 and z = 0 of your car. You can select the value that you want in the y axis,
                                    // however, you must notice that the higher this value is, the more unstable the car becomes.
                                    // Usually the y value goes from 0 to 1.5.

      [Header("Powertrain (Optional)")]
      public bool useHorsepowerModel = false;
      [Tooltip("Optional mass source for the power/1G model. If null, uses this GameObject's Rigidbody.")]
      public Rigidbody massSourceRigidbody;
      [Min(1f)]
      public float horsepower = 150f;
      [Range(0.1f, 1f)]
      public float drivetrainEfficiency = 0.85f;
      [Tooltip("Energy reservoir in kWh. Acceleration reduces when depleted.")]
      [Min(0.01f)]
      public float maxEnergyKWh = 10f;
      [Range(0f, 1f)]
      public float initialEnergyPercent = 1f;
      [Tooltip("Hard cap on longitudinal acceleration (1G = 9.81 m/s^2)")]
      [Min(0.1f)]
      public float maxLongitudinalAccelerationG = 1.0f;
      [Tooltip("Avoid infinite force at very low speeds. m/s")]
      [Min(0.01f)]
      public float powerModelMinSpeedMS = 1.0f;

      [Header("Traction Control (Anti-Slip)")]
      public bool tractionControlEnabled = true;
      [Tooltip("If |wheelSpeed - rbSpeed| exceeds this (km/h), start reducing motor torque.")]
      [Min(0f)]
      public float slipThresholdKmh = 3.0f;
      [Tooltip("Slip (km/h) above threshold that maps to max reduction.")]
      [Min(0.1f)]
      public float slipRangeKmh = 15.0f;
      [Tooltip("Minimum torque scale when slipping hard.")]
      [Range(0.05f, 1f)]
      public float minTorqueScale = 0.25f;
      [Tooltip("How fast traction control reacts (higher = faster).")]
      [Min(0.1f)]
      public float tcResponse = 8.0f;

    //WHEELS

      //[Header("WHEELS")]

      /*
      The following variables are used to store the wheels' data of the car. We need both the mesh-only game objects and wheel
      collider components of the wheels. The wheel collider components and 3D meshes of the wheels cannot come from the same
      game object; they must be separate game objects.
      */
      public GameObject frontLeftMesh;
      public WheelCollider frontLeftCollider;
      [Space(10)]
      public GameObject frontRightMesh;
      public WheelCollider frontRightCollider;
      [Space(10)]
      public GameObject rearLeftMesh;
      public WheelCollider rearLeftCollider;
      [Space(10)]
      public GameObject rearRightMesh;
      public WheelCollider rearRightCollider;

    //PARTICLE SYSTEMS

      [Space(20)]
      //[Header("EFFECTS")]
      [Space(10)]
      //The following variable lets you to set up particle systems in your car
      public bool useEffects = false;

      // The following particle systems are used as tire smoke when the car drifts.
      public ParticleSystem RLWParticleSystem;
      public ParticleSystem RRWParticleSystem;

      [Space(10)]
      // The following trail renderers are used as tire skids when the car loses traction.
      public TrailRenderer RLWTireSkid;
      public TrailRenderer RRWTireSkid;

    //CONTROLS

      [Space(20)]
      //[Header("CONTROLS")]
      [Space(10)]
      public InputActionAsset inputActions;
      private InputAction moveAction;
      private InputAction handbrakeAction;

    //CAR DATA

      [HideInInspector]
      public float carSpeed; // Used to store the speed of the car.
      [HideInInspector]
      public bool isDrifting; // Used to know whether the car is drifting or not.
      [HideInInspector]
      public bool isTractionLocked; // Used to know whether the traction of the car is locked or not.

    //PRIVATE VARIABLES

      /*
      IMPORTANT: The following variables should not be modified manually since their values are automatically given via script.
      */
      Rigidbody carRigidbody; // Stores the car's rigidbody.
      float steeringAxis; // Used to know whether the steering wheel has reached the maximum value. It goes from -1 to 1.
      float throttleAxis; // Used to know whether the throttle has reached the maximum value. It goes from -1 to 1.
      float driftingAxis;
      float localVelocityZ;
      float localVelocityX;
      bool deceleratingCar;
      bool lastHandbrakeState;
      Vector2 moveInput;
      bool handbrakePressed;

      [Header("Autonomous Control")]
      public bool useExternalInput = false;
      [Range(-1f, 1f)]
      public float externalSteering = 0f;
      [Range(-1f, 1f)]
      public float externalAcceleration = 0f;
      [Tooltip("When using external input, treat negative externalAcceleration as braking (recommended for AI). If false, negative means reverse.")]
      public bool externalNegativeMeansBrake = true;

      public Vector2 DebugMoveInput => moveInput;
      public float DebugSteeringAxis => steeringAxis;
      public float DebugThrottleAxis => throttleAxis;
      public bool DebugHandbrakePressed => handbrakePressed;
      public float DebugFrontSteerAngle => (frontLeftCollider != null) ? frontLeftCollider.steerAngle : 0f;
      public float DebugRearLeftMotorTorque => (rearLeftCollider != null) ? rearLeftCollider.motorTorque : 0f;
      public float DebugRearRightMotorTorque => (rearRightCollider != null) ? rearRightCollider.motorTorque : 0f;

      public float DebugSpeedFromRigidbodyKmh => speedFromRigidbodyKmh;
      public float DebugSpeedFromPositionKmh => speedFromPositionKmh;
      public float DebugSpeedFromRigidbodyKmhSmoothed => speedFromRigidbodyKmhSmoothed;
      public float DebugSpeedFromPositionKmhSmoothed => speedFromPositionKmhSmoothed;
      public float DebugForwardSpeedMS => forwardSpeedMS;
      public float DebugForwardAccelMS2 => forwardAccelMS2;

      public float DebugSlipKmh => slipKmh;
      public float DebugTorqueScale => torqueScale;

      public bool DebugUseHorsepowerModel => useHorsepowerModel;
      public float DebugHorsepower => horsepower;
      public float DebugDrivetrainEfficiency => drivetrainEfficiency;
      public float DebugMaxEnergyKWh => maxEnergyKWh;
      public float DebugCurrentEnergyKWh => currentEnergyJ / 3_600_000f;
      public float DebugLastAvailablePowerW => lastAvailablePowerW;
      public float DebugLastRequestedPowerW => lastRequestedPowerW;
      public float DebugLastTractiveForceN => lastTractiveForceN;
      public float DebugLastAccelCapMS2 => lastAccelCapMS2;
      public float DebugLastTorquePerWheelNm => lastTorquePerWheelNm;
      public float DebugLastPedal01 => lastPedal01;
      public bool DebugLastEnergyLimited => lastEnergyLimited;

      /*
      The following variables are used to store information about sideways friction of the wheels (such as
      extremumSlip,extremumValue, asymptoteSlip, asymptoteValue and stiffness). We change this values to
      make the car to start drifting.
      */
      WheelFrictionCurve FLwheelFriction;
      float FLWextremumSlip;
      WheelFrictionCurve FRwheelFriction;
      float FRWextremumSlip;
      WheelFrictionCurve RLwheelFriction;
      float RLWextremumSlip;
      WheelFrictionCurve RRwheelFriction;
      float RRWextremumSlip;

      float currentEnergyJ;
      float lastAvailablePowerW;
      float lastRequestedPowerW;
      float lastTractiveForceN;
      float lastAccelCapMS2;
      float lastTorquePerWheelNm;
      float lastPedal01;
      bool lastEnergyLimited;

      float speedFromRigidbodyKmh;
      float speedFromPositionKmh;
      float speedFromRigidbodyKmhSmoothed;
      float speedFromPositionKmhSmoothed;
      float forwardSpeedMS;
      float forwardAccelMS2;
      float lastForwardSpeedMS;

      Vector3 lastPosition;
      bool hasLastPosition;

      [Header("Telemetry Smoothing")]
      [Min(0f)]
      public float speedSmoothingTimeSeconds = 0.25f;

      float slipKmh;
      float torqueScale = 1f;

    // Start is called before the first frame update
    void Start()
    {
      //In this part, we set the 'carRigidbody' value with the Rigidbody attached to this
      //gameObject. Also, we define the center of mass of the car with the Vector3 given
      //in the inspector.
      carRigidbody = gameObject.GetComponent<Rigidbody>();
      carRigidbody.centerOfMass = bodyMassCenter;

      if (massSourceRigidbody == null)
        massSourceRigidbody = carRigidbody;

      lastPosition = transform.position;
      hasLastPosition = true;

      speedFromRigidbodyKmhSmoothed = 0f;
      speedFromPositionKmhSmoothed = 0f;

      // Initialize energy reservoir (Joules)
      currentEnergyJ = Mathf.Clamp01(initialEnergyPercent) * (maxEnergyKWh * 3_600_000f);

      //Initial setup to calculate the drift value of the car. This part could look a bit
      //complicated, but do not be afraid, the only thing we're doing here is to save the default
      //friction values of the car wheels so we can set an appropiate drifting value later.
      FLwheelFriction = new WheelFrictionCurve ();
        FLwheelFriction.extremumSlip = frontLeftCollider.sidewaysFriction.extremumSlip;
        FLWextremumSlip = frontLeftCollider.sidewaysFriction.extremumSlip;
        FLwheelFriction.extremumValue = frontLeftCollider.sidewaysFriction.extremumValue;
        FLwheelFriction.asymptoteSlip = frontLeftCollider.sidewaysFriction.asymptoteSlip;
        FLwheelFriction.asymptoteValue = frontLeftCollider.sidewaysFriction.asymptoteValue;
        FLwheelFriction.stiffness = frontLeftCollider.sidewaysFriction.stiffness * tireGripMultiplier;
      FRwheelFriction = new WheelFrictionCurve ();
        FRwheelFriction.extremumSlip = frontRightCollider.sidewaysFriction.extremumSlip;
        FRWextremumSlip = frontRightCollider.sidewaysFriction.extremumSlip;
        FRwheelFriction.extremumValue = frontRightCollider.sidewaysFriction.extremumValue;
        FRwheelFriction.asymptoteSlip = frontRightCollider.sidewaysFriction.asymptoteSlip;
        FRwheelFriction.asymptoteValue = frontRightCollider.sidewaysFriction.asymptoteValue;
        FRwheelFriction.stiffness = frontRightCollider.sidewaysFriction.stiffness * tireGripMultiplier;
      RLwheelFriction = new WheelFrictionCurve ();
        RLwheelFriction.extremumSlip = rearLeftCollider.sidewaysFriction.extremumSlip;
        RLWextremumSlip = rearLeftCollider.sidewaysFriction.extremumSlip;
        RLwheelFriction.extremumValue = rearLeftCollider.sidewaysFriction.extremumValue;
        RLwheelFriction.asymptoteSlip = rearLeftCollider.sidewaysFriction.asymptoteSlip;
        RLwheelFriction.asymptoteValue = rearLeftCollider.sidewaysFriction.asymptoteValue;
        RLwheelFriction.stiffness = rearLeftCollider.sidewaysFriction.stiffness * tireGripMultiplier;
      RRwheelFriction = new WheelFrictionCurve ();
        RRwheelFriction.extremumSlip = rearRightCollider.sidewaysFriction.extremumSlip;
        RRWextremumSlip = rearRightCollider.sidewaysFriction.extremumSlip;
        RRwheelFriction.extremumValue = rearRightCollider.sidewaysFriction.extremumValue;
        RRwheelFriction.asymptoteSlip = rearRightCollider.sidewaysFriction.asymptoteSlip;
        RRwheelFriction.asymptoteValue = rearRightCollider.sidewaysFriction.asymptoteValue;
        RRwheelFriction.stiffness = rearRightCollider.sidewaysFriction.stiffness * tireGripMultiplier;

        if(!useEffects){
          if(RLWParticleSystem != null){
            RLWParticleSystem.Stop();
          }
          if(RRWParticleSystem != null){
            RRWParticleSystem.Stop();
          }
          if(RLWTireSkid != null){
            RLWTireSkid.emitting = false;
          }
          if(RRWTireSkid != null){
            RRWTireSkid.emitting = false;
          }
        }

        if (inputActions != null)
        {
            moveAction = inputActions.FindAction("Move");
            handbrakeAction = inputActions.FindAction("Jump"); // Using Jump for Handbrake as per standard actions or search for Handbrake
            if (moveAction == null) moveAction = inputActions.FindAction("Player/Move");
            if (handbrakeAction == null) handbrakeAction = inputActions.FindAction("Player/Jump");
            
            moveAction?.Enable();
            handbrakeAction?.Enable();
        }

    }

    // Update is called once per frame
    void Update()
    {
      // Reading input in Update is best practice for responsiveness
      if (moveAction != null)
      {
          moveInput = moveAction.ReadValue<Vector2>();
      }
      if (handbrakeAction != null)
      {
          handbrakePressed = handbrakeAction.ReadValue<float>() > 0.5f;
      }

      // We call the method AnimateWheelMeshes in Update to match rendering framerate
      AnimateWheelMeshes();
    }

    void FixedUpdate()
    {
      // Physics calculations should always happen in FixedUpdate
      // Wheel RPM speed can be misleading under slip; keep it as legacy carSpeed,
      // but also compute a Rigidbody-based speed for telemetry and AI.
      carSpeed = (2 * Mathf.PI * frontLeftCollider.radius * frontLeftCollider.rpm * 60) / 1000;
      speedFromRigidbodyKmh = (carRigidbody != null) ? (carRigidbody.linearVelocity.magnitude * 3.6f) : carSpeed;

      // Position-delta speed (robust even if wheel RPM is wrong)
      {
        float dtPos = Time.fixedDeltaTime;
        if (dtPos <= 0f) dtPos = 0.02f;

        Vector3 pos = transform.position;
        if (!hasLastPosition)
        {
          lastPosition = pos;
          hasLastPosition = true;
          speedFromPositionKmh = speedFromRigidbodyKmh;
        }
        else
        {
          float v = (pos - lastPosition).magnitude / dtPos;
          // Avoid insane spikes from teleports
          if (v > 200f) v = 200f;
          speedFromPositionKmh = v * 3.6f;
          lastPosition = pos;
        }
      }

      // Exponential smoothing for HUD readability
      {
        float dtS = Time.fixedDeltaTime;
        if (dtS <= 0f) dtS = 0.02f;

        if (speedSmoothingTimeSeconds <= 0.0001f)
        {
          speedFromRigidbodyKmhSmoothed = speedFromRigidbodyKmh;
          speedFromPositionKmhSmoothed = speedFromPositionKmh;
        }
        else
        {
          float alpha = 1f - Mathf.Exp(-dtS / speedSmoothingTimeSeconds);
          if (speedFromRigidbodyKmhSmoothed <= 0.0001f) speedFromRigidbodyKmhSmoothed = speedFromRigidbodyKmh;
          if (speedFromPositionKmhSmoothed <= 0.0001f) speedFromPositionKmhSmoothed = speedFromPositionKmh;
          speedFromRigidbodyKmhSmoothed = Mathf.Lerp(speedFromRigidbodyKmhSmoothed, speedFromRigidbodyKmh, alpha);
          speedFromPositionKmhSmoothed = Mathf.Lerp(speedFromPositionKmhSmoothed, speedFromPositionKmh, alpha);
        }
      }

      // Traction control: compare wheel RPM speed to Rigidbody speed and scale torque smoothly.
      UpdateTractionControl();

      float dt = Time.fixedDeltaTime;
      if (dt <= 0f) dt = 0.02f;
      if (carRigidbody != null)
      {
        forwardSpeedMS = Vector3.Dot(carRigidbody.linearVelocity, transform.forward);
        forwardAccelMS2 = (forwardSpeedMS - lastForwardSpeedMS) / dt;
        lastForwardSpeedMS = forwardSpeedMS;
      }
      localVelocityX = transform.InverseTransformDirection(carRigidbody.linearVelocity).x;
      localVelocityZ = transform.InverseTransformDirection(carRigidbody.linearVelocity).z;

      if (useExternalInput)
      {
          // Override with external inputs
          ApplyExternalAcceleration(externalAcceleration);

          // Direct steering control for better precision in autonomous mode
          steeringAxis = Mathf.MoveTowards(steeringAxis, externalSteering, Time.fixedDeltaTime * steeringSpeed * 20f);
          var steeringAngle = steeringAxis * maxSteeringAngle;
          frontLeftCollider.steerAngle = steeringAngle;
          frontRightCollider.steerAngle = steeringAngle;
          
          return;
      }

      if (moveInput.y > 0.1f) {
          CancelInvoke("DecelerateCar");
          deceleratingCar = false;
          GoForward();
      } else if (moveInput.y < -0.1f) {
          CancelInvoke("DecelerateCar");
          deceleratingCar = false;
          GoReverse();
      } else {
          ThrottleOff();
      }

      if (moveInput.x < -0.1f) {
          TurnLeft();
      } else if (moveInput.x > 0.1f) {
          TurnRight();
      } else if (steeringAxis != 0f) {
          ResetSteeringAngle();
      }

      if (handbrakePressed) {
          CancelInvoke("DecelerateCar");
          deceleratingCar = false;
          Handbrake();
      } else if (lastHandbrakeState) {
          RecoverTraction();
      }
      lastHandbrakeState = handbrakePressed;

      if (moveInput.y == 0 && !handbrakePressed && !deceleratingCar) {
          if (!IsInvoking("DecelerateCar")) {
              InvokeRepeating("DecelerateCar", 0f, 0.1f);
              deceleratingCar = true;
          }
      }
    }

    //
    //STEERING METHODS
    //

    //The following method turns the front car wheels to the left. The speed of this movement will depend on the steeringSpeed variable.
    public void TurnLeft(){
      steeringAxis = steeringAxis - (Time.fixedDeltaTime * 10f * steeringSpeed);
      if(steeringAxis < -1f){
        steeringAxis = -1f;
      }
      var steeringAngle = steeringAxis * maxSteeringAngle;
      frontLeftCollider.steerAngle = Mathf.Lerp(frontLeftCollider.steerAngle, steeringAngle, steeringSpeed);
      frontRightCollider.steerAngle = Mathf.Lerp(frontRightCollider.steerAngle, steeringAngle, steeringSpeed);
    }

    //The following method turns the front car wheels to the right. The speed of this movement will depend on the steeringSpeed variable.
    public void TurnRight(){
      steeringAxis = steeringAxis + (Time.fixedDeltaTime * 10f * steeringSpeed);
      if(steeringAxis > 1f){
        steeringAxis = 1f;
      }
      var steeringAngle = steeringAxis * maxSteeringAngle;
      frontLeftCollider.steerAngle = Mathf.Lerp(frontLeftCollider.steerAngle, steeringAngle, steeringSpeed);
      frontRightCollider.steerAngle = Mathf.Lerp(frontRightCollider.steerAngle, steeringAngle, steeringSpeed);
    }

    //The following method takes the front car wheels to their default position (rotation = 0). The speed of this movement will depend
    // on the steeringSpeed variable.
    public void ResetSteeringAngle(){
      if(steeringAxis < 0f){
        steeringAxis = steeringAxis + (Time.fixedDeltaTime * 10f * steeringSpeed);
      }else if(steeringAxis > 0f){
        steeringAxis = steeringAxis - (Time.fixedDeltaTime * 10f * steeringSpeed);
      }
      if(Mathf.Abs(frontLeftCollider.steerAngle) < 1f){
        steeringAxis = 0f;
      }
      var steeringAngle = steeringAxis * maxSteeringAngle;
      frontLeftCollider.steerAngle = Mathf.Lerp(frontLeftCollider.steerAngle, steeringAngle, steeringSpeed);
      frontRightCollider.steerAngle = Mathf.Lerp(frontRightCollider.steerAngle, steeringAngle, steeringSpeed);
    }

    // This method matches both the position and rotation of the WheelColliders with the WheelMeshes.
    void AnimateWheelMeshes(){
      try{
        Quaternion FLWRotation;
        Vector3 FLWPosition;
        frontLeftCollider.GetWorldPose(out FLWPosition, out FLWRotation);
        frontLeftMesh.transform.position = FLWPosition;
        frontLeftMesh.transform.rotation = FLWRotation;

        Quaternion FRWRotation;
        Vector3 FRWPosition;
        frontRightCollider.GetWorldPose(out FRWPosition, out FRWRotation);
        frontRightMesh.transform.position = FRWPosition;
        frontRightMesh.transform.rotation = FRWRotation;

        Quaternion RLWRotation;
        Vector3 RLWPosition;
        rearLeftCollider.GetWorldPose(out RLWPosition, out RLWRotation);
        rearLeftMesh.transform.position = RLWPosition;
        rearLeftMesh.transform.rotation = RLWRotation;

        Quaternion RRWRotation;
        Vector3 RRWPosition;
        rearRightCollider.GetWorldPose(out RRWPosition, out RRWRotation);
        rearRightMesh.transform.position = RRWPosition;
        rearRightMesh.transform.rotation = RRWRotation;
      }catch(Exception ex){
        Debug.LogWarning(ex);
      }
    }

    //
    //ENGINE AND BRAKING METHODS
    //

    // This method apply positive torque to the wheels in order to go forward.
    public void GoForward(){
      //If the forces aplied to the rigidbody in the 'x' asis are greater than
      //3f, it means that the car is losing traction, then the car will start emitting particle systems.
      if(Mathf.Abs(localVelocityX) > 2.5f){
        isDrifting = true;
        DriftCarPS();
      }else{
        isDrifting = false;
        DriftCarPS();
      }
      // The following part sets the throttle power to 1 smoothly.
      throttleAxis = throttleAxis + (Time.fixedDeltaTime * 3f);
      if(throttleAxis > 1f){
        throttleAxis = 1f;
      }
      //If the car is going backwards, then apply brakes in order to avoid strange
      //behaviours. If the local velocity in the 'z' axis is less than -1f, then it
      //is safe to apply positive torque to go forward.
      if(localVelocityZ < -1f){
        Brakes();
      }else{
        if(Mathf.RoundToInt(carSpeed) < maxSpeed){
          // Apply positive torque in all wheels to go forward if maxSpeed has not been reached.
          frontLeftCollider.brakeTorque = 0;
          frontRightCollider.brakeTorque = 0;
          rearLeftCollider.brakeTorque = 0;
          rearRightCollider.brakeTorque = 0;

          if (useHorsepowerModel)
          {
            ApplyMotorTorqueFromPower(+1f);
          }
          else
          {
            frontLeftCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            frontRightCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            rearLeftCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            rearRightCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
          }
        }else {
          // If the maxSpeed has been reached, then stop applying torque to the wheels.
          // IMPORTANT: The maxSpeed variable should be considered as an approximation; the speed of the car
          // could be a bit higher than expected.
    			frontLeftCollider.motorTorque = 0;
    			frontRightCollider.motorTorque = 0;
          rearLeftCollider.motorTorque = 0;
    			rearRightCollider.motorTorque = 0;
    		}
      }
    }

    // This method apply negative torque to the wheels in order to go backwards.
    public void GoReverse(){
      //If the forces aplied to the rigidbody in the 'x' asis are greater than
      //3f, it means that the car is losing traction, then the car will start emitting particle systems.
      if(Mathf.Abs(localVelocityX) > 2.5f){
        isDrifting = true;
        DriftCarPS();
      }else{
        isDrifting = false;
        DriftCarPS();
      }
      // The following part sets the throttle power to -1 smoothly.
      throttleAxis = throttleAxis - (Time.fixedDeltaTime * 3f);
      if(throttleAxis < -1f){
        throttleAxis = -1f;
      }
      //If the car is still going forward, then apply brakes in order to avoid strange
      //behaviours. If the local velocity in the 'z' axis is greater than 1f, then it
      //is safe to apply negative torque to go reverse.
      if(localVelocityZ > 1f){
        Brakes();
      }else{
        if(Mathf.Abs(Mathf.RoundToInt(carSpeed)) < maxReverseSpeed){
          // Apply negative torque in all wheels to go in reverse if maxReverseSpeed has not been reached.
          frontLeftCollider.brakeTorque = 0;
          frontRightCollider.brakeTorque = 0;
          rearLeftCollider.brakeTorque = 0;
          rearRightCollider.brakeTorque = 0;

          if (useHorsepowerModel)
          {
            ApplyMotorTorqueFromPower(-1f);
          }
          else
          {
            frontLeftCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            frontRightCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            rearLeftCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
            rearRightCollider.motorTorque = (accelerationMultiplier * 50f) * throttleAxis;
          }
        }else {
          //If the maxReverseSpeed has been reached, then stop applying torque to the wheels.
          // IMPORTANT: The maxReverseSpeed variable should be considered as an approximation; the speed of the car
          // could be a bit higher than expected.
    			frontLeftCollider.motorTorque = 0;
    			frontRightCollider.motorTorque = 0;
          rearLeftCollider.motorTorque = 0;
    			rearRightCollider.motorTorque = 0;
    		}
      }
    }

    //The following function set the motor torque to 0 (in case the user is not pressing either W or S).
    public void ThrottleOff(){
      frontLeftCollider.motorTorque = 0;
      frontRightCollider.motorTorque = 0;
      rearLeftCollider.motorTorque = 0;
      rearRightCollider.motorTorque = 0;
    }

    void ApplyExternalAcceleration(float accelCmd)
    {
      // accelCmd is expected in [-1..1]. Positive = forward, negative = reverse.
      float cmd = Mathf.Clamp(accelCmd, -1f, 1f);

      // Drifting FX logic (same as forward/reverse)
      if(Mathf.Abs(localVelocityX) > 2.5f){
        isDrifting = true;
        DriftCarPS();
      }else{
        isDrifting = false;
        DriftCarPS();
      }

      // Smooth the internal throttleAxis towards the requested command
      throttleAxis = Mathf.MoveTowards(throttleAxis, cmd, Time.fixedDeltaTime * 3f);

      // Deadzone
      if (Mathf.Abs(throttleAxis) < 0.001f)
      {
        throttleAxis = 0f;
        DecelerateCar();
        return;
      }

      // Prevent instant direction flips
      if (throttleAxis > 0f && localVelocityZ < -1f)
      {
        Brakes();
        return;
      }
      if (throttleAxis < 0f && localVelocityZ > 1f)
      {
        Brakes();
        return;
      }

      // Apply torque
      frontLeftCollider.brakeTorque = 0;
      frontRightCollider.brakeTorque = 0;
      rearLeftCollider.brakeTorque = 0;
      rearRightCollider.brakeTorque = 0;

      if (throttleAxis > 0f)
      {
        float speedKmhForCap = (useHorsepowerModel ? speedFromRigidbodyKmh : carSpeed);
        if (Mathf.RoundToInt(speedKmhForCap) < maxSpeed)
        {
          if (useHorsepowerModel) ApplyMotorTorqueFromPower(+1f, Mathf.Abs(throttleAxis));
          else
          {
            float t = (accelerationMultiplier * 50f) * throttleAxis * torqueScale;
            frontLeftCollider.motorTorque = t;
            frontRightCollider.motorTorque = t;
            rearLeftCollider.motorTorque = t;
            rearRightCollider.motorTorque = t;
          }
        }
        else
        {
          ThrottleOff();
        }
      }
      else
      {
        if (externalNegativeMeansBrake)
        {
          // Brake proportionally to requested negative command
          ThrottleOff();
          float brake = Mathf.Clamp01(Mathf.Abs(throttleAxis));
          frontLeftCollider.brakeTorque = brakeForce * brake;
          frontRightCollider.brakeTorque = brakeForce * brake;
          rearLeftCollider.brakeTorque = brakeForce * brake;
          rearRightCollider.brakeTorque = brakeForce * brake;
          return;
        }

        float speedKmhForCap = (useHorsepowerModel ? speedFromRigidbodyKmh : carSpeed);
        if (Mathf.Abs(Mathf.RoundToInt(speedKmhForCap)) < maxReverseSpeed)
        {
          if (useHorsepowerModel) ApplyMotorTorqueFromPower(-1f, Mathf.Abs(throttleAxis));
          else
          {
            float t = (accelerationMultiplier * 50f) * throttleAxis * torqueScale;
            frontLeftCollider.motorTorque = t;
            frontRightCollider.motorTorque = t;
            rearLeftCollider.motorTorque = t;
            rearRightCollider.motorTorque = t;
          }
        }
        else
        {
          ThrottleOff();
        }
      }
    }

    void ApplyMotorTorqueFromPower(float directionSign)
    {
      ApplyMotorTorqueFromPower(directionSign, Mathf.Clamp01(Mathf.Abs(throttleAxis)));
    }

    void ApplyMotorTorqueFromPower(float directionSign, float pedalOverride01)
    {
      // directionSign: +1 forward, -1 reverse
      float pedal = Mathf.Clamp01(pedalOverride01);
      lastPedal01 = pedal;
      if (pedal <= 0.001f)
      {
        frontLeftCollider.motorTorque = 0;
        frontRightCollider.motorTorque = 0;
        rearLeftCollider.motorTorque = 0;
        rearRightCollider.motorTorque = 0;
        return;
      }

      float massKg = (massSourceRigidbody != null) ? massSourceRigidbody.mass : ((carRigidbody != null) ? carRigidbody.mass : 1200f);
      float g = 9.81f;
      float maxAccel = Mathf.Max(0.1f, maxLongitudinalAccelerationG) * g;
      float maxForceFromG = massKg * maxAccel;
      lastAccelCapMS2 = maxAccel;

      // Power available (Watts)
      float maxPowerW = Mathf.Max(1f, horsepower) * 745.699872f * Mathf.Clamp01(drivetrainEfficiency);
      float requestedPowerW = maxPowerW * pedal;
      lastAvailablePowerW = maxPowerW;

      // Energy limit (Joules)
      lastEnergyLimited = false;
      if (maxEnergyKWh > 0f)
      {
        float maxEnergyJ = maxEnergyKWh * 3_600_000f;
        currentEnergyJ = Mathf.Clamp(currentEnergyJ, 0f, maxEnergyJ);
        float dt = Time.fixedDeltaTime;
        float energyNeeded = requestedPowerW * dt;
        if (energyNeeded > currentEnergyJ)
        {
          requestedPowerW = (dt > 0f) ? (currentEnergyJ / dt) : 0f;
          lastEnergyLimited = true;
        }
        currentEnergyJ = Mathf.Max(0f, currentEnergyJ - requestedPowerW * dt);
      }

      lastRequestedPowerW = requestedPowerW;

      // Convert power to tractive force: P = F * v  =>  F = P / v
      float v = 0f;
      if (carRigidbody != null)
      {
        // Use forward speed magnitude (m/s). Avoid huge force at v~0.
        v = Mathf.Abs(Vector3.Dot(carRigidbody.linearVelocity, transform.forward));
      }
      v = Mathf.Max(powerModelMinSpeedMS, v);

      float forceN = requestedPowerW / v;
      forceN = Mathf.Min(forceN, maxForceFromG);
      lastTractiveForceN = forceN;

      float wheelRadius = frontLeftCollider != null ? frontLeftCollider.radius : 0.35f;
      float totalTorque = forceN * wheelRadius;

      int drivenCount = 0;
      if (frontLeftCollider != null) drivenCount++;
      if (frontRightCollider != null) drivenCount++;
      if (rearLeftCollider != null) drivenCount++;
      if (rearRightCollider != null) drivenCount++;
      drivenCount = Mathf.Max(1, drivenCount);

      float torquePerWheel = (totalTorque / drivenCount) * torqueScale;
      lastTorquePerWheelNm = torquePerWheel;

      float signedTorque = torquePerWheel * Mathf.Sign(directionSign);

      if (frontLeftCollider != null) frontLeftCollider.motorTorque = signedTorque;
      if (frontRightCollider != null) frontRightCollider.motorTorque = signedTorque;
      if (rearLeftCollider != null) rearLeftCollider.motorTorque = signedTorque;
      if (rearRightCollider != null) rearRightCollider.motorTorque = signedTorque;
    }

    void UpdateTractionControl()
    {
      if (!tractionControlEnabled)
      {
        slipKmh = 0f;
        torqueScale = 1f;
        return;
      }

      // Use smoothed RB speed for stability.
      float rbKmh = speedFromRigidbodyKmhSmoothed > 0.001f ? speedFromRigidbodyKmhSmoothed : speedFromRigidbodyKmh;
      float wheelKmh = carSpeed;

      // Positive means wheels are spinning faster than actual motion (slip).
      float signedSlip = wheelKmh - rbKmh;
      slipKmh = Mathf.Abs(signedSlip);

      float desired = 1f;
      // Only reduce torque for wheel spin (wheel > rb), not for wheel drag.
      if (signedSlip > slipThresholdKmh)
      {
        float excess = signedSlip - slipThresholdKmh;
        float t = Mathf.Clamp01(excess / Mathf.Max(0.1f, slipRangeKmh));
        desired = Mathf.Lerp(1f, minTorqueScale, t);
      }

      float dt = Time.fixedDeltaTime;
      if (dt <= 0f) dt = 0.02f;
      float alpha = 1f - Mathf.Exp(-dt * tcResponse);
      torqueScale = Mathf.Lerp(torqueScale, desired, alpha);
    }

    // The following method decelerates the speed of the car according to the decelerationMultiplier variable, where
    // 1 is the slowest and 10 is the fastest deceleration. This method is called by the function InvokeRepeating,
    // usually every 0.1f when the user is not pressing W (throttle), S (reverse) or Space bar (handbrake).
    public void DecelerateCar(){
      if(Mathf.Abs(localVelocityX) > 2.5f){
        isDrifting = true;
        DriftCarPS();
      }else{
        isDrifting = false;
        DriftCarPS();
      }
      // The following part resets the throttle power to 0 smoothly.
      if(throttleAxis != 0f){
        if(throttleAxis > 0f){
          throttleAxis = throttleAxis - (Time.fixedDeltaTime * 10f);
        }else if(throttleAxis < 0f){
            throttleAxis = throttleAxis + (Time.fixedDeltaTime * 10f);
        }
        if(Mathf.Abs(throttleAxis) < 0.15f){
          throttleAxis = 0f;
        }
      }
      carRigidbody.linearVelocity = carRigidbody.linearVelocity * (1f / (1f + (0.025f * decelerationMultiplier)));
      // Since we want to decelerate the car, we are going to remove the torque from the wheels of the car.
      frontLeftCollider.motorTorque = 0;
      frontRightCollider.motorTorque = 0;
      rearLeftCollider.motorTorque = 0;
      rearRightCollider.motorTorque = 0;
      // If the magnitude of the car's velocity is less than 0.25f (very slow velocity), then stop the car completely and
      // also cancel the invoke of this method.
      if(carRigidbody.linearVelocity.magnitude < 0.25f){
        carRigidbody.linearVelocity = Vector3.zero;
        CancelInvoke("DecelerateCar");
      }
    }

    // This function applies brake torque to the wheels according to the brake force given by the user.
    public void Brakes(){
      frontLeftCollider.brakeTorque = brakeForce;
      frontRightCollider.brakeTorque = brakeForce;
      rearLeftCollider.brakeTorque = brakeForce;
      rearRightCollider.brakeTorque = brakeForce;
    }

    // This function is used to make the car lose traction. By using this, the car will start drifting. The amount of traction lost
    // will depend on the handbrakeDriftMultiplier variable. If this value is small, then the car will not drift too much, but if
    // it is high, then you could make the car to feel like going on ice.
    public void Handbrake(){
      CancelInvoke("RecoverTraction");
      // We are going to start losing traction smoothly, there is were our 'driftingAxis' variable takes
      // place. This variable will start from 0 and will reach a top value of 1, which means that the maximum
      // drifting value has been reached. It will increase smoothly by using the variable Time.fixedDeltaTime.
      driftingAxis = driftingAxis + (Time.fixedDeltaTime);
      float secureStartingPoint = driftingAxis * FLWextremumSlip * handbrakeDriftMultiplier;

      if(secureStartingPoint < FLWextremumSlip){
        driftingAxis = FLWextremumSlip / (FLWextremumSlip * handbrakeDriftMultiplier);
      }
      if(driftingAxis > 1f){
        driftingAxis = 1f;
      }
      //If the forces aplied to the rigidbody in the 'x' asis are greater than
      //3f, it means that the car lost its traction, then the car will start emitting particle systems.
      if(Mathf.Abs(localVelocityX) > 2.5f){
        isDrifting = true;
      }else{
        isDrifting = false;
      }
      //If the 'driftingAxis' value is not 1f, it means that the wheels have not reach their maximum drifting
      //value, so, we are going to continue increasing the sideways friction of the wheels until driftingAxis
      // = 1f.
      if(driftingAxis < 1f){
        FLwheelFriction.extremumSlip = FLWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        frontLeftCollider.sidewaysFriction = FLwheelFriction;

        FRwheelFriction.extremumSlip = FRWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        frontRightCollider.sidewaysFriction = FRwheelFriction;

        RLwheelFriction.extremumSlip = RLWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        rearLeftCollider.sidewaysFriction = RLwheelFriction;

        RRwheelFriction.extremumSlip = RRWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        rearRightCollider.sidewaysFriction = RRwheelFriction;
      }

      // Whenever the player uses the handbrake, it means that the wheels are locked, so we set 'isTractionLocked = true'
      // and, as a consequense, the car starts to emit trails to simulate the wheel skids.
      isTractionLocked = true;
      DriftCarPS();

    }

    // This function is used to emit both the particle systems of the tires' smoke and the trail renderers of the tire skids
    // depending on the value of the bool variables 'isDrifting' and 'isTractionLocked'.
    public void DriftCarPS(){

      if(useEffects){
        try{
          if(isDrifting){
            RLWParticleSystem.Play();
            RRWParticleSystem.Play();
          }else if(!isDrifting){
            RLWParticleSystem.Stop();
            RRWParticleSystem.Stop();
          }
        }catch(Exception ex){
          Debug.LogWarning(ex);
        }

        try{
          if((isTractionLocked || Mathf.Abs(localVelocityX) > 5f) && Mathf.Abs(carSpeed) > 12f){
            RLWTireSkid.emitting = true;
            RRWTireSkid.emitting = true;
          }else {
            RLWTireSkid.emitting = false;
            RRWTireSkid.emitting = false;
          }
        }catch(Exception ex){
          Debug.LogWarning(ex);
        }
      }else if(!useEffects){
        if(RLWParticleSystem != null){
          RLWParticleSystem.Stop();
        }
        if(RRWParticleSystem != null){
          RRWParticleSystem.Stop();
        }
        if(RLWTireSkid != null){
          RLWTireSkid.emitting = false;
        }
        if(RRWTireSkid != null){
          RRWTireSkid.emitting = false;
        }
      }

    }

    // This function is used to recover the traction of the car when the user has stopped using the car's handbrake.
    public void RecoverTraction(){
      isTractionLocked = false;
      driftingAxis = driftingAxis - (Time.fixedDeltaTime / 1.5f);
      if(driftingAxis < 0f){
        driftingAxis = 0f;
      }

      //If the 'driftingAxis' value is not 0f, it means that the wheels have not recovered their traction.
      //We are going to continue decreasing the sideways friction of the wheels until we reach the initial
      // car's grip.
      if(FLwheelFriction.extremumSlip > FLWextremumSlip){
        FLwheelFriction.extremumSlip = FLWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        frontLeftCollider.sidewaysFriction = FLwheelFriction;

        FRwheelFriction.extremumSlip = FRWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        frontRightCollider.sidewaysFriction = FRwheelFriction;

        RLwheelFriction.extremumSlip = RLWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        rearLeftCollider.sidewaysFriction = RLwheelFriction;

        RRwheelFriction.extremumSlip = RRWextremumSlip * handbrakeDriftMultiplier * driftingAxis;
        rearRightCollider.sidewaysFriction = RRwheelFriction;

        Invoke("RecoverTraction", Time.fixedDeltaTime);

      }else if (FLwheelFriction.extremumSlip < FLWextremumSlip){
        FLwheelFriction.extremumSlip = FLWextremumSlip;
        frontLeftCollider.sidewaysFriction = FLwheelFriction;

        FRwheelFriction.extremumSlip = FRWextremumSlip;
        frontRightCollider.sidewaysFriction = FRwheelFriction;

        RLwheelFriction.extremumSlip = RLWextremumSlip;
        rearLeftCollider.sidewaysFriction = RLwheelFriction;

        RRwheelFriction.extremumSlip = RRWextremumSlip;
        rearRightCollider.sidewaysFriction = RRwheelFriction;

        driftingAxis = 0f;
      }
    }

}

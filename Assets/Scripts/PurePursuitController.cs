using UnityEngine;
using UnityEngine.InputSystem;

public class PurePursuitController : MonoBehaviour
{
    [Header("References")]
    public PrometeoCarController carController;
    public Transform destinationMarker;

    [Header("Pure Pursuit Settings")]
    public float lookAheadDistance = 5.0f;
    public float targetSpeed = 20.0f;
    public float wheelbase = 2.6f;
    public bool isAutonomous = true;

    [Header("Startup")]
    public bool startIdle = true;

    [Header("PID / Sensitivity")]
    [Range(0.1f, 10.0f)]
    public float steeringSensitivity = 1.5f;
    [Range(0.1f, 1.0f)]
    public float accelerationSensitivity = 0.8f;

    [Header("Speed PID (Acceleration)")]
    public bool useSpeedPid = true;
    public bool allowReverseForBraking = true;
    [Min(0f)]
    public float speedKp = 0.08f;
    [Min(0f)]
    public float speedKi = 0.02f;
    [Min(0f)]
    public float speedKd = 0.01f;
    [Min(0f)]
    public float speedIntegralLimit = 50f;
    [Min(0f)]
    public float speedIntegralDecay = 3f;
    [Min(0f)]
    public float accelOutputDeadzone = 0.02f;

    [Header("Debug")]
    public float debugDesiredSpeedKmh;

    [Header("Logic Settings")]
    public float arrivalThreshold = 3.0f;
    public float circleRadius = 5.0f;
    public bool isCircling = false;

    private Vector3 currentTargetPoint;
    private LineRenderer targetingLine;
    private Rigidbody rb;
    private bool hasForcedTarget;

    private float speedIntegral;
    private float lastSpeedError;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (carController == null) carController = GetComponent<PrometeoCarController>();
        
        targetingLine = GetComponent<LineRenderer>();
        if (targetingLine == null) targetingLine = gameObject.AddComponent<LineRenderer>();
        SetupLine(targetingLine, Color.blue, 0.4f);

        // Default behavior: stay idle until a click.
        // If a manager already set a forced target (e.g., optimizer), do not override it.
        if (!hasForcedTarget)
        {
            currentTargetPoint = transform.position;
            isCircling = false;
            if (startIdle)
            {
                isAutonomous = false;
            }
        }
    }

    void SetupLine(LineRenderer lr, Color col, float width)
    {
        Shader shader = Shader.Find("Hidden/Internal-Colored");
        if (shader != null) lr.material = new Material(shader);
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = col;
        lr.endColor = col;
        lr.positionCount = 0;
    }

    void Update()
    {
        // Mouse click logic to set target
        if (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame)
        {
            Vector2 mousePos = Mouse.current.position.ReadValue();
            Ray ray = Camera.main.ScreenPointToRay(mousePos);
            
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                Debug.Log("Target Updated: " + hit.point);
                currentTargetPoint = hit.point;
                isCircling = false; 
                isAutonomous = true; // Activate!

                ResetSpeedPid();
                
                if (destinationMarker != null)
                {
                    destinationMarker.position = currentTargetPoint + Vector3.up * 0.1f;
                }
            }
        }
    }

    void FixedUpdate()
    {
        if (!isAutonomous || carController == null)
        {
            if (targetingLine != null) targetingLine.enabled = false;
            if (carController != null && !isAutonomous) {
                carController.useExternalInput = false;
            }
            return;
        }

        float distToTarget = Vector3.Distance(transform.position, currentTargetPoint);

        // Arrival logic
        if (!isCircling && distToTarget < arrivalThreshold)
        {
            isCircling = true;
            Debug.Log("Arrived at target! Starting Circle.");
        }

        Vector3 targetPoint;
        if (isCircling)
        {
            // Point 5m side, 5m forward relative to car (parameterized)
            // request: 5 ahead, 5 left (using -radius for left)
            targetPoint = transform.TransformPoint(new Vector3(-circleRadius, 0, lookAheadDistance));
        }
        else
        {
            targetPoint = currentTargetPoint;
        }

        float steerCommand = CalculatePurePursuitSteering(targetPoint);
        
        carController.useExternalInput = true;
        carController.externalSteering = steerCommand;

        // Acceleration logic
        float steerAbs = Mathf.Abs(steerCommand);
        float speedFactor = Mathf.Clamp01(1.2f - steerAbs); // slow down slightly in hard turns
        float desiredSpeed = targetSpeed * speedFactor;
        debugDesiredSpeedKmh = desiredSpeed;
        carController.externalAcceleration = ComputeAccelerationCommand(desiredSpeed);

        if (targetingLine != null) DrawTelemetry(targetPoint);
    }

    float CalculatePurePursuitSteering(Vector3 targetPos)
    {
        Vector3 localTarget = transform.InverseTransformPoint(targetPos);
        
        // Pure Pursuit angle alpha
        float alpha = Mathf.Atan2(localTarget.x, localTarget.z);
        
        // Formula: delta = atan(2 * L * sin(alpha) / lookahead)
        // ld is lookahead distance.
        float ld = lookAheadDistance;
        
        float steeringAngle = Mathf.Atan2(2.0f * wheelbase * Mathf.Sin(alpha), ld) * Mathf.Rad2Deg;

        // Apply a sensitivity multiplier
        float command = (steeringAngle / carController.maxSteeringAngle) * steeringSensitivity;
        
        return Mathf.Clamp(command, -1.0f, 1.0f);
    }


    public void SetForceTarget(Vector3 pos)
    {
        hasForcedTarget = true;
        currentTargetPoint = pos;
        isAutonomous = true;
        isCircling = false;

        ResetSpeedPid();

        if (destinationMarker != null)
        {
            destinationMarker.position = currentTargetPoint + Vector3.up * 0.1f;
        }
    }

    float ComputeAccelerationCommand(float desiredSpeedKmh)
    {
        if (carController == null) return 0f;

        // If PID disabled, fall back to simple bang-bang throttle.
        if (!useSpeedPid)
        {
            return (carController.carSpeed < desiredSpeedKmh) ? accelerationSensitivity : 0f;
        }

        float dt = Time.fixedDeltaTime;
        if (dt <= 0f) dt = 0.02f;

        // carSpeed is already in km/h.
        float speedError = desiredSpeedKmh - carController.carSpeed;

        // Integrator management: if we don't allow braking (reverse), don't wind up negative.
        if (!allowReverseForBraking && speedError < 0f)
        {
            speedIntegral = Mathf.MoveTowards(speedIntegral, 0f, speedIntegralDecay * dt);
        }
        else
        {
            speedIntegral += speedError * dt;
            speedIntegral = Mathf.Clamp(speedIntegral, -speedIntegralLimit, speedIntegralLimit);
        }

        float speedDerivative = (speedError - lastSpeedError) / dt;
        lastSpeedError = speedError;

        float u = (speedKp * speedError) + (speedKi * speedIntegral) + (speedKd * speedDerivative);

        float minOut = allowReverseForBraking ? -1f : 0f;
        float maxOut = 1f;
        float cmd = Mathf.Clamp(u, minOut, maxOut);

        if (Mathf.Abs(cmd) < accelOutputDeadzone)
            cmd = 0f;

        return cmd;
    }

    void ResetSpeedPid()
    {
        speedIntegral = 0f;
        lastSpeedError = 0f;
    }


    void DrawTelemetry(Vector3 targetPoint)
    {
        targetingLine.enabled = true;
        targetingLine.positionCount = 2;
        targetingLine.SetPosition(0, transform.position + Vector3.up * 0.5f);
        targetingLine.SetPosition(1, targetPoint + Vector3.up * 0.5f);
    }
}


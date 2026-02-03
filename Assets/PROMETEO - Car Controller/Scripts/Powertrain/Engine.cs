using System;
using UnityEngine;

[Serializable]
public struct EngineConfig
{
    public float idleRPM;
    public float redlineRPM;
    public float inertia;
    public float drag;
    public float peakTorque;
    public AnimationCurve torqueCurve;
    public bool useRevLimiter;
}

public class Engine
{
    public float CurrentRPM { get; private set; }

    readonly float idleRPM;
    readonly float redlineRPM;
    readonly float inertia;
    readonly float engineDrag;
    readonly float peakTorque;
    readonly AnimationCurve torqueCurve;
    readonly bool useRevLimiter;
    readonly VehicleTelemetry telemetry;

    float rpmVelocity;

    public Engine(EngineConfig config, VehicleTelemetry telemetry)
    {
        idleRPM = config.idleRPM;
        redlineRPM = config.redlineRPM;
        inertia = config.inertia;
        engineDrag = config.drag;
        peakTorque = config.peakTorque;
        torqueCurve = config.torqueCurve;
        useRevLimiter = config.useRevLimiter;
        this.telemetry = telemetry;

        CurrentRPM = idleRPM;
        rpmVelocity = 0f;

        if (telemetry != null)
        {
            telemetry.engineRPM = CurrentRPM;
            telemetry.engineTorque = 0f;
        }
    }

    public float UpdateRPM(float wheelRPM, float throttle, float drivetrainLock, float dt)
    {
        float throttleAbs = Mathf.Clamp01(Mathf.Abs(throttle));
        float freeRPM = Mathf.Lerp(idleRPM, redlineRPM, throttleAbs);
        float target = Mathf.Lerp(freeRPM, wheelRPM, drivetrainLock);
        target = Mathf.Max(idleRPM, target);

        float smoothTime = 0.12f * inertia;
        CurrentRPM = Mathf.SmoothDamp(CurrentRPM, target, ref rpmVelocity, smoothTime, Mathf.Infinity, dt);

        if (useRevLimiter && CurrentRPM > redlineRPM)
        {
            CurrentRPM = redlineRPM;
        }

        if (telemetry != null)
        {
            telemetry.engineRPM = CurrentRPM;
        }

        return CurrentRPM;
    }

    public float GetTorque(float throttle)
    {
        float torque;
        if (Mathf.Abs(throttle) < 0.05f)
        {
            torque = -engineDrag * (CurrentRPM / redlineRPM);
        }
        else
        {
            torque = peakTorque * torqueCurve.Evaluate(CurrentRPM) * throttle;
        }

        if (telemetry != null)
        {
            telemetry.engineTorque = torque;
        }

        return torque;
    }
}

using System;
using UnityEngine;

[Serializable]
public struct DrivetrainConfig
{
    public float elasticity;
    public float lockRecoveryTime;
    public float converterMaxFactor;
    public float converterFadeSpeed;
    public int converterGear;
}

public class Drivetrain
{
    public float LockAmount => lockAmount;

    readonly float elasticity;
    readonly float lockRecovery;
    readonly float converterMaxFactor;
    readonly float converterFadeSpeed;
    readonly int converterGear;
    readonly VehicleTelemetry telemetry;

    float lockAmount = 1f;
    float smoothedTorque;

    public Drivetrain(DrivetrainConfig config, VehicleTelemetry telemetry)
    {
        elasticity = Mathf.Max(0.0001f, config.elasticity);
        lockRecovery = Mathf.Max(0.0001f, config.lockRecoveryTime);
        converterMaxFactor = config.converterMaxFactor;
        converterFadeSpeed = Mathf.Max(0.0001f, config.converterFadeSpeed);
        converterGear = Mathf.Max(1, config.converterGear);
        this.telemetry = telemetry;

        if (telemetry != null)
        {
            telemetry.drivetrainLock = lockAmount;
            telemetry.wheelTorque = smoothedTorque;
        }
    }

    public void OnShift()
    {
        lockAmount = 0f;
        if (telemetry != null)
        {
            telemetry.drivetrainLock = lockAmount;
        }
    }

    public void UpdateLock(float dt)
    {
        lockAmount = Mathf.MoveTowards(lockAmount, 1f, dt / lockRecovery);
        if (telemetry != null)
        {
            telemetry.drivetrainLock = lockAmount;
        }
    }

    public float Update(
        float engineTorque,
        float ratio,
        float carSpeed,
        int currentGear,
        bool isShifting,
        float dt)
    {
        engineTorque = ApplyTorqueConverter(engineTorque, carSpeed, currentGear);

        float wheelTorque = engineTorque * ratio * lockAmount;
        float targetTorque = isShifting ? 0f : wheelTorque;

        smoothedTorque = Mathf.Lerp(smoothedTorque, targetTorque, dt / elasticity);

        if (telemetry != null)
        {
            telemetry.wheelTorque = smoothedTorque;
        }

        return smoothedTorque;
    }

    float ApplyTorqueConverter(float engineTorque, float carSpeed, int currentGear)
    {
        if (currentGear == converterGear && carSpeed < converterFadeSpeed)
        {
            float t = Mathf.Clamp01(carSpeed / converterFadeSpeed);
            float converterFactor = Mathf.Lerp(converterMaxFactor, 1.0f, t);
            engineTorque *= converterFactor;
        }

        return engineTorque;
    }
}

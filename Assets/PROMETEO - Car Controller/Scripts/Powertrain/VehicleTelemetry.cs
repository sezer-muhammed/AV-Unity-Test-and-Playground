using System;
using UnityEngine;

[Serializable]
public class VehicleTelemetry
{
    [Header("Engine")]
    public float engineRPM;
    public float engineTorque;

    [Header("Transmission")]
    public int gear;
    public bool isShifting;

    [Header("Drivetrain")]
    public float drivetrainLock;
    public float wheelTorque;

    [Header("Vehicle")]
    public float carSpeed;
    public float wheelSpeed;
    public float wheelSlip;
    public float steeringAngle;
}

using System;

[Serializable]
public struct TransmissionConfig
{
    public float[] gears;
    public float finalDrive;
    public float shiftUpRPM;
    public float shiftDownRPM;
    public float shiftTime;
}

public class Transmission
{
    public int CurrentGear { get; private set; } = 1;
    public bool IsShifting { get; private set; }

    readonly float[] gears;
    readonly float finalDrive;
    readonly float shiftUpRPM;
    readonly float shiftDownRPM;
    readonly float shiftTime;
    readonly VehicleTelemetry telemetry;

    float timer;

    public float TotalRatio => gears[CurrentGear - 1] * finalDrive;

    public Transmission(TransmissionConfig config, VehicleTelemetry telemetry)
    {
        gears = config.gears;
        finalDrive = config.finalDrive;
        shiftUpRPM = config.shiftUpRPM;
        shiftDownRPM = config.shiftDownRPM;
        shiftTime = config.shiftTime;
        this.telemetry = telemetry;

        if (telemetry != null)
        {
            telemetry.gear = CurrentGear;
            telemetry.isShifting = IsShifting;
        }
    }

    public bool Update(float rpm, float dt)
    {
        bool shiftStarted = false;

        if (IsShifting)
        {
            timer -= dt;
            if (timer <= 0f)
            {
                IsShifting = false;
            }
        }
        else if (rpm > shiftUpRPM && CurrentGear < gears.Length)
        {
            Shift(CurrentGear + 1);
            shiftStarted = true;
        }
        else if (rpm < shiftDownRPM && CurrentGear > 1)
        {
            Shift(CurrentGear - 1);
            shiftStarted = true;
        }

        if (telemetry != null)
        {
            telemetry.gear = CurrentGear;
            telemetry.isShifting = IsShifting;
        }

        return shiftStarted;
    }

    void Shift(int newGear)
    {
        CurrentGear = newGear;
        IsShifting = true;
        timer = shiftTime;
    }
}

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

public enum GearMode
{
    Reverse,
    Neutral,
    Drive
}

public class Transmission
{
    public int CurrentGear { get; private set; } = 1;
    public bool IsShifting { get; private set; }
    public GearMode Mode { get; private set; } = GearMode.Drive;

    readonly float[] gears;
    readonly float finalDrive;
    readonly float reverseRatio;
    readonly float shiftUpRPM;
    readonly float shiftDownRPM;
    readonly float shiftTime;
    readonly VehicleTelemetry telemetry;

    float timer;

    public float TotalRatio
    {
        get
        {
            if (Mode == GearMode.Neutral) return 0f;
            if (Mode == GearMode.Reverse) return reverseRatio * finalDrive;
            return gears[CurrentGear - 1] * finalDrive;
        }
    }

    public Transmission(TransmissionConfig config, float reverseRatio, VehicleTelemetry telemetry)
    {
        gears = config.gears;
        finalDrive = config.finalDrive;
        this.reverseRatio = reverseRatio;
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

    public void SetMode(GearMode newMode)
    {
        Mode = newMode;
        if (Mode == GearMode.Drive) CurrentGear = 1;
    }

    public bool Update(float rpm, float dt, bool allowShift)
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
        else if (Mode == GearMode.Drive && allowShift)
        {
            if (rpm > shiftUpRPM && CurrentGear < gears.Length)
            {
                Shift(CurrentGear + 1);
                shiftStarted = true;
            }
            else if (rpm < shiftDownRPM && CurrentGear > 1)
            {
                Shift(CurrentGear - 1);
                shiftStarted = true;
            }
        }

        if (telemetry != null)
        {
            telemetry.gear = (Mode == GearMode.Reverse) ? -1 : (Mode == GearMode.Neutral ? 0 : CurrentGear);
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

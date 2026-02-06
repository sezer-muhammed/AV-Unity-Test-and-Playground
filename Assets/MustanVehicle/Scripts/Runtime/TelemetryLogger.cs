using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class TelemetryLogger : MonoBehaviour
{
    [Serializable]
    public class TelemetryEntry
    {
        public float time;
        public float rpm;
        public int gear;
        public bool isShifting;
        public float carSpeed;
        public float wheelSpeed;
        public float wheelSpeedFL;
        public float wheelSpeedFR;
        public float wheelSpeedRL;
        public float wheelSpeedRR;
        public float torque;
        public float engineTorque;
        public float drivetrainLock;
        public float slip;
        public float steering;
    }

    [Serializable]
    public class TelemetryWrapper
    {
        public List<TelemetryEntry> data = new List<TelemetryEntry>();
    }

    public PrometeoCarController car;
    public string fileName = "VehicleTelemetry.json";
    public bool isLogging = true;

    private TelemetryWrapper wrapper = new TelemetryWrapper();
    private float startTime;

    void Start()
    {
        if (car == null) car = GetComponent<PrometeoCarController>();
        startTime = Time.time;
        Debug.Log("Telemetry Logging Started. Data will save to: " + Path.Combine(Application.dataPath, fileName));
    }

    void FixedUpdate()
    {
        if (!isLogging || car == null || car.telemetry == null) return;

        VehicleTelemetry data = car.telemetry;
        TelemetryEntry entry = new TelemetryEntry
        {
            time = Time.time - startTime,
            rpm = data.engineRPM,
            gear = data.gear,
            isShifting = data.isShifting,
            carSpeed = data.carSpeed,
            wheelSpeed = data.wheelSpeed,
            wheelSpeedFL = data.wheelSpeedFL,
            wheelSpeedFR = data.wheelSpeedFR,
            wheelSpeedRL = data.wheelSpeedRL,
            wheelSpeedRR = data.wheelSpeedRR,
            torque = data.wheelTorque,
            engineTorque = data.engineTorque,
            drivetrainLock = data.drivetrainLock,
            slip = data.wheelSlip,
            steering = data.steeringAngle
        };

        wrapper.data.Add(entry);
    }

    void OnApplicationQuit()
    {
        SaveData();
    }

    public void SaveData()
    {
        string json = JsonUtility.ToJson(wrapper, true);
        string path = Path.Combine(Application.dataPath, fileName);
        File.WriteAllText(path, json);
        Debug.Log($"Telemetry Saved: {wrapper.data.Count} frames recorded to {path}");
    }
}

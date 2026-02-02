using UnityEngine;
using UnityEngine.InputSystem;

public class TelemetryHUD : MonoBehaviour
{
    [Header("References (optional)")]
    public PrometeoCarController car;
    public PurePursuitController pursuit;

    [Header("UI")]
    public bool visible = true;
    public int fontSize = 16;
    public Vector2 padding = new Vector2(12, 12);
    public Color textColor = new Color(1f, 1f, 1f, 0.95f);
    public Color backgroundColor = new Color(0f, 0f, 0f, 0.55f);

    [Header("Layout")]
    public float panelWidth = 720f;
    public float panelHeight = 300f;
    public float rowHeight = 18f;
    public float labelWidth = 180f;
    public float valueWidth = 320f;

    [Header("Bars")]
    public float barWidth = 240f;
    public float barHeight = 14f;
    public Color barBack = new Color(1f, 1f, 1f, 0.10f);
    public Color barFillPos = new Color(0.25f, 0.9f, 0.35f, 0.95f);
    public Color barFillNeg = new Color(0.95f, 0.35f, 0.25f, 0.95f);

    [Header("Hotkey")]
    public Key toggleKey = Key.F3;

    GUIStyle headerStyle;
    GUIStyle labelStyle;
    GUIStyle valueStyle;
    GUIStyle smallStyle;

    void Awake()
    {
        EnsureRefs();
    }

    void Update()
    {
        if (Keyboard.current != null)
        {
            Key keyToUse = toggleKey;
            if (keyToUse == Key.None)
                keyToUse = Key.F3;

            try
            {
                var key = Keyboard.current[keyToUse];
                if (key != null && key.wasPressedThisFrame)
                    visible = !visible;
            }
            catch
            {
                // If the serialized value is invalid for this InputSystem version/device, ignore.
            }
        }

        if (car == null || pursuit == null)
            EnsureRefs();
    }

    void EnsureRefs()
    {
        if (car == null)
            car = FindFirstObjectByType<PrometeoCarController>();

        if (pursuit == null)
            pursuit = FindFirstObjectByType<PurePursuitController>();
    }

    void OnGUI()
    {
        if (!visible) return;
        if (car == null && pursuit == null) return;

        EnsureStyles();

        Rect boxRect = new Rect(10, Screen.height - panelHeight - 10, panelWidth, panelHeight);
        DrawRect(boxRect, backgroundColor);

        float x = boxRect.x + padding.x;
        float y = boxRect.y + padding.y;

        // Left info panel (fixed rows, no wrapping)
        float gap = 18f;
        float rightPanelWidth = Mathf.Max(320f, barWidth + 210f);
        float leftWidth = boxRect.width - padding.x * 2f - rightPanelWidth - gap;
        if (leftWidth < 340f) leftWidth = 340f;

        Rect leftRect = new Rect(x, y, leftWidth, boxRect.height - padding.y * 2f);
        Rect rightRect = new Rect(leftRect.xMax + gap, y, rightPanelWidth, boxRect.height - padding.y * 2f);

        DrawTelemetryTable(leftRect);
        DrawInputsPanel(rightRect);
    }

    void EnsureStyles()
    {
        if (labelStyle != null) return;

        headerStyle = new GUIStyle(GUI.skin.label);
        headerStyle.fontSize = fontSize + 2;
        headerStyle.normal.textColor = textColor;
        headerStyle.richText = true;
        headerStyle.wordWrap = false;

        labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = fontSize;
        labelStyle.normal.textColor = new Color(textColor.r, textColor.g, textColor.b, 0.85f);
        labelStyle.richText = true;
        labelStyle.wordWrap = false;

        valueStyle = new GUIStyle(GUI.skin.label);
        valueStyle.fontSize = fontSize;
        valueStyle.normal.textColor = textColor;
        valueStyle.richText = true;
        valueStyle.wordWrap = false;

        smallStyle = new GUIStyle(GUI.skin.label);
        smallStyle.fontSize = Mathf.Max(12, fontSize - 2);
        smallStyle.normal.textColor = new Color(textColor.r, textColor.g, textColor.b, 0.70f);
        smallStyle.richText = true;
        smallStyle.wordWrap = false;
    }

    void DrawTelemetryTable(Rect rect)
    {
        float cursorY = rect.y;
        GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight + 2f), "<b>Telemetry</b>", headerStyle);
        cursorY += rowHeight + 6f;

        if (car != null)
        {
            DrawRow(rect, ref cursorY, "Mode", car.useExternalInput ? "External" : "Player");
            DrawRow(rect, ref cursorY, "Speed (Pos, smooth)", $"{car.DebugSpeedFromPositionKmhSmoothed:0.0} km/h");
            DrawRow(rect, ref cursorY, "Speed (RB, smooth)", $"{car.DebugSpeedFromRigidbodyKmhSmoothed:0.0} km/h");
            DrawRow(rect, ref cursorY, "Speed (wheel)", $"{car.carSpeed:0.0} km/h");
            DrawRow(rect, ref cursorY, "Forward v/a", $"{car.DebugForwardSpeedMS:0.00} m/s   {car.DebugForwardAccelMS2:0.00} m/s²");
            DrawRow(rect, ref cursorY, "Steer angle", $"{car.DebugFrontSteerAngle:0.0}° / {car.maxSteeringAngle}°");
            DrawRow(rect, ref cursorY, "TractionCtrl", $"slip={car.DebugSlipKmh:0.0} km/h  torqueScale={car.DebugTorqueScale:0.00}");

            if (car.useExternalInput)
            {
                DrawRow(rect, ref cursorY, "Cmd steer / accel", $"{car.externalSteering:0.00} / {car.externalAcceleration:0.00}");
                DrawRow(rect, ref cursorY, "Neg accel means", car.externalNegativeMeansBrake ? "Brake" : "Reverse");
            }
            else
            {
                Vector2 mv = car.DebugMoveInput;
                DrawRow(rect, ref cursorY, "Cmd steer / thr", $"{mv.x:0.00} / {mv.y:0.00}");
                DrawRow(rect, ref cursorY, "Handbrake", car.DebugHandbrakePressed ? "ON" : "off");
            }

            DrawRow(rect, ref cursorY, "Applied axes", $"steer={car.DebugSteeringAxis:0.00}  thr={car.DebugThrottleAxis:0.00}");

            if (car.DebugUseHorsepowerModel)
            {
                cursorY += 6f;
                GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight), "<b>Powertrain</b>", headerStyle);
                cursorY += rowHeight + 4f;
                DrawRow(rect, ref cursorY, "Power", $"max={car.DebugLastAvailablePowerW/1000f:0.0}kW  req={car.DebugLastRequestedPowerW/1000f:0.0}kW");
                DrawRow(rect, ref cursorY, "Pedal", $"{car.DebugLastPedal01:0.00} {(car.DebugLastEnergyLimited ? "(ENERGY LIMIT)" : "")}");
                DrawRow(rect, ref cursorY, "Energy", $"{car.DebugCurrentEnergyKWh:0.00}/{car.DebugMaxEnergyKWh:0.00} kWh");
                DrawRow(rect, ref cursorY, "Force / cap", $"{car.DebugLastTractiveForceN:0} N  cap={car.DebugLastAccelCapMS2:0.0} m/s²");
            }
        }
        else
        {
            GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight), "No car found", valueStyle);
            cursorY += rowHeight;
        }

        if (pursuit != null)
        {
            cursorY += 6f;
            GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight), "<b>Pure Pursuit</b>", headerStyle);
            cursorY += rowHeight + 4f;
            DrawRow(rect, ref cursorY, "Autonomous", pursuit.isAutonomous ? "ON" : "off");
            DrawRow(rect, ref cursorY, "Target / desired", $"{pursuit.targetSpeed:0.0} / {pursuit.debugDesiredSpeedKmh:0.0} km/h");
            DrawRow(rect, ref cursorY, "LookAhead", $"{pursuit.lookAheadDistance:0.0}");
            DrawRow(rect, ref cursorY, "Speed PID", $"{(pursuit.useSpeedPid ? "ON" : "off")}  Kp={pursuit.speedKp:0.000} Ki={pursuit.speedKi:0.000} Kd={pursuit.speedKd:0.000}");
        }
    }

    void DrawInputsPanel(Rect rect)
    {
        float cursorY = rect.y;
        GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight + 2f), "<b>Inputs</b>", headerStyle);
        cursorY += rowHeight + 10f;

        if (car == null)
        {
            GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight), "No car", valueStyle);
            return;
        }

        float steerVal;
        float throttleVal;
        string src;

        if (car.useExternalInput)
        {
            src = "External";
            steerVal = Mathf.Clamp(car.externalSteering, -1f, 1f);
            throttleVal = Mathf.Clamp(car.externalAcceleration, -1f, 1f);
        }
        else
        {
            src = "Player";
            steerVal = Mathf.Clamp(car.DebugMoveInput.x, -1f, 1f);
            throttleVal = Mathf.Clamp(car.DebugMoveInput.y, -1f, 1f);
        }

        GUI.Label(new Rect(rect.x, cursorY, rect.width, rowHeight), $"Source: {src}", smallStyle);
        cursorY += rowHeight + 8f;

        DrawSignedBar(new Rect(rect.x, cursorY, barWidth, barHeight), steerVal, "Steering");
        cursorY += barHeight + 18f;
        DrawSignedBar(new Rect(rect.x, cursorY, barWidth, barHeight), throttleVal, "Throttle / Brake");
    }

    void DrawRow(Rect rect, ref float y, string label, string value)
    {
        Rect labelRect = new Rect(rect.x, y, Mathf.Min(labelWidth, rect.width), rowHeight);
        Rect valueRect = new Rect(rect.x + Mathf.Min(labelWidth, rect.width) + 8f, y, rect.width - Mathf.Min(labelWidth, rect.width) - 8f, rowHeight);
        GUI.Label(labelRect, label, labelStyle);
        GUI.Label(valueRect, value, valueStyle);
        y += rowHeight;
    }

    static void DrawRect(Rect rect, Color color)
    {
        Color prev = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = prev;
    }

    void DrawSignedBar(Rect rect, float value, string label)
    {
        // Background
        DrawRect(rect, barBack);

        // Zero line
        float zeroX = rect.x + rect.width * 0.5f;
        DrawRect(new Rect(zeroX - 1f, rect.y, 2f, rect.height), new Color(1f, 1f, 1f, 0.20f));

        float v = Mathf.Clamp(value, -1f, 1f);
        float half = rect.width * 0.5f;
        float w = Mathf.Abs(v) * half;

        Rect fill;
        Color col = v >= 0f ? barFillPos : barFillNeg;
        if (v >= 0f)
            fill = new Rect(zeroX, rect.y, w, rect.height);
        else
            fill = new Rect(zeroX - w, rect.y, w, rect.height);

        DrawRect(fill, col);

        // Label
        GUI.Label(new Rect(rect.x + rect.width + 10f, rect.y - 3f, 240f, rect.height + 6f), $"{label}: {v:0.00}", valueStyle);
    }
}

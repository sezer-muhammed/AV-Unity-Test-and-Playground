using UnityEngine;
using UnityEngine.UI;

public class TelemetryUI : MonoBehaviour
{
    public PrometeoCarController car;
    public Text telemetryText;
    
    // In Unity 6, standard Text might be TMP_Text depending on setup, 
    // but using legacy Text for compatibility with existing Prometeo setup.

    void Update()
    {
        if (car == null || telemetryText == null) return;

        Rigidbody rb = car.GetComponent<Rigidbody>();
        float speed = car.carSpeed;
        float accel = rb.linearVelocity.magnitude / Time.fixedDeltaTime; // Approximate
        
        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine($"Speed: {speed:F1} km/h");
        sb.AppendLine($"Steering: {car.externalSteering:F2}");
        sb.AppendLine($"Autonomous: {car.useExternalInput}");
        
        telemetryText.text = sb.ToString();
    }
}
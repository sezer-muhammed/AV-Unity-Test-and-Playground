using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform carTransform;

    [Header("Tracking Sliders")]
    [Range(2, 10)]
    public float distance = 6f;
    [Range(-90, 90)]
    public float verticalAngle = 20f; // Up / Down
    [Range(-180, 180)]
    public float orbitAngle = 0f;    // Around the car

    [Header("Refinement")]
    public float followSpeed = 10f;
    public float lookSpeed = 10f;
    public Vector3 targetOffset = new Vector3(0, 1.5f, 0);

    void LateUpdate()
    {
        if (carTransform == null) return;

        // 1. Calculate the rotation for the camera offset
        // This combines the car's current heading with the user's custom orbit settings
        Quaternion rotation = carTransform.rotation * Quaternion.Euler(verticalAngle, orbitAngle, 0);

        // 2. Define the target point we are orbiting (the car's center + offset)
        Vector3 worldTargetOffset = carTransform.rotation * targetOffset;
        Vector3 targetPoint = carTransform.position + worldTargetOffset;

        // 3. Calculate target position
        // We move backwards from the target point based on the rotation and distance
        Vector3 targetPos = targetPoint + (rotation * Vector3.back * distance);

        // 4. Smoothly interpolate position
        transform.position = Vector3.Lerp(transform.position, targetPos, followSpeed * Time.deltaTime);

        // 5. Rotate to look at the target point
        Vector3 lookDirection = targetPoint - transform.position;
        if (lookDirection != Vector3.zero)
        {
            Quaternion targetRot = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, lookSpeed * Time.deltaTime);
        }
    }
}

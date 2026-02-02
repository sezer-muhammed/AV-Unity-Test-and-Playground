using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour {

	public Transform carTransform;
	[Range(1, 20)]
	public float followSpeed = 10;
	[Range(1, 20)]
	public float lookSpeed = 10;
    
    [Header("Camera Configuration")]
    public float distance = 10.0f;
    public float angle = 20.0f;
    public Vector3 vehicleOffset = new Vector3(0, 1.5f, 0);

	void LateUpdate()
	{
        if (carTransform == null) return;

        // Calculate the relative target position based on distance and angle
        // Angle is the vertical tilt, distance is how far back the camera stays
        float radAngle = angle * Mathf.Deg2Rad;
        Vector3 localOffset = new Vector3(0, Mathf.Sin(radAngle) * distance, -Mathf.Cos(radAngle) * distance);
        
        // Move to the position relative to the car's orientation
        Vector3 targetPos = carTransform.position + carTransform.rotation * localOffset;
		transform.position = Vector3.Lerp(transform.position, targetPos, followSpeed * Time.deltaTime);

		// Calculate look direction including the vehicle offset
        Vector3 targetLookAt = carTransform.position + carTransform.rotation * vehicleOffset;
		Vector3 lookDirection = targetLookAt - transform.position;
		
        if (lookDirection != Vector3.zero)
        {
             Quaternion targetRot = Quaternion.LookRotation(lookDirection, Vector3.up);
		    transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, lookSpeed * Time.deltaTime);
        }
	}

}

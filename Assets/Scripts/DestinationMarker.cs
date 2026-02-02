using UnityEngine;

[ExecuteInEditMode]
public class DestinationMarker : MonoBehaviour
{
    public Color color = Color.yellow;
    public float scale = 2.0f;
    private LineRenderer lineRenderer;

    void OnEnable()
    {
        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.startWidth = 0.2f;
            lineRenderer.endWidth = 0.2f;
            lineRenderer.positionCount = 5;
            lineRenderer.useWorldSpace = false;
            
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            if (shader != null) lineRenderer.material = new Material(shader);
        }
    }

    void Update()
    {
        if (lineRenderer != null)
        {
            lineRenderer.startColor = color;
            lineRenderer.endColor = color;

            // Draw an arrow in local space
            Vector3[] points = new Vector3[5];
            points[0] = Vector3.zero;
            points[1] = Vector3.forward * scale;
            points[2] = Vector3.forward * (scale * 0.8f) + Vector3.right * (scale * 0.2f);
            points[3] = Vector3.forward * scale;
            points[4] = Vector3.forward * (scale * 0.8f) - Vector3.right * (scale * 0.2f);
            
            lineRenderer.SetPositions(points);
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = color;
        Gizmos.DrawWireSphere(transform.position, 0.3f);
    }
}

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LaserScanVisualizer : MonoBehaviour
{
    // TextMesh to display obstacle proximity warnings
    public TextMesh obstacleWarningText;

    // ROS topic for laser scan data (modify via Inspector)
    [SerializeField] private string laserScanTopic = "/scan";

    // Color and brightness for rendered lines
    [SerializeField] private Color lineColor = Color.green;
    [SerializeField, Range(0, 8)] private float brightness = 1.0f;

    // Maximum distance between points to consider continuous
    [SerializeField] private float maxPointDistance = 0.5f;

    // Threshold for displaying obstacle warnings (meters)
    [SerializeField] private float obstacleWarningDistance = 0.3f;

    private ROSConnection ros;
    private List<GameObject> lineSegments = new List<GameObject>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(laserScanTopic, OnLaserScanReceived);
    }

    void OnLaserScanReceived(LaserScanMsg scanMsg)
    {
        ClearPreviousVisualization();
        VisualizeLaserScan(scanMsg);
    }

    void VisualizeLaserScan(LaserScanMsg scanMsg)
    {
        float angle = scanMsg.angle_min;
        Vector3 previousPoint = Vector3.zero;
        bool hasPreviousPoint = false;

        for (int i = 0; i < scanMsg.ranges.Length; i++)
        {
            float distance = scanMsg.ranges[i];

            // Check proximity and display warning if necessary
            obstacleWarningText.text = distance <= obstacleWarningDistance
                ? "Too close to an obstacle"
                : "";

            if (IsValidDistance(distance))
            {
                // Convert polar coordinates to Cartesian (Unity's coordinate frame)
                float x = distance * Mathf.Cos(angle);
                float y = distance * Mathf.Sin(angle);
                Vector3 currentPoint = new Vector3(-2.3f * y, 0, 2.3f * x);

                // Draw line segment if points are close enough
                if (!hasPreviousPoint || IsPointClose(previousPoint, currentPoint))
                {
                    DrawLineSegment(previousPoint, currentPoint);
                    previousPoint = currentPoint;
                    hasPreviousPoint = true;
                }
                else
                {
                    // Discontinuity detected, reset point tracking
                    previousPoint = currentPoint;
                    hasPreviousPoint = false;
                }
            }

            angle += scanMsg.angle_increment;
        }
    }

    void DrawLineSegment(Vector3 start, Vector3 end)
    {
        if (start == Vector3.zero)
            return; // Skip drawing from the zero vector (initialization)

        LineRenderer lineRenderer = new GameObject("LaserSegment").AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Standard"));
        lineRenderer.material.color = AdjustBrightness(lineColor, brightness);
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);

        lineSegments.Add(lineRenderer.gameObject);
    }

    bool IsValidDistance(float distance)
    {
        return distance >= 0.1f && distance <= 3.0f && !float.IsInfinity(distance) && !float.IsNaN(distance);
    }

    bool IsPointClose(Vector3 previous, Vector3 current)
    {
        return Vector3.Distance(previous, current) <= maxPointDistance;
    }

    Color AdjustBrightness(Color color, float brightnessFactor)
    {
        return new Color(
            color.r * brightnessFactor,
            color.g * brightnessFactor,
            color.b * brightnessFactor,
            color.a
        );
    }

    void ClearPreviousVisualization()
    {
        foreach (GameObject segment in lineSegments)
        {
            Destroy(segment);
        }
        lineSegments.Clear();
    }
}

// Usage in Unity Editor:
// Attach this script directly to the virtual robot GameObject.
// Assign two separate TextMesh components to the 'headingText' and 'positionText' fields 
// via the Inspector to visualize the robot’s real-time heading and position information.


using UnityEngine;

public class RobotInfoDisplay : MonoBehaviour
{
    // Text objects to display robot's heading and position info
    public TextMesh headingText;
    public TextMesh positionText;

    void Update()
    {
        // Update heading text with the object's Y-axis rotation (heading)
        headingText.text = "Heading: " + transform.localEulerAngles.y.ToString("F2") + "°";

        // Update position text with the object's X and Z positions
        positionText.text = "Position X: " + transform.position.x.ToString("F2") +
                            " | Position Z: " + transform.position.z.ToString("F2");
    }
}


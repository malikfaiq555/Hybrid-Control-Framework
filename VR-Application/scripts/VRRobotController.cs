using UnityEngine;

public class VRRobotController : MonoBehaviour
{
    // GameObject representing the virtual robot
    public GameObject virtualRobot;

    // Movement parameters
    [SerializeField] private float rotationSpeed = 0.35f;
    [SerializeField] private float moveSpeed = 0.01f;

    void Update()
    {
        // Get joystick inputs (e.g., Oculus secondary thumbstick)
        float rotationInput = Input.GetAxisRaw("Oculus_CrossPlatform_SecondaryThumbstickHorizontal");
        float movementInput = Input.GetAxisRaw("Oculus_CrossPlatform_SecondaryThumbstickVertical");

        // Rotate virtual robot based on horizontal input
        virtualRobot.transform.Rotate(0, rotationInput * rotationSpeed, 0);

        // Move virtual robot forward/backward based on vertical input
        virtualRobot.transform.Translate(movementInput * moveSpeed, 0, 0);
    }
}

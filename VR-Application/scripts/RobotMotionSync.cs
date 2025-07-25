using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using OdometryMsg = RosMessageTypes.Nav.OdometryMsg;
using TwistMsg = RosMessageTypes.Geometry.TwistMsg;
using RosMessageTypes.UnityRoboticsDemo;




public class RobotMotionSync : MonoBehaviour
{
    // TextMesh components to display real robot state information
    public TextMesh destinationStatusText;
    public TextMesh linearSpeedText;
    public TextMesh angularSpeedText;
    public TextMesh distanceText;
    public TextMesh headingText;
    public TextMesh positionText;

    // Reference to the virtual robot GameObject in Unity
    public GameObject virtualRobot;

    // ROS topic names (can be modified via Inspector)
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string odometryTopic = "/odom";
    [SerializeField] private string goalPositionTopic = "goal_position";

    // ROS connection instance
    private ROSConnection ros;

    // Internal state variables
    private bool receivedVelocity;
    private bool receivedOdom;

    private Vector3 goalPosition = new Vector3(2, 0, 0); // Predefined goal position (2 meters ahead)

    private float distanceToGoal;

    // Received data holders
    private double linearVelocity;
    private double angularVelocity;
    private Vector3 odomPosition;
    private Quaternion odomOrientation;

    private bool shouldPublish = true;

    void Start()
    {
        // Initialize ROS connection and subscriptions
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
        ros.Subscribe<OdometryMsg>(odometryTopic, OdometryCallback);

        // Register the publisher for the goal position (point-based custom message)
        // NOTE: Users should create a suitable ROS message type (e.g., PointMsg) as similar to the Unity-Robotics-Hub demo messages.
        ros.RegisterPublisher<PosRotMsg>(goalPositionTopic);
    }

    void Update()
    {
        // Publish the goal position when user presses a VR headset/controller button (e.g., OVRInput.Button.One)
        if (OVRInput.Get(OVRInput.Button.One))
        {
            // Currently sending static coordinates (0,0,6); adjust as needed.
            PosRotMsg goalMsg = new PosRotMsg(goalPosition.x, goalPosition.y, goalPosition.z, 0, 0, 0, 0);
            ros.Publish(goalPositionTopic, goalMsg);
        }

        // Process incoming ROS data
        if (receivedVelocity)
            ProcessVelocityData();

        if (receivedOdom)
            ProcessOdometryData();
    }

    void CmdVelCallback(TwistMsg velocityMsg)
    {
        linearVelocity = velocityMsg.linear.x;
        angularVelocity = velocityMsg.angular.z;
        receivedVelocity = true;
    }

    void OdometryCallback(OdometryMsg odometryMsg)
    {
        odomPosition.x = (float)odometryMsg.pose.pose.position.x;
        odomPosition.y = 0; // Assuming flat terrain
        odomPosition.z = (float)odometryMsg.pose.pose.position.y; // Adjust if necessary for your coordinate frame

        odomOrientation = new Quaternion(
            (float)odometryMsg.pose.pose.orientation.x,
            (float)odometryMsg.pose.pose.orientation.y,
            (float)odometryMsg.pose.pose.orientation.z,
            (float)odometryMsg.pose.pose.orientation.w
        );

        receivedOdom = true;
    }

    void ProcessVelocityData()
    {
        if (shouldPublish)
        {
            // Rotate and translate the virtual robot to mimic actual robot velocity adjust accordinf to your coordinate frame
            virtualRobot.transform.Rotate(0, -(float)angularVelocity, 0);
            virtualRobot.transform.Translate((float)linearVelocity * 0.05f, 0, 0);
        }

        // Update UI text displaying velocities
        linearSpeedText.text = "Linear Velocity: " + linearVelocity.ToString("F2") + " m/s";
        angularSpeedText.text = "Angular Velocity: " + angularVelocity.ToString("F2") + " rad/s";

        receivedVelocity = false;
    }

    void ProcessOdometryData()
    {
        // Calculate heading from quaternion orientation
        float headingAngle = odomOrientation.eulerAngles.z;

        // Normalize heading angle around 0-360 boundary
        if (headingAngle > 359f || headingAngle < 1f)
            headingAngle = 0f;

        headingText.text = "Heading: " + headingAngle.ToString("F2") + "°";

        positionText.text = $"Position X: {odomPosition.x:F2} | Position Z: {odomPosition.z:F2}";

        // Compute remaining distance to predefined goal position
        distanceToGoal = Vector3.Distance(goalPosition, odomPosition);
        distanceText.text = "Remaining Distance: " + distanceToGoal.ToString("F2") + " m";

        // Update status if goal reached (within threshold)
        if (distanceToGoal <= 0.3f)
        {
            destinationStatusText.text = "Goal Reached";
            shouldPublish = false; // Stop moving virtual robot
        }
        else
        {
            destinationStatusText.text = "";
        }

        receivedOdom = false;
    }
}

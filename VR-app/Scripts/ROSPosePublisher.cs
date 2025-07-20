using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class ROSPosePublisher : MonoBehaviour
{
    // ROS connection and topic details
    [SerializeField] private string rosTopicName = "vr_robot_pose";

    // Reference to the GameObject (VR Robot) whose pose is published
    public GameObject targetObject;

    // Publishing frequency (in seconds)
    [SerializeField] private float publishFrequency = 0.5f;

    private ROSConnection ros;
    private float timeElapsed;

    void Start()
    {
        // Initialize ROS connection and register publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(rosTopicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            // Example movement logic (moving the object backward over time)
            targetObject.transform.position += new Vector3(0, 0, -1 * Time.deltaTime);

            // Prepare ROS message with position and rotation data
            PosRotMsg poseMsg = new PosRotMsg(
                targetObject.transform.position.x,
                targetObject.transform.position.y,
                targetObject.transform.position.z,
                targetObject.transform.rotation.x,
                targetObject.transform.rotation.y,
                targetObject.transform.rotation.z,
                targetObject.transform.rotation.w
            );

            // Publish the pose message to the ROS network
            ros.Publish(rosTopicName, poseMsg);

            timeElapsed = 0;
        }
    }
}

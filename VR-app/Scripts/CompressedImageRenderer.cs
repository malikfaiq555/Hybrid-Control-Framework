using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using CompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;

/// Subscribes to a ROS topic streaming compressed image data.
/// Renders the received image onto a specified Unity GameObject.

public class CompressedImageRenderer : MonoBehaviour
{
    // ROS Connection Instance
    public ROSConnection rosConnection;

    // GameObject to render received images (e.g., sphere, dome, plane)
    public GameObject renderTarget;

    // Name of ROS topic publishing compressed images (adjustable in Inspector)
    [SerializeField]
    private string imageTopic = "/camera/image/compressed";

    // Internal variables for texture handling
    private Texture2D receivedTexture;
    private byte[] receivedImageData;
    private bool imageReceived;

    void Start()
    {
        // Subscribe to the ROS topic to receive compressed image messages
        rosConnection.Subscribe<CompressedImageMsg>(imageTopic, OnImageReceived);

        // Initialize texture with a placeholder size
        receivedTexture = new Texture2D(1, 1);

        // Assign a new standard material to render target to display texture
        renderTarget.GetComponent<Renderer>().material = new Material(Shader.Find("Standard"));
    }

    void Update()
    {
        // Process and display the image only when a new message is received
        if (imageReceived)
        {
            UpdateRenderedTexture();
        }
    }

    // Callback function to handle incoming ROS image messages
    private void OnImageReceived(CompressedImageMsg imageMessage)
    {
        receivedImageData = imageMessage.data;
        imageReceived = true;
        // Optionally log image format for debugging:
        // Debug.Log(imageMessage.format);
    }

    // Updates the render target's texture with the received image data
    private void UpdateRenderedTexture()
    {
        receivedTexture.LoadImage(receivedImageData);
        receivedTexture.Apply();

        renderTarget.GetComponent<Renderer>().material.SetTexture("_MainTex", receivedTexture);

        // Reset the received data state
        receivedImageData = null;
        imageReceived = false;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImagePublisher : MonoBehaviour
{
    public Camera ImageCamera;
    public string FrameId = "Camera";
    public string RosTopic = "/image";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;

    private string RosImageEncoding = "rgb8";

    private Unity.Robotics.ROSTCPConnector.ROSConnection mRos;

    //private MessageTypes.Sensor.CompressedImage message;
    private Texture2D texture2D;
    private Rect rect;
    private const int isBigEndian = 1;
    private const int step = 3;

    WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();

    //************************************************************************
    /// Description: Flip image horozontall, array in place
    /// Arg original: incoming texture
    /// Returns: void
    //*************************************************************************
    public static void FlipTextureVerticallyInplace(Texture2D original)
    {
        var originalPixels = original.GetPixels();

        var newPixels = new Color[originalPixels.Length];

        var width = original.width;
        var rows = original.height;

        for (var x = 0; x < width; x++)
        {
            for (var y = 0; y < rows; y++)
            {
                newPixels[x + y * width] = originalPixels[x + (rows - y -1) * width];
            }
        }

        original.SetPixels(newPixels);
        original.Apply();
    }

    //************************************************************************
    /// Description: Flip image horozontally
    /// Arg original: incoming texture
    /// Returns: Flipped texture
    //*************************************************************************
   public static Texture2D FlipTextureVertically(Texture2D original)
    {
        Texture2D flipped = new Texture2D(
            original.width, original.height, original.format, false);

        int xN = original.width;
        int yN = original.height;

        for (int i = 0; i < xN; i++)
        {
            for (int j = 0; j < yN; j++)
            {
                flipped.SetPixel(i, yN - j - 1, original.GetPixel(i, j));
            }
        }

        flipped.Apply();

        return flipped;
    }

    //************************************************************************
    /// Description: Called by Unity once at start up
    /// Returns: void
    //*************************************************************************
    void Start()
    {
        //base.Start();
        InitializeGameObject();
        InitializeMessage();
        Camera.onPostRender += UpdateImage;
    }

    //************************************************************************
    /// Description: Update image if appropriate
    /// Arg _camera:
    /// Returns: void
    //*************************************************************************
    private void UpdateImage(Camera _camera)
    {
        if (texture2D != null && _camera == this.ImageCamera)
            StartCoroutine(UpdateMessage());
    }

    //************************************************************************
    /// Description: Initialize game object, call this once at start up
    /// Returns: void
    //*************************************************************************
    private void InitializeGameObject()
    {
        ImageCamera.enabled = true;
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
    }

    //************************************************************************
    /// Description: Initialize ROS message, call this once at start up
    /// Returns: void
    //*************************************************************************
    private void InitializeMessage()
    {
        mRos = Unity.Robotics.ROSTCPConnector.ROSConnection.instance;
        mRos.RegisterPublisher<RosMessageTypes.Sensor.ImageMsg>(RosTopic);
    }

    //************************************************************************
    /// Description: Update message, send to ROS
    /// Returns: void
    //*************************************************************************
    public IEnumerator UpdateMessage()
    {
        yield return frameEnd;

        texture2D.ReadPixels(rect, 0, 0);

        //FlipTextureVertically(texture2D);
        //var imageData = texture2D.GetRawTextureData();
        var imageData = FlipTextureVertically(texture2D).GetRawTextureData();

        var timestamp = new Unity.Robotics.Core.TimeStamp(Unity.Robotics.Core.Clock.time);
        var header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = FrameId,
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }
            };        
        
        RosMessageTypes.Sensor.ImageMsg rosImage = 
            new RosMessageTypes.Sensor.ImageMsg(header, 
                (uint)resolutionHeight, (uint)resolutionWidth, RosImageEncoding, 
                isBigEndian, (uint)(step * resolutionWidth), imageData);

        mRos.Send(RosTopic, rosImage);
    }

    //************************************************************************
    /// Description: Called by Unity once per frame
    /// Returns: void
    //*************************************************************************
    void Update()
    {
        UpdateImage(ImageCamera);
    }
}

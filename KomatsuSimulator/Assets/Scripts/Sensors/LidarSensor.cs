//*********************************************************
//* Sources : https://github.com/SrinivasRavi/lidar_sim
//*********************************************************

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LidarSensor : MonoBehaviour
{
    int CurrentRayCount;
    int CurrentMeasurementsPerRotation;
    float CurrentFieldOfView;
    float CurrentCenterAngle;
    float CurrentMinDistance;
    float CurrentMaxDistance;

    const float HorizontalAngleLimit = 15.0f;

    public int RayCount = 32; //TODO: Test with other Lidar sensor configurations
    public float MinDistance = 0.5f; // meters
    public float MaxDistance = 100.0f; // meters
    public float RotationFrequency = 5.0f; // Hz
    public int MeasurementsPerRotation = 1500; // for each ray// minmimum is 360/HorizontalAngleLimit
    public float FieldOfView = 40.0f;
    public float CenterAngle = 10.0f;

    public Shader Shader = null;
    public Camera Camera = null;

    public string FrameId = "lidar";
    public string RosTopic = "/points_raw";
    //static Comm.Writer<Ros.PointCloud2> WriterPointCloud2;
    uint Sequence;
    Vector4[] PointCloud;
    byte[] RosPointCloud;

    private Unity.Robotics.ROSTCPConnector.ROSConnection mRos;

    struct ReadRequest
    {
        public AsyncTextureReader<Vector2> Reader;
        public int Count;
        public int MaxRayCount;
        public int StartRay;

        public Vector3 Origin;
        public Vector3 Start;
        public Vector3 DeltaX;
        public Vector3 DeltaY;

        public Matrix4x4 Transform;

        public float AngleEnd;
    }

    List<ReadRequest> Active = new List<ReadRequest>();
    Stack<AsyncTextureReader<Vector2>> Available = new Stack<AsyncTextureReader<Vector2>>();

    int CurrentIndex;
    float AngleStart;
    float AngleDelta;

    float AngleTopPart;

    float DebugStartAngle;
    int DebugCount;

    float MaxAngle;
    int RenderTextureWidth;
    int RenderTextureHeight;

    //************************************************************************
    /// Description: Initialize ROS message, call this once at start up
    /// Returns: void
    //*************************************************************************

    private void InitializeMessage()
    {
        mRos = Unity.Robotics.ROSTCPConnector.ROSConnection.GetOrCreateInstance();
        mRos.RegisterPublisher<RosMessageTypes.Sensor.PointCloud2Msg>(RosTopic);
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void Start()
    {
        InitializeMessage();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void Awake()
    {
        enabled = true;

        Camera.enabled = false;
        Camera.renderingPath = RenderingPath.Forward;
        Camera.clearFlags = CameraClearFlags.Color;
        Camera.backgroundColor = Color.black;
        Camera.allowMSAA = false;
        Camera.allowHDR = false;

        Reset();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    public void Reset()
    {
        Active.ForEach(req =>
        {
            req.Reader.Destroy();
            req.Reader.Texture.Release();
        });
        Active.Clear();

        foreach (var rt in Available)
        {
            rt.Destroy();
            rt.Texture.Release();
        };
        Available.Clear();

        AngleStart = 0.0f;
        MaxAngle = Mathf.Abs(CenterAngle) + FieldOfView / 2.0f;
        RenderTextureHeight = 2 * (int)(2.0f * MaxAngle * RayCount / FieldOfView);
        RenderTextureWidth = 2 * (int)(HorizontalAngleLimit / (360.0f / MeasurementsPerRotation));

        // construct custom aspect ratio projection matrix
        // math from https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix

        float v = 1.0f / Mathf.Tan(MaxAngle * Mathf.Deg2Rad);
        float h = 1.0f / Mathf.Tan(HorizontalAngleLimit * Mathf.Deg2Rad / 2.0f);
        float a = (MaxDistance + MinDistance) / (MinDistance - MaxDistance);
        float b = 2.0f * MaxDistance * MinDistance / (MinDistance - MaxDistance);

        var projection = new Matrix4x4(
            new Vector4(h, 0, 0, 0),
            new Vector4(0, v, 0, 0),
            new Vector4(0, 0, a, -1),
            new Vector4(0, 0, b, 0));
        Camera.projectionMatrix = projection;

        int count = RayCount * MeasurementsPerRotation;
        PointCloud = new Vector4[count];

        RosPointCloud = new byte[32 * count];

        CurrentRayCount = RayCount;
        CurrentMeasurementsPerRotation = MeasurementsPerRotation;
        CurrentFieldOfView = FieldOfView;
        CurrentCenterAngle = CenterAngle;
        CurrentMinDistance = MinDistance;
        CurrentMaxDistance = MaxDistance;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void Update()
    {

        if (RayCount != CurrentRayCount ||
        MeasurementsPerRotation != CurrentMeasurementsPerRotation ||
        FieldOfView != CurrentFieldOfView ||
        CenterAngle != CurrentCenterAngle ||
        MinDistance != CurrentMinDistance ||
        MaxDistance != CurrentMaxDistance)
        {
            if (MinDistance > 0 && MaxDistance > 0 && RayCount > 0 && MeasurementsPerRotation >= (360.0f / HorizontalAngleLimit))
            {
                Reset();
            }
        }

        bool pointCloudUpdated = false;

        for (int i = 0; i < Active.Count; i++)
        {
            var req = Active[i];
            req.Reader.Update();
            if (req.Reader.Status == AsyncTextureReaderStatus.Finished)
            {
                pointCloudUpdated = true;
                ReadLasers(req);
                Available.Push(req.Reader);
                Active.RemoveAt(i);
                i--;
            }
            else if (req.Reader.Status == AsyncTextureReaderStatus.Idle)
            {
                // reader was reset, probably due to loosing RenderTexture
                Available.Push(req.Reader);
                Active.RemoveAt(i);
                i--;
            }
            else
            {
                for (int j = i + 1; j < Active.Count; j++)
                {
                    Active[j].Reader.Update();
                }
                break;
            }
        }

        float minAngle = 360.0f / CurrentMeasurementsPerRotation;

        AngleDelta += Time.deltaTime * 360.0f * RotationFrequency;
        int count = (int)(HorizontalAngleLimit / minAngle);

        while (AngleDelta >= HorizontalAngleLimit)
        {
            float angle = AngleStart + HorizontalAngleLimit / 2.0f;
            var rotation = Quaternion.AngleAxis(angle, Vector3.up);
            Camera.transform.localRotation = rotation;

            if (count != 0)
            {
                pointCloudUpdated |= RenderLasers(count, AngleStart, HorizontalAngleLimit);
            }

            AngleDelta -= HorizontalAngleLimit;
            AngleStart += HorizontalAngleLimit;

            if (AngleStart >= 360.0f)
            {
                AngleStart -= 360.0f;
            }
        }
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void OnDestroy()
    {
        Active.ForEach(req =>
        {
            req.Reader.Destroy();
            req.Reader.Texture.Release();
        });
        Active.Clear();

        foreach (var rt in Available)
        {
            rt.Destroy();
            rt.Texture.Release();
        };
        Available.Clear();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    bool RenderLasers(int count, float angleStart, float angleUse)
    {
        bool pointCloudUpdated = false;


        AsyncTextureReader<Vector2> reader = null;
        if (Available.Count == 0)
        {
            var texture = new RenderTexture(RenderTextureWidth, RenderTextureHeight, 24, RenderTextureFormat.RGFloat, RenderTextureReadWrite.Linear);
            reader = new AsyncTextureReader<Vector2>(texture);
        }
        else
        {
            reader = Available.Pop();
        }

        Camera.targetTexture = reader.Texture;
        Camera.RenderWithShader(Shader, "RenderType");
        reader.Start();

        var pos = Camera.transform.position;

        var topLeft = Camera.ViewportPointToRay(new Vector3(0, 0, 1)).direction;
        var topRight = Camera.ViewportPointToRay(new Vector3(1, 0, 1)).direction;
        var bottomLeft = Camera.ViewportPointToRay(new Vector3(0, 1, 1)).direction;
        var bottomRight = Camera.ViewportPointToRay(new Vector3(1, 1, 1)).direction;

        int maxRayCount = (int)(2.0f * MaxAngle * RayCount / FieldOfView);
        var deltaX = (topRight - topLeft) / count;
        var deltaY = (bottomLeft - topLeft) / maxRayCount;

        int startRay = 0;
        var start = topLeft;
        if (CenterAngle < 0.0f)
        {
            startRay = maxRayCount - RayCount;
        }

#if VISUALIZE_LIDAR_CAMERA_BOUNDING_BOX
        var a = start + deltaY * startRay;
        var b = a + deltaX * count;

        Debug.DrawLine(pos, pos + MaxDistance * a, Color.yellow, 1.0f, true);
        Debug.DrawLine(pos, pos + MaxDistance * b, Color.yellow, 1.0f, true);
        Debug.DrawLine(pos + MaxDistance * a, pos + MaxDistance * b, Color.yellow, 1.0f, true);

        a = start + deltaY * (startRay + RayCount);
        b = a + deltaX * count;

        Debug.DrawLine(pos, pos + MaxDistance * a, Color.magenta, 1.0f, true);
        Debug.DrawLine(pos, pos + MaxDistance * b, Color.magenta, 1.0f, true);
        Debug.DrawLine(pos + MaxDistance * a, pos + MaxDistance * b, Color.magenta, 1.0f, true);
#endif

        var req = new ReadRequest()
        {
            Reader = reader,
            Count = count,
            MaxRayCount = maxRayCount,
            StartRay = startRay,
            Origin = pos,
            Start = start,
            DeltaX = deltaX,
            DeltaY = deltaY,
            AngleEnd = angleStart + angleUse,
        };

        req.Reader.Update();
        if (req.Reader.Status == AsyncTextureReaderStatus.Finished)
        {
            pointCloudUpdated = true;
            ReadLasers(req);
            Available.Push(req.Reader);
        }
        else
        {
            Active.Add(req);
        }

        return pointCloudUpdated;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void ReadLasers(ReadRequest req)
    {
        var data = req.Reader.GetData();

        var startRay = req.StartRay;
        var maxRayCount = req.MaxRayCount;

        var startDir = req.Start + startRay * req.DeltaY;
        var origin = req.Origin;

        for (int j = 0; j < CurrentRayCount; j++)
        {
            var dir = startDir;
            int y = (j + startRay) * RenderTextureHeight / maxRayCount;
            int yOffset = y * RenderTextureWidth;
            int indexOffset = j * CurrentMeasurementsPerRotation;

            for (int i = 0; i < req.Count; i++)
            {
                int x = i * RenderTextureWidth / req.Count;

                var di = data[yOffset + x];
                float distance = di.x;
                float intensity = di.y;

                var position = origin + dir.normalized * distance;

                int index = indexOffset + (CurrentIndex + i) % CurrentMeasurementsPerRotation;

                PointCloud[index] = distance == 0 ? Vector4.zero : new Vector4(position.x, position.y, position.z, intensity);

                dir += req.DeltaX;
            }

            startDir += req.DeltaY;
        }

        if (CurrentIndex + req.Count >= CurrentMeasurementsPerRotation)
        {
            SendMessage(CurrentIndex + req.Count - CurrentMeasurementsPerRotation, req.AngleEnd);
        }

        CurrentIndex = (CurrentIndex + req.Count) % CurrentMeasurementsPerRotation;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************

    void SendMessage(int currentEndIndex, float currentEndAngle)
    {
        // Lidar x is forward, y is left, z is up
        var worldToLocal = new Matrix4x4(new Vector4(0, -1, 0, 0), new Vector4(0, 0, 1, 0), new Vector4(1, 0, 0, 0), Vector4.zero);

        worldToLocal = worldToLocal * transform.worldToLocalMatrix;

        int count = 0;
        unsafe
        {
            fixed (byte* ptr = RosPointCloud)
            {
                int offset = 0;
                for (int i = 0; i < PointCloud.Length; i++)
                {
                    var point = PointCloud[i];
                    if (point == Vector4.zero)
                    {
                        continue;
                    }

                    var pos = new Vector3(point.x, point.y, point.z);
                    float intensity = point.w;

                    *(Vector3*)(ptr + offset) = worldToLocal.MultiplyPoint3x4(pos);
                    *(ptr + offset + 16) = (byte)(intensity * 255);

                    offset += 32;
                    count++; //if (i == 0) { Debug.Log("<color=blue>:</color>"); }
                }
            }
        }

        RosMessageTypes.Sensor.PointFieldMsg[] fields = new RosMessageTypes.Sensor.PointFieldMsg[]
        {
            new RosMessageTypes.Sensor.PointFieldMsg()
            {
                name = "x",
                offset = 0,
                datatype = 7,
                count = 1,
            },
            new RosMessageTypes.Sensor.PointFieldMsg()
            {
                name = "y",
                offset = 4,
                datatype = 7,
                count = 1,
            },
            new RosMessageTypes.Sensor.PointFieldMsg()
            {
                name = "z",
                offset = 8,
                datatype = 7,
                count = 1,
            },
            new RosMessageTypes.Sensor.PointFieldMsg()
            {
                name = "intensity",
                offset = 16,
                datatype = 2,
                count = 1,
            },
            new RosMessageTypes.Sensor.PointFieldMsg()
            {
                name = "timestamp",
                offset = 24,
                datatype = 8,
                count = 1,
            }
        };

        uint point_step = 32;
        uint row_step = (uint)(point_step * MeasurementsPerRotation);

        RosMessageTypes.Sensor.PointCloud2Msg rosPC2 =
        new RosMessageTypes.Sensor.PointCloud2Msg()
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            height = (uint)RayCount,
            width = (uint)MeasurementsPerRotation,
            fields = fields,
            is_bigendian = false,
            point_step = point_step,
            row_step = row_step,
            data = RosPointCloud,
            is_dense = true
        };

        mRos.Publish(RosTopic, rosPC2);
    }
}
using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;

namespace Publishers
{
    /// <summary>
    /// Publish lidar data in ROS PointCloud2 format
    /// </summary>
    [RequireComponent(typeof(Sensors.LidarSensor3D))]
    public class ROSPointCloud2Publisher : MonoBehaviour
    {
        private Unity.Robotics.ROSTCPConnector.ROSConnection mRos;

        private byte[] _PointCloudData = null;

        private RosMessageTypes.Sensor.PointFieldMsg[] _PointFieldArray =
            CreateXYZPointFieldArray();

        /// <summary> The ROS frame ID of the lidar </summary>
        public string RosFrameId = "lidar";

        /// <summary> The ROS topic name </summary>
        public string RosTopic = "/points_raw";

        /// <summary> The lidar sensor to be published </summary>
        public Sensors.LidarSensor3D LidarSensor;

        //*********************************************************************
        /// <summary>
        /// Called by the Unity engine prior to first frame
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>void</returns>
        //*********************************************************************
        void Start()
        {
            Initialize();
        }

        //*********************************************************************
        /// <summary>
        /// Initialize things
        /// </summary>
        /// <returns>void</returns>
        //*********************************************************************
        private void Initialize()
        {
            // connect
            mRos = Unity.Robotics.ROSTCPConnector.ROSConnection.GetOrCreateInstance();

            // advertise
            mRos.RegisterPublisher<RosMessageTypes.Sensor.PointCloud2Msg>(RosTopic);

            // hookup to the sensor update delegate
            LidarSensor.OnPublishDelegate = OnPublishLidarScanDelegate;
        }

        //*********************************************************************
        /// <summary>
        /// Create an XYZ point field array
        /// </summary>
        /// <returns>XYZ point field array</returns>
        //*********************************************************************
        private static RosMessageTypes.Sensor.PointFieldMsg[] CreateXYZPointFieldArray()
        {
            RosMessageTypes.Sensor.PointFieldMsg[] fields =
            new RosMessageTypes.Sensor.PointFieldMsg[]
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
            }
            };

            return fields;
        }

        //*********************************************************************
        /// <summary>
        /// Create an XYZ PointCloud
        /// </summary>
        /// <param name="pointsInUnityCoordinates">Points in Unity coordinates
        /// </param>
        /// <returns>ROS PointCloud2 message</returns>
        //*********************************************************************
        private RosMessageTypes.Sensor.PointCloud2Msg CreateXYZPointCloud(
            UnityEngine.Vector3[] pointsInUnityCoordinates)
        {
            if (null == _PointCloudData)
                _PointCloudData = new byte[sizeof(float) * 3 * pointsInUnityCoordinates.Length];

            int byteDataOffset = 0;

            foreach (UnityEngine.Vector3 v in pointsInUnityCoordinates)
            {
                System.Buffer.BlockCopy(System.BitConverter.GetBytes(v.z), 0,
                    _PointCloudData, byteDataOffset + (0 * sizeof(float)), sizeof(float));
                System.Buffer.BlockCopy(System.BitConverter.GetBytes(-v.x), 0,
                    _PointCloudData, byteDataOffset + (1 * sizeof(float)), sizeof(float));
                System.Buffer.BlockCopy(System.BitConverter.GetBytes(v.y), 0,
                    _PointCloudData, byteDataOffset + (2 * sizeof(float)), sizeof(float));

                byteDataOffset += (3 * sizeof(float));
            }

            var pointCloud2Message = new RosMessageTypes.Sensor.PointCloud2Msg()
            {
                header = new RosMessageTypes.Std.HeaderMsg(),
                height = (uint)1,
                width = (uint)pointsInUnityCoordinates.Length,
                fields = _PointFieldArray,
                is_bigendian = false,
                point_step = sizeof(float) * 3,
                row_step = sizeof(float) * 12 * (uint)pointsInUnityCoordinates.Length,
                data = _PointCloudData,
                is_dense = false
            };

            return pointCloud2Message;
        }

        //*********************************************************************
        /// <summary>
        /// Publish a lidar scan
        /// </summary>
        /// <param name="lidar"></param>
        /// <param name="name"></param>
        /// <param name="positions"></param>
        /// <param name="normals"></param>
        /// <returns>Task result</returns>
        //*********************************************************************
        private Task OnPublishLidarScanDelegate(Sensors.LidarSensor3D lidar, string name,
            NativeArray<Vector3> positions, NativeArray<Vector3> normals)
        {
            var rosPC2Msg = CreateXYZPointCloud(positions.ToArray());
            //rosPC2Msg.header.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg(); // TODO : Set this
            rosPC2Msg.header.frame_id = RosFrameId;

            mRos.Publish(RosTopic, rosPC2Msg);

            return Task.CompletedTask;
        }
    }
}

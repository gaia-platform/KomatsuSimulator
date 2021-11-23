using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;

namespace RosUtil
{
    /// <summary>
    /// 
    /// </summary>

    public class Utils : MonoBehaviour
    {
        private Unity.Robotics.ROSTCPConnector.ROSConnection mRos;

        public static RosMessageTypes.BuiltinInterfaces.TimeMsg RosTime
        {
            get
            {
                var timestamp = new Unity.Robotics.Core.TimeStamp(Unity.Robotics.Core.Clock.time);
                return new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }; 
            }
        }

        public static string _worldFrameId = "map";

        /// <summary>
        /// The name of the world frame.  Usually "map" or "world".
        /// </summary>
        /// <value></value>
        public static string WorldFrameId 
        {
            get => _worldFrameId;
        }

        //*********************************************************************
        /// <summary>
        /// Called by the Unity engine prior to first frame
        /// </summary>
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
        }
    }
}

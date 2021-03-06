//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Vision
{
    [Serializable]
    public class ObjectHypothesisWithPoseMsg : Message
    {
        public const string k_RosMessageName = "vision_msgs/ObjectHypothesisWithPose";
        public override string RosMessageName => k_RosMessageName;

        //  An object hypothesis that contains pose information.
        //  If you would like to define an array of ObjectHypothesisWithPose messages,
        //    please see the Detection2D or Detection3D message types.
        //  The object hypothesis (ID and score).
        public ObjectHypothesisMsg hypothesis;
        //  The 6D pose of the object hypothesis. This pose should be
        //    defined as the pose of some fixed reference point on the object, such as
        //    the geometric center of the bounding box, the center of mass of the
        //    object or the origin of a reference mesh of the object.
        //  Note that this pose is not stamped; frame information can be defined by
        //    parent messages.
        //  Also note that different classes predicted for the same input data may have
        //    different predicted 6D poses.
        public Geometry.PoseWithCovarianceMsg pose;

        public ObjectHypothesisWithPoseMsg()
        {
            this.hypothesis = new ObjectHypothesisMsg();
            this.pose = new Geometry.PoseWithCovarianceMsg();
        }

        public ObjectHypothesisWithPoseMsg(ObjectHypothesisMsg hypothesis, Geometry.PoseWithCovarianceMsg pose)
        {
            this.hypothesis = hypothesis;
            this.pose = pose;
        }

        public static ObjectHypothesisWithPoseMsg Deserialize(MessageDeserializer deserializer) => new ObjectHypothesisWithPoseMsg(deserializer);

        private ObjectHypothesisWithPoseMsg(MessageDeserializer deserializer)
        {
            this.hypothesis = ObjectHypothesisMsg.Deserialize(deserializer);
            this.pose = Geometry.PoseWithCovarianceMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.hypothesis);
            serializer.Write(this.pose);
        }

        public override string ToString()
        {
            return "ObjectHypothesisWithPoseMsg: " +
            "\nhypothesis: " + hypothesis.ToString() +
            "\npose: " + pose.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}

//Do not edit! This file was generated by Unity-ROS MessageGeneration.

using System;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.KomatsuSimulator
{
    [Serializable]
    public class ObstacleArrayMsg : Message
    {
        public const string k_RosMessageName = "KomatsuSimulator/ObstacleArray";

        public ObstacleMsg[] obstacles;

        public ObstacleArrayMsg()
        {
            this.obstacles = new ObstacleMsg[0];
        }

        public ObstacleArrayMsg(ObstacleMsg[] obstacles)
        {
            this.obstacles = obstacles;
        }

        private ObstacleArrayMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.obstacles, ObstacleMsg.Deserialize, deserializer.ReadLength());
        }

        public override string RosMessageName => k_RosMessageName;

        public static ObstacleArrayMsg Deserialize(MessageDeserializer deserializer) =>
            new ObstacleArrayMsg(deserializer);

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.obstacles);
            serializer.Write(this.obstacles);
        }

        public override string ToString()
        {
            return "ObstacleArrayMsg: " +
                   "\nobstacles: " + System.String.Join(", ", obstacles.ToList());
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
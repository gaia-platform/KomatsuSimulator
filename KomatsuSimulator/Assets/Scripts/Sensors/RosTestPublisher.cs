using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;

namespace Sensors
{
    public class RosTestPublisher : MonoBehaviour
    {
        private ROSConnection _ros;

        // SECTION: Editable properties
        [SerializeField] private string topicName = "pos_rot";
        [SerializeField] private float publishFrequency = 0.5f;
        
        // SECTION: Internal Properties
        private float _elapsedTime;
        
        // Start is called before the first frame update
        void Start()
        {
            _ros = ROSConnection.instance;
            _ros.RegisterPublisher<PointMsg>(topicName);
        }

        // Update is called once per frame
        void Update()
        {
            _elapsedTime += Time.deltaTime;
            if (_elapsedTime > publishFrequency)
            {
                Vector3 curPos = transform.position;
                PointMsg msg = new PointMsg(curPos.x, curPos.y, curPos.z);

                _ros.Send(topicName, msg);

                _elapsedTime = 0;
            }
        }
    }
}

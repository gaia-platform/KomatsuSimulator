using UnityEngine;

#pragma warning disable 219

public class DetectorGroundTruthSensor : MonoBehaviour
{
    // the name of the ros topic which to publish
    public string topic;

    // the name of the tag which marks game objects as detectable
    public string tagName;

    Unity.Robotics.ROSTCPConnector.ROSConnection m_Ros;

    void Start()
    {
        // get the ROS connection
        m_Ros = Unity.Robotics.ROSTCPConnector.ROSConnection.instance;

        // register the publisher
        m_Ros.RegisterPublisher(topic, RosMessageTypes.Vision.Detection3DArrayMsg.k_RosMessageName);
    }

    void Update()
    {
        //*** TODO : maybe we should find on a different thread
        var msg = FindAllObjects(tagName);

        // publish
        m_Ros.Send(topic, msg);
    }

    public RosMessageTypes.Vision.Detection3DArrayMsg FindAllObjects(string tagName)
    {
        RosMessageTypes.Vision.Detection3DArrayMsg detectedObjectArray =
            new RosMessageTypes.Vision.Detection3DArrayMsg();

        GameObject[] gos;
        gos = GameObject.FindGameObjectsWithTag(tagName);

        GameObject closest = null;
        float distance = Mathf.Infinity;
        Vector3 position = transform.position;
        int index = 0;

        // make an array of length equal to number of found game objects
        detectedObjectArray.detections = new RosMessageTypes.Vision.Detection3DMsg[gos.GetLength(0)];

        // for each found game object
        foreach (GameObject go in gos)
        {
            // transform position relative to ego
            Vector3 objPos = go.transform.position - position;

            //*** TODO : get game object size
            Vector3 objSize = new Vector3(1.0f, 1.0f, 1.0f);

            float curDistance = objPos.sqrMagnitude;

            detectedObjectArray.detections[index] = new RosMessageTypes.Vision.Detection3DMsg();

            // set bbox position
            var bbPosition = new RosMessageTypes.Geometry.PointMsg(objPos.x, objPos.y, objPos.z);

            // set orientation to identity, maybe we can improve this, not sure if it makes sense to do so
            var bbOrientation = new RosMessageTypes.Geometry.QuaternionMsg(
                Quaternion.identity.x, Quaternion.identity.y, Quaternion.identity.z, Quaternion.identity.w);

            // set bbox pose and size
            RosMessageTypes.Geometry.PoseMsg center = new RosMessageTypes.Geometry.PoseMsg(bbPosition, bbOrientation);
            RosMessageTypes.Geometry.Vector3Msg size =
                new RosMessageTypes.Geometry.Vector3Msg(objSize.x, objSize.y, objSize.z);

            detectedObjectArray.detections[index].bbox = new RosMessageTypes.Vision.BoundingBox3DMsg(center, size);
            detectedObjectArray.detections[index].is_tracking = false;
            detectedObjectArray.detections[index].results = new RosMessageTypes.Vision.ObjectHypothesisWithPoseMsg[1];
            //detectedObjectArray.detections[index].source_cloud
            //detectedObjectArray.detections[index].tracking_id

            //*** TODO : game object name is not correct here. We want an object type, like 'parson' or 'truck'. Maybe we 
            //*** can use a tag for this.
            detectedObjectArray.detections[index].results[0].id = go.gameObject.name;

            index++;
        }

        return detectedObjectArray;
    }
}
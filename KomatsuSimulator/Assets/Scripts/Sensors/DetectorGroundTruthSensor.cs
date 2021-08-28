using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DetectorGroundTruthSensor : MonoBehaviour
{
    // the name of the ros topic which to publish
    public string rosTopic;
    
    // the name of the tag which marks game objects as detectable
    public string tagName;

    public string rosFrame;

    Unity.Robotics.ROSTCPConnector.ROSConnection m_Ros;

    // that's one way to do it, maybe we should align with ros /clock topic
    private RosMessageTypes.BuiltinInterfaces.TimeMsg GetRosTime()    
    {
        var now = System.DateTime.Now;
        var span = now - new System.DateTime(now.Year, now.Month, now.Day);

        return new RosMessageTypes.BuiltinInterfaces.TimeMsg( (int)span.TotalSeconds, (uint)(span.Ticks * 100) );
    }

    public RosMessageTypes.Vision.Detection3DArrayMsg FindAllObjects(string tagName)
    {
        RosMessageTypes.Vision.Detection3DArrayMsg detectedObjectArray =
            new RosMessageTypes.Vision.Detection3DArrayMsg();

        var rosTime = GetRosTime();

        detectedObjectArray.header = new RosMessageTypes.Std.HeaderMsg(rosTime, rosFrame);

        GameObject[] gos;
        gos = GameObject.FindGameObjectsWithTag(tagName); 

        GameObject closest = null;
        float distance = Mathf.Infinity;
        Vector3 position = transform.position;
        int index = 0;
        double[] covariance = 
            {0,0,0,0,0,0,
             0,0,0,0,0,0,
             0,0,0,0,0,0,
             0,0,0,0,0,0,
             0,0,0,0,0,0,
             0,0,0,0,0,0};

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

            // set bbox position
            var bbPosition = new RosMessageTypes.Geometry.PointMsg(objPos.x, objPos.y, objPos.z);

            // set orientation to identity, maybe we can improve this, not sure if it makes sense to do so
            var bbOrientation = new RosMessageTypes.Geometry.QuaternionMsg(
                Quaternion.identity.x, Quaternion.identity.y, Quaternion.identity.z, Quaternion.identity.w);

            // set bbox pose and size
            RosMessageTypes.Geometry.PoseMsg center = new RosMessageTypes.Geometry.PoseMsg(bbPosition,bbOrientation);
            RosMessageTypes.Geometry.Vector3Msg size = new  RosMessageTypes.Geometry.Vector3Msg( objSize.x, objSize.y, objSize.z);

            detectedObjectArray.detections[index] = new RosMessageTypes.Vision.Detection3DMsg();
            detectedObjectArray.detections[index].header = new RosMessageTypes.Std.HeaderMsg(rosTime, rosFrame);
            detectedObjectArray.detections[index].bbox = new RosMessageTypes.Vision.BoundingBox3DMsg(center,size);
            detectedObjectArray.detections[index].results = new RosMessageTypes.Vision.ObjectHypothesisWithPoseMsg[1];

            //*** TODO : game object name is not correct here. We want an object type, like 'person' or 'truck'. Maybe we 
            //*** can use a tag for this.

            var hyp = new RosMessageTypes.Vision.ObjectHypothesisMsg(go.name,1.0);
            var pwc = new RosMessageTypes.Geometry.PoseWithCovarianceMsg(center,covariance);

            detectedObjectArray.detections[index].results[0] = new RosMessageTypes.Vision.ObjectHypothesisWithPoseMsg(hyp,pwc);

            index++;
        }
        return detectedObjectArray;
    }

    void Start()
    {
        // get the ROS connection
        m_Ros = Unity.Robotics.ROSTCPConnector.ROSConnection.instance;

        // register the publisher
        m_Ros.RegisterPublisher(rosTopic, RosMessageTypes.Vision.Detection3DArrayMsg.k_RosMessageName);
    }

    void Update()
    {
        //*** TODO : maybe we should find on a different thread
        var msg = FindAllObjects(tagName);

        // publish
        m_Ros.Send(rosTopic, msg);
    }
}

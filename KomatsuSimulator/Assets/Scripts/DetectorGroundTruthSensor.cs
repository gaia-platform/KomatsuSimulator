using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DetectorGroundTruthSensor : MonoBehaviour
{
    public RosMessageTypes.Vision.Detection3DArrayMsg FindAllObjects(string tagName)
    {
        RosMessageTypes.Vision.Detection3DArrayMsg detectedObjectArray =
            new RosMessageTypes.Vision.Detection3DArrayMsg();

        GameObject[] gos;
        gos = GameObject.FindGameObjectsWithTag(tagName);

        detectedObjectArray.detections = new RosMessageTypes.Vision.Detection3DMsg[4];

        GameObject closest = null;
        float distance = Mathf.Infinity;
        Vector3 position = transform.position;
        int index = 0;

        foreach (GameObject go in gos)
        {
            Vector3 diff = go.transform.position - position;
            float curDistance = diff.sqrMagnitude;

            detectedObjectArray.detections[index] = new RosMessageTypes.Vision.Detection3DMsg();

            index++;
        }
        return detectedObjectArray;
    }

    void Start()
    {
        
    }

    void Update()
    {
        
    }
}

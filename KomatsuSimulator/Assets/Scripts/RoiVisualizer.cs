using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.DangerZone;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class RoiVisualizer : MonoBehaviour
{
    // SECTION: Nodes and properties
    [SerializeField] private GameObject roiPrefab;
    private string _obstacles_topic_name = "/komatsu/obstacles"; // Topic to subscribe to

    private Dictionary<GameObject, Renderer> _roiBoxesAndRenderers; // Registry of ROI boxes and their renderers


    // Start is called before the first frame update
    private void Start()
    {
        ROSConnection.instance.Subscribe<ObstacleArrayMsg>(_obstacles_topic_name, VisualizeObstacles);
    }

    private void VisualizeObstacles(ObstacleArrayMsg obstacleArray)
    {
        // Add or remove prefabs
        int deltaSize = obstacleArray.obstacles.Length - _roiBoxesAndRenderers.Count;
        for (int i = 0; i < Mathf.Abs(deltaSize); i++)
        {
            if (deltaSize > 0) // New obstacles in array
            {
                GameObject roiBoxInstance = Instantiate(roiPrefab);
                _roiBoxesAndRenderers.Add(roiBoxInstance, roiBoxInstance.GetComponent<Renderer>());
            }
            else // Otherwise, shrink dictionary
            {
                GameObject lastBoxInstance = _roiBoxesAndRenderers.Keys.Last();
                _roiBoxesAndRenderers.Remove(lastBoxInstance);
                Destroy(lastBoxInstance);
            }
        }
    }
}
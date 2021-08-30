using System.Collections.Generic;
using RosMessageTypes.DangerZone;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class RoiVisualizer : MonoBehaviour
{
    private const string ObstaclesTopicName = "/komatsu/obstacles"; // Topic to subscribe to

    // SECTION: Nodes and properties
    [SerializeField] private GameObject roiBoxPrefab;

    private readonly List<GameObject> _roiBoxes = new List<GameObject>(); // List of ROI Boxes
    private readonly List<Renderer> _roiBoxRenderers = new List<Renderer>(); // Associated list of renderers


    // Start is called before the first frame update
    private void Start()
    {
        ROSConnection.instance.Subscribe<ObstacleArrayMsg>(ObstaclesTopicName, VisualizeObstacles);
    }

    private void VisualizeObstacles(ObstacleArrayMsg obstacleArray)
    {
        // Add or remove prefabs
        int deltaSize = obstacleArray.obstacles.Length - _roiBoxes.Count;
        for (int i = 0; i < Mathf.Abs(deltaSize); i++)
        {
            if (deltaSize > 0) // New obstacles in array
            {
                GameObject roiBoxInstance = Instantiate(roiBoxPrefab);
                _roiBoxes.Add(roiBoxInstance);
                _roiBoxRenderers.Add(roiBoxInstance.GetComponent<Renderer>());
            }
            else // Otherwise, shrink dictionary
            {
                int removeIndex = _roiBoxes.Count - 1;
                _roiBoxes.RemoveAt(removeIndex);
                _roiBoxRenderers.RemoveAt(removeIndex);
            }
        }

        // Loop through each obstacle
        for (int i = 0; i < obstacleArray.obstacles.Length; i++)
        {
            // Get current obstacle
            ObstacleMsg curObstacle = obstacleArray.obstacles[i];

            // Set ROI color
            _roiBoxRenderers[i].material.color = curObstacle.roi switch
            {
                ObstacleMsg.RED => new Color(1, 0, 0, 0.3f),
                ObstacleMsg.YELLOW => new Color(1, 1, 0, 0.3f),
                _ => new Color(0, 1, 0, 0.3f)
            };

            // Set box
            GameObject curRoiBox = _roiBoxes[i];

            PointMsg boundsPos = curObstacle.bounds.center.position;
            QuaternionMsg boundsOri = curObstacle.bounds.center.orientation;
            Vector3Msg boundsSize = curObstacle.bounds.size;

            Vector3 position = new Vector3((float)boundsPos.x, (float)boundsPos.y, (float)boundsPos.z) +
                               transform.position;

            curRoiBox.transform.position = position;
            curRoiBox.transform.rotation = new Quaternion((float)boundsOri.x, (float)boundsOri.y,
                (float)boundsOri.z, (float)boundsOri.w);
            curRoiBox.transform.localScale = new Vector3((float)boundsSize.x, (float)boundsSize.y, (float)boundsSize.z);
        }
    }
}
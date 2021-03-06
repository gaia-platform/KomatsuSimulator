using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using UnityEngine;

namespace Sensors
{
    public class RoiSpaces : MonoBehaviour
    {
        // SECTION: Variables
        private readonly Dictionary<string, List<Vector3>> _thisFrameObstaclePoints =
            new Dictionary<string, List<Vector3>>();

        private void FixedUpdate()
        {
            // Create Obstacles list (Temp data generated by colliders
            List<Obstacle> thisFrameObstacles = new List<Obstacle>();
            foreach (string obstacle in _thisFrameObstaclePoints.Keys)
            {
                List<Vector3> obstacleHits = _thisFrameObstaclePoints[obstacle];
                if (obstacleHits.Count > 1)
                {
                    obstacleHits = obstacleHits.OrderBy(hit => (hit - transform.position).sqrMagnitude).ToList();
                }

                // Add to obstacles with temp ROI and Direction values
                thisFrameObstacles.Add(new Obstacle(obstacle, obstacleHits[0], Roi.Yellow, Direction.SideRight));
            }

            _thisFrameObstaclePoints.Clear();


            foreach (Obstacle thisFrameObstacle in thisFrameObstacles)
            {
                Debug.DrawLine(transform.position, thisFrameObstacle.ClosestPoint);
            }
        }

        [SuppressMessage("ReSharper", "UnusedMember.Local")]
        private void OnTriggerStay(Collider other)
        {
            if (!other.CompareTag("Obstacle")) return;

            Vector3 hitClosestPoint = other.ClosestPoint(transform.position);

            if (!_thisFrameObstaclePoints.ContainsKey(other.name))
            {
                _thisFrameObstaclePoints[other.name] = new List<Vector3>();
            }

            _thisFrameObstaclePoints[other.name].Add(hitClosestPoint);
        }

        // SECTION: Structs and enums
        private enum Roi
        {
            Red,
            Yellow,
            Green
        }

        private enum Direction
        {
            FrontRight,
            SideRight,
            BackRight,
            BackLeft,
            SideLeft,
            FrontLeft
        }

        private struct Obstacle
        {
            public string Name { get; set; }
            public Vector3 ClosestPoint { get; set; }
            public Roi Roi { get; set; }
            public Direction Direction { get; set; }

            public Obstacle(string name, Vector3 closestPoint, Roi roi, Direction direction)
            {
                Name = name;
                ClosestPoint = closestPoint;
                Roi = roi;
                Direction = direction;
            }
        }
    }
}
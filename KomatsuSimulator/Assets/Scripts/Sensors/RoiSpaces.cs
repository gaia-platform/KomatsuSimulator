using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using UnityEngine;

namespace Sensors
{
    public class RoiSpaces : MonoBehaviour
    {
        // SECTION: Structs and enums
        // private enum Roi
        // {
        //     Red,
        //     Green,
        //     Blue
        // }
        //
        // private enum Direction
        // {
        //     FrontRight,
        //     SideRight,
        //     BackRight,
        //     BackLeft,
        //     SideLeft,
        //     FrontLeft
        // }

        // private struct Obstacle
        // {
        //     public Vector3 ClosestPoint { get; set; }
        //     public Roi Roi { get; set; }
        //     public Direction Direction { get; set; }
        //
        //     public Obstacle(Vector3 closestPoint, Roi roi, Direction direction)
        //     {
        //         ClosestPoint = closestPoint;
        //         Roi = roi;
        //         Direction = direction;
        //     }
        // }

        // SECTION: Variables
        private readonly Dictionary<string, List<Vector3>> _thisFrameObstaclePoints =
            new Dictionary<string, List<Vector3>>();

        private void FixedUpdate()
        {
            foreach (string obstacle in _thisFrameObstaclePoints.Keys)
            {
                List<Vector3> obstacleHits = _thisFrameObstaclePoints[obstacle];
                if (obstacleHits.Count > 1)
                {
                    obstacleHits = obstacleHits.OrderBy(hit => (hit - transform.position).sqrMagnitude).ToList();
                }

                Debug.DrawLine(transform.position, obstacleHits[0]);
            }

            _thisFrameObstaclePoints.Clear();
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
    }
}
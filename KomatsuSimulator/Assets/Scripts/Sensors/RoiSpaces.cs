using System;
using System.Diagnostics.CodeAnalysis;
using UnityEngine;

namespace Sensors
{
    public class RoiSpaces : MonoBehaviour
    {
        // SECTION: Properties

        // Start is called before the first frame update
        void Start()
        {
        }

        // Update is called once per frame
        void Update()
        {
        }

        [SuppressMessage("ReSharper", "UnusedMember.Local")]
        private void OnTriggerStay(Collider other)
        {
            print(other.name);
            if (other.tag == "Obstacle")
            {
                Vector3 transformPosition = transform.position;
                Vector3 hitClosestPoint = other.ClosestPoint(transformPosition);

                Debug.DrawLine(transformPosition, hitClosestPoint, Color.cyan);
            }
        }
    }
}
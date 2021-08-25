using System.Diagnostics.CodeAnalysis;
using UnityEngine;

namespace Assets.Scripts.Sensors
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
            if (other.tag == "Obstacle")
            {
                print(other.name);
                Vector3 transformPosition = transform.position;
                Vector3 hitClosestPoint = other.ClosestPointOnBounds(transformPosition);
        
                Debug.DrawLine(transformPosition, hitClosestPoint, Color.cyan);
            }
        }
    }
}
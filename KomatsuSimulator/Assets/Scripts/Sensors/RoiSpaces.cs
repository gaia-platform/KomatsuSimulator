using System;
using UnityEngine;

namespace Sensors
{
    public class RoiSpaces : MonoBehaviour
    {
        // SECTION: Properties
        private ContactPoint[] _contactPoints = new ContactPoint[5];
        
        // Start is called before the first frame update
        void Start()
        {
        
        }

        // Update is called once per frame
        void Update()
        {
        
        }

        private void OnTriggerStay(Collider other)
        {
        }
    }
}

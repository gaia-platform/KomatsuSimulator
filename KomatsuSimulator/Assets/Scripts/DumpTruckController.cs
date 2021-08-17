using UnityEngine;

public class DumpTruckController : MonoBehaviour
{
    // SECTION: Component References
    // Cameras
    public GameObject thirdPersonCamera;
    public GameObject topDownCamera;

    // Wheel colliders and transforms
    [SerializeField] private WheelCollider[] wheelColliders;
    [SerializeField] private Transform[] wheelTransforms;

    // RigidBody
    [SerializeField] private Rigidbody truckRigidbody;

    // SECTION: Properties
    [SerializeField] private float thrust;
    [SerializeField] private float maxSteeringAngle;
    [SerializeField] private float breakForce;


    // Start is called before the first frame update
    void Start()
    {
        truckRigidbody.centerOfMass = new Vector3(0, 1.34f, 0);
    }

    private void FixedUpdate()
    {
        // Get user input
        float forwardDrive = Input.GetAxis("Vertical");
        float turn = Input.GetAxis("Horizontal");
        bool isBreaking = Input.GetKey(KeyCode.Space);

        // Handle driving the motors
        for (int i = 0; i < 2; i++) // Apply motor torque on front two wheels
        {
            wheelColliders[i].motorTorque = forwardDrive * thrust;
        }

        float curBreakForce = isBreaking ? breakForce : 0;
        foreach (WheelCollider wheelCollider in wheelColliders)
        {
            wheelCollider.brakeTorque = curBreakForce;
        }

        // Handle Steering
        var curSteerAngle = maxSteeringAngle * turn;
        for (int i = 0; i < 2; i++)
        {
            wheelColliders[i].steerAngle = curSteerAngle;
        }

        // Update wheel visuals
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            wheelColliders[i].GetWorldPose(out Vector3 pos, out Quaternion rot);
            wheelTransforms[i].position = pos;
            wheelTransforms[i].rotation = rot;
        }
    }
}
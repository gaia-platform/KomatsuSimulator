using UnityEngine;

public class DumpTruckController : MonoBehaviour
{
    // SECTION: Component References
    // Cameras
    [SerializeField] private Camera thirdPersonCamera;

    [SerializeField] private Camera topDownCamera;

    // Axles
    [SerializeField] private GameObject frontLeftWheel;
    [SerializeField] private GameObject frontRightWheel;
    [SerializeField] private GameObject middleAxle;

    [SerializeField] private GameObject backAxle;

    // RigidBody
    [SerializeField] private Rigidbody truckRigidbody;

    // SECTION: Properties
    [SerializeField] private float thrust;
    [SerializeField] private float torque;


    // Start is called before the first frame update
    void Start()
    {
        truckRigidbody.centerOfMass = Vector3.zero;
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void FixedUpdate()
    {
        float forwardDrive = Input.GetAxis("Vertical");
        float turn = Input.GetAxis("Horizontal");


        truckRigidbody.AddForce(transform.forward * thrust * forwardDrive);
        float overSpeed = truckRigidbody.velocity.sqrMagnitude - 100;
        if (overSpeed > 0)
        {
            truckRigidbody.AddForce(transform.forward * overSpeed * forwardDrive * -1);
        }

        truckRigidbody.AddTorque(transform.up * turn);
        // float overTorque = truckRigidbody.angularVelocity.sqrMagnitude - 0.3f;
        // if (overTorque > 0)
        // {
        //     truckRigidbody.AddTorque(transform.up * overTorque * turn * -1);
        // }
        //
        // Quaternion deltaRotation = Quaternion.Euler(new Vector3(0, angularVelocity, 0) * Time.fixedDeltaTime * turn * forwardDrive);
        // truckRigidbody.MoveRotation(truckRigidbody.rotation * deltaRotation);

        print(truckRigidbody.velocity.sqrMagnitude + ", " + truckRigidbody.angularVelocity.sqrMagnitude);
    }
}
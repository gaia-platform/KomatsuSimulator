using UnityEngine;

public class DumpTruckController : MonoBehaviour
{
    // SECTION: Component References
    // Cameras
    public GameObject thirdPersonCamera;
    public GameObject topDownCamera;

    // Axles
    [SerializeField] private GameObject frontRightWheel;
    [SerializeField] private GameObject frontLeftWheel;
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

        // Handle forward/backward driving
        truckRigidbody.AddForce(transform.forward * (thrust * forwardDrive)); // Add force
        float overSpeed = truckRigidbody.velocity.sqrMagnitude - 100; // Check if over speed-limit of 10 m/s
        if (overSpeed > 0) // If so, apply counter force to keep at 10 m/s
        {
            truckRigidbody.AddForce(transform.forward * (overSpeed * forwardDrive * -1));
        }

        // Rotate wheels to speed
        var wheelRotateSpeed = new Vector3(truckRigidbody.velocity.sqrMagnitude * thrust * Time.fixedDeltaTime, 0, 0);
        backAxle.transform.Rotate(wheelRotateSpeed);
        middleAxle.transform.Rotate(wheelRotateSpeed);
        frontRightWheel.transform.Rotate(wheelRotateSpeed);
        frontLeftWheel.transform.Rotate(wheelRotateSpeed);

        // Handle turning
        truckRigidbody.AddTorque(transform.up * (torque * turn)); // Add torque
        float overTorque = Mathf.Abs(truckRigidbody.angularVelocity.y) - 0.3f; // check if over speed-limit
        if (overTorque > 0) // If so, apply counter torque to keep at this speed
        {
            truckRigidbody.AddTorque(transform.up * (overTorque * torque * turn * -1));
        }
    }
}
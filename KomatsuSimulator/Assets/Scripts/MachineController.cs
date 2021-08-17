using UnityEngine;

public class MachineController : MonoBehaviour
{
    // SECTION: Component References
    // Cameras
    [SerializeField] private GameObject[] cameras;
    [SerializeField] private uint onCameraIndex;

    // Wheel colliders and transforms
    [SerializeField] private WheelCollider[] wheelColliders;
    [SerializeField] private Transform[] wheelTransforms;

    // RigidBody
    [SerializeField] private Rigidbody truckRigidbody;

    // SECTION: Properties
    [SerializeField] private float thrust;
    [SerializeField] private float maxSteeringAngle;


    // Start is called before the first frame update
    void Start()
    {
        truckRigidbody.centerOfMass = new Vector3(0, 1.34f, 0);
        Activate();
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

        float curBreakForce = isBreaking ? thrust : 0;
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
    
    /// Controller Methods
    // Camera handler
    public void SwitchCameras()
    {
        onCameraIndex++;
        if (onCameraIndex == cameras.Length)
        {
            onCameraIndex = 0;
        }
        SwitchToCamera(onCameraIndex);
    }
    // Deactivate (on switch to another machine)
    public void Activate(bool state=true)
    {
        if (state)
        {
            SwitchToCamera(0);
        }
        else
        {
            FullStopBrake();
        }
    }
    
    // Full Stop braking
    public void FullStopBrake()
    {
        
    }
    
    // Get velocity magnitude
    public float GetSpeed()
    {
        return truckRigidbody.velocity.magnitude;
    }
    
    // Private methods
    private void SwitchToCamera(uint index)
    {
        for (int i = 0; i < cameras.Length; i++)
        {
            cameras[i].SetActive(i == index);
        }
    }
}
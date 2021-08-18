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

    // SECTION: Properties
    [SerializeField] private float thrust;
    [SerializeField] private float maxSteeringAngle;

    private Rigidbody _machineRigidBody;

    private bool _isFullStopBraking;
    private uint _brakingLoop;

    // Start is called before the first frame update
    private void Start()
    {
        _machineRigidBody = GetComponent<Rigidbody>();
        _machineRigidBody.centerOfMass = new Vector3(0, 1.34f, 0);
        Activate();
    }

    private void FixedUpdate()
    {
        if (_isFullStopBraking)
        {
            if (Mathf.Round(_machineRigidBody.velocity.sqrMagnitude * 100) / 100 > 0)
            {
                if (wheelColliders[0].brakeTorque == 0)
                {
                    foreach (WheelCollider wheelCollider in wheelColliders)
                    {
                        wheelCollider.motorTorque = 0;
                        wheelCollider.brakeTorque = Mathf.Pow(thrust, 5);
                    }

                    _machineRigidBody.velocity = Vector3.zero;
                }
            }
            else
            {
                foreach (WheelCollider wheelCollider in wheelColliders)
                {
                    wheelCollider.brakeTorque = 0;
                }

                _machineRigidBody.velocity = Vector3.zero;
                _brakingLoop++;
                if (_brakingLoop == 5)
                {
                    _brakingLoop = 0;
                    _isFullStopBraking = false;
                }
            }

            return;
        }

        // Get user input
        float forwardDrive = Input.GetAxis("Vertical");
        float turn = Input.GetAxis("Horizontal");

        // Handle driving the motors
        float machineDirection = Vector3.Dot(_machineRigidBody.velocity, transform.forward);
        if (forwardDrive < 0 && machineDirection > 0 ||
            forwardDrive > 0 && machineDirection < 0)
        {
            foreach (WheelCollider wheelCollider in wheelColliders)
            {
                wheelCollider.brakeTorque = thrust;
            }
        }
        else
        {
            if (forwardDrive != 0 || wheelColliders[0].brakeTorque != 0)
            {
                foreach (WheelCollider wheelCollider in wheelColliders)
                {
                    wheelCollider.brakeTorque = 0;
                    wheelCollider.motorTorque = forwardDrive * thrust;
                }
            }
            else
            {
                foreach (WheelCollider wheelCollider in wheelColliders)
                {
                    wheelCollider.brakeTorque = thrust;
                }
            }
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
    public void Activate(bool state = true)
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
        // foreach (WheelCollider wheelCollider in wheelColliders)
        // {
        //     wheelCollider.motorTorque = 0;
        //     wheelCollider.brakeTorque = Mathf.Pow(thrust, 5);
        // }
        //
        // _machineRigidBody.velocity = Vector3.zero;
        _isFullStopBraking = true;
    }

    // Get velocity magnitude
    public float GetSpeed()
    {
        return _machineRigidBody.velocity.magnitude;
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
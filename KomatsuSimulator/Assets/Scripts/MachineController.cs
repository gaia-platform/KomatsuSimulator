using System;
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

    // Roi Visualizer
    [SerializeField] private RoiVisualizer visualizerScript;

    // SECTION: Properties
    [SerializeField] private float thrust;
    [SerializeField] private float maxSteeringAngle;

    // Automatic movement items
    [SerializeField] private Transform targetTransform;
    private readonly Vector3[] _goalLocations = new Vector3[2];
    private uint _brakingLoop;

    // Disabled
    private bool _isDisabled;

    // Braking
    private bool _isFullStopBraking;
    private Vector3 _lookDirection;
    private Quaternion _lookRotation;

    private Rigidbody _machineRigidBody;
    private Vector3 _moveDirection;
    private bool _stopMovement;

    // Start is called before the first frame update
    private void Start()
    {
        _machineRigidBody = GetComponent<Rigidbody>();
        _machineRigidBody.centerOfMass = new Vector3(0, 1.34f, 0);

        // Setup automation stuff (copy from PersonController)
        if (!targetTransform) return;
        _goalLocations[0] = transform.position;
        _goalLocations[1] = targetTransform.position;
        _moveDirection = (_goalLocations[1] - _goalLocations[0]).normalized;
        _lookDirection = _moveDirection;
        _lookRotation = Quaternion.LookRotation(_lookDirection);
    }

    private void FixedUpdate()
    {
        void UpdateWheelVisuals()
        {
            // Update wheel visuals
            for (int i = 0; i < wheelTransforms.Length; i++)
            {
                wheelColliders[i].GetWorldPose(out Vector3 pos, out Quaternion rot);
                wheelTransforms[i].position = pos;
                wheelTransforms[i].rotation = rot;
            }
        }

        // First, check if we are trying to do a full stop brake
        if (_isFullStopBraking)
        {
            if (Mathf.Round(_machineRigidBody.velocity.sqrMagnitude * 100) / 100 > 0)
            {
                foreach (WheelCollider wheelCollider in wheelColliders)
                {
                    wheelCollider.motorTorque = 0;
                    wheelCollider.brakeTorque = Mathf.Pow(thrust, 5);
                }

                _machineRigidBody.velocity = Vector3.zero;
            }
            else
            {
                foreach (WheelCollider wheelCollider in wheelColliders)
                {
                    wheelCollider.brakeTorque = 0;
                }

                _machineRigidBody.velocity = Vector3.zero;
                _brakingLoop++;
                if (_brakingLoop != 5) return;
                _brakingLoop = 0;
                _isFullStopBraking = false;
            }

            return;
        }

        // Then check for automation items
        if (targetTransform)
        {
            if (_stopMovement) return;

            // Get this frame's transform
            Transform thisFrameTransform = transform;

            // Rotate while not facing right direction
            if (Vector3.Dot(thisFrameTransform.forward, _lookDirection) < 0.99999)
            {
                thisFrameTransform.rotation = Quaternion.Lerp(thisFrameTransform.rotation, _lookRotation,
                    Time.fixedDeltaTime * 10);
                return;
            }

            // If not rotating, continue moving
            foreach (WheelCollider wheelCollider in wheelColliders)
            {
                wheelCollider.motorTorque = thrust;
            }

            UpdateWheelVisuals();

            // Switch direction when close to goal location
            Vector3 curDirectionToGoalLocation = _goalLocations[1] - thisFrameTransform.position;
            if (Vector3.Dot(curDirectionToGoalLocation, _lookDirection) > 0) return; // Skip if not there yet
            Array.Reverse(_goalLocations);
            _lookDirection *= -1;
            _lookRotation = Quaternion.LookRotation(_lookDirection);
            FullStopBrake(); // Stop before turn
            return;
        }

        // Next check if this machine is disabled
        if (_isDisabled)
        {
            return;
        }


        // If not braking, driving automatically, or disabled:

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


        UpdateWheelVisuals();
    }

    // SECTION: Collider signals
    private void OnCollisionEnter()
    {
        _stopMovement = true;
    }

    private void OnCollisionExit()
    {
        _stopMovement = false;
    }

    /// Controller Methods
    // Camera handler
    public void SwitchCameras()
    {
        // Disable current camera
        GetActiveCamera().SetActive(false);

        // Increment index
        onCameraIndex++;
        if (onCameraIndex == cameras.Length)
        {
            onCameraIndex = 0;
        }

        // Activate new camera
        GetActiveCamera().SetActive(true);
    }

    // Deactivate (on switch to another machine)
    public void Activate(bool state = true)
    {
        if (!state)
        {
            if (!targetTransform)
            {
                FullStopBrake();
            }
        }

        GetActiveCamera().SetActive(state);
        visualizerScript.enabled = state;
        _isDisabled = !state;
    }

    // Full Stop braking
    public void FullStopBrake()
    {
        _isFullStopBraking = true;
    }

    // Get velocity magnitude
    public float GetSpeed()
    {
        return _machineRigidBody.velocity.magnitude;
    }

    // Get current active camera
    public GameObject GetActiveCamera()
    {
        return cameras[onCameraIndex];
    }
}
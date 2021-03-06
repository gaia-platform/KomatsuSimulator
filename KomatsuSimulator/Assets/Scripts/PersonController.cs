using System;
using UnityEngine;

public class PersonController : MonoBehaviour
{
    private static readonly int IsMoving = Animator.StringToHash("isMoving");

    // SECTION: Variables
    [SerializeField] private Animator animator;
    [SerializeField] private Transform targetTransform; // Get target location from editor

    // Movement properties
    [SerializeField] private float movementSpeed;

    [SerializeField] private float rotationSpeed;

    // General properties
    private readonly Vector3[] _goalLocations = new Vector3[2];
    private Quaternion _lookRotation;
    private Vector3 _moveDirection;
    private bool _stopMovement;


    // Start is called before the first frame update
    void Start()
    {
        // Set goal locations only if a transform was given
        if (!targetTransform) return;
        _goalLocations[0] = transform.position;
        _goalLocations[1] = targetTransform.position;
        _moveDirection = (_goalLocations[1] - _goalLocations[0]).normalized;
        _lookRotation = Quaternion.LookRotation(_moveDirection);
    }

    void Update()
    {
        if (!targetTransform) return; // Skip out if no transform
        animator.SetBool(IsMoving, !_stopMovement);
        if (_stopMovement) return; // Skip if collided

        // Get this frame's transform
        Transform thisFrameTransform = transform;

        // Rotate while not facing right direction
        if (thisFrameTransform.rotation != _lookRotation)
        {
            animator.SetBool(IsMoving, false);
            thisFrameTransform.rotation = Quaternion.Slerp(thisFrameTransform.rotation, _lookRotation,
                Time.deltaTime * rotationSpeed);
            return;
        }

        // If not rotating, continue moving
        thisFrameTransform.Translate(_moveDirection * (movementSpeed * Time.deltaTime), Space.World);

        // Switch direction when close to goal location
        Vector3 curDirectionToGoalLocation = _goalLocations[1] - thisFrameTransform.position;
        if (Vector3.Dot(curDirectionToGoalLocation, _moveDirection) > 0) return; // Skip if not there yet
        Array.Reverse(_goalLocations);
        _moveDirection *= -1;
        _lookRotation = Quaternion.LookRotation(_moveDirection);
    }

    // Update is called once per frame

    // If Colliding with something, stop movement
    private void OnCollisionEnter()
    {
        _stopMovement = true;
    }

    private void OnCollisionExit()
    {
        _stopMovement = false;
    }
}
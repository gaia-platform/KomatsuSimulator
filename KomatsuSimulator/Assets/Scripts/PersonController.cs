using System;
using UnityEngine;

public class PersonController : MonoBehaviour
{
    // SECTION: Variables
    [SerializeField] private Transform targetTransform; // Get target location from editor
    [SerializeField] private float movementSpeed;
    private readonly Vector3[] _goalLocations = new Vector3[2];
    private bool _stopMovement;


    // Start is called before the first frame update
    void Start()
    {
        if (!targetTransform) return;
        _goalLocations[0] = transform.position;
        _goalLocations[1] = targetTransform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if (!targetTransform) return;
        if (_stopMovement) return;

        Vector3 movement = _goalLocations[1] - _goalLocations[0];

        transform.Translate(movement * movementSpeed * Time.deltaTime);

        if ((transform.position - _goalLocations[1]).sqrMagnitude < 0.1)
        {
            Array.Reverse(_goalLocations);
        }
    }

    private void OnCollisionEnter(Collision other)
    {
        _stopMovement = true;
    }

    private void OnCollisionExit(Collision other)
    {
        _stopMovement = false;
    }
}
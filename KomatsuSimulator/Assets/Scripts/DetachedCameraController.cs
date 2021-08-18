using UnityEngine;

public class DetachedCameraController : MonoBehaviour
{
    // SECTION: Properties
    [SerializeField] private float mouseSensitivity;
    [SerializeField] private float baseMovementSensitivity;


    // Update is called once per frame
    void Update()
    {
        // Get input
        float mouseX = Input.GetAxis("Mouse X");
        float mouseY = Input.GetAxis("Mouse Y");
        float moveLateral = Input.GetAxis("Horizontal");
        float moveForward = Input.GetAxis("Vertical");


        if (Input.GetMouseButtonDown(0))
        {
            transform.parent.gameObject.GetComponent<UIController>().ToggleDetachView();
        }

        // Set sensitivity, get access to transform
        float movementSensitivity =
            Input.GetKey(KeyCode.LeftShift) ? baseMovementSensitivity + 30 : baseMovementSensitivity;
        Transform thisFrameTransforms = transform;

        // Set position
        thisFrameTransforms.Translate((Vector3.forward * moveForward + Vector3.right * moveLateral) *
                                      (movementSensitivity * Time.deltaTime));

        // Set rotation
        thisFrameTransforms.Rotate((Vector3.right * (mouseY * -1) + Vector3.up * mouseX) *
                                   (mouseSensitivity * Time.deltaTime));

        // reset z axis tilt if needed
        Vector3 localRotationEulerAngles = thisFrameTransforms.localRotation.eulerAngles;
        if (localRotationEulerAngles.z != 0)
        {
            thisFrameTransforms.localRotation = Quaternion.Euler(localRotationEulerAngles.x,
                localRotationEulerAngles.y, 0);
        }
    }
}
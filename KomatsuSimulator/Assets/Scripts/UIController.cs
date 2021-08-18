using UnityEngine;
using UnityEngine.UI;

public class UIController : MonoBehaviour
{
    // SECTION: Component References
    // Machines
    [SerializeField] private MachineController[] machines;
    [SerializeField] private uint onMachineIndex;

    // Detached camera
    [SerializeField] private GameObject detachedCamera;

    // Labels
    [SerializeField] private Text currentMachineLabel;
    [SerializeField] private Text detachButtonLabel;
    [SerializeField] private Text speedometerLabel;

    // SECTION: Variables
    private bool _isDetached;

    // SECTION: Main Loop
    private void FixedUpdate()
    {
        float round = Mathf.Round(machines[onMachineIndex].GetSpeed() * 2.236936f * 100) / 100;
        speedometerLabel.text = "Speed (mph): " + round;
    }

    // SECTION: UI Actions
    public void ToggleDetachView() // Switch between free camera and current machine
    {
        // Change detached value to new state
        _isDetached = !_isDetached;

        if (_isDetached) // Was just set to true
        {
            detachButtonLabel.text = "Reattached View";
            FullStopBrake(); // Stop current machine between detaching
            SwitchToMachine(0); // Disable machines

            // Align detached camera to current view
            Transform activeCameraTransform = machines[onMachineIndex].GetActiveCamera().gameObject.transform;
            detachedCamera.transform.rotation = activeCameraTransform.rotation;
            detachedCamera.transform.position = activeCameraTransform.position;
        }
        else // Switch back to current machine
        {
            detachButtonLabel.text = "Detach View";
            SwitchToMachine(onMachineIndex);
        }

        // Finally, de/activate detached camera
        detachedCamera.SetActive(_isDetached);
    }

    public void SwitchMachines() // Change machine
    {
        // Increment index and set machine
        onMachineIndex++;
        if (onMachineIndex == machines.Length)
        {
            onMachineIndex = 0;
        }

        SwitchToMachine(onMachineIndex);
    }

    public void SwitchCameras() // Change camera
    {
        machines[onMachineIndex].SwitchCameras();
    }

    public void FullStopBrake() // Stop machine
    {
        machines[onMachineIndex].FullStopBrake();
    }

    // Private methods
    private void SwitchToMachine(uint index)
    {
        // If detached, disable machines
        if (_isDetached)
        {
            foreach (MachineController machineController in machines)
            {
                machineController.Activate(false);
            }

            return;
        }

        // Otherwise, deactivate everyone but the selected index
        onMachineIndex = index;
        for (int i = 0; i < machines.Length; i++)
        {
            if (i != onMachineIndex)
            {
                machines[i].Activate(false);
            }
        }

        machines[onMachineIndex].Activate();
        currentMachineLabel.text = machines[onMachineIndex].gameObject.name;
    }
}
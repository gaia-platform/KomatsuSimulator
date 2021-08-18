using UnityEngine;
using UnityEngine.UI;

public class UIController : MonoBehaviour
{
    // SECTION: Component References
    // Machines
    [SerializeField] private MachineController[] machines;
    [SerializeField] private uint onMachineIndex;

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
        _isDetached = !_isDetached;

        if (_isDetached) // Was just set to true
        {
            detachButtonLabel.text = "Reattached View";
            FullStopBrake(); // Stop current machine between detaching
            // Do stuff to detach camera 
        }
        else // Switch back to current machine
        {
            detachButtonLabel.text = "Detach View";
            SwitchToMachine(onMachineIndex);
        }
    }

    public void SwitchMachines() // Change machine
    {
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
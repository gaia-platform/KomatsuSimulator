using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.DangerZone;
using Unity.Robotics.ROSTCPConnector;

//*********************************************************************
/// <summary>
/// Attach to a game object to control visibility in response to a ROS
/// SnapshotTriggeredMsg
/// </summary>
//*********************************************************************   
public class TriggerVisualizer : MonoBehaviour
{
    // display time span in seconds
    public float DispalySpanSeconds = 4;

    // subscribe to this topic
    public string m_TriggeredTopicName = "/komatsu/triggered";

    public GameObject ControlledObject;

    // reference to coroutine
    private Coroutine m_DeactivateCoroutineRef = null;

    System.TimeSpan mShowSpan = new System.TimeSpan(0, 0, 4);

    //*********************************************************************
    /// <summary>
    /// Called by the game engine once before the first frame is updated.
    /// </summary>
    /// <param name="triggerData"></param>
    /// <returns>void</returns>
    //*********************************************************************    
    private void Start()
    {
        // subscribe
        ROSConnection.GetOrCreateInstance().Subscribe<SnapshotTriggeredMsg>(
            m_TriggeredTopicName, TriggeredCallback);
    }

    //*********************************************************************
    /// <summary>
    /// Called by the game engine when a SnapshotTriggeredMsg is received
    /// </summary>
    /// <param name="triggerData"></param>
    /// <returns>void</returns>
    //*********************************************************************
    private void TriggeredCallback(SnapshotTriggeredMsg triggerData)
    {
        // show the panel game object
        ControlledObject?.SetActive(true);

        // if a previous coroutine is still active, kill it 
        if (null != m_DeactivateCoroutineRef)
            StopCoroutine(m_DeactivateCoroutineRef);

        // start a coroutine to hide the panel in x seconds
        m_DeactivateCoroutineRef = StartCoroutine(Deactivate());
    }

    //*********************************************************************
    /// <summary>
    /// Coroutine to hide game object after x seconds
    /// </summary>
    /// <returns>IEnumerator</returns>
    //*********************************************************************
    IEnumerator Deactivate()
    {
        // sleep for x seconds
        yield return new WaitForSeconds(DispalySpanSeconds);

        // remove the reference to this coroutine
        m_DeactivateCoroutineRef = null;

        // hide the panel game object
        ControlledObject?.SetActive(false);
    }
}

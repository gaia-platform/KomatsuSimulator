using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Sensors 
{
public class SensorReadOnlyAttribute : PropertyAttribute
{}

/// <summary>
/// Use this class to calculate frequency of some process by creating an instance and calling Tick() for counting.
/// </summary>
public class FrequencyCounter
{
    private int _tickCount;
    private float _startLapseTimeStamp = 0.0f;
    private float _timeLapse = 1.0f; // default to 1 second for calculating Hz
    private float _currentHz;
    private TimeSourceType _timeSourceType;

    /// <summary>
    /// used to specify if we need to use Time.time or Time.fixedTime
    /// </summary>
    public enum TimeSourceType { RENDER_UPDATE, FIXED_UPDATE}

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    public FrequencyCounter(TimeSourceType timeSourceType = TimeSourceType.FIXED_UPDATE)
    {
        this._timeSourceType = timeSourceType;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    public float Tick()
    {
        UpdateHZValue();
        _tickCount++;

        return _currentHz;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    public float GetHZ()
    {
        UpdateHZValue();

        return _currentHz;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void UpdateHZValue()
    {
        float currentTimeStamp = _timeSourceType == TimeSourceType.RENDER_UPDATE ? Time.time : Time.fixedTime;
        float elapsedTime = currentTimeStamp - _startLapseTimeStamp;
            
        if(elapsedTime >= _timeLapse)
        {
            // Current time lapse has ended. Calculate Hz and reset
            _currentHz = _tickCount / elapsedTime;

            _startLapseTimeStamp = currentTimeStamp;

            _tickCount = 0;

        }
    }
}


//************************************************************************
/// <summary>
/// Abstract class that defines a lot of Sensor core game object behavior.
///  
/// Update Rates:  Can setup update rate using the <c>SensorUpdateHzSynchronized()</c> and
/// Startup: Use <c>SensorStart()</c> instead standard Unity <c>Start()</c> method.
/// <c>SensorFixedUpdateHzSynchronized</c> methods.
/// </summary> 
//*************************************************************************
        
public abstract class SensorBase : MonoBehaviour 
{
    [Header("Sensor Base")]
    public float _updateRateHz;

    /// <summary>
    /// The update rate in Hz
    /// </summary>
    /// <value></value>
    public float UpdateRateHz 
    {
        get => _updateRateHz;
        set => _updateRateHz = value;
    }

    /// <summary>
    /// The delta update time in seconds.
    /// </summary>
    /// <value></value>
    public float UpdateTimeSeconds 
    {
        get 
        {
            if (UpdateRateHz == 0.0f) 
            {
                return 0.0f; // avoid divide by zero
            }

            return 1.0f / UpdateRateHz;
        }
    }

    public bool _debug;

    /// <summary>
    /// Debug flag dependent on implementation. May not do anything.
    /// </summary>
    /// <value></value>
    public bool IsDebug 
    {
        get => _debug;
        set => _debug = value;
    }

    private float _nextUpdateTime = 0.0f;
    public float NextUpdateTime 
    {
        get { return _nextUpdateTime; }
        set { _nextUpdateTime = value; }
    }
    private double _nextFixedUpdateTime = 0.0f;
    public double NextFixedUpdateTime 
    {
        get { return _nextFixedUpdateTime; }
        set { _nextFixedUpdateTime = value; }
    }

    private static float _nextUpdateTimeOffset = 0.0f;

    #region Threading
    // See: https://stackoverflow.com/questions/41330771/use-unity-api-from-another-thread-or-call-a-function-in-the-main-thread

    private List<System.Action> _actionQueuesUpdateFunc = new List<System.Action>();
    private List<System.Action> _actionCopiedQueueUpdateFunc = new List<System.Action>();

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    public System.Action ExecuteInUpdate 
    {
        set 
        {
            _actionQueuesUpdateFunc.Add(value);
        }
    }

    #endregion // Threading

#if UNITY_EDITOR

    private FrequencyCounter _updateHzCounter = new FrequencyCounter(FrequencyCounter.TimeSourceType.RENDER_UPDATE);
    private FrequencyCounter _fixedUpdateHzCounter = new FrequencyCounter(FrequencyCounter.TimeSourceType.FIXED_UPDATE);

    [SensorReadOnly]
    [SerializeField]
    private float _currentUpdateHz;
    [SensorReadOnly]
    [SerializeField]
    private float _currentFixedUpdateHz;

    //private bool _isFirstUpdate = true;

#endif

    // -------------------------------------------------------------------------------------------------------------------------
    // --------------- VIRTUAL METHODS FOR SENSOR IMPLEMENTATION ---------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------------------------


    //************************************************************************
    /// <summary>
    /// Start function for Sensor Objects
    /// </summary>    
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorStart() { }

    //************************************************************************
    /// <summary>
    /// Awake function for Sensor Objects
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorAwake() { }

    //************************************************************************
    /// <summary>
    /// OnEnable function for Sensor Objects
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorOnEnable() { }

    //************************************************************************
    /// <summary>
    /// OnValidate function for Sensor Obects
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorOnValidate() { }

    //************************************************************************
    /// <summary>
    /// Reset function for Sensor Objects
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorReset() { }

    //************************************************************************
    /// <summary>
    /// OnGUI is called for rendering and handling GUI events.
    /// OnGUI is the only function that can implement the "Immediate Mode" GUI (IMGUI) system for rendering and 
    /// handling GUI events. Your OnGUI implementation might be called several times per frame (one call per 
    /// event). For more information on GUI events see the Event reference. If the MonoBehaviour's enabled 
    /// property is set to false, OnGUI() will not be called.
    /// 
    /// If Debug property is set to false `SensorOnGUI` will not be called.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorOnGUI() { }

    //************************************************************************
    /// <summary>
    /// Destruction function for Sensor Objects
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorOnDestroy() { }

    //************************************************************************
    /// <summary>Update function for Sensor Objects. Will run inside this 
    /// Monobehavior's Update function.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorUpdate() { }

    //************************************************************************
    /// <summary>FixedUpdate function for Sensor Objects. Will run inside this 
    /// Monobehavior's Update function
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorFixedUpdate() { }

    //************************************************************************
    /// <summary>
    /// HZ synched function that runs inside Monobehavior's Update function. 
    /// It's guaranteed to use the set frequency HZ for this sensor.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorUpdateHzSynchronized() { }

    //************************************************************************
    /// <summary>
    /// HZ synched function that runs inside Monobehavior's FixedUpdate function. 
    /// It's guaranteed to use the set frequency HZ for this sensor.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected virtual void SensorFixedUpdateHzSynchronized() { }

    //************************************************************************
    /// <summary>
    /// Do not redefine this method in child classes, use SensorStart() instead.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void Start() 
    {
        SensorStart();

        /// Offset update time so we don't get big CPU spikes
        NextUpdateTime = _nextUpdateTimeOffset;
        NextFixedUpdateTime = _nextUpdateTimeOffset;
        _nextUpdateTimeOffset += 0.13f;
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void OnEnable() 
    {
        SensorOnEnable();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void OnValidate() 
    {
        SensorOnValidate();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void Reset() 
    {
        SensorReset();
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void Awake() 
    {
        SensorAwake();
    }

     //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************follow mew 
        
   private void OnDestroy() 
    {
        SensorOnDestroy();
    }


    //************************************************************************
    /// <summary>
    /// OnGUI is called for rendering and handling GUI events.
    /// OnGUI is the only function that can implement the "Immediate Mode" 
    /// GUI (IMGUI) system for rendering and handling GUI events. Your OnGUI 
    /// implementation might be called several times per frame (one call per 
    /// event). For more information on GUI events see the Event reference. If the MonoBehaviour's enabled 
    /// property is set to false, OnGUI() will not be called.
    /// </summary>    
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private void OnGUI() 
    {
        if (IsDebug == true) 
        {
            SensorOnGUI();
        }
    }

    //************************************************************************
    /// <summary>
    /// Update is called once per frame
    /// IMPORTANT: do not redefine this method in child classes, use SensorUpdate() instead.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    void Update() 
    {
        // run queued actions
        _actionCopiedQueueUpdateFunc.Clear();

        lock (_actionQueuesUpdateFunc) 
        {
            _actionCopiedQueueUpdateFunc.AddRange(_actionQueuesUpdateFunc);
            _actionQueuesUpdateFunc.Clear();
        }

        foreach (System.Action action in _actionCopiedQueueUpdateFunc) 
            action.Invoke();

        SensorUpdate();

        if (Time.time >= _nextUpdateTime) 
        {
            _nextUpdateTime = Time.time + (1.0f / _updateRateHz);

            SensorUpdateHzSynchronized();

#if UNITY_EDITOR
            _currentUpdateHz = _updateHzCounter.Tick();
#endif
        }
    }

    //************************************************************************
    /// <summary>
    /// IMPORTANT: do not redefine this method in child classes, use 
    /// SensorFixedUpdate() instead.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    void FixedUpdate() 
    {
        SensorFixedUpdate();

        if (Time.fixedUnscaledTimeAsDouble >= _nextFixedUpdateTime) 
        {
            _nextFixedUpdateTime = Time.fixedUnscaledTimeAsDouble + (1.0 / _updateRateHz);

            SensorFixedUpdateHzSynchronized();

#if UNITY_EDITOR
            _currentFixedUpdateHz = _fixedUpdateHzCounter.Tick();
#endif
        }
    }

    //************************************************************************
    /// <summary>Starts a coroutine that runs 'singleLoopLogic' using the HZ 
    /// update rate.
    /// </summary>
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    protected Coroutine StartHZEnforcedCoroutine(Action singleLoopLogic) 
    {
        return StartCoroutine(HZEnforcedCoroutine(singleLoopLogic));
    }

    //************************************************************************
    /// Description: 
    /// Arg:
    /// Returns: 
    //*************************************************************************
        
    private IEnumerator HZEnforcedCoroutine(Action singleLoopLogic) 
    {
        float nextUpdateTime = 0f;

        while (true) {
            // run single loop logic if it's time to do it
            if (Time.time >= nextUpdateTime) 
            {
                nextUpdateTime = Time.time + (1.0f / _updateRateHz);
                singleLoopLogic();
            }

            yield return new WaitForEndOfFrame();
        }

    }
}
}
//*********************************************************
//* Sources : https://github.com/fsstudio-team/ZeroSimROSUnity
//*********************************************************

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

namespace Sensors
{
    public struct RaycastJobBatch : System.IDisposable
    {
        private NativeArray<RaycastCommand> _raycastCommands;
        public NativeArray<RaycastCommand> RaycastCommands
        {
            get { return _raycastCommands; }
        }
        public void SetCommand(int index, Vector3 start, Vector3 direction, float maxDistance)
        {
            UnityEngine.Profiling.Profiler.BeginSample("RaycastJobBatch::SetCommand");
            _raycastCommands[index] = new RaycastCommand(start, direction, maxDistance);
            UnityEngine.Profiling.Profiler.EndSample();
        }
        private NativeArray<RaycastHit> _raycastHitresults;
        public NativeArray<RaycastHit> RaycastHitResults
        {
            get { return _raycastHitresults; }
        }
        private JobHandle _raycastBatchJobHandle;
        public JobHandle RaycastBatchJobHandle => _raycastBatchJobHandle;

        Allocator _allocator;

        public bool IsCompleted
        {
            get { return _raycastBatchJobHandle.IsCompleted; }
        }

        public int Length
        {
            get
            {
                int numCommands = _raycastCommands.Length;
                int numResults = _raycastHitresults.Length;
                return numCommands == numResults ? numCommands : -1;//-1 when array lengths are not equal
            }
            set
            {
                if (Length != value)
                {
                    Dispose();
                    _raycastCommands = new NativeArray<RaycastCommand>(value, _allocator);
                    _raycastHitresults = new NativeArray<RaycastHit>(value, _allocator, NativeArrayOptions.UninitializedMemory);
                }
            }
        }

        public RaycastJobBatch(int length, Allocator allocator)
        {
            _allocator = allocator;
            _raycastCommands = new NativeArray<RaycastCommand>(length, _allocator);
            _raycastHitresults = new NativeArray<RaycastHit>(length, _allocator, NativeArrayOptions.UninitializedMemory);
            _raycastBatchJobHandle = default(JobHandle);
        }

        public void Schedule(int minCommandsPerJob = 32, JobHandle dependsOn = default(JobHandle))
        {
#if DEBUG
            if (_raycastBatchJobHandle.IsCompleted == false)
                Debug.LogWarning("WARNING: Scheduling when job is not complete");
#endif
            _raycastBatchJobHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHitresults, minCommandsPerJob, dependsOn);
        }

        public void Complete()
        {
            _raycastBatchJobHandle.Complete();
            _raycastBatchJobHandle = default(JobHandle);
        }

        public void CopyResults(ref RaycastHit[] array)
        {
#if DEBUG
            if (_raycastBatchJobHandle.IsCompleted == false)
                Debug.LogWarning("WARNING: Copying results when job is not complete");
#endif
            if (array.Length != _raycastHitresults.Length)
                System.Array.Resize(ref array, _raycastHitresults.Length);

            _raycastHitresults.CopyTo(array);
        }

        public void Dispose()
        {
            Complete();
            if (_raycastCommands.IsCreated)
                _raycastCommands.Dispose();

            if (_raycastHitresults.IsCreated)
                _raycastHitresults.Dispose();
        }

    }

    // this job creates a batch of RaycastCommands we are going to use for
    // collision detection against the world. these can be sent to PhysX
    // as a batch that will be executed in a job, rather than us having to
    // call Physics.Raycast in a loop just on the main thread! 
    [BurstCompile(CompileSynchronously = true)]
    struct PrepareRaycastCommands : IJobParallelFor
    {
        public Vector3 Position;
        public Quaternion Orientation;
        public float Distance;
        public NativeArray<RaycastCommand> RaycastCommands;
        [ReadOnly]
        public NativeArray<Vector3> RaycastDirections;

        public void Execute(int i)
        {
            RaycastCommands[i] = new RaycastCommand(Position, Orientation * RaycastDirections[i], Distance);
        }
    };

    [BurstCompile(CompileSynchronously = true)]
    struct TransformHits : IJobParallelFor
    {
        [ReadOnly] public LidarSensor3D.ReferenceFrame ReferenceFrame;
        [ReadOnly] public Vector3 Position;

        [ReadOnly] public NativeArray<RaycastHit> RaycastHits;

        public bool CollectNormals;

        public NativeArray<Vector3> HitPositions;
        public NativeArray<Vector3> HitNormals;

        public void Execute(int i)
        {
            RaycastHit hit = RaycastHits[i];

            if (ReferenceFrame == LidarSensor3D.ReferenceFrame.LeftHanded_XRight_YUp_ZForward)
            {
                if(CollectNormals)
                    HitNormals[i] = hit.normal;

                HitPositions[i] = hit.point - Position;
            }
            else if (ReferenceFrame == LidarSensor3D.ReferenceFrame.RightHanded_XForward_YLeft_ZUp)
            {
                if(CollectNormals)
                    HitNormals[i] = new Vector3(hit.normal.z, -hit.normal.x, hit.normal.y);

                Vector3 pos = hit.point - Position;
                HitPositions[i] = new Vector3(pos.z, -pos.x, pos.y);
            }
        }
    };

    public class LidarSensor3D : Sensors.SensorBase
    {
        /// <summary>
        /// A generic LIDAR sensor.  
        /// </summary>

        public enum ReferenceFrame
        {
            RightHanded_XForward_YLeft_ZUp, // ROS Standard
            LeftHanded_XRight_YUp_ZForward // Unity Standard
        };

        public ReferenceFrame _referenceFrame = ReferenceFrame.RightHanded_XForward_YLeft_ZUp;

        [Header("FOV")]
        public float _verticalUpFovDegrees = 21.0f;
        public float _verticalDownFovDegrees = 74.0f;
        public float _horizontalFovDegrees = 360.0f;

        [Header("Resolution")]
        public float _verticalResolutionDegrees = 0.76f;
        public float _horizontalResolutionDegrees = 1.2f;
        public float _minRange = 0.0f;
        public float _maxRange = 20.0f;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// LidarSensor3D, string lidar id, int number of hits, Vector3 position, Vector3 normals
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<LidarSensor3D, string, NativeArray<Vector3>, NativeArray<Vector3>, Task> OnPublishDelegate { get; set; }

        // Property Accessors
        public float HorizontalFOVDegrees { get => _horizontalFovDegrees; }
        public float HorizontalResolutionDegrees { get => _horizontalResolutionDegrees; }
        public float MinRange { get => _minRange; }
        public float MaxRange { get => _maxRange; }

        private int _horizontalRayCount = -1;
        private int _verticalScanCount = -1;
        private int _totalRayCount = -1;

        private RaycastJobBatch _raycastBatchJob;
        private RaycastHit[] _rayCastHits;

        private NativeArray<Vector3> _hitPositions;
        private NativeArray<Vector3> _hitNormals;

        /// Pre-calculated rays
        NativeArray<Vector3> _rayDirections;

        ///
        JobHandle _transformHitsJobHandle = default(JobHandle);

        // Start is called before the first frame update
        protected override void SensorStart()
        //void Start() 
        {
            Debug.Log("INFO: LidarSensor3D::Start");

            _horizontalRayCount = Mathf.RoundToInt(_horizontalFovDegrees / _horizontalResolutionDegrees);
            _verticalScanCount = Mathf.RoundToInt((_verticalDownFovDegrees + _verticalUpFovDegrees) / _verticalResolutionDegrees);
            _totalRayCount = _horizontalRayCount * _verticalScanCount;

            _raycastBatchJob = new RaycastJobBatch(_totalRayCount, Allocator.TempJob);

            _rayCastHits = new RaycastHit[_totalRayCount];

            // build up the ray directions
            _rayDirections = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);
            Vector3 rayDirection = Quaternion.AngleAxis(_verticalUpFovDegrees, transform.right) * transform.forward;
            Quaternion horizontalRotationStep = Quaternion.AngleAxis(_horizontalResolutionDegrees, transform.up);
            Quaternion verticalRotationStep = Quaternion.AngleAxis(-_verticalResolutionDegrees, transform.right);
            int rayIndex = 0;

            for (int verticalStep = 0; verticalStep < _verticalScanCount; verticalStep++)
            {
                for (int horizontalStep = 0; horizontalStep < _horizontalRayCount; horizontalStep++)
                {
                    _rayDirections[rayIndex] = rayDirection;
                    rayIndex++;
                    rayDirection = horizontalRotationStep * rayDirection;
                }

                // BUGBUG:??? transform.right may change so do we need to have a rightDirection that moves with the rayDirection
                rayDirection = verticalRotationStep * rayDirection;
            }

            _hitPositions = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);
            _hitNormals = new NativeArray<Vector3>(_totalRayCount, Allocator.Persistent);
        }

        protected override void SensorOnDestroy()
        {
            base.SensorOnDestroy();

            _transformHitsJobHandle.Complete();
            _rayDirections.Dispose();
            _hitNormals.Dispose();
            _hitPositions.Dispose();
            _raycastBatchJob.Dispose();
        }

        protected override void SensorOnValidate()
        {
            base.SensorOnValidate();

            if (UpdateRateHz == 0)
                UpdateRateHz = 10;

            if (Name == "" || Name == null)
                Name = gameObject.name + "_" + Type;
        }

        protected override async void SensorFixedUpdateHzSynchronized()
        {
            UnityEngine.Profiling.Profiler.BeginSample("LidarSensor3D::UpdateHzSynchronized");

            if (_transformHitsJobHandle.IsCompleted == true)
            {
                _transformHitsJobHandle.Complete();

                if (OnPublishDelegate != null)
                {
                    UnityEngine.Profiling.Profiler.BeginSample("LidarSensor3D::UpdateHzSynchronized::Publish");
                    await OnPublishDelegate(this, Name, _hitPositions, _hitNormals);
                    UnityEngine.Profiling.Profiler.EndSample();
                }

                // Ref: https://github.com/LotteMakesStuff/SimplePhysicsDemo/blob/master/Assets/SimpleJobifiedPhysics.cs
                // create new raycast job
                _raycastBatchJob.Dispose();
                _raycastBatchJob = new RaycastJobBatch(_totalRayCount, Allocator.TempJob);
                // SetupRaycasts();

                var setupRaycastsJob = new PrepareRaycastCommands()
                {
                    Position = transform.position,
                    Orientation = transform.rotation,
                    Distance = _maxRange,
                    RaycastDirections = _rayDirections,
                    RaycastCommands = _raycastBatchJob.RaycastCommands
                };

                JobHandle setupRaycastsJobHandle = setupRaycastsJob.Schedule(_totalRayCount, 32);

                _raycastBatchJob.Schedule(32, setupRaycastsJobHandle);

                var transformHitJob = new TransformHits()
                {
                    ReferenceFrame = _referenceFrame,
                    Position = transform.position,
                    RaycastHits = _raycastBatchJob.RaycastHitResults,
                    HitPositions = _hitPositions,
                    HitNormals = _hitNormals,
                    CollectNormals = false
                };

                _transformHitsJobHandle = transformHitJob.Schedule(_totalRayCount, 32, _raycastBatchJob.RaycastBatchJobHandle);
            }

            UnityEngine.Profiling.Profiler.EndSample();
        }

        #region SerializationInterface
        public string Type
        {
            get { return "sensor.lidar3d"; }
        }

        [SerializeField] public string _name = "";
        public string Name
        {
            get
            {
                return _name;
            }
            private set
            {
                _name = value;
            }
        }
        #endregion

    }
}

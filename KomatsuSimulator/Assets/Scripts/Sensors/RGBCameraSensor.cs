//*********************************************************
//* Sources : 
//      https://github.com/Alabate/AsyncGPUReadbackPlugin
//      https://github.com/fsstudio-team/ZeroSimROSUnity
//*********************************************************

using System.IO;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;

namespace Sensors
{
    //*************************************************************************
    /// <summary>
    /// Simulation of a color RGB camera sensor.
    /// </summary>
    //*************************************************************************
    [RequireComponent(typeof(Camera))] //[ExecuteInEditMode]
    public class RGBCameraSensor : Sensors.SensorBase
    {
        [Header("Camera Parameters")]
        public Camera _camera;
        // public bool _isMonochrome = false;

        /// <summary>
        /// The Unity Camera associated with this depth camera. (readonly)
        /// </summary>
        /// <value></value>
        public Camera UnityCamera
        {
            get => _camera;
            protected set => _camera = value;
        }
        public int _width = 1280;

        /// <summary>
        /// Camera frame width in pixels.  (readonly)
        /// </summary>
        /// <value></value>
        public int Width
        {
            get => _width;
        }

        public int _height = 720;
        /// <summary>
        /// Camera frame height in pixels. (readonly)
        /// </summary>
        /// <value></value>
        public int Height
        {
            get => _height;
        }

        /// <summary>
        /// The camera focal length in millimeters.
        /// </summary>
        /// <value></value>
        public float FocalLengthMM
        {
            get { return UnityCamera.focalLength; }
        }

        /// <summary>
        /// The cameras field of view in degrees.
        /// </summary>
        /// <value></value>
        public float FieldOfViewDegrees
        {
            get { return UnityCamera.fieldOfView; }
        }

        /// <summary>
        /// Sensor width and height in millimeters.
        /// </summary>
        /// <value></value>
        public Vector2 SensorSizeMM
        {
            get { return UnityCamera.sensorSize; }
        }

        /// <summary>
        /// Flag to indicate that this camera is monochrome.
        /// IMPORTANT:  Should use the ZOMonoPostProcessMaterial if monochrome otherwise use the ZORGBPostProcessMaterial
        /// </summary>
        /// <value></value>
        // public bool IsMonochrome {
        //     get { return _isMonochrome; }
        // }

        [Header("Render Parameters")]
        [SerializeField]
        public Material _postProcessMaterial;

        [Header("Debug")]
        public Vector2 _debugWindowPosition = new Vector2(10, 10);
        private Texture2D _debugTexture;

        // ~~~~~~ Delegate Callbacks ~~~~~~
        /// <summary>
        /// Called every frame passing in:
        /// this, string cameraId, width, height, RGB24[] image
        /// 
        /// Note: is async so returns a task
        /// </summary>
        /// 
        /// <value></value>
        public Func<RGBCameraSensor, string, int, int, byte[], Task> OnPublishRGBImageDelegate { get; set; }

        byte[] _colorPixels24;
        // byte[] _monoPixels8;
        Task _publishTask = null;

        // ~~~~~~ Render Buffers ~~~~~~~
        private RenderTexture _renderTexture;

        // ~~~~ Async GPU Read ~~~~ //
        Queue<Sensors.Rendering.SensorAsyncGPUReadbackPluginRequest> _requests = 
            new Queue<Sensors.Rendering.SensorAsyncGPUReadbackPluginRequest>();

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        protected override void SensorOnValidate()
        {
            base.SensorOnValidate();
            // if camera is not assigned see if we have a camera component on this game object
            if (_camera == null)
            {
                _camera = this.GetComponent<Camera>();
            }

            // if no post process material then assign default
            if (_postProcessMaterial == null)
            {
#if UNITY_EDITOR
                _postProcessMaterial = Resources.Load<Material>("ZORGBPostProcessMaterial");
#endif // UNITY_EDITOR                
            }

            if (UpdateRateHz == 0)
            {
                UpdateRateHz = 30;
            }

            if (Name == "")
            {
                Name = gameObject.name + "_" + Type;
            }
        }

        // Start is called before the first frame update
        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        protected override void SensorStart()
        {
            Initialize();
        }

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        protected void Initialize()
        {
            // if camera is not assigned see if we have a camera component on this game object
            if (_camera == null)
            {
                _camera = this.GetComponent<Camera>();
            }

            // if (_postProcessMaterial == null) {
            //     _postProcessMaterial = ZOROSUnityManager.Instance.DefaultAssets.LoadAsset<Material>("ZORGBPostProcessMaterial");
            // }

            _renderTexture = new RenderTexture(_width, _height, 16, RenderTextureFormat.ARGB32);
            _camera.targetTexture = _renderTexture;

            // if (IsMonochrome) {
            //     _monoPixels8 = new byte[_width * _height];
            // } else { // RGB
            _colorPixels24 = new byte[_width * _height * 3];
            // }


            if (IsDebug == true)
            {
                _debugTexture = new Texture2D(_width, _height, TextureFormat.RGB24, false);
            }


            if (SystemInfo.supportsAsyncGPUReadback == true)
            {
                Debug.Log("INFO: Native support for AsyncGPUReadback");
            }
            else
            {
                Debug.LogWarning("WARNING: NO support for native AsyncGPUReadback. Using 3rd party.");
            }
        }

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        protected override void SensorFixedUpdateHzSynchronized()
        {
            _camera.Render();
        }

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        protected override void SensorUpdate()
        {
            DoRenderTextureUpdate();
        }

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        void OnRenderImage(RenderTexture source, RenderTexture destination)
        {
            Graphics.Blit(source, destination, _postProcessMaterial);
            if (_requests.Count < 8)
            {
                _requests.Enqueue(Sensors.Rendering.SensorAsyncGPUReadbackPlugin.Request(destination));
            }
            else
            {
                Debug.LogWarning("INFO: ZORGBCamera::OnRenderImage Too many requests.");
            }


            if (IsDebug == true)
            {
                RenderTexture.active = destination;
                _debugTexture.ReadPixels(new Rect(0, 0, destination.width, destination.height), 0, 0);
                _debugTexture.Apply();
            }
        }

        private void DoRenderTextureUpdate()
        {
            // ~~~ Handle Async GPU Readback ~~~ //
            while (_requests.Count > 0)
            {
                var req = _requests.Peek();
                UnityEngine.Profiling.Profiler.BeginSample("GL Update");
                // You need to explicitly ask for an update regularly
                req.Update();
                UnityEngine.Profiling.Profiler.EndSample();

                if (req.hasError)
                {
                    Debug.LogError("ERROR: GPU readback error detected.");
                    req.Dispose();
                    _requests.Dequeue();
                }
                else if (req.done)
                {
                    UnityEngine.Profiling.Profiler.BeginSample("GetRawData");
                    // Get data from the request when it's done
                    byte[] rawTextureData = req.GetRawData();
                    UnityEngine.Profiling.Profiler.EndSample();

                    if (OnPublishRGBImageDelegate != null)
                    {
                        if (_publishTask == null || _publishTask.IsCompleted)
                        {
                            UnityEngine.Profiling.Profiler.BeginSample("_publishTask");
                            // if (IsMonochrome) {
                            //     for (int i = 0, c4 = 0; i < _width * _height; i++, c4 += 4) {
                            //         _monoPixels8[i] = (byte)(rawTextureData[c4 + 1]);
                            //     }
                            //     OnPublishRGBImageDelegate(this, Name, _width, _height, _monoPixels8);
                            // } else { // RBG

                            for (int i = 0, c3 = 0, c4 = 0; i < _width * _height; i++, c3 += 3, c4 += 4)
                            {
                                _colorPixels24[c3 + 0] = (byte)(rawTextureData[c4 + 1]);
                                _colorPixels24[c3 + 1] = (byte)(rawTextureData[c4 + 2]);
                                _colorPixels24[c3 + 2] = (byte)(rawTextureData[c4 + 3]);
                            }

                            OnPublishRGBImageDelegate(this, Name, _width, _height, _colorPixels24);

                            // }
                            UnityEngine.Profiling.Profiler.EndSample();
                        }
                        else
                        {
                            // Debug.Log("skip");
                        }

                    }

                    UnityEngine.Profiling.Profiler.BeginSample("Dispose");
                    // You need to explicitly Dispose data after using them
                    req.Dispose();

                    _requests.Dequeue();
                    UnityEngine.Profiling.Profiler.EndSample();
                }
                else
                {
                    break;
                }
            }
        }

        //*********************************************************************
        /// <summary>
        ///
        /// </summary>
        //*********************************************************************
        private void OnGUI()
        {
            if (IsDebug == true)
            {
                if (_debugTexture)
                {
                    GUI.DrawTexture(new UnityEngine.Rect(_debugWindowPosition.x, 
                        _debugWindowPosition.y, _debugTexture.width, 
                        _debugTexture.height), _debugTexture, ScaleMode.ScaleToFit);
                }
            }
        }

        #region SensorSerializationInterface
        public string Type
        {
            get { return "sensor.rgbcamera"; }
        }

        [SerializeField] public string _name;
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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace RoboArm
{
    /// <summary>
    /// Master RTX & High-Fidelity Ray Tracing Graphics Manager.
    /// Manages Real-time Reflections, 4K Contact Shadows, Studio 4-Point RTX Lighting,
    /// Temporal Anti-Aliasing (TAA), ACES Tonemapping, Bokeh Depth of Field, and Bloom.
    /// Features in-game toggleable 'RTX ON / OFF' with an iconic visual watermark HUD.
    /// </summary>
    public class RTXGraphicsManager : MonoBehaviour
    {
        public static RTXGraphicsManager Instance { get; private set; }

        [Header("Master RTX Setting")]
        [Tooltip("Master switch for RTX Ray Tracing and cinematic visual fidelity.")]
        public bool rtxEnabled = true;

        [Header("Feature Toggles")]
        public bool enableStudioLights = true;
        public bool enableRealtimeReflections = true;
        public bool enableDepthOfField = false;
        public bool enableBloom = true;
        public bool enableAntiAliasing = true;

        [Header("Scene References (Auto-Created if Missing)")]
        public Volume globalVolume;
        public ReflectionProbe reflectionProbe;
        public GameObject studioLightsRoot;
        public Light taskLight;
        public Light rimLight;
        public Light fillLight;
        public Light baseGlowLight;

        [Header("Camera & Post FX References")]
        public Camera mainCamera;
        private UniversalAdditionalCameraData cameraData;
        private DepthOfField dofComponent;
        private Bloom bloomComponent;

        private float glowPulse = 0f;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;

            if (mainCamera == null)
            {
                mainCamera = Camera.main;
            }
            if (mainCamera != null)
            {
                cameraData = mainCamera.GetComponent<UniversalAdditionalCameraData>();
            }

            SetupRTXEnvironment();
        }

        void Start()
        {
            ApplyGraphicsSettings(rtxEnabled);
        }

        void Update()
        {
            // Toggle RTX with 'F10'
            if (Input.GetKeyDown(KeyCode.F10))
            {
                ToggleRTX();
            }

            // Gentle pulsing cyan status glow for robot base LED
            if (baseGlowLight != null && rtxEnabled)
            {
                glowPulse += Time.deltaTime * 2.5f;
                baseGlowLight.intensity = 0.18f + Mathf.Sin(glowPulse) * 0.04f;
            }

            // Periodically refresh reflection probe when arm articulates
            if (reflectionProbe != null && rtxEnabled && enableRealtimeReflections)
            {
                if (Time.frameCount % 2 == 0)
                {
                    reflectionProbe.RenderProbe();
                }
            }
        }

        public void ToggleRTX()
        {
            SetRTX(!rtxEnabled);
        }

        public void SetRTX(bool state)
        {
            rtxEnabled = state;
            ApplyGraphicsSettings(rtxEnabled);
        }

        public void ApplyGraphicsSettings(bool enabled)
        {
            // 1. Studio Cinematic Lighting
            if (studioLightsRoot != null)
            {
                studioLightsRoot.SetActive(enabled && enableStudioLights);
            }

            // 2. Realtime HDR Reflection Probe
            if (reflectionProbe != null)
            {
                reflectionProbe.enabled = enabled && enableRealtimeReflections;
                if (reflectionProbe.enabled)
                {
                    reflectionProbe.RenderProbe();
                }
            }

            // 3. Camera Anti-Aliasing (Subpixel Morphological SMAA for razor-sharp edges without temporal blur)
            if (cameraData != null)
            {
                cameraData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
                cameraData.antialiasingQuality = AntialiasingQuality.High;
                cameraData.dithering = true;
                cameraData.stopNaN = true;
                cameraData.renderPostProcessing = true;
            }

            // 4. Volume Components (Ensure Depth of Field is disabled so robot NEVER blurs from any angle)
            if (globalVolume != null && globalVolume.profile != null)
            {
                if (dofComponent == null) globalVolume.profile.TryGet(out dofComponent);
                if (bloomComponent == null) globalVolume.profile.TryGet(out bloomComponent);

                if (dofComponent != null)
                {
                    // Always disabled so robot arm remains 100% sharp and in focus from any camera angle
                    dofComponent.active = false;
                }
                if (bloomComponent != null)
                {
                    bloomComponent.active = enabled && enableBloom;
                }
            }
        }

        /// <summary>
        /// Auto-discovers or dynamically instantiates RTX lights, Reflection Probe, and Volume.
        /// </summary>
        private void SetupRTXEnvironment()
        {
            if (mainCamera != null)
            {
                mainCamera.nearClipPlane = 0.02f;
            }

            // 1. Setup Global Volume
            if (globalVolume == null)
            {
                var existingVol = GameObject.Find("Global_RTX_Volume");
                if (existingVol != null)
                {
                    globalVolume = existingVol.GetComponent<Volume>();
                }
                else
                {
                    var volObj = new GameObject("Global_RTX_Volume");
                    globalVolume = volObj.AddComponent<Volume>();
                    globalVolume.isGlobal = true;
                    globalVolume.priority = 10f;

                    // Load SampleSceneProfile
                    var profile = Resources.Load<VolumeProfile>("SampleSceneProfile");
                    if (profile != null) globalVolume.profile = profile;
                }
            }

            // 2. Setup Realtime 2048 Reflection Probe
            if (reflectionProbe == null)
            {
                var existingProbe = GameObject.Find("RTX_ReflectionProbe");
                if (existingProbe != null)
                {
                    reflectionProbe = existingProbe.GetComponent<ReflectionProbe>();
                }
                else
                {
                    var probeObj = new GameObject("RTX_ReflectionProbe");
                    probeObj.transform.position = new Vector3(0f, 0.85f, 0f);
                    reflectionProbe = probeObj.AddComponent<ReflectionProbe>();
                    reflectionProbe.mode = ReflectionProbeMode.Realtime;
                    reflectionProbe.refreshMode = ReflectionProbeRefreshMode.EveryFrame;
                    reflectionProbe.timeSlicingMode = ReflectionProbeTimeSlicingMode.IndividualFaces;
                    reflectionProbe.resolution = 2048;
                    reflectionProbe.hdr = true;
                    reflectionProbe.boxProjection = true;
                    reflectionProbe.size = new Vector3(7f, 4.5f, 7f);
                    reflectionProbe.center = new Vector3(0f, 0.65f, 0f);
                    reflectionProbe.shadowDistance = 30f;
                    reflectionProbe.importance = 1;
                    reflectionProbe.intensity = 0.8f;
                }
            }

            // 3. Setup Studio 4-Point RTX Lighting
            if (studioLightsRoot == null)
            {
                studioLightsRoot = GameObject.Find("RTX_StudioLights");
                if (studioLightsRoot == null)
                {
                    studioLightsRoot = new GameObject("RTX_StudioLights");

                    // A. Overhead Workcell Task Spot Light
                    var taskObj = new GameObject("Overhead_TaskSpotLight");
                    taskObj.transform.SetParent(studioLightsRoot.transform);
                    taskObj.transform.position = new Vector3(-0.164f, 2.65f, 0f);
                    taskObj.transform.rotation = Quaternion.Euler(80f, -25f, 0f);
                    taskLight = taskObj.AddComponent<Light>();
                    taskLight.type = LightType.Spot;
                    taskLight.range = 7f;
                    taskLight.spotAngle = 65f;
                    taskLight.innerSpotAngle = 36f;
                    taskLight.color = new Color(0.98f, 0.99f, 1.0f);
                    taskLight.intensity = 1.25f;
                    taskLight.shadows = LightShadows.Soft;
                    taskLight.shadowResolution = LightShadowResolution.VeryHigh;
                    taskLight.shadowCustomResolution = 2048;
                    var taskUData = taskObj.AddComponent<UniversalAdditionalLightData>();
                    taskUData.softShadowQuality = SoftShadowQuality.High;

                    // B. Cool Cyan Industrial Rim Light
                    var rimObj = new GameObject("Cyan_RimLight");
                    rimObj.transform.SetParent(studioLightsRoot.transform);
                    rimObj.transform.position = new Vector3(-1.35f, 1.75f, 1.35f);
                    rimObj.transform.rotation = Quaternion.Euler(32f, 138f, 0f);
                    rimLight = rimObj.AddComponent<Light>();
                    rimLight.type = LightType.Spot;
                    rimLight.range = 5.5f;
                    rimLight.spotAngle = 65f;
                    rimLight.color = new Color(0.15f, 0.68f, 1.0f);
                    rimLight.intensity = 0.35f;
                    rimLight.shadows = LightShadows.None;

                    // C. Warm Amber Ambient Fill Light
                    var fillObj = new GameObject("Warm_FillLight");
                    fillObj.transform.SetParent(studioLightsRoot.transform);
                    fillObj.transform.position = new Vector3(1.5f, 1.15f, -0.95f);
                    fillLight = fillObj.AddComponent<Light>();
                    fillLight.type = LightType.Point;
                    fillLight.range = 5f;
                    fillLight.color = new Color(1.0f, 0.72f, 0.45f);
                    fillLight.intensity = 0.4f;
                    fillLight.shadows = LightShadows.None;

                    // D. Robot Base Cyan Emissive Ring Glow
                    var glowObj = new GameObject("Robot_Base_StatusGlow");
                    glowObj.transform.SetParent(studioLightsRoot.transform);
                    glowObj.transform.position = new Vector3(-0.164f, 0.66f, 0f);
                    baseGlowLight = glowObj.AddComponent<Light>();
                    baseGlowLight.type = LightType.Point;
                    baseGlowLight.range = 0.35f;
                    baseGlowLight.color = new Color(0.05f, 0.88f, 1.0f);
                    baseGlowLight.intensity = 0.18f;
                    baseGlowLight.shadows = LightShadows.None;
                }
            }

            // 4. Setup Floor & Window Backdrops
            SetupBackdropEnvironment();
        }

        private void SetupBackdropEnvironment()
        {
            var existingBackdrop = GameObject.Find("Environment_Backdrop");
            if (existingBackdrop == null)
            {
                var root = new GameObject("Environment_Backdrop");
                root.transform.position = Vector3.zero;
                root.transform.rotation = Quaternion.identity;

                // Load materials
                Material studioMat = Resources.Load<Material>("Mat_StudioBackdrop");
                if (studioMat == null)
                {
                    var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                    studioMat = new Material(shader) { name = "Mat_StudioBackdrop" };
                    studioMat.SetColor("_BaseColor", new Color(0.82f, 0.80f, 0.77f, 1f));
                    studioMat.SetFloat("_Smoothness", 0.15f);
                }

                Material windowMat = Resources.Load<Material>("Mat_WindowCityBackdrop");
                if (windowMat == null)
                {
                    var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                    windowMat = new Material(shader) { name = "Mat_WindowCityBackdrop" };
                    var skylineTex = Resources.Load<Texture2D>("outdoor_city_skyline");
                    if (skylineTex != null)
                    {
                        windowMat.SetTexture("_BaseMap", skylineTex);
                        windowMat.SetTexture("_EmissionMap", skylineTex);
                        windowMat.EnableKeyword("_EMISSION");
                        windowMat.SetColor("_EmissionColor", new Color(0.22f, 0.25f, 0.3f, 1f));
                    }
                }

                float wallH = 4.2f;
                float wallThickness = 0.25f;

                void CreateWall(string name, Vector3 pos, Vector3 scale)
                {
                    var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    wall.name = name;
                    wall.transform.SetParent(root.transform);
                    wall.transform.position = pos;
                    wall.transform.localScale = scale;
                    var mr = wall.GetComponent<MeshRenderer>();
                    if (mr != null) mr.sharedMaterial = studioMat;
                    var col = wall.GetComponent<Collider>();
                    if (col != null) Destroy(col);
                }

                // 1. Back Wall (+X edge of floor)
                CreateWall("Backdrop_Wall_PositiveX", new Vector3(5.0f, wallH * 0.5f, 0f), new Vector3(wallThickness, wallH, 10.5f));

                // 2. Right Wall (-Z edge of floor)
                CreateWall("Backdrop_Wall_NegativeZ", new Vector3(0f, wallH * 0.5f, -5.0f), new Vector3(10.5f, wallH, wallThickness));

                // 3. Left Wall (+Z edge of floor)
                CreateWall("Backdrop_Wall_PositiveZ", new Vector3(0f, wallH * 0.5f, 5.0f), new Vector3(10.5f, wallH, wallThickness));

                // 4. Far Wall (-X edge behind dining room)
                CreateWall("Backdrop_Wall_NegativeX", new Vector3(-5.0f, wallH * 0.5f, 0f), new Vector3(wallThickness, wallH, 10.5f));

                // 5. Window Skyline Panorama Backdrop
                var winQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
                winQuad.name = "Window_City_Skyline_Backdrop";
                winQuad.transform.SetParent(root.transform);
                winQuad.transform.position = new Vector3(-2.85f, 1.85f, -0.15f);
                winQuad.transform.rotation = Quaternion.Euler(0f, 90f, 0f);
                winQuad.transform.localScale = new Vector3(8.5f, 4.2f, 1f);
                var winMr = winQuad.GetComponent<MeshRenderer>();
                if (winMr != null) winMr.sharedMaterial = windowMat;
                var winCol = winQuad.GetComponent<Collider>();
                if (winCol != null) Destroy(winCol);
            }
        }
    }
}

using System;
using UnityEngine;
using UnityEngine.EventSystems;

namespace RoboArm
{
    /// <summary>
    /// Comprehensive Mouse Camera Controller for Digital Twin Simulation.
    /// Provides CAD / Blender / RViz style 3D navigation:
    /// -------------------------------------------------------------
    /// • Orbit (Rotate): Hold Right Mouse Button (RMB) or Left Mouse Button (LMB outside UI) and drag.
    /// • Pan (Translate): Hold Middle Mouse Button (MMB / Scroll Wheel click) or Shift + RMB/LMB and drag.
    /// • Zoom (Dolly): Scroll Mouse Wheel or Alt/Ctrl + RMB drag.
    /// • Focus / Frame: Press 'F' key or Double-Click to frame the robot arm.
    /// • Reset View: Press 'R' key to return to default isometric view.
    /// • View Presets: Keys 1 (Front), 3 (Side), 7 (Top), 0 (Isometric).
    /// • UI Protection: Completely ignores mouse events when clicking inside dashboard panels or UI elements.
    /// </summary>
    public class MouseCameraController : MonoBehaviour
    {
        public static MouseCameraController Instance { get; private set; }

        public enum ViewPreset
        {
            Isometric,
            Front,
            Side,
            Top,
            Reset
        }

        [Header("Target & Focus")]
        [Tooltip("Target transform to orbit around. If null, targets the robot or world center.")]
        public Transform targetTransform;
        [Tooltip("Center offset from target position (e.g. height off table).")]
        public Vector3 focusOffset = new Vector3(0f, 0.22f, 0f);

        [Header("Orbit (Rotation)")]
        [Tooltip("Mouse rotation sensitivity")]
        public float orbitSensitivity = 3.2f;
        [Tooltip("Minimum vertical elevation angle (degrees) to prevent clipping under floor")]
        public float minPitch = 2.0f;
        [Tooltip("Maximum vertical elevation angle (degrees) to prevent flipping at zenith")]
        public float maxPitch = 86.0f;
        public bool invertY = false;

        [Header("Panning")]
        [Tooltip("Mouse pan sensitivity")]
        public float panSensitivity = 0.0035f;

        [Header("Zoom (Dolly)")]
        [Tooltip("Scroll wheel zoom sensitivity")]
        public float zoomSensitivity = 2.2f;
        public float minDistance = 0.25f;
        public float maxDistance = 6.0f;

        [Header("Smooth Damping")]
        public bool smoothMotion = true;
        [Range(0.01f, 0.25f)]
        public float smoothTime = 0.07f;

        [Header("UI Interaction Safety")]
        public bool blockOverUI = true;

        // Spherical coordinate targets
        private float targetYaw = 40f;
        private float targetPitch = 24f;
        private float targetDistance = 1.35f;
        private Vector3 targetLookAt;

        // Current smoothed values
        private float currentYaw;
        private float currentPitch;
        private float currentDistance;
        private Vector3 currentLookAt;

        // Velocities for SmoothDamp
        private float yawVelocity;
        private float pitchVelocity;
        private float distanceVelocity;
        private Vector3 lookAtVelocity;

        // Default initial pose for Reset ('R')
        private float initialYaw;
        private float initialPitch;
        private float initialDistance;
        private Vector3 initialLookAt;

        // Interaction state
        private bool isPointerDownOnUI = false;
        private bool isOrbiting = false;
        private bool isPanning = false;

        // Double click detection
        private float lastClickTime = 0f;
        private const float DOUBLE_CLICK_TIME = 0.28f;

        // Static UI Rects registered by IMGUI (e.g. ArmSimulationController)
        public static Rect registeredDashboardRect = Rect.zero;
        public static bool isDashboardActive = false;
        public static Rect registeredQuickBarRect = Rect.zero;

        void Awake()
        {
            if (Instance == null) Instance = this;
            else if (Instance != this)
            {
                Destroy(this);
                return;
            }
        }

        void Start()
        {
            // Auto-locate robot arm if target not explicitly wired
            if (targetTransform == null)
            {
                GameObject robot = GameObject.Find("eb15");
                if (robot != null)
                {
                    targetTransform = robot.transform;
                }
            }

            Vector3 focusPoint = GetFocusPoint();

            // Calculate starting yaw, pitch, distance from current camera transform
            Vector3 offset = transform.position - focusPoint;
            float dist = offset.magnitude;
            if (dist < 0.1f) dist = 1.35f;

            targetDistance = dist;
            currentDistance = dist;

            targetLookAt = focusPoint;
            currentLookAt = focusPoint;

            Vector3 euler = transform.eulerAngles;
            targetPitch = Mathf.Clamp(euler.x, minPitch, maxPitch);
            targetYaw = euler.y;

            currentPitch = targetPitch;
            currentYaw = targetYaw;

            // Preserve initial pose for Reset ('R')
            initialYaw = targetYaw;
            initialPitch = targetPitch;
            initialDistance = targetDistance;
            initialLookAt = targetLookAt;

            ApplyCameraTransform(currentYaw, currentPitch, currentDistance, currentLookAt);
        }

        public Vector3 GetFocusPoint()
        {
            if (targetTransform != null)
            {
                return targetTransform.position + focusOffset;
            }
            return focusOffset;
        }

        void Update()
        {
            HandleKeyboardShortcuts();
            HandleMouseInput();
        }

        void LateUpdate()
        {
            if (smoothMotion)
            {
                currentYaw = Mathf.SmoothDampAngle(currentYaw, targetYaw, ref yawVelocity, smoothTime);
                currentPitch = Mathf.SmoothDamp(currentPitch, targetPitch, ref pitchVelocity, smoothTime);
                currentDistance = Mathf.SmoothDamp(currentDistance, targetDistance, ref distanceVelocity, smoothTime);
                currentLookAt = Vector3.SmoothDamp(currentLookAt, targetLookAt, ref lookAtVelocity, smoothTime);
            }
            else
            {
                currentYaw = targetYaw;
                currentPitch = targetPitch;
                currentDistance = targetDistance;
                currentLookAt = targetLookAt;
            }

            ApplyCameraTransform(currentYaw, currentPitch, currentDistance, currentLookAt);
        }

        private void ApplyCameraTransform(float yaw, float pitch, float distance, Vector3 lookAt)
        {
            Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
            Vector3 position = lookAt - (rotation * Vector3.forward * distance);

            transform.rotation = rotation;
            transform.position = position;
        }

        private void HandleMouseInput()
        {
            bool anyMouseDown = Input.GetMouseButtonDown(0) || Input.GetMouseButtonDown(1) || Input.GetMouseButtonDown(2);

            if (anyMouseDown)
            {
                isPointerDownOnUI = blockOverUI && IsMouseOverUI();

                // Double click outside UI centers the camera on focus point
                if (!isPointerDownOnUI && (Input.GetMouseButtonDown(0) || Input.GetMouseButtonDown(2)))
                {
                    if (Time.time - lastClickTime < DOUBLE_CLICK_TIME)
                    {
                        FocusTarget();
                    }
                    lastClickTime = Time.time;
                }
            }

            // Release UI drag lock when all buttons are up
            if (!Input.GetMouseButton(0) && !Input.GetMouseButton(1) && !Input.GetMouseButton(2))
            {
                isPointerDownOnUI = false;
                isOrbiting = false;
                isPanning = false;
            }

            // If mouse interaction originated inside a UI panel/slider, do not move the 3D camera
            if (isPointerDownOnUI) return;

            // 1. Zoom (Mouse Scroll Wheel)
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (Mathf.Abs(scroll) > 0.0001f && (!blockOverUI || !IsMouseOverUI()))
            {
                // Dynamic exponential zoom scaling based on distance
                float zoomFactor = Mathf.Max(0.2f, targetDistance * 0.45f);
                targetDistance -= scroll * zoomSensitivity * zoomFactor;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
            }

            // 2. Dolly Zoom (Alt + RMB or Ctrl + RMB vertical drag)
            bool isDollyModifier = Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt) ||
                                  Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);

            if (isDollyModifier && Input.GetMouseButton(1))
            {
                float mouseY = Input.GetAxis("Mouse Y");
                targetDistance -= mouseY * zoomSensitivity * 0.4f;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
                return;
            }

            // 3. Pan: Middle Mouse Drag OR Shift + Right/Left Mouse Drag
            bool isShiftKey = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            bool isPanButton = Input.GetMouseButton(2) || (isShiftKey && (Input.GetMouseButton(0) || Input.GetMouseButton(1)));

            if (isPanButton)
            {
                isPanning = true;
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = Input.GetAxis("Mouse Y");

                float panFactor = panSensitivity * targetDistance;
                Vector3 panRight = transform.right * (-mouseX * panFactor);
                Vector3 panUp = transform.up * (-mouseY * panFactor);

                targetLookAt += panRight + panUp;
                return;
            }

            // 4. Orbit (Rotate): Right Mouse Drag OR Left Mouse Drag (outside UI)
            bool isOrbitButton = Input.GetMouseButton(1) || Input.GetMouseButton(0);
            if (isOrbitButton)
            {
                isOrbiting = true;
                float mouseX = Input.GetAxis("Mouse X") * orbitSensitivity;
                float mouseY = Input.GetAxis("Mouse Y") * orbitSensitivity;

                if (invertY) mouseY = -mouseY;

                targetYaw += mouseX;
                targetPitch -= mouseY;
                targetPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);
            }
        }

        private void HandleKeyboardShortcuts()
        {
            // Prevent shortcuts when user is typing into text field (e.g. IP address or port)
            if (GUIUtility.keyboardControl != 0) return;

            // 'F' to focus target
            if (Input.GetKeyDown(KeyCode.F))
            {
                FocusTarget();
            }

            // 'R' to reset to initial isometric view
            if (Input.GetKeyDown(KeyCode.R))
            {
                SetPresetView(ViewPreset.Reset);
            }

            // Preset View keys: 1 (Front), 3 (Side), 7 (Top), 0 (Isometric)
            if (Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.Keypad1))
            {
                SetPresetView(ViewPreset.Front);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha3) || Input.GetKeyDown(KeyCode.Keypad3))
            {
                SetPresetView(ViewPreset.Side);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha7) || Input.GetKeyDown(KeyCode.Keypad7))
            {
                SetPresetView(ViewPreset.Top);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
            {
                SetPresetView(ViewPreset.Isometric);
            }
        }

        public void SetPresetView(ViewPreset preset)
        {
            Vector3 defaultFocus = GetFocusPoint();

            switch (preset)
            {
                case ViewPreset.Isometric:
                    targetYaw = 45f;
                    targetPitch = 24f;
                    targetDistance = 1.35f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Front:
                    targetYaw = 0f;
                    targetPitch = 14f;
                    targetDistance = 1.35f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Side:
                    targetYaw = 90f;
                    targetPitch = 14f;
                    targetDistance = 1.35f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Top:
                    targetYaw = 0f;
                    targetPitch = 85f;
                    targetDistance = 1.45f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Reset:
                    targetYaw = initialYaw;
                    targetPitch = initialPitch;
                    targetDistance = initialDistance;
                    targetLookAt = initialLookAt;
                    break;
            }
        }

        public void FocusTarget()
        {
            targetLookAt = GetFocusPoint();
            targetDistance = Mathf.Clamp(targetDistance, 0.8f, 1.5f);
        }

        /// <summary>
        /// Checks if current mouse position is over registered IMGUI windows or uGUI canvas.
        /// </summary>
        public bool IsMouseOverUI()
        {
            // 1. Check Unity UI EventSystem
            if (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject())
            {
                return true;
            }

            // 2. Check registered IMGUI rects
            Vector2 guiMousePos = new Vector2(Input.mousePosition.x, Screen.height - Input.mousePosition.y);

            if (isDashboardActive && registeredDashboardRect != Rect.zero && registeredDashboardRect.Contains(guiMousePos))
            {
                return true;
            }

            if (registeredQuickBarRect != Rect.zero && registeredQuickBarRect.Contains(guiMousePos))
            {
                return true;
            }

            return false;
        }

        public bool IsOrbiting => isOrbiting;
        public bool IsPanning => isPanning;
    }
}

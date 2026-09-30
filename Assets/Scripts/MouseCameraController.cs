using System;
using UnityEngine;
using UnityEngine.EventSystems;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

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
        public Vector3 focusOffset = new Vector3(0f, 0.28f, 0f);

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
        public float maxDistance = 2.4f;

        [Header("Smooth Damping")]
        public bool smoothMotion = true;
        [Range(0.01f, 0.25f)]
        public float smoothTime = 0.07f;

        [Header("Room Boundary Clamping")]
        [Tooltip("Constrain camera movement and panning strictly to the inside of the room")]
        public bool clampInsideRoom = true;
        public Vector3 roomMin = new Vector3(-1.15f, 0.35f, -1.60f);
        public Vector3 roomMax = new Vector3(1.15f, 2.30f, 1.15f);
        public Vector3 lookAtMin = new Vector3(-1.00f, 0.35f, -1.45f);
        public Vector3 lookAtMax = new Vector3(1.00f, 2.20f, 1.00f);

        [Header("Keyboard Movement (Inside Room)")]
        [Tooltip("Camera move speed using keyboard (WASD/QE/Arrows) within room boundaries")]
        public float keyboardMoveSpeed = 1.35f;
        public float sprintMultiplier = 2.0f;
        public bool keyboardMovementEnabled = true;

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

            // Ensure our Main Camera is the dominant rendering camera
            Camera cam = GetComponent<Camera>();
            if (cam != null)
            {
                cam.depth = 100f;
                cam.targetDisplay = 0;
                cam.enabled = true;
                gameObject.tag = "MainCamera";
            }

            // Disable any extra/imported cameras (e.g. from FBX) so our camera always renders
            Camera[] allCams = FindObjectsByType<Camera>();
            foreach (var c in allCams)
            {
                if (c != cam && c.gameObject != gameObject)
                {
                    Debug.Log($"[MouseCameraController] Disabling extra camera: '{c.gameObject.name}'");
                    c.enabled = false;
                }
            }
        }

        void Start()
        {
            // Ensure camera never clips close-up angles of the robot arm
            Camera cam = GetComponent<Camera>();
            if (cam != null)
            {
                cam.nearClipPlane = 0.02f;
                cam.fieldOfView = 48f;
            }

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
            if (clampInsideRoom)
            {
                focusPoint.x = Mathf.Clamp(focusPoint.x, lookAtMin.x, lookAtMax.x);
                focusPoint.y = Mathf.Clamp(focusPoint.y, lookAtMin.y, lookAtMax.y);
                focusPoint.z = Mathf.Clamp(focusPoint.z, lookAtMin.z, lookAtMax.z);
            }

            Vector3 startPos = transform.position;
            if (clampInsideRoom)
            {
                startPos.x = Mathf.Clamp(startPos.x, roomMin.x, roomMax.x);
                startPos.y = Mathf.Clamp(startPos.y, roomMin.y, roomMax.y);
                startPos.z = Mathf.Clamp(startPos.z, roomMin.z, roomMax.z);
            }

            // Calculate starting yaw, pitch, distance from current camera transform
            Vector3 offset = startPos - focusPoint;
            float dist = offset.magnitude;
            if (dist < 0.1f) dist = 1.35f;
            dist = Mathf.Clamp(dist, minDistance, maxDistance);

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
            HandleKeyboardNavigation();
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
            if (clampInsideRoom)
            {
                lookAt.x = Mathf.Clamp(lookAt.x, lookAtMin.x, lookAtMax.x);
                lookAt.y = Mathf.Clamp(lookAt.y, lookAtMin.y, lookAtMax.y);
                lookAt.z = Mathf.Clamp(lookAt.z, lookAtMin.z, lookAtMax.z);
            }

            Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
            Vector3 position = lookAt - (rotation * Vector3.forward * distance);

            if (clampInsideRoom)
            {
                position.x = Mathf.Clamp(position.x, roomMin.x, roomMax.x);
                position.y = Mathf.Clamp(position.y, roomMin.y, roomMax.y);
                position.z = Mathf.Clamp(position.z, roomMin.z, roomMax.z);
            }

            transform.rotation = rotation;
            transform.position = position;
        }

        private void HandleMouseInput()
        {
            bool anyMouseDown = GetMouseButtonDownState(0) || GetMouseButtonDownState(1) || GetMouseButtonDownState(2);

            if (anyMouseDown)
            {
                isPointerDownOnUI = blockOverUI && IsMouseOverUI();

                // Double click outside UI centers the camera on focus point
                if (!isPointerDownOnUI && (GetMouseButtonDownState(0) || GetMouseButtonDownState(2)))
                {
                    if (Time.time - lastClickTime < DOUBLE_CLICK_TIME)
                    {
                        FocusTarget();
                    }
                    lastClickTime = Time.time;
                }
            }

            // Release UI drag lock when all buttons are up
            if (!GetMouseButtonState(0) && !GetMouseButtonState(1) && !GetMouseButtonState(2))
            {
                isPointerDownOnUI = false;
                isOrbiting = false;
                isPanning = false;
            }

            // If mouse interaction originated inside a UI panel/slider, do not move the 3D camera
            if (isPointerDownOnUI) return;

            // 1. Zoom (Mouse Scroll Wheel)
            float scroll = GetScrollWheelDelta();
            if (Mathf.Abs(scroll) > 0.0001f && (!blockOverUI || !IsMouseOverUI()))
            {
                // Dynamic exponential zoom scaling based on distance
                float zoomFactor = Mathf.Max(0.2f, targetDistance * 0.45f);
                targetDistance -= scroll * zoomSensitivity * zoomFactor;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
            }

            // 2. Dolly Zoom (Alt + RMB or Ctrl + RMB vertical drag)
            bool isDollyModifier = IsAltOrCtrlDown();

            if (isDollyModifier && GetMouseButtonState(1))
            {
                float mouseY = GetMouseDeltaY();
                targetDistance -= mouseY * zoomSensitivity * 0.4f;
                targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
                return;
            }

            // 3. Pan: Middle Mouse Drag OR Shift + Right/Left Mouse Drag
            bool isShiftKey = IsShiftDown();
            bool isPanButton = GetMouseButtonState(2) || (isShiftKey && (GetMouseButtonState(0) || GetMouseButtonState(1)));

            if (isPanButton)
            {
                isPanning = true;
                float mouseX = GetMouseDeltaX();
                float mouseY = GetMouseDeltaY();

                float panFactor = panSensitivity * targetDistance;
                Vector3 panRight = transform.right * (-mouseX * panFactor);
                Vector3 panUp = transform.up * (-mouseY * panFactor);

                targetLookAt += panRight + panUp;

                if (clampInsideRoom)
                {
                    targetLookAt.x = Mathf.Clamp(targetLookAt.x, lookAtMin.x, lookAtMax.x);
                    targetLookAt.y = Mathf.Clamp(targetLookAt.y, lookAtMin.y, lookAtMax.y);
                    targetLookAt.z = Mathf.Clamp(targetLookAt.z, lookAtMin.z, lookAtMax.z);
                }
                return;
            }

            // 4. Orbit (Rotate): Right Mouse Drag OR Left Mouse Drag (outside UI)
            bool isOrbitButton = GetMouseButtonState(1) || GetMouseButtonState(0);
            if (isOrbitButton)
            {
                isOrbiting = true;
                float mouseX = GetMouseDeltaX() * orbitSensitivity;
                float mouseY = GetMouseDeltaY() * orbitSensitivity;

                if (invertY) mouseY = -mouseY;

                targetYaw += mouseX;
                targetPitch -= mouseY;
                targetPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);
            }
        }

        private void HandleKeyboardNavigation()
        {
            if (!keyboardMovementEnabled) return;
            // Prevent shortcuts when user is typing into text field (e.g. IP address or port)
            if (GUIUtility.keyboardControl != 0) return;

            Vector3 inputDir = Vector3.zero;

            // W / S or Up / Down -> Forward / Back
            if (IsKeyHeld(KeyCode.W) || IsKeyHeld(KeyCode.UpArrow)) inputDir.z += 1f;
            if (IsKeyHeld(KeyCode.S) || IsKeyHeld(KeyCode.DownArrow)) inputDir.z -= 1f;

            // A / D or Left / Right -> Strafe Left / Right
            if (IsKeyHeld(KeyCode.D) || IsKeyHeld(KeyCode.RightArrow)) inputDir.x += 1f;
            if (IsKeyHeld(KeyCode.A) || IsKeyHeld(KeyCode.LeftArrow)) inputDir.x -= 1f;

            // E / Space -> Elevate Up, Q / C / Ctrl -> Elevate Down
            if (IsKeyHeld(KeyCode.E) || IsKeyHeld(KeyCode.Space)) inputDir.y += 1f;
            if (IsKeyHeld(KeyCode.Q) || IsKeyHeld(KeyCode.LeftControl) || IsKeyHeld(KeyCode.C)) inputDir.y -= 1f;

            if (inputDir.sqrMagnitude > 0.001f)
            {
                Vector3 fwd = transform.forward;
                fwd.y = 0f;
                if (fwd.sqrMagnitude > 0.001f) fwd.Normalize(); else fwd = Vector3.forward;

                Vector3 right = transform.right;
                right.y = 0f;
                if (right.sqrMagnitude > 0.001f) right.Normalize(); else right = Vector3.right;

                Vector3 moveDir = (fwd * inputDir.z) + (right * inputDir.x) + (Vector3.up * inputDir.y);
                moveDir.Normalize();

                float speed = keyboardMoveSpeed;
                if (IsShiftDown())
                {
                    speed *= sprintMultiplier;
                }

                targetLookAt += moveDir * (speed * Time.deltaTime);

                if (clampInsideRoom)
                {
                    targetLookAt.x = Mathf.Clamp(targetLookAt.x, lookAtMin.x, lookAtMax.x);
                    targetLookAt.y = Mathf.Clamp(targetLookAt.y, lookAtMin.y, lookAtMax.y);
                    targetLookAt.z = Mathf.Clamp(targetLookAt.z, lookAtMin.z, lookAtMax.z);
                }
            }

            // 'F' or 'R' to center / focus robot arm
            if (IsKeyDown(KeyCode.F) || IsKeyDown(KeyCode.R))
            {
                FocusTarget();
            }

            // Preset View keys: 1 (Front), 3 (Side), 7 (Top), 0 (Isometric)
            if (IsKeyDown(KeyCode.Alpha1))
            {
                SetPresetView(ViewPreset.Front);
            }
            else if (IsKeyDown(KeyCode.Alpha3))
            {
                SetPresetView(ViewPreset.Side);
            }
            else if (IsKeyDown(KeyCode.Alpha7))
            {
                SetPresetView(ViewPreset.Top);
            }
            else if (IsKeyDown(KeyCode.Alpha0))
            {
                SetPresetView(ViewPreset.Isometric);
            }
        }

        public void MoveForward(float multiplier = 1f) => MoveRelative(Vector3.forward * multiplier);
        public void MoveBackward(float multiplier = 1f) => MoveRelative(-Vector3.forward * multiplier);
        public void MoveLeft(float multiplier = 1f) => MoveRelative(-Vector3.right * multiplier);
        public void MoveRight(float multiplier = 1f) => MoveRelative(Vector3.right * multiplier);
        public void MoveUp(float multiplier = 1f) => MoveRelative(Vector3.up * multiplier);
        public void MoveDown(float multiplier = 1f) => MoveRelative(-Vector3.up * multiplier);

        public void MoveRelative(Vector3 localDir)
        {
            Vector3 fwd = transform.forward;
            fwd.y = 0f;
            if (fwd.sqrMagnitude > 0.001f) fwd.Normalize(); else fwd = Vector3.forward;

            Vector3 right = transform.right;
            right.y = 0f;
            if (right.sqrMagnitude > 0.001f) right.Normalize(); else right = Vector3.right;

            Vector3 moveDir = (fwd * localDir.z) + (right * localDir.x) + (Vector3.up * localDir.y);
            targetLookAt += moveDir * (keyboardMoveSpeed * 0.15f);

            if (clampInsideRoom)
            {
                targetLookAt.x = Mathf.Clamp(targetLookAt.x, lookAtMin.x, lookAtMax.x);
                targetLookAt.y = Mathf.Clamp(targetLookAt.y, lookAtMin.y, lookAtMax.y);
                targetLookAt.z = Mathf.Clamp(targetLookAt.z, lookAtMin.z, lookAtMax.z);
            }
        }

        public void SetPresetView(ViewPreset preset)
        {
            Vector3 defaultFocus = GetFocusPoint();

            switch (preset)
            {
                case ViewPreset.Isometric:
                    targetYaw = 42f;
                    targetPitch = 20f;
                    targetDistance = 1.18f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Front:
                    targetYaw = 0f;
                    targetPitch = 12f;
                    targetDistance = 1.15f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Side:
                    targetYaw = 90f;
                    targetPitch = 12f;
                    targetDistance = 1.15f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Top:
                    targetYaw = 0f;
                    targetPitch = 85f;
                    targetDistance = 1.25f;
                    targetLookAt = defaultFocus;
                    break;

                case ViewPreset.Reset:
                    targetYaw = 42f;
                    targetPitch = 20f;
                    targetDistance = 1.18f;
                    targetLookAt = defaultFocus;
                    break;
            }
        }

        public void FocusTarget()
        {
            targetLookAt = GetFocusPoint();
            targetDistance = 1.18f;
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
            Vector2 mousePos = GetCurrentMousePosition();
            Vector2 guiMousePos = new Vector2(mousePos.x, Screen.height - mousePos.y);

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

        // ==================== Safe Dual-Input Helpers ====================
        public static Vector2 GetCurrentMousePosition()
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null) return Mouse.current.position.ReadValue();
#endif
            try { return Input.mousePosition; } catch { return Vector2.zero; }
        }

        public static bool GetMouseButtonState(int button)
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null)
            {
                if (button == 0) return Mouse.current.leftButton.isPressed;
                if (button == 1) return Mouse.current.rightButton.isPressed;
                if (button == 2) return Mouse.current.middleButton.isPressed;
            }
#endif
            try { return Input.GetMouseButton(button); } catch { return false; }
        }

        public static bool GetMouseButtonDownState(int button)
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null)
            {
                if (button == 0) return Mouse.current.leftButton.wasPressedThisFrame;
                if (button == 1) return Mouse.current.rightButton.wasPressedThisFrame;
                if (button == 2) return Mouse.current.middleButton.wasPressedThisFrame;
            }
#endif
            try { return Input.GetMouseButtonDown(button); } catch { return false; }
        }

        public static float GetMouseDeltaX()
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null)
            {
                return Mouse.current.delta.ReadValue().x * 0.1f;
            }
#endif
            try { return Input.GetAxis("Mouse X"); } catch { return 0f; }
        }

        public static float GetMouseDeltaY()
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null)
            {
                return Mouse.current.delta.ReadValue().y * 0.1f;
            }
#endif
            try { return Input.GetAxis("Mouse Y"); } catch { return 0f; }
        }

        public static float GetScrollWheelDelta()
        {
#if ENABLE_INPUT_SYSTEM
            if (Mouse.current != null)
            {
                float sy = Mouse.current.scroll.ReadValue().y;
                if (Mathf.Abs(sy) > 0.01f)
                    return Mathf.Sign(sy) * 0.15f;
            }
#endif
            try { return Input.GetAxis("Mouse ScrollWheel"); } catch { return 0f; }
        }

        public static bool IsShiftDown()
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null)
            {
                return Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed;
            }
#endif
            try { return Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift); } catch { return false; }
        }

        public static bool IsAltOrCtrlDown()
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null)
            {
                return Keyboard.current.leftAltKey.isPressed || Keyboard.current.rightAltKey.isPressed ||
                       Keyboard.current.leftCtrlKey.isPressed || Keyboard.current.rightCtrlKey.isPressed;
            }
#endif
            try
            {
                return Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt) ||
                       Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
            }
            catch { return false; }
        }

        public static bool IsKeyDown(KeyCode key)
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null)
            {
                switch (key)
                {
                    case KeyCode.F: return Keyboard.current.fKey.wasPressedThisFrame;
                    case KeyCode.R: return Keyboard.current.rKey.wasPressedThisFrame;
                    case KeyCode.Alpha1: case KeyCode.Keypad1: return Keyboard.current.digit1Key.wasPressedThisFrame || Keyboard.current.numpad1Key.wasPressedThisFrame;
                    case KeyCode.Alpha3: case KeyCode.Keypad3: return Keyboard.current.digit3Key.wasPressedThisFrame || Keyboard.current.numpad3Key.wasPressedThisFrame;
                    case KeyCode.Alpha7: case KeyCode.Keypad7: return Keyboard.current.digit7Key.wasPressedThisFrame || Keyboard.current.numpad7Key.wasPressedThisFrame;
                    case KeyCode.Alpha0: case KeyCode.Keypad0: return Keyboard.current.digit0Key.wasPressedThisFrame || Keyboard.current.numpad0Key.wasPressedThisFrame;
                }
            }
#endif
            try { return Input.GetKeyDown(key); } catch { return false; }
        }

        public static bool IsKeyHeld(KeyCode key)
        {
#if ENABLE_INPUT_SYSTEM
            if (Keyboard.current != null)
            {
                switch (key)
                {
                    case KeyCode.W: return Keyboard.current.wKey.isPressed;
                    case KeyCode.S: return Keyboard.current.sKey.isPressed;
                    case KeyCode.A: return Keyboard.current.aKey.isPressed;
                    case KeyCode.D: return Keyboard.current.dKey.isPressed;
                    case KeyCode.Q: return Keyboard.current.qKey.isPressed;
                    case KeyCode.E: return Keyboard.current.eKey.isPressed;
                    case KeyCode.Space: return Keyboard.current.spaceKey.isPressed;
                    case KeyCode.C: return Keyboard.current.cKey.isPressed;
                    case KeyCode.R: return Keyboard.current.rKey.isPressed;
                    case KeyCode.F: return Keyboard.current.fKey.isPressed;
                    case KeyCode.UpArrow: return Keyboard.current.upArrowKey.isPressed;
                    case KeyCode.DownArrow: return Keyboard.current.downArrowKey.isPressed;
                    case KeyCode.LeftArrow: return Keyboard.current.leftArrowKey.isPressed;
                    case KeyCode.RightArrow: return Keyboard.current.rightArrowKey.isPressed;
                    case KeyCode.LeftShift: return Keyboard.current.leftShiftKey.isPressed;
                    case KeyCode.RightShift: return Keyboard.current.rightShiftKey.isPressed;
                    case KeyCode.LeftControl: return Keyboard.current.leftCtrlKey.isPressed;
                    case KeyCode.RightControl: return Keyboard.current.rightCtrlKey.isPressed;
                }
            }
#endif
            try { return Input.GetKey(key); } catch { return false; }
        }

        public bool IsOrbiting => isOrbiting;
        public bool IsPanning => isPanning;
    }
}

using System;
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;
#endif
using UnityEngine.XR;

namespace RoboArm
{
    /// <summary>
    /// Meta Quest 2 (Oculus Touch) VR Controller Driver for EB15 Robotic Arm.
    /// Bridges Quest 2 controller gyroscopic values (IMU Roll/Pitch/Yaw & Angular Velocity)
    /// and buttons (Trigger, Grip, A/B, Thumbstick) to robot arm functions and commands.
    /// 
    /// Features:
    /// • Dual-Stack Input: UnityEngine.XR.InputDevices (OpenXR/Oculus) + Unity Input System.
    /// • Deadman Clutch (Grip Button): Hold to teleoperate, release to lock pose in place.
    /// • Gyroscopic Angular Teleoperation: Real-time tilt/twist mapping to Base, Shoulder, Elbow & Wrist.
    /// • Closed-Form Cartesian IK Mode: 6DoF position tracking + gyro orientation.
    /// • Analog Trigger Gripper: Smooth pressure-sensitive clamp with haptic feedback.
    /// • Command Hotkeys: Primary Button (A) cycles presets / holds for Home; Secondary (B) toggles Ghost / ROS Stream.
    /// • Editor Desktop Emulation: Test gyro & buttons without VR headset via mouse & keyboard.
    /// </summary>
    [RequireComponent(typeof(ArmSimulationController))]
    public class Quest2RoboArmController : MonoBehaviour
    {
        public enum ControllerHand
        {
            RightHand,
            LeftHand,
            AutoDetect
        }

        public enum TeleoperationMode
        {
            DirectGyroMimic,   // Controller yaw/pitch/roll directly controls joints
            CartesianIK,       // Controller 3D position translates end effector, gyro points wrist
            AngularRateJog     // Tilting past deadzone spins joints at proportional speed
        }

        [Header("Target Digital Twin Master")]
        public ArmSimulationController armController;
        public GhostArmPreview ghostArm;
        public RosRoboArmBridge rosBridge;

        [Header("Controller Configuration")]
        [Tooltip("Handedness for the primary teleoperation controller.")]
        public ControllerHand controllerHand = ControllerHand.RightHand;

        [Tooltip("Teleoperation tracking paradigm.")]
        public TeleoperationMode controlMode = TeleoperationMode.DirectGyroMimic;

        [Tooltip("Require holding the Grip button (Deadman Clutch) to move the robot.")]
        public bool requireClutchToMove = true;

        [Tooltip("Relative tracking zeros gyro orientation when clutch is engaged, preventing wrist strain.")]
        public bool relativeTracking = true;

        [Header("Gyroscopic Sensitivities")]
        [Range(0.2f, 3.0f)] public float yawSensitivity = 1.0f;     // Joint 1 (Base Pan)
        [Range(0.2f, 3.0f)] public float pitchSensitivity = 1.0f;   // Joint 2 (Shoulder Pitch)
        [Range(0.0f, 2.0f)] public float elbowCoupling = 0.8f;      // Joint 3 (Elbow response to pitch)
        [Range(0.2f, 3.0f)] public float rollSensitivity = 1.0f;    // Joint 4 (Wrist Roll)

        [Header("Inversions & Deadzones")]
        public bool invertYaw = false;
        public bool invertPitch = false;
        public bool invertRoll = false;
        public bool invertGripper = false;
        [Range(0.0f, 15.0f)] public float gyroDeadzoneDeg = 1.5f;

        [Header("Motion Smoothing")]
        [Tooltip("Smooth time for joint interpolations in seconds.")]
        [Range(0.01f, 0.3f)] public float smoothingTime = 0.05f;

        [Header("Cartesian IK Parameters")]
        public float ikReachScale = 1.2f;
        public Vector3 ikWorkspaceCenter = new Vector3(0f, 0.25f, 0.35f);

        [Header("Haptic Feedback")]
        public bool enableHaptics = true;
        [Range(0.1f, 1.0f)] public float hapticIntensity = 0.45f;

        [Header("Desktop / Editor Emulation")]
        [Tooltip("Allow keyboard and mouse to emulate Quest 2 gyro and buttons when no headset is connected.")]
        public bool enableEditorEmulation = true;
        public bool forceEmulationMode = false;

        [Header("Live Controller State (Telemetry Readout)")]
        [SerializeField] private bool isConnected = false;
        [SerializeField] private bool isClutchEngaged = false;
        [SerializeField] private Vector3 gyroEuler;
        [SerializeField] private Vector3 gyroAngularVelocity;
        [SerializeField] private Vector3 controllerPosition;
        [SerializeField] private float triggerValue = 0f;
        [SerializeField] private float gripValue = 0f;
        [SerializeField] private Vector2 thumbstickAxis = Vector2.zero;
        [SerializeField] private bool buttonAPressed = false;
        [SerializeField] private bool buttonBPressed = false;
        [SerializeField] private bool thumbstickClicked = false;
        [SerializeField] private bool menuButtonPressed = false;
        [SerializeField] private string activeDeviceName = "None";

        // Internal tracking states
        private UnityEngine.XR.InputDevice xrDevice;
        private Quaternion baselineOrientation = Quaternion.identity;
        private Vector3 baselinePosition = Vector3.zero;
        private Vector3 lastTrackedEuler = Vector3.zero;

        // Robot target snapshot when clutch was engaged
        private float clutchSnapJ1 = 0f;
        private float clutchSnapJ2 = 0f;
        private float clutchSnapJ3 = 0f;
        private float clutchSnapWrist = 0f;
        private float clutchSnapGripper = 0.5f;

        // Damping velocities
        private float velJ1, velJ2, velJ3, velWrist, velGrip;

        // Button state edge detection & timers
        private bool prevButtonA = false;
        private bool prevButtonB = false;
        private bool prevThumbstickClick = false;
        private bool prevMenuButton = false;
        private float buttonAPressStartTime = 0f;
        private float lastButtonBClickTime = 0f;
        private const float DOUBLE_CLICK_THRESHOLD = 0.35f;
        private const float LONG_PRESS_THRESHOLD = 0.75f;

        // Preset index tracker
        private int currentPresetIndex = 0;

        // Emulation state
        private Vector3 emulatedEuler = Vector3.zero;
        private bool isEmulating = false;

        // Notifications
        private string lastStatusNotification = "Quest 2 Controller Ready";
        private float notificationTimer = 0f;

        // Public accessors for UI
        public bool IsConnected => isConnected || isEmulating;
        public bool IsClutchEngaged => isClutchEngaged;
        public Vector3 GyroEuler => gyroEuler;
        public Vector3 GyroAngularVelocity => gyroAngularVelocity;
        public Vector3 ControllerPosition => controllerPosition;
        public float TriggerValue => triggerValue;
        public float GripValue => gripValue;
        public Vector2 ThumbstickAxis => thumbstickAxis;
        public bool ButtonA => buttonAPressed;
        public bool ButtonB => buttonBPressed;
        public string ActiveDeviceName => activeDeviceName;
        public string StatusNotification => lastStatusNotification;

        void Awake()
        {
            if (armController == null)
            {
                armController = GetComponent<ArmSimulationController>();
            }
            if (ghostArm == null)
            {
                ghostArm = FindAnyObjectByType<GhostArmPreview>();
            }
            if (rosBridge == null)
            {
                rosBridge = GetComponent<RosRoboArmBridge>();
            }
        }

        void Start()
        {
            FindXRController();
            ShowNotification("Quest 2 Subsystem Initialized");
        }

        void OnEnable()
        {
            UnityEngine.XR.InputDevices.deviceConnected += OnDeviceConnected;
            UnityEngine.XR.InputDevices.deviceDisconnected += OnDeviceDisconnected;
        }

        void OnDisable()
        {
            UnityEngine.XR.InputDevices.deviceConnected -= OnDeviceConnected;
            UnityEngine.XR.InputDevices.deviceDisconnected -= OnDeviceDisconnected;
        }

        private void OnDeviceConnected(UnityEngine.XR.InputDevice device)
        {
            FindXRController();
        }

        private void OnDeviceDisconnected(UnityEngine.XR.InputDevice device)
        {
            FindXRController();
        }

        public void FindXRController()
        {
            if (forceEmulationMode)
            {
                isConnected = false;
                activeDeviceName = "Emulated Quest 2 (Keyboard/Mouse)";
                return;
            }

            var desiredCharacteristics = InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.HeldInHand;

            if (controllerHand == ControllerHand.RightHand)
            {
                desiredCharacteristics |= InputDeviceCharacteristics.Right;
            }
            else if (controllerHand == ControllerHand.LeftHand)
            {
                desiredCharacteristics |= InputDeviceCharacteristics.Left;
            }

            var devices = new List<UnityEngine.XR.InputDevice>();
            UnityEngine.XR.InputDevices.GetDevicesWithCharacteristics(desiredCharacteristics, devices);

            if (devices.Count > 0)
            {
                xrDevice = devices[0];
                isConnected = true;
                activeDeviceName = xrDevice.name;
                ShowNotification($"Connected: {xrDevice.name}");
                Debug.Log($"[Quest2Controller] Bound XR Controller: {xrDevice.name}");
                return;
            }

            // Fallback: search for any controller with Oculus or Quest in the name
            UnityEngine.XR.InputDevices.GetDevices(devices);
            foreach (var d in devices)
            {
                if (d.name.IndexOf("Oculus", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    d.name.IndexOf("Quest", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    d.name.IndexOf("Touch", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    xrDevice = d;
                    isConnected = true;
                    activeDeviceName = d.name;
                    ShowNotification($"Connected: {d.name}");
                    Debug.Log($"[Quest2Controller] Fallback bound: {d.name}");
                    return;
                }
            }

            isConnected = false;
            activeDeviceName = "No Hardware Detected (Emulation Active)";
        }

        void Update()
        {
            if (notificationTimer > 0f)
            {
                notificationTimer -= Time.deltaTime;
            }

            // Poll hardware device or fallback to emulation
            if (!forceEmulationMode && isConnected && xrDevice.isValid)
            {
                PollHardwareController();
            }
            else if (enableEditorEmulation)
            {
                PollEmulatedController();
            }

            // Process Clutch engagement / disengagement edge
            bool prevClutch = isClutchEngaged;
            isClutchEngaged = !requireClutchToMove || (gripValue > 0.45f);

            if (isClutchEngaged && !prevClutch)
            {
                OnClutchEngaged();
            }
            else if (!isClutchEngaged && prevClutch)
            {
                OnClutchDisengaged();
            }

            // Process Button Commands
            HandleButtonCommands();

            // Process Gyroscopic Teleoperation
            if (isClutchEngaged && armController != null)
            {
                switch (controlMode)
                {
                    case TeleoperationMode.DirectGyroMimic:
                        ProcessDirectGyroMimic();
                        break;

                    case TeleoperationMode.CartesianIK:
                        ProcessCartesianIK();
                        break;

                    case TeleoperationMode.AngularRateJog:
                        ProcessAngularRateJog();
                        break;
                }
            }

            // Gripper is always controllable via trigger
            ProcessGripperControl();

            // Thumbstick fine-tuning (azimuth & reach trim)
            ProcessThumbstickTrim();
        }

        private void PollHardwareController()
        {
            isEmulating = false;

            // 1. Gyroscopic Orientation (IMU)
            if (xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceRotation, out Quaternion rot))
            {
                gyroEuler = rot.eulerAngles;
            }

            // 2. Gyroscopic Angular Velocity
            if (xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.deviceAngularVelocity, out Vector3 angVel))
            {
                gyroAngularVelocity = angVel;
            }

            // 3. 6DoF Spatial Position
            if (xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 pos))
            {
                controllerPosition = pos;
            }

            // 4. Analog Controls
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.trigger, out triggerValue);
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.grip, out gripValue);
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxis, out thumbstickAxis);

            // 5. Digital Buttons
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primaryButton, out buttonAPressed);
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.secondaryButton, out buttonBPressed);
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primary2DAxisClick, out thumbstickClicked);
            xrDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.menuButton, out menuButtonPressed);
        }

        private void PollEmulatedController()
        {
            isEmulating = true;

            // Gyro emulation: Hold 'Alt' or 'G' and move mouse
            bool gyroModifier = Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt) || Input.GetKey(KeyCode.G);

            if (gyroModifier)
            {
                float mx = Input.GetAxis("Mouse X") * 3.5f;
                float my = Input.GetAxis("Mouse Y") * 3.5f;
                float scroll = Input.GetAxis("Mouse ScrollWheel") * 50f;

                emulatedEuler.y = Mathf.Repeat(emulatedEuler.y + mx, 360f);
                emulatedEuler.x = Mathf.Clamp(emulatedEuler.x - my, -85f, 85f);
                emulatedEuler.z = Mathf.Clamp(emulatedEuler.z + scroll, -90f, 90f);
            }

            gyroEuler = emulatedEuler;
            gyroAngularVelocity = Vector3.zero;

            // Clutch / Grip: Hold Left Shift or Middle Mouse Button
            bool gripHeld = Input.GetKey(KeyCode.LeftShift) || Input.GetMouseButton(2);
            gripValue = gripHeld ? 1.0f : 0f;

            // Trigger / Gripper: Hold Space or Left Mouse Click (when holding gyro modifier)
            if (gyroModifier && Input.GetMouseButton(0))
            {
                triggerValue = Mathf.MoveTowards(triggerValue, 1f, Time.deltaTime * 3f);
            }
            else if (Input.GetKey(KeyCode.C))
            {
                triggerValue = Mathf.MoveTowards(triggerValue, 1f, Time.deltaTime * 3f);
            }
            else
            {
                triggerValue = Mathf.MoveTowards(triggerValue, 0f, Time.deltaTime * 3f);
            }

            // Buttons
            buttonAPressed = Input.GetKey(KeyCode.Alpha1) || Input.GetKey(KeyCode.Keypad1);
            buttonBPressed = Input.GetKey(KeyCode.Alpha2) || Input.GetKey(KeyCode.Keypad2);
            thumbstickClicked = Input.GetKeyDown(KeyCode.T);
            menuButtonPressed = Input.GetKeyDown(KeyCode.M);

            // Thumbstick
            Vector2 stick = Vector2.zero;
            if (Input.GetKey(KeyCode.I)) stick.y += 1f;
            if (Input.GetKey(KeyCode.K)) stick.y -= 1f;
            if (Input.GetKey(KeyCode.J)) stick.x -= 1f;
            if (Input.GetKey(KeyCode.L)) stick.x += 1f;
            thumbstickAxis = stick;
        }

        private void OnClutchEngaged()
        {
            if (armController == null) return;

            // Zero baseline: capture starting controller orientation and current robot target pose
            baselineOrientation = Quaternion.Euler(gyroEuler);
            baselinePosition = controllerPosition;

            clutchSnapJ1 = armController.targetJoint1;
            clutchSnapJ2 = armController.targetJoint2;
            clutchSnapJ3 = armController.targetJoint3;
            clutchSnapWrist = armController.targetWrist;
            clutchSnapGripper = armController.targetGripper;

            TriggerHaptic(0.35f, 0.08f);
            ShowNotification("✓ CLUTCH ENGAGED (Gyroscopic Control Active)");
        }

        private void OnClutchDisengaged()
        {
            TriggerHaptic(0.2f, 0.05f);
            ShowNotification("⏸ CLUTCH DISENGAGED (Pose Locked)");
        }

        public void CalibrateZero()
        {
            baselineOrientation = Quaternion.Euler(gyroEuler);
            baselinePosition = controllerPosition;

            if (armController != null)
            {
                clutchSnapJ1 = armController.targetJoint1;
                clutchSnapJ2 = armController.targetJoint2;
                clutchSnapJ3 = armController.targetJoint3;
                clutchSnapWrist = armController.targetWrist;
            }

            TriggerHaptic(0.5f, 0.12f);
            ShowNotification("🎯 Gyro Baseline Calibrated (Zero Reference Set)");
        }

        private void HandleButtonCommands()
        {
            if (armController == null) return;

            // ========================================================
            // Button A (Primary Button): Preset Cycle / Home Pose
            // ========================================================
            if (buttonAPressed && !prevButtonA)
            {
                buttonAPressStartTime = Time.time;
            }
            else if (!buttonAPressed && prevButtonA)
            {
                float pressDuration = Time.time - buttonAPressStartTime;

                if (pressDuration >= LONG_PRESS_THRESHOLD)
                {
                    // Long Press -> Return Home (Upright)
                    armController.ApplyPresetPose(0f, 0f, 0f, 0f, 0.5f);
                    TriggerHaptic(0.7f, 0.2f);
                    ShowNotification("🏠 COMMAND: Return to Home Pose");
                }
                else
                {
                    // Short Tap -> Cycle to Next Preset Waypoint
                    if (armController.waypoints != null && armController.waypoints.Length > 0)
                    {
                        currentPresetIndex = (currentPresetIndex + 1) % armController.waypoints.Length;
                        var wp = armController.waypoints[currentPresetIndex];
                        armController.ApplyPresetPose(wp.j1, wp.j2, wp.j3, wp.wrist, wp.grip);
                        TriggerHaptic(0.4f, 0.08f);
                        ShowNotification($"📦 PRESET [{currentPresetIndex + 1}/{armController.waypoints.Length}]: {wp.name}");
                    }
                }
            }
            prevButtonA = buttonAPressed;

            // ========================================================
            // Button B (Secondary Button): Toggle Ghost / ROS Stream
            // ========================================================
            if (buttonBPressed && !prevButtonB)
            {
                float timeSinceLastClick = Time.time - lastButtonBClickTime;

                if (timeSinceLastClick <= DOUBLE_CLICK_THRESHOLD)
                {
                    // Double Click -> Toggle ROS 2 Auto-Streaming
                    if (rosBridge != null)
                    {
                        bool newStream = !rosBridge.streamCommandsToRos;
                        rosBridge.SetAutoStream(newStream);
                        TriggerHaptic(0.6f, 0.15f);
                        ShowNotification(newStream ? "📡 ROS 2 STREAMING: ENABLED" : "⏸ ROS 2 STREAMING: DISABLED");
                    }
                }
                else
                {
                    // Single Click -> Toggle Ghost Hologram Preview
                    if (ghostArm != null)
                    {
                        ghostArm.SetVisible(!ghostArm.isVisible);
                        TriggerHaptic(0.35f, 0.08f);
                        ShowNotification(ghostArm.isVisible ? "👻 GHOST PREVIEW: ON" : "👻 GHOST PREVIEW: OFF");
                    }
                }

                lastButtonBClickTime = Time.time;
            }
            prevButtonB = buttonBPressed;

            // ========================================================
            // Thumbstick Click: Re-calibrate Zero Reference
            // ========================================================
            if (thumbstickClicked && !prevThumbstickClick)
            {
                CalibrateZero();
            }
            prevThumbstickClick = thumbstickClicked;

            // ========================================================
            // Menu Button: Emergency Hold
            // ========================================================
            if (menuButtonPressed && !prevMenuButton)
            {
                armController.EmergencyHold();
                TriggerHaptic(1.0f, 0.35f);
                ShowNotification("🛑 EMERGENCY HOLD ACTIVATED");
            }
            prevMenuButton = menuButtonPressed;
        }

        private void ProcessDirectGyroMimic()
        {
            // Calculate delta rotation relative to baseline orientation
            Quaternion currentRot = Quaternion.Euler(gyroEuler);
            Quaternion deltaRot = currentRot * Quaternion.Inverse(baselineOrientation);

            // Convert to delta Euler angles in range [-180, 180]
            Vector3 deltaEuler = deltaRot.eulerAngles;
            float deltaPitch = NormalizeAngle(deltaEuler.x);
            float deltaYaw = NormalizeAngle(deltaEuler.y);
            float deltaRoll = NormalizeAngle(deltaEuler.z);

            // Apply deadzones
            if (Mathf.Abs(deltaYaw) < gyroDeadzoneDeg) deltaYaw = 0f;
            if (Mathf.Abs(deltaPitch) < gyroDeadzoneDeg) deltaPitch = 0f;
            if (Mathf.Abs(deltaRoll) < gyroDeadzoneDeg) deltaRoll = 0f;

            // Inversions
            if (invertYaw) deltaYaw = -deltaYaw;
            if (invertPitch) deltaPitch = -deltaPitch;
            if (invertRoll) deltaRoll = -deltaRoll;

            // Calculate commanded joint angles
            // Joint 1: Base Pan follows Controller Yaw
            float targetJ1 = clutchSnapJ1 + (deltaYaw * yawSensitivity);
            targetJ1 = Mathf.Clamp(targetJ1, -180f, 180f);

            // Joint 2: Shoulder Pitch follows Controller Pitch
            float targetJ2 = clutchSnapJ2 + (deltaPitch * pitchSensitivity);
            targetJ2 = Mathf.Clamp(targetJ2, -90f, 90f);

            // Joint 3: Elbow Pitch complements shoulder pitch for natural reach
            float targetJ3 = clutchSnapJ3 - (deltaPitch * pitchSensitivity * elbowCoupling);
            targetJ3 = Mathf.Clamp(targetJ3, -154.7f, 154.7f);

            // Wrist: Follows Controller Roll (wrist axial rotation)
            float targetWrist = clutchSnapWrist + (deltaRoll * rollSensitivity);
            targetWrist = Mathf.Clamp(targetWrist, -90f, 90f);

            // Smoothly interpolate arm targets
            armController.targetJoint1 = Mathf.SmoothDamp(armController.targetJoint1, targetJ1, ref velJ1, smoothingTime);
            armController.targetJoint2 = Mathf.SmoothDamp(armController.targetJoint2, targetJ2, ref velJ2, smoothingTime);
            armController.targetJoint3 = Mathf.SmoothDamp(armController.targetJoint3, targetJ3, ref velJ3, smoothingTime);
            armController.targetWrist = Mathf.SmoothDamp(armController.targetWrist, targetWrist, ref velWrist, smoothingTime);
        }

        private void ProcessCartesianIK()
        {
            // Position delta relative to clutch baseline
            Vector3 deltaPos = (controllerPosition - baselinePosition) * ikReachScale;

            // Forward reach (Z), lateral swing (X), elevation (Y)
            Vector3 targetP = ikWorkspaceCenter + deltaPos;

            // Analytical 2-Link + Azimuth Planar IK for EB15:
            // 1. Azimuth (Joint 1 Base Pan)
            float basePan = Mathf.Atan2(targetP.x, targetP.z) * Mathf.Rad2Deg;
            if (invertYaw) basePan = -basePan;
            basePan = Mathf.Clamp(basePan, -180f, 180f);

            // 2. Planar reach in shoulder plane
            float r = Mathf.Sqrt((targetP.x * targetP.x) + (targetP.z * targetP.z));
            float y = targetP.y - 0.0738f; // Shoulder joint height offset
            float d = Mathf.Sqrt((r * r) + (y * y));

            // Link lengths (meters): L1 (upper arm) = 0.228m, L2 (forearm) = 0.178m
            float L1 = 0.228f;
            float L2 = 0.178f;

            d = Mathf.Clamp(d, 0.06f, (L1 + L2) * 0.98f);

            // Law of Cosines for elbow angle (theta3)
            float cosElbow = ((d * d) - (L1 * L1) - (L2 * L2)) / (2f * L1 * L2);
            cosElbow = Mathf.Clamp(cosElbow, -1f, 1f);
            float elbowAngleRad = Mathf.Acos(cosElbow);
            float targetJ3 = (Mathf.PI - elbowAngleRad) * Mathf.Rad2Deg;

            // Shoulder angle (theta2)
            float alpha = Mathf.Atan2(y, r);
            float beta = Mathf.Atan2(L2 * Mathf.Sin(elbowAngleRad), L1 + (L2 * Mathf.Cos(elbowAngleRad)));
            float targetJ2 = (alpha + beta) * Mathf.Rad2Deg - 90f;

            // Wrist tracks controller roll
            Quaternion currentRot = Quaternion.Euler(gyroEuler);
            Quaternion deltaRot = currentRot * Quaternion.Inverse(baselineOrientation);
            float deltaRoll = NormalizeAngle(deltaRot.eulerAngles.z);
            float targetWrist = clutchSnapWrist + (deltaRoll * rollSensitivity);

            // Apply clamped targets
            armController.targetJoint1 = Mathf.SmoothDamp(armController.targetJoint1, Mathf.Clamp(basePan, -180f, 180f), ref velJ1, smoothingTime);
            armController.targetJoint2 = Mathf.SmoothDamp(armController.targetJoint2, Mathf.Clamp(targetJ2, -90f, 90f), ref velJ2, smoothingTime);
            armController.targetJoint3 = Mathf.SmoothDamp(armController.targetJoint3, Mathf.Clamp(targetJ3, -154.7f, 154.7f), ref velJ3, smoothingTime);
            armController.targetWrist = Mathf.SmoothDamp(armController.targetWrist, Mathf.Clamp(targetWrist, -90f, 90f), ref velWrist, smoothingTime);
        }

        private void ProcessAngularRateJog()
        {
            Quaternion currentRot = Quaternion.Euler(gyroEuler);
            Quaternion deltaRot = currentRot * Quaternion.Inverse(baselineOrientation);
            Vector3 deltaEuler = deltaRot.eulerAngles;

            float pitchRate = NormalizeAngle(deltaEuler.x);
            float yawRate = NormalizeAngle(deltaEuler.y);
            float rollRate = NormalizeAngle(deltaEuler.z);

            float dead = gyroDeadzoneDeg * 1.5f;
            float dt = Time.deltaTime;

            if (Mathf.Abs(yawRate) > dead)
            {
                float speed = (yawRate - (Mathf.Sign(yawRate) * dead)) * yawSensitivity * 0.8f;
                armController.targetJoint1 = Mathf.Clamp(armController.targetJoint1 + (speed * dt), -180f, 180f);
            }

            if (Mathf.Abs(pitchRate) > dead)
            {
                float speed = (pitchRate - (Mathf.Sign(pitchRate) * dead)) * pitchSensitivity * 0.6f;
                armController.targetJoint2 = Mathf.Clamp(armController.targetJoint2 + (speed * dt), -90f, 90f);
                armController.targetJoint3 = Mathf.Clamp(armController.targetJoint3 - (speed * elbowCoupling * dt), -154.7f, 154.7f);
            }

            if (Mathf.Abs(rollRate) > dead)
            {
                float speed = (rollRate - (Mathf.Sign(rollRate) * dead)) * rollSensitivity * 1.0f;
                armController.targetWrist = Mathf.Clamp(armController.targetWrist + (speed * dt), -90f, 90f);
            }
        }

        private void ProcessGripperControl()
        {
            if (armController == null) return;

            // Trigger squeeze [0..1] -> Gripper stroke [1..0] (squeezing closes the gripper)
            float rawGrip = invertGripper ? triggerValue : (1.0f - triggerValue);

            // Smooth gripper movement
            armController.targetGripper = Mathf.SmoothDamp(armController.targetGripper, rawGrip, ref velGrip, 0.04f);

            // Haptic vibration feedback on tight squeeze
            if (enableHaptics && triggerValue > 0.85f && isConnected)
            {
                TriggerHaptic(0.25f * triggerValue, 0.04f);
            }
        }

        private void ProcessThumbstickTrim()
        {
            if (armController == null) return;

            // Thumbstick X fine-tunes Base Pan (Joint 1)
            if (Mathf.Abs(thumbstickAxis.x) > 0.15f)
            {
                float trimX = thumbstickAxis.x * 45f * Time.deltaTime;
                armController.targetJoint1 = Mathf.Clamp(armController.targetJoint1 + trimX, -180f, 180f);
            }

            // Thumbstick Y fine-tunes Reach (Joint 2 & 3)
            if (Mathf.Abs(thumbstickAxis.y) > 0.15f)
            {
                float trimY = thumbstickAxis.y * 35f * Time.deltaTime;
                armController.targetJoint2 = Mathf.Clamp(armController.targetJoint2 + trimY, -90f, 90f);
                armController.targetJoint3 = Mathf.Clamp(armController.targetJoint3 + (trimY * 0.7f), -154.7f, 154.7f);
            }
        }

        public void TriggerHaptic(float amplitude, float duration)
        {
            if (!enableHaptics) return;

            if (isConnected && xrDevice.isValid)
            {
                xrDevice.SendHapticImpulse(0, Mathf.Clamp01(amplitude * hapticIntensity), duration);
            }
        }

        private float NormalizeAngle(float angle)
        {
            while (angle > 180f) angle -= 360f;
            while (angle < -180f) angle += 360f;
            return angle;
        }

        public void ShowNotification(string message)
        {
            lastStatusNotification = message;
            notificationTimer = 3.5f;
            Debug.Log($"[Quest2 Controller] {message}");
        }

        // =========================================================================
        // ON-SCREEN HUD / VISUAL FEEDBACK
        // =========================================================================
        void OnGUI()
        {
            // Only draw floating telemetry badge when dashboard is hidden or minimized
            if (armController != null && !armController.showDashboard)
            {
                DrawFloatingBadge();
            }
        }

        private void DrawFloatingBadge()
        {
            float badgeW = 260f;
            float badgeH = 72f;
            float x = Screen.width - badgeW - 24f;
            float y = Screen.height - badgeH - 24f;

            GUI.color = new Color(0.08f, 0.11f, 0.16f, 0.88f);
            GUI.Box(new Rect(x, y, badgeW, badgeH), "");
            GUI.color = Color.white;

            GUILayout.BeginArea(new Rect(x + 10, y + 8, badgeW - 20, badgeH - 16));

            GUILayout.BeginHorizontal();
            GUI.color = isClutchEngaged ? new Color(0.2f, 0.95f, 0.5f) : new Color(1f, 0.65f, 0.2f);
            GUILayout.Label(isClutchEngaged ? "● QUEST 2 CLUTCH ON" : "○ QUEST 2 CLUTCH OFF", GUILayout.Width(170));
            GUI.color = Color.white;
            GUILayout.Label(isEmulating ? "EMU" : "VR", GUILayout.Width(35));
            GUILayout.EndHorizontal();

            GUILayout.Label($"Gyro: P:{gyroEuler.x:F0}° Y:{gyroEuler.y:F0}° R:{gyroEuler.z:F0}° | Grip: {gripValue * 100:F0}%");
            GUILayout.Label(notificationTimer > 0f ? lastStatusNotification : "Hold Grip to Teleoperate • Squeeze Trigger to Pinch");

            GUILayout.EndArea();
        }
    }
}

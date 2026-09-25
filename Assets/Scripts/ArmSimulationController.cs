using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoboArm
{
    /// <summary>
    /// Master Digital Twin & Interactive Dashboard for the EB15 Robotic Arm.
    /// Features a modern, semi-transparent frosted glass UI inspired by game/simulator settings menus.
    /// Manages the Physical Twin (Solid Arm) and Commanded Goal (Ghost Preview).
    /// </summary>
    public class ArmSimulationController : MonoBehaviour
    {
        [Header("Physical Twin Joints (Solid Arm)")]
        public ArticulationBody joint1;        // Revolute (-180 to 180 deg)
        public ArticulationBody joint2;        // Revolute (-90 to 90 deg)
        public ArticulationBody joint3;        // Revolute (-154.7 to 154.7 deg)
        public ArticulationBody wristJoint;    // Revolute (-90 to 90 deg)
        public ArticulationBody gripperLeft;   // Prismatic (-0.016 to 0.012 m)
        public ArticulationBody gripperRight;  // Prismatic (-0.016 to 0.012 m)

        [Header("Ghost Preview Reference")]
        public GhostArmPreview ghostArm;

        [Header("Commanded Target Angles (Ghost Preview & Real Hardware Goal)")]
        [Range(-180f, 180f)] public float targetJoint1 = 0f;
        [Range(-90f, 90f)] public float targetJoint2 = 0f;
        [Range(-154.7f, 154.7f)] public float targetJoint3 = 0f;
        [Range(-90f, 90f)] public float targetWrist = 0f;
        [Range(0f, 1f)] public float targetGripper = 0.5f;

        [Header("Physical Arm Measured Angles (Solid Digital Twin)")]
        public float currentJoint1 = 0f;
        public float currentJoint2 = 0f;
        public float currentJoint3 = 0f;
        public float currentWrist = 0f;
        public float currentGripper = 0.5f;

        [Header("Real-World Physical Speed Limits (deg/sec & m/sec)")]
        public float maxSpeedJoint1 = 57.3f;   // 1.0 rad/s
        public float maxSpeedJoint2 = 45.8f;   // 0.8 rad/s
        public float maxSpeedJoint3 = 57.3f;   // 1.0 rad/s
        public float maxSpeedWrist = 114.6f;   // 2.0 rad/s
        public float maxSpeedGripper = 1.78f;  // Normalized travel rate (0.05 m/s)

        [Header("Drive Dynamics")]
        public float driveStiffness = 30000f;
        public float driveDamping = 500f;
        public float driveForceLimit = 5000f;

        [Header("Operation Mode")]
        public bool autoDemo = false;
        public float demoSpeed = 0.6f;

        [Header("Hardware Encoder Status")]
        public bool isHardwareEncoderLive = false;
        private float lastEncoderReceiveTime = -10f;

        [Header("UI Dashboard Settings")]
        public bool showDashboard = true;
        private int selectedSidebarTab = 0; // 0: Controls, 1: Network, 2: Presets, 3: Guide

        // Network inputs
        private string inputRosHost = "192.168.1.54";
        private string inputListenPort = "5005";
        private string inputRosPort = "5006";
        private string networkStatusMsg = "";

        // Real-time teleoperation streaming
        private float streamTimer = 0f;
        private float lastStreamedJ1 = 9999f;
        private float lastStreamedJ2 = 9999f;
        private float lastStreamedJ3 = 9999f;
        private float lastStreamedWrist = 9999f;
        private float lastStreamedGrip = 9999f;

        [System.Serializable]
        public struct Waypoint
        {
            public string name;
            public float j1, j2, j3, wrist, grip;
            public Waypoint(string name, float j1, float j2, float j3, float wrist, float grip)
            {
                this.name = name; this.j1 = j1; this.j2 = j2; this.j3 = j3; this.wrist = wrist; this.grip = grip;
            }
        }

        public Waypoint[] waypoints = new Waypoint[]
        {
            new Waypoint("Home (Upright)", 0f, 0f, 0f, 0f, 1f),
            new Waypoint("Reach Forward Right", 45f, -35f, 50f, 25f, 1f),
            new Waypoint("Pick Target", 45f, -55f, 75f, 35f, 0f),
            new Waypoint("Lift Object", 45f, -25f, 35f, 15f, 0f),
            new Waypoint("Swing to Left", -50f, -25f, 35f, -15f, 0f),
            new Waypoint("Place Target", -50f, -55f, 75f, -35f, 1f),
            new Waypoint("Retract", -50f, -10f, 20f, 0f, 1f),
            new Waypoint("Return Home", 0f, 0f, 0f, 0f, 0.5f)
        };

        private int currentWaypointIndex = 0;
        private float waypointProgress = 0f;
        private RosRoboArmBridge rosBridge;

        // Custom Frosted Glass Textures & Styles
        private Texture2D texSidebar;
        private Texture2D texMainPanel;
        private Texture2D texDropdownPill;
        private Texture2D texDivider;
        private Texture2D texCheckmarkOn;
        private Texture2D texCheckmarkOff;
        private Texture2D texBtnReset;
        private Texture2D texBtnClose;
        private Texture2D texBtnApply;
        private Texture2D texBtnSave;

        private GUIStyle styleSidebar;
        private GUIStyle styleMainPanel;
        private GUIStyle styleTabActive;
        private GUIStyle styleTabInactive;
        private GUIStyle styleSectionHeader;
        private GUIStyle styleDivider;
        private GUIStyle styleSubtext;
        private GUIStyle styleBodyText;
        private GUIStyle styleCodeBox;
        private GUIStyle styleCardPill;
        private GUIStyle styleValueLabel;
        private GUIStyle styleCheckLabel;
        private GUIStyle styleCheckIcon;
        private GUIStyle styleCheckIconOff;
        private GUIStyle styleBtnReset;
        private GUIStyle styleBtnClose;
        private GUIStyle styleBtnApply;
        private GUIStyle styleBtnSave;
        private bool stylesReady = false;

        void Awake()
        {
            Application.runInBackground = true;
            FindJoints();
            ConfigureJointDrives();

            // Stabilize ArticulationBodies with high solver iterations
            var abs = GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in abs)
            {
                ab.solverIterations = 30;
                ab.solverVelocityIterations = 15;
            }

            var baseLink = transform.Find("world/base_link");
            if (baseLink != null)
            {
                var baseAb = baseLink.GetComponent<ArticulationBody>();
                if (baseAb != null) baseAb.immovable = true;
            }

            // Ignore all internal collisions between robot links to completely prevent physics fighting
            var colliders = GetComponentsInChildren<Collider>();
            for (int i = 0; i < colliders.Length; i++)
            {
                for (int j = i + 1; j < colliders.Length; j++)
                {
                    Physics.IgnoreCollision(colliders[i], colliders[j], true);
                }
            }

            if (ghostArm == null)
            {
                ghostArm = FindAnyObjectByType<GhostArmPreview>();
            }

            rosBridge = GetComponent<RosRoboArmBridge>();
            if (rosBridge != null)
            {
                inputRosHost = rosBridge.rosHost;
                inputListenPort = rosBridge.listenPort.ToString();
                inputRosPort = rosBridge.rosPort.ToString();
            }
        }

        void OnEnable()
        {
            stylesReady = false;
        }

        public void FindJoints()
        {
            var bodies = GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in bodies)
            {
                switch (ab.gameObject.name)
                {
                    case "link1": joint1 = ab; break;
                    case "link2": joint2 = ab; break;
                    case "link3": joint3 = ab; break;
                    case "wrist_link": wristJoint = ab; break;
                    case "gripper_left_finger_link": gripperLeft = ab; break;
                    case "gripper_right_finger_link": gripperRight = ab; break;
                }
            }
        }

        public void ConfigureJointDrives()
        {
            SetDrive(joint1, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(joint2, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(joint3, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(wristJoint, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(gripperLeft, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(gripperRight, driveStiffness, driveDamping, driveForceLimit);
        }

        private void SetDrive(ArticulationBody ab, float stiffness, float damping, float forceLimit)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            ab.xDrive = drive;
        }

        void Update()
        {
            HandleKeyboardInput();

            isHardwareEncoderLive = (Time.time - lastEncoderReceiveTime) < 1.2f;

            if (autoDemo && waypoints != null && waypoints.Length > 1)
            {
                UpdateAutoDemo();
            }

            // Ghost Preview updates immediately
            if (ghostArm != null)
            {
                ghostArm.SetPose(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }

            // Sync UI input field with bridge host if bridge auto-detected or updated
            if (rosBridge != null && !string.IsNullOrEmpty(rosBridge.rosHost) && rosBridge.rosHost != "127.0.0.1" && inputRosHost == "127.0.0.1")
            {
                inputRosHost = rosBridge.rosHost;
            }

            // Continuous Real-Time Streaming: Stream target ghost pose to real robot when Auto-Stream is enabled
            if (rosBridge != null && rosBridge.streamCommandsToRos)
            {
                streamTimer += Time.deltaTime;
                bool poseChanged = Mathf.Abs(targetJoint1 - lastStreamedJ1) > 0.05f ||
                                   Mathf.Abs(targetJoint2 - lastStreamedJ2) > 0.05f ||
                                   Mathf.Abs(targetJoint3 - lastStreamedJ3) > 0.05f ||
                                   Mathf.Abs(targetWrist - lastStreamedWrist) > 0.05f ||
                                   Mathf.Abs(targetGripper - lastStreamedGrip) > 0.005f;

                // Stream at 25 Hz when dragging sliders or moving in demo, or 2 Hz heartbeat to maintain latched pose
                if ((poseChanged && streamTimer >= 0.04f) || streamTimer >= 0.5f)
                {
                    rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
                    lastStreamedJ1 = targetJoint1;
                    lastStreamedJ2 = targetJoint2;
                    lastStreamedJ3 = targetJoint3;
                    lastStreamedWrist = targetWrist;
                    lastStreamedGrip = targetGripper;
                    streamTimer = 0f;
                }
            }

            // Solid Arm (Physical Twin) tracking
            if (!isHardwareEncoderLive)
            {
                float dt = Time.deltaTime;
                currentJoint1 = Mathf.MoveTowards(currentJoint1, targetJoint1, maxSpeedJoint1 * dt);
                currentJoint2 = Mathf.MoveTowards(currentJoint2, targetJoint2, maxSpeedJoint2 * dt);
                currentJoint3 = Mathf.MoveTowards(currentJoint3, targetJoint3, maxSpeedJoint3 * dt);
                currentWrist = Mathf.MoveTowards(currentWrist, targetWrist, maxSpeedWrist * dt);
                currentGripper = Mathf.MoveTowards(currentGripper, targetGripper, maxSpeedGripper * dt);
            }

            ApplyCurrentJointsToPhysicalTwin();
        }

        void FixedUpdate()
        {
            ApplyCurrentJointsToPhysicalTwin();
        }

        public void OnHardwareEncoderReceived(float j1Deg, float j2Deg, float j3Deg, float wristDeg, float gripNorm)
        {
            currentJoint1 = Mathf.Clamp(j1Deg, -180f, 180f);
            currentJoint2 = Mathf.Clamp(j2Deg, -90f, 90f);
            currentJoint3 = Mathf.Clamp(j3Deg, -154.7f, 154.7f);
            currentWrist = Mathf.Clamp(wristDeg, -90f, 90f);
            currentGripper = Mathf.Clamp01(gripNorm);

            lastEncoderReceiveTime = Time.time;
            isHardwareEncoderLive = true;
        }

        public void SnapGhostToPhysicalPose()
        {
            targetJoint1 = currentJoint1;
            targetJoint2 = currentJoint2;
            targetJoint3 = currentJoint3;
            targetWrist = currentWrist;
            targetGripper = currentGripper;

            if (ghostArm != null)
            {
                ghostArm.SetPose(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
        }

        public void EmergencyHold()
        {
            autoDemo = false;
            SnapGhostToPhysicalPose();
            if (rosBridge != null)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
            Debug.LogWarning("[EB-15 Control] EMERGENCY HOLD: Commanded goals locked to current physical position.");
        }

        public void ApplyPresetPose(float j1, float j2, float j3, float wrist, float grip)
        {
            targetJoint1 = j1;
            targetJoint2 = j2;
            targetJoint3 = j3;
            targetWrist = wrist;
            targetGripper = grip;

            if (ghostArm != null)
            {
                ghostArm.SetPose(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }

            if (rosBridge != null && rosBridge.streamCommandsToRos)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
        }

        private void ApplyCurrentJointsToPhysicalTwin()
        {
            SetTarget(joint1, currentJoint1);
            SetTarget(joint2, currentJoint2);
            SetTarget(joint3, currentJoint3);
            SetTarget(wristJoint, currentWrist);

            float gripM = Mathf.Lerp(-0.016f, 0.012f, currentGripper);
            SetTarget(gripperLeft, gripM);
            SetTarget(gripperRight, gripM);
        }

        private void SetTarget(ArticulationBody ab, float target)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.target = target;
            ab.xDrive = drive;
        }

        private void HandleKeyboardInput()
        {
#if ENABLE_INPUT_SYSTEM
            try
            {
                if (Keyboard.current != null)
                {
                    if (Keyboard.current.spaceKey.wasPressedThisFrame) autoDemo = !autoDemo;
                    if (Keyboard.current.tabKey.wasPressedThisFrame) showDashboard = !showDashboard;
                    if (Keyboard.current.gKey.wasPressedThisFrame && ghostArm != null)
                        ghostArm.SetVisible(!ghostArm.isVisible);
                    if (Keyboard.current.hKey.wasPressedThisFrame) ApplyPresetPose(0, 0, 0, 0, 0.5f);
                }
            }
            catch (System.Exception) { }
#else
            try
            {
                if (Input.GetKeyDown(KeyCode.Space)) autoDemo = !autoDemo;
                if (Input.GetKeyDown(KeyCode.Tab)) showDashboard = !showDashboard;
                if (Input.GetKeyDown(KeyCode.G) && ghostArm != null)
                    ghostArm.SetVisible(!ghostArm.isVisible);
                if (Input.GetKeyDown(KeyCode.H)) ApplyPresetPose(0, 0, 0, 0, 0.5f);
            }
            catch (System.Exception) { }
#endif
        }

        private void UpdateAutoDemo()
        {
            waypointProgress += Time.deltaTime * demoSpeed;
            if (waypointProgress >= 1f)
            {
                waypointProgress = 0f;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }

            int nextIndex = (currentWaypointIndex + 1) % waypoints.Length;
            var from = waypoints[currentWaypointIndex];
            var to = waypoints[nextIndex];
            float t = Mathf.SmoothStep(0f, 1f, waypointProgress);

            targetJoint1 = Mathf.Lerp(from.j1, to.j1, t);
            targetJoint2 = Mathf.Lerp(from.j2, to.j2, t);
            targetJoint3 = Mathf.Lerp(from.j3, to.j3, t);
            targetWrist = Mathf.Lerp(from.wrist, to.wrist, t);
            targetGripper = Mathf.Lerp(from.grip, to.grip, t);

            if (rosBridge != null && rosBridge.streamCommandsToRos)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
        }

        // =========================================================================
        // MODERN THEMED IMGUI RENDERER (Matching User's Uploaded Style)
        // =========================================================================

        private Texture2D CreateRoundedRectTexture(int width, int height, int radius, Color fillColor,
            bool roundTL = true, bool roundTR = true, bool roundBL = true, bool roundBR = true)
        {
            Texture2D tex = new Texture2D(width, height, TextureFormat.RGBA32, false);
            Color transparent = new Color(0, 0, 0, 0);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    bool isTL = (x < radius && y >= height - radius && roundTL);
                    bool isTR = (x >= width - radius && y >= height - radius && roundTR);
                    bool isBL = (x < radius && y < radius && roundBL);
                    bool isBR = (x >= width - radius && y < radius && roundBR);

                    if (isTL || isTR || isBL || isBR)
                    {
                        int cx = (x < radius) ? radius : (width - radius - 1);
                        int cy = (y < radius) ? radius : (height - radius - 1);

                        float dist = Vector2.Distance(new Vector2(x, y), new Vector2(cx, cy));
                        if (dist <= radius - 0.5f)
                        {
                            tex.SetPixel(x, y, fillColor);
                        }
                        else if (dist <= radius + 0.5f)
                        {
                            float a = Mathf.Clamp01(radius + 0.5f - dist) * fillColor.a;
                            tex.SetPixel(x, y, new Color(fillColor.r, fillColor.g, fillColor.b, a));
                        }
                        else
                        {
                            tex.SetPixel(x, y, transparent);
                        }
                    }
                    else
                    {
                        tex.SetPixel(x, y, fillColor);
                    }
                }
            }
            tex.Apply();
            return tex;
        }

        private Texture2D CreateSolidTexture(int width, int height, Color col)
        {
            Texture2D tex = new Texture2D(width, height);
            Color[] pix = new Color[width * height];
            for (int i = 0; i < pix.Length; i++) pix[i] = col;
            tex.SetPixels(pix);
            tex.Apply();
            return tex;
        }

        private void InitModernStyles()
        {
            if (stylesReady && styleBodyText != null && styleCodeBox != null && texSidebar != null) return;

            // Semi-transparent frosted textures with 9-slice support
            // Left sidebar: Dark translucent obsidian (rgba: 14, 18, 24, 0.90), rounded left corners only
            texSidebar = CreateRoundedRectTexture(64, 64, 16, new Color(0.05f, 0.07f, 0.10f, 0.90f), roundTL: true, roundTR: false, roundBL: true, roundBR: false);

            // Main panel: Frosted milky glass (rgba: 242, 245, 249, 0.68), rounded right corners only
            texMainPanel = CreateRoundedRectTexture(64, 64, 16, new Color(0.93f, 0.95f, 0.97f, 0.68f), roundTL: false, roundTR: true, roundBL: false, roundBR: true);

            // Sleek dark-slate card/dropdown pills
            texDropdownPill = CreateRoundedRectTexture(32, 32, 8, new Color(0.38f, 0.42f, 0.48f, 0.95f));

            // Column Header Divider line
            texDivider = CreateSolidTexture(4, 4, new Color(0.72f, 0.77f, 0.84f, 0.75f));

            // Checkbox textures
            texCheckmarkOn = CreateRoundedRectTexture(22, 22, 5, new Color(0.18f, 0.48f, 0.92f, 1f));
            texCheckmarkOff = CreateRoundedRectTexture(22, 22, 5, new Color(0.72f, 0.76f, 0.82f, 0.90f));

            // Bottom Action Buttons
            texBtnReset = CreateRoundedRectTexture(32, 32, 8, new Color(0.56f, 0.61f, 0.68f, 0.96f));
            texBtnClose = CreateRoundedRectTexture(32, 32, 8, new Color(0.42f, 0.48f, 0.55f, 0.96f));
            texBtnApply = CreateRoundedRectTexture(32, 32, 8, new Color(0.18f, 0.46f, 0.88f, 0.98f));
            texBtnSave = CreateRoundedRectTexture(32, 32, 8, new Color(0.14f, 0.36f, 0.76f, 0.98f));

            // GUIStyles with 9-slice borders
            styleSidebar = new GUIStyle(GUI.skin.box);
            styleSidebar.normal.background = texSidebar;
            styleSidebar.border = new RectOffset(16, 0, 16, 16);

            styleMainPanel = new GUIStyle(GUI.skin.box);
            styleMainPanel.normal.background = texMainPanel;
            styleMainPanel.border = new RectOffset(0, 16, 16, 16);

            styleDivider = new GUIStyle(GUI.skin.box);
            styleDivider.normal.background = texDivider;

            styleTabActive = new GUIStyle(GUI.skin.label);
            styleTabActive.fontSize = 15;
            styleTabActive.fontStyle = FontStyle.Bold;
            styleTabActive.normal.textColor = Color.white;
            styleTabActive.alignment = TextAnchor.MiddleLeft;

            styleTabInactive = new GUIStyle(GUI.skin.label);
            styleTabInactive.fontSize = 14;
            styleTabInactive.normal.textColor = new Color(0.55f, 0.62f, 0.72f);
            styleTabInactive.alignment = TextAnchor.MiddleLeft;

            styleSectionHeader = new GUIStyle(GUI.skin.label);
            styleSectionHeader.fontSize = 15;
            styleSectionHeader.fontStyle = FontStyle.Bold;
            styleSectionHeader.normal.textColor = new Color(0.12f, 0.16f, 0.22f);

            styleSubtext = new GUIStyle(GUI.skin.label);
            styleSubtext.fontSize = 11;
            styleSubtext.fontStyle = FontStyle.Bold;
            styleSubtext.normal.textColor = new Color(0.25f, 0.31f, 0.39f);
            styleSubtext.alignment = TextAnchor.MiddleLeft;
            styleSubtext.wordWrap = true;

            styleBodyText = new GUIStyle(GUI.skin.label);
            styleBodyText.fontSize = 11;
            styleBodyText.normal.textColor = new Color(0.20f, 0.25f, 0.33f);
            styleBodyText.alignment = TextAnchor.UpperLeft;
            styleBodyText.wordWrap = true;
            styleBodyText.richText = true;

            styleCodeBox = new GUIStyle(GUI.skin.box);
            styleCodeBox.normal.background = texDropdownPill;
            styleCodeBox.border = new RectOffset(8, 8, 8, 8);
            styleCodeBox.fontSize = 10;
            styleCodeBox.fontStyle = FontStyle.Bold;
            styleCodeBox.normal.textColor = new Color(0.92f, 0.96f, 1.0f);
            styleCodeBox.alignment = TextAnchor.MiddleLeft;
            styleCodeBox.wordWrap = true;
            styleCodeBox.padding = new RectOffset(10, 10, 8, 8);

            styleCardPill = new GUIStyle(GUI.skin.box);
            styleCardPill.normal.background = texDropdownPill;
            styleCardPill.border = new RectOffset(8, 8, 8, 8);
            styleCardPill.fontSize = 11;
            styleCardPill.fontStyle = FontStyle.Bold;
            styleCardPill.normal.textColor = Color.white;
            styleCardPill.alignment = TextAnchor.MiddleCenter;

            styleValueLabel = new GUIStyle(GUI.skin.label);
            styleValueLabel.fontSize = 11;
            styleValueLabel.fontStyle = FontStyle.Bold;
            styleValueLabel.normal.textColor = new Color(0.12f, 0.16f, 0.22f);
            styleValueLabel.alignment = TextAnchor.MiddleRight;
            styleValueLabel.clipping = TextClipping.Overflow;

            styleCheckLabel = new GUIStyle(GUI.skin.label);
            styleCheckLabel.fontSize = 11;
            styleCheckLabel.fontStyle = FontStyle.Bold;
            styleCheckLabel.normal.textColor = new Color(0.16f, 0.22f, 0.30f);
            styleCheckLabel.alignment = TextAnchor.MiddleLeft;
            styleCheckLabel.wordWrap = true;

            styleCheckIcon = new GUIStyle(GUI.skin.button);
            styleCheckIcon.normal.background = texCheckmarkOn;
            styleCheckIcon.border = new RectOffset(4, 4, 4, 4);
            styleCheckIcon.fontSize = 12;
            styleCheckIcon.fontStyle = FontStyle.Bold;
            styleCheckIcon.normal.textColor = Color.white;
            styleCheckIcon.alignment = TextAnchor.MiddleCenter;

            styleCheckIconOff = new GUIStyle(GUI.skin.button);
            styleCheckIconOff.normal.background = texCheckmarkOff;
            styleCheckIconOff.border = new RectOffset(4, 4, 4, 4);
            styleCheckIconOff.alignment = TextAnchor.MiddleCenter;

            styleBtnReset = new GUIStyle(GUI.skin.button);
            styleBtnReset.normal.background = texBtnReset;
            styleBtnReset.border = new RectOffset(8, 8, 8, 8);
            styleBtnReset.fontSize = 11;
            styleBtnReset.fontStyle = FontStyle.Bold;
            styleBtnReset.normal.textColor = Color.white;
            styleBtnReset.alignment = TextAnchor.MiddleCenter;

            styleBtnClose = new GUIStyle(GUI.skin.button);
            styleBtnClose.normal.background = texBtnClose;
            styleBtnClose.border = new RectOffset(8, 8, 8, 8);
            styleBtnClose.fontSize = 11;
            styleBtnClose.fontStyle = FontStyle.Bold;
            styleBtnClose.normal.textColor = Color.white;
            styleBtnClose.alignment = TextAnchor.MiddleCenter;

            styleBtnApply = new GUIStyle(GUI.skin.button);
            styleBtnApply.normal.background = texBtnApply;
            styleBtnApply.border = new RectOffset(8, 8, 8, 8);
            styleBtnApply.fontSize = 11;
            styleBtnApply.fontStyle = FontStyle.Bold;
            styleBtnApply.normal.textColor = Color.white;
            styleBtnApply.alignment = TextAnchor.MiddleCenter;

            styleBtnSave = new GUIStyle(GUI.skin.button);
            styleBtnSave.normal.background = texBtnSave;
            styleBtnSave.border = new RectOffset(8, 8, 8, 8);
            styleBtnSave.fontSize = 11;
            styleBtnSave.fontStyle = FontStyle.Bold;
            styleBtnSave.normal.textColor = Color.white;
            styleBtnSave.alignment = TextAnchor.MiddleCenter;

            stylesReady = true;
        }

        void OnGUI()
        {
            if (Event.current.type == EventType.KeyDown)
            {
                if (Event.current.keyCode == KeyCode.Space) { autoDemo = !autoDemo; Event.current.Use(); }
                else if (Event.current.keyCode == KeyCode.Tab) { showDashboard = !showDashboard; Event.current.Use(); }
                else if (Event.current.keyCode == KeyCode.G && ghostArm != null)
                {
                    ghostArm.SetVisible(!ghostArm.isVisible);
                    Event.current.Use();
                }
            }

            InitModernStyles();

            if (!showDashboard)
            {
                // Sleek Open Pill in top left
                if (GUI.Button(new Rect(24, 24, 175, 34), "⚙️ Open Controls (Tab)", styleBtnApply))
                {
                    showDashboard = true;
                }
                return;
            }

            // Docked in top-left corner so robot remains completely visible in the center/right
            float panelW = Mathf.Clamp(Screen.width * 0.48f, 760f, 850f);
            float panelH = Mathf.Clamp(Screen.height * 0.74f, 440f, 510f);
            if (panelW > Screen.width - 44f) panelW = Screen.width - 44f;
            if (panelH > Screen.height - 44f) panelH = Screen.height - 44f;
            float panelX = 22f;
            float panelY = 22f;

            float sidebarW = 140f;
            float mainW = panelW - sidebarW;

            // Draw Dark Frosted Left Sidebar
            GUI.Box(new Rect(panelX, panelY, sidebarW, panelH), "", styleSidebar);

            // Draw Light Frosted Glass Main Body (transparent enough for roboarm behind!)
            GUI.Box(new Rect(panelX + sidebarW, panelY, mainW, panelH), "", styleMainPanel);

            // -------------------------------------------------------------
            // SIDEBAR CONTENT
            // -------------------------------------------------------------
            GUILayout.BeginArea(new Rect(panelX + 14, panelY + 24, sidebarW - 20, panelH - 44));

            string[] tabs = new string[] { "Controls", "Network", "Presets", "Guide" };
            for (int i = 0; i < tabs.Length; i++)
            {
                bool isSelected = (i == selectedSidebarTab);
                GUILayout.BeginHorizontal();
                if (isSelected)
                {
                    // Cyan/Blue diamond indicator matching reference image
                    GUI.color = new Color(0.18f, 0.52f, 1.0f);
                    GUILayout.Label("◆", styleTabActive, GUILayout.Width(14));
                    GUI.color = Color.white;
                    if (GUILayout.Button(tabs[i], styleTabActive, GUILayout.Height(34))) selectedSidebarTab = i;
                }
                else
                {
                    GUILayout.Space(18);
                    if (GUILayout.Button(tabs[i], styleTabInactive, GUILayout.Height(34))) selectedSidebarTab = i;
                }
                GUILayout.EndHorizontal();
                GUILayout.Space(4);
            }

            GUILayout.FlexibleSpace();

            // Bottom brand logo
            GUILayout.Label("EB-15", styleTabActive);
            GUILayout.Label("Digital Twin UI", styleSubtext);

            GUILayout.EndArea();

            // -------------------------------------------------------------
            // MAIN BODY CONTENT (3 Columns + Bottom Action Bar)
            // -------------------------------------------------------------
            float contentMargin = 16f;
            float contentW = mainW - (contentMargin * 2);
            float bottomBarH = 42f;
            float contentH = panelH - (contentMargin * 2) - bottomBarH;

            GUILayout.BeginArea(new Rect(panelX + sidebarW + contentMargin, panelY + contentMargin, contentW, contentH));

            switch (selectedSidebarTab)
            {
                case 0: DrawModernControlsTab(contentW); break;
                case 1: DrawModernNetworkTab(contentW); break;
                case 2: DrawModernPresetsTab(contentW); break;
                case 3: DrawModernGuideTab(contentW); break;
            }

            GUILayout.EndArea();

            // -------------------------------------------------------------
            // BOTTOM ACTION BAR (Matching Image Buttons)
            // -------------------------------------------------------------
            float btnY = panelY + panelH - bottomBarH - 8;
            float btnW = 82f;
            float btnGap = 8f;
            float rightEdge = panelX + panelW - contentMargin;

            // Buttons: RESET ALL, CLOSE, APPLY, SAVE
            if (GUI.Button(new Rect(rightEdge - (btnW * 4 + btnGap * 3), btnY, btnW, 30), "RESET ALL", styleBtnReset))
            {
                SnapGhostToPhysicalPose();
            }

            if (GUI.Button(new Rect(rightEdge - (btnW * 3 + btnGap * 2), btnY, btnW, 30), "CLOSE", styleBtnClose))
            {
                showDashboard = false;
            }

            if (GUI.Button(new Rect(rightEdge - (btnW * 2 + btnGap), btnY, btnW, 30), "APPLY", styleBtnApply))
            {
                if (rosBridge != null)
                {
                    if (inputRosHost != rosBridge.rosHost)
                    {
                        int.TryParse(inputListenPort, out int inP);
                        int.TryParse(inputRosPort, out int outP);
                        rosBridge.Reconnect(inputRosHost, inP > 0 ? inP : 5005, outP > 0 ? outP : 5006);
                    }
                    rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
                }
            }

            if (GUI.Button(new Rect(rightEdge - btnW, btnY, btnW, 30), "SAVE", styleBtnSave))
            {
                if (rosBridge != null)
                {
                    int.TryParse(inputListenPort, out int inP);
                    int.TryParse(inputRosPort, out int outP);
                    rosBridge.Reconnect(inputRosHost, inP > 0 ? inP : 5005, outP > 0 ? outP : 5006);
                    rosBridge.SetAutoStream(true);
                    rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
                    Debug.Log($"[EB-15 Control] SAVE: Settings and Target Pose streamed to {inputRosHost}:{outP}.");
                }
            }
        }

        private void DrawColumnHeader(string title)
        {
            GUILayout.Label(title, styleSectionHeader);
            GUILayout.Space(2);
            GUILayout.Box("", styleDivider, GUILayout.Height(1), GUILayout.ExpandWidth(true));
            GUILayout.Space(8);
        }

        private void DrawModernControlsTab(float totalW)
        {
            float colW = (totalW - 30f) / 3f;

            GUILayout.BeginHorizontal();

            // ==========================================
            // COLUMN 1: JOINT TARGETS (Ghost Preview)
            // ==========================================
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Joint Targets");

            // Live Teleoperation Streaming Pill
            if (rosBridge != null && rosBridge.streamCommandsToRos)
            {
                GUI.color = new Color(0.2f, 0.95f, 0.65f, 1f);
                if (GUILayout.Button($"● LIVE TO ROBOT ({rosBridge.TotalPacketsSent} sent)", styleCardPill, GUILayout.Height(26)))
                {
                    rosBridge.SetAutoStream(false);
                }
                GUI.color = Color.white;
            }
            else
            {
                GUI.color = new Color(1f, 0.65f, 0.2f, 1f);
                if (GUILayout.Button("⏸️ STREAM OFF (Click to Enable)", styleCardPill, GUILayout.Height(26)))
                {
                    if (rosBridge != null) rosBridge.SetAutoStream(true);
                }
                GUI.color = Color.white;
            }
            GUILayout.Space(6);

            DrawStyledSlider("Joint 1 (Base)", ref targetJoint1, -180f, 180f, "°");
            DrawStyledSlider("Joint 2 (Shoulder)", ref targetJoint2, -90f, 90f, "°");
            DrawStyledSlider("Joint 3 (Elbow)", ref targetJoint3, -154.7f, 154.7f, "°");
            DrawStyledSlider("Joint 4 (Wrist)", ref targetWrist, -90f, 90f, "°");
            DrawStyledSlider("Gripper Stroke", ref targetGripper, 0f, 1f, "%", true);

            GUILayout.EndVertical();

            GUILayout.Space(15);

            // ==========================================
            // COLUMN 2: HARDWARE TWIN & VELOCITY
            // ==========================================
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Physical Twin");

            // Encoder Pill Display
            GUILayout.Box(isHardwareEncoderLive ? "● HARDWARE ENCODERS LIVE" : "● VELOCITY SIMULATION", styleCardPill, GUILayout.Height(30));
            GUILayout.Space(6);

            DrawReadoutCard("Real Joint 1", $"{currentJoint1:F1}°", $"Lag: {Mathf.Abs(targetJoint1 - currentJoint1):F1}°");
            DrawReadoutCard("Real Joint 2", $"{currentJoint2:F1}°", $"Lag: {Mathf.Abs(targetJoint2 - currentJoint2):F1}°");
            DrawReadoutCard("Real Joint 3", $"{currentJoint3:F1}°", $"Lag: {Mathf.Abs(targetJoint3 - currentJoint3):F1}°");
            DrawReadoutCard("Real Wrist", $"{currentWrist:F1}°", $"Lag: {Mathf.Abs(targetWrist - currentWrist):F1}°");
            DrawReadoutCard("Real Gripper", $"{(currentGripper * 100f):F0}%", "");

            GUILayout.EndVertical();

            GUILayout.Space(15);

            // ==========================================
            // COLUMN 3: TOGGLES & QUICK CONTROLS
            // ==========================================
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Sync & Tools");

            if (ghostArm != null)
            {
                bool ghostVis = DrawStyledCheckbox("Hologram Ghost Preview", ghostArm.isVisible);
                if (ghostVis != ghostArm.isVisible) ghostArm.SetVisible(ghostVis);
            }

            if (rosBridge != null)
            {
                bool nextStream = DrawStyledCheckbox("Auto-Stream to Real Robot", rosBridge.streamCommandsToRos);
                if (nextStream != rosBridge.streamCommandsToRos)
                {
                    rosBridge.SetAutoStream(nextStream);
                }
            }

            autoDemo = DrawStyledCheckbox("Waypoint Demo Active", autoDemo);

            GUILayout.Space(12);
            DrawColumnHeader("Gripper Tools");

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Pinch 0%", styleBtnReset, GUILayout.Height(28))) targetGripper = 0f;
            if (GUILayout.Button("Half 50%", styleBtnReset, GUILayout.Height(28))) targetGripper = 0.5f;
            if (GUILayout.Button("Open 100%", styleBtnReset, GUILayout.Height(28))) targetGripper = 1f;
            GUILayout.EndHorizontal();

            GUILayout.Space(12);
            if (GUILayout.Button("🛑 EMERGENCY HOLD", styleBtnClose, GUILayout.Height(32)))
            {
                EmergencyHold();
            }

            GUILayout.EndVertical();

            GUILayout.EndHorizontal();
        }

        private void DrawStyledSlider(string label, ref float val, float min, float max, string unit, bool isPercent = false)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, styleSubtext);
            GUILayout.FlexibleSpace();
            float displayVal = isPercent ? (val * 100f) : val;
            GUILayout.Label($"{displayVal:F1}{unit}", styleValueLabel);
            GUILayout.EndHorizontal();

            val = GUILayout.HorizontalSlider(val, min, max);
            GUILayout.Space(5);
        }

        private void DrawReadoutCard(string title, string value, string lagText)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(title, styleSubtext, GUILayout.Width(58));
            GUILayout.FlexibleSpace();
            if (!string.IsNullOrEmpty(lagText))
            {
                GUI.color = new Color(0.18f, 0.46f, 0.88f);
                GUILayout.Label(lagText, styleSubtext);
                GUI.color = Color.white;
                GUILayout.Space(4);
            }
            GUILayout.Label(value, styleValueLabel);
            GUILayout.Space(12);
            GUILayout.EndHorizontal();
            GUILayout.Space(4);
        }

        private bool DrawStyledCheckbox(string label, bool current)
        {
            GUILayout.BeginHorizontal();
            bool boxClicked = GUILayout.Button(current ? "✔" : "", current ? styleCheckIcon : styleCheckIconOff, GUILayout.Width(22), GUILayout.Height(22));
            GUILayout.Space(8);
            bool labelClicked = GUILayout.Button(label, styleCheckLabel);
            GUILayout.EndHorizontal();
            GUILayout.Space(6);

            return (boxClicked || labelClicked) ? !current : current;
        }

        private void DrawModernNetworkTab(float totalW)
        {
            float colW = (totalW - 24f) / 3f;

            GUILayout.BeginHorizontal();

            // Column 1: Remote Host IP
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("ROS 2 Remote Host");
            GUILayout.Label("Robot Host IP Address:", styleSubtext);
            inputRosHost = GUILayout.TextField(inputRosHost, GUILayout.Height(28));

            GUILayout.Space(8);
            GUILayout.Label("Quick IP Presets:", styleSubtext);
            if (GUILayout.Button("Localhost (127.0.0.1)", styleCardPill, GUILayout.Height(26))) inputRosHost = "127.0.0.1";
            GUILayout.Space(4);
            if (GUILayout.Button("WSL2 (127.0.0.1)", styleCardPill, GUILayout.Height(26))) inputRosHost = "127.0.0.1";
            GUILayout.EndVertical();

            GUILayout.Space(12);

            // Column 2: Ports
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("UDP Ports");

            GUILayout.Label("Listen Port (Telemetry):", styleSubtext);
            inputListenPort = GUILayout.TextField(inputListenPort, GUILayout.Height(28));

            GUILayout.Space(6);
            GUILayout.Label("Command Port (ROS):", styleSubtext);
            inputRosPort = GUILayout.TextField(inputRosPort, GUILayout.Height(28));

            GUILayout.Space(8);
            if (GUILayout.Button("🔄 Reconnect", styleBtnApply, GUILayout.Height(30)))
            {
                if (int.TryParse(inputListenPort, out int inP) && int.TryParse(inputRosPort, out int outP))
                {
                    if (rosBridge != null)
                    {
                        rosBridge.Reconnect(inputRosHost, inP, outP);
                        networkStatusMsg = $"✓ Connected to\n{inputRosHost}:{outP}";
                    }
                }
                else
                {
                    networkStatusMsg = "❌ Error: Port must be an integer.";
                }
            }

            if (!string.IsNullOrEmpty(networkStatusMsg))
            {
                GUILayout.Space(4);
                GUILayout.Label(networkStatusMsg, styleSubtext, GUILayout.Width(colW));
            }

            GUILayout.EndVertical();

            GUILayout.Space(12);

            // Column 3: Live Diagnostics
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Diagnostics");

            if (rosBridge != null)
            {
                DrawReadoutCard("Status", rosBridge.IsConnected ? "Connected" : "Listening", "");
                DrawReadoutCard("In Rate", $"{rosBridge.PacketsPerSecond:F1} Hz", "");
                DrawReadoutCard("In Packets", $"{rosBridge.TotalPacketsReceived}", "");
                DrawReadoutCard("Out Sent", $"{rosBridge.TotalPacketsSent}", "");
                DrawReadoutCard("Encoders", rosBridge.IsReceivingRealEncoders ? "Active" : "Awaiting", "");
                string lastTime = rosBridge.LastPacketTimestamp;
                if (!string.IsNullOrEmpty(lastTime) && lastTime.Contains(".") && lastTime.Length > 8)
                {
                    lastTime = lastTime.Substring(0, 8);
                }
                DrawReadoutCard("Last Recv", lastTime, "");
            }

            GUILayout.EndVertical();

            GUILayout.EndHorizontal();
        }

        private void DrawModernPresetsTab(float totalW)
        {
            float colW = (totalW - 30f) / 3f;

            GUILayout.BeginHorizontal();

            // Column 1: Core Poses
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Standard Poses");

            if (GUILayout.Button("🏠 Home (Upright)", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(0f, 0f, 0f, 0f, 0.5f);
            GUILayout.Space(4);

            if (GUILayout.Button("📦 Reach Forward", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(45f, -35f, 50f, 25f, 1f);
            GUILayout.Space(4);

            if (GUILayout.Button("🎯 Pick Object", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(45f, -55f, 75f, 35f, 0f);
            GUILayout.EndVertical();

            GUILayout.Space(15);

            // Column 2: Manipulation Poses
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Place & Zero");

            if (GUILayout.Button("⬆️ Lift Object Up", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(45f, -25f, 35f, 15f, 0f);
            GUILayout.Space(4);

            if (GUILayout.Button("⬅️ Place Left", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(-50f, -55f, 75f, -35f, 1f);
            GUILayout.Space(4);

            if (GUILayout.Button("📐 Zero Pose", styleCardPill, GUILayout.Height(30)))
                ApplyPresetPose(0f, 0f, 0f, 0f, 0f);
            GUILayout.EndVertical();

            GUILayout.Space(15);

            // Column 3: Automated Demo
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Sequencer");

            if (GUILayout.Button(autoDemo ? "⏸️ Pause Demo" : "▶️ Play Demo", styleBtnApply, GUILayout.Height(32)))
            {
                autoDemo = !autoDemo;
            }

            GUILayout.Space(8);
            GUILayout.Label($"Speed Multiplier: {demoSpeed:F2}x", styleSubtext);
            demoSpeed = GUILayout.HorizontalSlider(demoSpeed, 0.2f, 2.0f);
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();
        }

        private void DrawModernGuideTab(float totalW)
        {
            float colW = (totalW - 24f) / 2f;

            GUILayout.BeginHorizontal();

            // Column 1: Digital Twin Architecture & ROS Command
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Digital Twin Flow");
            GUILayout.Label("• <b>Cyan Ghost:</b> Commanded target pose set by user sliders or trajectory planner.\n\n" +
                            "• <b>Solid Arm:</b> Physical twin driven strictly by hardware optical encoders at real motor velocity.", styleBodyText, GUILayout.Width(colW));

            GUILayout.Space(14);
            DrawColumnHeader("Remote ROS 2 Driver");
            GUILayout.Label("Run on Linux PC / Robot host:", styleSubtext);
            GUILayout.Space(4);
            GUILayout.Label("ros2 launch eb15_driver hardware_control.launch.py\nunity_ip:=<THIS_PC_IP> gui:=false feedback:=true", styleCodeBox, GUILayout.Width(colW));
            GUILayout.EndVertical();

            GUILayout.Space(24);

            // Column 2: Keyboard Shortcuts & Tips
            GUILayout.BeginVertical(GUILayout.Width(colW));
            DrawColumnHeader("Keyboard Shortcuts");

            DrawShortcutRow("Tab", "Toggle Dashboard Window");
            DrawShortcutRow("G", "Toggle Hologram Ghost Preview");
            DrawShortcutRow("Space", "Play / Pause Waypoint Demo");
            DrawShortcutRow("H", "Snap Directly to Home Pose");

            GUILayout.Space(14);
            DrawColumnHeader("Operational Tips");
            GUILayout.Label("• Adjust sliders to inspect reach before sending to physical arm.\n\n" +
                            "• Click <b>APPLY</b> or <b>SAVE</b> to stream goals to ROS 2 controller.", styleBodyText, GUILayout.Width(colW));
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();
        }

        private void DrawShortcutRow(string key, string description)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Box(key, styleCardPill, GUILayout.Width(46), GUILayout.Height(22));
            GUILayout.Space(8);
            GUILayout.Label(description, styleSubtext, GUILayout.Height(22));
            GUILayout.EndHorizontal();
            GUILayout.Space(3);
        }
    }
}

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
    /// Manages the Physical Twin (Solid Arm) and Commanded Goal (Ghost Preview).
    /// Provides an intuitive, self-explanatory UI with runtime IP/port configuration for remote ROS PCs.
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
        public float driveStiffness = 20000f;
        public float driveDamping = 350f;
        public float driveForceLimit = 1500f;

        [Header("Operation Mode")]
        public bool autoDemo = false;
        public float demoSpeed = 0.6f;

        [Header("Hardware Encoder Status")]
        public bool isHardwareEncoderLive = false;
        private float lastEncoderReceiveTime = -10f;

        [Header("UI Dashboard Settings")]
        public bool showDashboard = true;
        public bool isMinimized = false;
        private Rect windowRect = new Rect(20, 20, 450, 680);
        private int selectedTab = 0; // 0: Controls, 1: Network, 2: Presets, 3: Help
        private Vector2 scrollPos = Vector2.zero;

        // Network input fields (for editing in UI)
        private string inputRosHost = "127.0.0.1";
        private string inputListenPort = "5005";
        private string inputRosPort = "5006";
        private string networkStatusMsg = "";

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

        // Custom GUI styles
        private GUIStyle winStyle;
        private GUIStyle headerStyle;
        private GUIStyle cardStyle;
        private GUIStyle tabActiveStyle;
        private GUIStyle tabInactiveStyle;
        private GUIStyle labelBold;
        private GUIStyle labelDim;
        private GUIStyle btnAccent;
        private GUIStyle btnDanger;
        private Texture2D texDark;
        private Texture2D texCard;
        private Texture2D texActiveTab;
        private Texture2D texAccent;
        private Texture2D texDanger;
        private bool stylesInitialized = false;

        void Awake()
        {
            Application.runInBackground = true;
            FindJoints();
            ConfigureJointDrives();

            var baseLink = transform.Find("world/base_link");
            if (baseLink != null)
            {
                var baseAb = baseLink.GetComponent<ArticulationBody>();
                if (baseAb != null) baseAb.immovable = true;
            }

            if (ghostArm == null)
            {
                ghostArm = FindFirstObjectByType<GhostArmPreview>();
            }

            rosBridge = GetComponent<RosRoboArmBridge>();
            if (rosBridge != null)
            {
                inputRosHost = rosBridge.rosHost;
                inputListenPort = rosBridge.listenPort.ToString();
                inputRosPort = rosBridge.rosPort.ToString();
            }
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

        private Texture2D MakeTex(int width, int height, Color col)
        {
            Color[] pix = new Color[width * height];
            for (int i = 0; i < pix.Length; i++) pix[i] = col;
            Texture2D result = new Texture2D(width, height);
            result.SetPixels(pix);
            result.Apply();
            return result;
        }

        private void InitStyles()
        {
            if (stylesInitialized) return;

            texDark = MakeTex(2, 2, new Color(0.10f, 0.12f, 0.16f, 0.95f));
            texCard = MakeTex(2, 2, new Color(0.15f, 0.18f, 0.24f, 0.90f));
            texActiveTab = MakeTex(2, 2, new Color(0.0f, 0.55f, 0.9f, 0.95f));
            texAccent = MakeTex(2, 2, new Color(0.0f, 0.75f, 0.55f, 0.95f));
            texDanger = MakeTex(2, 2, new Color(0.85f, 0.2f, 0.2f, 0.95f));

            winStyle = new GUIStyle(GUI.skin.window);
            winStyle.normal.background = texDark;
            winStyle.focused.background = texDark;
            winStyle.onNormal.background = texDark;
            winStyle.border = new RectOffset(4, 4, 4, 4);
            winStyle.padding = new RectOffset(12, 12, 8, 8);

            headerStyle = new GUIStyle(GUI.skin.label);
            headerStyle.fontSize = 13;
            headerStyle.fontStyle = FontStyle.Bold;
            headerStyle.normal.textColor = Color.white;

            cardStyle = new GUIStyle(GUI.skin.box);
            cardStyle.normal.background = texCard;
            cardStyle.padding = new RectOffset(10, 10, 8, 8);

            tabActiveStyle = new GUIStyle(GUI.skin.button);
            tabActiveStyle.normal.background = texActiveTab;
            tabActiveStyle.fontStyle = FontStyle.Bold;
            tabActiveStyle.normal.textColor = Color.white;

            tabInactiveStyle = new GUIStyle(GUI.skin.button);
            tabInactiveStyle.normal.textColor = new Color(0.8f, 0.85f, 0.9f);

            labelBold = new GUIStyle(GUI.skin.label);
            labelBold.fontStyle = FontStyle.Bold;
            labelBold.normal.textColor = Color.white;

            labelDim = new GUIStyle(GUI.skin.label);
            labelDim.fontSize = 11;
            labelDim.normal.textColor = new Color(0.7f, 0.75f, 0.82f);

            btnAccent = new GUIStyle(GUI.skin.button);
            btnAccent.normal.background = texAccent;
            btnAccent.fontStyle = FontStyle.Bold;
            btnAccent.normal.textColor = Color.white;

            btnDanger = new GUIStyle(GUI.skin.button);
            btnDanger.normal.background = texDanger;
            btnDanger.fontStyle = FontStyle.Bold;
            btnDanger.normal.textColor = Color.white;

            stylesInitialized = true;
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

            if (!showDashboard)
            {
                if (GUI.Button(new Rect(20, 20, 150, 34), "📊 Open Robot Center"))
                {
                    showDashboard = true;
                }
                return;
            }

            InitStyles();

            // Minimized Floating Pill Mode
            if (isMinimized)
            {
                GUI.Box(new Rect(20, 20, 360, 48), "", cardStyle);
                GUILayout.BeginArea(new Rect(25, 26, 350, 40));
                GUILayout.BeginHorizontal();
                DrawStatusBadge(false);
                if (GUILayout.Button("Expand ↗", GUILayout.Width(75))) isMinimized = false;
                GUILayout.EndHorizontal();
                GUILayout.EndArea();
                return;
            }

            // Full Master Dashboard Window
            windowRect = GUI.Window(999, windowRect, DrawDashboardWindow, "🤖 EB-15 ROBOTIC ARM — DIGITAL TWIN DASHBOARD", winStyle);
        }

        private void DrawDashboardWindow(int windowID)
        {
            // Top Window Bar (Status Badge & Minimize Button)
            GUILayout.BeginHorizontal();
            DrawStatusBadge(true);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button(ghostArm != null && ghostArm.isVisible ? "👻 Ghost: ON" : "👻 Ghost: OFF", GUILayout.Width(95)))
            {
                if (ghostArm != null) ghostArm.SetVisible(!ghostArm.isVisible);
            }
            if (GUILayout.Button("-", GUILayout.Width(26))) isMinimized = true;
            if (GUILayout.Button("✕", GUILayout.Width(26))) showDashboard = false;
            GUILayout.EndHorizontal();

            GUILayout.Space(6);

            // Tab Navigation Bar
            string[] tabs = new string[] { "🎮 Controls", "🌐 Network & IP", "📐 Presets", "ℹ️ Guide" };
            GUILayout.BeginHorizontal();
            for (int i = 0; i < tabs.Length; i++)
            {
                GUIStyle style = (i == selectedTab) ? tabActiveStyle : tabInactiveStyle;
                if (GUILayout.Button(tabs[i], style, GUILayout.Height(30)))
                {
                    selectedTab = i;
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(8);

            scrollPos = GUILayout.BeginScrollView(scrollPos);

            switch (selectedTab)
            {
                case 0: DrawControlsTab(); break;
                case 1: DrawNetworkTab(); break;
                case 2: DrawPresetsTab(); break;
                case 3: DrawGuideTab(); break;
            }

            GUILayout.EndScrollView();

            GUI.DragWindow(new Rect(0, 0, 10000, 30));
        }

        private void DrawStatusBadge(bool showDetail)
        {
            Color prevColor = GUI.color;
            if (isHardwareEncoderLive)
            {
                GUI.color = new Color(0.2f, 1f, 0.4f);
                GUILayout.Label(showDetail ? "● REAL HARDWARE ENCODERS LIVE (50 Hz)" : "● ENCODERS LIVE", labelBold);
            }
            else if (rosBridge != null && rosBridge.IsConnected)
            {
                GUI.color = new Color(0.3f, 0.85f, 1f);
                GUILayout.Label(showDetail ? "● ROS 2 CONNECTED (Telemetry Active)" : "● ROS 2 ACTIVE", labelBold);
            }
            else
            {
                GUI.color = new Color(1f, 0.7f, 0.2f);
                GUILayout.Label(showDetail ? "● VELOCITY-MATCHED SIMULATION (Offline)" : "● SIMULATION", labelBold);
            }
            GUI.color = prevColor;
        }

        private void DrawControlsTab()
        {
            // Interactive banner explaining the twin concept
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("💡 DIGITAL TWIN CONCEPT", labelBold);
            GUILayout.Label("• 👻 Cyan Ghost: Commanded Target Pose (Drag sliders below to set goal)", labelDim);
            GUILayout.Label("• 🦾 Solid Arm: Physical Robot (Tracks real optical encoders at hardware speed)", labelDim);
            GUILayout.EndVertical();

            GUILayout.Space(6);

            // Joint Sliders with anatomical explanations
            float prevJ1 = targetJoint1;
            float prevJ2 = targetJoint2;
            float prevJ3 = targetJoint3;
            float prevWrist = targetWrist;
            float prevGrip = targetGripper;

            DrawJointCard("Joint 1: Base Rotate (Yaw)", "Rotates the entire arm left / right (-180° to +180°)", ref targetJoint1, currentJoint1, -180f, 180f, "°");
            DrawJointCard("Joint 2: Shoulder Reach (Pitch)", "Tilts the main upper arm forward / back (-90° to +90°)", ref targetJoint2, currentJoint2, -90f, 90f, "°");
            DrawJointCard("Joint 3: Elbow Arm (Pitch)", "Raises / lowers the forearm (-154.7° to +154.7°)", ref targetJoint3, currentJoint3, -154.7f, 154.7f, "°");
            DrawJointCard("Joint 4: Wrist Tilt (Pitch)", "Angles the end-effector gripper up / down (-90° to +90°)", ref targetWrist, currentWrist, -90f, 90f, "°");

            // Gripper
            GUILayout.BeginVertical(cardStyle);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Joint 5: Parallel Gripper", labelBold);
            GUILayout.FlexibleSpace();
            GUILayout.Label($"Target: {(targetGripper * 100f):F0}% | Real: {(currentGripper * 100f):F0}%", labelDim);
            GUILayout.EndHorizontal();
            GUILayout.Label("Opens and closes the dual parallel fingers (0% Closed, 100% Fully Open)", labelDim);
            targetGripper = GUILayout.HorizontalSlider(targetGripper, 0f, 1f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Pinch Closed (0%)")) targetGripper = 0f;
            if (GUILayout.Button("Half (50%)")) targetGripper = 0.5f;
            if (GUILayout.Button("Fully Open (100%)")) targetGripper = 1f;
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            // Check if user moved any slider
            bool changed = Mathf.Abs(targetJoint1 - prevJ1) > 0.01f ||
                           Mathf.Abs(targetJoint2 - prevJ2) > 0.01f ||
                           Mathf.Abs(targetJoint3 - prevJ3) > 0.01f ||
                           Mathf.Abs(targetWrist - prevWrist) > 0.01f ||
                           Mathf.Abs(targetGripper - prevGrip) > 0.005f;

            if (changed && rosBridge != null && rosBridge.streamCommandsToRos)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }

            GUILayout.Space(8);

            // Action Toolbar
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("⚡ DISPATCH & SYNC CONTROLS", labelBold);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("🧲 Snap Ghost to Arm", GUILayout.Height(32)))
            {
                SnapGhostToPhysicalPose();
            }
            if (GUILayout.Button("🚀 Send Goal Now", btnAccent, GUILayout.Height(32)))
            {
                if (rosBridge != null)
                {
                    rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (rosBridge != null)
            {
                rosBridge.streamCommandsToRos = GUILayout.Toggle(rosBridge.streamCommandsToRos, " Auto-Stream Sliders to Real Arm");
            }
            if (GUILayout.Button("🛑 Emergency Hold", btnDanger, GUILayout.Width(140)))
            {
                EmergencyHold();
            }
            GUILayout.EndHorizontal();

            // Tracking delta / lag
            float lag = Mathf.Abs(targetJoint1 - currentJoint1) +
                        Mathf.Abs(targetJoint2 - currentJoint2) +
                        Mathf.Abs(targetJoint3 - currentJoint3) +
                        Mathf.Abs(targetWrist - currentWrist);
            GUILayout.Label($"Total Goal-to-Hardware Delta: {lag:F1}° (Physical arm catching up)", labelDim);

            GUILayout.EndVertical();
        }

        private void DrawJointCard(string title, string description, ref float targetVal, float currentVal, float min, float max, string unit)
        {
            GUILayout.BeginVertical(cardStyle);
            GUILayout.BeginHorizontal();
            GUILayout.Label(title, labelBold);
            GUILayout.FlexibleSpace();
            float lag = Mathf.Abs(targetVal - currentVal);
            Color lagColor = lag > 1.0f ? new Color(1f, 0.75f, 0.3f) : new Color(0.4f, 1f, 0.5f);
            Color old = GUI.color;
            GUI.color = lagColor;
            GUILayout.Label($"Goal: {targetVal:F1}{unit} | Real: {currentVal:F1}{unit}", labelBold);
            GUI.color = old;
            GUILayout.EndHorizontal();

            GUILayout.Label(description, labelDim);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("-5°", GUILayout.Width(38))) targetVal = Mathf.Clamp(targetVal - 5f, min, max);
            if (GUILayout.Button("-1°", GUILayout.Width(38))) targetVal = Mathf.Clamp(targetVal - 1f, min, max);
            targetVal = GUILayout.HorizontalSlider(targetVal, min, max);
            if (GUILayout.Button("+1°", GUILayout.Width(38))) targetVal = Mathf.Clamp(targetVal + 1f, min, max);
            if (GUILayout.Button("+5°", GUILayout.Width(38))) targetVal = Mathf.Clamp(targetVal + 5f, min, max);
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            GUILayout.Space(2);
        }

        private void DrawNetworkTab()
        {
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("🌐 ROS 2 REMOTE / LOCAL CONNECTION", labelBold);
            GUILayout.Label("Configure connection when the ROS 2 driver is running on another PC, laptop, or inside WSL2.", labelDim);
            GUILayout.EndVertical();

            GUILayout.Space(6);

            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("Remote ROS 2 Machine IP Address:", labelBold);
            inputRosHost = GUILayout.TextField(inputRosHost, 40);

            GUILayout.Label("Quick IP Presets:", labelDim);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("🖥️ Localhost (127.0.0.1)")) inputRosHost = "127.0.0.1";
            if (GUILayout.Button("🐧 WSL2 (127.0.0.1)")) inputRosHost = "127.0.0.1";
            if (GUILayout.Button("🌐 Clear IP")) inputRosHost = "";
            GUILayout.EndHorizontal();

            GUILayout.Space(8);
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Label("Incoming Port (Unity Listen):", labelBold);
            inputListenPort = GUILayout.TextField(inputListenPort);
            GUILayout.EndVertical();

            GUILayout.BeginVertical();
            GUILayout.Label("Outgoing Port (ROS Listen):", labelBold);
            inputRosPort = GUILayout.TextField(inputRosPort);
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            if (GUILayout.Button("🔄 Reconnect & Apply Network Settings", btnAccent, GUILayout.Height(36)))
            {
                if (int.TryParse(inputListenPort, out int inP) && int.TryParse(inputRosPort, out int outP))
                {
                    if (rosBridge != null)
                    {
                        rosBridge.Reconnect(inputRosHost, inP, outP);
                        networkStatusMsg = $"✓ Successfully rebound UDP sockets: Host={inputRosHost}, Ports={inP}/{outP}";
                    }
                }
                else
                {
                    networkStatusMsg = "❌ Error: Port numbers must be valid integers.";
                }
            }

            if (!string.IsNullOrEmpty(networkStatusMsg))
            {
                GUILayout.Space(4);
                GUILayout.Label(networkStatusMsg, labelDim);
            }

            GUILayout.EndVertical();

            GUILayout.Space(6);

            // Live Diagnostics Card
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("📊 LIVE NETWORK DIAGNOSTICS", labelBold);
            if (rosBridge != null)
            {
                GUILayout.Label($"• Target ROS Host: {rosBridge.rosHost}:{rosBridge.rosPort}", labelDim);
                GUILayout.Label($"• Unity Listen Socket: 0.0.0.0:{rosBridge.listenPort} (UDP)", labelDim);
                GUILayout.Label($"• Telemetry Receive Rate: {rosBridge.PacketsPerSecond:F1} packets/sec", labelDim);
                GUILayout.Label($"• Total Packets Received: {rosBridge.TotalPacketsReceived}", labelDim);
                GUILayout.Label($"• Last Packet Received: {rosBridge.LastPacketTimestamp}", labelDim);
                GUILayout.Label($"• Real Encoders Active: {(rosBridge.IsReceivingRealEncoders ? "YES (Hardware Stream)" : "Awaiting Hardware Feedback")}", labelDim);
            }
            GUILayout.EndVertical();

            GUILayout.Space(6);
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("💡 REMOTE HOST COMMAND TIP", labelBold);
            GUILayout.Label("If ROS is running on another Linux PC, start it with your Windows PC's IP:", labelDim);
            GUILayout.TextArea("ros2 launch eb15_driver hardware_control.launch.py unity_ip:=<THIS_PC_IP>");
            GUILayout.EndVertical();
        }

        private void DrawPresetsTab()
        {
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("🎯 1-CLICK POSE PRESETS", labelBold);
            GUILayout.Label("Click any preset to command the Ghost preview and send goal to the arm.", labelDim);

            GUILayout.Space(6);
            if (GUILayout.Button("🏠 Home Position (Upright Standby)", GUILayout.Height(32)))
                ApplyPresetPose(0f, 0f, 0f, 0f, 0.5f);

            if (GUILayout.Button("📦 Reach Forward Right (Preparation)", GUILayout.Height(32)))
                ApplyPresetPose(45f, -35f, 50f, 25f, 1f);

            if (GUILayout.Button("🎯 Pick Object (Grip Closed)", GUILayout.Height(32)))
                ApplyPresetPose(45f, -55f, 75f, 35f, 0f);

            if (GUILayout.Button("⬆️ Lift Object Upward", GUILayout.Height(32)))
                ApplyPresetPose(45f, -25f, 35f, 15f, 0f);

            if (GUILayout.Button("⬅️ Swing & Place Target (Left)", GUILayout.Height(32)))
                ApplyPresetPose(-50f, -55f, 75f, -35f, 1f);

            if (GUILayout.Button("🔄 Retract to Safe Standby", GUILayout.Height(32)))
                ApplyPresetPose(-50f, -10f, 20f, 0f, 1f);

            if (GUILayout.Button("📐 Zero Calibration Pose (All 0°)", GUILayout.Height(32)))
                ApplyPresetPose(0f, 0f, 0f, 0f, 0f);

            GUILayout.EndVertical();

            GUILayout.Space(6);

            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("▶️ AUTONOMOUS WAYPOINT SEQUENCE", labelBold);
            GUILayout.Label("Cycles the arm through a continuous automated pick-and-place sequence.", labelDim);

            GUILayout.Space(4);
            if (GUILayout.Button(autoDemo ? "⏸️ Pause Demo" : "▶️ Start Continuous Demo", autoDemo ? btnDanger : btnAccent, GUILayout.Height(34)))
            {
                autoDemo = !autoDemo;
            }

            GUILayout.Label($"Demo Speed Multiplier: {demoSpeed:F2}x", labelDim);
            demoSpeed = GUILayout.HorizontalSlider(demoSpeed, 0.2f, 2.0f);
            GUILayout.EndVertical();
        }

        private void DrawGuideTab()
        {
            GUILayout.BeginVertical(cardStyle);
            GUILayout.Label("📖 USER GUIDE & SYSTEM REFERENCE", labelBold);
            GUILayout.Space(4);

            GUILayout.Label("1. Ghost vs Physical Twin", labelBold);
            GUILayout.Label("• The Cyan Holographic Ghost shows where you are instructing the arm to move.\n" +
                            "• The Solid Arm reflects where the robot arm actually is right now in the real world, based on optical encoder feedback.", labelDim);

            GUILayout.Space(4);
            GUILayout.Label("2. Velocity-Matched Physics", labelBold);
            GUILayout.Label("The robot arm in Unity enforces the exact same physical stepper motor speeds (57.3°/s base, 45.8°/s shoulder, 57.3°/s elbow) so simulation time perfectly equals real-world movement time.", labelDim);

            GUILayout.Space(4);
            GUILayout.Label("3. Remote PC Setup", labelBold);
            GUILayout.Label("Go to the 'Network & IP' tab, enter the remote Linux machine's IP, and click 'Apply & Reconnect'. The UDP socket will connect without restarting Unity.", labelDim);

            GUILayout.Space(4);
            GUILayout.Label("4. Keyboard Hotkeys", labelBold);
            GUILayout.Label("• Tab: Hide / Show Dashboard\n" +
                            "• G: Toggle Ghost Hologram Visibility\n" +
                            "• Space: Play / Pause Waypoint Demo\n" +
                            "• H: Snap to Home Position", labelDim);

            GUILayout.EndVertical();
        }
    }
}

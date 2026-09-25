using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace RoboArm
{
    /// <summary>
    /// High-performance UDP Bridge connecting Unity Digital Twin with ROS 2 (eb15_ws).
    /// Receives joint state telemetry from ROS 2 (/joint_states or /eb15/measured_joint_states)
    /// and drives the EB15 ArticulationBody joints in real time.
    /// Can also send teleoperation/pose commands back to ROS 2 (/eb15/joint_commands).
    /// </summary>
    [RequireComponent(typeof(ArmSimulationController))]
    public class RosRoboArmBridge : MonoBehaviour
    {
        [Header("ROS 2 Network Configuration")]
        [Tooltip("Port on which Unity listens for incoming ROS 2 joint states (UDP).")]
        public int listenPort = 5005;

        [Tooltip("ROS 2 machine IP address for sending commands back.")]
        public string rosHost = "127.0.0.1";

        [Tooltip("ROS 2 port for incoming joint commands from Unity (UDP).")]
        public int rosPort = 5006;

        [Header("Bridge Control Mode")]
        [Tooltip("Enable overriding Unity arm target positions with ROS 2 joint states.")]
        public bool enableRosControl = true;

        [Tooltip("When ROS packets are received, automatically pause the built-in auto demo.")]
        public bool autoPauseDemoOnRosData = true;

        [Tooltip("Smooth joint motion interpolation.")]
        public bool smoothMotion = true;

        [Tooltip("Speed multiplier for smoothing (higher = faster response).")]
        public float smoothSpeed = 25f;

        [Tooltip("Send current Unity joint targets to ROS 2 when changed.")]
        public bool streamCommandsToRos = false;

        [Header("Controller Reference")]
        public ArmSimulationController armController;

        [Header("Diagnostics & Telemetry")]
        [SerializeField] private bool isConnected = false;
        [SerializeField] private float packetsPerSecond = 0f;
        [SerializeField] private int totalPacketsReceived = 0;
        [SerializeField] private string lastPacketTimestamp = "Never";
        public bool showHUD = true;

        // Data packet structure matching unity_bridge_node.py
        [Serializable]
        public class RosJointPacket
        {
            public float joint1;
            public float joint2;
            public float joint3;
            public float wrist_joint;
            public float gripper_left_joint;
            public float gripper_right_joint;
        }

        // Thread-safe state buffer
        private readonly object stateLock = new object();
        private RosJointPacket latestPacket = null;
        private bool hasNewPacket = false;
        private float lastReceiveTime = 0f;
        private int packetCounter = 0;
        private float fpsTimer = 0f;

        // Target angles in degrees / normalized gripper
        private float desiredJ1 = 0f;
        private float desiredJ2 = 0f;
        private float desiredJ3 = 0f;
        private float desiredWrist = 0f;
        private float desiredGripper = 0.5f;

        // Sockets and background thread
        private UdpClient udpReceiver;
        private UdpClient udpSender;
        private Thread receiveThread;
        private volatile bool isRunning = false;

        void Awake()
        {
            if (armController == null)
            {
                armController = GetComponent<ArmSimulationController>();
            }
        }

        void OnEnable()
        {
            StartBridge();
        }

        void OnDisable()
        {
            StopBridge();
        }

        void OnDestroy()
        {
            StopBridge();
        }

        private void StartBridge()
        {
            if (isRunning) return;

            try
            {
                udpReceiver = new UdpClient();
                udpReceiver.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                udpReceiver.Client.Bind(new IPEndPoint(IPAddress.Any, listenPort));
                udpReceiver.Client.ReceiveTimeout = 500; // ms

                udpSender = new UdpClient();

                isRunning = true;
                receiveThread = new Thread(ReceiveWorkerLoop)
                {
                    IsBackground = true,
                    Name = "ROS2_UDP_Receiver"
                };
                receiveThread.Start();

                Debug.Log($"[RosRoboArmBridge] Started listening on UDP port {listenPort}. Target ROS host: {rosHost}:{rosPort}");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[RosRoboArmBridge] Failed to initialize UDP sockets: {ex.Message}");
            }
        }

        private void StopBridge()
        {
            isRunning = false;

            if (receiveThread != null && receiveThread.IsAlive)
            {
                receiveThread.Abort();
                receiveThread = null;
            }

            if (udpReceiver != null)
            {
                try { udpReceiver.Close(); } catch { }
                udpReceiver = null;
            }

            if (udpSender != null)
            {
                try { udpSender.Close(); } catch { }
                udpSender = null;
            }

            isConnected = false;
        }

        private void ReceiveWorkerLoop()
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

            while (isRunning)
            {
                try
                {
                    byte[] data = udpReceiver.Receive(ref remoteEndPoint);
                    if (data != null && data.Length > 0)
                    {
                        string json = Encoding.UTF8.GetString(data);
                        RosJointPacket packet = JsonUtility.FromJson<RosJointPacket>(json);

                        if (packet != null)
                        {
                            lock (stateLock)
                            {
                                latestPacket = packet;
                                hasNewPacket = true;
                                packetCounter++;
                            }
                        }
                    }
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode != SocketError.TimedOut && ex.SocketErrorCode != SocketError.Interrupted)
                    {
                        // Ignore periodic timeout to allow loop check
                    }
                }
                catch (ThreadAbortException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[RosRoboArmBridge] Receive parsing error: {ex.Message}");
                }
            }
        }

        void Update()
        {
            // Update connection diagnostics
            fpsTimer += Time.deltaTime;
            if (fpsTimer >= 1.0f)
            {
                lock (stateLock)
                {
                    packetsPerSecond = packetCounter / fpsTimer;
                    totalPacketsReceived += packetCounter;
                    packetCounter = 0;
                }
                fpsTimer = 0f;
            }

            bool receivedThisFrame = false;
            RosJointPacket packetToApply = null;

            lock (stateLock)
            {
                if (hasNewPacket)
                {
                    packetToApply = latestPacket;
                    hasNewPacket = false;
                    receivedThisFrame = true;
                    lastReceiveTime = Time.realtimeSinceStartup;
                    lastPacketTimestamp = DateTime.Now.ToString("HH:mm:ss.fff");
                }
            }

            // Consider connected if packet received within last 1.5 seconds
            isConnected = (Time.realtimeSinceStartup - lastReceiveTime) < 1.5f && totalPacketsReceived > 0;

            if (receivedThisFrame && packetToApply != null && enableRosControl)
            {
                if (autoPauseDemoOnRosData && armController != null && armController.autoDemo)
                {
                    armController.autoDemo = false;
                    Debug.Log("[RosRoboArmBridge] Received ROS 2 joint state -> Disabling autonomous demo to prioritize ROS control.");
                }

                // Convert ROS units (radians) to Unity degrees
                desiredJ1 = packetToApply.joint1 * Mathf.Rad2Deg;
                desiredJ2 = packetToApply.joint2 * Mathf.Rad2Deg;
                desiredJ3 = packetToApply.joint3 * Mathf.Rad2Deg;
                desiredWrist = packetToApply.wrist_joint * Mathf.Rad2Deg;

                // Gripper in ROS: range is [-0.016m, 0.012m].
                // Unity targetGripper is normalized [0, 1].
                desiredGripper = Mathf.InverseLerp(-0.016f, 0.012f, packetToApply.gripper_left_joint);
            }

            // Apply to arm controller
            if (enableRosControl && isConnected && armController != null)
            {
                if (smoothMotion)
                {
                    float factor = Mathf.Clamp01(smoothSpeed * Time.deltaTime);
                    armController.targetJoint1 = Mathf.Lerp(armController.targetJoint1, desiredJ1, factor);
                    armController.targetJoint2 = Mathf.Lerp(armController.targetJoint2, desiredJ2, factor);
                    armController.targetJoint3 = Mathf.Lerp(armController.targetJoint3, desiredJ3, factor);
                    armController.targetWrist = Mathf.Lerp(armController.targetWrist, desiredWrist, factor);
                    armController.targetGripper = Mathf.Lerp(armController.targetGripper, desiredGripper, factor);
                }
                else
                {
                    armController.targetJoint1 = desiredJ1;
                    armController.targetJoint2 = desiredJ2;
                    armController.targetJoint3 = desiredJ3;
                    armController.targetWrist = desiredWrist;
                    armController.targetGripper = desiredGripper;
                }
            }

            // Stream commands back to ROS if enabled
            if (streamCommandsToRos && armController != null)
            {
                SendCurrentPoseToRos();
            }
        }

        /// <summary>
        /// Sends the current Unity arm targets to ROS 2 on port 5006 as a JSON command.
        /// </summary>
        public void SendCurrentPoseToRos()
        {
            if (armController == null || udpSender == null) return;

            try
            {
                // Convert Unity degrees -> ROS radians
                var pkt = new RosJointPacket
                {
                    joint1 = armController.targetJoint1 * Mathf.Deg2Rad,
                    joint2 = armController.targetJoint2 * Mathf.Deg2Rad,
                    joint3 = armController.targetJoint3 * Mathf.Deg2Rad,
                    wrist_joint = armController.targetWrist * Mathf.Deg2Rad,
                    gripper_left_joint = Mathf.Lerp(-0.016f, 0.012f, armController.targetGripper),
                    gripper_right_joint = Mathf.Lerp(-0.016f, 0.012f, armController.targetGripper)
                };

                string json = JsonUtility.ToJson(pkt);
                byte[] bytes = Encoding.UTF8.GetBytes(json);
                udpSender.Send(bytes, bytes.Length, rosHost, rosPort);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[RosRoboArmBridge] Failed to send command to ROS: {ex.Message}");
            }
        }

        void OnGUI()
        {
            if (!showHUD) return;

            // Compact ROS Connection Badge in top-right corner
            int width = 310;
            int height = 150;
            int x = Screen.width - width - 15;
            int y = 15;

            GUI.Box(new Rect(x, y, width, height), "ROS 2 Digital Twin Bridge");
            GUILayout.BeginArea(new Rect(x + 10, y + 25, width - 20, height - 30));

            // Status indicator with color
            Color oldColor = GUI.color;
            if (isConnected)
            {
                GUI.color = Color.green;
                GUILayout.Label($" STATUS: CONNECTED ({packetsPerSecond:F0} Hz)");
            }
            else
            {
                GUI.color = new Color(1f, 0.65f, 0.1f);
                GUILayout.Label($" STATUS: LISTENING ON UDP:{listenPort}");
            }
            GUI.color = oldColor;

            GUILayout.Label($"Packets Received: {totalPacketsReceived} | Last: {lastPacketTimestamp}");

            GUILayout.BeginHorizontal();
            enableRosControl = GUILayout.Toggle(enableRosControl, " ROS Control Active");
            smoothMotion = GUILayout.Toggle(smoothMotion, " Smooth");
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(streamCommandsToRos ? "Streaming to ROS (ON)" : "Send Pose to ROS"))
            {
                SendCurrentPoseToRos();
            }
            if (armController != null && GUILayout.Button(armController.autoDemo ? "Stop Demo" : "Start Demo"))
            {
                armController.autoDemo = !armController.autoDemo;
            }
            GUILayout.EndHorizontal();

            GUILayout.EndArea();
        }
    }
}

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
    /// - Telemetry In (Real Arm -> Unity): Receives measured optical encoder values from /eb15/measured_joint_states
    ///   and streams them directly to the Solid Arm Digital Twin.
    /// - Teleoperation Out (Unity Ghost Preview -> Real Arm): Sends commanded target goals from sliders
    ///   to /eb15/joint_commands and /eb15/target_joint_states.
    /// - Supports dynamic runtime IP/port reconfiguration to connect to remote Linux PCs/WSL2.
    /// </summary>
    [RequireComponent(typeof(ArmSimulationController))]
    public class RosRoboArmBridge : MonoBehaviour
    {
        private const string PREF_HOST = "EB15_RosHost";
        private const string PREF_IN_PORT = "EB15_ListenPort";
        private const string PREF_OUT_PORT = "EB15_RosPort";

        [Header("ROS 2 Network Configuration")]
        [Tooltip("Port on which Unity listens for incoming ROS 2 joint states (UDP).")]
        public int listenPort = 5005;

        [Tooltip("ROS 2 machine IP address for sending commands back.")]
        public string rosHost = "127.0.0.1";

        [Tooltip("ROS 2 port for incoming joint commands from Unity (UDP).")]
        public int rosPort = 5006;

        [Header("Teleoperation Streaming")]
        [Tooltip("Stream target slider poses directly to physical robot arm.")]
        public bool streamCommandsToRos = true;

        [Header("Controller Reference")]
        public ArmSimulationController armController;

        [Header("Diagnostics & Telemetry")]
        [SerializeField] private bool isConnected = false;
        [SerializeField] private bool isReceivingRealEncoders = false;
        [SerializeField] private float packetsPerSecond = 0f;
        [SerializeField] private int totalPacketsReceived = 0;
        [SerializeField] private string lastPacketTimestamp = "Never";
        public bool showHUD = false; // Managed by Master Dashboard

        // Public accessors for the Dashboard UI
        public bool IsConnected => isConnected;
        public bool IsReceivingRealEncoders => isReceivingRealEncoders;
        public float PacketsPerSecond => packetsPerSecond;
        public int TotalPacketsReceived => totalPacketsReceived;
        public string LastPacketTimestamp => lastPacketTimestamp;

        // Data packet structure matching unity_bridge_node.py
        [Serializable]
        public class RosJointPacket
        {
            public bool is_encoder;
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

            // Load saved network preferences
            rosHost = PlayerPrefs.GetString(PREF_HOST, rosHost);
            listenPort = PlayerPrefs.GetInt(PREF_IN_PORT, listenPort);
            rosPort = PlayerPrefs.GetInt(PREF_OUT_PORT, rosPort);
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

        /// <summary>
        /// Reconfigures network sockets at runtime when user changes IP or ports in the UI.
        /// </summary>
        public void Reconnect(string newRosHost, int newListenPort, int newRosPort)
        {
            Debug.Log($"[RosRoboArmBridge] Reconnecting to ROS 2: Host={newRosHost}, ListenPort={newListenPort}, SendPort={newRosPort}");

            StopBridge();

            rosHost = newRosHost.Trim();
            listenPort = newListenPort;
            rosPort = newRosPort;

            // Save preferences
            PlayerPrefs.SetString(PREF_HOST, rosHost);
            PlayerPrefs.SetInt(PREF_IN_PORT, listenPort);
            PlayerPrefs.SetInt(PREF_OUT_PORT, rosPort);
            PlayerPrefs.Save();

            StartBridge();
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

                Debug.Log($"[RosRoboArmBridge] Listening on UDP:{listenPort} for physical encoder telemetry. Sending commands to {rosHost}:{rosPort}");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[RosRoboArmBridge] Failed to initialize UDP sockets on port {listenPort}: {ex.Message}");
            }
        }

        private void StopBridge()
        {
            isRunning = false;

            if (receiveThread != null && receiveThread.IsAlive)
            {
                try { receiveThread.Abort(); } catch { }
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
            isReceivingRealEncoders = false;
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
                    Debug.LogWarning($"[RosRoboArmBridge] Receive error: {ex.Message}");
                }
            }
        }

        void Update()
        {
            // FPS & data rate calculation
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

            isConnected = (Time.realtimeSinceStartup - lastReceiveTime) < 1.5f && totalPacketsReceived > 0;

            if (receivedThisFrame && packetToApply != null && armController != null)
            {
                isReceivingRealEncoders = packetToApply.is_encoder;

                // Convert ROS radians to Unity degrees
                float j1Deg = packetToApply.joint1 * Mathf.Rad2Deg;
                float j2Deg = packetToApply.joint2 * Mathf.Rad2Deg;
                float j3Deg = packetToApply.joint3 * Mathf.Rad2Deg;
                float wristDeg = packetToApply.wrist_joint * Mathf.Rad2Deg;

                // Gripper range [-0.016m, 0.012m] -> normalized [0, 1]
                float gripNorm = Mathf.InverseLerp(-0.016f, 0.012f, packetToApply.gripper_left_joint);

                // Pass directly to the Solid Arm Digital Twin!
                armController.OnHardwareEncoderReceived(j1Deg, j2Deg, j3Deg, wristDeg, gripNorm);
            }
        }

        /// <summary>
        /// Sends commanded target angles (from Ghost Preview / Sliders) to the real robot arm via ROS 2.
        /// </summary>
        public void SendTargetPoseToRos(float j1Deg, float j2Deg, float j3Deg, float wristDeg, float gripNorm)
        {
            if (udpSender == null || string.IsNullOrEmpty(rosHost)) return;

            try
            {
                // Convert Unity degrees to ROS radians
                var pkt = new RosJointPacket
                {
                    is_encoder = false,
                    joint1 = j1Deg * Mathf.Deg2Rad,
                    joint2 = j2Deg * Mathf.Deg2Rad,
                    joint3 = j3Deg * Mathf.Deg2Rad,
                    wrist_joint = wristDeg * Mathf.Deg2Rad,
                    gripper_left_joint = Mathf.Lerp(-0.016f, 0.012f, gripNorm),
                    gripper_right_joint = Mathf.Lerp(-0.016f, 0.012f, gripNorm)
                };

                string json = JsonUtility.ToJson(pkt);
                byte[] bytes = Encoding.UTF8.GetBytes(json);
                udpSender.Send(bytes, bytes.Length, rosHost, rosPort);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[RosRoboArmBridge] Failed to send command to {rosHost}:{rosPort}: {ex.Message}");
            }
        }
    }
}

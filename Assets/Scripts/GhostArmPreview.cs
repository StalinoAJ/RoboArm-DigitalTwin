using System;
using UnityEngine;

namespace RoboArm
{
    /// <summary>
    /// Holographic Ghost Preview for the EB15 Robotic Arm.
    /// Provides instantaneous visual feedback of the commanded target pose
    /// while the physical robot arm and solid digital twin move at their real-world velocity.
    /// </summary>
    public class GhostArmPreview : MonoBehaviour
    {
        [Header("Ghost Joint References")]
        public ArticulationBody joint1;
        public ArticulationBody joint2;
        public ArticulationBody joint3;
        public ArticulationBody wristJoint;
        public ArticulationBody gripperLeft;
        public ArticulationBody gripperRight;

        [Header("Appearance & Visibility")]
        [Tooltip("Material used for holographic ghost preview.")]
        public Material ghostMaterial;

        [Tooltip("Toggle visibility of the ghost preview.")]
        public bool isVisible = true;

        private MeshRenderer[] renderers;

        void Awake()
        {
            FindJoints();
            renderers = GetComponentsInChildren<MeshRenderer>();
            ConfigureGhostDrives();
            ApplyGhostMaterial();
            SetVisible(isVisible);
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

            // Remove all colliders from ghost arm to prevent any physics interaction
            var colliders = GetComponentsInChildren<Collider>();
            foreach (var col in colliders)
            {
                Destroy(col);
            }

            // Anchor base link firmly to world
            var baseLink = transform.Find("world/base_link");
            if (baseLink != null)
            {
                var baseAb = baseLink.GetComponent<ArticulationBody>();
                if (baseAb != null) baseAb.immovable = true;
            }
        }

        public void ConfigureGhostDrives()
        {
            // High stiffness for instant preview response
            SetDrive(joint1, 80000f, 400f, 10000f);
            SetDrive(joint2, 80000f, 400f, 10000f);
            SetDrive(joint3, 80000f, 400f, 10000f);
            SetDrive(wristJoint, 80000f, 400f, 10000f);
            SetDrive(gripperLeft, 80000f, 400f, 10000f);
            SetDrive(gripperRight, 80000f, 400f, 10000f);
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

        public void ApplyGhostMaterial()
        {
            if (ghostMaterial == null) return;

            if (renderers == null || renderers.Length == 0)
                renderers = GetComponentsInChildren<MeshRenderer>();

            foreach (var mr in renderers)
            {
                if (mr == null) continue;
                var mats = new Material[mr.sharedMaterials.Length];
                for (int i = 0; i < mats.Length; i++)
                {
                    mats[i] = ghostMaterial;
                }
                mr.sharedMaterials = mats;
            }
        }

        /// <summary>
        /// Updates the ghost arm pose instantly to preview the target setpoints.
        /// </summary>
        public void SetPose(float j1, float j2, float j3, float wrist, float gripNormalized)
        {
            SetTarget(joint1, Mathf.Clamp(j1, -180f, 180f));
            SetTarget(joint2, Mathf.Clamp(j2, -90f, 90f));
            SetTarget(joint3, Mathf.Clamp(j3, -154.7f, 154.7f));
            SetTarget(wristJoint, Mathf.Clamp(wrist, -90f, 90f));

            float gripMeters = Mathf.Lerp(-0.016f, 0.012f, Mathf.Clamp01(gripNormalized));
            SetTarget(gripperLeft, gripMeters);
            SetTarget(gripperRight, gripMeters);
        }

        private void SetTarget(ArticulationBody ab, float target)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.target = target;
            ab.xDrive = drive;
        }

        /// <summary>
        /// Toggles rendering of the ghost arm.
        /// </summary>
        public void SetVisible(bool visible)
        {
            isVisible = visible;
            if (renderers == null || renderers.Length == 0)
                renderers = GetComponentsInChildren<MeshRenderer>();

            foreach (var mr in renderers)
            {
                if (mr != null) mr.enabled = visible;
            }
        }
    }
}

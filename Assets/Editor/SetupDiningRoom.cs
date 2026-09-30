using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

namespace RoboArm.Editor
{
    public static class SetupDiningRoom
    {
        [MenuItem("RoboArm/Setup Modern Dining Room Environment")]
        public static void RunSetup()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
            {
                Debug.LogWarning("[SetupDiningRoom] Cannot run room setup while in play mode.");
                return;
            }

            Debug.Log("[SetupDiningRoom] Starting environment alignment setup...");

            // 1. Ensure Materials directory exists
            string matDir = "Assets/Environment/ModernDiningRoom/Materials";
            if (!AssetDatabase.IsValidFolder(matDir))
            {
                AssetDatabase.CreateFolder("Assets/Environment/ModernDiningRoom", "Materials");
            }

            // 2. Setup URP Lit materials for each texture
            Shader urpLitShader = Shader.Find("Universal Render Pipeline/Lit");
            if (urpLitShader == null)
            {
                urpLitShader = Shader.Find("Standard");
            }

            string fbmDir = "Assets/Environment/ModernDiningRoom/Modern Dining Room.fbm";

            Material CreateOrGetMat(string matName, string texFileName, bool isTransparent = false)
            {
                string path = $"{matDir}/{matName}.mat";
                Material mat = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (mat == null)
                {
                    mat = new Material(urpLitShader);
                    mat.name = matName;
                    AssetDatabase.CreateAsset(mat, path);
                }

                if (!string.IsNullOrEmpty(texFileName))
                {
                    string texPath = $"{fbmDir}/{texFileName}";
                    Texture2D tex = AssetDatabase.LoadAssetAtPath<Texture2D>(texPath);
                    if (tex != null)
                    {
                        mat.SetTexture("_BaseMap", tex);
                        mat.SetTexture("_MainTex", tex);
                    }
                }

                if (isTransparent)
                {
                    mat.SetFloat("_Surface", 1); // Transparent
                    mat.SetFloat("_Blend", 0);
                    mat.renderQueue = 3000;
                    mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                    mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                    mat.SetInt("_ZWrite", 0);
                    mat.DisableKeyword("_ALPHATEST_ON");
                    mat.EnableKeyword("_ALPHABLEND_ON");
                    mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                }
                else
                {
                    mat.SetFloat("_Surface", 0); // Opaque
                }

                EditorUtility.SetDirty(mat);
                return mat;
            }

            Material matWalls = CreateOrGetMat("Mat_Walls", "Walls_c.png");
            Material matFloor = CreateOrGetMat("Mat_Floor", null);
            string laminateTexDir = "Assets/Environment/ModernDiningRoom/LaminateFloor/textures";
            Texture2D floorDiff = AssetDatabase.LoadAssetAtPath<Texture2D>($"{laminateTexDir}/laminate_floor_03_diff_4k.jpg");
            if (floorDiff != null)
            {
                matFloor.SetTexture("_BaseMap", floorDiff);
                matFloor.SetTexture("_MainTex", floorDiff);
            }
            Texture2D floorNorm = AssetDatabase.LoadAssetAtPath<Texture2D>($"{laminateTexDir}/laminate_floor_03_nor_gl_4k.exr");
            if (floorNorm != null)
            {
                matFloor.SetTexture("_BumpMap", floorNorm);
                matFloor.EnableKeyword("_NORMALMAP");
            }
            Texture2D floorDisp = AssetDatabase.LoadAssetAtPath<Texture2D>($"{laminateTexDir}/laminate_floor_03_disp_4k.png");
            if (floorDisp != null)
            {
                matFloor.SetTexture("_ParallaxMap", floorDisp);
            }
            matFloor.SetFloat("_Smoothness", 0.68f);
            matFloor.SetFloat("_Metallic", 0.0f);
            matFloor.SetTextureScale("_BaseMap", new Vector2(4f, 4f));
            matFloor.SetTextureScale("_MainTex", new Vector2(4f, 4f));
            matFloor.SetTextureScale("_BumpMap", new Vector2(4f, 4f));
            matFloor.SetTextureScale("_ParallaxMap", new Vector2(4f, 4f));
            Material matRoof = CreateOrGetMat("Mat_Roof", "Roof_c.png");
            Material matCarpet = CreateOrGetMat("Mat_Carpet", "Carpet_c.png");
            Material matTable = CreateOrGetMat("Mat_DiningTable", "DiningTable_c.png");
            Material matCurtains = CreateOrGetMat("Mat_Curtains", "Curtains_c.png");
            Material matPole = CreateOrGetMat("Mat_CurtainPole", "CurtainPole_c.png");
            Material matPaintings = CreateOrGetMat("Mat_Paintings", "Paintings_c.png");
            Material matPicture = CreateOrGetMat("Mat_Picture", "Picture_c.png");
            Material matFrame = CreateOrGetMat("Mat_PictureFrame", "PictureFrame_c.png");
            Material matDoor = CreateOrGetMat("Mat_DoorFrame", "DoorFrame2_c.png");
            Material matChrome = CreateOrGetMat("Mat_LightChrome", "LightChrome_c.png");
            Material matGlass = CreateOrGetMat("Mat_LightGlass", "LightGlass_c.png", true);
            matGlass.SetColor("_BaseColor", new Color(1.0f, 1.0f, 1.0f, 0.5f));
            matGlass.SetFloat("_Smoothness", 0.9f);

            Material matWindow = CreateOrGetMat("Mat_Window", null, true);
            matWindow.SetColor("_BaseColor", new Color(0.92f, 0.95f, 1.0f, 0.12f));
            matWindow.color = new Color(0.92f, 0.95f, 1.0f, 0.12f);
            matWindow.SetFloat("_Smoothness", 0.95f);
            Texture2D glassNorm = AssetDatabase.LoadAssetAtPath<Texture2D>($"{fbmDir}/GlassNormal.jpg");
            if (glassNorm != null)
            {
                matWindow.SetTexture("_BumpMap", glassNorm);
                matWindow.EnableKeyword("_NORMALMAP");
            }

            AssetDatabase.SaveAssets();

            // 3. Open or get active scene
            var activeScene = EditorSceneManager.GetActiveScene();

            // 4. Hide/Disable existing Industrial_Environment
            GameObject oldEnv = GameObject.Find("Industrial_Environment");
            if (oldEnv != null)
            {
                oldEnv.SetActive(false);
                Debug.Log("[SetupDiningRoom] Disabled old Industrial_Environment.");
            }

            // 5. Re-instantiate Modern_Dining_Room from fresh FBX
            GameObject oldRoom = GameObject.Find("Modern_Dining_Room");
            if (oldRoom != null)
            {
                Undo.DestroyObjectImmediate(oldRoom);
            }
            GameObject oldRoomHyphen = GameObject.Find("modern-dining-room");
            if (oldRoomHyphen != null)
            {
                Undo.DestroyObjectImmediate(oldRoomHyphen);
            }

            string fbxPath = "Assets/Environment/ModernDiningRoom/Modern Dining Room.fbx";
            AssetDatabase.ImportAsset(fbxPath, ImportAssetOptions.ForceUpdate);
            GameObject fbxPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(fbxPath);
            if (fbxPrefab == null)
            {
                Debug.LogError("[SetupDiningRoom] Could not load FBX at " + fbxPath);
                return;
            }

            GameObject roomGo = (GameObject)PrefabUtility.InstantiatePrefab(fbxPrefab);
            roomGo.name = "Modern_Dining_Room";
            Undo.RegisterCreatedObjectUndo(roomGo, "Instantiate Modern Dining Room");

            // Apply exact coordinate transformation:
            // Blender (Right-Handed, Z-Up) -> Unity (Left-Handed, Y-Up):
            //   x_Unity = -y_Blender  (places Window on LEFT at -X)
            //   y_Unity =  z_Blender  (places Floor at bottom Y=0, Roof at top Y=2.5)
            //   z_Unity =  x_Blender  (places 3 Paintings on BACK wall at +Z)
            // Scale multiplier 1.15 gives standard 2.5m ceiling and 0.75m dining table height.
            float s = 1.15f;
            roomGo.transform.position = new Vector3(0f, 1.09f * s, 0f);
            roomGo.transform.rotation = Quaternion.Euler(-90f, 90f, 0f);
            roomGo.transform.localScale = new Vector3(-s, s, s);

            // 6. Assign Materials to room parts and locate Plant & Pot
            Transform plantT = null;
            Transform potT = null;
            Transform tableT = null;

            // Deactivate any rogue cameras or lights imported with the FBX
            foreach (var cam in roomGo.GetComponentsInChildren<Camera>(true))
            {
                cam.gameObject.SetActive(false);
                cam.enabled = false;
                Debug.Log("[SetupDiningRoom] Deactivated imported Camera: " + cam.gameObject.name);
            }
            foreach (var light in roomGo.GetComponentsInChildren<Light>(true))
            {
                light.gameObject.SetActive(false);
                light.enabled = false;
                Debug.Log("[SetupDiningRoom] Deactivated imported Light: " + light.gameObject.name);
            }

            foreach (var renderer in roomGo.GetComponentsInChildren<Renderer>(true))
            {
                string n = renderer.gameObject.name.ToLower();
                if (n.Contains("plant") || n.Contains("leaf") || n.Contains("leaves"))
                {
                    plantT = renderer.transform;
                    renderer.gameObject.SetActive(false);
                    Debug.Log("[SetupDiningRoom] Deactivated Plant: " + renderer.gameObject.name);
                }
                else if (n.Contains("pot"))
                {
                    potT = renderer.transform;
                    renderer.gameObject.SetActive(false);
                    Debug.Log("[SetupDiningRoom] Deactivated Pot: " + renderer.gameObject.name);
                }
                else if (n.Contains("table"))
                {
                    tableT = renderer.transform;
                    renderer.sharedMaterial = matTable;
                }
                else if (n.Contains("wall"))
                {
                    renderer.sharedMaterial = matWalls;
                }
                else if (n.Contains("roof"))
                {
                    renderer.sharedMaterial = matRoof;
                }
                else if (n.Contains("floor") || n.Contains("modern dining room"))
                {
                    renderer.sharedMaterial = matFloor;
                }
                else if (n.Contains("carpet"))
                {
                    renderer.sharedMaterial = matCarpet;
                }
                else if (n.Contains("curtainpole"))
                {
                    renderer.sharedMaterial = matPole;
                }
                else if (n.Contains("curtain"))
                {
                    renderer.sharedMaterial = matCurtains;
                }
                else if (n.Contains("paintings"))
                {
                    renderer.sharedMaterial = matPaintings;
                }
                else if (n.Contains("pictureframe"))
                {
                    renderer.sharedMaterial = matFrame;
                }
                else if (n.Contains("picture"))
                {
                    renderer.sharedMaterial = matPicture;
                }
                else if (n.Contains("door"))
                {
                    renderer.sharedMaterial = matDoor;
                }
                else if (n.Contains("chrome"))
                {
                    renderer.sharedMaterial = matChrome;
                }
                else if (n.Contains("lightglass") || n.Contains("glass"))
                {
                    renderer.sharedMaterial = matGlass;
                }
                else if (n.Contains("window"))
                {
                    renderer.sharedMaterial = matWindow;
                }
            }

            // Assign Mat_Floor to scene Floor GameObject
            GameObject sceneFloor = GameObject.Find("Floor");
            if (sceneFloor != null)
            {
                var mr = sceneFloor.GetComponent<MeshRenderer>();
                if (mr != null)
                {
                    Undo.RecordObject(mr, "Assign Mat_Floor to scene Floor");
                    mr.sharedMaterial = matFloor;
                    Debug.Log("[SetupDiningRoom] Assigned Mat_Floor to scene Floor GameObject.");
                }
            }

            // 7. Place robot arm flush on tabletop surface
            // The marble dining table surface is at Y = 0.62825m, centered at X = -0.164m, Z = 0.000m
            Vector3 robotSpawnPos = new Vector3(-0.164f, 0.62825f, 0.000f);
            Debug.Log($"[SetupDiningRoom] Target Robot position on table (pot replacement): {robotSpawnPos}");

            // 8. Position eb15 and eb15_Ghost
            GameObject eb15 = GameObject.Find("eb15");
            if (eb15 != null)
            {
                Undo.RecordObject(eb15.transform, "Move eb15 to Dining Table");
                eb15.transform.position = robotSpawnPos;
                eb15.transform.rotation = Quaternion.Euler(0f, 0f, 0f);

                var bodies = eb15.GetComponentsInChildren<ArticulationBody>(true);
                foreach (var ab in bodies)
                {
                    if (ab.isRoot)
                    {
                        ab.TeleportRoot(robotSpawnPos, Quaternion.Euler(0f, 0f, 0f));
                        break;
                    }
                }
                Debug.Log($"[SetupDiningRoom] eb15 positioned at {robotSpawnPos}");
            }

            GameObject ghost = GameObject.Find("eb15_Ghost");
            if (ghost != null)
            {
                Undo.RecordObject(ghost.transform, "Move eb15_Ghost to Dining Table");
                ghost.transform.position = robotSpawnPos;
                ghost.transform.rotation = Quaternion.Euler(0f, 0f, 0f);

                var bodiesG = ghost.GetComponentsInChildren<ArticulationBody>(true);
                foreach (var abG in bodiesG)
                {
                    if (abG.isRoot)
                    {
                        abG.TeleportRoot(robotSpawnPos, Quaternion.Euler(0f, 0f, 0f));
                        break;
                    }
                }
                Debug.Log($"[SetupDiningRoom] eb15_Ghost positioned at {robotSpawnPos}");
            }

            // 9. Position Camera inside room viewing robot on table
            Camera mainCam = Camera.main;
            if (mainCam != null)
            {
                mainCam.depth = 100f;
                mainCam.enabled = true;
                mainCam.nearClipPlane = 0.02f;
                mainCam.fieldOfView = 48f;
                mainCam.transform.position = new Vector3(0.591f, 1.318f, -0.838f);
                mainCam.transform.LookAt(robotSpawnPos + new Vector3(0f, 0.28f, 0f));

                var camCtrl = mainCam.GetComponent<MouseCameraController>();
                if (camCtrl == null)
                {
                    camCtrl = mainCam.gameObject.AddComponent<MouseCameraController>();
                }
                if (eb15 != null) camCtrl.targetTransform = eb15.transform;
                camCtrl.focusOffset = new Vector3(0f, 0.28f, 0f);
                camCtrl.clampInsideRoom = true;
                camCtrl.roomMin = new Vector3(-1.15f, 0.35f, -1.60f);
                camCtrl.roomMax = new Vector3(1.15f, 2.30f, 1.15f);
                camCtrl.lookAtMin = new Vector3(-1.00f, 0.35f, -1.45f);
                camCtrl.lookAtMax = new Vector3(1.00f, 2.20f, 1.00f);
                camCtrl.minDistance = 0.20f;
                camCtrl.maxDistance = 2.20f;
                camCtrl.keyboardMoveSpeed = 1.35f;
            }

            // 10. Adjust Directional Light shining from window on the LEFT (-X)
            GameObject dirLightGo = GameObject.Find("Directional Light");
            if (dirLightGo != null)
            {
                dirLightGo.transform.rotation = Quaternion.Euler(35f, 65f, 0f);
                Light dirLight = dirLightGo.GetComponent<Light>();
                if (dirLight != null)
                {
                    dirLight.color = new Color(1.0f, 0.98f, 0.94f);
                    dirLight.intensity = 0.85f;
                    dirLight.shadows = LightShadows.Soft;
                }
            }

            // 11. Add Floor and Window Backdrops
            BuildBackdrops();

            // 12. Mark scene dirty and save
            EditorSceneManager.MarkSceneDirty(activeScene);
            EditorSceneManager.SaveScene(activeScene);

            Debug.Log("[SetupDiningRoom] Corrected Modern Dining Room alignment complete and scene saved successfully!");
        }

        [MenuItem("RoboArm/Add Floor Backdrop")]
        public static void BuildBackdrops()
        {
            var activeScene = EditorSceneManager.GetActiveScene();

            // 1. Load or verify materials
            string matDir = "Assets/Environment/ModernDiningRoom/Materials";
            Material studioMat = AssetDatabase.LoadAssetAtPath<Material>($"{matDir}/Mat_StudioBackdrop.mat");
            Material windowMat = AssetDatabase.LoadAssetAtPath<Material>($"{matDir}/Mat_WindowCityBackdrop.mat");

            if (studioMat == null)
            {
                Shader urpLit = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                studioMat = new Material(urpLit) { name = "Mat_StudioBackdrop" };
                studioMat.SetColor("_BaseColor", new Color(0.82f, 0.80f, 0.77f, 1f));
                studioMat.SetFloat("_Smoothness", 0.15f);
                AssetDatabase.CreateAsset(studioMat, $"{matDir}/Mat_StudioBackdrop.mat");
            }

            if (windowMat == null)
            {
                Shader urpLit = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                windowMat = new Material(urpLit) { name = "Mat_WindowCityBackdrop" };
                Texture2D skylineTex = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Environment/Textures/outdoor_city_skyline.jpg");
                if (skylineTex != null)
                {
                    windowMat.SetTexture("_BaseMap", skylineTex);
                    windowMat.SetTexture("_EmissionMap", skylineTex);
                    windowMat.EnableKeyword("_EMISSION");
                    windowMat.SetColor("_EmissionColor", new Color(0.22f, 0.25f, 0.3f, 1f));
                }
                AssetDatabase.CreateAsset(windowMat, $"{matDir}/Mat_WindowCityBackdrop.mat");
            }

            // 2. Remove existing backdrop root if present to avoid duplication
            GameObject existingRoot = GameObject.Find("Environment_Backdrop");
            if (existingRoot != null)
            {
                Undo.DestroyObjectImmediate(existingRoot);
            }

            // 3. Create Root GameObject
            GameObject root = new GameObject("Environment_Backdrop");
            Undo.RegisterCreatedObjectUndo(root, "Create Environment Backdrop");
            root.transform.position = Vector3.zero;
            root.transform.rotation = Quaternion.identity;

            // 4. Create Studio Perimeter Backdrop Walls (End of Floor)
            float wallH = 4.2f;
            float wallThickness = 0.25f;

            void CreateWall(string name, Vector3 pos, Vector3 scale)
            {
                GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                wall.name = name;
                wall.transform.SetParent(root.transform);
                wall.transform.position = pos;
                wall.transform.localScale = scale;
                var mr = wall.GetComponent<MeshRenderer>();
                if (mr != null) mr.sharedMaterial = studioMat;
                var col = wall.GetComponent<Collider>();
                if (col != null) UnityEngine.Object.DestroyImmediate(col);
            }

            // A. Back Wall (+X edge of floor)
            CreateWall("Backdrop_Wall_PositiveX", new Vector3(5.0f, wallH * 0.5f, 0f), new Vector3(wallThickness, wallH, 10.5f));

            // B. Right Wall (-Z edge of floor)
            CreateWall("Backdrop_Wall_NegativeZ", new Vector3(0f, wallH * 0.5f, -5.0f), new Vector3(10.5f, wallH, wallThickness));

            // C. Left Wall (+Z edge of floor)
            CreateWall("Backdrop_Wall_PositiveZ", new Vector3(0f, wallH * 0.5f, 5.0f), new Vector3(10.5f, wallH, wallThickness));

            // D. Far Wall (-X edge behind dining room)
            CreateWall("Backdrop_Wall_NegativeX", new Vector3(-5.0f, wallH * 0.5f, 0f), new Vector3(wallThickness, wallH, 10.5f));

            // 5. Create Outside Window Panorama Backdrop
            GameObject windowBackdrop = GameObject.CreatePrimitive(PrimitiveType.Quad);
            windowBackdrop.name = "Window_City_Skyline_Backdrop";
            windowBackdrop.transform.SetParent(root.transform);
            windowBackdrop.transform.position = new Vector3(-2.85f, 1.85f, -0.15f);
            windowBackdrop.transform.rotation = Quaternion.Euler(0f, 90f, 0f);
            windowBackdrop.transform.localScale = new Vector3(8.5f, 4.2f, 1f);
            var winMr = windowBackdrop.GetComponent<MeshRenderer>();
            if (winMr != null) winMr.sharedMaterial = windowMat;
            var winCol = windowBackdrop.GetComponent<Collider>();
            if (winCol != null) UnityEngine.Object.DestroyImmediate(winCol);

            EditorSceneManager.MarkSceneDirty(activeScene);
            EditorSceneManager.SaveScene(activeScene);
            Debug.Log("[SetupDiningRoom] Successfully created floor perimeter backdrop and window skyline vista!");
        }
    }
}

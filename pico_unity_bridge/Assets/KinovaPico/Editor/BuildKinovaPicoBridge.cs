using System;
using System.IO;
using System.Reflection;
using System.Xml;
using Unity.XR.PXR;
using Unity.XR.OpenXR.Features.PICOSupport;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEditor.XR.OpenXR.Features;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Hands.OpenXR;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features;
using UnityEngine.XR.OpenXR.Features.Interactions;

namespace Yezqin.KinovaPico.Editor
{
    public sealed class KinovaPicoBuildSpec
    {
        static readonly KinovaPicoBuildSpec ApprovedSpec = new KinovaPicoBuildSpec(
            "com.yezqin.kinovapicobridge",
            "Kinova PICO Bridge",
            AndroidArchitecture.ARM64,
            ScriptingImplementation.IL2CPP,
            AndroidSdkVersions.AndroidApiLevel29,
            "Assets/KinovaPico/KinovaPicoBridge.unity",
            "Builds/KinovaPicoBridge-development.apk",
            BuildOptions.NoUniqueIdentifier,
            new[] { typeof(PICO4UltraControllerProfile) });

        readonly Type[] _controllerProfileTypes;

        KinovaPicoBuildSpec(
            string packageId,
            string applicationName,
            AndroidArchitecture architectures,
            ScriptingImplementation scriptingBackend,
            AndroidSdkVersions minimumSdkVersion,
            string scenePath,
            string buildPath,
            BuildOptions buildOptions,
            Type[] controllerProfileTypes)
        {
            PackageId = packageId;
            ApplicationName = applicationName;
            Architectures = architectures;
            ScriptingBackend = scriptingBackend;
            MinimumSdkVersion = minimumSdkVersion;
            ScenePath = scenePath;
            BuildPath = buildPath;
            BuildOptions = buildOptions;
            _controllerProfileTypes = controllerProfileTypes;
        }

        public static KinovaPicoBuildSpec Approved => ApprovedSpec;
        public string PackageId { get; }
        public string ApplicationName { get; }
        public AndroidArchitecture Architectures { get; }
        public ScriptingImplementation ScriptingBackend { get; }
        public AndroidSdkVersions MinimumSdkVersion { get; }
        public string ScenePath { get; }
        public string BuildPath { get; }
        public BuildOptions BuildOptions { get; }
        public Type[] ControllerProfileTypes => (Type[])_controllerProfileTypes.Clone();
    }

    public static class BuildKinovaPicoBridge
    {
        const string XrSettingsAssetPath =
            "Assets/XR/Settings/XRGeneralSettingsPerBuildTarget.asset";

        [MenuItem("Kinova PICO/Configure Android Project")]
        public static void Configure()
        {
            var spec = KinovaPicoBuildSpec.Approved;
            if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android
                && !EditorUserBuildSettings.SwitchActiveBuildTarget(
                    BuildTargetGroup.Android,
                    BuildTarget.Android))
            {
                throw new BuildFailedException("Unable to switch the Unity project to Android.");
            }

            PlayerSettings.productName = spec.ApplicationName;
            PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, spec.PackageId);
            PlayerSettings.Android.forceInternetPermission = true;
            PlayerSettings.Android.targetArchitectures = spec.Architectures;
            PlayerSettings.SetScriptingBackend(
                BuildTargetGroup.Android,
                spec.ScriptingBackend);
            PlayerSettings.Android.minSdkVersion = spec.MinimumSdkVersion;
            PlayerSettings.colorSpace = ColorSpace.Linear;
            ConfigureSerializedPlayerSettings();

            var manager = GetOrCreateAndroidManager();
            if (!XRPackageMetadataStore.AssignLoader(
                    manager,
                    typeof(OpenXRLoader).FullName,
                    BuildTargetGroup.Android)
                && !XRPackageMetadataStore.IsLoaderAssigned(
                    typeof(OpenXRLoader).FullName,
                    BuildTargetGroup.Android))
            {
                throw new BuildFailedException("Unable to assign the OpenXR loader for Android.");
            }

            FeatureHelpers.RefreshFeatures(BuildTargetGroup.Android);
            var openXrSettings =
                OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Android);
            if (openXrSettings == null)
                throw new BuildFailedException("OpenXR Android settings were not created.");

            SetFeatureEnabled<PICOFeature>(openXrSettings, true, "PICO Support");
            SetFeatureEnabled<OpenXRExtensions>(
                openXrSettings,
                true,
                "PICO OpenXR extensions");
            SetFeatureEnabled<HandTracking>(
                openXrSettings,
                false,
                "XR Hands tracking");
            SetFeatureEnabled<PICO4ControllerProfile>(
                openXrSettings,
                false,
                "PICO4 controller profile");
            SetFeatureEnabled<PICONeo3ControllerProfile>(
                openXrSettings,
                false,
                "PICO Neo3 controller profile");
            SetFeatureEnabled<PICOG3ControllerProfile>(
                openXrSettings,
                false,
                "PICO G3 controller profile");
            foreach (var profileType in spec.ControllerProfileTypes)
                SetFeatureEnabled(openXrSettings, profileType, true);

            var picoSettings = PICOProjectSetting.GetProjectConfig();
            if (picoSettings == null)
                throw new BuildFailedException("PICO project settings were not created.");
            picoSettings.isHandTracking = false;
            picoSettings.handTrackingSupportType = HandTrackingSupport.ControllersAndHands;
            picoSettings.highFrequencyHand = false;
            EditorUtility.SetDirty(picoSettings);

            // The pinned platform preprocessor calls Trim() on this optional value.
            // Whitespace keeps it logically unconfigured while avoiding a null dereference.
            var platformSettings = PXR_PlatformSetting.Instance;
            if (platformSettings != null && string.IsNullOrEmpty(platformSettings.appID))
            {
                platformSettings.appID = " ";
                EditorUtility.SetDirty(platformSettings);
            }

            EnsureBuildScene(spec.ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log(
                "Kinova PICO Android project configured: OpenXR/PICO4 Ultra controller, "
                + "ARM64, IL2CPP, INTERNET.");
        }

        [MenuItem("Kinova PICO/Validate Android Project")]
        public static void Validate()
        {
            Configure();
            var spec = KinovaPicoBuildSpec.Approved;
            var manager = XRGeneralSettingsPerBuildTarget
                .XRGeneralSettingsForBuildTarget(BuildTargetGroup.Android)?.Manager;
            var openXrSettings =
                OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Android);
            var picoSettings = PICOProjectSetting.GetProjectConfig();

            Require(
                EditorUserBuildSettings.activeBuildTarget == BuildTarget.Android,
                "Android target is not active.");
            Require(
                PlayerSettings.productName == spec.ApplicationName,
                "Application name is wrong.");
            Require(
                PlayerSettings.GetApplicationIdentifier(BuildTargetGroup.Android)
                    == spec.PackageId,
                "Android package id is wrong.");
            Require(
                PlayerSettings.Android.forceInternetPermission,
                "Android INTERNET permission is not forced.");
            Require(
                PlayerSettings.Android.targetArchitectures == spec.Architectures,
                "Android architecture must be ARM64 only.");
            Require(
                PlayerSettings.GetScriptingBackend(BuildTargetGroup.Android)
                    == spec.ScriptingBackend,
                "Android scripting backend must be IL2CPP.");
            Require(
                PlayerSettings.Android.minSdkVersion == spec.MinimumSdkVersion,
                "Android minimum SDK is wrong.");
            ValidateSerializedPlayerSettings();
            Require(manager != null, "Android XR manager is missing.");
            Require(
                XRPackageMetadataStore.IsLoaderAssigned(
                    typeof(OpenXRLoader).FullName,
                    BuildTargetGroup.Android),
                "OpenXR loader is missing.");
            Require(
                GetFeature<PICOFeature>(openXrSettings)?.enabled == true,
                "PICO OpenXR feature is disabled.");
            Require(
                GetFeature<OpenXRExtensions>(openXrSettings)?.enabled == true,
                "PICO OpenXR extensions are disabled.");
            Require(
                GetFeature<HandTracking>(openXrSettings)?.enabled == false,
                "XR Hands must be disabled.");
            foreach (var profileType in spec.ControllerProfileTypes)
            {
                Require(
                    GetFeature(openXrSettings, profileType)?.enabled == true,
                    $"{profileType.Name} is disabled.");
            }
            Require(
                GetFeature<PICO4ControllerProfile>(openXrSettings)?.enabled == false,
                "Non-Ultra PICO4 controller profile must be disabled.");
            Require(
                picoSettings != null && !picoSettings.isHandTracking,
                "PICO hand tracking must be disabled.");
            Require(
                EditorBuildSettings.scenes.Length == 1
                    && EditorBuildSettings.scenes[0].path == spec.ScenePath
                    && EditorBuildSettings.scenes[0].enabled,
                "Generated build scene is missing.");
            Require(
                File.Exists(Path.Combine(
                    Directory.GetCurrentDirectory(),
                    "Assets/Plugins/Android/AndroidManifest.xml")),
                "Custom Android manifest is missing.");

            Debug.Log("KINOVA_PICO_VALIDATION_OK");
        }

        public static void ValidateBatch()
        {
            Validate();
            EditorApplication.Exit(0);
        }

        [MenuItem("Kinova PICO/Build Android Development")]
        public static void Build()
        {
            Validate();
            var spec = KinovaPicoBuildSpec.Approved;
            Directory.CreateDirectory(Path.GetDirectoryName(spec.BuildPath) ?? "Builds");
            var options = new BuildPlayerOptions
            {
                scenes = new[] { spec.ScenePath },
                locationPathName = spec.BuildPath,
                target = BuildTarget.Android,
                targetGroup = BuildTargetGroup.Android,
                options = spec.BuildOptions,
            };

            var report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
            {
                throw new BuildFailedException(
                    $"Android development build failed: {report.summary.result}");
            }
            Debug.Log(
                $"KINOVA_PICO_ANDROID_BUILD_OK path={spec.BuildPath} "
                + $"bytes={report.summary.totalSize}");
            if (Application.isBatchMode)
                EditorApplication.Exit(0);
        }

        static XRManagerSettings GetOrCreateAndroidManager()
        {
            if (!EditorBuildSettings.TryGetConfigObject<XRGeneralSettingsPerBuildTarget>(
                    XRGeneralSettings.k_SettingsKey,
                    out var perBuildTarget))
            {
                EnsureAssetFolder("Assets/XR/Settings");
                perBuildTarget =
                    ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
                AssetDatabase.CreateAsset(perBuildTarget, XrSettingsAssetPath);
                EditorBuildSettings.AddConfigObject(
                    XRGeneralSettings.k_SettingsKey,
                    perBuildTarget,
                    true);
            }

            if (!perBuildTarget.HasManagerSettingsForBuildTarget(BuildTargetGroup.Android))
            {
                perBuildTarget.CreateDefaultManagerSettingsForBuildTarget(
                    BuildTargetGroup.Android);
            }
            var generalSettings =
                perBuildTarget.SettingsForBuildTarget(BuildTargetGroup.Android);
            generalSettings.InitManagerOnStart = true;
            EditorUtility.SetDirty(generalSettings);
            return generalSettings.Manager;
        }

        static void ConfigureSerializedPlayerSettings()
        {
            var serialized = GetSerializedPlayerSettings();
            serialized.Update();
            RequireSerializedProperty(serialized, "activeInputHandler").intValue = 1;
            RequireSerializedProperty(serialized, "useCustomMainManifest").boolValue = true;
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }

        static void ValidateSerializedPlayerSettings()
        {
            var serialized = GetSerializedPlayerSettings();
            serialized.Update();
            Require(
                RequireSerializedProperty(serialized, "activeInputHandler").intValue == 1,
                "Active Input Handling must use the Input System package.");
            Require(
                RequireSerializedProperty(
                    serialized,
                    "useCustomMainManifest").boolValue,
                "Custom Android main manifest must be enabled.");
        }

        static SerializedObject GetSerializedPlayerSettings()
        {
            var method = typeof(PlayerSettings).GetMethod(
                "GetSerializedObject",
                BindingFlags.NonPublic | BindingFlags.Static);
            var serialized = method?.Invoke(null, null) as SerializedObject;
            if (serialized == null)
            {
                throw new BuildFailedException(
                    "Unable to access serialized PlayerSettings.");
            }
            return serialized;
        }

        static SerializedProperty RequireSerializedProperty(
            SerializedObject serialized,
            string propertyName)
        {
            var property = serialized.FindProperty(propertyName);
            if (property == null)
            {
                throw new BuildFailedException(
                    $"PlayerSettings property is unavailable: {propertyName}");
            }
            return property;
        }

        static void SetFeatureEnabled<TFeature>(
            OpenXRSettings settings,
            bool enabled,
            string description)
            where TFeature : OpenXRFeature
        {
            var feature = GetFeature<TFeature>(settings);
            if (feature == null)
                throw new BuildFailedException($"{description} feature was not discovered.");
            feature.enabled = enabled;
            EditorUtility.SetDirty(feature);
        }

        static void SetFeatureEnabled(
            OpenXRSettings settings,
            Type featureType,
            bool enabled)
        {
            var feature = GetFeature(settings, featureType);
            if (feature == null)
            {
                throw new BuildFailedException(
                    $"{featureType.Name} feature was not discovered.");
            }
            feature.enabled = enabled;
            EditorUtility.SetDirty(feature);
        }

        static TFeature GetFeature<TFeature>(OpenXRSettings settings)
            where TFeature : OpenXRFeature
        {
            return settings == null ? null : settings.GetFeature<TFeature>();
        }

        static OpenXRFeature GetFeature(OpenXRSettings settings, Type featureType)
        {
            return settings == null || featureType == null
                ? null
                : settings.GetFeature(featureType);
        }

        static void EnsureBuildScene(string scenePath)
        {
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(scenePath) == null)
            {
                var activeScene = SceneManager.GetActiveScene();
                Scene sceneToSave;
                var closeAfterSave = false;
                if (activeScene.IsValid()
                    && string.IsNullOrEmpty(activeScene.path)
                    && activeScene.rootCount == 0)
                {
                    sceneToSave = activeScene;
                }
                else
                {
                    if (activeScene.IsValid() && string.IsNullOrEmpty(activeScene.path))
                    {
                        throw new BuildFailedException(
                            "Save the current nonempty untitled scene before configuring "
                            + "the Kinova PICO project.");
                    }
                    sceneToSave = EditorSceneManager.NewScene(
                        NewSceneSetup.EmptyScene,
                        NewSceneMode.Additive);
                    closeAfterSave = true;
                }

                sceneToSave.name = "KinovaPicoBridge";
                if (!EditorSceneManager.SaveScene(sceneToSave, scenePath))
                {
                    throw new BuildFailedException(
                        $"Unable to save build scene at {scenePath}.");
                }
                if (closeAfterSave)
                    EditorSceneManager.CloseScene(sceneToSave, true);
            }

            EditorBuildSettings.scenes =
                new[] { new EditorBuildSettingsScene(scenePath, true) };
        }

        static void EnsureAssetFolder(string assetPath)
        {
            var parts = assetPath.Split('/');
            var current = parts[0];
            for (var index = 1; index < parts.Length; index++)
            {
                var next = $"{current}/{parts[index]}";
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[index]);
                current = next;
            }
        }

        static void Require(bool condition, string message)
        {
            if (!condition)
                throw new BuildFailedException(message);
        }
    }

    internal sealed class KinovaInternetPermissionBuildHook : OpenXRFeatureBuildHooks
    {
        const string AndroidXmlNamespace =
            "http://schemas.android.com/apk/res/android";
        const string InternetPermission = "android.permission.INTERNET";

        public override int callbackOrder => 1000;
        public override Type featureType => typeof(PICOFeature);

        protected override void OnPreprocessBuildExt(BuildReport report) { }

        protected override void OnPostGenerateGradleAndroidProjectExt(string path)
        {
            var manifestPath = Path.Combine(path, "src", "main", "AndroidManifest.xml");
            var document = new XmlDocument();
            document.Load(manifestPath);
            var manifest = document.DocumentElement;
            if (manifest == null)
            {
                throw new BuildFailedException(
                    $"Android manifest is invalid: {manifestPath}");
            }

            foreach (XmlNode permission in manifest.SelectNodes("uses-permission"))
            {
                if (permission is XmlElement element
                    && element.GetAttribute("name", AndroidXmlNamespace)
                        == InternetPermission)
                {
                    return;
                }
            }

            var internet = document.CreateElement("uses-permission");
            internet.SetAttribute("name", AndroidXmlNamespace, InternetPermission);
            manifest.AppendChild(internet);
            document.Save(manifestPath);
        }

        protected override void OnPostprocessBuildExt(BuildReport report) { }
    }
}

using UnityEngine;
using UnityEngine.SpatialTracking;

namespace Yezqin.KinovaPico
{
    public static class KinovaBridgeBootstrap
    {
        const string BridgeObjectName = "Kinova PICO UDP Bridge";

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void CreateBridge()
        {
            if (Object.FindObjectOfType<KinovaControllerUdpBridge>() != null)
                return;

            var bridgeObject = new GameObject(BridgeObjectName);
            bridgeObject.AddComponent<KinovaControllerUdpBridge>();
            Object.DontDestroyOnLoad(bridgeObject);
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void CreatePresentationIfAbsent()
        {
            var camera = Camera.main;
            if (camera == null)
            {
                var cameraObject = new GameObject("Kinova PICO XR Camera");
                cameraObject.tag = "MainCamera";
                camera = cameraObject.AddComponent<Camera>();
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = Color.black;
                camera.nearClipPlane = 0.05f;
                camera.stereoTargetEye = StereoTargetEyeMask.Both;

                var poseDriver = cameraObject.AddComponent<TrackedPoseDriver>();
                poseDriver.SetPoseSource(
                    TrackedPoseDriver.DeviceType.GenericXRDevice,
                    TrackedPoseDriver.TrackedPose.Center);
                poseDriver.trackingType = TrackedPoseDriver.TrackingType.RotationAndPosition;
                poseDriver.updateType = TrackedPoseDriver.UpdateType.UpdateAndBeforeRender;
                if (Object.FindObjectOfType<AudioListener>() == null)
                    cameraObject.AddComponent<AudioListener>();
                Object.DontDestroyOnLoad(cameraObject);
            }

            if (Object.FindObjectOfType<KinovaBridgeStatusText>() != null)
                return;

            var statusObject = new GameObject("Kinova PICO Bridge Status");
            statusObject.transform.SetParent(camera.transform, false);
            statusObject.transform.localPosition = new Vector3(0.0f, -0.12f, 1.5f);
            var text = statusObject.AddComponent<TextMesh>();
            text.anchor = TextAnchor.MiddleCenter;
            text.alignment = TextAlignment.Center;
            text.characterSize = 0.05f;
            text.fontSize = 48;
            text.color = Color.white;
            text.text = "Kinova PICO UDP bridge\nhost: searching";
            statusObject.AddComponent<KinovaBridgeStatusText>();
        }
    }

    sealed class KinovaBridgeStatusText : MonoBehaviour
    {
        TextMesh _text;
        KinovaControllerUdpBridge _bridge;
        float _nextRefreshAt;

        void Awake()
        {
            _text = GetComponent<TextMesh>();
        }

        void Update()
        {
            if (Time.unscaledTime < _nextRefreshAt)
                return;
            _nextRefreshAt = Time.unscaledTime + 0.25f;

            if (_bridge == null)
                _bridge = Object.FindObjectOfType<KinovaControllerUdpBridge>();
            _text.text = _bridge == null
                ? "Kinova PICO UDP bridge\nhost: unavailable"
                : _bridge.StatusText;
        }
    }
}

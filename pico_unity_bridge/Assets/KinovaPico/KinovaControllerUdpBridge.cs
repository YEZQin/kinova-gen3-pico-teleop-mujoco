using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

namespace Yezqin.KinovaPico
{
    public sealed class KinovaControllerUdpBridge : MonoBehaviour
    {
        public const int ProtocolPort = 15031;
        public const string DiscoverMessage = "KINOVA_DISCOVER_V1";
        public const string ReadyMessage = "KINOVA_READY_V1";

        const double DiscoveryIntervalSeconds = 1.0;
        const int MaxReadyDatagramsPerFrame = 16;
        const double SocketRetryIntervalSeconds = 1.0;

        static readonly byte[] DiscoverBytes = Encoding.ASCII.GetBytes(DiscoverMessage);
        static readonly byte[] ReadyBytes = Encoding.ASCII.GetBytes(ReadyMessage);

        readonly KinovaDiscoveryState _discovery =
            new KinovaDiscoveryState(DiscoveryIntervalSeconds);

        UdpClient _udp;
        bool _running;
        bool _paused;
        double _nextSocketRetryAt;
        uint _sequence;
        bool _tracked;
        float _grip;
        double _rateWindowStartedAt;
        int _rateWindowPackets;
        int _packetsPerSecond;

        public IPEndPoint HostEndpoint => _discovery.HostEndpoint;
        public bool IsReady => HostEndpoint != null;
        public uint Sequence => _sequence;
        public bool Tracked => _tracked;
        public float Grip => _grip;
        public int PacketsPerSecond => _packetsPerSecond;
        public string Status { get; private set; } = "Stopped";

        public string StatusText =>
            KinovaBridgeStatus.Format(
                HostEndpoint,
                _tracked,
                _grip,
                _sequence,
                _packetsPerSecond);

        public static IPEndPoint DiscoveryEndpoint =>
            new IPEndPoint(IPAddress.Broadcast, ProtocolPort);

        public static byte[] CreateDiscoveryDatagram()
        {
            return (byte[])DiscoverBytes.Clone();
        }

        public static bool TryAcceptReadyDatagram(
            byte[] payload,
            IPEndPoint source,
            KinovaDiscoveryState state)
        {
            if (payload == null)
                throw new ArgumentNullException(nameof(payload));
            if (source == null)
                throw new ArgumentNullException(nameof(source));
            if (state == null)
                throw new ArgumentNullException(nameof(state));
            if (!ByteArraysEqual(payload, ReadyBytes))
                return false;

            state.AcceptReady(source);
            return true;
        }

        public static ulong ToSourceTimeUs(double realtimeSeconds)
        {
            if (double.IsNaN(realtimeSeconds)
                || double.IsInfinity(realtimeSeconds)
                || realtimeSeconds < 0.0
                || realtimeSeconds > ulong.MaxValue / 1_000_000.0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(realtimeSeconds),
                    "Realtime clock must be finite, non-negative, and representable in microseconds.");
            }

            return (ulong)(realtimeSeconds * 1_000_000.0);
        }

        void Awake()
        {
            DontDestroyOnLoad(gameObject);
        }

        void OnEnable()
        {
            _paused = false;
            StartBridge(Time.realtimeSinceStartupAsDouble);
        }

        void Update()
        {
            if (_paused)
                return;

            var now = Time.realtimeSinceStartupAsDouble;
            if (!_running)
            {
                if (now >= _nextSocketRetryAt)
                    StartBridge(now);
                return;
            }

            PollReadyMessages();
            if (!_running)
                return;

            if (_discovery.DiscoveryDue(now))
                SendDiscovery(now);
            if (_discovery.HostEndpoint != null && _running)
                SendControllerSample(now);
            UpdatePacketRate(now);
        }

        void StartBridge(double now)
        {
            if (_running || _paused)
                return;

            CloseSocket();
            try
            {
                _udp = new UdpClient(new IPEndPoint(IPAddress.Any, 0));
                _udp.EnableBroadcast = true;
                _udp.Client.Blocking = false;
                _discovery.Reset();
                _running = true;
                _nextSocketRetryAt = 0.0;
                ResetPacketRate(now);
                Status = "Searching for Kinova host";
            }
            catch (SocketException error)
            {
                CloseSocket();
                _running = false;
                _nextSocketRetryAt = now + SocketRetryIntervalSeconds;
                Status = $"UDP unavailable: {error.SocketErrorCode}";
            }
        }

        void PollReadyMessages()
        {
            if (_udp == null)
                return;

            try
            {
                for (var index = 0; index < MaxReadyDatagramsPerFrame; index++)
                {
                    if (_udp.Available <= 0)
                        break;

                    var source = new IPEndPoint(IPAddress.Any, 0);
                    var payload = _udp.Receive(ref source);
                    if (TryAcceptReadyDatagram(payload, source, _discovery))
                        Status = $"Ready: {source.Address}:{source.Port}";
                }
            }
            catch (SocketException error) when (IsWouldBlock(error.SocketErrorCode))
            {
                // A nonblocking poll can race the last datagram. Retry on the next frame.
            }
            catch (SocketException error)
            {
                HandleNetworkFailure(error);
            }
            catch (ObjectDisposedException)
            {
                HandleDisposedSocket();
            }
        }

        void SendDiscovery(double now)
        {
            if (_udp == null)
                return;

            try
            {
                _udp.Send(DiscoverBytes, DiscoverBytes.Length, DiscoveryEndpoint);
                _discovery.MarkDiscoverySent(now);
                Status = "Searching for Kinova host";
            }
            catch (SocketException error) when (IsWouldBlock(error.SocketErrorCode))
            {
                _discovery.MarkDiscoverySent(now);
                Status = "UDP busy; discovery will retry";
            }
            catch (SocketException error)
            {
                HandleNetworkFailure(error);
            }
            catch (ObjectDisposedException)
            {
                HandleDisposedSocket();
            }
        }

        void SendControllerSample(double now)
        {
            if (_udp == null || _discovery.HostEndpoint == null)
                return;

            var sample = ReadLeftController();
            _tracked = sample.Tracked;
            _grip = sample.Grip;
            _sequence = KinovaControllerSequence.Next(_sequence);

            try
            {
                var packet = KinovaControllerPacket.Encode(
                    _sequence,
                    ToSourceTimeUs(now),
                    sample.Tracked,
                    sample.Position,
                    sample.Rotation,
                    sample.Grip);
                _udp.Send(packet, packet.Length, _discovery.HostEndpoint);
                _rateWindowPackets++;
                Status = sample.Tracked ? "Streaming tracked controller" : "Streaming untracked safety state";
            }
            catch (SocketException error) when (IsWouldBlock(error.SocketErrorCode))
            {
                Status = "UDP busy; pose dropped";
            }
            catch (SocketException error)
            {
                HandleNetworkFailure(error);
            }
            catch (ObjectDisposedException)
            {
                HandleDisposedSocket();
            }
            catch (ArgumentException error)
            {
                _tracked = false;
                _grip = 0.0f;
                Status = $"Invalid XR controller pose: {error.Message}";
            }
        }

        static KinovaControllerSample ReadLeftController()
        {
            var device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            var isTracked = false;
            var position = Vector3.zero;
            var rotation = Quaternion.identity;
            var grip = 0.0f;
            var hasTracking = device.isValid
                && device.TryGetFeatureValue(CommonUsages.isTracked, out isTracked);
            var hasPosition = device.isValid
                && device.TryGetFeatureValue(CommonUsages.devicePosition, out position);
            var hasRotation = device.isValid
                && device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation);
            var hasGrip = device.isValid
                && device.TryGetFeatureValue(CommonUsages.grip, out grip);

            return KinovaControllerSample.FromFeatureValues(
                device.isValid,
                hasTracking,
                isTracked,
                hasPosition,
                hasRotation,
                hasGrip,
                position,
                rotation,
                grip);
        }

        void UpdatePacketRate(double now)
        {
            var elapsed = now - _rateWindowStartedAt;
            if (elapsed < 1.0)
                return;

            _packetsPerSecond = elapsed > 0.0
                ? Mathf.RoundToInt((float)(_rateWindowPackets / elapsed))
                : 0;
            _rateWindowStartedAt = now;
            _rateWindowPackets = 0;
        }

        void ResetPacketRate(double now)
        {
            _rateWindowStartedAt = now;
            _rateWindowPackets = 0;
            _packetsPerSecond = 0;
        }

        void HandleNetworkFailure(SocketException error)
        {
            Status = $"Network error {error.SocketErrorCode}; rediscovering";
            RecoverSocket();
        }

        void HandleDisposedSocket()
        {
            Status = "UDP socket closed; rediscovering";
            RecoverSocket();
        }

        void RecoverSocket()
        {
            _discovery.MarkNetworkFailure();
            CloseSocket();
            _running = false;
            _nextSocketRetryAt = 0.0;
        }

        static bool IsWouldBlock(SocketError error)
        {
            return error == SocketError.WouldBlock || error == SocketError.IOPending;
        }

        static bool ByteArraysEqual(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length)
                return false;
            for (var index = 0; index < left.Length; index++)
            {
                if (left[index] != right[index])
                    return false;
            }
            return true;
        }

        void StopBridge()
        {
            _discovery.Reset();
            CloseSocket();
            _running = false;
            _tracked = false;
            _grip = 0.0f;
            _packetsPerSecond = 0;
            Status = "Stopped";
        }

        void CloseSocket()
        {
            if (_udp == null)
                return;

            _udp.Close();
            _udp.Dispose();
            _udp = null;
        }

        void OnApplicationPause(bool pauseStatus)
        {
            _paused = pauseStatus;
            if (pauseStatus)
                StopBridge();
            else if (isActiveAndEnabled)
                StartBridge(Time.realtimeSinceStartupAsDouble);
        }

        void OnDisable()
        {
            StopBridge();
        }

        void OnApplicationQuit()
        {
            StopBridge();
        }

        void OnDestroy()
        {
            StopBridge();
        }
    }

    public static class KinovaBridgeStatus
    {
        public static string Format(
            IPEndPoint hostEndpoint,
            bool tracked,
            float grip,
            uint sequence,
            int packetsPerSecond)
        {
            var host = hostEndpoint == null
                ? "searching"
                : $"{hostEndpoint.Address}:{hostEndpoint.Port}";
            return string.Format(
                CultureInfo.InvariantCulture,
                "Kinova PICO UDP bridge\n"
                + "host: {0}\n"
                + "tracking: {1}\n"
                + "grip: {2:F3}\n"
                + "sequence: {3}\n"
                + "rate: {4}",
                host,
                tracked ? "tracked" : "untracked",
                grip,
                sequence,
                packetsPerSecond);
        }
    }
}

using System;
using System.Net;

namespace Yezqin.KinovaPico
{
    public sealed class KinovaDiscoveryState
    {
        readonly double _discoveryIntervalSeconds;
        double _nextDiscoveryAt;

        public KinovaDiscoveryState(double discoveryIntervalSeconds)
        {
            if (!IsFinite(discoveryIntervalSeconds) || discoveryIntervalSeconds <= 0.0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(discoveryIntervalSeconds),
                    "Discovery interval must be positive and finite.");
            }

            _discoveryIntervalSeconds = discoveryIntervalSeconds;
            Reset();
        }

        public IPEndPoint HostEndpoint { get; private set; }

        public bool DiscoveryDue(double now)
        {
            ValidateClock(now);
            return now >= _nextDiscoveryAt;
        }

        public void MarkDiscoverySent(double now)
        {
            ValidateClock(now);
            _nextDiscoveryAt = now + _discoveryIntervalSeconds;
        }

        public void AcceptReady(IPEndPoint endpoint)
        {
            HostEndpoint = endpoint ?? throw new ArgumentNullException(nameof(endpoint));
        }

        public void MarkNetworkFailure()
        {
            Reset();
        }

        public void Reset()
        {
            HostEndpoint = null;
            _nextDiscoveryAt = double.NegativeInfinity;
        }

        static void ValidateClock(double now)
        {
            if (!IsFinite(now))
                throw new ArgumentOutOfRangeException(nameof(now), "Discovery clock must be finite.");
        }

        static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}

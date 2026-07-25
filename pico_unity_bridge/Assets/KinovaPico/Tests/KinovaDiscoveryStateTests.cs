using System;
using System.Net;
using NUnit.Framework;

namespace Yezqin.KinovaPico.Tests
{
    public sealed class KinovaDiscoveryStateTests
    {
        [Test]
        public void ReadyEndpointKeepsPeriodicDiscoveryDueWithoutClearingHost()
        {
            var state = new KinovaDiscoveryState(1.0);
            var host = new IPEndPoint(IPAddress.Parse("192.168.1.10"), 15031);

            Assert.IsTrue(state.DiscoveryDue(0.0));
            state.MarkDiscoverySent(0.0);
            Assert.IsFalse(state.DiscoveryDue(0.5));
            state.AcceptReady(host);

            Assert.IsFalse(state.DiscoveryDue(0.999));
            Assert.IsTrue(state.DiscoveryDue(1.0));
            Assert.AreSame(host, state.HostEndpoint);

            state.MarkDiscoverySent(1.0);
            Assert.IsFalse(state.DiscoveryDue(1.999));
            Assert.IsTrue(state.DiscoveryDue(2.0));
            var replacement = new IPEndPoint(
                IPAddress.Parse("192.168.1.20"),
                15031);
            state.AcceptReady(replacement);
            Assert.AreSame(replacement, state.HostEndpoint);
        }

        [Test]
        public void DiscoveryBecomesDueAtConfiguredInterval()
        {
            var state = new KinovaDiscoveryState(1.0);

            state.MarkDiscoverySent(5.0);

            Assert.IsFalse(state.DiscoveryDue(5.999));
            Assert.IsTrue(state.DiscoveryDue(6.0));
        }

        [Test]
        public void ResetClearsEndpointAndMakesDiscoveryImmediatelyDue()
        {
            var state = new KinovaDiscoveryState(1.0);
            state.AcceptReady(new IPEndPoint(IPAddress.Loopback, 15031));

            state.Reset();

            Assert.IsNull(state.HostEndpoint);
            Assert.IsTrue(state.DiscoveryDue(100.0));
        }

        [Test]
        public void AcceptReadyRejectsNullEndpoint()
        {
            var state = new KinovaDiscoveryState(1.0);

            Assert.Throws<ArgumentNullException>(() => state.AcceptReady(null));
        }

        [TestCase(0.0)]
        [TestCase(-1.0)]
        [TestCase(double.NaN)]
        [TestCase(double.PositiveInfinity)]
        public void DiscoveryIntervalMustBePositiveAndFinite(double interval)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new KinovaDiscoveryState(interval));
        }

        [TestCase(double.NaN)]
        [TestCase(double.PositiveInfinity)]
        public void DiscoveryClockMustBeFinite(double now)
        {
            var state = new KinovaDiscoveryState(1.0);

            Assert.Throws<ArgumentOutOfRangeException>(() => state.DiscoveryDue(now));
            Assert.Throws<ArgumentOutOfRangeException>(() => state.MarkDiscoverySent(now));
        }
    }
}

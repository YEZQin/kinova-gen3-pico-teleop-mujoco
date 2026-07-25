using System;
using System.Net;
using System.Text;
using NUnit.Framework;
using UnityEngine;

namespace Yezqin.KinovaPico.Tests
{
    public sealed class KinovaControllerUdpBridgeTests
    {
        [TestCase(false, true, true, true, true, true)]
        [TestCase(true, false, true, true, true, true)]
        [TestCase(true, true, false, true, true, true)]
        [TestCase(true, true, true, false, true, true)]
        [TestCase(true, true, true, true, false, true)]
        [TestCase(true, true, true, true, true, false)]
        public void MissingControllerFeatureProducesNeutralSafetySample(
            bool deviceValid,
            bool hasTracking,
            bool isTracked,
            bool hasPosition,
            bool hasRotation,
            bool hasGrip)
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                deviceValid,
                hasTracking,
                isTracked,
                hasPosition,
                hasRotation,
                hasGrip,
                new Vector3(1.0f, 2.0f, 3.0f),
                Quaternion.Euler(10.0f, 20.0f, 30.0f),
                0.75f);

            Assert.IsFalse(sample.Tracked);
            Assert.AreEqual(Vector3.zero, sample.Position);
            Assert.AreEqual(Quaternion.identity, sample.Rotation);
            Assert.AreEqual(0.0f, sample.Grip);
        }

        [Test]
        public void InvalidTrackedFeatureValuesProduceNeutralSafetySample()
        {
            var nonFinitePosition = KinovaControllerSample.FromFeatureValues(
                true, true, true, true, true, true,
                new Vector3(float.NaN, 0.0f, 0.0f),
                Quaternion.identity,
                0.5f);
            var zeroRotation = KinovaControllerSample.FromFeatureValues(
                true, true, true, true, true, true,
                Vector3.zero,
                new Quaternion(0.0f, 0.0f, 0.0f, 0.0f),
                0.5f);
            var nonFiniteGrip = KinovaControllerSample.FromFeatureValues(
                true, true, true, true, true, true,
                Vector3.zero,
                Quaternion.identity,
                float.NaN);

            Assert.IsFalse(nonFinitePosition.Tracked);
            Assert.IsFalse(zeroRotation.Tracked);
            Assert.IsFalse(nonFiniteGrip.Tracked);
            Assert.AreEqual(Vector3.zero, nonFinitePosition.Position);
            Assert.AreEqual(Quaternion.identity, zeroRotation.Rotation);
            Assert.AreEqual(0.0f, nonFiniteGrip.Grip);
        }

        [TestCase(float.NaN, 0.0f, 0.0f, 1.0f)]
        [TestCase(0.0f, float.PositiveInfinity, 0.0f, 1.0f)]
        public void NonFiniteQuaternionProducesFullyNeutralSafetySample(
            float x,
            float y,
            float z,
            float w)
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                new Vector3(1.0f, 2.0f, 3.0f),
                new Quaternion(x, y, z, w),
                0.75f);

            Assert.IsFalse(sample.Tracked);
            Assert.AreEqual(Vector3.zero, sample.Position);
            Assert.AreEqual(Quaternion.identity, sample.Rotation);
            Assert.AreEqual(0.0f, sample.Grip);
        }

        [Test]
        public void DiscoveryDatagramTargetsProtocolBroadcastEndpoint()
        {
            CollectionAssert.AreEqual(
                Encoding.ASCII.GetBytes("KINOVA_DISCOVER_V1"),
                KinovaControllerUdpBridge.CreateDiscoveryDatagram());
            Assert.AreEqual(IPAddress.Broadcast, KinovaControllerUdpBridge.DiscoveryEndpoint.Address);
            Assert.AreEqual(15031, KinovaControllerUdpBridge.DiscoveryEndpoint.Port);
        }

        [Test]
        public void ExactReadyDatagramSelectsSenderAsHost()
        {
            var state = new KinovaDiscoveryState(1.0);
            var source = new IPEndPoint(IPAddress.Parse("192.168.1.20"), 15031);

            var accepted = KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                Encoding.ASCII.GetBytes("KINOVA_READY_V1"),
                source,
                state);

            Assert.IsTrue(accepted);
            Assert.AreSame(source, state.HostEndpoint);
        }

        [Test]
        public void LaterExactReadyDatagramReplacesExistingHost()
        {
            var state = new KinovaDiscoveryState(1.0);
            var original = new IPEndPoint(
                IPAddress.Parse("192.168.1.10"),
                15031);
            var replacement = new IPEndPoint(
                IPAddress.Parse("192.168.1.20"),
                15031);
            state.AcceptReady(original);

            var accepted = KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                Encoding.ASCII.GetBytes("KINOVA_READY_V1"),
                replacement,
                state);

            Assert.IsTrue(accepted);
            Assert.AreSame(replacement, state.HostEndpoint);
        }

        [TestCase("KINOVA_READY_V1_EXTRA")]
        [TestCase("kinova_ready_v1")]
        [TestCase("")]
        public void NonProtocolDatagramCannotSelectHost(string text)
        {
            var state = new KinovaDiscoveryState(1.0);

            var accepted = KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                Encoding.ASCII.GetBytes(text),
                new IPEndPoint(IPAddress.Loopback, 15031),
                state);

            Assert.IsFalse(accepted);
            Assert.IsNull(state.HostEndpoint);
        }

        [TestCase("KINOVA_READY_V1_EXTRA")]
        [TestCase("kinova_ready_v1")]
        [TestCase("")]
        public void MalformedReadyDatagramPreservesExistingHost(string text)
        {
            var state = new KinovaDiscoveryState(1.0);
            var original = new IPEndPoint(
                IPAddress.Parse("192.168.1.10"),
                15031);
            state.AcceptReady(original);

            var accepted = KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                Encoding.ASCII.GetBytes(text),
                new IPEndPoint(IPAddress.Parse("192.168.1.20"), 15031),
                state);

            Assert.IsFalse(accepted);
            Assert.AreSame(original, state.HostEndpoint);
        }

        [Test]
        public void ReadyDatagramRejectsMissingInputs()
        {
            var state = new KinovaDiscoveryState(1.0);
            var source = new IPEndPoint(IPAddress.Loopback, 15031);

            Assert.Throws<ArgumentNullException>(() =>
                KinovaControllerUdpBridge.TryAcceptReadyDatagram(null, source, state));
            Assert.Throws<ArgumentNullException>(() =>
                KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                    Encoding.ASCII.GetBytes("KINOVA_READY_V1"),
                    null,
                    state));
            Assert.Throws<ArgumentNullException>(() =>
                KinovaControllerUdpBridge.TryAcceptReadyDatagram(
                    Encoding.ASCII.GetBytes("KINOVA_READY_V1"),
                    source,
                    null));
        }

        [Test]
        public void RealtimeClockConvertsToMonotonicMicroseconds()
        {
            Assert.AreEqual(1_234_567ul, KinovaControllerUdpBridge.ToSourceTimeUs(1.234567));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                KinovaControllerUdpBridge.ToSourceTimeUs(-0.001));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                KinovaControllerUdpBridge.ToSourceTimeUs(double.NaN));
        }

        [Test]
        public void StatusFormatterShowsRequiredBridgeFields()
        {
            Assert.AreEqual(
                "Kinova PICO UDP bridge\n"
                + "host: 192.168.1.20:15031\n"
                + "tracking: tracked\n"
                + "grip: 0.375\n"
                + "sequence: 42\n"
                + "rate: 72",
                KinovaBridgeStatus.Format(
                    new IPEndPoint(IPAddress.Parse("192.168.1.20"), 15031),
                    true,
                    0.375f,
                    42u,
                    72));
        }

        [Test]
        public void StatusFormatterShowsSearchingAndNeutralSafetyState()
        {
            Assert.AreEqual(
                "Kinova PICO UDP bridge\n"
                + "host: searching\n"
                + "tracking: untracked\n"
                + "grip: 0.000\n"
                + "sequence: 0\n"
                + "rate: 0",
                KinovaBridgeStatus.Format(null, false, 0.0f, 0u, 0));
        }
    }
}

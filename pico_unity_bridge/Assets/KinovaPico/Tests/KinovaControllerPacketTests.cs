using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace Yezqin.KinovaPico.Tests
{
    public sealed class KinovaControllerPacketTests
    {
        [Test]
        public void TrackedPacketWritesCanonicalLittleEndianFixture()
        {
            var payload = KinovaControllerPacket.Encode(
                42u,
                1_234_567ul,
                true,
                new Vector3(1.0f, -2.0f, 0.5f),
                Quaternion.identity,
                0.25f,
                0.5f);

            Assert.AreEqual(60, payload.Length);
            CollectionAssert.AreEqual(
                new byte[]
                {
                    (byte)'K', (byte)'I', (byte)'N', (byte)'V',
                    (byte)'P', (byte)'I', (byte)'C', (byte)'O',
                },
                payload.Take(8).ToArray());
            CollectionAssert.AreEqual(
                new byte[]
                {
                    2, 1, 0, 0,
                    42, 0, 0, 0,
                    0x87, 0xD6, 0x12, 0, 0, 0, 0, 0,
                    0, 0, 0x80, 0x3F,
                    0, 0, 0, 0xC0,
                    0, 0, 0, 0x3F,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0x80, 0x3F,
                    0, 0, 0x80, 0x3E,
                    0, 0, 0, 0x3F,
                },
                payload.Skip(8).ToArray());
            Assert.AreEqual(42u, BitConverter.ToUInt32(payload, 12));
            Assert.AreEqual(1_234_567ul, BitConverter.ToUInt64(payload, 16));
        }

        [Test]
        public void UntrackedPacketAlwaysWritesNeutralSafetyPayload()
        {
            var payload = KinovaControllerPacket.Encode(
                9u,
                12ul,
                false,
                new Vector3(1.0f, 2.0f, 3.0f),
                Quaternion.Euler(20.0f, 30.0f, 40.0f),
                0.75f,
                0.6f);

            Assert.AreEqual(0, payload[9]);
            CollectionAssert.AreEqual(new byte[12], payload.Skip(24).Take(12).ToArray());
            CollectionAssert.AreEqual(new byte[12], payload.Skip(36).Take(12).ToArray());
            CollectionAssert.AreEqual(
                new byte[] { 0, 0, 0x80, 0x3F },
                payload.Skip(48).Take(4).ToArray());
            CollectionAssert.AreEqual(new byte[4], payload.Skip(52).Take(4).ToArray());
            CollectionAssert.AreEqual(new byte[4], payload.Skip(56).Take(4).ToArray());
        }

        [Test]
        public void TrackedPacketRejectsNonFinitePosition()
        {
            Assert.Throws<ArgumentException>(() => KinovaControllerPacket.Encode(
                0u,
                0ul,
                true,
                new Vector3(float.NaN, 0.0f, 0.0f),
                Quaternion.identity,
                0.0f,
                0.0f));
        }

        [Test]
        public void TrackedPacketRejectsNonFiniteRotation()
        {
            Assert.Throws<ArgumentException>(() => KinovaControllerPacket.Encode(
                0u,
                0ul,
                true,
                Vector3.zero,
                new Quaternion(0.0f, float.PositiveInfinity, 0.0f, 1.0f),
                0.0f,
                0.0f));
        }

        [Test]
        public void TrackedPacketRejectsZeroQuaternion()
        {
            Assert.Throws<ArgumentException>(() => KinovaControllerPacket.Encode(
                0u,
                0ul,
                true,
                Vector3.zero,
                new Quaternion(0.0f, 0.0f, 0.0f, 0.0f),
                0.0f,
                0.0f));
        }

        [TestCase(-0.001f)]
        [TestCase(1.001f)]
        [TestCase(float.NaN)]
        [TestCase(float.PositiveInfinity)]
        public void TrackedPacketRejectsInvalidGrip(float grip)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => KinovaControllerPacket.Encode(
                0u,
                0ul,
                true,
                Vector3.zero,
                Quaternion.identity,
                grip,
                0.0f));
        }

        [TestCase(-0.001f)]
        [TestCase(1.001f)]
        [TestCase(float.NaN)]
        [TestCase(float.PositiveInfinity)]
        public void TrackedPacketRejectsInvalidTrigger(float trigger)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => KinovaControllerPacket.Encode(
                0u,
                0ul,
                true,
                Vector3.zero,
                Quaternion.identity,
                0.0f,
                trigger));
        }

        [TestCase(-2.0f, 0.0f)]
        [TestCase(0.375f, 0.375f)]
        [TestCase(3.0f, 1.0f)]
        public void SamplingLayerClampsGripBeforePacketCreation(float rawGrip, float expected)
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                Vector3.zero,
                Quaternion.identity,
                rawGrip);

            Assert.AreEqual(expected, sample.Grip);
            Assert.DoesNotThrow(() => KinovaControllerPacket.Encode(
                1u,
                1ul,
                sample.Tracked,
                sample.Position,
                sample.Rotation,
                sample.Grip,
                sample.Trigger));
        }

        [TestCase(-2.0f, 0.0f)]
        [TestCase(0.375f, 0.375f)]
        [TestCase(3.0f, 1.0f)]
        [TestCase(float.NaN, 0.0f)]
        public void SamplingLayerClampsTriggerBeforePacketCreation(
            float rawTrigger,
            float expected)
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                Vector3.zero,
                Quaternion.identity,
                0.5f,
                true,
                rawTrigger);

            Assert.IsTrue(sample.Tracked);
            Assert.AreEqual(expected, sample.Trigger);
            Assert.DoesNotThrow(() => KinovaControllerPacket.Encode(
                1u,
                1ul,
                sample.Tracked,
                sample.Position,
                sample.Rotation,
                sample.Grip,
                sample.Trigger));
        }

        [Test]
        public void SamplingLayerWithoutTriggerFeatureStaysTracked()
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                Vector3.zero,
                Quaternion.identity,
                0.5f);

            Assert.IsTrue(sample.Tracked);
            Assert.AreEqual(0.0f, sample.Trigger);
        }

        [Test]
        public void SamplingLayerNormalizesNonUnitQuaternionBeforePacketCreation()
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                Vector3.zero,
                new Quaternion(0.0f, 0.0f, 1.2f, 1.6f),
                0.5f);

            Assert.IsTrue(sample.Tracked);
            Assert.That(sample.Rotation.x, Is.EqualTo(0.0f).Within(0.000001f));
            Assert.That(sample.Rotation.y, Is.EqualTo(0.0f).Within(0.000001f));
            Assert.That(sample.Rotation.z, Is.EqualTo(0.6f).Within(0.000001f));
            Assert.That(sample.Rotation.w, Is.EqualTo(0.8f).Within(0.000001f));

            var payload = KinovaControllerPacket.Encode(
                1u,
                1ul,
                sample.Tracked,
                sample.Position,
                sample.Rotation,
                sample.Grip,
                sample.Trigger);
            Assert.AreEqual(0.0f, BitConverter.ToSingle(payload, 36));
            Assert.AreEqual(0.0f, BitConverter.ToSingle(payload, 40));
            Assert.That(
                BitConverter.ToSingle(payload, 44),
                Is.EqualTo(0.6f).Within(0.000001f));
            Assert.That(
                BitConverter.ToSingle(payload, 48),
                Is.EqualTo(0.8f).Within(0.000001f));
        }

        [Test]
        public void SamplingLayerRejectsNegligibleQuaternionNorm()
        {
            var sample = KinovaControllerSample.FromFeatureValues(
                true,
                true,
                true,
                true,
                true,
                true,
                Vector3.zero,
                new Quaternion(0.00000001f, 0.0f, 0.0f, 0.0f),
                0.5f,
                true,
                0.75f);

            Assert.IsFalse(sample.Tracked);
            Assert.AreEqual(Quaternion.identity, sample.Rotation);
            Assert.AreEqual(0.0f, sample.Grip);
            Assert.AreEqual(0.0f, sample.Trigger);
        }

        [Test]
        public void SequenceWrapsFromMaximumToZero()
        {
            Assert.AreEqual(0u, KinovaControllerSequence.Next(uint.MaxValue));
        }
    }
}

using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace Yezqin.KinovaPico
{
    public static class KinovaControllerPacket
    {
        public const int PacketSize = 60;
        public const byte Version = 2;
        public const byte TrackedFlag = 1;

        static readonly byte[] Magic =
        {
            (byte)'K', (byte)'I', (byte)'N', (byte)'V',
            (byte)'P', (byte)'I', (byte)'C', (byte)'O',
        };

        public static byte[] Encode(
            uint sequence,
            ulong sourceTimeUs,
            bool tracked,
            Vector3 position,
            Quaternion rotation,
            float grip,
            float trigger)
        {
            Validate(tracked, position, rotation, grip, trigger);

            var safePosition = tracked ? position : Vector3.zero;
            var safeRotation = tracked ? rotation : Quaternion.identity;
            var safeGrip = tracked ? grip : 0.0f;
            var safeTrigger = tracked ? trigger : 0.0f;

            using (var stream = new MemoryStream(PacketSize))
            using (var writer = new BinaryWriter(stream, Encoding.UTF8, true))
            {
                writer.Write(Magic);
                writer.Write(Version);
                writer.Write((byte)(tracked ? TrackedFlag : 0));
                writer.Write((ushort)0);
                writer.Write(sequence);
                writer.Write(sourceTimeUs);
                writer.Write(safePosition.x);
                writer.Write(safePosition.y);
                writer.Write(safePosition.z);
                writer.Write(safeRotation.x);
                writer.Write(safeRotation.y);
                writer.Write(safeRotation.z);
                writer.Write(safeRotation.w);
                writer.Write(safeGrip);
                writer.Write(safeTrigger);
                writer.Flush();

                if (stream.Position != PacketSize)
                {
                    throw new InvalidOperationException(
                        $"Kinova controller packet must be exactly {PacketSize} bytes.");
                }
                return stream.ToArray();
            }
        }

        static void Validate(
            bool tracked,
            Vector3 position,
            Quaternion rotation,
            float grip,
            float trigger)
        {
            if (!tracked)
                return;
            if (!IsFinite(position))
                throw new ArgumentException("Tracked position must be finite.", nameof(position));
            if (!IsFinite(rotation))
                throw new ArgumentException("Tracked rotation must be finite.", nameof(rotation));
            if (rotation.x == 0.0f
                && rotation.y == 0.0f
                && rotation.z == 0.0f
                && rotation.w == 0.0f)
            {
                throw new ArgumentException("Tracked rotation cannot be the zero quaternion.", nameof(rotation));
            }
            if (!IsFinite(grip) || grip < 0.0f || grip > 1.0f)
                throw new ArgumentOutOfRangeException(nameof(grip), "Tracked Grip must be finite and in [0, 1].");
            if (!IsFinite(trigger) || trigger < 0.0f || trigger > 1.0f)
                throw new ArgumentOutOfRangeException(nameof(trigger), "Tracked Trigger must be finite and in [0, 1].");
        }

        static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x)
                && IsFinite(value.y)
                && IsFinite(value.z)
                && IsFinite(value.w);
        }

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }

    public readonly struct KinovaControllerSample
    {
        const double MinimumRotationNorm = 0.000001;

        public KinovaControllerSample(
            bool tracked,
            Vector3 position,
            Quaternion rotation,
            float grip,
            float trigger)
        {
            Tracked = tracked;
            Position = position;
            Rotation = rotation;
            Grip = grip;
            Trigger = trigger;
        }

        public bool Tracked { get; }
        public Vector3 Position { get; }
        public Quaternion Rotation { get; }
        public float Grip { get; }
        public float Trigger { get; }

        public static KinovaControllerSample FromFeatureValues(
            bool deviceValid,
            bool hasTracking,
            bool isTracked,
            bool hasPosition,
            bool hasRotation,
            bool hasGrip,
            Vector3 position,
            Quaternion rotation,
            float rawGrip,
            bool hasTrigger = false,
            float rawTrigger = 0.0f)
        {
            if (!(deviceValid
                && hasTracking
                && isTracked
                && hasPosition
                && hasRotation
                && hasGrip)
                || !IsFinite(position)
                || !IsFinite(rawGrip))
            {
                return Untracked;
            }

            if (!TryNormalizeRotation(rotation, out var normalizedRotation))
                return Untracked;

            // The trigger channel is optional so V1-era callers keep working;
            // a missing or invalid trigger degrades to 0 without untracking.
            var trigger = hasTrigger && IsFinite(rawTrigger)
                ? Mathf.Clamp01(rawTrigger)
                : 0.0f;

            return new KinovaControllerSample(
                true,
                position,
                normalizedRotation,
                Mathf.Clamp01(rawGrip),
                trigger);
        }

        public static KinovaControllerSample Untracked =>
            new KinovaControllerSample(false, Vector3.zero, Quaternion.identity, 0.0f, 0.0f);

        static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x)
                && IsFinite(value.y)
                && IsFinite(value.z)
                && IsFinite(value.w);
        }

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        static bool TryNormalizeRotation(
            Quaternion value,
            out Quaternion normalized)
        {
            normalized = Quaternion.identity;
            if (!IsFinite(value))
                return false;

            var x = (double)value.x;
            var y = (double)value.y;
            var z = (double)value.z;
            var w = (double)value.w;
            var normSquared = x * x + y * y + z * z + w * w;
            if (normSquared <= MinimumRotationNorm * MinimumRotationNorm)
                return false;

            var inverseNorm = 1.0 / Math.Sqrt(normSquared);
            normalized = new Quaternion(
                (float)(x * inverseNorm),
                (float)(y * inverseNorm),
                (float)(z * inverseNorm),
                (float)(w * inverseNorm));
            return IsFinite(normalized);
        }
    }

    public static class KinovaControllerSequence
    {
        public static uint Next(uint current)
        {
            unchecked
            {
                return current + 1u;
            }
        }
    }
}

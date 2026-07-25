using System;
using NUnit.Framework;
using UnityEditor;
using UnityEngine.XR.OpenXR.Features.Interactions;

namespace Yezqin.KinovaPico.Tests
{
    public sealed class KinovaBuildConfigurationTests
    {
        [Test]
        public void BuildSpecTargetsTheApprovedAndroidApplication()
        {
            var spec = Editor.KinovaPicoBuildSpec.Approved;

            Assert.AreEqual("com.yezqin.kinovapicobridge", spec.PackageId);
            Assert.AreEqual("Kinova PICO Bridge", spec.ApplicationName);
            Assert.AreEqual(AndroidArchitecture.ARM64, spec.Architectures);
            Assert.AreEqual(ScriptingImplementation.IL2CPP, spec.ScriptingBackend);
            Assert.AreEqual(AndroidSdkVersions.AndroidApiLevel29, spec.MinimumSdkVersion);
            Assert.AreEqual(
                "Builds/KinovaPicoBridge-development.apk",
                spec.BuildPath.Replace('\\', '/'));
        }

        [Test]
        public void BuildSpecEnablesOnlyThePico4UltraControllerProfile()
        {
            var profiles = Editor.KinovaPicoBuildSpec.Approved.ControllerProfileTypes;

            CollectionAssert.AreEqual(
                new[] { typeof(PICO4UltraControllerProfile) },
                profiles);
            Assert.IsFalse(Array.Exists(
                profiles,
                profile => profile.Name.Contains("Hand", StringComparison.OrdinalIgnoreCase)));
        }
    }
}

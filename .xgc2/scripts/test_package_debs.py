#!/usr/bin/env python3
"""Exercise real Deb payload assembly using isolated, synthetic install trees."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class PackageDebsTest(unittest.TestCase):
    def assemble(self, distro, missing_package=None):
        temporary = tempfile.TemporaryDirectory(prefix="scene-generation-deb-test-")
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        install = root / "install"
        output = root / "debs"
        prefix = install / "opt" / "ros" / distro
        # A foreign installed message tree must never be repackaged by this owner.
        packages = ["cluttered_environment", "mockamap", "xgc2_scene_runtime",
                    "xgc2_world_lidar", "xgc2_geometry_msgs"]
        python_dir = "python2.7" if distro == "melodic" else "python3"
        for package in packages:
            if package == missing_package:
                continue
            files = {
                "share/{}/package.xml".format(package): "<package/>",
                "include/{}/ConvexBodyArray.h".format(package): "// fixture\n",
                "lib/pkgconfig/{}.pc".format(package): "Name: {}\n".format(package),
                "lib/{}/dist-packages/{}/__init__.py".format(python_dir, package): "# fixture\n",
            }
            for name, content in files.items():
                path = prefix / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
        environment = os.environ.copy()
        environment.update(ROS_DISTRO=distro, PACKAGE_VERSION="1.2.0-15")
        result = subprocess.run(
            [str(ROOT / ".xgc2/scripts/package_debs.sh"),
             "--install-root", str(install), "--output-dir", str(output)],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
        )
        return root, output, result

    def test_melodic_messages_are_no_longer_published_here(self):
        _, _, result = self.assemble("melodic")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported ROS_DISTRO", result.stderr)

    def test_noetic_preserves_implementations_without_repackaging_messages(self):
        root, output, result = self.assemble("noetic")
        self.assertEqual(result.returncode, 0, result.stderr)
        actual = {subprocess.check_output(["dpkg-deb", "-f", str(deb), "Package"], universal_newlines=True).strip()
                  for deb in output.glob("*.deb")}
        self.assertEqual(actual, {"ros-noetic-xgc2-" + name for name in
                                 ["cluttered-environment", "mockamap", "scene-generation", "scene-runtime",
                                  "world-lidar"]})
        for deb in output.glob("*.deb"):
            payload = root / deb.stem
            subprocess.check_call(["dpkg-deb", "-x", str(deb), str(payload)])
            self.assertFalse(any("xgc2_geometry_msgs" in str(path) for path in payload.rglob("*")))

    def test_runtime_message_dependency_has_an_independent_version(self):
        _, output, result = self.assemble("noetic")
        self.assertEqual(result.returncode, 0, result.stderr)
        for package in ("scene-runtime", "world-lidar"):
            deb = next(output.glob("ros-noetic-xgc2-{}_*.deb".format(package)))
            depends = subprocess.check_output(["dpkg-deb", "-f", str(deb), "Depends"], universal_newlines=True)
            self.assertIn("ros-noetic-xgc2-geometry-msgs (>= 1.2.0-13)", depends)
            self.assertNotIn("ros-noetic-xgc2-geometry-msgs (=", depends)

    def test_missing_implementation_install_fails_instead_of_empty_deb(self):
        _, _, result = self.assemble("noetic", missing_package="cluttered_environment")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing installed ROS package", result.stderr)

    def test_unknown_distro_fails(self):
        _, _, result = self.assemble("rolling")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported ROS_DISTRO", result.stderr)


if __name__ == "__main__":
    unittest.main()

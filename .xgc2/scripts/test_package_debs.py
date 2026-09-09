#!/usr/bin/env python3
"""Exercise real Deb payload assembly using isolated, synthetic install trees."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class PackageDebsTest(unittest.TestCase):
    def assemble(self, distro, include_messages=True):
        temporary = tempfile.TemporaryDirectory(prefix="scene-generation-deb-test-")
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        install = root / "install"
        output = root / "debs"
        prefix = install / "opt" / "ros" / distro
        # Include the environment packages even in Melodic's fixture: their
        # presence must never silently expand that suite's declared install set.
        packages = ["cluttered_environment", "mockamap"]
        if include_messages:
            packages.append("xgc2_geometry_msgs")
        python_dir = "python2.7" if distro == "melodic" else "python3"
        for package in packages:
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
        environment.update(ROS_DISTRO=distro, PACKAGE_VERSION="1.1.4-14")
        result = subprocess.run(
            [str(ROOT / ".xgc2/scripts/package_debs.sh"),
             "--install-root", str(install), "--output-dir", str(output)],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
        )
        return root, output, result

    def test_melodic_contains_only_message_package_and_python2(self):
        root, output, result = self.assemble("melodic")
        self.assertEqual(result.returncode, 0, result.stderr)
        debs = list(output.glob("*.deb"))
        self.assertEqual(len(debs), 1)
        self.assertTrue(debs[0].name.startswith("ros-melodic-xgc2-geometry-msgs_"))
        depends = subprocess.check_output(["dpkg-deb", "-f", str(debs[0]), "Depends"], universal_newlines=True)
        self.assertIn("ros-melodic-message-runtime", depends)
        self.assertNotIn("noetic", depends)
        self.assertNotIn("libxgc2-math", depends)
        payload = root / "payload"
        subprocess.check_call(["dpkg-deb", "-x", str(debs[0]), str(payload)])
        prefix = payload / "opt/ros/melodic"
        self.assertTrue((prefix / "lib/python2.7/dist-packages/xgc2_geometry_msgs/__init__.py").is_file())
        self.assertTrue((prefix / "lib/pkgconfig/xgc2_geometry_msgs.pc").is_file())
        self.assertFalse((payload / "opt/ros/noetic").exists())

    def test_noetic_preserves_all_existing_packages(self):
        _, output, result = self.assemble("noetic")
        self.assertEqual(result.returncode, 0, result.stderr)
        actual = {subprocess.check_output(["dpkg-deb", "-f", str(deb), "Package"], universal_newlines=True).strip()
                  for deb in output.glob("*.deb")}
        self.assertEqual(actual, {"ros-noetic-xgc2-" + name for name in
                                 ["geometry-msgs", "cluttered-environment", "mockamap", "scene-generation"]})

    def test_missing_message_install_fails_instead_of_empty_deb(self):
        _, _, result = self.assemble("melodic", include_messages=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing installed ROS package", result.stderr)

    def test_unknown_distro_fails(self):
        _, _, result = self.assemble("rolling")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported ROS_DISTRO", result.stderr)


if __name__ == "__main__":
    unittest.main()

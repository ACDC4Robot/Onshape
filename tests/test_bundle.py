import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import stat
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from xml.etree import ElementTree as ET
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from acdc4robot_onshape.bundle import inspect_bundle, prepare_bundle
from acdc4robot_onshape.cli import main
from acdc4robot_onshape.serialization import decimal_number


MESH = b'''solid triangle
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
endsolid triangle
'''
INERTIAL = '''<inertial><origin xyz="1e-3 -2e-3 0" rpy="0.1 0.2 0.3"/>
<mass value="2.5e-3"/><inertia ixx="2e-8" iyy="3e-8" izz="4e-8" ixy="1e-9" ixz="-1e-9" iyz="2e-9"/></inertial>'''
ROBOT = f'''<robot name="example"><!-- source comment -->
<material name="blue"><color rgba="0 0 1 1"/><texture filename="../media/paint.png"/></material>
<link name="world"/><link name="base">{INERTIAL}
 <visual name="cad"><origin xyz="1e-2 0 0" rpy="0 0 1.57"/><geometry><mesh filename="../geometry/part.stl" scale="1e-3 2e-3 3e-3"/></geometry><material name="blue"/></visual>
 <collision name="contact"><origin xyz="-1e-2 0 0" rpy="0.2 0.3 0.4"/><geometry><mesh filename="../geometry/part.stl" scale="1e-3 2e-3 3e-3"/></geometry></collision>
</link><link name="tip">{INERTIAL}</link>
<joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint>
<joint name="elbow" type="revolute"><parent link="base"/><child link="tip"/><origin xyz="0 0 1e-2" rpy="0.1 0.2 0.3"/><axis xyz="0 0 1"/><limit lower="-1.2e-3" upper="2e-3" effort="1" velocity="2"/></joint>
</robot>'''


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.source = self.folder / "export"
        for directory in ("description", "geometry", "media"):
            (self.source / directory).mkdir(parents=True)
        self.urdf = self.source / "description" / "assembly.urdf"
        self.urdf.write_text(ROBOT, encoding="utf-8")
        (self.source / "geometry/part.stl").write_bytes(MESH)
        (self.source / "media/paint.png").write_bytes(b"image byte preservation fixture")
        self.output = self.folder / "prepared"

    def mutate(self, operation):
        root = ET.parse(self.urdf).getroot()
        operation(root)
        ET.ElementTree(root).write(self.urdf, encoding="utf-8")

    def assert_rejected(self, message=None):
        context = self.assertRaisesRegex(ValueError, message) if message else self.assertRaises(ValueError)
        with context:
            prepare_bundle(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.folder.glob(".prepared.stage-*")), [])

    def test_preparation_preserves_physics_names_source_and_asset_bytes(self):
        before = {path.relative_to(self.source): path.read_bytes() for path in self.source.rglob("*") if path.is_file()}
        report = prepare_bundle(self.source, self.output,
            metadata={"source_url": "https://cad.onshape.com/documents/example/v/frozen/e/assembly",
                      "revision": "frozen", "configuration": {"length": "10 mm"}})
        self.assertEqual(report["validation"]["root_link"], "world")
        self.assertEqual(report["validation"]["massless_fixed_frames"], ["world"])
        self.assertFalse(report["live_onshape_validation"])
        root = ET.parse(self.output / "robot.urdf").getroot()
        base = root.find("link[@name='base']")
        self.assertEqual(base.find("visual/origin").get("rpy"), "0 0 1.57")
        self.assertEqual(base.find("collision/origin").get("xyz"), "-0.01 0 0")
        self.assertEqual(base.find("inertial/origin").get("rpy"), "0.1 0.2 0.3")
        self.assertEqual(base.find("inertial/inertia").get("ixx"), "0.00000002")
        self.assertEqual(base.find("inertial/mass").get("value"), "0.0025")
        self.assertEqual(base.find("visual/geometry/mesh").get("scale"), "0.001 0.002 0.003")
        self.assertEqual(base.find("visual/material").get("name"), "blue")
        self.assertIsNone(root.find("link[@name='world']/inertial"))
        self.assertEqual(root.find("joint[@name='elbow']/limit").get("lower"), "-0.0012")
        self.assertEqual((self.output / base.find("visual/geometry/mesh").get("filename")).read_bytes(), MESH)
        self.assertIn("source comment", (self.output / "robot.urdf").read_text())
        self.assertEqual((self.output / "provenance/source-urdf.xml").read_bytes(), before[Path("description/assembly.urdf")])
        self.assertEqual({path.relative_to(self.source): path.read_bytes() for path in self.source.rglob("*") if path.is_file()}, before)
        saved = json.loads((self.output / "acdc4robot-onshape-report.json").read_text())
        self.assertEqual(saved, report)
        self.assertEqual(saved["onshape"]["configuration"], {"length": "10 mm"})

    def test_zip_layout_selection_and_source_archive_integrity(self):
        archive = self.folder / "download.zip"
        with zipfile.ZipFile(archive, "w") as target:
            for path in self.source.rglob("*"):
                if path.is_file():
                    target.write(path, "arbitrary-root/" + path.relative_to(self.source).as_posix())
        before = archive.read_bytes()
        report = prepare_bundle(archive, self.output)
        self.assertEqual(report["input"]["urdf"], "arbitrary-root/description/assembly.urdf")
        self.assertEqual(archive.read_bytes(), before)
        self.assertIn("archive_sha256", report["input"])
        (self.source / "second.urdf").write_text('<robot name="other"><link name="frame"/></robot>')
        with self.assertRaisesRegex(ValueError, "--urdf"):
            inspect_bundle(self.source)
        self.assertEqual(inspect_bundle(self.source, urdf="description/assembly.urdf")["validation"]["links"], 3)

    def test_explicit_package_roots_and_asset_escape_rejection(self):
        self.urdf.write_text(ROBOT.replace("../geometry/part.stl", "package://robot/part.stl"))
        self.assert_rejected("explicit --package")
        report = prepare_bundle(self.source, self.output, package_roots={"robot": self.source / "geometry"})
        self.assertEqual(report["assets"][0]["source_reference"], "package://robot/part.stl")
        self.assertFalse("package://" in (self.output / "robot.urdf").read_text())

    def test_relative_traversal_external_symlinks_and_remote_uris_rejected(self):
        outside = self.folder / "outside.stl"
        outside.write_bytes(MESH)
        for reference in ("../../outside.stl", "https://example.com/part.stl", "file:///tmp/part.stl"):
            self.urdf.write_text(ROBOT.replace("../geometry/part.stl", reference))
            self.assert_rejected()
        link = self.source / "geometry/link.stl"
        link.symlink_to(outside)
        self.urdf.write_text(ROBOT.replace("../geometry/part.stl", "../geometry/link.stl"))
        self.assert_rejected("escapes")

    def test_unsafe_zip_entries_and_symlinks_rejected(self):
        for filename, mode in (("../escape", 0), ("/absolute", 0), ("link", stat.S_IFLNK | 0o777)):
            archive = self.folder / "unsafe.zip"
            info = zipfile.ZipInfo(filename)
            info.external_attr = mode << 16
            with zipfile.ZipFile(archive, "w") as target:
                target.writestr(info, "content")
            with self.subTest(filename=filename), self.assertRaises(ValueError):
                prepare_bundle(archive, self.output)
            self.assertFalse(self.output.exists())
        archive = self.folder / "duplicate.zip"
        with zipfile.ZipFile(archive, "w") as target:
            target.writestr("Robot.urdf", "a")
            target.writestr("robot.urdf", "b")
        with self.assertRaisesRegex(ValueError, "case-colliding"):
            prepare_bundle(archive, self.output)

    def test_invalid_graphs_joints_axes_and_numbers_rejected(self):
        mutations = [
            lambda root: root.append(deepcopy(root.find("link"))),
            lambda root: root.find("joint[@name='elbow']/child").set("link", "missing"),
            lambda root: root.find("joint[@name='elbow']/child").set("link", "base"),
            lambda root: root.find("joint[@name='elbow']").set("type", "floating"),
            lambda root: root.find("joint[@name='elbow']/axis").set("xyz", "0 0 0"),
            lambda root: root.find("joint[@name='elbow']/origin").set("rpy", "nan 0 0"),
            lambda root: root.find("joint[@name='elbow']/origin").set("xyz", "1 2"),
            lambda root: root.find("link[@name='base']/visual/geometry/mesh").set("scale", "0 1 1"),
            lambda root: root.find("joint[@name='elbow']/limit").set("upper", "-100"),
        ]
        for operation in mutations:
            self.urdf.write_text(ROBOT)
            self.mutate(operation)
            with self.subTest(operation=operation):
                self.assert_rejected()

    def test_disconnected_cycle_and_mimic_cycle_rejected(self):
        self.mutate(lambda root: root.append(ET.Element("link", name="extra")))
        self.assert_rejected("one root")
        self.urdf.write_text(ROBOT)
        self.mutate(lambda root: root.find("joint[@name='mount']/parent").set("link", "tip"))
        self.assert_rejected("cycle")
        self.urdf.write_text(ROBOT)
        def mimic_cycle(root):
            ET.SubElement(root.find("joint[@name='mount']"), "mimic", joint="elbow")
            ET.SubElement(root.find("joint[@name='elbow']"), "mimic", joint="mount")
        self.mutate(mimic_cycle)
        self.assert_rejected("mimic.*cycle")

    def test_inertia_validity_and_legitimate_massless_fixed_frames(self):
        self.mutate(lambda root: root.find("link[@name='tip']/inertial/inertia").set("izz", "100"))
        self.assert_rejected("physically valid")
        self.urdf.write_text(ROBOT)
        self.mutate(lambda root: root.find("link[@name='tip']/inertial/inertia").set("ixx", "-1"))
        self.assert_rejected("physically valid")
        self.urdf.write_text(ROBOT)
        self.mutate(lambda root: root.find("link[@name='tip']").remove(root.find("link[@name='tip']/inertial")))
        self.assert_rejected("explicit inertial")
        self.urdf.write_text(ROBOT)
        def add_fixed_frame(root):
            ET.SubElement(root, "link", name="camera_frame")
            joint = ET.SubElement(root, "joint", name="camera_mount", type="fixed")
            ET.SubElement(joint, "parent", link="tip")
            ET.SubElement(joint, "child", link="camera_frame")
        self.mutate(add_fixed_frame)
        report = inspect_bundle(self.source)
        self.assertEqual(report["validation"]["massless_fixed_frames"], ["world", "camera_frame"])

    def test_explicit_zero_mass_fixed_visual_markers_are_preserved(self):
        root = ET.parse(self.urdf).getroot()
        marker = ET.SubElement(root, "link", name="foot_marker")
        geometry = ET.SubElement(ET.SubElement(marker, "visual"), "geometry")
        ET.SubElement(geometry, "sphere", radius="0.001")
        inertial = ET.SubElement(marker, "inertial")
        ET.SubElement(inertial, "mass", value="0")
        ET.SubElement(inertial, "inertia", **dict.fromkeys(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), "0"))
        joint = ET.SubElement(root, "joint", name="marker_mount", type="fixed")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link="foot_marker")
        ET.ElementTree(root).write(self.urdf, encoding="utf-8")
        report = prepare_bundle(self.source, self.output)
        self.assertIn("foot_marker", report["validation"]["massless_fixed_frames"])
        self.assertTrue(any("visual-only zero-mass marker" in message for message in report["validation"]["warnings"]))
        prepared = ET.parse(self.output / "robot.urdf").getroot().find("link[@name='foot_marker']")
        self.assertEqual(prepared.find("inertial/mass").get("value"), "0")
        self.assertEqual(prepared.find("inertial/inertia").get("ixx"), "0")
        for invalid in ("moving", "collision"):
            bad = deepcopy(root)
            if invalid == "moving":
                bad.find("joint[@name='marker_mount']").set("type", "revolute")
            else:
                bad.find("link[@name='foot_marker']").append(deepcopy(prepared.find("visual")))
                bad.find("link[@name='foot_marker']")[2].tag = "collision"
            ET.ElementTree(bad).write(self.urdf, encoding="utf-8")
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "positive mass"):
                inspect_bundle(self.source)

    def test_missing_limits_warn_without_fabricating_values(self):
        self.mutate(lambda root: root.find("joint[@name='elbow']").remove(root.find("joint[@name='elbow']/limit")))
        report = prepare_bundle(self.source, self.output)
        self.assertTrue(any("lacks limits" in warning for warning in report["validation"]["warnings"]))
        self.assertIsNone(ET.parse(self.output / "robot.urdf").getroot().find("joint[@name='elbow']/limit"))

    def test_empty_missing_and_unsupported_assets_rejected(self):
        asset = self.source / "geometry/part.stl"
        asset.write_bytes(b"")
        self.assert_rejected("empty")
        asset.unlink()
        with self.assertRaises(FileNotFoundError):
            prepare_bundle(self.source, self.output)
        asset.with_suffix(".obj").write_text("v 0 0 0")
        self.urdf.write_text(ROBOT.replace("part.stl", "part.obj"))
        self.assert_rejected("STL only")

    def test_existing_nested_and_failure_output_are_safe(self):
        self.output.mkdir()
        (self.output / "keep").write_text("original")
        with self.assertRaisesRegex(ValueError, "already exist"):
            prepare_bundle(self.source, self.output)
        self.assertEqual((self.output / "keep").read_text(), "original")

    def test_atomic_copy_failure_and_nested_destination(self):
        with self.assertRaisesRegex(ValueError, "non-nested"):
            prepare_bundle(self.source, self.source / "nested")
        with patch("acdc4robot_onshape.bundle.shutil.copyfile", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                prepare_bundle(self.source, self.output)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.folder.glob(".prepared.stage-*")), [])

    def test_repeatable_provenance_and_decimal_precision(self):
        first = prepare_bundle(self.source, self.output)
        second = prepare_bundle(self.source, self.folder / "again")
        self.assertEqual(first, second)
        self.assertEqual(decimal_number("1.234e-61"), "0." + "0" * 60 + "1234")
        self.assertEqual(decimal_number("-0.0"), "0")
        with self.assertRaisesRegex(ValueError, "4096"):
            decimal_number("1e-999999")

    def test_cli_validate_prepare_and_errors(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(["validate", str(self.source)]), 0)
        self.assertTrue(json.loads(output.getvalue())["passed"])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(["prepare", str(self.source), str(self.output), "--configuration", '{"size":"small"}']), 0)
        self.assertTrue((self.output / "robot.urdf").exists())
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["validate", str(self.source), "--package", "broken"])
        self.assertEqual(error.exception.code, 1)

    def test_xml_entities_and_selector_escape_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unsafe"):
            inspect_bundle(self.source, urdf="../outside.urdf")
        self.urdf.write_text('<!DOCTYPE robot [<!ENTITY x "test">]><robot name="a"><link name="frame"/></robot>')
        self.assert_rejected("DTDs")


if __name__ == "__main__":
    unittest.main()

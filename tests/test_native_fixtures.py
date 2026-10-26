"""Regression coverage from actual Onshape 1.222 exports."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from xml.etree import ElementTree as ET
from acdc4robot_onshape.bundle import input_directory, prepare_bundle

class NativeFixtureTests(unittest.TestCase):
    def prepare(self, fixture, output):
        archive = Path(__file__).parent / 'fixtures' / fixture
        with input_directory(archive) as (directory, _):
            return prepare_bundle(directory, output, package_roots={'assembly_1': directory / 'assembly_1'})

    def test_one_part_native_archive(self):
        with TemporaryDirectory() as temporary:
            output = Path(temporary) / 'prepared'
            self.prepare('native-one-part-onshape-1.222.zip', output)
            robot = ET.parse(output / 'robot.urdf').getroot()
            self.assertEqual(len(robot.findall('link')), 1)
            self.assertAlmostEqual(float(robot.find('./link/inertial/mass').get('value')), .0539006)
            self.assertTrue((output / 'provenance/source-urdf.xml').is_file())

    def test_repeated_instances_and_limited_revolute_survive(self):
        with TemporaryDirectory() as temporary:
            output = Path(temporary) / 'prepared'
            self.prepare('native-two-link-onshape-1.222.zip', output)
            robot = ET.parse(output / 'robot.urdf').getroot()
            self.assertEqual(len(robot.findall('link')), 5)
            self.assertEqual(len(robot.findall('joint')), 4)
            meshes = list(robot.iter('mesh'))
            self.assertEqual(len(meshes), 2)
            self.assertEqual(meshes[0].get('filename'), meshes[1].get('filename'))
            joint = robot.find("./joint[@name='revolute_1']")
            self.assertEqual(joint.get('type'), 'revolute')
            self.assertAlmostEqual(float(joint.find('limit').get('lower')), -.785398)
            self.assertAlmostEqual(float(joint.find('limit').get('upper')), 1.0472)
            self.assertEqual(joint.find('axis').get('xyz'), '0 0 1')
            self.assertIsNone(robot.find("./link[@name='root']/inertial"))
            child = robot.find("./link[@name='part_1_1']")
            self.assertAlmostEqual(float(child.find('./visual/origin').get('xyz').split()[0]), .08)
            self.assertAlmostEqual(float(child.find('./inertial/origin').get('xyz').split()[0]), .115829)

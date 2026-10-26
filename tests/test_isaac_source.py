"""Reference-frame exemptions must never hide invalid physical links."""
from pathlib import Path
import tempfile
import unittest
from acdc4robot_onshape.shared.smoke_test_isaac_urdf import source_data

INERTIAL = '<inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>'

class IsaacSourceTests(unittest.TestCase):
    def check_source(self, xml, **options):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'robot.urdf'
            path.write_text(xml)
            return source_data(path, **options)

    def test_massless_root_is_explicit_opt_in(self):
        xml = f'<robot name="r"><link name="root"/><link name="body">{INERTIAL}</link><joint name="j" type="fixed"><parent link="root"/><child link="body"/></joint></robot>'
        with self.assertRaisesRegex(ValueError, 'explicit inertial'):
            self.check_source(xml)
        links, joints, _, _ = self.check_source(xml, allow_massless_fixed_frames=True)
        self.assertEqual(set(links), {'body'})
        self.assertEqual(joints, ['j'])

    def test_moving_massless_link_is_rejected(self):
        xml = f'<robot name="r"><link name="root">{INERTIAL}</link><link name="moving"/><joint name="j" type="continuous"><parent link="root"/><child link="moving"/><axis xyz="0 0 1"/></joint></robot>'
        with self.assertRaisesRegex(ValueError, 'explicit inertial'):
            self.check_source(xml, allow_massless_fixed_frames=True)

    def test_duplicate_massless_names_are_rejected(self):
        with self.assertRaisesRegex(ValueError, '[Dd]uplicate'):
            self.check_source('<robot name="r"><link name="root"/><link name="root"/></robot>', allow_massless_fixed_frames=True)

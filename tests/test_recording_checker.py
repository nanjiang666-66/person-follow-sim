import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('checker', ROOT / 'scripts/check_recording.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def recording():
    rows = []
    for i in range(701):
        t = i * 0.05
        row = dict.fromkeys(checker.NUMERIC, 0.0)
        row.update(wall_s=t, ros_s=t+1, target_x=2.5, distance_m=1.52,
                   visible=1, cmd_age_s=0.01, cmd_v=0.05 if t < 5 else 0.0,
                   state='following' if t < 5 else 'hold_distance')
        rows.append(row)
    return rows


class RecordingCheckerTests(unittest.TestCase):
    def test_stationary_pass(self):
        self.assertFalse(checker.evaluate(recording(), 'stationary')[0])

    def test_stationary_frozen_robot_fails(self):
        rows = recording()
        for row in rows:
            row.update(cmd_v=0, distance_m=2.5)
        self.assertTrue(checker.evaluate(rows, 'stationary')[0])

    def test_missing_command_fails(self):
        rows = recording()
        rows[-1]['cmd_age_s'] = 1
        self.assertTrue(checker.evaluate(rows)[0])

    def test_invalid_input_moving_fails(self):
        rows = recording()
        for row in rows[:100]:
            row['visible'] = 0
        self.assertTrue(checker.evaluate(rows)[0])

    def test_command_limits_fail(self):
        for field, value in [('cmd_v', -0.1), ('cmd_v', 0.3), ('cmd_w', 0.7)]:
            rows = recording()
            rows[10][field] = value
            self.assertTrue(checker.evaluate(rows)[0])

    def test_missing_occlusions_fail(self):
        self.assertTrue(checker.evaluate(recording(), 'occlusion')[0])

    def test_invalid_csv_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.csv'
            path.write_text('wall_s,state\n1,following\n', encoding='utf-8')
            with self.assertRaises(ValueError):
                checker.load_rows(path)


if __name__ == '__main__':
    unittest.main()

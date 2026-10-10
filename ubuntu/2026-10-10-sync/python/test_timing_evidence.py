"""Timestamp semantics and concurrent manifest writes, without hardware."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from hipexo_session import SessionManifest, camera_frame_timing


class TimingTests(unittest.TestCase):
    def test_native_camera_clock_domain_is_preserved(self):
        class Frame:
            def get_frame_number(self):return 18
            def get_timestamp(self):return 132.5
            def get_frame_timestamp_domain(self):return 'hardware_clock'
        timing=camera_frame_timing(Frame(),1000000,500000,'stream1')
        self.assertEqual(timing['device_timestamp_ms'],132.5)
        self.assertEqual(timing['device_timestamp_domain'],'hardware_clock')
        self.assertEqual(timing['host_frame_received_wall_ns'],1000000)
        missing=camera_frame_timing(object(),1,2,'stream2')
        self.assertIsNone(missing['device_timestamp_ms'])

    def test_shared_directory_has_distinct_session_ids_and_complete_events(self):
        with tempfile.TemporaryDirectory() as d:
            first=SessionManifest(d,subject_id='fake')
            def write(i):
                for j in range(20):first.artifact('test',Path(d)/f'{i}-{j}.bin')
            threads=[threading.Thread(target=write,args=(i,)) for i in range(4)]
            for t in threads:t.start()
            for t in threads:t.join()
            second=SessionManifest(d)
            events=[json.loads(s) for s in first.path.read_text().splitlines()]
            self.assertEqual(len(events),82)
            self.assertEqual(len({e['path'] for e in events if e['kind']=='artifact'}),80)
            self.assertNotEqual(first.session_id,second.session_id)
            self.assertEqual(events[-1]['session_id'],second.session_id)


if __name__=='__main__':unittest.main()

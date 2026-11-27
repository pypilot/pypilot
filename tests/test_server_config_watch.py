"""External config edits should reach the server across file replacements."""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import server

try:
    import inotify.adapters
except ImportError:
    inotify = None


@unittest.skipUnless(inotify, "requires Linux inotify")
class ConfigWatchTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / server.configfilename
        self.path.write_text("rudder.range=15\n")
        config_dir = patch.object(server, "configfilepath", directory.name + "/")
        config_dir.start()
        self.addCleanup(config_dir.stop)
        self.values = server.ServerValues(None)
        self.values.poll_config(self.values.inotify_time + 20)
        self.assertIsNotNone(self.values.inotify)

    def poll(self):
        self.values.poll_config(self.values.inotify_time + 3)

    def test_modify_close_and_replace(self):
        value = self.values.values["rudder.range"]
        with self.path.open("w") as editor:
            editor.write("rudder.range=1")
            editor.flush()
            self.poll()
            self.assertEqual(value.msg, "rudder.range=15\n")
            editor.write("6\n")
            editor.flush()
            self.poll()  # IN_MODIFY, with the writer still open
            self.assertEqual(value.msg, "rudder.range=16\n")
        with self.path.open("w") as editor:
            editor.write("rudder.range=17")
        self.poll()
        self.assertEqual(value.msg, "rudder.range=17\n")
        replacement = self.path.with_suffix(".tmp")
        replacement.write_text("rudder.range=18\n")
        os.replace(replacement, self.path)
        self.poll()
        self.assertEqual(value.msg, "rudder.range=18\n")

    def test_server_save_and_rearm(self):
        with patch.object(self.values, "load_file", wraps=self.values.load_file) as reload:
            self.path.with_name("pypilot.conf~").write_text("rudder.range=99\n")
            self.values.store_file(str(self.path))
            self.values.store_file(str(self.path) + ".bak")
            self.poll()
            reload.assert_not_called()
            with patch.object(os, "replace", side_effect=OSError("save failed")):
                with self.assertRaises(OSError):
                    self.values.store_file(str(self.path))
            self.path.write_text("rudder.range=16\n")
            self.poll()
            reload.assert_called_once()
            self.assertEqual(self.values.values["rudder.range"].msg, "rudder.range=16\n")

    def test_in_place_edit_between_poll_and_save(self):
        original = self.path.stat()
        self.path.write_text("rudder.range=16\n")
        os.utime(self.path, ns=(original.st_atime_ns, original.st_mtime_ns + 1_000_000))
        self.values.need_store = True
        self.values.store()
        self.assertEqual(self.path.read_text(), "rudder.range=16\n")
        self.assertEqual(self.values.values["rudder.range"].msg, self.path.read_text())
        self.assertTrue(self.values.need_store)
        self.assertEqual(list(self.path.parent.glob(".pypilot_conf.*")), [])

        self.values.store()
        self.assertFalse(self.values.need_store)
        self.assertIn("rudder.range=16\n", self.path.read_text())

    def test_replacement_after_tempfile_sync_with_same_timestamp(self):
        original = self.path.stat()
        replacement = self.path.with_suffix(".editor")
        replacement.write_text("rudder.range=16\n")
        os.utime(replacement, ns=(original.st_atime_ns, original.st_mtime_ns))
        remove_watch = self.values.inotify.remove_watch

        def edit_while_watch_is_paused(path):
            remove_watch(path)
            os.replace(replacement, self.path)

        with patch.object(self.values.inotify, "remove_watch", edit_while_watch_is_paused):
            self.assertFalse(self.values.store_file(str(self.path)))
        self.assertEqual(self.path.read_text(), "rudder.range=16\n")
        self.assertEqual(self.values.values["rudder.range"].msg, self.path.read_text())
        self.path.write_text("rudder.range=17\n")
        self.poll()
        self.assertEqual(self.values.values["rudder.range"].msg, self.path.read_text())

    def test_recreate_missing_primary_without_overwriting_new_file(self):
        self.path.unlink()
        self.assertFalse(self.values.store_file(str(self.path)))
        self.assertFalse(self.path.exists())
        self.path.write_text("rudder.range=19\n")
        self.assertFalse(self.values.store_file(str(self.path)))
        self.assertEqual(self.path.read_text(), "rudder.range=19\n")

    def test_save_while_editor_has_a_partial_record(self):
        value = self.values.values["rudder.range"]
        with self.path.open("w") as editor:
            editor.write("rudder.range=1")
            editor.flush()
            self.assertFalse(self.values.store_file(str(self.path)))
            self.assertEqual(value.msg, "rudder.range=15\n")
            editor.write("6")
        self.poll()  # Closing the file completes the record even without a final newline.
        self.assertEqual(value.msg, "rudder.range=16\n")

    def test_missed_close_without_final_newline(self):
        self.path.write_text("rudder.range=17")
        self.assertFalse(self.values.store_file(str(self.path)))
        self.assertFalse(self.values.store_file(str(self.path)))
        self.assertEqual(self.values.values["rudder.range"].msg, "rudder.range=17\n")
        self.assertTrue(self.values.store_file(str(self.path)))

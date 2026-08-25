import argparse
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hambot_link import camera_stream


def read_frames(raw: bytes):
    """Decode the wire format the way the console reader will."""
    stream = io.BytesIO(raw)
    frames = []
    while True:
        header = stream.readline()
        if not header:
            return frames
        magic, count = header.split()
        assert magic == camera_stream.FRAME_MAGIC
        frames.append(stream.read(int(count)))


class ParseResolutionTests(unittest.TestCase):
    def test_parses_a_size(self):
        self.assertEqual((640, 480), camera_stream.parse_resolution("640x480"))
        self.assertEqual((1280, 720), camera_stream.parse_resolution(" 1280X720 "))

    def test_rejects_junk(self):
        for text in ("640", "640*480", "x480", "640x", "-1x480", "abc"):
            with self.subTest(text=text):
                with self.assertRaises(argparse.ArgumentTypeError):
                    camera_stream.parse_resolution(text)

    def test_rejects_a_zero_dimension(self):
        with self.assertRaises(argparse.ArgumentTypeError):
            camera_stream.parse_resolution("0x480")


class FrameWriterTests(unittest.TestCase):
    def test_frames_are_length_prefixed_and_recoverable(self):
        sink = io.BytesIO()
        writer = camera_stream.FrameWriter(sink)
        writer.write(b"\xff\xd8first")
        writer.write(b"\xff\xd8second-frame")
        self.assertEqual([b"\xff\xd8first", b"\xff\xd8second-frame"],
                         read_frames(sink.getvalue()))
        self.assertEqual(2, writer.frames)

    def test_binary_payloads_survive_newlines_in_the_data(self):
        # A JPEG can contain 0x0A; the length prefix is what makes that safe.
        payload = b"\xff\xd8\n\n\nJPEG 99\n\xff\xd9"
        sink = io.BytesIO()
        writer = camera_stream.FrameWriter(sink)
        writer.write(payload)
        self.assertEqual([payload], read_frames(sink.getvalue()))

    def test_a_broken_pipe_ends_the_session_instead_of_raising(self):
        class BrokenSink:
            def write(self, _data):
                raise BrokenPipeError("viewer went away")

            def flush(self):
                pass

        writer = camera_stream.FrameWriter(BrokenSink())
        writer.write(b"frame")
        self.assertTrue(writer.closed)
        self.assertEqual(0, writer.frames)

    def test_writes_after_close_are_dropped(self):
        sink = io.BytesIO()
        writer = camera_stream.FrameWriter(sink)
        writer.close()
        writer.write(b"frame")
        self.assertEqual(b"", sink.getvalue())
        self.assertEqual(0, writer.frames)


class HangupTests(unittest.TestCase):
    def test_stdin_eof_closes_the_session(self):
        writer = camera_stream.FrameWriter(io.BytesIO())
        camera_stream.watch_for_hangup(io.BytesIO(b""), writer)
        self.assertTrue(writer.closed)

    def test_a_stdin_error_also_closes_the_session(self):
        class Exploding(io.BytesIO):
            def readline(self):
                raise OSError("connection reset")

        writer = camera_stream.FrameWriter(io.BytesIO())
        camera_stream.watch_for_hangup(Exploding(), writer)
        self.assertTrue(writer.closed)


if __name__ == "__main__":
    unittest.main()


class HangupThreadTests(unittest.TestCase):
    """The hangup watcher must not still hold stdin at interpreter shutdown."""

    def test_it_returns_when_the_session_ends_without_stdin_eof(self):
        import os
        import threading
        import time

        read_fd, write_fd = os.pipe()          # stays open: no EOF, like real ssh
        self.addCleanup(os.close, write_fd)
        stream = os.fdopen(read_fd, "rb")
        self.addCleanup(stream.close)

        writer = camera_stream.FrameWriter(io.BytesIO())
        thread = threading.Thread(
            target=camera_stream.watch_for_hangup,
            args=(stream, writer, 0.01), daemon=True)
        thread.start()
        time.sleep(0.05)
        self.assertTrue(thread.is_alive(), "should still be watching")

        writer.close()                          # viewer's pipe broke instead
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(),
                         "must exit when the session ends, or shutdown aborts")

    def test_stdin_eof_still_closes_the_session(self):
        import os
        import threading

        read_fd, write_fd = os.pipe()
        stream = os.fdopen(read_fd, "rb")
        self.addCleanup(stream.close)
        writer = camera_stream.FrameWriter(io.BytesIO())
        thread = threading.Thread(
            target=camera_stream.watch_for_hangup,
            args=(stream, writer, 0.01), daemon=True)
        thread.start()
        os.close(write_fd)                      # console hangs up
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(writer.closed)

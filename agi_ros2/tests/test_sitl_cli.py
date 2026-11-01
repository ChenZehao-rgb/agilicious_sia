#!/usr/bin/env python3
"""CLI startup handshake without opening a device or changing its configuration."""
from pathlib import Path
import socket
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'betaflight_sitl'))
import run


class CliSocket:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.sent = []
        self.now = 0.

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, size):
        self.now += .25
        reply = next(self.replies, None)
        if reply is None:
            raise socket.timeout()
        return reply


class SitlCliTests(unittest.TestCase):
    def test_entry_waits_for_complete_prompt_without_resending(self):
        connection = CliSocket([None, None, b'Entering CLI Mode\r\n', b'\r\n# '])
        with patch.object(run.time, 'monotonic', side_effect=lambda: connection.now):
            run.enter_betaflight_cli(connection)
        self.assertEqual(connection.sent, [b'#'])

    def test_partial_response_does_not_inject_more_entry_characters(self):
        connection = CliSocket([b'Entering CLI Mode', None, None, b'\r\n# '])
        with patch.object(run.time, 'monotonic', side_effect=lambda: connection.now):
            run.enter_betaflight_cli(connection)
        self.assertEqual(connection.sent, [b'#'])

    def test_configuration_commands_are_never_retried(self):
        connection = CliSocket([])
        with patch.object(run.time, 'monotonic', side_effect=lambda: connection.now):
            with self.assertRaisesRegex(RuntimeError, 'did not finish'):
                run.run_cli_command(connection, 'save')
        self.assertEqual(connection.sent, [b'save\n'])

    def test_disconnect_and_rejected_commands_remain_failures(self):
        for reply in (b'', b'###ERROR rejected\r\n# '):
            with self.subTest(reply=reply), self.assertRaises(RuntimeError):
                run.enter_betaflight_cli(CliSocket([reply]))


if __name__ == '__main__':
    unittest.main()

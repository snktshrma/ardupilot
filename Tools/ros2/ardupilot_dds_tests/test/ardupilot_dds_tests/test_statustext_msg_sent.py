# Copyright 2026 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

# flake8: noqa

"""
Bring up ArduPilot SITL and check text sent by a ROS 2 node reaches the GCS.

The text is published to /ap/statustext, and is expected to show up in the
MAVProxy output once ArduPilot has sent it on as a MAVLink STATUSTEXT.

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k statustext

"""

import pytest
import rclpy
import rclpy.node
import threading
import time

from launch_pytest.tools import process as process_tools
from ros_helpers import ros_node

from ardupilot_msgs.msg import StatusText

from launch_fixtures import launch_sitl_copter_dds_udp

TOPIC = "/ap/statustext"
TEXT = "hello from ROS 2"
# ArduPilot prefixes the text it forwards, and MAVProxy prefixes what it prints
PREFIX = "AP: DDS: "
EXPECTED_OUTPUT = f"{PREFIX}{TEXT}"
SEND_PERIOD = 1.0
WAIT_FOR_START_TIMEOUT = 5.0
WAIT_FOR_TEXT_TIMEOUT = 20.0
SEND_JOIN_TIMEOUT = 5.0

# Long enough for a few sends of text that is expected to produce no output.
DROPPED_SEND_TIME = 3.0

# Severities outside MAV_SEVERITY are reported as INFO rather than dropped.
OUT_OF_RANGE_SEVERITY = 200
OUT_OF_RANGE_TEXT = "severity is out of range"

# Text that does not fit the 255 byte DDS buffer is dropped whole.
OVERLONG_TEXT = "Y" * 300
# Enough of a run to be certain none of the dropped text was printed.
OVERLONG_SAMPLE = "Y" * 64

# The GCS send buffer holds 256 characters including the "DDS: " prefix, so
# this much text arrives truncated rather than dropped.
LONGEST_TEXT = "X" * 254
TRUNCATED_LENGTH = 251

# Sent after text that is expected to be dropped, to show the output caught up.
MARKER_TEXT = "marker"


class StatustextSender(rclpy.node.Node):
    """Send text for the GCS to display."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("statustext_sender")
        self.stop_event = threading.Event()
        self.send_thread = None
        self.msg_lock = threading.Lock()
        self.msg = StatusText()

        self.publisher = self.create_publisher(StatusText, TOPIC, 1)

    def set_message(self, severity, text):
        """Choose the text sent from now on."""
        msg = StatusText()
        msg.severity = severity
        msg.text = text
        with self.msg_lock:
            self.msg = msg

    def send_statustext(self):
        """Publish the text once."""
        with self.msg_lock:
            msg = self.msg
        self.publisher.publish(msg)

    def process_send(self):
        """Keep sending, as the first messages may go out before discovery completes."""
        while not self.stop_event.wait(SEND_PERIOD):
            self.send_statustext()

    def start_sender(self):
        """Start sending in the background."""
        self.send_thread = threading.Thread(target=self.process_send)
        self.send_thread.start()

    def stop_sender(self):
        """Stop sending, so that nothing publishes on a node being destroyed."""
        self.stop_event.set()
        if self.send_thread is not None:
            self.send_thread.join(timeout=SEND_JOIN_TIMEOUT)


def wait_for_text(launch_context, mavproxy, expected, timeout=WAIT_FOR_TEXT_TIMEOUT):
    """
    Wait for expected in the MAVProxy output.

    Returns whether it was seen, along with the output that arrived while
    waiting, so that a test may also assert on what did *not* show up.
    """
    seen = []

    def condition(output):
        seen.append(output)
        return expected in output

    found = process_tools.wait_for_output_sync(launch_context, mavproxy, condition, timeout=timeout)
    return found, "".join(seen)


def wait_for_processes(launch_context, actions):
    """Wait for the launched processes to start."""
    for name in ("micro_ros_agent", "mavproxy", "sitl"):
        process_tools.wait_for_start_sync(launch_context, actions[name].action, timeout=WAIT_FOR_START_TIMEOUT)


def send_dropped_then_marker(launch_context, mavproxy, node, severity, text):
    """
    Send text that is expected to be dropped, then a marker that is not.

    Returns the output up to and including the marker, which is where the
    dropped text would have appeared had ArduPilot forwarded it.
    """
    node.set_message(severity, text)
    time.sleep(DROPPED_SEND_TIME)
    node.set_message(StatusText.INFO, MARKER_TEXT)
    found, output = wait_for_text(launch_context, mavproxy, f"{PREFIX}{MARKER_TEXT}")
    assert found, "Did not see the marker in the MAVProxy output."
    return output


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_statustext_msg_sent(launch_context, launch_sitl_copter_dds_udp):
    """Test text published by a ROS 2 node is sent on to the GCS."""
    _, actions = launch_sitl_copter_dds_udp
    mavproxy = actions["mavproxy"].action

    wait_for_processes(launch_context, actions)

    with ros_node(StatustextSender) as node:
        node.set_message(StatusText.WARNING, TEXT)
        node.start_sender()
        try:
            text_displayed, _ = wait_for_text(launch_context, mavproxy, EXPECTED_OUTPUT)
        finally:
            node.stop_sender()
        assert text_displayed, f"Did not see '{EXPECTED_OUTPUT}' in the MAVProxy output."
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_statustext_limits(launch_context, launch_sitl_copter_dds_udp):
    """Test how the severity and the length of the text are handled."""
    _, actions = launch_sitl_copter_dds_udp
    mavproxy = actions["mavproxy"].action

    wait_for_processes(launch_context, actions)

    with ros_node(StatustextSender) as node:
        node.set_message(OUT_OF_RANGE_SEVERITY, OUT_OF_RANGE_TEXT)
        node.start_sender()
        try:
            expected = f"{PREFIX}{OUT_OF_RANGE_TEXT}"
            found, _ = wait_for_text(launch_context, mavproxy, expected)
            assert found, f"Did not see '{expected}' in the MAVProxy output."

            output = send_dropped_then_marker(launch_context, mavproxy, node, StatusText.INFO, "")
            empty = [line for line in output.splitlines() if line.strip() == PREFIX.strip()]
            assert not empty, f"Empty text was displayed: {empty}"

            output = send_dropped_then_marker(launch_context, mavproxy, node, StatusText.INFO, OVERLONG_TEXT)
            assert OVERLONG_SAMPLE not in output, "Text too long for the DDS buffer was displayed."

            node.set_message(StatusText.INFO, LONGEST_TEXT)
            expected = "X" * TRUNCATED_LENGTH
            found, output = wait_for_text(launch_context, mavproxy, expected)
            assert found, f"Did not see {TRUNCATED_LENGTH} characters of the longest text."
            assert "X" * (TRUNCATED_LENGTH + 1) not in output, "More text arrived than the GCS buffer holds."
        finally:
            node.stop_sender()
    yield

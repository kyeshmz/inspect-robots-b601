"""Default collection is offline; real CAN sockets are forbidden in every test."""

import socket
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest


def pytest_addoption(parser):
    parser.addoption("--ros", action="store_true", help="Collect isolated ROS protocol tests")


def pytest_ignore_collect(collection_path, config):
    if collection_path.name == "packaging":
        return True
    if collection_path.name == "ros" and not config.getoption("--ros"):
        return True
    return None


@pytest.fixture(autouse=True)
def forbid_real_can(monkeypatch):
    original = socket.socket

    def guarded_socket(family=socket.AF_INET, *args, **kwargs):
        if family == socket.AF_CAN:
            raise AssertionError("Tests must inject a fake transport, never open real CAN")
        return original(family, *args, **kwargs)

    monkeypatch.setattr(socket, "socket", guarded_socket)


class FakeRosSession:
    def __init__(self, config):
        self.positions = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        self.frozen = False
        self.closed = False

    def observe(self):
        return self.positions.copy(), time.monotonic(), {}, {}

    def move(self, target, duration):
        if not self.frozen:
            self.positions = target.copy()

    def close(self):
        self.closed = True


@pytest.fixture
def fake_ros(monkeypatch):
    from inspect_robots_b601 import B601Embodiment

    monkeypatch.setitem(
        sys.modules, "inspect_robots_b601._ros", SimpleNamespace(RosSession=FakeRosSession)
    )
    monkeypatch.setattr("inspect_robots_b601.embodiment.time.sleep", lambda _: None)
    robot = B601Embodiment()
    try:
        yield robot
    finally:
        robot.close()

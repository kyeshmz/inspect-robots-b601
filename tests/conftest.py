"""Default collection is offline; real CAN sockets are forbidden in every test."""

import socket

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

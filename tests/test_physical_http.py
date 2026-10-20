import base64
import json
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx
import pytest

from inspect_robots_b601 import _physical_http


def test_http_child_reads_bounded_body_without_inherited_environment(monkeypatch, capsys):
    import io

    request = {
        "url": "https://physical.invalid/v1/responses",
        "headers": {"authorization": "Bearer fake"},
        "body": base64.b64encode(b"{}").decode(),
    }
    monkeypatch.setattr(sys, "stdin", io.TextIOWrapper(io.BytesIO(json.dumps(request).encode())))
    real_client = httpx.Client

    def client(**kwargs):
        assert kwargs["trust_env"] is False
        assert kwargs["follow_redirects"] is False
        assert kwargs["timeout"] == 4.0
        kwargs["transport"].close()
        kwargs["transport"] = httpx.MockTransport(
            lambda req: httpx.Response(200, content=b'{"output":[]}')
        )
        return real_client(**kwargs)

    monkeypatch.setattr(_physical_http.httpx, "Client", client)
    _physical_http.main()
    result = json.loads(capsys.readouterr().out)
    assert result["status"] == 200
    assert base64.b64decode(result["body"]) == b'{"output":[]}'


def test_oversized_body_is_rejected(monkeypatch, capsys):
    import io

    monkeypatch.setattr(
        sys,
        "stdin",
        io.TextIOWrapper(
            io.BytesIO(
                json.dumps(
                    {"url": "https://physical.invalid", "headers": {}, "body": "e30="}
                ).encode()
            )
        ),
    )
    real_client = httpx.Client

    def client(**kwargs):
        kwargs["transport"].close()
        kwargs["transport"] = httpx.MockTransport(
            lambda req: httpx.Response(200, content=b"x" * (1024 * 1024 + 1))
        )
        return real_client(**kwargs)

    monkeypatch.setattr(_physical_http.httpx, "Client", client)
    _physical_http.main()
    assert json.loads(capsys.readouterr().out)["failures"] == [
        {"operation": "request", "type": "ValueError"}
    ]


def test_child_preserves_request_and_both_close_failures_without_secrets(monkeypatch, capsys):
    import io

    class Response:
        status_code = 200

        def iter_bytes(self):
            raise ValueError("secret-must-not-be-logged")

        def close(self):
            raise OSError("secret-must-not-be-logged")

    class Client:
        def build_request(self, *a, **kw):
            return object()

        def send(self, request, **kw):
            return Response()

        def close(self):
            raise RuntimeError("secret-must-not-be-logged")

    monkeypatch.setattr(
        sys,
        "stdin",
        io.TextIOWrapper(
            io.BytesIO(
                json.dumps(
                    {"url": "https://physical.invalid", "headers": {}, "body": "e30="}
                ).encode()
            )
        ),
    )
    monkeypatch.setattr(_physical_http.httpx, "Client", lambda **kw: Client())
    _physical_http.main()
    output = capsys.readouterr().out
    assert "secret-must-not-be-logged" not in output
    assert json.loads(output)["failures"] == [
        {"operation": "request", "type": "ValueError"},
        {"operation": "response.close", "type": "OSError"},
        {"operation": "client.close", "type": "RuntimeError"},
    ]


def test_child_withholds_http_failure_body(monkeypatch, capsys):
    import io

    class Response:
        status_code = 401

        def iter_bytes(self):
            pytest.fail("An HTTP failure body must not be consumed or forwarded")

        def close(self):
            pass

    class Client:
        def build_request(self, *args, **kwargs):
            return object()

        def send(self, *args, **kwargs):
            return Response()

        def close(self):
            pass

    monkeypatch.setattr(
        sys,
        "stdin",
        io.TextIOWrapper(
            io.BytesIO(
                json.dumps(
                    {"url": "https://physical.invalid", "headers": {}, "body": "e30="}
                ).encode()
            )
        ),
    )
    monkeypatch.setattr(_physical_http.httpx, "Client", lambda **kwargs: Client())
    _physical_http.main()
    assert json.loads(capsys.readouterr().out) == {"status": 401, "failures": []}


@pytest.mark.parametrize("stall", [False, True])
def test_real_provider_child_is_joined_on_total_lifetime_deadline(monkeypatch, stall):
    arrived, finish = threading.Event(), threading.Event()

    class Provider(BaseHTTPRequestHandler):
        def do_POST(self):
            arrived.set()
            self.rfile.read(int(self.headers["Content-Length"]))
            if stall:
                self.send_response(200)
                self.send_header("Content-Length", "50")
                self.end_headers()
                try:
                    for _ in range(100):
                        self.wfile.write(b" ")
                        self.wfile.flush()
                        if finish.wait(0.03):
                            break
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_response(200)
                self.send_header("Content-Length", "13")
                self.end_headers()
                self.wfile.write(b'{"output":[]}')

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Provider)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    monkeypatch.setattr(_physical_http, "HTTP_SECONDS", 1.0)
    replies = []
    transport = _physical_http._BoundedTransport(lambda: None, replies.append, max_requests=3)
    try:
        request = httpx.Request(
            "POST", f"http://127.0.0.1:{server.server_port}/responses", content=b"{}"
        )
        if stall:
            with pytest.raises(subprocess.TimeoutExpired):
                transport.handle_request(request)
            assert replies == []
        else:
            assert transport.handle_request(request).status_code == 200
            assert replies == [b'{"output":[]}']
        assert arrived.is_set()
        assert transport.requests == 1
    finally:
        finish.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        transport.close()
    assert not thread.is_alive()


def test_request_budgets_and_child_failure_are_terminal(monkeypatch):
    from inspect_robots.errors import SafetyAbort

    transport = _physical_http._BoundedTransport(lambda: None, lambda body: None, max_requests=3)
    monkeypatch.setattr(
        _physical_http.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a[0], 1, b"", b"fake"),
    )
    request = httpx.Request("POST", "https://physical.invalid", content=b"{}")
    for _ in range(3):
        with pytest.raises(SafetyAbort, match="no retry"):
            transport.handle_request(request)
    with pytest.raises(SafetyAbort, match="budget"):
        transport.handle_request(request)
    assert transport.requests == 3
    transport.close()
    with pytest.raises(SafetyAbort):
        transport.handle_request(request)
    oversized = _physical_http._BoundedTransport(lambda: None, lambda body: None, max_requests=3)
    with pytest.raises(SafetyAbort, match="body budget"):
        oversized.handle_request(
            httpx.Request("POST", "https://physical.invalid", content=b"x" * (1024 * 1024 + 1))
        )
    assert oversized.requests == 0

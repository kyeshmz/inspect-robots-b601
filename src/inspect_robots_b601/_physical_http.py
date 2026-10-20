from __future__ import annotations

import base64
import json
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path

import httpx
from inspect_robots.errors import SafetyAbort

MAX_BODY = 1024 * 1024
HTTP_SECONDS = 5.0


class _BoundedTransport(httpx.BaseTransport):
    """The child owns only HTTP. Timeout kills and joins it before evaluation resumes."""

    def __init__(
        self,
        gate: Callable[[], None],
        check_reply: Callable[[bytes], None],
        *,
        max_requests: int,
    ) -> None:
        self._gate, self._check_reply = gate, check_reply
        self._max_requests = max_requests
        self.requests = 0
        self.closed = False

    def handle_request(self, request: httpx.Request) -> httpx.Response:
        self._gate()
        if self.closed or self.requests >= self._max_requests:
            raise SafetyAbort("Physical provider request budget exhausted or transport closed")
        body = request.read()
        if len(body) > MAX_BODY:
            raise SafetyAbort("Physical provider request body budget exhausted")
        self.requests += 1
        started = time.monotonic()
        completed = subprocess.run(
            [sys.executable, "-I", "-m", "inspect_robots_b601._physical_http"],
            input=json.dumps(
                {
                    "url": str(request.url),
                    "headers": dict(request.headers),
                    "body": base64.b64encode(body).decode("ascii"),
                }
            ).encode(),
            capture_output=True,
            timeout=HTTP_SECONDS,
            check=False,
            env={"PATH": str(Path(sys.executable).parent)},
        )
        self._gate()
        if time.monotonic() - started > HTTP_SECONDS:
            raise SafetyAbort("Late provider response rejected")
        if completed.returncode:
            raise SafetyAbort("Isolated provider request failed; no retry or recovery")
        result = json.loads(completed.stdout)
        if result.get("failures"):
            raise SafetyAbort("Isolated provider failures: " + json.dumps(result["failures"]))
        if result["status"] != 200:
            raise SafetyAbort(f"Physical provider HTTP {result['status']}; no retry or recovery")
        response_body = base64.b64decode(result["body"], validate=True)
        if json.loads(response_body).get("status") == "failed":
            raise SafetyAbort("Physical provider Responses status failed; no retry or recovery")
        self._check_reply(response_body)
        return httpx.Response(200, content=response_body)

    def close(self) -> None:
        self.closed = True


def main() -> None:
    request = json.loads(sys.stdin.buffer.read(MAX_BODY + 1))
    client = httpx.Client(
        timeout=4.0,
        trust_env=False,
        follow_redirects=False,
        transport=httpx.HTTPTransport(retries=0),
    )
    response = None
    failures: list[dict[str, str]] = []
    result: dict[str, object] = {"failures": failures}
    try:
        response = client.send(
            client.build_request(
                "POST",
                request["url"],
                headers=request["headers"],
                content=base64.b64decode(request["body"], validate=True),
            ),
            stream=True,
        )
        result["status"] = response.status_code
        if response.status_code == 200:
            body = bytearray()
            for chunk in response.iter_bytes():
                body.extend(chunk)
                if len(body) > MAX_BODY:
                    raise ValueError("Provider response exceeds the physical trial body budget")
            result["body"] = base64.b64encode(body).decode("ascii")
    except BaseException as error:
        failures.append({"operation": "request", "type": type(error).__name__})
    finally:
        for operation, close in (
            ("response.close", None if response is None else response.close),
            ("client.close", client.close),
        ):
            if close is not None:
                try:
                    close()
                except BaseException as error:
                    failures.append({"operation": operation, "type": type(error).__name__})
    print(json.dumps(result))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(type(error).__name__, file=sys.stderr)
        raise SystemExit(1) from None

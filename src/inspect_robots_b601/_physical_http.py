"""Isolated HTTP I/O only. This child never imports or owns a robot session."""

from __future__ import annotations

import base64
import json
import sys

import httpx

MAX_BODY = 1024 * 1024


def main() -> None:
    request = json.loads(sys.stdin.buffer.read(MAX_BODY + 1))
    with httpx.Client(
        timeout=4.0,
        trust_env=False,
        follow_redirects=False,
        transport=httpx.HTTPTransport(retries=0),
    ) as client:
        with client.stream(
            "POST",
            request["url"],
            headers=request["headers"],
            content=base64.b64decode(request["body"], validate=True),
        ) as response:
            body = bytearray()
            for chunk in response.iter_bytes():
                body.extend(chunk)
                if len(body) > MAX_BODY:
                    raise ValueError("Provider response exceeds the physical trial body budget")
            print(
                json.dumps(
                    {
                        "status": response.status_code,
                        "body": base64.b64encode(body).decode("ascii"),
                    }
                )
            )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(type(error).__name__, file=sys.stderr)
        raise SystemExit(1) from None

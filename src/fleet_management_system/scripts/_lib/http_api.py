from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Optional


def http_json(
    url: str,
    method: str = "GET",
    body: Optional[dict] = None,
    timeout: float = 10.0,
) -> Any:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode()
        return json.loads(raw) if raw else {}


def http_json_strict(
    method: str,
    url: str,
    payload: Optional[dict] = None,
    timeout: float = 5.0,
) -> dict:
    try:
        data = http_json(url=url, method=method, body=payload, timeout=timeout)
        return data if isinstance(data, dict) else {}
    except urllib.error.HTTPError as e:
        msg = e.read().decode("utf-8", errors="ignore")
        raise RuntimeError(f"HTTP {e.code} {method} {url}: {msg}") from e
    except urllib.error.URLError as e:
        raise RuntimeError(f"URL error {method} {url}: {e}") from e

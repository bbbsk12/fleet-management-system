"""HTTP API 调用工具模块。

提供基于 urllib 的 JSON HTTP 客户端封装，包含简单的 JSON 请求
和带错误处理的严格模式请求函数，用于与后端 REST API 通信。
"""

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
    """发起 HTTP JSON 请求并返回解析后的结果。

    使用 urllib 发送指定方法的 HTTP 请求，自动将请求体序列化为 JSON，
    并解析 JSON 格式的响应内容。

    Args:
        url: 请求的目标 URL。
        method: HTTP 请求方法（如 GET、POST、PUT、DELETE），默认为 GET。
        body: 可选的请求体字典，非 None 时会作为 JSON 发送。
        timeout: 请求超时时间（秒），默认为 10.0。

    Returns:
        解析后的 JSON 数据，若响应为空则返回空字典。
    """
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
    """发起 HTTP JSON 请求（严格模式），在失败时抛出详细异常。

    对 http_json 进行封装，确保返回值始终为字典类型，并在遇到
    HTTP 错误或 URL 错误时抛出包含详细信息的 RuntimeError。

    Args:
        method: HTTP 请求方法。
        url: 请求的目标 URL。
        payload: 可选的请求体字典。
        timeout: 请求超时时间（秒），默认为 5.0。

    Returns:
        解析后的 JSON 字典，若结果非字典类型则返回空字典。

    Raises:
        RuntimeError: 当 HTTP 请求失败或 URL 无法访问时抛出，
                      异常信息包含状态码、方法和 URL 详情。
    """
    try:
        data = http_json(url=url, method=method, body=payload, timeout=timeout)
        return data if isinstance(data, dict) else {}
    except urllib.error.HTTPError as e:
        msg = e.read().decode("utf-8", errors="ignore")
        raise RuntimeError(f"HTTP {e.code} {method} {url}: {msg}") from e
    except urllib.error.URLError as e:
        raise RuntimeError(f"URL error {method} {url}: {e}") from e

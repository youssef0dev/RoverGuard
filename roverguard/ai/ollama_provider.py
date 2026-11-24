"""Minimal Ollama client using only Python's standard library."""

from __future__ import annotations

import json
import os
import socket
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping


class OllamaProviderError(RuntimeError):
    """Clear boundary error for local-model failures."""


@dataclass(frozen=True)
class OllamaProvider:
    model: str = "qwen3.5:4b-q4_K_M"
    base_url: str = "http://127.0.0.1:11434"
    timeout_s: float = 300.0

    def __post_init__(self) -> None:
        configured = os.environ.get("OLLAMA_HOST", self.base_url).strip()
        if not configured.startswith(("http://", "https://")):
            configured = f"http://{configured}"
        object.__setattr__(self, "base_url", configured.rstrip("/"))

    def generate_mission(
        self,
        *,
        system_prompt: str,
        user_command: str,
        json_schema: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        request_body = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_command},
            ],
            "stream": False,
            "think": False,
            "format": json_schema,
            "options": {"temperature": 0},
            "keep_alive": "10m",
        }
        encoded = json.dumps(request_body).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}/api/chat",
            data=encoded,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                response_payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            raise OllamaProviderError(
                f"Ollama HTTP {error.code}: {detail[:300]}"
            ) from error
        except urllib.error.URLError as error:
            if isinstance(error.reason, (socket.timeout, TimeoutError)):
                raise OllamaProviderError(
                    "Ollama kayjawb, walakin model ma-kmlch dakhel "
                    f"{self.timeout_s:.0f} seconds."
                ) from error
            raise OllamaProviderError(
                "Ma9drnach nwslou l Ollama. T2akked belli Ollama khddam "
                f"f {self.base_url}. Sabab: {error.reason}"
            ) from error
        except (socket.timeout, TimeoutError) as error:
            raise OllamaProviderError(
                "Ollama kayjawb, walakin model ma-kmlch dakhel "
                f"{self.timeout_s:.0f} seconds."
            ) from error
        except json.JSONDecodeError as error:
            raise OllamaProviderError("Ollama returned invalid API JSON") from error

        try:
            content = response_payload["message"]["content"]
            mission_payload = json.loads(content)
        except (KeyError, TypeError, json.JSONDecodeError) as error:
            raise OllamaProviderError(
                "Model ma-rj3ch mission JSON s7i7a"
            ) from error
        if not isinstance(mission_payload, dict):
            raise OllamaProviderError("Model mission must be one JSON object")
        return mission_payload

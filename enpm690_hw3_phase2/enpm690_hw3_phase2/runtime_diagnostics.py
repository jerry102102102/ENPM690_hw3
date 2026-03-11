from __future__ import annotations

import importlib
import sys


def log_python_runtime(context: str) -> None:
    print(f"[{context}] sys.executable={sys.executable}", flush=True)
    print(f"[{context}] sys.version={sys.version}", flush=True)
    for module_name in ("gymnasium", "stable_baselines3"):
        try:
            module = importlib.import_module(module_name)
        except Exception as exc:
            print(f"[{context}] import {module_name}=FAIL ({type(exc).__name__}: {exc})", flush=True)
        else:
            version = getattr(module, "__version__", "unknown")
            print(f"[{context}] import {module_name}=OK (version={version})", flush=True)

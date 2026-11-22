"""Exercise real native serialization, batching and mixed callback clocks."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    env = dict(os.environ)
    env.pop("ECC_REPLAY_CAPTURE_DIR", None)
    subprocess.run([sys.argv[1]], env=env, cwd=root, check=True)
    assert not list(root.iterdir())
    env.update(ECC_REPLAY_CAPTURE_DIR=str(root), ECC_REPLAY_OFFSET_NS="123456")
    subprocess.run([sys.argv[1]], env=env, cwd=root, check=True)
    frames = sorted((json.loads(p.read_text()) for p in root.glob("*.json")), key=lambda f: f["sequence"])
    assert [f["sequence"] for f in frames] == list(range(5))
    patch = frames[0]
    assert patch["phase"] == "changed" and patch["scope"] == "patch"
    assert set(patch["objects"]) == {"inst:moved"}
    assert patch["objects"]["inst:moved"]["bbox"] == [300, 400, 400, 500]
    assert patch["remove"] == ["inst:removed"]
    assert all(a["elapsed_ns"] <= b["elapsed_ns"] for a, b in zip(frames, frames[1:]))
    assert frames[0]["elapsed_ns"] >= 123456
    decisions = frames[1:]
    assert sorted(len(f["metrics"]["decisions"]) for f in decisions) == [1, 1, 128, 128]
    assert all(f["scope"] == "patch" and f["state_domain"] == "candidate" and f["objects"] == {} for f in decisions)
    assert sum(f["metrics"]["accepted"] for f in decisions) == 130
    assert sum(f["metrics"]["reverted"] for f in decisions) == 128
    assert all(f["metrics"]["decisions"][0]["instance"].startswith('u"\\\n') for f in decisions)
    env["ECC_REPLAY_CAPTURE_DIR"] = str(root / "missing")
    subprocess.run([sys.argv[1]], env=env, cwd=root, check=True)

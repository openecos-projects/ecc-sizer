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
    assert [f["sequence"] for f in frames] == list(range(6))
    patch, next_stage = frames[:2]
    assert (patch["phase"], patch["scope"], patch["state_domain"]) == (
        "changed", "patch", "native_committed"
    )
    assert set(patch["objects"]) == {"inst:moved", "inst:new_name"}
    assert patch["objects"]["inst:moved"]["bbox"] == [300, 400, 400, 500]
    assert patch["objects"]["inst:moved"]["pins"]["A"]["net"] == "other"
    assert patch["objects"]["inst:new_name"]["name"] == "new_name"
    assert patch["remove"] == ["inst:old_name", "inst:removed"]
    assert next_stage["phase"] == "next_stage"
    assert set(next_stage["objects"]) == {"inst:moved"}
    assert next_stage["objects"]["inst:moved"]["bbox"] == [0, 0, 100, 100]
    assert all(a["elapsed_ns"] <= b["elapsed_ns"] for a, b in zip(frames, frames[1:]))
    assert frames[0]["elapsed_ns"] >= 123456
    decisions = frames[2:]
    assert sorted(len(f["metrics"]["decisions"]) for f in decisions) == [1, 1, 128, 128]
    assert all(f["scope"] == "patch" and f["state_domain"] == "candidate" and f["objects"] == {} for f in decisions)
    assert sum(f["metrics"]["accepted"] for f in decisions) == 130
    assert sum(f["metrics"]["reverted"] for f in decisions) == 128
    assert all(f["metrics"]["decisions"][0]["instance"].startswith('u"\\\n') for f in decisions)
    env["ECC_REPLAY_CAPTURE_DIR"] = str(root / "missing")
    subprocess.run([sys.argv[1]], env=env, cwd=root, check=True)

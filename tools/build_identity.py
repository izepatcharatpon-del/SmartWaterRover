Import("env")

import hashlib
from pathlib import Path

root = Path(env.subst("$PROJECT_DIR"))
inputs = [root / "platformio.ini", root / "tools" / "build_identity.py"]
for source_root in (root / "include", root / "src"):
    inputs.extend(path for path in source_root.rglob("*") if path.is_file())
inputs = sorted(set(inputs), key=lambda path: path.relative_to(root).as_posix())
digest = hashlib.sha256()
for path in inputs:
    relative = path.relative_to(root).as_posix().encode("utf-8")
    payload = path.read_bytes()
    digest.update(len(relative).to_bytes(4, "big"))
    digest.update(relative)
    digest.update(len(payload).to_bytes(8, "big"))
    digest.update(payload)
build_id = digest.hexdigest()[:16]
env.Append(CPPDEFINES=[("USV_BUILD_ID", f'\\"{build_id}\\"')])
print(f"[BUILD_ID] {build_id} from {len(inputs)} dirty working-copy inputs")

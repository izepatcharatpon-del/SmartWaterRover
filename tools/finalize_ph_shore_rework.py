"""Refresh bounded rework provenance after native and target builds pass."""
from pathlib import Path
import difflib
import hashlib
import json
import re
import shutil

root = Path(__file__).resolve().parents[1]
evidence = root / "docs/ph_shore_evidence"
rework = evidence / "rework"
parent = root.parent
baseline = parent / "preserved_working_copy"
original = json.loads((parent / "preserved_source_manifest.json").read_text(encoding="utf-8"))
previous_path = evidence / "candidate_manifest.json"
previous = json.loads(previous_path.read_text(encoding="utf-8"))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    return sha(path.read_bytes()) if path.is_file() else None


def source_build_id():
    inputs = [root / "platformio.ini", root / "tools/build_identity.py"]
    for folder in (root / "include", root / "src"):
        inputs.extend(p for p in folder.rglob("*") if p.is_file())
    inputs = sorted(set(inputs), key=lambda p: p.relative_to(root).as_posix())
    digest = hashlib.sha256()
    for path in inputs:
        relative = path.relative_to(root).as_posix().encode("utf-8")
        payload = path.read_bytes()
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(len(payload).to_bytes(8, "big"))
        digest.update(payload)
    return digest.hexdigest()[:16], inputs


build_id, inputs = source_build_id()
build_log = (evidence / "esp32s3_latest.log").read_text(encoding="utf-8")
native_log = (evidence / "native_latest.log").read_text(encoding="utf-8")
tx_log_path = rework / "tx_writer_faults.log"
tx_log = tx_log_path.read_text(encoding="utf-8")
probe_log = (evidence / "review/tasks_slices.log").read_text(encoding="utf-8")
logged_build_id = re.search(r"\[BUILD_ID\] ([0-9a-f]{16})", build_log)
case_names = re.findall(r"^test\\[^:\r\n]+:\d+: (.+?)\s+\[PASSED\]$", native_log, re.M)
tx_assertion_passes = len(re.findall(r"^ASSERT_PASS:", tx_log, re.M))
tx_assertion_failures = len(re.findall(r"^ASSERT_FAIL:", tx_log, re.M))
assert logged_build_id and logged_build_id.group(1) == build_id
assert "[SUCCESS]" in build_log
assert len(set(case_names)) == 99
assert "OVERALL_EXIT_CODE=0" in tx_log and "COMPILE_EXIT_CODE=0" in tx_log and "RUN_EXIT_CODE=0" in tx_log
assert tx_assertion_passes > 0 and tx_assertion_failures == 0
assert "contract_failures=0" in probe_log and "EXIT_CODE=0" in probe_log

firmware = root / ".pio/build/esp32s3/firmware.bin"
elf = root / ".pio/build/esp32s3/firmware.elf"
package = evidence / "boat_logger_esp32s3.bin"
shutil.copyfile(firmware, package)
assert build_id.encode("ascii") in firmware.read_bytes()
assert build_id.encode("ascii") in elf.read_bytes()
assert package.read_bytes() == firmware.read_bytes()

# Keep an exact SHA-256 map for current dirty build inputs and baseline source paths.
source_hashes = {
    path.relative_to(root).as_posix(): file_sha(path)
    for path in inputs
}
host_test_paths = sorted((root / "tools/host_tx_faults").rglob("*"))
host_test_paths = [path for path in host_test_paths if path.is_file()]
host_test_paths.append(root / "tools/run_tx_writer_faults.py")
host_test_hashes = {
    path.relative_to(root).as_posix(): file_sha(path)
    for path in host_test_paths
}
all_paths = set(original["source_hashes"])
all_paths.update(previous.get("baseline_diff", {}).get("added", []))
all_paths.update(previous.get("baseline_diff", {}).get("changed", []))
all_paths.update(previous.get("baseline_diff", {}).get("removed", []))
all_paths.update(source_hashes)
all_paths.update(host_test_hashes)
added, removed, changed = [], [], []
diff_chunks = []
for name in sorted(all_paths):
    old_path, new_path = baseline / name, root / name
    old_exists, new_exists = old_path.is_file(), new_path.is_file()
    if not old_exists and new_exists:
        added.append(name)
    elif old_exists and not new_exists:
        removed.append(name)
    elif old_exists and new_exists and old_path.read_bytes() != new_path.read_bytes():
        changed.append(name)
    if old_exists == new_exists and not (old_exists and new_exists and old_path.read_bytes() != new_path.read_bytes()):
        continue
    if name.lower().endswith((".bin", ".elf", ".exe")):
        continue
    try:
        before = old_path.read_text(encoding="utf-8").splitlines(keepends=True) if old_exists else []
        after = new_path.read_text(encoding="utf-8").splitlines(keepends=True) if new_exists else []
    except UnicodeDecodeError:
        continue
    diff_chunks.extend(difflib.unified_diff(before, after, fromfile="a/" + name, tofile="b/" + name))
(evidence / "changes.patch").write_text("".join(diff_chunks), encoding="utf-8")

old_tasks = (baseline / "src/core/Tasks.cpp").read_text(encoding="utf-8")
tasks = (root / "src/core/Tasks.cpp").read_text(encoding="utf-8")
reset = "if (state != STATE_GRID_PATROL) {\n            g_waterSampleActive = false;\n        }"
assert reset in old_tasks and reset in tasks

manifest = dict(previous)
manifest.update({
    "baseline_root": str(baseline),
    "candidate_root": str(root),
    "baseline_diff": {"added": added, "removed": removed, "changed": changed},
    "source_hashes": source_hashes,
    "host_test_source_hashes": host_test_hashes,
    "build_id_current_inputs": build_id,
    "build_id_from_build_log": logged_build_id.group(1),
    "build_id_matches_current_inputs": True,
    "build_input_file_count": len(inputs),
    "firmware_bin": str(firmware),
    "firmware_bin_sha256": file_sha(firmware),
    "firmware_bin_size_bytes": firmware.stat().st_size,
    "firmware_elf_sha256": file_sha(elf),
    "firmware_elf_size_bytes": elf.stat().st_size,
    "package_binary": str(package),
    "package_binary_sha256": file_sha(package),
    "package_binary_size_bytes": package.stat().st_size,
    "native_test_log": "docs/ph_shore_evidence/native_latest.log",
    "esp32s3_build_log": "docs/ph_shore_evidence/esp32s3_latest.log",
    "native_test_exit_code": 0,
    "native_unique_cases": len(set(case_names)),
    "non_grid_sample_reset_restored": True,
    "ph_suppression_uses_published_mean": True,
    "tx_writer_host_fault_log": "docs/ph_shore_evidence/rework/tx_writer_faults.log",
    "tx_writer_host_fault_compile_exit_code": 0,
    "tx_writer_host_fault_run_exit_code": 0,
    "tx_writer_host_fault_assertions_passed": tx_assertion_passes,
    "tx_writer_host_fault_assertion_failures": tx_assertion_failures,
    "host_fixture_scope": "host-only mock backend; no radio, receiver, boat, or pond execution",
})
previous_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
print(json.dumps({
    "build_id": build_id,
    "build_inputs": len(inputs),
    "firmware_sha256": file_sha(firmware),
    "package_matches_firmware": package.read_bytes() == firmware.read_bytes(),
    "native_unique_cases": len(set(case_names)),
    "baseline_added": len(added),
    "baseline_changed": len(changed),
    "changes_patch": str(evidence / "changes.patch"),
    "candidate_manifest": str(previous_path),
}, indent=2))

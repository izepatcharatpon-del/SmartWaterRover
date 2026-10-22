"""Host-only raw-byte regression; no newline-normalizing text reads.

Requires the immutable preserved_working_copy next to this task candidate.
Checks exact protected slices and every unchanged line matched in source order.
"""
from pathlib import Path
import difflib
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT.parent / "preserved_working_copy"
SLICES = (
    ("non_grid_reset", b"bool navSensorsOk = gpsOk && compassOk;", b"switch (state)"),
    ("quiet_mean", b"        if (!phQuietMeasure) {", b"        if (turbR.valid && isfinite(turbR.value))"),
    ("GRID", b"case STATE_GRID_PATROL: {", b"case STATE_SPIRAL_MAPPING: {"),
    ("SPIRAL", b"case STATE_SPIRAL_MAPPING: {", b"case STATE_RTH: {"),
    ("RTH", b"case STATE_RTH: {", b"if (fix.valid && isfinite(logTargetLat)"),
)


def normalized(data):
    return data.replace(b"\r\n", b"\n")


def unchanged_matches(old, new):
    old_lines = old.splitlines(keepends=True)
    new_lines = new.splitlines(keepends=True)
    matches = difflib.SequenceMatcher(
        None, [normalized(line) for line in old_lines],
        [normalized(line) for line in new_lines], autojunk=False,
    ).get_matching_blocks()
    return old_lines, new_lines, matches


def extract(data, start, end):
    index = data.index(start)
    return data[index:data.index(end, index)]


def endings(data):
    return {
        "crlf": data.count(b"\r\n"),
        "bare_lf": data.count(b"\n") - data.count(b"\r\n"),
        "bare_cr": data.count(b"\r") - data.count(b"\r\n"),
    }


def check(old, new):
    old_lines, new_lines, matches = unchanged_matches(old, new)
    differences = []
    unchanged_count = 0
    for match in matches:
        for offset in range(match.size):
            unchanged_count += 1
            if old_lines[match.a + offset] != new_lines[match.b + offset]:
                differences.append({
                    "baseline_line": match.a + offset + 1,
                    "candidate_line": match.b + offset + 1,
                })
    slices = {}
    for name, start, end in SLICES:
        before, after = extract(old, start, end), extract(new, start, end)
        slices[name] = {
            "raw_identical": before == after,
            "baseline_bytes": len(before), "candidate_bytes": len(after),
            "baseline_sha256": hashlib.sha256(before).hexdigest(),
            "candidate_sha256": hashlib.sha256(after).hexdigest(),
        }
    passed = not differences and all(item["raw_identical"] for item in slices.values())
    return {
        "passed": passed,
        "baseline_sha256": hashlib.sha256(old).hexdigest(),
        "candidate_sha256": hashlib.sha256(new).hexdigest(),
        "line_endings": {"baseline": endings(old), "candidate": endings(new)},
        "unchanged_lines_checked": unchanged_count,
        "unchanged_line_byte_mismatches": differences,
        "protected_slices": slices,
    }


if __name__ == "__main__":
    old = (BASELINE / "src/core/Tasks.cpp").read_bytes()
    new = (ROOT / "src/core/Tasks.cpp").read_bytes()
    expected = json.loads((ROOT.parent / "preserved_source_manifest.json").read_bytes())
    assert hashlib.sha256(old).hexdigest() == expected["source_hashes"]["src/core/Tasks.cpp"]
    result = check(old, new)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)

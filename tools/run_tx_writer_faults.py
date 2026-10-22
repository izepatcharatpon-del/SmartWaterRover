"""Build and run actual DataLogger.cpp against a bounded host-only fault backend."""
from pathlib import Path
import os
import shlex
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
fixture = Path(__file__).resolve().parent / "host_tx_faults"
stubs = fixture / "host_stubs"
compiler = shutil.which("g++") or "C:/msys64/ucrt64/bin/g++.exe"
scratch = Path(os.environ.get("TMPDIR") or tempfile.gettempdir())
scratch.mkdir(parents=True, exist_ok=True)

with tempfile.TemporaryDirectory(dir=scratch) as temporary:
    executable = Path(temporary) / "tx_writer_faults.exe"
    command = [
        compiler, "-std=c++17", "-DUSV_NATIVE_TEST",
        "-I", str(stubs), "-I", str(root / "include"), "-I", str(root / "src"),
        str(fixture / "test_datalogger_tx_faults.cpp"),
        str(root / "src/storage/DataLogger.cpp"),
        str(root / "src/core/DiagnosticFormatting.cpp"),
        "-o", str(executable),
    ]
    transcript = ["HOST-ONLY TX writer fault probe; actual production DataLogger.cpp compiled"]
    compile_result = subprocess.run(command, capture_output=True, text=True)
    transcript.extend(("$ " + shlex.join(command), compile_result.stdout + compile_result.stderr,
                       f"COMPILE_EXIT_CODE={compile_result.returncode}"))
    if compile_result.returncode == 0:
        run_command = [str(executable)]
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        transcript.extend(("$ " + shlex.join(run_command), run_result.stdout + run_result.stderr,
                           f"RUN_EXIT_CODE={run_result.returncode}"))
    else:
        run_result = None

    tasks = (root / "src/core/Tasks.cpp").read_text(encoding="utf-8")
    tx_start = tasks.index("const bool ok = ctx->link->sendTelemetry(tp);")
    tx_end = tasks.index("\n        if (!ctx->radioTxPaused &&\n            millis() - lastBattReport", tx_start)
    tx_wiring = tasks[tx_start:tx_end]
    send_count = tx_wiring.count("sendTelemetry(")
    writer_count = tx_wiring.count("logTxDiagnostic(")
    source_wiring_ok = (
        send_count == 1 and writer_count == 1 and
        tx_wiring.index("logTxDiagnostic(") > tx_wiring.index("sendTelemetry(") and
        "sendTelemetry(" not in tx_wiring[tx_wiring.index("logTxDiagnostic("):]
    )
    transcript.append(
        f"TASKS_SOURCE_WIRING={int(source_wiring_ok)} "
        f"sendTelemetry_calls={send_count} logTxDiagnostic_calls={writer_count} "
        "(static source contract only; no radio execution)"
    )
    final_code = compile_result.returncode
    if run_result is not None:
        final_code = final_code or run_result.returncode
    if not source_wiring_ok:
        final_code = final_code or 1
    transcript.append(f"OVERALL_EXIT_CODE={final_code}")

    log_path = root / "docs/ph_shore_evidence/rework/tx_writer_faults.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text("\n".join(transcript) + "\n", encoding="utf-8")
    print("\n".join(transcript))
    raise SystemExit(final_code)

"""Actual production DataLogger host comparison; simulated latency, not HIL."""
from pathlib import Path
import argparse,hashlib,os,shutil,subprocess,sys
r=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,help='Preserved pre-change source directory')
args=parser.parse_args()
baseline=args.baseline or (r/'docs/sd_baseline_20261015' if (r/'docs/sd_baseline_20261015').is_dir() else r.parent/'sd_baseline_20261015')
if not (baseline/'src/storage/DataLogger.cpp').is_file():
    parser.error('Baseline DataLogger.cpp not found; pass --baseline PATH')
evidence=r/'docs/sd_rate_evidence';evidence.mkdir(parents=True,exist_ok=True)
number=1
while (evidence/f'attempt_{number}.log').exists():number+=1
log=evidence/f'attempt_{number}.log'
compiler=shutil.which('g++') or 'C:/msys64/ucrt64/bin/g++.exe'
stubs=r/'tools/host_tx_faults/host_stubs';transcript=[];status=0

def run(command):
    global status
    result=subprocess.run([str(x) for x in command],capture_output=True,text=True)
    # Keep every failure; compact successful repeated row assertions.
    lines=(result.stdout+result.stderr).splitlines()
    passed=sum(x.startswith('ASSERT_PASS:') for x in lines)
    kept=[x for x in lines if not x.startswith('ASSERT_PASS:')]
    transcript.extend(['$ '+' '.join(str(x) for x in command),*kept,f'ASSERT_PASS_COUNT={passed}',f'EXIT_CODE={result.returncode}'])
    status=status or result.returncode
    return result.returncode==0

for kind in ['baseline','candidate']:
    binary=evidence/(kind+'.exe')
    source=baseline/'src/storage/DataLogger.cpp' if kind=='baseline' else r/'src/storage/DataLogger.cpp'
    includes=[baseline/'src',r/'include',r/'src'] if kind=='baseline' else [r/'include',r/'src']
    command=[compiler,'-std=c++17','-DUSV_NATIVE_TEST']
    if kind=='candidate':command+=['-DUSV_BUFFERED_SD']
    command+=['-I',stubs]
    for inc in includes:command+=['-I',inc]
    command+=[r/'tools/host_tx_faults/test_datalogger_rate.cpp',source,r/'src/core/DiagnosticFormatting.cpp','-o',binary]
    if run(command):
        for profile in (['reference'] if kind=='baseline' else ['fast','stall']):run([binary,profile,evidence/(kind+'_'+profile)])
if not status:
    for profile in ['fast','stall']:
        for suffix in ['.csv','.ph.csv','.tx.csv']:
            a=(evidence/('baseline_reference'+suffix)).read_bytes();b=(evidence/('candidate_'+profile+suffix)).read_bytes()
            same=a==b
            transcript.append(f'EXACT_BYTES_{profile}_{suffix}={int(same)} BASELINE_ROWS={a.count(bytes([10]))-1} CANDIDATE_ROWS={b.count(bytes([10]))-1} SHA256={hashlib.sha256(b).hexdigest()}')
            if not same:status=1
transcript.append(f'OVERALL_EXIT_CODE={status}')
log.write_text('\n'.join(transcript)+'\n',encoding='utf-8')
print('\n'.join(transcript));print('EVIDENCE_LOG='+str(log));raise SystemExit(status)

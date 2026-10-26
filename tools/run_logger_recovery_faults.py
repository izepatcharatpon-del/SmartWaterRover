"""Actual DataLogger with host fault injection. No radio, SD card, or FreeRTOS execution."""
from pathlib import Path
import argparse,json,os,shutil,subprocess

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,default=root,help='Frozen source tree or candidate to compile')
parser.add_argument('--output',type=Path,default=root/'docs/logger_recovery_faults')
args=parser.parse_args();source=args.source.resolve();args.output.mkdir(parents=True,exist_ok=True)
number=1
while (args.output/f'attempt_{number}.log').exists():number+=1
compiler=Path(shutil.which('g++') or 'C:/msys64/ucrt64/bin/g++.exe')
env=os.environ.copy();env['PATH']=str(compiler.parent)+os.pathsep+env.get('PATH','')
fixture=root/'tools/host_logger_faults';binary=args.output/f'probe_{number}.exe'
cmd=[str(compiler),'-std=c++17','-DUSV_NATIVE_TEST','-I',str(fixture/'host_stubs'),'-I',str(source/'include'),'-I',str(source/'src'),str(fixture/'test_logger_recovery.cpp'),str(source/'src/storage/DataLogger.cpp'),str(source/'src/core/DiagnosticFormatting.cpp'),'-o',str(binary)]
p=subprocess.run(cmd,capture_output=True,text=True,env=env)
parts=['COMPILE_COMMAND='+subprocess.list2cmdline(cmd),'COMPILE_EXIT='+str(p.returncode),p.stdout,p.stderr]
run_exit=None
if not p.returncode:
    q=subprocess.run([str(binary)],capture_output=True,text=True,env=env)
    run_exit=q.returncode;parts+=['RUN_EXIT='+str(q.returncode),q.stdout,q.stderr]
transcript='\n'.join(parts);(args.output/f'attempt_{number}.log').write_text(transcript,encoding='utf-8')
cases=[dict(result=line.split()[1],name=line.split()[2]) for line in transcript.splitlines() if line.startswith('CASE ')]
report=dict(compile_exit=p.returncode,run_exit=run_exit,passed=sum(x['result']=='PASS' for x in cases),failed=sum(x['result']=='FAIL' for x in cases),cases=cases,limits=__doc__)
(args.output/f'attempt_{number}.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(transcript);raise SystemExit(p.returncode or run_exit or 0)

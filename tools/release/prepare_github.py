"""Create a curated publication tree. Does not rewrite the local Git history."""
import argparse
from datetime import datetime,timezone,timedelta
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT=Path(__file__).resolve().parents[2]


def sha(raw):return hashlib.sha256(raw).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'publish/github/vision-guided-robotic-sorting')
    args=parser.parse_args()
    output=args.output.resolve()
    boundary=(ROOT/'publish').resolve()
    if boundary not in output.parents:raise ValueError('Publication must stay inside project/publish')
    if output.exists():raise FileExistsError('Will not overwrite existing publication: '+str(output))
    payload={}
    for name in ('deployment/visual-pick-sort-v23-files.sha256',
                 'deployment/visual-task-control-v24-files.sha256'):
        raw=(ROOT/name).read_bytes();payload[name]=raw
        for line in raw.decode('ascii').splitlines():
            digest,relative=line.split('  ',1)
            source=(ROOT/relative).resolve()
            if ROOT not in source.parents:raise ValueError('Manifest path escapes project')
            content=source.read_bytes()
            if sha(content)!=digest:raise ValueError('Runtime file differs: '+relative)
            payload[relative]=content
    # Include the ROS/source package, excluding model/data binaries and generated files.
    for source in (ROOT/'src').rglob('*'):
        if not source.is_file() or source.is_symlink():continue
        relative=source.relative_to(ROOT).as_posix()
        parts=source.relative_to(ROOT/'src').parts
        if any(p in ('__pycache__','.pytest_cache','test_data') for p in parts):continue
        if source.suffix.lower() not in ('.py','.yaml','.yml','.xml','.launch','.txt','.md','.sh','.cfg','.rviz'):continue
        if '/models/' in relative and source.name!='README.md':continue
        payload[relative]=source.read_bytes()
    for name in ('README.md','README.en.md','requirements-web.txt','THIRD_PARTY_NOTICES.md',
                 '.gitattributes','docs/ASSETS.md','docs/guide.zh-CN.html','docs/guide.en.html',
                 'docs/images/head-camera-scene.png','docs/images/head-camera-scene.provenance.json','deployment/README.md',
                 'deployment/CURRENT_RELEASE.json','deployment/VISUAL_WEB_CONTROL_V24_ONSITE_COMMAND.txt',
                 'tools/release/build_runtime_archive.py','tools/release/prepare_github.py',
                 'tools/release/render_project_guides.py'):
        payload[name]=(ROOT/name).read_bytes()
    payload['.gitignore']=b'''__pycache__/
*.py[cod]
.pytest_cache/
.venv/
venv/
node_modules/
/build/
/devel/
/logs/
/captures/
/calibration/
/tmp/
/output/
/publish/
/dist/
/docs/acceptance/
*.zip
*.7z
*.tar.gz
*.pt
*.torchscript
*.onnx
*.npy
*.npz
*.so
*.dll
*.exe
*.log
.env
.env.*
'''
    expressions=[re.compile(r'\bgh[pousr]_[A-Za-z0-9]{20,}'),
        re.compile(r'\bgithub_pat_[A-Za-z0-9_]{20,}'),
        re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
        re.compile(r'(?i)(?:access_token|api_secret|client_secret)\s*[:=]\s*[\"\x27][A-Za-z0-9_\-]{24,}')]
    findings=[]
    for name,raw in payload.items():
        if len(raw)>10*1024*1024:raise ValueError('Large source-publication file: '+name)
        try:text=raw.decode('utf8')
        except UnicodeError:continue
        for number,line in enumerate(text.splitlines(),1):
            if any(pattern.search(line) for pattern in expressions):findings.append(dict(path=name,line=number))
    if findings:raise ValueError('Potential credentials found at paths/lines: '+json.dumps(findings))
    # Exact copies preserve the runtime hash chain; no source rewrites or SDK loading.
    for name,raw in sorted(payload.items()):
        target=output/name
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(raw)
        if sha(target.read_bytes())!=sha(raw):raise ValueError('Publication copy differs: '+name)
    listing={name:dict(bytes=len(raw),sha256=sha(raw)) for name,raw in sorted(payload.items())}
    receipt=dict(title='Vision-Guided Robotic Grasping and Sorting System',
        repository='VincentCassano/vision-guided-robotic-sorting',visibility='public',
        prepared_cst=datetime.now(timezone(timedelta(hours=8))).isoformat(),
        output=str(output),files=len(payload),bytes=sum(len(raw) for raw in payload.values()),
        local_history_rewritten=False,credential_pattern_findings=findings,
        excluded=['local acceptance history','raw captures','teaching bundles','weights',
                  'vendor SDK binaries','on-site calibration','temporary files','historical bulk packages'],
        device_or_project_execution=False,manifest=listing)
    local=ROOT/'docs/maintenance/github_publication_20261026.json'
    local.write_bytes((json.dumps(receipt,ensure_ascii=False,indent=2)+'\n').encode('utf8'))
    public=dict(project_title=receipt['title'],repository=receipt['repository'],
        files=receipt['files'],bytes=receipt['bytes'],manifest=listing)
    (output/'SOURCE_MANIFEST.json').write_bytes((json.dumps(public,ensure_ascii=False,indent=2)+'\n').encode('utf8'))
    print(json.dumps({k:v for k,v in receipt.items() if k!='manifest'},ensure_ascii=False))


if __name__=='__main__':main()

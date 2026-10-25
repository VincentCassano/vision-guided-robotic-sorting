"""Package manifest-listed files without loading SDKs or running project code."""
import argparse
import hashlib
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[2]
MANIFESTS=('deployment/visual-pick-sort-v23-files.sha256',
           'deployment/visual-task-control-v24-files.sha256')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if output==ROOT or ROOT not in output.parents:raise ValueError('Output must stay inside the project')
    payload={}
    for name in MANIFESTS:
        manifest=(ROOT/name).read_bytes()
        payload[name]=manifest
        for line in manifest.decode('ascii').splitlines():
            expected,relative=line.split('  ',1)
            path=(ROOT/relative).resolve()
            if ROOT not in path.parents:raise ValueError('Manifest path escapes project')
            raw=path.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('File differs: '+relative)
            payload[relative]=raw
    for name in ('README.md','README.en.md','requirements-web.txt','THIRD_PARTY_NOTICES.md',
                 'docs/ASSETS.md','docs/guide.zh-CN.html','docs/guide.en.html',
                 'deployment/README.md','deployment/CURRENT_RELEASE.json',
                 'src/battery_grasp/models/README.md'):
        if (ROOT/name).is_file():payload[name]=(ROOT/name).read_bytes()
    output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output,'x',zipfile.ZIP_DEFLATED) as archive:
        for name,raw in sorted(payload.items()):archive.writestr(name,raw)
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:raise ValueError('Archive CRC differs')
    print(str(output))
    print('SHA256 '+hashlib.sha256(output.read_bytes()).hexdigest())
    print('File packaging only; no device/project execution.')


if __name__=='__main__':main()

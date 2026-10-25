"""Native JSON records and archive I/O only; no model or device imports."""
import hashlib
import json
from pathlib import Path
import zipfile


def native(value):
    if value is None or type(value) in (bool, int, float, str):
        return value
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise TypeError('Record keys must be strings')
        return {key: native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [native(item) for item in value]
    if type(value).__module__.split('.')[0] == 'numpy':
        # Scalars (including bool_) and ndarrays become explicit native values.
        converted = value.tolist() if hasattr(value, 'tolist') else value.item()
        return native(converted)
    raise TypeError('Unsupported record type: '+type(value).__name__)


def encode(value):
    return json.dumps(native(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n'


def dump(path, value):
    path = Path(path)
    encoded = encode(value).encode('utf8')
    pending = path.with_name(path.name+'.pending')
    pending.write_bytes(encoded)
    pending.replace(path)


def archive_folder(folder):
    folder = Path(folder)
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in folder.iterdir() if p.is_file() and p.name != 'source_hashes.json'}
    dump(folder/'source_hashes.json', hashes)
    archive = folder.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
        for p in sorted(folder.iterdir()):
            if p.is_file():
                z.write(p, folder.name+'/'+p.name)
    return archive

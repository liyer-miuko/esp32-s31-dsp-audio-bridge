"""Check public snapshot bytes. No firmware execution or hardware access."""
from pathlib import Path
import hashlib,json,re,sys
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'SOURCE_MANIFEST.json').read_text('utf-8'))
errors=[]
for entry in manifest['copied_files']:
    path=root/entry['path']
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest().upper()!=entry['sha256']:
        errors.append(entry['path'])
sdk=json.loads((root/'sdk/manifest.json').read_text('utf-8'))
for entry in sdk['entries']+sdk['lgpl_support']:
    if entry.get('published',True):
        path=root/'sdk/overlay'/entry['path']
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest().upper()!=entry['sha256']:
            errors.append('SDK:'+entry['path'])
for entry in sdk['tinyusb']['entries']:
    path=root/'sdk/tinyusb-overlay'/entry['path']
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest().upper()!=entry['sha256']:
        errors.append('TinyUSB:'+entry['path'])
frozen=json.loads((root/'firmware/main/core1_frozen.json').read_text('utf-8'))
for name,identity in frozen['sources'].items():
    path=root/'firmware'/name
    if not path.is_file() or hashlib.sha256(path.read_bytes().replace(b'\r\n',b'\n')).hexdigest().upper()!=identity['lf_sha256']:
        errors.append('Frozen source:'+name)
# This source guard is not an object-build or device test.
html=(root/'webui/dsp webui.html').read_text('utf-8')
if re.search(r'<script[^>]+src=|<link[^>]+href=',html): errors.append('HTML external dependency')
if len(re.findall(r'<script\b',html))!=6:errors.append('HTML incomplete scripts')
print(json.dumps({'ok':not errors,'snapshot_files':len(manifest['copied_files']),
                  'frozen_source_files':len(frozen['sources']),'binary_distributed':False,
                  'firmware_build_checked':False,'hardware_tested':False,'errors':errors},ensure_ascii=False))
sys.exit(bool(errors))

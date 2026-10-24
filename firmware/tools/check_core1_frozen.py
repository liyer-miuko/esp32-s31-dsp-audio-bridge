"""Offline build guard. No device access, tests or changes to frozen files."""
from pathlib import Path
import argparse,hashlib,json,re,sys,os

ROOT=Path(__file__).resolve().parents[1]
def digest(b):return hashlib.sha256(b).hexdigest().upper()
def source_digest(p):return digest(p.read_bytes().replace(b'\r\n',b'\n'))
def interface_digest(p):
    s=p.read_text(encoding='utf-8')
    a=s.index('/* S31 HWLOOP 生产预算');b=s.index('/* ---------------- DSP 精度模式',a)
    s=s[:a]+s[b:]   # core0 admission/budget constants may change; ABI/math may not.
    s=re.sub(r'/\*[\s\S]*?\*/|//[^\n]*','',s)
    return digest(' '.join(s.split()).encode('utf-8'))
def object_sections(p):
    from elftools.elf.elffile import ELFFile
    with p.open('rb') as f:
        e=ELFFile(f);out={}
        for s in e.iter_sections():
            if int(s['sh_flags'])&2:
                out[s.name]=dict(type=s['sh_type'],bytes=int(s['sh_size']),sha256=digest(s.data()))
            elif s['sh_type'] in ('SHT_REL','SHT_RELA'):
                target=e.get_section(s['sh_info'])
                if not int(target['sh_flags'])&2:continue
                sym=e.get_section(s['sh_link']);rs=[]
                for r in s.iter_relocations():
                    t=sym.get_symbol(r['r_info_sym'])
                    name=t.name or ('section:'+e.get_section(t['st_shndx']).name
                        if isinstance(t['st_shndx'],int) else str(t['st_shndx']))
                    rs.append([int(r['r_offset']),int(r['r_info_type']),name,int(r.entry.get('r_addend',0))])
                out[s.name]=dict(type=s['sh_type'],target=target.name,relocations=rs)
        return out
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--objects',action='store_true');a=ap.parse_args()
    m=json.loads((ROOT/'main/core1_frozen.json').read_text(encoding='utf-8'));failed=[]
    for name,v in m['sources'].items():
        if not (ROOT/name).is_file() or source_digest(ROOT/name)!=v['lf_sha256']:failed.append(name)
    if interface_digest(ROOT/'main/dsp_config.h')!=m['shared_config_interface_sha256']:
        failed.append('main/dsp_config.h ABI/math contract (budget block excluded)')
    sdk=Path(os.environ.get('IDF_PATH',m['sdk']['path']))
    for name,v in m['sdk']['files'].items():
        if not (sdk/name).is_file() or source_digest(sdk/name)!=v:failed.append('SDK:'+name)
    cfg=(ROOT/'sdkconfig').read_text(encoding='utf-8')
    for k,v in m['critical_config'].items():
        if not re.search(r'^'+re.escape(k)+'='+re.escape(v)+r'$',cfg,re.M):failed.append(k)
    if a.objects:
        od=ROOT/'build-fork/esp-idf/main/CMakeFiles/__idf_main.dir'
        for n,v in m['objects'].items():
            if not (od/n).is_file() or object_sections(od/n)!=v['sections']:failed.append('object:'+n)
    print(json.dumps(dict(ok=not failed,source_files=len(m['sources']),objects_checked=a.objects,
                         failures=failed,policy='Change frozen code only after explicit user unfreeze; do not regenerate anchors automatically.'),ensure_ascii=False))
    return 1 if failed else 0
if __name__=='__main__':sys.exit(main())

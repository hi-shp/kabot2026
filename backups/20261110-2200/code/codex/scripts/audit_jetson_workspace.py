#!/usr/bin/env python3
"""Inventory and sanitized source snapshot; never deletes or runs boat nodes."""
import argparse
import ast
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import tarfile
import xml.etree.ElementTree as ET
import zipfile

SKIP_DIRS = {'.git', 'build', 'install', 'log', 'logs', '__pycache__', '.pytest_cache',
             '.agents', '.codex', '.cursor', '.venv', 'venv'}
LOCAL_RULES = {'AGENTS.md', 'CLAUDE.md', 'GEMINI.md'}
DATA_EXT = {'.log', '.bag', '.db3', '.mcap', '.pcap', '.pcapng'}
ARCHIVE_EXT = {'.zip', '.tar', '.gz', '.bz2', '.xz', '.7z'}
SECRET_KEYS = r'(?:[A-Za-z_][\w.-]*[_-])?(?:password|passwd|username|access_token|api_key|secret_key|client_secret|authorization|auth_token)'
QUOTED_SECRET = re.compile(r'(?im)(\b' + SECRET_KEYS + r'[\"\x27]?\s*[:=]\s*)([\"\x27])([^\r\n]*?)(\2)')
YAML_SECRET = re.compile(r'(?im)^(\s*' + SECRET_KEYS + r'\s*:\s*)([^\r\n#]+)')
URL_CREDENTIAL = re.compile(r'(https?://)[^/@\s:]+:[^/@\s]+@')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def scrub(data):
    text = data.decode('utf-8', errors='surrogateescape')
    changes = 0
    def quoted(match):
        nonlocal changes
        value = match.group(3)
        # Empty/default placeholders and environment references are not credentials.
        if not value.strip() or '<REDACTED>' in value or '${' in value or value.lower() in ('none','null','your_password','your_username'):
            return match.group(0)
        changes += 1
        return match.group(1) + match.group(2) + '<REDACTED>' + match.group(4)
    text = QUOTED_SECRET.sub(quoted, text)
    def yaml_value(match):
        nonlocal changes
        value=match.group(2).strip()
        if value.startswith(('"', "'", '{', '[', '${')) or value in ('null','~','false','true'):
            return match.group(0)
        changes += 1
        return match.group(1) + '"<REDACTED>" '
    text = YAML_SECRET.sub(yaml_value, text)
    text, n = URL_CREDENTIAL.subn(r'\1<REDACTED>@', text)
    changes += n
    # Never publish a private key even if it was mistakenly placed in a source tree.
    if re.search(r'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----', text):
        return None, changes + 1
    return text.encode('utf-8', errors='surrogateescape'), changes


def public_text(value):
    data, _ = scrub(str(value).encode())
    return data.decode() if data is not None else '[PRIVATE KEY OMITTED]'


def literal(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, (str,int,float,bool)):
        return node.value
    try:
        return ast.unparse(node)[:180]
    except Exception:
        return '<expression>'


def inspect_python(data, path):
    try:
        tree=ast.parse(data, filename=path)
    except (SyntaxError, ValueError) as exc:
        return {'syntax_error': {'type':type(exc).__name__,'line':getattr(exc,'lineno',None)}}
    imports=set(); endpoints=[]; nodes=[]; entries=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import): imports.update(n.name for n in node.names)
        elif isinstance(node,ast.ImportFrom) and node.module: imports.add(node.module)
        elif isinstance(node,ast.Call):
            name=node.func.attr if isinstance(node.func,ast.Attribute) else getattr(node.func,'id','')
            if name in ('create_subscription','create_publisher','create_client','create_service'):
                endpoints.append({'line':node.lineno,'call':name,'type':literal(node.args[0]) if node.args else None,
                                  'topic_or_service':literal(node.args[1]) if len(node.args)>1 else None})
            elif name in ('Node','LifecycleNode','ComposableNode','get_package_share_directory'):
                info={'line':node.lineno,'call':name}
                info.update({k.arg:literal(k.value) for k in node.keywords if k.arg in ('package','executable','name','namespace')})
                if name=='get_package_share_directory' and node.args:info['package']=literal(node.args[0])
                nodes.append(info)
            elif name=='setup':
                for k in node.keywords:
                    if k.arg=='entry_points':
                        try:entries=ast.literal_eval(k.value).get('console_scripts',[])
                        except (ValueError,TypeError):pass
    return {'imports':sorted(imports),'endpoints':endpoints,'launch_nodes':nodes,'console_scripts':entries}


def run_git(path, *args):
    p=subprocess.run(['git','-C',str(path),*args],capture_output=True,text=True)
    return p.stdout.strip() if p.returncode==0 else None


def shell_evidence(home):
    result={'definitions':[],'history_command_counts':[],'colcon_build_forms':[],'ros_log_sessions':0,'build_summaries':[]}
    for path in [home/'.bashrc',home/'.bash_aliases',home/'.profile',Path('/etc/bash.bashrc'),Path('/etc/profile')]:
        if not path.is_file():continue
        for i,line in enumerate(path.read_text(errors='replace').splitlines(),1):
            if re.search(r'alias\s+(cb|sb|cw|cs|cba)\b|function\s+cb\b|\bcb\s*\(\)|ros2_ws|setup\.bash',line):
                result['definitions'].append({'file':str(path),'line':i,'definition':public_text(line)})
    history=home/'.bash_history'; counts=Counter();builds=Counter()
    if history.exists():
        for line in history.read_text(errors='replace').splitlines():
            for match in re.finditer(r'ros2\s+(launch|run)\s+([\w/-]+)\s+([\w./-]+)',line):
                counts['ros2 '+' '.join(match.groups())]+=1
            if re.match(r'^\s*cb\s*$',line):counts['cb']+=1
            if re.match(r'^\s*colcon build(?:\s|$)',line):
                # Only known build flags/package words. Discard shell suffixes or arbitrary arguments.
                prefix=re.split(r'[;&|<>]',line,maxsplit=1)[0].strip()
                if re.fullmatch(r'[A-Za-z0-9_./=+ \t-]+',prefix):builds[prefix]+=1
    result['history_command_counts']=counts.most_common()
    result['colcon_build_forms']=builds.most_common()
    logs=home/'.ros/log'
    if logs.exists():result['ros_log_sessions']=sum(p.is_dir() and not p.is_symlink() for p in logs.iterdir())
    logroot=home/'ros2_ws/log'
    for path in sorted(logroot.glob('build_*/logger_all.log'))[-15:]:
        for line in path.read_text(errors='replace').splitlines():
            if re.search(r'Starting >>>|Finished <<<|Failed <<<|Aborted <<<|Summary:',line):
                result['build_summaries'].append({'file':str(path.relative_to(home)), 'summary':public_text(line)})
    services=[]
    for service_root in [Path('/etc/systemd/system'),home/'.config/systemd/user',home/'.config/autostart']:
        if not service_root.exists():continue
        for path in service_root.rglob('*'):
            if not path.is_file() or path.is_symlink() or path.stat().st_size>100000:continue
            try:text=path.read_text()
            except (UnicodeError,PermissionError):continue
            if re.search('ros2_ws|micro_ros|vectornav|velodyne',text):
                commands=[line.split('=',1)[1].strip().split()[0] for line in text.splitlines()
                          if re.match(r'^(ExecStart|Exec)=\S',line)]
                services.append({'file':str(path),'executable_only':commands})
    result['startup_services']=services
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',default=str(Path.home()/'ros2_ws/src'))
    parser.add_argument('--output',required=True,help='NEW directory outside source/workspace')
    args=parser.parse_args()
    source=Path(args.source).resolve();output=Path(args.output).resolve()
    if output==source or source in output.parents:
        raise SystemExit('Snapshot output must be outside the source tree')
    output.mkdir(parents=True,exist_ok=False)
    target=output/'ros2_ws/src';target.mkdir(parents=True)
    inventory=[];python_files={};packages=[];repos=[];redacted=[];excluded=Counter()
    for candidate in [source,*sorted(p for p in source.rglob('.git'))]:
        repo=candidate.parent if candidate.name=='.git' else candidate
        head=run_git(repo,'rev-parse','HEAD')
        if head:
            repos.append({'path':str(repo.relative_to(source)), 'head':head,
                          'status':run_git(repo,'status','--short'),
                          'origin':public_text(run_git(repo,'remote','get-url','origin') or '')})

    def save(data, relative, mode=0o644, original=None):
        nonlocal inventory
        item={'path':relative,'source_bytes':len(data),'source_sha256':digest(data)}
        if b'\x00' in data[:8192] or Path(relative).suffix.lower() in ('.pt','.onnx','.stl','.png','.jpg','.jpeg','.gif','.ico'):
            public=data;n=0
        else:
            public,n=scrub(data)
        if public is None:
            item['disposition']='private_key_excluded';excluded['private_key']+=1
        else:
            dest=target/relative;dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes(public);dest.chmod(stat.S_IMODE(mode))
            item.update(disposition='redacted' if n else 'exact',snapshot_bytes=len(public),snapshot_sha256=digest(public))
            if n:redacted.append({'path':relative,'replacements':n})
            if Path(relative).suffix=='.py':python_files[relative]=inspect_python(public,relative)
            if Path(relative).name=='package.xml':
                try:
                    xml=ET.fromstring(public)
                    packages.append({'path':str(Path(relative).parent),'name':xml.findtext('name'),
                                     'build_type':xml.findtext('export/build_type'),
                                     'dependencies':sorted({c.text for c in xml if c.tag.endswith('depend') and c.text})})
                except ET.ParseError:pass
        inventory.append(item)

    for parent, dirs, files in os.walk(source,followlinks=False):
        directory=Path(parent)
        kept=[]
        for name in sorted(dirs):
            p=directory/name
            if name in SKIP_DIRS or name.endswith('.egg-info'):
                excluded['generated_or_local_directory']+=1
            elif p.is_symlink():
                inventory.append({'path':p.relative_to(source).as_posix(),'disposition':'symlink_not_dereferenced','link_target':os.readlink(p)})
            else:kept.append(name)
        dirs[:]=kept
        for name in sorted(files):
            path=directory/name;rel=path.relative_to(source).as_posix()
            if path.is_symlink():
                inventory.append({'path':rel,'disposition':'symlink_not_dereferenced','link_target':os.readlink(path)})
                continue
            suffix=path.suffix.lower()
            if name in LOCAL_RULES or name=='.git' or suffix in DATA_EXT or suffix in ('.pyc','.pyo') or name=='gps_log.txt':
                excluded['local_rules_metadata_or_measurement_logs']+=1
                inventory.append({'path':rel,'disposition':'excluded_non_source','bytes':path.stat().st_size})
                continue
            if suffix in ARCHIVE_EXT:
                inventory.append({'path':rel,'disposition':'original_archive_not_published','source_sha256':digest(path.read_bytes()),'bytes':path.stat().st_size})
                if suffix=='.zip':
                    try:
                        with zipfile.ZipFile(path) as z:
                            if sum(i.file_size for i in z.infolist())>250*1024**2:raise ValueError('Archive too large')
                            for member in z.infolist():
                                member_path=Path(member.filename)
                                if member.is_dir() or member_path.is_absolute() or '..' in member_path.parts:continue
                                if any(p in SKIP_DIRS or p in LOCAL_RULES for p in member_path.parts):continue
                                if member_path.suffix.lower() in DATA_EXT|ARCHIVE_EXT|{'.pyc','.pyo'}:continue
                                if member.file_size>95*1024**2:continue
                                save(z.read(member),rel+'.contents/'+member_path.as_posix())
                    except (zipfile.BadZipFile,ValueError,RuntimeError):excluded['unreadable_archive']+=1
                continue
            if path.stat().st_size>95*1024**2:
                inventory.append({'path':rel,'disposition':'oversize_not_published','bytes':path.stat().st_size})
                excluded['oversize']+=1
                continue
            save(path.read_bytes(),rel,path.stat().st_mode)

    # Prevent duplicate package discovery if this backup is cloned under a ROS source tree.
    (output/'COLCON_IGNORE').write_text('Archived snapshot; not an active ROS source tree.\n')
    report={'schema_version':1,'work_date':'2026-10-29 Asia/Seoul','source':str(source),
            'deletion_performed':False,'inventory':inventory,'packages':packages,'python_files':python_files,
            'repositories':repos,'redacted_files':redacted,'excluded_counts':dict(excluded),
            'execution_evidence':shell_evidence(Path.home()),
            'notes':['History is evidence of commands typed, not proof of successful execution.',
                     'Generated trees, Git internals, AI rules and measurement logs are not published.',
                     'Source archives are expanded into sanitized .contents directories; original archives remain unchanged.',
                     'Credentials are redacted only in this public snapshot; original files are untouched.']}
    (output/'inventory.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    archive=output.with_suffix('.tar.gz')
    with tarfile.open(archive,'w:gz') as tar:
        tar.add(output,arcname=output.name,recursive=True)
    summary={'snapshot':str(output),'archive':str(archive),'archive_sha256':digest(archive.read_bytes()),
             'files':len(inventory),'exact_files':sum(r['disposition']=='exact' for r in inventory),
             'redacted_files':redacted,'packages':len(packages),'python_files':len(python_files),
             'excluded_counts':dict(excluded),'archive_bytes':archive.stat().st_size}
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()

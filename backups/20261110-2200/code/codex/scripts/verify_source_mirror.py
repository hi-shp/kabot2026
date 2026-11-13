#!/usr/bin/env python3
"""Verify the recorded Jetson snapshot and exact winter commit without running ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-index', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    current = json.loads((root/'codex/docs/JETSON_SOURCE_MANIFEST.json').read_text(encoding='utf8'))
    winter = json.loads((root/'codex/레거시코드/WINTER_SOURCE_MANIFEST.json').read_text(encoding='utf8'))
    index = {}
    if args.check_index:
        for entry in subprocess.check_output(['git', 'ls-files', '--stage', '-z'], cwd=root).split(b'\0'):
            if entry:
                header, name = entry.split(b'\t')
                mode, oid, stage = header.decode().split()
                assert stage == '0', name
                index[name.decode()] = (mode, oid)

    def check(name, data, mode):
        if args.check_index:
            oid = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
            assert index.get(name) == (mode, oid), 'Git index mismatch: '+name

    for entry in current['files']:
        name = entry['path']
        path = root/name
        if entry['type'] == 'symlink':
            data = entry['target'].encode()
            assert (str(path.readlink()) if path.is_symlink() else path.read_text(encoding='utf8')) == entry['target'], name
            mode = '120000'
        else:
            data = path.read_bytes()
            assert hashlib.sha256(data).hexdigest() == entry['sha256'], name
            mode = '100755' if entry['mode'] & 0o111 else '100644'
        check(name, data, mode)
    actual = {p.name for p in root.iterdir() if p.is_dir() and p.name not in ('.git', 'codex', 'firmware')}
    assert actual == set(current['top_level_source_directories']), actual
    for name in current['directories']:
        assert (root/name).is_dir(), name
    for entry in winter['files']:
        data = (root/entry['path']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == entry.get('published_sha256', entry['sha256']), entry['path']
        oid = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        assert oid == entry.get('published_git_blob', entry['git_blob']), entry['path']
        check(entry['path'], data, entry['mode'])
    assert (root/'codex/COLCON_IGNORE').is_file()
    print('Jetson source and firmware:', len(current['files']), 'verified')
    redacted = sum('published_sha256' in entry for entry in winter['files'])
    print('Winter commit:', winter['commit'], len(winter['files']), 'verified;', redacted, 'credential-redacted configurations')
    print('Top-level source directories:', len(actual), 'matched')
    if args.check_index:
        print('Git index content and modes: matched')


if __name__ == '__main__':
    main()

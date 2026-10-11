#!/usr/bin/env python3
"""Create a source deployment snapshot; never edits the working repository.

Run with Python 3 on the packaging machine. The shipped runtime uses Python 2.7.
The runtime snapshot is obtained separately from the actual working robot VM.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile
import zipfile

ROOT = Path(__file__).resolve().parent.parent
TEXT_SUFFIXES = {'.py', '.sh', '.yaml', '.yml', '.launch', '.xml', '.md', '.txt',
                 '.json', '.rules', '.rviz', '.cpp', '.h', '.hpp'}


def git(root, *args):
    return subprocess.check_output(['git', '-c', 'core.autocrlf=false', '-C', str(root)] + list(args))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def clean_copy(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    data = src.read_bytes()
    if src.suffix in TEXT_SUFFIXES or src.name in ('.gitattributes', '.gitignore', 'CMakeLists.txt'):
        data = data.replace(b'\r\n', b'\n')
    dst.write_bytes(data)
    if src.suffix == '.sh' or (src.suffix == '.py' and 'scripts' in src.parts):
        dst.chmod(0o755)


def prepare(stage, audit):
    if stage.exists():
        raise RuntimeError('Stage must be a new directory: ' + str(stage))
    stage.mkdir(parents=True)
    git(stage, 'init')
    git(stage, 'fetch', '--no-tags', str(ROOT), 'HEAD')
    git(stage, 'checkout', '-b', 'codex/method3-team-package', 'FETCH_HEAD')
    # Fetching local HEAD retains upstream ancestry, without copying local config,
    # credential helpers, hooks, reflogs, other branches, or hard-linked objects.
    git(stage, 'remote', 'add', 'origin', 'https://github.com/aliensco/intelligent_robot_system.git')
    (stage / '.git' / 'FETCH_HEAD').unlink()
    paths = git(ROOT, 'ls-files', '-z', '--cached', '--others', '--exclude-standard').split(b'\0')
    for raw in sorted(set(paths)):
        if not raw:
            continue
        rel = Path(raw.decode('utf-8'))
        if rel.is_absolute() or '..' in rel.parts:
            raise RuntimeError('Unsafe source path: ' + str(rel))
        src, dst = ROOT / rel, stage / rel
        if src.is_file():
            if src.is_symlink():
                raise RuntimeError('Unexpected source symlink: ' + str(rel))
            clean_copy(src, dst)
        elif dst.is_file():
            dst.unlink()  # Mirror intentionally deleted tracked files, in staging only.
    profile = stage / 'profiles' / 'team_current'
    for name in ('sensors.launch', 'turtlebot_follow_current.launch', 'turtlebot_follow_current.yaml'):
        clean_copy(audit / 'runtime_snapshot' / 'runtime' / name, profile / name)
    launch = profile / 'turtlebot_follow_current.launch'
    data = launch.read_text(encoding='utf-8').replace(
        '/home/haiden/codex_method3_runtime/turtlebot_follow_current.yaml',
        'turtlebot_follow_current.yaml')
    if '/home/haiden/' in data:
        raise RuntimeError('Unexpected machine-local path left in current launch')
    launch.write_text(data, encoding='utf-8', newline='\n')
    observed = json.loads((audit / 'environment.json').read_text(encoding='utf-8'))
    # Installed package versions are useful for diagnosis; they are not an exact
    # mandatory apt lock and contain neither account credentials nor recordings.
    selected = {key: observed[key] for key in ('cv2', 'numpy', 'yaml', 'installed_packages') if key in observed}
    selected.update({'platform': 'Ubuntu 16.04.7 LTS amd64', 'ros': 'Kinetic', 'python': '2.7.12'})
    write_json(stage / 'environment' / 'source-vm-versions.json', selected)
    clean_copy(audit / 'runtime_snapshot' / 'manifest.json', stage / 'environment' / 'runtime-provenance.json')
    clean_copy(stage / 'docs' / 'method3-package-quickstart.md', stage / 'START_HERE.md')
    quickstart = stage / 'START_HERE.md'
    quickstart.write_text(quickstart.read_text(encoding='utf-8').replace(
        '](method3-team-handoff.md)', '](docs/method3-team-handoff.md)').replace(
        '](method3_acceptance.md)', '](docs/method3_acceptance.md)'), encoding='utf-8', newline='\n')
    # Preserve intent when extracted by an archiver that honors Unix modes.
    # install.sh also restores executable bits after Windows ZIP extraction.
    for path in stage.rglob('*'):
        if path.is_file() and '.git' not in path.relative_to(stage).parts and path.suffix == '.sh':
            path.chmod(0o755)
    write_json(stage / 'package-info.json', {
        'name': stage.name, 'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_head': git(ROOT, 'rev-parse', 'HEAD').decode().strip(),
        'source_branch': git(ROOT, 'branch', '--show-current').decode().strip(),
        'includes_working_tree_changes': True,
        'target': 'Ubuntu 16.04 + ROS Kinetic + Python 2.7',
        'runtime_profile': 'profiles/team_current',
        'installation': 'bash install.sh', 'entrypoint': 'bash run.sh',
        'validated_scope': 'See environment/package-validation.json; field competition acceptance is incomplete.'})
    print('STAGED', stage)


def archive(stage, output):
    if not (stage / 'package-info.json').is_file() or not (stage / '.git').is_dir():
        raise RuntimeError('Not a prepared method3 package')
    # Build artifacts never enter the source package, even if the stage was used
    # for local installation before archiving.
    forbidden = {'.venv', '__pycache__', 'workspace', 'runtime', '.tools', 'artifacts'}
    source_files = []
    for path in sorted(stage.rglob('*')):
        rel = path.relative_to(stage)
        if not path.is_file() or '.git' in rel.parts or any(part in forbidden for part in rel.parts):
            continue
        if path.suffix in ('.pyc', '.bag', '.mp4', '.avi', '.pem', '.key') or path.name == '.env':
            raise RuntimeError('Unexpected private/generated file in package: ' + str(rel))
        if path.is_symlink():
            raise RuntimeError('Unexpected symlink: ' + str(rel))
        source_files.append(path)
    checksums = stage / 'FILES.sha256'
    checksums.write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.relative_to(stage).as_posix() + '\n'
                                for p in source_files if p != checksums), encoding='utf-8', newline='\n')
    git(stage, 'add', '-A')
    for path in source_files:
        if path.suffix == '.sh' or (path.suffix == '.py' and 'scripts' in path.relative_to(stage).parts):
            git(stage, 'update-index', '--chmod=+x', '--', path.relative_to(stage).as_posix())
    git(stage, '-c', 'user.name=Method3 Team Package', '-c', 'user.email=method3-package@localhost',
        'commit', '-m', 'Package current Method 3 source, robot profile and portable Kinetic deployment')
    # Local packaging identity is not a teammate's commit identity.
    # Discard source-host paths in transient Git messages/reflogs and sample hooks.
    for item in ('FETCH_HEAD', 'logs', 'hooks'):
        path = stage / '.git' / item
        if path.is_dir():
            shutil.rmtree(path)
        elif path.is_file():
            path.unlink()
    git(stage, 'gc', '--prune=now', '--quiet')
    output.mkdir(parents=True, exist_ok=True)
    files = [p for p in sorted(stage.rglob('*')) if p.is_file() and
             not any(x in forbidden for x in p.relative_to(stage).parts)]
    # Git 2.x requires .git/refs even when gc packed every reference. Preserve
    # empty directories as well as files, so extraction remains a Git checkout.
    directories = [stage] + [p for p in sorted(stage.rglob('*')) if p.is_dir() and
                            not any(x in forbidden for x in p.relative_to(stage).parts)]
    tgz = output / (stage.name + '.tar.gz')
    zip_path = output / (stage.name + '.zip')
    if tgz.exists() or zip_path.exists():
        raise RuntimeError('Output exists; choose a new package name/output directory')
    with tarfile.open(tgz, 'w:gz', format=tarfile.PAX_FORMAT) as tar:
        for path in directories:
            rel = path.relative_to(stage).as_posix()
            name = stage.name if rel == '.' else stage.name + '/' + rel
            info = tar.gettarinfo(str(path), name)
            info.uid = info.gid = 0
            info.uname = info.gname = ''
            info.mode = 0o755
            tar.addfile(info)
        for path in files:
            info = tar.gettarinfo(str(path), stage.name + '/' + path.relative_to(stage).as_posix())
            info.uid = info.gid = 0
            info.uname = info.gname = ''
            info.mode = 0o755 if path.suffix == '.sh' or (path.suffix == '.py' and 'scripts' in path.parts) else 0o644
            with path.open('rb') as stream:
                tar.addfile(info, stream)
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as archive_file:
        for path in directories:
            rel = path.relative_to(stage).as_posix()
            name = stage.name + '/' if rel == '.' else stage.name + '/' + rel + '/'
            info = zipfile.ZipInfo(name)
            info.create_system = 3
            info.external_attr = ((stat.S_IFDIR | 0o755) << 16) | 0x10
            archive_file.writestr(info, b'')
        for path in files:
            info = zipfile.ZipInfo(stage.name + '/' + path.relative_to(stage).as_posix())
            info.create_system = 3
            mode = 0o755 if path.suffix == '.sh' or (path.suffix == '.py' and 'scripts' in path.parts) else 0o644
            info.external_attr = (stat.S_IFREG | mode) << 16
            archive_file.writestr(info, path.read_bytes(), compress_type=zipfile.ZIP_DEFLATED)
    (output / (stage.name + '.sha256')).write_text(''.join(
        hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.name + '\n' for p in (tgz, zip_path)), encoding='ascii')
    print(json.dumps({'tar_gz': str(tgz), 'zip': str(zip_path), 'files': len(files),
                      'snapshot_commit': git(stage, 'rev-parse', 'HEAD').decode().strip()}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--audit', type=Path, default=ROOT / '.tools' / 'method3-package-audit')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts')
    parser.add_argument('--archive', action='store_true', help='Freeze and archive a previously prepared stage')
    args = parser.parse_args()
    if args.archive:
        archive(args.stage.resolve(), args.output.resolve())
    else:
        prepare(args.stage.resolve(), args.audit.resolve())


if __name__ == '__main__':
    main()

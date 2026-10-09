#!/usr/bin/env python3
"""Fetch the exact public PAL revisions; never update a checkout implicitly."""
import argparse
import pathlib
import subprocess
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('destination', type=pathlib.Path)
    parser.add_argument('--manifest', type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1] / 'dependencies.repos')
    args = parser.parse_args()
    for name, spec in yaml.safe_load(args.manifest.read_text())['repositories'].items():
        target = args.destination / name
        if not target.exists():
            target.mkdir(parents=True)
            subprocess.run(['git', 'init', '-q', str(target)], check=True)
            subprocess.run(['git', '-C', str(target), 'remote', 'add', 'origin', spec['url']], check=True)
        head = subprocess.run(['git', '-C', str(target), 'rev-parse', '--verify', 'HEAD'], capture_output=True)
        if head.returncode:
            subprocess.run(['git', '-C', str(target), 'fetch', '--depth=1', 'origin', spec['version']], check=True)
            subprocess.run(['git', '-C', str(target), 'checkout', '--detach', '-q', 'FETCH_HEAD'], check=True)
        revision = subprocess.check_output(['git', '-C', str(target), 'rev-parse', 'HEAD'], text=True).strip()
        if revision != spec['version']:
            raise RuntimeError(f'{target}: expected {spec["version"]}, found {revision}')


if __name__ == '__main__':
    main()

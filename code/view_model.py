"""Open a project MJCF in MuJoCo Studio; resolve paths independently of cwd."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = Path('models/Venti_skeleton_updated/venti_skeleton_plain.xml')
STUDIO_NAME = 'mujoco_studio.exe'
# Match the native release documented in README and the Python engine version.
DEFAULT_STUDIO = Path('.tools/mujoco-3.17.0-windows-x86_64/bin') / STUDIO_NAME


def project_path(path):
    path = Path(path).expanduser()
    return (path if path.is_absolute() else PROJECT / path).resolve()


def find_studio(explicit_path):
    """Prefer explicit configuration; never silently ignore a broken setting."""
    configured = explicit_path or os.environ.get('MUJOCO_STUDIO_PATH')
    if configured:
        path = project_path(configured)
        if not path.is_file():
            raise FileNotFoundError(f'MuJoCo Studio not found: {path}')
        if path.name.lower() != STUDIO_NAME:
            raise ValueError(f'Expected {STUDIO_NAME}, got: {path.name}')
        return path

    bundled = PROJECT / DEFAULT_STUDIO
    if bundled.is_file():
        return bundled.resolve()
    installed = shutil.which(STUDIO_NAME)
    if installed:
        return Path(installed).resolve()
    raise FileNotFoundError(
        f'MuJoCo Studio not found. Extract MuJoCo 3.17.0 to '
        f'{PROJECT / DEFAULT_STUDIO.parents[1]}, or set MUJOCO_STUDIO_PATH '
        f'or --studio to the full path of {STUDIO_NAME}. See README.md.'
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL,
                        help='Absolute path or a path relative to the project root.')
    parser.add_argument('--check', action='store_true',
                        help='Load the model without opening a viewer.')
    parser.add_argument('--studio', type=Path,
                        help='Path to mujoco_studio.exe (relative to the project '
                             'root or absolute). Overrides MUJOCO_STUDIO_PATH, '
                             'the project .tools installation, and PATH.')
    args = parser.parse_args()
    path = project_path(args.model)
    if not path.is_file():
        parser.error(f'Model not found: {path}')
    if args.check:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(path))
        print(f'{path.name}: bodies={model.nbody - 1}, joints={model.njnt}, '
              f'nq={model.nq}, nv={model.nv}, actuators={model.nu}')
        return 0
    try:
        studio = find_studio(args.studio)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    # Studio uses --model_file, unlike simulate's positional model argument.
    # A list preserves spaces/Unicode in paths and avoids shell interpretation.
    print(f'Starting MuJoCo Studio: {studio}', flush=True)
    print(f'Model: {path}', flush=True)
    try:
        return subprocess.run(
            [str(studio), f'--model_file={path}'], cwd=PROJECT, check=False
        ).returncode
    except OSError as exc:
        parser.error(f'Could not start MuJoCo Studio: {exc}')


if __name__ == '__main__':
    raise SystemExit(main())

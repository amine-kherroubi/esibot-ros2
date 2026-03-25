# Tools

## Project Snapshot Script

`project_snapshot.sh` captures a folder tree and the text contents of files into a single snapshot file. It is tuned for this ROS 2 workspace (ignores common IDE artifacts) but can target any directory on your system.

### Location

The script lives in `src/tools/project_snapshot.sh`. Snapshots are always written to `.snapshots/`. That folder is gitignored, so outputs never get committed.

### Usage

```bash
# from the repo root
./tools/project_snapshot.sh esibot_camera

# from robot_ws/
./src/tools/project_snapshot.sh ./src/esibot_camera
```

### Output Naming

Snapshots follow this pattern: `snapshot_<target_folder>_<YYYYMMDD_HHMMSS>.txt`. Example: `snapshot_esibot_camera_20260325_153012.txt`.

### Options

- `-I, --ignore <pattern>`: Add extra ignore patterns (pipe or comma separated). Example: `-I "*.log|*.bag"`.
- `-q, --quiet`: Suppress non-error output.
- `-h, --help`: Show help.

### Default Ignores

The script skips common workspace and IDE artifacts by default: `.git`, `.snapshots`, `__pycache__`, `*.pyc`, `.vscode`, `.idea`, `*.db3`, `*.mcap`, `metadata.yaml`.

### Notes

- Binary files are ignored automatically.
- The snapshot output always stays inside the repo’s `.snapshots/` directory, even if the target is outside the repo.

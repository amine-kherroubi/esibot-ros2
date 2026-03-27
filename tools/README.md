# Tools

## Project Snapshot Script

`project_snapshot.sh` captures a folder tree and the text contents of files into a single snapshot file. It is tuned for this ROS 2 workspace (ignores common IDE artifacts) but can target any directory on your system.

### Location

The script lives in `src/tools/project_snapshot.sh`.

Snapshots are always written to `.snapshots/` at the git repo root (the folder that contains `.git`).
In this workspace, the git repo root is `robot_ws/src`, not `robot_ws`.
That folder is gitignored, so outputs never get committed.

### Usage

```bash
# from the repo root
./tools/project_snapshot.sh esibot_camera
```

### Output Naming

Snapshots follow this pattern: `snapshot_<target_folder>_<YYYYMMDD_HHMMSS>.txt`. Example: `snapshot_esibot_camera_20260325_153012.txt`.

### Options

- `-I, --ignore <pattern>`: Add extra ignore patterns (pipe or comma separated). These are shell-style globs and can match file names, folder names, extensions, or general patterns. Examples: `-I "build|install|log"`, `-I "*.log|*.bag"`, `-I "data_*|*.tmp"`.
- `-q, --quiet`: Suppress non-error output.
- `-h, --help`: Show help.

### Default Ignores

The script skips common workspace and IDE artifacts by default: `.git`, `.snapshots`, `__pycache__`, `*.pyc`, `.vscode`, `.idea`, `*.db3`, `*.mcap`, `metadata.yaml`.

### Notes

- Binary files are ignored automatically.
- The script requires the `tree` command. If it is missing, the script stops and prints an Ubuntu install example: `sudo apt-get update && sudo apt-get install -y tree`.
- If your target folder is outside the git repo root, the script prints a yellow `[WARN]` line to make the convention explicit. This is expected when you snapshot paths outside `robot_ws/src`.
- The snapshot output always stays inside the repo's `.snapshots/` directory, even if the target is outside the repo.

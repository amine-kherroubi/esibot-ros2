#!/usr/bin/env bash

set -euo pipefail
IFS=$'\n\t'

readonly COLOR_RESET="\033[0m"
readonly COLOR_ERROR="\033[1;31m"
readonly COLOR_INFO="\033[1;34m"
readonly COLOR_SUCCESS="\033[1;32m"
readonly COLOR_BOLD="\033[1m"

QUIET=0
IGNORE_PATTERNS=()
TARGET_REL=""

log_info()    { [[ $QUIET -eq 0 ]] && echo -e "${COLOR_INFO}[INFO]${COLOR_RESET} $1"; }
log_error()   { echo -e "${COLOR_ERROR}[ERROR]${COLOR_RESET} $1" >&2; }
log_success() { [[ $QUIET -eq 0 ]] && echo -e "${COLOR_SUCCESS}[SUCCESS]${COLOR_RESET} $1"; }
die()         { log_error "$1"; exit 1; }

resolve_path() {
    if command -v realpath >/dev/null 2>&1; then
        realpath -m "$1"
    else
        python3 - <<'PY'
import os, sys
print(os.path.realpath(sys.argv[1]))
PY
    fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT=""
if command -v git >/dev/null 2>&1; then
    PROJECT_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"
fi
if [[ -z "$PROJECT_ROOT" ]]; then
    PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
fi
PROJECT_ROOT="$(resolve_path "$PROJECT_ROOT")"

SNAPSHOT_DIR="${PROJECT_ROOT}/.snapshots"

DEFAULT_IGNORES=(
    ".git"
    ".gitignore"
    ".snapshots"
    "__pycache__"
    "*.pyc"
    "*.pyo"
    "*.egg-info"
    ".vscode"
    ".idea"
    "*.db3"
    "*.mcap"
    "metadata.yaml"
    ".DS_Store"
)

usage() {
    cat <<EOF
${COLOR_BOLD}NAME${COLOR_RESET}
    $(basename "$0") - Export directory tree and file contents (project snapshot)

${COLOR_BOLD}SYNOPSIS${COLOR_RESET}
    $(basename "$0") [options] <path>

${COLOR_BOLD}DESCRIPTION${COLOR_RESET}
    Creates a snapshot of the target folder's tree and file contents.
    The target path can be anywhere on your system. Relative paths are
    resolved from your current working directory.

    Output files are always written to:
        $SNAPSHOT_DIR

${COLOR_BOLD}OPTIONS${COLOR_RESET}
    -I, --ignore <ptn>    Additional ignore patterns (pipe/comma separated)
    -q, --quiet           Suppress non-error output
    -h, --help            Display this help and exit

${COLOR_BOLD}EXAMPLES${COLOR_RESET}
    $(basename "$0") esibot_camera
    $(basename "$0") esibot_description/urdf
    $(basename "$0") /tmp
    $(basename "$0") -I "*.log|*.bag" esibot_gazebo
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -I|--ignore)
            [[ -z "${2:-}" || "$2" == -* ]] && die "Option $1 requires an argument."
            IFS='|,' read -r -a new_ignores <<< "$2"
            IGNORE_PATTERNS+=("${new_ignores[@]}")
            shift 2
            ;;
        -q|--quiet)
            QUIET=1
            shift 1
            ;;
        -h|--help)
            usage
            ;;
        --)
            shift
            break
            ;;
        -* )
            die "Unknown option: $1"
            ;;
        * )
            TARGET_REL="$1"
            shift
            break
            ;;
    esac
done

if [[ -z "$TARGET_REL" && $# -gt 0 ]]; then
    TARGET_REL="$1"
    shift
fi

[[ -z "$TARGET_REL" ]] && die "Target path is required. Try '$(basename "$0") --help'."
[[ $# -gt 0 ]] && die "Unexpected extra arguments: $*"

if [[ "$TARGET_REL" = /* ]]; then
    TARGET_ABS="$TARGET_REL"
else
    TARGET_ABS="$(pwd)/$TARGET_REL"
fi

TARGET_ABS="$(resolve_path "$TARGET_ABS")"

[[ ! -d "$TARGET_ABS" ]] && die "Target '$TARGET_ABS' is not a valid directory."

mkdir -p "$SNAPSHOT_DIR" || die "Cannot create snapshot directory: $SNAPSHOT_DIR"

IGNORE_PATTERNS=("${DEFAULT_IGNORES[@]}" "${IGNORE_PATTERNS[@]}")

TARGET_NAME="$(basename "$TARGET_ABS")"
TARGET_SAFE="$(echo "$TARGET_NAME" | tr ' ' '_' | tr -cd 'A-Za-z0-9._-')"
[[ -z "$TARGET_SAFE" ]] && TARGET_SAFE="target"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_FILE="$SNAPSHOT_DIR/snapshot_${TARGET_SAFE}_${TIMESTAMP}.txt"

> "$OUTPUT_FILE" || die "Cannot write to output file: $OUTPUT_FILE"

log_info "Project Root    : $PROJECT_ROOT"
log_info "Target Directory: $TARGET_ABS"
log_info "Output File     : $OUTPUT_FILE"
if [[ "$TARGET_ABS/" != "$PROJECT_ROOT"/* ]]; then
    log_info "Note            : Target is outside project root; output still in $SNAPSHOT_DIR"
fi

REL_ROOT="${TARGET_ABS#"$PROJECT_ROOT"/}"
[[ "$REL_ROOT" == "$TARGET_ABS" ]] && REL_ROOT="$TARGET_ABS"

{
    echo "===== FOLDER TREE OF $REL_ROOT ====="

    if command -v tree >/dev/null 2>&1; then
        if [[ ${#IGNORE_PATTERNS[@]} -gt 0 ]]; then
            ignore_string=$(IFS='|'; echo "${IGNORE_PATTERNS[*]}")
            tree -a -I "$ignore_string" "$TARGET_ABS"
        else
            tree -a "$TARGET_ABS"
        fi
    else
        echo "(Notice: 'tree' command not installed. Visual tree skipped.)"
    fi

    echo -e "\n===== FILE CONTENTS =====\n"
} >> "$OUTPUT_FILE"

find_args=( "$TARGET_ABS" )

if [[ ${#IGNORE_PATTERNS[@]} -gt 0 ]]; then
    find_args+=( "(" )
    for i in "${!IGNORE_PATTERNS[@]}"; do
        [[ $i -gt 0 ]] && find_args+=( "-o" )
        find_args+=( "-name" "${IGNORE_PATTERNS[$i]}" )
    done
    find_args+=( ")" "-prune" "-o" )
fi

find_args+=( "-type" "f" "-print0" )

file_count=0
has_file_cmd=$(command -v file >/dev/null 2>&1 && echo 1 || echo 0)

while IFS= read -r -d $'\0' file; do
    rel_path="${file#"$TARGET_ABS"/}"
    [[ "$rel_path" == "$file" ]] && rel_path="${file#"$TARGET_ABS"}"

    is_text=0
    if [[ -s "$file" ]]; then
        if [[ "$has_file_cmd" == "1" ]]; then
            if ! file -b --mime-encoding "$file" | grep -q "binary"; then
                is_text=1
            fi
        else
            if grep -qI . "$file" 2>/dev/null; then
                is_text=1
            fi
        fi
    fi

    if [[ $is_text -eq 1 ]]; then
        {
            echo "=== FILE START: $rel_path ==="
            cat "$file" 2>/dev/null || echo "(Error reading file)"
            echo ""
            echo "=== FILE END: $rel_path ==="
            echo ""
        } >> "$OUTPUT_FILE"
        file_count=$((file_count + 1))
    fi

done < <(find "${find_args[@]}" | sort -z)

log_success "Processed $file_count text files."
log_success "Snapshot saved to: $OUTPUT_FILE"

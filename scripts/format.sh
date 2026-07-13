#!/usr/bin/env bash
# Format (or check) AM32 application / MCU port C sources with clang-format.
#
# Usage:
#   scripts/format.sh              # rewrite files in place (make format)
#   scripts/format.sh --check      # fail if any file would change (CI)
#   scripts/format.sh --changed    # only files changed vs git merge-base/HEAD
#   scripts/format.sh --check --changed
#
# Style config: repo-root .clang-format
# Excludes vendor HAL (Drivers), CMSIS, generated DroneCAN DSDL, libcanard, etc.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CHECK=0
CHANGED_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK=1 ;;
    --changed|--diff-only) CHANGED_ONLY=1 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      echo "Usage: $0 [--check] [--changed]" >&2
      exit 2
      ;;
  esac
done

if ! command -v clang-format >/dev/null 2>&1; then
  echo "clang-format not found on PATH." >&2
  echo "Install one of:" >&2
  echo "  pip install --user 'clang-format==22.1.5'" >&2
  echo "  sudo apt-get install clang-format" >&2
  exit 1
fi

# Collect sources under application and MCU trees, pruning third-party /
# generated paths. Keep this list in sync with .clang-format-ignore.
collect_all() {
  find Src Inc Mcu \
    \( \
      -path '*/Drivers/*' -o \
      -path '*/CMSIS/*' -o \
      -path '*/dsdl_generated/*' -o \
      -path '*/libcanard/*' -o \
      -path '*/Startup/*' \
    \) -prune -o \
    -type f \( -name '*.c' -o -name '*.h' \) -print \
    | grep -Ev '(^|/)jsmn\.[ch]$' \
    | sort
}

if [[ "$CHANGED_ONLY" -eq 1 ]]; then
  # Prefer merge-base with origin/ark-release or origin/main when present.
  base=""
  for cand in origin/ark-release origin/main; do
    if git rev-parse --verify "$cand" >/dev/null 2>&1; then
      base="$(git merge-base HEAD "$cand" 2>/dev/null || true)"
      [[ -n "$base" ]] && break
    fi
  done
  if [[ -z "$base" ]]; then
    base="HEAD"
  fi
  mapfile -t ALL_FILES < <(collect_all)
  mapfile -t CHANGED < <(
    git diff --name-only --diff-filter=d "$base" -- '*.c' '*.h'
    git diff --name-only --diff-filter=d --cached -- '*.c' '*.h'
  )
  # Unique changed paths that are in our format set.
  declare -A want=()
  for f in "${CHANGED[@]}"; do
    [[ -n "$f" ]] && want["$f"]=1
  done
  FILES=()
  for f in "${ALL_FILES[@]}"; do
    [[ -n "${want[$f]+x}" ]] && FILES+=("$f")
  done
else
  mapfile -t FILES < <(collect_all)
fi

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No files to format."
  exit 0
fi

echo "clang-format $(clang-format --version | head -1)"
echo "Files: ${#FILES[@]}  mode: $([[ $CHECK -eq 1 ]] && echo check || echo fix)"

# Canonical form = clang-format, then drop trailing whitespace (clang-format
# can leave spaces at EOL inside comments). Used by both check and fix.
canonical_form() {
  clang-format "$1" | sed 's/[[:space:]]*$//'
}

if [[ "$CHECK" -eq 1 ]]; then
  bad=()
  for f in "${FILES[@]}"; do
    if ! canonical_form "$f" | cmp -s "$f" -; then
      bad+=("$f")
    fi
  done
  if [[ ${#bad[@]} -ne 0 ]]; then
    echo "Format check failed for ${#bad[@]} file(s):" >&2
    printf '  %s\n' "${bad[@]}" >&2
    echo >&2
    echo 'Fix with:  make format' >&2
    echo 'Or only changed files:  make format_changed' >&2
    exit 1
  fi
  echo "Format checks passed."
  exit 0
fi

# In-place format. Run clang-format twice: some files with awkward comments
# need a second pass to stabilize (clang-format is not always idempotent).
# Then strip trailing whitespace so on-disk form matches --check.
format_batch() {
  printf '%s\0' "${FILES[@]}" | xargs -0 -n 32 -P "$(nproc 2>/dev/null || echo 4)" \
    clang-format -i
}
format_batch
format_batch

# Portable EOL-whitespace strip.
if command -v perl >/dev/null 2>&1; then
  printf '%s\0' "${FILES[@]}" | xargs -0 -n 64 perl -pi -e 's/[ \t]+$//'
else
  for f in "${FILES[@]}"; do
    sed -i.bak 's/[[:space:]]*$//' "$f" && rm -f "$f.bak"
  done
fi

echo "Formatting done (${#FILES[@]} files)."

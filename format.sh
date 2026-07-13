#!/usr/bin/env bash
# Backward-compatible entry point — prefer `make format`.
exec "$(cd "$(dirname "$0")" && pwd)/scripts/format.sh" "$@"

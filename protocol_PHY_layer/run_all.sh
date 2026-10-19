#!/usr/bin/env bash
# Thin wrapper - the real runner is run_all.py (portable: Windows/macOS/Linux).
cd "$(dirname "$0")" && exec python3 run_all.py "$@"

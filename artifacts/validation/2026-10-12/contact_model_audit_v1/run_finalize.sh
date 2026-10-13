#!/usr/bin/env bash
set -euo pipefail
cd /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
base=artifacts/validation/2026-10-12/contact_model_audit_v1
test ! -e "$base/finalize.log"
/usr/bin/python3 -B "$base/finalize.py" > "$base/finalize.log" 2>&1

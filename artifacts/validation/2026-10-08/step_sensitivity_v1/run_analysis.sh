#!/usr/bin/env bash
# 분석 1회. 인자: <새 출력 json 이름> <실행 경로...>. 기존 출력은 덮어쓰지 않는다(open 'x').
set -e
source "$(dirname "$0")/env.sh"
name="${1:?output}"; shift
case "$name" in analysis_[a-z0-9_]*.json) ;; *) exit 2;; esac
python3 -B scripts/analyze_physics_step.py "$@" --output "$base/$name" > "$base/${name%.json}.log" 2>&1

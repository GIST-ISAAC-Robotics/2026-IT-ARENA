#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/grid_cost_r43_v1/checks
PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests python3 -B $A/mutation_probe.py > $A/mutation_probe.log 2>&1
echo "ACTUAL_EXIT_CODE=$?" > $A/mutation_probe.exit

#!/usr/bin/env bash
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
p='install/arena_bringup/share/arena_bringup/launch/g1_follow.launch.py'
if test -L "$p"; then
  printf 'symlink target: '
  readlink "$p"
  exit 0
fi
printf 'regular_file\n'
stat -c '%F %s bytes' "$p"
cmp -s "$p" src/arena_bringup/launch/g1_follow.launch.py
cmp_rc=$?
printf 'same_source_rc=%s\n' "$cmp_rc"
exit 1

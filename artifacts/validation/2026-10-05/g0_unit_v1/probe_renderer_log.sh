#!/usr/bin/env bash
set +e
path="$HOME/.gz/rendering/ogre2.log"
if [ -e "$path" ]; then
  stat -c 'exists=yes mtime=%y path=%n' "$path"
  grep -m 1 'GL_RENDERER' "$path" || true
else
  printf 'exists=no path=%s\n' "$path"
fi

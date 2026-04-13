#!/bin/bash
set -u

INPUT=$(cat)
FILE=$(printf '%s' "$INPUT" | jq -r '.tool_input.file_path // empty')

case "$FILE" in
  */figures/perception_pipeline.tex) ;;
  *) exit 0 ;;
esac

DIR="$(dirname "$FILE")"
cd "$DIR" || exit 0

LOG=/tmp/perception_build.log
if ! pdflatex -interaction=nonstopmode -halt-on-error perception_pipeline.tex > "$LOG" 2>&1; then
  {
    echo "pdflatex failed for perception_pipeline.tex:"
    grep -E '^(! |l\.[0-9])' "$LOG" | head -40
    echo "--- tail ---"
    tail -20 "$LOG"
  } >&2
  exit 2
fi

if ! pdftoppm -r 160 -png -singlefile perception_pipeline.pdf perception_pipeline >> "$LOG" 2>&1; then
  echo "pdftoppm failed - see $LOG" >&2
  exit 2
fi

exit 0

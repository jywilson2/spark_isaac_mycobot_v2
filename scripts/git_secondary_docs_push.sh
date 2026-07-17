#!/usr/bin/env bash
# Secondary docs push — STATUS / CHANGES / last_prompt only.
#
# Why
# ---
# Primary code pushes often leave ``STATUS.md``, ``CHANGES.md``, and
# ``docs/last_prompt.md`` uncommitted because those files are updated *after*
# the push (push status, analysis appends, prompt log). This script commits
# **only** those three paths and pushes once.
#
# Contract (do not break)
# -----------------------
# 1. Update STATUS / CHANGES / last_prompt **before** invoking this script.
# 2. Run this script.
# 3. Do **not** edit those three files again in the same agent turn after the
#    push — that is what creates the infinite "docs lag the push" loop.
#
# Usage (on wip_phase3 or any feature branch):
#   ./scripts/git_secondary_docs_push.sh
#   ./scripts/git_secondary_docs_push.sh "docs: sync STATUS/CHANGES/last_prompt"
#
# Requires: clean intent for only those paths; git + SSH/credentials configured.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

MSG="${1:-docs: sync STATUS/CHANGES/last_prompt after primary push}"

FILES=(
  STATUS.md
  CHANGES.md
  docs/last_prompt.md
)

echo "=== Secondary docs push ==="
echo "Branch: $(git branch --show-current)"
echo "Files:  ${FILES[*]}"

# Stage only the docs triad (ignore other dirty files).
git add -- "${FILES[@]}"

if git diff --cached --quiet; then
  echo "Nothing staged — STATUS/CHANGES/last_prompt already match HEAD."
  echo "Pushing branch tip (no new commit)…"
  git push origin HEAD
  echo "Secondary docs push: no-op commit, push attempted."
  exit 0
fi

git commit -m "$(cat <<EOF
${MSG}

Secondary docs-only commit. Do not amend STATUS/CHANGES/last_prompt again
in the same turn after this push (see scripts/git_secondary_docs_push.sh).
EOF
)"

git push origin HEAD
echo "Secondary docs push COMPLETE."
echo "STOP: do not edit STATUS.md / CHANGES.md / docs/last_prompt.md again this turn."

#!/usr/bin/env bash
# Restore host/container file ownership after editing the repo as root in Docker/Cursor.
#
# Run on the DGX Spark HOST (recommended):
#   cd /path/to/spark_isaac_mycobot_demo
#   ./scripts/fix_repo_permissions.sh
#
# Or from inside the Isaac ROS container as root (one-time repair):
#   sudo ./scripts/fix_repo_permissions.sh
#
# The script maps ownership to HOST_USER_UID/HOST_USER_GID when set (Isaac ROS sets
# these from the host user at container start). Otherwise it uses the invoking user.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET_UID="${HOST_USER_UID:-$(id -u)}"
TARGET_GID="${HOST_USER_GID:-$(id -g)}"

if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    exec sudo env \
      HOST_USER_UID="${TARGET_UID}" \
      HOST_USER_GID="${TARGET_GID}" \
      "${BASH_SOURCE[0]}" "$@"
  fi
  echo "Run as root or with sudo to change ownership." >&2
  exit 1
fi

echo "Repairing ownership for ${REPO_ROOT} -> uid=${TARGET_UID} gid=${TARGET_GID}"

find "${REPO_ROOT}" -name '*.lock' -delete 2>/dev/null || true
chown -R "${TARGET_UID}:${TARGET_GID}" "${REPO_ROOT}"

# Submodule git metadata can retain root ownership even after the parent repo is fixed.
if [[ -d "${REPO_ROOT}/.git/modules" ]]; then
  chown -R "${TARGET_UID}:${TARGET_GID}" "${REPO_ROOT}/.git/modules"
fi

echo "Done. Verify on the host:"
echo "  ls -la ${REPO_ROOT}/.git"
echo "  git -C ${REPO_ROOT} status"

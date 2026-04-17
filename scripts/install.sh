#!/usr/bin/env bash
# Build (and optionally install) dds-tf-tree-tool.
# CMake dependency: message_manager (Botcrew layout often uses /opt/botcrew).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BOTCREW_PREFIX="${BOTCREW_PREFIX:-/opt/botcrew}"
BUILD_DIR="${BUILD_DIR:-build}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
DO_APT=0
DO_INSTALL=0

usage() {
  echo "Build (and optionally install) dds-tf-tree-tool (needs message_manager for CMake)."
  echo "Usage: $(basename "$0") [options]"
  echo "  --apt-deps       Install Debian/Ubuntu build tools + graphviz + xdot (sudo apt)."
  echo "  --install        Run cmake --install after build (prefix: INSTALL_PREFIX or /usr/local)."
  echo "  --build-dir DIR  CMake build directory (default: build)."
  echo "  -h, --help       Show this help."
  echo ""
  echo "Environment:"
  echo "  BOTCREW_PREFIX=/opt/botcrew   Prepended to CMAKE_PREFIX_PATH when that directory exists."
  echo "  CMAKE_PREFIX_PATH             Additional prefixes for find_package(message_manager)."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apt-deps) DO_APT=1 ;;
    --install) DO_INSTALL=1 ;;
    --build-dir)
      BUILD_DIR="${2:?}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if [[ "$DO_APT" -eq 1 ]]; then
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "This script only supports --apt-deps on apt-based systems." >&2
    exit 1
  fi
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    graphviz \
    xdot
fi

if [[ -d "$BOTCREW_PREFIX" ]]; then
  export CMAKE_PREFIX_PATH="${BOTCREW_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi

cmake -S . -B "$BUILD_DIR"
cmake --build "$BUILD_DIR"

if [[ "$DO_INSTALL" -eq 1 ]]; then
  cmake --install "$BUILD_DIR" --prefix "$INSTALL_PREFIX"
fi

echo "Built: $ROOT/$BUILD_DIR/tf_tree_viewer"
if [[ "$DO_INSTALL" -eq 1 ]]; then
  echo "Installed under: $INSTALL_PREFIX/bin"
fi

#!/usr/bin/env bash
# Build the dev image and run a shell or a named task inside it.
#   ./docker/run.sh         interactive shell
#   ./docker/run.sh test    build everything and run unit tests
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="degenbench-dev:latest"

docker build -q -t "${IMAGE}" "${ROOT}/docker" >/dev/null

TTY_FLAGS=()
[ -t 0 ] && TTY_FLAGS=(-it)

run() {
  docker run --rm "${TTY_FLAGS[@]}" \
    -u "$(id -u):$(id -g)" \
    -v "${ROOT}:/ws" -w /ws \
    "${IMAGE}" bash -c "$1"
}

case "${1:-shell}" in
  shell) run "bash" ;;
  test)
    run "cmake -S labeler -B build/labeler -G Ninja -DCMAKE_BUILD_TYPE=Release \
         && cmake --build build/labeler \
         && ctest --test-dir build/labeler --output-on-failure \
         && cd leaderboard && python3 -m pytest -q -p no:cacheprovider"
    ;;
  *) echo "unknown task: $1" >&2; exit 2 ;;
esac

#!/usr/bin/env bash
# Build the dev image and run a shell or a named task inside it.
#   ./docker/run.sh         interactive shell
#   ./docker/run.sh test    build everything and run unit tests
#   ./docker/run.sh build   (re)build the image only
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="degenbench-dev:latest"

build_image() {
  echo ">> building ${IMAGE} (cached after the first run)" >&2
  local log
  log="$(mktemp)"
  if ! docker build -t "${IMAGE}" \
      --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
      "${ROOT}/docker" >"${log}" 2>&1; then
    cat "${log}" >&2
    rm -f "${log}"
    echo ">> image build failed" >&2
    exit 1
  fi
  rm -f "${log}"
}

TTY_FLAGS=()
[ -t 0 ] && [ -t 1 ] && TTY_FLAGS=(-it)

run() {
  docker run --rm --init "${TTY_FLAGS[@]}" \
    --hostname degenbench \
    -v "${ROOT}:/ws" -w /ws \
    "${IMAGE}" bash -c "$1"
}

TEST_CMD="cmake -S labeler -B build/labeler -G Ninja -DCMAKE_BUILD_TYPE=Release \
  && cmake --build build/labeler \
  && ctest --test-dir build/labeler --output-on-failure \
  && cd leaderboard && python3 -m pytest -q -p no:cacheprovider"

case "${1:-shell}" in
  build) build_image ;;
  shell) build_image; run "exec bash" ;;
  test)  build_image; run "${TEST_CMD}" ;;
  *) echo "usage: $0 [shell|test|build]" >&2; exit 2 ;;
esac

#!/usr/bin/env bash
# Prepare one sequence end to end, inside the dev container:
# frames -> world-to-map alignment -> labels -> detector features.
#
#   ./docker/run.sh shell
#   OUT=data/derived/floor01 BAGS="a.bag b.bag" TOPIC=/velodyne_packets \
#     GT=data/subt_mrs/gt/floor01_traj MAP=data/subt_mrs/map/floor.pcd \
#     [EXTRINSIC=handeye.yaml] [IMU_TOPIC=/imu/data] [THREADS=12] \
#     scripts/prepare_sequence.sh [frames|align|label|features|all]
#
# Decisions that depend on the data are not automated and are reported per
# sequence: whether the ground-truth clock needs shifting (IMU_TOPIC runs
# time_offset), and which extrinsic to use (identity unless EXTRINSIC is given;
# hand-eye via scan_pairs/register_pairs/handeye when it is not identity).
set -euo pipefail

: "${OUT:?}" "${BAGS:?}" "${TOPIC:?}" "${GT:?}" "${MAP:?}"
THREADS="${THREADS:-12}"
CROP="${CROP:-40}"
BUILD="${BUILD:-build/all}"
STEP="${1:-all}"
mkdir -p "${OUT}"
export PYTHONPATH="/ws/converters:/ws/leaderboard${PYTHONPATH:+:${PYTHONPATH}}"

frames() {
  local gt="${GT}"
  if [ -n "${IMU_TOPIC:-}" ]; then
    python3 -m degenbench_converters.time_offset --bag ${BAGS} --imu-topic "${IMU_TOPIC}" \
      --gt "${GT}" --out-gt "${OUT}/gt_bagtime.txt" | tee "${OUT}/time_offset.txt"
    gt="${OUT}/gt_bagtime.txt"
  fi
  local extr=()
  [ -n "${EXTRINSIC:-}" ] && extr=(--extrinsic "${EXTRINSIC}")
  python3 -m degenbench_converters.extract_frames --bag ${BAGS} --topic "${TOPIC}" \
    --gt "${gt}" --out "${OUT}/frames" --max-gt-gap 0.3 "${extr[@]}"
}

align() {
  "${BUILD}/labeler/map_to_bin" "${MAP}" "${OUT}/map.bin"
  python3 -m degenbench_converters.map_align --frames "${OUT}/frames" --map "${OUT}/map.bin" \
    --out-cloud "${OUT}/accum.bin" --out-candidates "${OUT}/candidates.txt"
  "${BUILD}/labeler/refine_alignment" --map "${MAP}" --cloud "${OUT}/accum.bin" \
    --candidates "${OUT}/candidates.txt" --out "${OUT}/T_map_world.txt" \
    --report "${OUT}/align_report.yaml"
  cat "${OUT}/align_report.yaml"
}

label() {
  "${BUILD}/labeler/label_sequence" --frames "${OUT}/frames" --map "${MAP}" \
    --T-map-world "${OUT}/T_map_world.txt" --out "${OUT}/labels.csv" \
    --threads "${THREADS}" --crop-radius "${CROP}" --robustness 1
}

features() {
  "${BUILD}/detectors/detector_features" --frames "${OUT}/frames" --map "${MAP}" \
    --T-map-world "${OUT}/T_map_world.txt" --out "${OUT}/features.csv" \
    --threads "${THREADS}" --crop-radius "${CROP}"
}

case "${STEP}" in
  frames) frames ;;
  align) align ;;
  label) label ;;
  features) features ;;
  all) frames; align; label; features ;;
  *) echo "unknown step ${STEP}" >&2; exit 2 ;;
esac

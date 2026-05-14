#!/usr/bin/env bash
# 将 OSRF gazebo_models 中 pose_course.world 用到的模型下载到 course_pkg/gazebo_models
# 用法: bash $(rospack find course_pkg)/scripts/fetch_gazebo_models.sh
set -euo pipefail
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${PKG_DIR}/gazebo_models"
TMP="${TMPDIR:-/tmp}/gazebo_models_course_pkg_$$"
REPO="https://github.com/osrf/gazebo_models.git"
MODELS=(
  bookshelf cafe_table table cabinet table_marble
  wood_cube_5cm wood_cube_10cm
  coke_can bowl cardboard_box
)

echo "[fetch_gazebo_models] 目标目录: ${DEST}"
mkdir -p "${DEST}"
git clone --depth 1 "${REPO}" "${TMP}"
for m in "${MODELS[@]}"; do
  if [[ ! -d "${TMP}/${m}" ]]; then
    echo "[fetch_gazebo_models] 警告: 仓库中无模型目录 ${m}" >&2
    continue
  fi
  rm -rf "${DEST}/${m}"
  cp -a "${TMP}/${m}" "${DEST}/${m}"
  echo "[fetch_gazebo_models] 已复制 ${m}"
done
rm -rf "${TMP}"
echo "[fetch_gazebo_models] 完成。请在 launch 中设置 GAZEBO_MODEL_PATH 包含: ${DEST}"

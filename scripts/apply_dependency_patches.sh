#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

apply_patch_at_revision() {
  local repository="$1"
  local expected_revision="$2"
  local patch_file="$3"
  local actual_revision

  actual_revision="$(git -C "${project_root}/${repository}" rev-parse HEAD)"
  if [[ "${actual_revision}" != "${expected_revision}" ]]; then
    echo "${repository} is at ${actual_revision}; expected ${expected_revision}." >&2
    exit 1
  fi

  git -C "${project_root}/${repository}" apply --check "${project_root}/${patch_file}"
  git -C "${project_root}/${repository}" apply "${project_root}/${patch_file}"
  echo "Applied ${patch_file} to ${repository}."
}

apply_patch_at_revision \
  src/ros2_so_arm \
  e166df9d51f43b24da9b99047c6c51c306bda74f \
  patches/ros2_so_arm.patch

apply_patch_at_revision \
  src/mujoco_ros2_control \
  e6a6160c471a48609fc3f1f7508d7f571e8fa18e \
  patches/mujoco_ros2_control.patch

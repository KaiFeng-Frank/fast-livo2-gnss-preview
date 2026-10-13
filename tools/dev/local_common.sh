# SPDX-License-Identifier: BSD-3-Clause
# Shared layout for the local build/test/tool scripts.
# Sourced, not executed. Remote wrappers require explicit FL2G_REMOTE and FL2G_ROOT.
#
#   FL2G_ROOT          required writable project root mounted at /r.
#   FL2G_BUILD_ROOT    cmake/catkin trees. Default: FL2G_ROOT when that filesystem can hold a symlink,
#                      otherwise $HOME/fastlivo2_gnss_build (override with FL2G_BUILD_FALLBACK).
#                      Docker bind-mounts those trees over /r/wt/<name>/{build_core,ws/build,ws/devel,...}
#                      so container paths stay identical to the remote layout.
#   FL2G_IMAGE         default fastlivo2-gnss:noetic (must already be loaded; this script never pulls).
#   FL2G_JOBS          default 4, and never above 4: this host shares its CPUs with other work.
#   FL2G_CPUS          docker --cpus quota (default 4). This is a CFS quota, not --cpuset-cpus
#                      (cpuset would change nproc and the compile-command gate).
#   FL2G_DOCKER_USER   default $(id -u):$(id -g). Never sudo.
#   FL2G_VIKIT_SRC     pristine rpg_vikit (xuankuzcr@6c886c8). Default: the first of
#                      $FL2G_ROOT/src/rpg_vikit and $FL2G_BUILD_ROOT/src/rpg_vikit that exists.
#   FL2G_ROOT_FLOOR_GIB  when the build root is on /, refuse to start below this many GiB free (default 6,
#                      so a build that stays small leaves the root filesystem at or above 5 GiB).

: "${FL2G_TAG:=local}"

fl2g_die() { echo "$FL2G_TAG: $*" >&2; exit 2; }

fl2g_supports_symlinks() {
  local d=$1/.__fl2g_ln_$$
  ln -s . "$d" 2>/dev/null || return 1
  rm -f "$d"
  return 0
}

fl2g_init_layout() {
  : "${FL2G_ROOT:?set FL2G_ROOT to a writable data/build directory}"
  mkdir -p "$FL2G_ROOT"
  FL2G_ROOT=$(realpath -e "$FL2G_ROOT")

  if [ -n "${FL2G_BUILD_ROOT:-}" ]; then
    mkdir -p "$FL2G_BUILD_ROOT"
    FL2G_BUILD_ROOT=$(realpath -e "$FL2G_BUILD_ROOT")
  elif fl2g_supports_symlinks "$FL2G_ROOT"; then
    FL2G_BUILD_ROOT=$FL2G_ROOT
  else
    FL2G_BUILD_ROOT=${FL2G_BUILD_FALLBACK:-$HOME/fastlivo2_gnss_build}
    mkdir -p "$FL2G_BUILD_ROOT"
    FL2G_BUILD_ROOT=$(realpath -e "$FL2G_BUILD_ROOT")
    echo "[$FL2G_TAG] $FL2G_ROOT has no symlinks; cmake/catkin trees go to $FL2G_BUILD_ROOT" >&2
  fi

  FL2G_IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
  [[ "$FL2G_IMAGE" =~ ^[A-Za-z0-9_./:-]+$ ]] || fl2g_die "bad image name"
  FL2G_JOBS=${FL2G_JOBS:-4}
  [[ "$FL2G_JOBS" =~ ^[1-9][0-9]*$ ]] || fl2g_die "FL2G_JOBS must be a positive integer"
  [ "$FL2G_JOBS" -le 4 ] || fl2g_die "FL2G_JOBS=$FL2G_JOBS is above 4 (this host shares its CPUs)"
  FL2G_CPUS=${FL2G_CPUS:-4}
  [[ "$FL2G_CPUS" =~ ^[1-4]([.][0-9]+)?$ ]] || fl2g_die "FL2G_CPUS must be in 1..4"
  FL2G_DOCKER_USER=${FL2G_DOCKER_USER:-$(id -u):$(id -g)}
  [[ "$FL2G_DOCKER_USER" =~ ^[0-9]+:[0-9]+$ ]] || fl2g_die "FL2G_DOCKER_USER must be UID:GID"
  FL2G_ROOT_FLOOR_GIB=${FL2G_ROOT_FLOOR_GIB:-6}
  if [ -z "${FL2G_VIKIT_SRC:-}" ]; then
    if [ -d "$FL2G_ROOT/src/rpg_vikit" ]; then
      FL2G_VIKIT_SRC=$FL2G_ROOT/src/rpg_vikit
    else
      FL2G_VIKIT_SRC=$FL2G_BUILD_ROOT/src/rpg_vikit
    fi
  fi
  export FL2G_ROOT FL2G_BUILD_ROOT FL2G_IMAGE FL2G_JOBS FL2G_CPUS FL2G_DOCKER_USER FL2G_VIKIT_SRC
  mkdir -p "$FL2G_ROOT/wt" "$FL2G_BUILD_ROOT/wt" "$FL2G_ROOT/data" "$FL2G_ROOT/audit" "$FL2G_ROOT/runs"
}

fl2g_check_root_floor() {
  local dev_build dev_root free_gib
  dev_build=$(df -P "$FL2G_BUILD_ROOT" | awk 'NR==2{print $1}')
  dev_root=$(df -P / | awk 'NR==2{print $1}')
  free_gib=$(df -P / | awk 'NR==2{printf "%.2f", $4/1024/1024}')
  echo "[$FL2G_TAG] disk: / has ${free_gib} GiB free; build root $FL2G_BUILD_ROOT; project root $FL2G_ROOT"
  if [ "$dev_build" = "$dev_root" ]; then
    # awk exits 0 when free < floor; `&&` keeps set -e from aborting on the complementary case
    if awk -v f="$free_gib" -v n="$FL2G_ROOT_FLOOR_GIB" 'BEGIN{exit !(f+0 < n+0)}'; then
      fl2g_die "/ has ${free_gib} GiB free, below the ${FL2G_ROOT_FLOOR_GIB} GiB floor (leave >= 5 GiB after the build). Move FL2G_BUILD_ROOT or free space on /"
    fi
  fi
}

fl2g_require_image() {
  command -v docker >/dev/null || fl2g_die "docker is not on PATH"
  docker image inspect "$FL2G_IMAGE" >/dev/null 2>&1 || \
    fl2g_die "image $FL2G_IMAGE is not loaded. This script does not pull or delete images. Load fastlivo2-gnss:noetic as in docs/dev/LOCAL_HOST.md, or set FL2G_IMAGE to an image already present"
}

fl2g_check_name() {
  [[ "$1" =~ ^[A-Za-z0-9_-]+$ ]] || fl2g_die "worktree name '$1' must match ^[A-Za-z0-9_-]+\$"
}

fl2g_check_tree() {
  local local_tree=$1
  [ -f "$local_tree/package.xml" ] && grep -q '<name>fast_livo</name>' "$local_tree/package.xml" || \
    fl2g_die "$local_tree is not the fast_livo checkout"
  [ -f "$local_tree/gnss/CMakeLists.txt" ] || fl2g_die "$local_tree/gnss/CMakeLists.txt missing"
}

# Bind-mount the symlink-capable build trees over the /r paths the remote layout uses.
# Mount points on FL2G_ROOT are empty directories when that filesystem cannot store the build.
fl2g_prepare_mounts() {
  local name=$1 rel host
  FL2G_MOUNTS=(-v "$FL2G_ROOT:/r")
  [ "$FL2G_BUILD_ROOT" = "$FL2G_ROOT" ] && return 0
  for rel in \
      "wt/$name/build_core" "wt/$name/build_core_vg" \
      "wt/$name/ws/build" "wt/$name/ws/devel" \
      "wt/$name/ws/build_vg" "wt/$name/ws/devel_vg"
  do
    host="$FL2G_BUILD_ROOT/$rel"
    mkdir -p "$host" "$FL2G_ROOT/$rel"
    FL2G_MOUNTS+=(-v "$host:/r/$rel")
  done
}

fl2g_sync_tree() {
  local local_tree=$1 name=$2 dest
  dest="$FL2G_ROOT/wt/$name/ws/src/FAST-LIVO2"
  mkdir -p "$dest" "$FL2G_ROOT/wt/$name/home" "$dest/Log"
  echo "[$FL2G_TAG $(date +%T)] rsync $local_tree -> $dest"
  rsync -a --delete \
    --exclude '.git' --exclude '.claude' --exclude 'Log/' --exclude 'pics/' --exclude 'Supplementary/' \
    --exclude '*.pcd' --exclude '*.bag' --exclude '*.pdf' \
    --exclude 'build/' --exclude 'build_core/' --exclude 'devel/' --exclude '__pycache__/' \
    "$local_tree/" "$dest/"
}

# Host-side flock. The caller must not be a subshell: fd 9 stays open for the rest of the process,
# which is what releases the lock (the remote scripts' "client still alive" check has no sshd here).
fl2g_lock() {
  local lock=$FL2G_ROOT/.build.lock
  exec 9>>"$lock"
  echo "[$FL2G_TAG $(date +%T)] waiting for $lock"
  flock -x 9
  echo "[$FL2G_TAG $(date +%T)] lock held"
}

# shell-quote each argument for the container's bash
fl2g_q() { if [ $# -gt 0 ]; then printf '%q ' "$@"; fi; }

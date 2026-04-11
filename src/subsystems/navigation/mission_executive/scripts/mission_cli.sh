#!/usr/bin/env bash
# mission_cli.sh — Interactive CLI for the MissionExecutive node
#
# Usage:
#   ./mission_cli.sh [command] [args...]
#
# Commands:
#   status                          Print current nav status once
#   watch                           Stream nav status (Ctrl-C to stop)
#   abort                           Abort the current mission
#   teleop on|off                   Enable / disable teleop mode
#
#   nav gps   <lat> <lon>  [tol]   Navigate to GPS coords (GNSS_ONLY target)
#   nav meter <x>   <y>    [tol]   Navigate to ENU metre coords
#   nav aruco <lat> <lon>  [tol]   Navigate + spiral for ArUco post
#   nav object <lat> <lon> [tol]   Navigate + spiral for OBJECT detection
#
#   set-target gps   <id> <lat> <lon> [type] [tol]   Register GPS target
#   set-target meter <id> <x>   <y>   [type] [tol]   Register ENU target
#   nav-by-id <id>                  Navigate to a pre-registered target id
#
#   menu                            Interactive menu (default if no args)
#
# target_type: 0=GNSS_ONLY  1=ARUCO_POST  2=OBJECT  3=LOCAL
# tol: arrival radius in metres (default 3.0)
#
# For ARUCO_POST and OBJECT targets the node automatically enters
# SPIRAL_COVERAGE after arrival; the action completes when a detection
# is received OR the spiral times out.  The live status display shows
# the state transition so you can see when this happens.
#
# Ctrl-C during a nav command calls ~/abort before exiting.

set -euo pipefail

# ── Constants ─────────────────────────────────────────────────────────────────

NODE_NS="/mission_executive"
STATUS_TOPIC="/nav_status"
STATUS_TYPE="msgs/msg/NavStatus"

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
BLD='\033[1m'
DIM='\033[2m'
RST='\033[0m'

# ── Helpers ───────────────────────────────────────────────────────────────────

info()  { echo -e "${GRN}[✓]${RST} $*"; }
warn()  { echo -e "${YLW}[!]${RST} $*"; }
error() { echo -e "${RED}[✗]${RST} $*" >&2; }
title() { echo -e "\n${BLD}$*${RST}"; }

require_ros() {
  if ! command -v ros2 &>/dev/null; then
    error "ros2 not found. Source your ROS 2 workspace first."
    exit 1
  fi
}

target_type_name() {
  case "$1" in
    0) echo "GNSS_ONLY" ;;
    1) echo "ARUCO_POST" ;;
    2) echo "OBJECT" ;;
    3) echo "LOCAL" ;;
    *) echo "UNKNOWN($1)" ;;
  esac
}

# Return one field from the next /nav_status message, or "?" on timeout.
_status_field() {
  local field="$1"
  timeout 2 ros2 topic echo --once --field "$field" "$STATUS_TOPIC" 2>/dev/null \
    | tr -d ' \n' || echo "?"
}

# ── cmd_status / cmd_watch ────────────────────────────────────────────────────

cmd_status() {
  title "Nav Status"
  local raw
  raw=$(timeout 2 ros2 topic echo --once "$STATUS_TOPIC" "$STATUS_TYPE" 2>/dev/null) || {
    warn "No message received — is mission_executive running?"
    return
  }
  local state dist xte speed active
  state=$(echo  "$raw" | awk '/^state:/{print $2}')
  active=$(echo "$raw" | awk '/^active_target_id:/{print $2}')
  dist=$(echo   "$raw" | awk '/^distance_to_goal_m:/{print $2}')
  xte=$(echo    "$raw" | awk '/^cross_track_error_m:/{print $2}')
  speed=$(echo  "$raw" | awk '/^robot_speed_mps:/{print $2}')

  printf "  %-22s %s\n"  "State:"         "${BLD}${state}${RST}"
  printf "  %-22s %s\n"  "Active target:"  "${active:-(none)}"
  printf "  %-22s %s m\n" "Dist to goal:"  "${dist:--}"
  printf "  %-22s %s m\n" "Cross-track err:" "${xte:--}"
  printf "  %-22s %s rad/s\n" "Angular vel:" "${speed:--}"
}

cmd_watch() {
  title "Watching $STATUS_TOPIC  (Ctrl-C to stop)"
  # Pretty-print each message as a single status line
  ros2 topic echo "$STATUS_TOPIC" "$STATUS_TYPE" | \
  awk '
    /^state:/              { state  = $2 }
    /^active_target_id:/   { tgt    = $2 }
    /^distance_to_goal_m:/ { dist   = $2 }
    /^cross_track_error_m:/{ xte    = $2 }
    /^---/                 {
      printf "state=%-20s  tgt=%-12s  dist=%6s m  xte=%6s m\n",
             state, tgt, dist, xte
      state=""; tgt=""; dist=""; xte=""
    }
  '
}

# ── cmd_abort / cmd_teleop ────────────────────────────────────────────────────

cmd_abort() {
  title "Abort Mission"
  ros2 service call "${NODE_NS}/abort" std_srvs/srv/Trigger '{}'
}

cmd_teleop() {
  local mode="${1:-}"
  case "$mode" in
    on|1|true)
      title "Teleop ON"
      ros2 service call "${NODE_NS}/teleop" std_srvs/srv/SetBool '{data: true}'
      ;;
    off|0|false)
      title "Teleop OFF"
      ros2 service call "${NODE_NS}/teleop" std_srvs/srv/SetBool '{data: false}'
      ;;
    *)
      error "Usage: teleop on|off"
      exit 1
      ;;
  esac
}

# ── Live-status display while an action runs ──────────────────────────────────
#
# Runs the action goal in the background, streams /nav_status to the terminal
# as a single updating line, and waits for the action to finish.
# Ctrl-C calls ~/abort and cancels the background job.

_run_nav_with_status() {
  local goal_yaml="$1"
  local target_type="$2"
  local result_file
  result_file=$(mktemp /tmp/mission_result.XXXXXX)

  # Warn about spiral phases for detection targets
  if [[ "$target_type" == "1" || "$target_type" == "2" ]]; then
    echo -e "  ${CYN}Note: After arrival the node will enter SPIRAL_COVERAGE${RST}"
    echo -e "  ${CYN}      and complete when a detection is received or it times out.${RST}"
    echo
  fi

  # --feedback keeps the process alive until the action completes;
  # without it ros2 action send_goal exits as soon as the goal is accepted.
  ros2 action send_goal \
    "${NODE_NS}/navigate_to_target" \
    msgs/action/NavigateToTarget \
    "$goal_yaml" \
    --feedback \
    > "$result_file" 2>&1 &
  local action_pid=$!

  # Ctrl-C: abort mission then clean up
  local interrupted=false
  trap '
    interrupted=true
    echo
    warn "Interrupted — calling abort..."
    ros2 service call "${NODE_NS}/abort" std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
    kill "$action_pid" 2>/dev/null || true
  ' INT

  echo -e "  ${DIM}Waiting for first status message...${RST}"

  local prev_state=""
  local start_s=$SECONDS

  while kill -0 "$action_pid" 2>/dev/null; do
    local raw
    raw=$(timeout 1.2 ros2 topic echo --once "$STATUS_TOPIC" "$STATUS_TYPE" 2>/dev/null) || true

    if [[ -n "$raw" ]]; then
      local state dist xte elapsed
      state=$(echo "$raw" | awk '/^state:/{print $2}')
      dist=$(echo  "$raw" | awk '/^distance_to_goal_m:/{print $2}')
      xte=$(echo   "$raw" | awk '/^cross_track_error_m:/{print $2}')
      elapsed=$(( SECONDS - start_s ))

      # Format to 2 decimal places if numeric, else show "-"
      local dist_s="-" xte_s="-"
      [[ -n "$dist" ]] && dist_s=$(printf "%.2f" "$dist" 2>/dev/null) || true
      [[ -n "$xte"  ]] && xte_s=$(printf "%.2f" "$xte"  2>/dev/null) || true

      # Highlight state transitions on their own line
      if [[ "$state" != "$prev_state" && -n "$state" ]]; then
        printf "\n  ${YLW}→ State: ${BLD}%s${RST}\n" "$state"
        prev_state="$state"
      fi

      printf "\r  [%3ds]  dist=%8s m  xte=%8s m  " \
        "$elapsed" "$dist_s" "$xte_s"
    fi

    sleep 0.5
  done

  # Restore default INT handler
  trap - INT

  # Print the action result
  echo -e "\n"
  if [[ "$interrupted" == false ]]; then
    echo -e "${BLD}── Action Result ──────────────────────────────${RST}"
    # Show just the result block (last few lines after "Result:")
    grep -A5 "Result:" "$result_file" 2>/dev/null || cat "$result_file"
    echo -e "${BLD}───────────────────────────────────────────────${RST}"
  fi

  rm -f "$result_file"
}

# ── _send_nav_goal (builds YAML and dispatches) ───────────────────────────────
# $1: goal_type  (0=GPS 1=METER)
# $2: target_id  (may be empty string for inline)
# $3: lat or x_m
# $4: lon or y_m
# $5: target_type (0-3)
# $6: tolerance_m
# $7: is_return   (true|false)

_send_nav_goal() {
  local goal_type="$1" target_id="$2" a="$3" b="$4"
  local target_type="${5:-0}" tolerance="${6:-3.0}" is_return="${7:-false}"

  local lat=0.0 lon=0.0 x=0.0 y=0.0
  [[ "$goal_type" == "0" ]] && { lat="$a"; lon="$b"; } || { x="$a"; y="$b"; }

  title "navigate_to_target"
  [[ -n "$target_id" ]] && echo "  target_id   : $target_id" || echo "  target_id   : (inline)"
  echo "  goal_type   : $([ "$goal_type" = "0" ] && echo GPS || echo METER)"
  [[ "$goal_type" == "0" ]] \
    && echo "  lat / lon   : $lat / $lon" \
    || echo "  x_m / y_m  : $x / $y"
  echo "  target_type : $(target_type_name "$target_type")"
  echo "  tolerance_m : $tolerance"
  echo "  is_return   : $is_return"
  echo

  local yaml="{target_id: \"$target_id\", lat: $lat, lon: $lon, x_m: $x, y_m: $y, goal_type: $goal_type, target_type: $target_type, tolerance_m: $tolerance, is_return: $is_return}"

  _run_nav_with_status "$yaml" "$target_type"
}

# ── cmd_nav ───────────────────────────────────────────────────────────────────

cmd_nav() {
  local coord_type="${1:-}"
  case "$coord_type" in
    gps|GPS)
      local lat="${2:?lat required}" lon="${3:?lon required}"
      local tol="${4:-3.0}"
      _send_nav_goal 0 "" "$lat" "$lon" 0 "$tol" false
      ;;
    meter|METER|m)
      local x="${2:?x_m required}" y="${3:?y_m required}"
      local tol="${4:-3.0}"
      _send_nav_goal 1 "" "$x" "$y" 0 "$tol" false
      ;;
    aruco|ARUCO)
      # ARUCO_POST (type=1) — navigate then auto-spiral, done on detection or timeout
      local lat="${2:?lat required}" lon="${3:?lon required}"
      local tol="${4:-3.0}"
      _send_nav_goal 0 "" "$lat" "$lon" 1 "$tol" false
      ;;
    object|OBJECT|obj)
      # OBJECT (type=2) — same spiral flow as aruco but for YOLO detections
      local lat="${2:?lat required}" lon="${3:?lon required}"
      local tol="${4:-3.0}"
      _send_nav_goal 0 "" "$lat" "$lon" 2 "$tol" false
      ;;
    *)
      error "Usage:"
      error "  nav gps    <lat> <lon>  [tol]   — GNSS_ONLY target"
      error "  nav meter  <x>   <y>    [tol]   — ENU metre target"
      error "  nav aruco  <lat> <lon>  [tol]   — navigate + spiral (ArUco)"
      error "  nav object <lat> <lon>  [tol]   — navigate + spiral (YOLO object)"
      exit 1
      ;;
  esac
}

# Navigate to a pre-registered target by ID
cmd_nav_by_id() {
  local id="${1:?target_id required}"
  local is_return="${2:-false}"
  title "navigate_to_target  (id=$id  is_return=$is_return)"

  local yaml="{target_id: \"$id\", lat: 0.0, lon: 0.0, x_m: 0.0, y_m: 0.0, goal_type: 0, target_type: 0, tolerance_m: 3.0, is_return: $is_return}"

  # target_type unknown here — pass 0 (spiral check uses what's in the registry)
  _run_nav_with_status "$yaml" 0
}

# ── cmd_set_target ────────────────────────────────────────────────────────────

_call_set_target() {
  local goal_type="$1" target_id="$2" a="$3" b="$4"
  local target_type="${5:-0}" tolerance="${6:-3.0}"

  local lat=0.0 lon=0.0 x=0.0 y=0.0
  [[ "$goal_type" == "0" ]] && { lat="$a"; lon="$b"; } || { x="$a"; y="$b"; }

  title "set_target  (id=$target_id  type=$(target_type_name "$target_type"))"
  ros2 service call "${NODE_NS}/set_target" msgs/srv/SetTarget \
    "{target_id: '$target_id', lat: $lat, lon: $lon, x_m: $x, y_m: $y, \
goal_type: $goal_type, target_type: $target_type, tolerance_m: $tolerance}"
}

cmd_set_target() {
  local coord_type="${1:-}"
  case "$coord_type" in
    gps|GPS)
      local id="${2:?id}" lat="${3:?lat}" lon="${4:?lon}"
      local ttype="${5:-0}" tol="${6:-3.0}"
      _call_set_target 0 "$id" "$lat" "$lon" "$ttype" "$tol"
      ;;
    meter|METER|m)
      local id="${2:?id}" x="${3:?x}" y="${4:?y}"
      local ttype="${5:-0}" tol="${6:-3.0}"
      _call_set_target 1 "$id" "$x" "$y" "$ttype" "$tol"
      ;;
    *)
      error "Usage: set-target gps   <id> <lat> <lon> [type 0-3] [tol]"
      error "       set-target meter <id> <x>   <y>   [type 0-3] [tol]"
      exit 1
      ;;
  esac
}

# ── Interactive menu ──────────────────────────────────────────────────────────

_menu_nav_goal() {
  echo
  echo "Navigation type:"
  echo "  1) GPS point  (GNSS_ONLY)"
  echo "  2) GPS point  (ArUco post  — will spiral after arrival)"
  echo "  3) GPS point  (YOLO object — will spiral after arrival)"
  echo "  4) ENU metres (GNSS_ONLY)"
  echo "  5) By registered target ID"
  read -rp "Choice: " ct

  case "$ct" in
    5)
      read -rp "Target ID: " tid
      read -rp "Is return? [y/N]: " ret
      [[ "$ret" =~ ^[Yy]$ ]] && cmd_nav_by_id "$tid" true || cmd_nav_by_id "$tid" false
      return
      ;;
  esac

  local goal_type=0 target_type=0 a b tol
  case "$ct" in
    1) target_type=0 ;;
    2) target_type=1 ;;
    3) target_type=2 ;;
    4) goal_type=1   ;;
  esac

  if [[ "$goal_type" == "0" ]]; then
    read -rp "Latitude:  " a
    read -rp "Longitude: " b
  else
    read -rp "x_m: " a
    read -rp "y_m: " b
  fi

  read -rp "Arrival tolerance in metres [3.0]: " tol
  tol="${tol:-3.0}"

  _send_nav_goal "$goal_type" "" "$a" "$b" "$target_type" "$tol" false
}

_menu_set_target() {
  echo
  echo "Coordinate type:"
  echo "  1) GPS (lat/lon)    2) ENU metres (x/y)"
  read -rp "Choice: " ct
  read -rp "Target ID: " tid

  local goal_type=0 a b ttype tol
  if [[ "$ct" == "1" ]]; then
    read -rp "Latitude:  " a; read -rp "Longitude: " b; goal_type=0
  else
    read -rp "x_m: " a; read -rp "y_m: " b; goal_type=1
  fi

  echo "Target type:  0=GNSS_ONLY  1=ARUCO_POST  2=OBJECT  3=LOCAL"
  read -rp "Target type [0]: " ttype; ttype="${ttype:-0}"
  read -rp "Arrival tolerance [3.0]: " tol; tol="${tol:-3.0}"

  _call_set_target "$goal_type" "$tid" "$a" "$b" "$ttype" "$tol"
}

cmd_menu() {
  while true; do
    echo
    echo -e "${BLD}╔══ Mission Executive CLI ════════════════════╗${RST}"
    echo -e "${BLD}║${RST}  1) Status (snapshot)                       ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  2) Watch status (stream)                   ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  3) Navigate (GPS / meter / aruco / object) ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  4) Register target (set_target service)    ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  5) Abort mission                           ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  6) Teleop ON                               ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  7) Teleop OFF                              ${BLD}║${RST}"
    echo -e "${BLD}║${RST}  q) Quit                                    ${BLD}║${RST}"
    echo -e "${BLD}╚═════════════════════════════════════════════╝${RST}"
    read -rp "Choice: " choice

    case "$choice" in
      1) cmd_status ;;
      2) cmd_watch ;;
      3) _menu_nav_goal ;;
      4) _menu_set_target ;;
      5) cmd_abort ;;
      6) cmd_teleop on ;;
      7) cmd_teleop off ;;
      q|Q) info "Bye."; exit 0 ;;
      *) warn "Unknown option '$choice'" ;;
    esac
  done
}

# ── Entry point ───────────────────────────────────────────────────────────────

require_ros

CMD="${1:-menu}"
shift || true

case "$CMD" in
  status)      cmd_status ;;
  watch)       cmd_watch ;;
  abort)       cmd_abort ;;
  teleop)      cmd_teleop "$@" ;;
  nav)         cmd_nav "$@" ;;
  nav-by-id)   cmd_nav_by_id "$@" ;;
  set-target)  cmd_set_target "$@" ;;
  menu)        cmd_menu ;;
  -h|--help|help) sed -n '3,33p' "$0" ;;
  *)
    error "Unknown command: $CMD"
    echo "Run '$0 help' for usage."
    exit 1
    ;;
esac

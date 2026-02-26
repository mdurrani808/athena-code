#!/usr/bin/env bash
# Send a GPS navigation goal to the navigate_to_gps action server.
#
# Usage:
#   ./send_gps_goal.sh <latitude> <longitude> [tolerance_m]
#
# Examples:
#   ./send_gps_goal.sh 38.423911 -110.784905
#   ./send_gps_goal.sh 38.423911 -110.784905 3.0

set -e

usage() {
    echo "Usage: $0 <latitude> <longitude> [tolerance_m]"
    echo ""
    echo "  latitude    Target latitude in decimal degrees"
    echo "  longitude   Target longitude in decimal degrees"
    echo "  tolerance_m Position tolerance in metres (default: 0.0 = use server default)"
    exit 1
}

if [[ $# -lt 2 ]]; then
    usage
fi

LAT="$1"
LON="$2"
TOLERANCE="${3:-0.0}"

echo "=========================================="
echo " GPS Goal"
echo "=========================================="
echo "  Latitude  : $LAT"
echo "  Longitude : $LON"
echo "  Tolerance : $TOLERANCE m"
echo "=========================================="
echo ""

ros2 action send_goal \
    --feedback \
    /navigate_to_gps \
    msgs/action/NavigateToGPS \
    "{latitude: $LAT, longitude: $LON, position_tolerance: $TOLERANCE}"

# Published by the local planner (vector_field_planner) when its tentacle FSM
# enters the REPLAN state — i.e. forward arcs are blocked and reverse arcs
# cannot open a new vantage on their own.
#
# Future consumers (mission_executive, waypoint manager, web GUI) use this to
# insert a detour waypoint, mark the original waypoint as failed after N
# attempts, or surface "rover is stuck" to the operator.
#
# Lifecycle:
#   - On REPLAN entry:  publish once with stuck=true.
#   - While stuck:      heartbeat at ~1 Hz with stuck=true (so a late
#                       subscriber catches up after a restart).
#   - On REPLAN exit:   publish once with stuck=false.
#
# The "suggested_detour" is the local planner's best guess at a pose that
# would escape the current obstacle pocket — derived from the highest-
# clearance tentacle (forward OR reverse, whichever was clearer). Consumers
# may use it directly or override with their own policy.

std_msgs/Header header              # stamp at publish time; frame_id = map frame
bool stuck                          # true while in REPLAN, false on exit
geometry_msgs/PoseStamped stuck_pose         # rover pose when REPLAN entered
geometry_msgs/PoseStamped suggested_detour   # local planner's escape suggestion
float64 best_forward_clearance_m    # clearance along best forward tentacle (m); 0 if none clear
float64 best_reverse_clearance_m    # clearance along best reverse tentacle (m); 0 if none clear

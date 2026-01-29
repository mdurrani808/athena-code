!#/bin/bash
SESSION="rover-dev"
WORKSPACE="$HOME/ROS/ros_dev_ws"

tmux new-session -d -s $SESSION -c $WORKSPACE

# Pane 0: Neovim for code editing
tmux rename-window -t $SESSION:0 'Code'
tmux send-keys -t $SESSION:0 'nvim .' C-m


# Pane 1: Build monitor
tmux new-window

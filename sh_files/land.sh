#!/bin/zsh
SCRIPT_DIR="${0:A:h}"
source "${SCRIPT_DIR:h}/devel/setup.zsh" || exit 1
# The mission manager first stops trajectories, then retries LAND until px4ctrl
# can accept it in AUTO_HOVER. A single raw LAND during CMD_CTRL can be rejected.
rostopic pub -1 /mission/land std_msgs/Empty '{}'

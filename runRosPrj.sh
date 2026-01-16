#!/bin/bash

echo "[Launcher the ROS2 Services]"

# Environment files
envCmd="/opt/ros/humble/setup.bash"
envCmdLocal="./install/local_setup.sh"

# Parse package_name from setup.py
get_package_name() {
	grep 'package_name\s*=' setup.py | head -1 | sed 's/.*"\([^"]*\)".*/\1/'
}

# Parse console_scripts from setup.py
parse_console_scripts() {
	grep -A 100 '"console_scripts"' setup.py |
		grep -E '^\s*"[^"]+\s*=' |
		sed 's/.*"\([^"]*\)\s*=\s*\(.*\)".*/\1|\2/' |
		sed 's/,$//' |
		sed 's/^\s*|\s*$//g' |
		sed 's/\s*|\s*/|/g'
}

# Get package name and command list
package=$(get_package_name)
commands=$(parse_console_scripts)

if [ -z "$package" ]; then
	echo "Could not parse package_name from setup.py"
	exit 1
fi

if [ -z "$commands" ]; then
	echo "No console_scripts found in setup.py"
	exit 1
fi

# Display command list
echo "Available commands:"
i=1
declare -a cmd_array
while IFS='|' read -r cmd desc; do
	cmd_array[$i]="$cmd"
	echo "$i) $cmd = $desc"
	((i++))
done <<<"$commands"

# Get user selection
read -p "Select command (1-$((i - 1))): " selection

if [[ ! "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -ge "$i" ]; then
	echo "Invalid selection"
	exit 1
fi

selected_cmd="${cmd_array[$selection]}"

# Source environment files
echo "source: $envCmd"
source "$envCmd"
echo "source: $envCmdLocal"
source ./install/local_setup.sh

# Run selected command
echo "[$selected_cmd]"
echo "Running: ros2 run $package $selected_cmd"
ros2
ros2 run "$package" "$selected_cmd"

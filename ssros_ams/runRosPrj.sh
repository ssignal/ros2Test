#!/bin/bash

echo "[Launcher the ROS2 Services]"

# Environment files
envCmd="/opt/ros/humble/setup.bash"
envCmdLocal="./install/local_setup.sh"

# Parse package_name from package.xml
get_package_name() {
	grep '<name>' package.xml | head -1 | sed 's/.*<name>\([^<]*\)<\/name>.*/\1/'
}

# Get list of executables from the install directory
get_commands() {
    local package="$1"
    if [ -d "install/$package/lib/$package" ]; then
        ls "install/$package/lib/$package"
    fi
}

# Get package name and command list
package=$(get_package_name)
commands=$(get_commands "$package")

if [ -z "$package" ]; then
	echo "Could not parse package_name from package.xml"
	exit 1
fi

if [ -z "$commands" ]; then
	echo "No executables found in install/$package/lib/$package. Please build the project first."
	exit 1
fi

# Display command list
echo "Available commands:"
i=1
declare -a cmd_array
for cmd in $commands; do
	cmd_array[$i]="$cmd"
	echo "$i) $cmd"
	((i++))
done

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
source "$envCmdLocal"

# Run selected command
echo "[$selected_cmd]"
echo "Running: ros2 run $package $selected_cmd"
ros2 run "$package" "$selected_cmd"
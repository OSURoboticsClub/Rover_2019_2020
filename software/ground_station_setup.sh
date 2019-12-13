#!/usr/bin/env bash
# As a point of reference, the environment layout should be as follows
# /home/$user/Github/Rover_2019_2020 for the OSURC github repo
# /home/$user/catkin_workspace for the user's catkin catkin_workspace
# By keeping this consistent across all development machines, it will make it
# easier to keep track of things

# Which folders should be symbolically_linked?
folders_to_link=(
    ground_station
    rover_control
    nimbro_topic_transport
    rover_main
    rover_camera
    rover_status
    rover_arm
    rover_science
)

# Print heading
echo "Setting up ROS packages for ground_station."

# Get the catkin_workspace directory
catkin_workspace_dir="catkin_workspace"
catkin_workspace_path="$HOME/$catkin_workspace_dir"
catkin_src_path="$catkin_workspace_path/src"

# Get the rover software directory
github_rover_repo_dir="Github/Rover_2019_2020"
github_rover_packages_path="$HOME/$github_rover_repo_dir/software/ros_packages"

# Remove existing symbolic links if necessary
symlinked_folders=$(find $catkin_src_path -maxdepth 1 -type l)
if [ -z $symlinked_folders ]; then
    echo "No symlinks to remove from catkin_workspace. Skipping."
else
    echo "Removing existing symlinks in catkin_workspace."
    rm $symlinked_folders
fi

# Make the new symbolic link connections
echo "Making new symlinks."
for folder in ${folders_to_link[@]}; do
    ln -s "$github_rover_packages_path/$folder" "$catkin_src_path/."
    echo "Adding symlink for $folder."
done

# catkin_make so the new pacakges are available and re-source bash
cd "$catkin_workspace_path"
catkin_make

source ~/.bashrc

exit 0

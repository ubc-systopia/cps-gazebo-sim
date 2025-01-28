#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir/.."
default_dest_dir="multi_arm_lab_sim_description/models/"

# Initialize default values
new_name=""
dest_dir="$default_dest_dir"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n)
            new_name="$2"
            shift 2
            ;;
        -d)
            dest_dir="$2"
            shift 2
            ;;
        *)
            zip_path="$1"
            shift
            ;;
    esac
done

# Check if zip_path is provided
if [[ -z "$zip_path" ]]; then
    echo "Usage: $0 [-n new_name] [-d destination_directory] <zip_path>"
    exit 1
fi

# Check if the ZIP file exists
if [[ ! -f "$zip_path" ]]; then
    echo "File not found. Please provide a valid file path."
    exit 1
fi

# Check if the destination directory exists, create it if it doesn't
if [[ ! -d "$dest_dir" ]]; then
    echo "Destination directory not found. Creating $dest_dir..."
    mkdir -p "$dest_dir"
fi

# Default archive name based on zip file name (without extension)
archive_name=$(basename "$zip_path" .zip)

# Unzip the file directly into the destination directory
unzip "$zip_path" -d "$dest_dir/$archive_name"

# Use the extracted folder name if no new name is provided
if [[ -z "$new_name" ]]; then
    new_name="$archive_name"
fi

# Move the extracted folder to the destination with the new name
mv "$dest_dir/$archive_name" "$dest_dir/$new_name"
echo "The file has been extracted and renamed to $dest_dir/$new_name."

# Search for a .dae file within the renamed folder
dae_file=$(find "$dest_dir/$new_name" -type f -name "*.dae" | head -n 1)

if [[ -n "$dae_file" ]]; then
    mv "$dae_file" "$dest_dir/$new_name/$new_name.dae"
    dae_file="$dest_dir/$new_name/$new_name.dae" # Update variable
    echo "Found and renamed the .dae file to $new_name.dae."
else
    echo "No .dae file found in $dest_dir/$new_name."
fi

# model.config
model_config_path="$dest_dir/$new_name/model.config"
cat <<EOF > "$model_config_path"
<?xml version="1.0"?>
<model>
    <name>$new_name</name>
    <version>1.0</version>
    <sdf version="1.8">model.sdf</sdf>
    <author>
        <name>Your Name</name>
        <email>your.email@example.com</email>
    </author>
    <description>TODO: A description of the $new_name model for Gazebo.</description>
</model>
EOF

echo "Created model.config for Gazebo in $dest_dir/$new_name."

# model.sdf
model_sdf_path="$dest_dir/$new_name/model.sdf"
cat <<EOF > "$model_sdf_path"
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.8">
  <model name="$new_name">
    <static>true</static>
    <link name="link">
      <collision name="object_collision">
        <geometry>
          <mesh>
            <uri>$new_name.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="object_visual">
        <geometry>
          <mesh>
            <uri>$new_name.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
    <plugin filename="ignition-gazebo-touchplugin-system"
                name="ignition::gazebo::systems::TouchPlugin">
      <time>0.001</time>
      <enabled>true</enabled>
    </plugin>
  </model>
</sdf>
EOF

echo "Created model.sdf for Gazebo in $dest_dir/$new_name."

# Resave the .dae file using Blender to ensure compatibility with Gazebo
blender -b -P "$script_dir/blender_resave.py" -- "$dae_file" "$dae_file"

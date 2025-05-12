# Adding Models to the Simulator

## Uploading DAE Meshes from Polycam

**Requirements**:
- Blender 2.8 or higher
- `bpy` Python library (install with `pip install bpy`)

Tested with:
- `bpy==4.0.0`
- Blender 3.0.1

Some COLLADA (`.dae`) files, such as those downloaded directly from Polycam, may not be fully compatible with Gazebo. To ensure compatibility, we resave these files using Blender. This step is included in the `load_model.sh` script, but you can also do it manually by running the [`blender_resave.py`](../../scripts/blender_resave.py) script.

To resave a model, run the following command:

```bash
blender -b -P blender_resave.py -- /path/to/input_file.dae /path/to/output_file.dae
```

### Steps

1. **Export Mesh from Polycam**: Download the 3D model from Polycam, then export it as a `MESH` in DAE format. This will create a zip folder containing the mesh and texture files.

2. **Load the Model**: Use the [`load_model.sh`](../../scripts/load_model.sh) script to load the model into the description package. Example:

    ```bash
    cd cps-gazebo-sim
    ./scripts/load_model.sh [-n new_name] [-d destination_directory] <zip_path>
    ```

    - `-n new_name`: Optionally rename the model.
    - `-d destination_directory`: Optionally specify a different directory to load the model into. If not provided, the model will be loaded into `multi_arm_lab_sim_description/models/` by default.

---

## Creating and Viewing Worlds

1. **Modify the Configuration File**: Loaded models are available for inclusion in a world, which is an environment that contains a number of models. Worlds are defined in the `multi_arm_lab_sim_gazebo` package and can be configured in the [`worlds_config.json`](../../multi_arm_lab_sim_gazebo/worlds/worlds_config.json) file.

    For example, if you loaded models named `lab` and `ur3e`, you can include them in the world configuration as follows:

    ```json
    {
        "worlds": [
            {
                "name": "lab",
                "models": [
                    {
                        "source_model": "lab",
                        "pose": "0 0 0 0 0 -2.5"
                    },
                    {
                        "source_model": "ur3e",
                        "name": "robot",
                        "pose": "0.25 0 1.5 0 0 0"
                    }
                ]
            }
        ]
    }
    ```

    See [`worlds.schema.json`](../../multi_arm_lab_sim_gazebo/worlds/worlds.schema.json) for details on how to customize the setup of your worlds.

2. **Create the Worlds**: To generate the worlds that you configured, run the [`create_worlds.py`](../../scripts/create_worlds.py) script. This will produce `.sdf` files in the `multi_arm_lab_sim_gazebo/worlds` directory:

    ```bash
    python scripts/create_worlds.py
    ```

3. **Launch the World**: Once the worlds are created, they can be used in [simulator launch files](../../multi_arm_lab_sim_bringup/launch/). To view the statically defined world after building, run:

    ```bash
    ign gazebo install/multi_arm_lab_sim_gazebo/share/multi_arm_lab_sim_gazebo/worlds/<world_name>.sdf
    ```

"""
Generates SDF (Simulation Description Format) world files for Gazebo from a JSON configuration.

### Steps:
1. Loads and validates `worlds_config.json` against `worlds.schema.json`.
2. Processes the template SDF file, inserting models and their poses from the configuration.
3. Generates an SDF file for each world defined in the configuration.

### Prerequisites:
- `worlds.schema.json`: JSON schema for world configuration.
- `worlds_config.json`: JSON configuration defining worlds and models.
- `template.sdf`: Template file for world configuration.
"""

import json
from jsonschema import exceptions, validate

WORLDS_DIR = "multi_arm_lab_sim_gazebo/worlds"

with open(f"{WORLDS_DIR}/worlds.schema.json", "r", encoding="utf-8") as schema_file:
    schema = json.load(schema_file)

with open(f"{WORLDS_DIR}/worlds_config.json", "r", encoding="utf-8") as f:
    worlds_data = json.load(f)

try:
    validate(instance=worlds_data, schema=schema)
except exceptions.ValidationError as err:
    print(f"Invalid JSON: {err.message}")

with open(f"{WORLDS_DIR}/template.sdf", "r", encoding="utf-8") as f:
    template_sdf = f.read()

worlds = worlds_data.get("worlds", [])
for world in worlds:
    world_name = world["name"]
    models = world.get("models", [])

    model_entries = []
    for model in models:
        source_model = model["source_model"]
        model_name = model.get("name", source_model)
        pose = model.get("pose", "0 0 0 0 0 0")
        MODEL_ENTRY = f"""
    <model name="{model_name}">
    <self_collide>true</self_collide>
    <include merge="true">
        <uri>package://multi_arm_lab_sim_description/models/{source_model}</uri>
        <pose>{pose}</pose>
    </include>
    </model>
    """
        model_entries.append(MODEL_ENTRY)
    MODELS_BLOCK = "\n".join(model_entries)

    # Replace the placeholders in the template
    final_sdf = template_sdf.replace("<!-- models -->", MODELS_BLOCK)
    final_sdf = final_sdf.replace("<!-- world_name -->", world_name)

    with open(f"{WORLDS_DIR}/{world_name}.sdf", "w", encoding="utf-8") as f:
        f.write(final_sdf)

    print(f"SDF world generated: '{WORLDS_DIR}/{world_name}.sdf'")

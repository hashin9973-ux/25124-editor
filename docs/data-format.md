# Data format: version 1

The editor writes two independent JSON documents. Both use SI units and stable string IDs. Runtime entity IDs and engine components are intentionally excluded.

## organism.json

Top-level fields:

- format: organism
- schema_version: integer schema version
- id, name: stable organism identity and display name
- units: length, mass, and time base units
- parts: body graph nodes
- joints: physical connections between parts
- actuators: capabilities attached to joints

Parts describe geometry, local/world transform data, and physical properties. Body physics is tagged as either rigid or soft. Soft-body fields are reserved in the data contract; they do not imply the current editor has a working deformable-body solver.

Joints reference part IDs and store each attachment point in the corresponding part's local coordinates. Hinge angles are measured in radians; linear joint ranges are measured in meters. Distance joints store a positive `rest_length_m` between body-local anchors. The field is optional for other joint types.

## motion.json

Top-level fields:

- format: organism_motion
- schema_version: integer schema version
- organism_id: ID of the organism this motion document targets
- behaviors: named controller definitions and goals

Motion goals refer to actuator IDs rather than engine entities. This allows one organism definition to be paired with multiple behavior documents.

The current prototype exports empty behavior and actuator arrays until actuator and controller authoring is implemented.

## Compatibility and validation

Readers must reject unknown major schema versions, duplicate IDs, dangling joint/actuator references, non-finite values, and invalid physical ranges. Schema evolution should use explicit migrations rather than silently reinterpreting old fields.

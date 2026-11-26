# Data format: version 1

The application writes two independent JSON documents. Both use SI units and stable string IDs. Runtime Bevy entity IDs, ECS components, and Avian structs are deliberately excluded.

## Shared conventions

- Length: meters.
- Mass: kilograms.
- Time: seconds.
- Force: newtons.
- Torque: newton-meters.
- Angular joint limits and angular actuator amplitudes: radians.
- Linear joint limits and linear actuator amplitudes: meters.

The schema version is explicit. Readers should reject unsupported versions, duplicate IDs, dangling references, non-finite values, and invalid physical ranges.

## organism.json

Top-level fields:

- `format`: `organism`
- `schema_version`: integer schema version
- `id`, `name`: stable identity and display name
- `units`: length, mass, and time base units
- `parts`: body graph nodes
- `joints`: physical connections between parts
- `actuators`: movement capabilities attached to joints

### Body parts

A body part stores a stable ID, name, geometry, transform, and tagged physics properties.

Primitive geometry currently includes:

- Sphere: `radius_m`
- Cuboid: `half_extents_m` (half-size on each axis)
- Capsule: `radius_m` and `half_height_m` (half the straight center segment; hemispheres are additional)

Rigid properties include:

- `mass_kg`: explicit physical mass used by the runtime.
- `density_kg_m3`: material density assumption.
- `friction`, `restitution`, `linear_damping`, and `angular_damping`.

For editor-created primitives, estimated mass is (m = \rho V). The geometry volume is computed from the primitive dimensions and multiplied by the chosen density. The estimated mass and density are both serialized. Imported models retain their explicit mass; density is not silently substituted for mass.

Soft-body properties (density, elastic modulus, damping ratio, and resolution) are reserved in the schema. The current runtime does not implement soft-body simulation.

### Joints

Joints reference parent and child part IDs. Attachment points are stored separately in each part's local coordinates.

Current joint kinds are:

- `fixed`
- `ball`
- `hinge`
- `slider`
- `distance`

Hinge limits are angular and measured in radians. Slider and distance values are measured in meters. Distance joints require a positive `rest_length_m`; other joint kinds omit that field.

### Actuators

An actuator references a joint ID and defines a physical capability, not an animation track.

Current kinds:

- `angular_motor`: uses `max_torque_nm`.
- `linear_motor`: uses `max_force_n`.
- `muscle_tendon`: reserved for the future deformable/muscle system.

The current editor's powered hinge and powered slider tools create matching Avian motors and actuator records.

## motion.json

Top-level fields:

- `format`: `organism_motion`
- `schema_version`: integer schema version
- `organism_id`: the organism this document controls
- `behaviors`: named controller definitions and goals

A behavior references actuator IDs rather than engine entities. The same organism can be paired with several independent motion documents.

The current oscillator goal has:

- `actuator_id`
- `frequency_hz`
- `amplitude` (radians for angular motors, meters for linear motors)

The oscillator computes a target signal. It does not directly write body transforms; the Avian joint motor applies bounded force/torque and the physics solver determines the actual movement.

## Compatibility and validation

The engine-independent model validates IDs, references, finite values, physical ranges, and the requirement for distance-joint rest lengths. Schema evolution should use explicit migrations rather than silently reinterpreting old fields.

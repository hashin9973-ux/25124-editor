# 25124 Editor

A mouse-first, physics-first organism construction studio built with Rust, Bevy, and Avian.

The editor models **structure**, **physical capabilities**, and **behavior** separately. Movement is intended to emerge from physical simulation, actuators, and feedback controllers—not authored transform keyframes.

## Current prototype

The first vertical slice provides:

- A Bevy 3D viewport with an Avian physics world and a static ground plane.
- Left-click placement of rigid spheres, cuboids, and capsules on the ground plane.
- Fixed, ball-and-socket, hinge, slider, and distance connections between clicked body parts.
- Powered hinges and sliders with torque-/force-limited Avian motors plus data-driven oscillation controllers.
- Joint links visualized in the viewport.
- Physical mass, friction, restitution, and damping applied to the Avian bodies.
- Separate organism.json and motion.json export through the Save button.
- An engine-independent organism model crate with validation.
- A serialization crate with load/save functions and a sample data format.

This is an early construction prototype, not yet a complete creature editor. It currently creates spheres and passive hinges. Mesh sculpting, soft-body simulation, actuator authoring, controller execution, undo/redo, and a full inspector are not implemented yet.

## Run

Requires a Rust toolchain compatible with the 2024 edition and the Bevy 0.20 / Avian 0.8 dependency line.

```bash
cargo run -p organism-editor
```

Use the left panel:

1. **Place spheres**: left-click the ground to add rigid parts.
2. **Connect hinge**: click one part, then another part. The editor creates a hinge between them.
3. **Save JSON**: writes exports/organism.json and exports/motion.json relative to the working directory.

The current editor remains an early construction prototype. It does not yet include freeform sculpting, soft-body simulation, actuator authoring, a runtime behavior controller, camera orbit controls, or undo/redo.

## Workspace

- organism-core: engine-independent organism, joints, actuator, and behavior data types.
- organism-serialization: validation-aware JSON loading and saving.
- organism-editor: Bevy UI, viewport interactions, and Avian runtime adapter.

## Format boundary

The JSON contracts do not contain Bevy Entity values, ECS component types, or Avian-specific structs. The editor translates these stable IDs and physical descriptions into runtime entities and constraints.

The two files have distinct responsibilities:

- organism.json: geometry, transforms, physical properties, joints, and actuator capabilities.
- motion.json: behavior controllers, goals, and references to actuator IDs.

Soft-body properties are represented in the model format, but the current runtime does not simulate them. The editor must not present a soft-body definition as simulated until a deformable-body backend exists.

## Validation

```bash
cargo fmt --all -- --check
cargo test --workspace
cargo check --workspace
```

The GitHub Actions workflow runs formatting and workspace tests on pushes and pull requests.

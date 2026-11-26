# 25124 Editor

A mouse-first, physics-first organism construction studio built with Rust, Bevy, and Avian.

The editor separates three concerns:

- **Structure** — body parts, geometry, physical materials, joints, and actuators.
- **Capability** — the movement a body can produce through bounded forces and torques.
- **Behavior** — data-driven controllers and goals that drive actuators using feedback or time-dependent signals.

The intended output is a physical organism definition plus a separate behavior definition. Runtime movement comes from the physics solver, not authored transform keyframes.

## Current prototype

The current vertical slice provides:

- A Bevy 3D viewport with Avian rigid-body simulation and a static ground plane.
- Mouse orbit (right-drag) and zoom (wheel); left-click is reserved for construction.
- Primitive placement for rigid spheres, cuboids, and capsules.
- Fixed, ball-and-socket, hinge, slider, and distance joints.
- Powered hinge and powered slider tools using Avian angular/linear motors with torque/force limits.
- An initial oscillator controller that updates motor targets while Avian computes the resulting movement.
- Joint-link visualization.
- Primitive volume and material-density-based mass estimation.
- Per-body friction, restitution, and velocity damping.
- Separate `organism.json` and `motion.json` export.
- An engine-independent model crate with ID/reference/physical validation.
- Validation-aware JSON load and save functions.

This is a construction and physics proof-of-concept, not a complete creature editor. The primitive dimensions, density, friction, damping, motor frequency, amplitude, and limits currently use assisted defaults. The inspector, adjustable joint frames and limits, direct actuator editing, contact-driven gait controllers, freeform mesh modeling, undo/redo, and soft-body runtime are not implemented yet.

## Run

Requires Rust with the 2024 edition and dependencies from the Bevy 0.20 / Avian 0.8 line.

```bash
cargo run -p organism-editor
```

Use the left tool panel:

1. Choose **Add sphere**, **Add cuboid**, or **Add capsule**, then left-click the ground to place a part.
2. Choose a connection tool, then click two parts to connect them.
3. Choose a powered hinge or powered slider to create both an Avian motor and a matching actuator/oscillator entry in the data model.
4. Right-drag to orbit and use the wheel to zoom.
5. Choose **Save organism + motion JSON** to write `exports/organism.json` and `exports/motion.json`.

The construction UI is intentionally mouse-operated. Current part sizes and physical parameters are defaults, not an inspector yet.

## Workspace

- `crates/organism-core`: engine-independent organism graph, geometry, physical properties, joints, actuators, behaviors, and validation.
- `crates/serialization`: validation-aware JSON load/save for the two independent documents.
- `crates/editor`: Bevy user interface, viewport tools, and Avian runtime adapter.

## Format boundary

The JSON contracts do not contain Bevy `Entity` values, ECS components, or Avian-specific structs. The editor maps stable model IDs and physical descriptions into runtime entities and constraints.

- `organism.json` stores body parts, primitive geometry, transforms, rigid/soft body properties, joints, and actuator capability limits.
- `motion.json` stores behaviors, controller kinds, goals, and actuator references.

Primitive mass estimates use (m = \rho V), with the editor currently using a default density of 1,000 kg/m³. The resulting mass is stored explicitly in the organism file; density is retained as the material assumption. Imported documents preserve their explicit mass.

Soft-body properties exist in the data contract for future use, but the current runtime does not simulate deformable bodies. The application must not imply otherwise.

## Local validation

Run these commands on the development machine after pulling the branch:

```bash
cargo fmt --all -- --check
cargo test --workspace
cargo check --workspace
```

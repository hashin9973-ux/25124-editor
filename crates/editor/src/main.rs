mod actuator_editing;
mod camera_controls;
mod joint_editing;
mod part_editing;
mod runtime_geometry;

use actuator_editing::{ActuatorControllerEdit, apply_actuator_controller_edit};
use avian3d::prelude::*;
use bevy::camera_controller::pan_orbit_camera::prelude::*;
use bevy::picking::hover::Hovered;
use bevy::prelude::*;
use bevy::text::EditableText;
use bevy::ui_widgets::{Activate, Button, TextInput, observe};
use bevy::window::PrimaryWindow;
use camera_controls::CameraControlsPlugin;
use joint_editing::{JointEdit, apply_joint_edit};
use organism_core::{
    Actuator, ActuatorKind, Behavior, BodyPart, BodyPhysics, ControllerKind, Geometry, Joint,
    JointKind, MotionDefinition, MotionGoal, Organism, RigidBodyProperties, Transform3,
    Vec3 as DataVec3,
};
use organism_serialization::save_project;
use part_editing::{PartEdit, apply_part_edit};
use runtime_geometry::build_runtime_geometry;
use std::path::Path;

const EXPORT_DIRECTORY: &str = "exports";
const PART_FRICTION: f32 = 0.75;
const PART_RESTITUTION: f32 = 0.0;
const PART_LINEAR_DAMPING: f32 = 0.03;
const PART_ANGULAR_DAMPING: f32 = 0.05;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            MeshPickingPlugin,
            DefaultPanOrbitCameraPlugins,
            CameraControlsPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.025, 0.03, 0.045)))
        .init_resource::<DraftProject>()
        .init_resource::<EditorTool>()
        .init_resource::<PlacementSettings>()
        .init_resource::<PartSelection>()
        .init_resource::<PartInspectorState>()
        .init_resource::<JointSelection>()
        .init_resource::<JointInspectorState>()
        .init_resource::<ActuatorInspectorState>()
        .add_systems(Startup, (setup_scene, setup_editor_panel))
        .add_systems(
            Update,
            (
                edit_from_viewport,
                draw_joint_links,
                drive_actuator_behaviors,
                update_placement_settings_label,
                update_part_inspector,
                update_joint_inspector,
                update_actuator_inspector,
                draw_selected_part_outline,
            ),
        )
        .run();
}

#[derive(Resource)]
struct PlacementSettings {
    size_scale: f32,
    density_kg_m3: f32,
}

impl Default for PlacementSettings {
    fn default() -> Self {
        Self {
            size_scale: 1.0,
            density_kg_m3: 1000.0,
        }
    }
}

#[derive(Component)]
struct PlacementSettingsLabel;

#[derive(Component)]
struct PartInspectorLabel;

#[derive(Component)]
struct InspectorStatusLabel;

#[derive(Component)]
struct JointInspectorLabel;

#[derive(Component)]
struct JointInspectorStatusLabel;

#[derive(Component)]
struct ActuatorInspectorLabel;

#[derive(Component)]
struct ActuatorInspectorStatusLabel;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InspectorField {
    Name,
    DimensionA,
    DimensionB,
    DimensionC,
    Density,
    Friction,
    Restitution,
    LinearDamping,
    AngularDamping,
}

#[derive(Component, Clone, Copy)]
struct InspectorFieldInput(InspectorField);

#[derive(Component, Clone, Copy)]
struct InspectorFieldLabel(InspectorField);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JointInspectorField {
    Name,
    ParentAnchorX,
    ParentAnchorY,
    ParentAnchorZ,
    ChildAnchorX,
    ChildAnchorY,
    ChildAnchorZ,
    RestLength,
}

#[derive(Component, Clone, Copy)]
struct JointInspectorFieldInput(JointInspectorField);

#[derive(Component, Clone, Copy)]
struct JointInspectorFieldLabel(JointInspectorField);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ActuatorInspectorField {
    ActuatorName,
    MaxForce,
    MaxTorque,
    BehaviorName,
    Frequency,
    Amplitude,
}

#[derive(Component, Clone, Copy)]
struct ActuatorInspectorFieldInput(ActuatorInspectorField);

#[derive(Component, Clone, Copy)]
struct ActuatorInspectorFieldLabel(ActuatorInspectorField);

#[derive(Resource, Default)]
struct ActuatorInspectorState {
    selected_joint_id: Option<String>,
    selected_actuator: Option<Actuator>,
    selected_behavior: Option<Behavior>,
    availability_message: String,
}

#[derive(Resource, Default)]
struct JointInspectorState {
    selected_joint_id: Option<String>,
    selected_joint: Option<Joint>,
}

#[derive(Resource, Default)]
struct JointSelection {
    joint_id: Option<String>,
}

#[derive(Resource, Default)]
struct PartInspectorState {
    selected_part_id: Option<String>,
    selected_part: Option<BodyPart>,
}

/// Selection is stored by stable model ID rather than by a transient ECS entity.
#[derive(Resource, Default)]
struct PartSelection {
    part_id: Option<String>,
}

#[derive(Resource)]
struct DraftProject {
    organism: Organism,
    motion: MotionDefinition,
    next_part_id: u64,
    next_joint_id: u64,
    next_actuator_id: u64,
    pending_joint_part: Option<String>,
}

impl Default for DraftProject {
    fn default() -> Self {
        let organism = Organism::new("Untitled organism");
        let motion = MotionDefinition::new(organism.id.clone());
        Self {
            organism,
            motion,
            next_part_id: 1,
            next_joint_id: 1,
            next_actuator_id: 1,
            pending_joint_part: None,
        }
    }
}

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq)]
enum EditorTool {
    #[default]
    PlaceSphere,
    Select,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectPoweredSlider,
    ConnectDistance,
}

impl EditorTool {
    fn joint_kind(self) -> Option<JointKind> {
        match self {
            Self::ConnectFixed => Some(JointKind::Fixed),
            Self::ConnectBall => Some(JointKind::Ball),
            Self::ConnectHinge | Self::ConnectPoweredHinge => Some(JointKind::Hinge),
            Self::ConnectSlider | Self::ConnectPoweredSlider => Some(JointKind::Slider),
            Self::ConnectDistance => Some(JointKind::Distance),
            Self::Select | Self::PlaceSphere | Self::PlaceCuboid | Self::PlaceCapsule => None,
        }
    }

    fn is_powered_hinge(self) -> bool {
        self == Self::ConnectPoweredHinge
    }

    fn is_powered_slider(self) -> bool {
        self == Self::ConnectPoweredSlider
    }

    fn geometry(self, size_scale: f32) -> Option<Geometry> {
        match self {
            Self::PlaceSphere => Some(Geometry::Sphere {
                radius_m: 0.3 * size_scale,
            }),
            Self::PlaceCuboid => Some(Geometry::Cuboid {
                half_extents_m: DataVec3::new(
                    0.25 * size_scale,
                    0.25 * size_scale,
                    0.25 * size_scale,
                ),
            }),
            Self::PlaceCapsule => Some(Geometry::Capsule {
                radius_m: 0.2 * size_scale,
                half_height_m: 0.45 * size_scale,
            }),
            Self::ConnectFixed
            | Self::ConnectBall
            | Self::ConnectHinge
            | Self::ConnectPoweredHinge
            | Self::ConnectSlider
            | Self::ConnectPoweredSlider
            | Self::ConnectDistance
            | Self::Select => None,
        }
    }
}

#[derive(Component)]
struct RuntimeJointTag {
    id: String,
}

#[derive(Component)]
struct PartTag {
    id: String,
    pick_radius: f32,
}

#[derive(Component, Clone, Copy)]
enum EditorButtonAction {
    PlaceSphere,
    SelectPart,
    PlaceCuboid,
    PlaceCapsule,
    ConnectFixed,
    ConnectBall,
    ConnectHinge,
    ConnectPoweredHinge,
    ConnectSlider,
    ConnectPoweredSlider,
    ConnectDistance,
    SetSizeSmall,
    SetSizeStandard,
    SetSizeLarge,
    SetDensityLight,
    SetDensityMedium,
    SetDensityDense,
    Save,
    ApplyPartProperties,
    ApplyJointProperties,
    ApplyActuatorControllerProperties,
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let floor_size = 24.0;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_size, 0.2, floor_size))),
        MeshMaterial3d(materials.add(Color::srgb(0.11, 0.13, 0.17))),
        Transform::from_xyz(0.0, -0.1, 0.0),
        RigidBody::Static,
        Collider::cuboid(floor_size * 0.5, 0.1, floor_size * 0.5),
        Friction::new(0.85),
        Restitution::new(0.0),
    ));

    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            range: 80.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 10.0, 5.0),
    ));

    commands.spawn((
        Camera3d::default(),
        PanOrbitCamera {
            enabled_motion: EnabledMotion {
                pan: false,
                orbit: true,
                zoom: true,
            },
            ..default()
        },
        Transform::from_xyz(9.0, 8.0, 12.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

macro_rules! action_button {
    ($panel:expr, $label:expr, $action:expr, $color:expr) => {
        $panel
            .spawn((
                Button,
                Node {
                    width: Val::Px(236.0),
                    height: Val::Px(34.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor($color),
                $action,
                observe(handle_editor_button_activated),
            ))
            .with_children(|button| {
                button.spawn((
                    Text::new($label),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    };
}

macro_rules! inspector_field {
    ($panel:expr, $label:expr, $field:ident) => {
        $panel.spawn((
            Text::new($label),
            TextFont {
                font_size: FontSize::Px(10.0),
                ..default()
            },
            TextColor(Color::srgb(0.72, 0.76, 0.82)),
            InspectorFieldLabel(InspectorField::$field),
        ));
        $panel.spawn((
            TextInput,
            EditableText::new(""),
            Node {
                width: Val::Px(236.0),
                height: Val::Px(32.0),
                padding: UiRect::axes(Val::Px(9.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(Color::srgb(0.1, 0.12, 0.16)),
            TextFont {
                font_size: FontSize::Px(13.0),
                ..default()
            },
            TextColor(Color::WHITE),
            InspectorFieldInput(InspectorField::$field),
        ));
    };
}

macro_rules! joint_inspector_field {
    ($panel:expr, $label:expr, $field:ident) => {
        $panel.spawn((
            Text::new($label),
            TextFont {
                font_size: FontSize::Px(10.0),
                ..default()
            },
            TextColor(Color::srgb(0.72, 0.76, 0.82)),
            JointInspectorFieldLabel(JointInspectorField::$field),
        ));
        $panel.spawn((
            TextInput,
            EditableText::new(""),
            Node {
                width: Val::Px(236.0),
                height: Val::Px(32.0),
                padding: UiRect::axes(Val::Px(9.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(Color::srgb(0.1, 0.12, 0.16)),
            TextFont {
                font_size: FontSize::Px(13.0),
                ..default()
            },
            TextColor(Color::WHITE),
            JointInspectorFieldInput(JointInspectorField::$field),
        ));
    };
}

macro_rules! joint_action_button {
    ($panel:expr, $label:expr, $action:expr, $color:expr) => {
        $panel
            .spawn((
                Button,
                Node {
                    width: Val::Px(236.0),
                    height: Val::Px(34.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor($color),
                $action,
                observe(handle_joint_properties_activated),
            ))
            .with_children(|button| {
                button.spawn((
                    Text::new($label),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    };
}

macro_rules! actuator_inspector_field {
    ($panel:expr, $label:expr, $field:ident) => {
        $panel.spawn((
            Text::new($label),
            TextFont {
                font_size: FontSize::Px(10.0),
                ..default()
            },
            TextColor(Color::srgb(0.72, 0.76, 0.82)),
            ActuatorInspectorFieldLabel(ActuatorInspectorField::$field),
        ));
        $panel.spawn((
            TextInput,
            EditableText::new(""),
            Node {
                width: Val::Px(236.0),
                height: Val::Px(32.0),
                padding: UiRect::axes(Val::Px(9.0), Val::Px(5.0)),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(Color::srgb(0.1, 0.12, 0.16)),
            TextFont {
                font_size: FontSize::Px(13.0),
                ..default()
            },
            TextColor(Color::WHITE),
            ActuatorInspectorFieldInput(ActuatorInspectorField::$field),
        ));
    };
}

macro_rules! actuator_action_button {
    ($panel:expr, $label:expr, $action:expr, $color:expr) => {
        $panel
            .spawn((
                Button,
                Node {
                    width: Val::Px(236.0),
                    height: Val::Px(34.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor($color),
                $action,
                observe(handle_actuator_controller_activated),
            ))
            .with_children(|button| {
                button.spawn((
                    Text::new($label),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
    };
}

fn setup_editor_panel(mut commands: Commands) {
    commands
        .spawn((
            Hovered::default(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                left: Val::Px(16.0),
                width: Val::Px(270.0),
                height: Val::Vh(90.0),
                overflow: Overflow {
                    x: OverflowAxis::Hidden,
                    y: OverflowAxis::Scroll,
                },
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(7.0),
                padding: UiRect::all(Val::Px(12.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.035, 0.045, 0.065, 0.96)),
        ))
        .with_children(|panel| {
            panel.spawn((
                Text::new("ORGANISM STUDIO"),
                TextFont {
                    font_size: FontSize::Px(18.0),
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
            panel.spawn((
                Text::new("PART INSPECTOR"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));

            action_button!(
                panel,
                "Select / inspect part",
                EditorButtonAction::SelectPart,
                Color::srgb(0.27, 0.22, 0.12)
            );
            panel.spawn((
                Text::new("No part selected. Choose Select and click a body part."),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.88, 0.86, 0.72)),
                PartInspectorLabel,
            ));
            inspector_field!(panel, "Part name", Name);
            inspector_field!(panel, "Dimension A", DimensionA);
            inspector_field!(panel, "Dimension B", DimensionB);
            inspector_field!(panel, "Dimension C", DimensionC);
            inspector_field!(panel, "Density ┬╖ kg/m┬│", Density);
            inspector_field!(panel, "Friction coefficient", Friction);
            inspector_field!(panel, "Restitution ┬╖ 0ΓÇô1", Restitution);
            inspector_field!(panel, "Linear damping", LinearDamping);
            inspector_field!(panel, "Angular damping", AngularDamping);
            action_button!(
                panel,
                "Apply part properties",
                EditorButtonAction::ApplyPartProperties,
                Color::srgb(0.23, 0.3, 0.42)
            );
            panel.spawn((
                Text::new("Select a rigid part to edit its properties."),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.82, 0.78, 0.57)),
                InspectorStatusLabel,
            ));


            panel.spawn((
                Text::new("JOINT INSPECTOR"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            panel.spawn((
                Text::new("No joint selected. Choose Select / inspect part, then click a joint link."),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.88, 0.86, 0.72)),
                JointInspectorLabel,
            ));
            joint_inspector_field!(panel, "Joint name", Name);
            joint_inspector_field!(panel, "Parent local anchor X (m)", ParentAnchorX);
            joint_inspector_field!(panel, "Parent local anchor Y (m)", ParentAnchorY);
            joint_inspector_field!(panel, "Parent local anchor Z (m)", ParentAnchorZ);
            joint_inspector_field!(panel, "Child local anchor X (m)", ChildAnchorX);
            joint_inspector_field!(panel, "Child local anchor Y (m)", ChildAnchorY);
            joint_inspector_field!(panel, "Child local anchor Z (m)", ChildAnchorZ);
            joint_inspector_field!(panel, "Distance rest length (m)", RestLength);
            joint_action_button!(
                panel,
                "Apply joint properties",
                EditorButtonAction::ApplyJointProperties,
                Color::srgb(0.23, 0.3, 0.42)
            );
            panel.spawn((
                Text::new("Select a joint link to edit its anchors and properties."),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.82, 0.78, 0.57)),
                JointInspectorStatusLabel,
            ));


            panel.spawn((
                Text::new("ACTUATOR / CONTROLLER INSPECTOR"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            panel.spawn((
                Text::new("Select a powered joint link to inspect its actuator and oscillator controller."),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.88, 0.86, 0.72)),
                ActuatorInspectorLabel,
            ));
            actuator_inspector_field!(panel, "Actuator name", ActuatorName);
            actuator_inspector_field!(panel, "Maximum force (N)", MaxForce);
            actuator_inspector_field!(panel, "Maximum torque (N m)", MaxTorque);
            actuator_inspector_field!(panel, "Controller behavior name", BehaviorName);
            actuator_inspector_field!(panel, "Oscillation frequency (Hz)", Frequency);
            actuator_inspector_field!(panel, "Oscillation amplitude", Amplitude);
            actuator_action_button!(
                panel,
                "Apply actuator / controller",
                EditorButtonAction::ApplyActuatorControllerProperties,
                Color::srgb(0.23, 0.3, 0.42)
            );
            panel.spawn((
                Text::new("Select a powered hinge or slider link to edit its live motor."),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.82, 0.78, 0.57)),
                ActuatorInspectorStatusLabel,
            ));

            panel.spawn((
                Text::new("BODY CONSTRUCTION"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            action_button!(
                panel,
                "Add sphere",
                EditorButtonAction::PlaceSphere,
                Color::srgb(0.12, 0.2, 0.3)
            );
            action_button!(
                panel,
                "Add cuboid",
                EditorButtonAction::PlaceCuboid,
                Color::srgb(0.12, 0.2, 0.3)
            );
            action_button!(
                panel,
                "Add capsule",
                EditorButtonAction::PlaceCapsule,
                Color::srgb(0.12, 0.2, 0.3)
            );

            panel.spawn((
                Text::new("ASSISTED DEFAULTS"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            action_button!(
                panel,
                "Small parts",
                EditorButtonAction::SetSizeSmall,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Standard parts",
                EditorButtonAction::SetSizeStandard,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Large parts",
                EditorButtonAction::SetSizeLarge,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Light material ┬╖ 300 kg/m┬│",
                EditorButtonAction::SetDensityLight,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Medium material ┬╖ 1,000 kg/m┬│",
                EditorButtonAction::SetDensityMedium,
                Color::srgb(0.18, 0.2, 0.27)
            );
            action_button!(
                panel,
                "Dense material ┬╖ 1,800 kg/m┬│",
                EditorButtonAction::SetDensityDense,
                Color::srgb(0.18, 0.2, 0.27)
            );
            panel.spawn((
                Text::new("Size 1.00├ù ┬╖ Density 1,000 kg/m┬│"),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.85, 0.83, 0.64)),
                PlacementSettingsLabel,
            ));

            panel.spawn((
                Text::new("CONNECTIONS"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));

            action_button!(
                panel,
                "Connect fixed",
                EditorButtonAction::ConnectFixed,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect ball-and-socket",
                EditorButtonAction::ConnectBall,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect hinge",
                EditorButtonAction::ConnectHinge,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect powered hinge (oscillating)",
                EditorButtonAction::ConnectPoweredHinge,
                Color::srgb(0.36, 0.25, 0.12)
            );
            action_button!(
                panel,
                "Connect slider",
                EditorButtonAction::ConnectSlider,
                Color::srgb(0.16, 0.19, 0.26)
            );
            action_button!(
                panel,
                "Connect powered slider",
                EditorButtonAction::ConnectPoweredSlider,
                Color::srgb(0.36, 0.25, 0.12)
            );
            action_button!(
                panel,
                "Connect distance",
                EditorButtonAction::ConnectDistance,
                Color::srgb(0.16, 0.19, 0.26)
            );

            panel.spawn((
                Text::new("EXPORT"),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(Color::srgb(0.45, 0.74, 0.95)),
            ));
            action_button!(
                panel,
                "Save organism + motion JSON",
                EditorButtonAction::Save,
                Color::srgb(0.16, 0.34, 0.23)
            );

            panel.spawn((
                Text::new(
                    "Place: click the ground\nConnect: select two parts\nJoints connect at their midpoint\nUnits: meters / kilograms / seconds",
                ),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(Color::srgb(0.72, 0.76, 0.82)),
            ));
        });
}

fn update_placement_settings_label(
    placement: Res<PlacementSettings>,
    mut labels: Query<&mut Text, With<PlacementSettingsLabel>>,
) {
    if !placement.is_changed() {
        return;
    }

    for mut label in &mut labels {
        label.0 = format!(
            "Size {:.2}├ù ┬╖ Density {:.0} kg/m┬│",
            placement.size_scale, placement.density_kg_m3
        );
    }
}

fn handle_editor_button_activated(
    activate: On<Activate>,
    actions: Query<&EditorButtonAction>,
    mut tool: ResMut<EditorTool>,
    mut placement: ResMut<PlacementSettings>,
    mut project: ResMut<DraftProject>,
    selection: Res<PartSelection>,
    inspector_inputs: Query<(&EditableText, &InspectorFieldInput)>,
    part_entities: Query<(Entity, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut inspector_status: Query<&mut Text, With<InspectorStatusLabel>>,
) {
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };

    match *action {
        EditorButtonAction::PlaceSphere => *tool = EditorTool::PlaceSphere,
        EditorButtonAction::SelectPart => *tool = EditorTool::Select,
        EditorButtonAction::PlaceCuboid => *tool = EditorTool::PlaceCuboid,
        EditorButtonAction::PlaceCapsule => *tool = EditorTool::PlaceCapsule,
        EditorButtonAction::ConnectFixed => *tool = EditorTool::ConnectFixed,
        EditorButtonAction::ConnectBall => *tool = EditorTool::ConnectBall,
        EditorButtonAction::ConnectHinge => *tool = EditorTool::ConnectHinge,
        EditorButtonAction::ConnectPoweredHinge => *tool = EditorTool::ConnectPoweredHinge,
        EditorButtonAction::ConnectSlider => *tool = EditorTool::ConnectSlider,
        EditorButtonAction::ConnectPoweredSlider => *tool = EditorTool::ConnectPoweredSlider,
        EditorButtonAction::ConnectDistance => *tool = EditorTool::ConnectDistance,
        EditorButtonAction::SetSizeSmall => placement.size_scale = 0.5,
        EditorButtonAction::SetSizeStandard => placement.size_scale = 1.0,
        EditorButtonAction::SetSizeLarge => placement.size_scale = 2.0,
        EditorButtonAction::SetDensityLight => placement.density_kg_m3 = 300.0,
        EditorButtonAction::SetDensityMedium => placement.density_kg_m3 = 1000.0,
        EditorButtonAction::SetDensityDense => placement.density_kg_m3 = 1800.0,
        EditorButtonAction::ApplyPartProperties => {
            apply_selected_part_edit(
                selection.part_id.as_deref(),
                &mut project,
                &inspector_inputs,
                &part_entities,
                &mut commands,
                &mut meshes,
                &mut inspector_status,
            );
        }
        EditorButtonAction::Save => {
            match save_project(
                Path::new(EXPORT_DIRECTORY),
                &project.organism,
                &project.motion,
            ) {
                Ok(paths) => info!(
                    "Saved organism to {} and motion to {}",
                    paths.organism.display(),
                    paths.motion.display()
                ),
                Err(error) => error!("Could not save project: {error}"),
            }
        }
        EditorButtonAction::ApplyJointProperties => {}
        EditorButtonAction::ApplyActuatorControllerProperties => {}
    }
}

fn handle_actuator_controller_activated(
    activate: On<Activate>,
    actions: Query<&EditorButtonAction>,
    state: Res<ActuatorInspectorState>,
    mut project: ResMut<DraftProject>,
    inputs: Query<(&EditableText, &ActuatorInspectorFieldInput)>,
    mut status: Query<&mut Text, With<ActuatorInspectorStatusLabel>>,
) {
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };
    if !matches!(
        *action,
        EditorButtonAction::ApplyActuatorControllerProperties
    ) {
        return;
    }

    apply_selected_actuator_controller_edit(
        state
            .selected_actuator
            .as_ref()
            .map(|actuator| actuator.id.as_str()),
        state
            .selected_behavior
            .as_ref()
            .map(|behavior| behavior.id.as_str()),
        &mut project,
        &inputs,
        &mut status,
    );
}

fn handle_joint_properties_activated(
    activate: On<Activate>,
    actions: Query<&EditorButtonAction>,
    joint_selection: Res<JointSelection>,
    mut project: ResMut<DraftProject>,
    joint_inspector_inputs: Query<(&EditableText, &JointInspectorFieldInput)>,
    runtime_joint_entities: Query<(Entity, &RuntimeJointTag)>,
    fixed_joints: Query<&FixedJoint>,
    spherical_joints: Query<&SphericalJoint>,
    revolute_joints: Query<&RevoluteJoint>,
    prismatic_joints: Query<&PrismaticJoint>,
    distance_joints: Query<&DistanceJoint>,
    mut commands: Commands,
    mut status: Query<&mut Text, With<JointInspectorStatusLabel>>,
) {
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };
    if !matches!(*action, EditorButtonAction::ApplyJointProperties) {
        return;
    }

    apply_selected_joint_edit(
        joint_selection.joint_id.as_deref(),
        &mut project,
        &joint_inspector_inputs,
        &runtime_joint_entities,
        &fixed_joints,
        &spherical_joints,
        &revolute_joints,
        &prismatic_joints,
        &distance_joints,
        &mut commands,
        &mut status,
    );
}
fn edit_from_viewport(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    hovered_panels: Query<&Hovered>,
    part_query: Query<(Entity, &GlobalTransform, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut project: ResMut<DraftProject>,
    tool: Res<EditorTool>,
    placement: Res<PlacementSettings>,
    mut selection: ResMut<PartSelection>,
    mut joint_selection: ResMut<JointSelection>,
) {
    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    if hovered_panels.iter().any(|hovered| hovered.get()) {
        return;
    }

    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor_position) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        return;
    };
    let ray_direction = ray.direction.as_vec3();

    if *tool == EditorTool::Select {
        if let Some((_, joint_id)) = pick_joint(
            ray.origin,
            ray_direction,
            &project.organism.joints,
            &part_query,
        ) {
            selection.part_id = None;
            if joint_selection.joint_id.as_deref() != Some(joint_id.as_str()) {
                info!("Selected joint {joint_id}");
            }
            joint_selection.joint_id = Some(joint_id);
        } else {
            let picked = pick_body_part(ray.origin, ray_direction, &part_query)
                .map(|(_, _, part_id, _)| part_id);
            joint_selection.joint_id = None;
            if selection.part_id != picked {
                match &picked {
                    Some(part_id) => info!("Selected {part_id}"),
                    None => info!("Selection cleared"),
                }
                selection.part_id = picked;
            }
        }
        return;
    }

    if let Some(geometry) = tool.geometry(placement.size_scale) {
        place_body_part(
            geometry,
            ray.origin,
            ray_direction,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut project,
            placement.density_kg_m3,
        );
        return;
    }

    let Some(joint_kind) = tool.joint_kind() else {
        return;
    };
    connect_body_parts(
        joint_kind,
        tool.is_powered_hinge(),
        tool.is_powered_slider(),
        ray.origin,
        ray_direction,
        &part_query,
        &mut commands,
        &mut project,
    );
}

fn place_body_part(
    geometry: Geometry,
    ray_origin: Vec3,
    ray_direction: Vec3,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    project: &mut DraftProject,
    density_kg_m3: f32,
) {
    if ray_direction.y.abs() < 1.0e-5 {
        return;
    }
    let ground_distance = -ray_origin.y / ray_direction.y;
    if ground_distance <= 0.0 {
        return;
    }
    let ground_point = ray_origin + ray_direction * ground_distance;

    let Some(runtime_geometry) = build_runtime_geometry(&geometry) else {
        warn!("Cannot construct a runtime shape from invalid geometry");
        return;
    };

    let position = ground_point + Vec3::Y * runtime_geometry.ground_offset;
    let sequence = project.next_part_id;
    project.next_part_id += 1;
    let part_id = format!("part-{sequence:04}");
    let Some(estimated_mass_kg) = geometry.mass_kg_at_density(density_kg_m3) else {
        warn!("Cannot place a part with invalid geometry or estimated mass");
        return;
    };
    let properties = RigidBodyProperties {
        mass_kg: estimated_mass_kg,
        density_kg_m3,
        friction: PART_FRICTION,
        restitution: PART_RESTITUTION,
        linear_damping: PART_LINEAR_DAMPING,
        angular_damping: PART_ANGULAR_DAMPING,
    };

    // Avian expects density here and derives mass/inertia from the collider volume.
    let mass_properties =
        MassPropertiesBundle::from_shape(&runtime_geometry.collider, properties.density_kg_m3);
    commands.spawn((
        Mesh3d(meshes.add(runtime_geometry.mesh)),
        MeshMaterial3d(materials.add(runtime_geometry.color)),
        Transform::from_translation(position),
        RigidBody::Dynamic,
        runtime_geometry.collider,
        mass_properties,
        Friction::new(properties.friction),
        Restitution::new(properties.restitution),
        LinearDamping(properties.linear_damping),
        AngularDamping(properties.angular_damping),
        PartTag {
            id: part_id.clone(),
            pick_radius: runtime_geometry.pick_radius,
        },
    ));

    project.organism.parts.push(BodyPart {
        id: part_id.clone(),
        name: format!("Part {sequence}"),
        geometry,
        transform: Transform3::at(DataVec3::new(position.x, position.y, position.z)),
        physics: BodyPhysics::Rigid(properties),
    });
    info!("Added {part_id}");
}

fn pick_joint(
    ray_origin: Vec3,
    ray_direction: Vec3,
    joints: &[Joint],
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
) -> Option<(f32, String)> {
    joints
        .iter()
        .filter_map(|joint| {
            let parent = part_query
                .iter()
                .find(|(_, _, tag)| tag.id == joint.parent_part_id)
                .map(|(_, transform, _)| transform.translation())?;
            let child = part_query
                .iter()
                .find(|(_, _, tag)| tag.id == joint.child_part_id)
                .map(|(_, transform, _)| transform.translation())?;
            let distance = ray_segment_distance(ray_origin, ray_direction, parent, child);
            let pick_radius = 0.14 + parent.distance(child) * 0.01;
            (distance <= pick_radius).then(|| (distance, joint.id.clone()))
        })
        .min_by(|left, right| left.0.total_cmp(&right.0))
}

fn ray_segment_distance(
    ray_origin: Vec3,
    ray_direction: Vec3,
    segment_start: Vec3,
    segment_end: Vec3,
) -> f32 {
    let segment = segment_end - segment_start;
    let offset = ray_origin - segment_start;
    let ray_length_squared = ray_direction.length_squared();
    let segment_length_squared = segment.length_squared();

    if segment_length_squared <= f32::EPSILON {
        let ray_parameter = (-offset.dot(ray_direction) / ray_length_squared).max(0.0);
        return (offset + ray_direction * ray_parameter).length();
    }

    let ray_segment_dot = ray_direction.dot(segment);
    let ray_offset_dot = ray_direction.dot(offset);
    let segment_offset_dot = segment.dot(offset);
    let denominator = ray_length_squared * segment_length_squared - ray_segment_dot.powi(2);

    let mut segment_parameter = if denominator.abs() > f32::EPSILON {
        (ray_length_squared * segment_offset_dot - ray_segment_dot * ray_offset_dot) / denominator
    } else {
        segment_offset_dot / segment_length_squared
    }
    .clamp(0.0, 1.0);

    let mut ray_parameter =
        ((ray_segment_dot * segment_parameter - ray_offset_dot) / ray_length_squared).max(0.0);
    segment_parameter = ((ray_segment_dot * ray_parameter + segment_offset_dot)
        / segment_length_squared)
        .clamp(0.0, 1.0);
    ray_parameter =
        ((ray_segment_dot * segment_parameter - ray_offset_dot) / ray_length_squared).max(0.0);

    let closest_ray = ray_origin + ray_direction * ray_parameter;
    let closest_segment = segment_start + segment * segment_parameter;
    closest_ray.distance(closest_segment)
}

fn pick_body_part(
    ray_origin: Vec3,
    ray_direction: Vec3,
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
) -> Option<(f32, Entity, String, GlobalTransform)> {
    part_query
        .iter()
        .filter_map(|(entity, transform, tag)| {
            ray_sphere_distance(
                ray_origin,
                ray_direction,
                transform.translation(),
                tag.pick_radius * 1.08,
            )
            .map(|distance| (distance, entity, tag.id.clone(), *transform))
        })
        .min_by(|left, right| left.0.total_cmp(&right.0))
}

fn connect_body_parts(
    kind: JointKind,
    powered_hinge: bool,
    powered_slider: bool,
    ray_origin: Vec3,
    ray_direction: Vec3,
    part_query: &Query<(Entity, &GlobalTransform, &PartTag)>,
    commands: &mut Commands,
    project: &mut DraftProject,
) {
    let picked = pick_body_part(ray_origin, ray_direction, part_query);

    let Some((_, child_entity, child_id, child_transform)) = picked else {
        project.pending_joint_part = None;
        info!("No body part under cursor; select a body part");
        return;
    };

    let Some(parent_id) = project.pending_joint_part.take() else {
        project.pending_joint_part = Some(child_id.clone());
        info!("Selected {child_id}; now select the second body part");
        return;
    };

    if parent_id == child_id {
        project.pending_joint_part = Some(child_id);
        info!("Choose a different body part for the second endpoint");
        return;
    }

    let Some((parent_entity, parent_transform)) = part_query
        .iter()
        .find(|(_, _, tag)| tag.id == parent_id)
        .map(|(entity, transform, _)| (entity, *transform))
    else {
        warn!("Selected parent part {parent_id} no longer exists");
        return;
    };

    let parent_position = parent_transform.translation();
    let child_position = child_transform.translation();
    let center_distance = parent_position.distance(child_position);
    let world_anchor = (parent_position + child_position) * 0.5;
    let joint_sequence = project.next_joint_id;
    let joint_id = format!("joint-{joint_sequence:04}");

    match kind {
        JointKind::Fixed => {
            commands.spawn((
                FixedJoint::new(parent_entity, child_entity).with_anchor(world_anchor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
        }
        JointKind::Ball => {
            commands.spawn((
                SphericalJoint::new(parent_entity, child_entity).with_anchor(world_anchor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
        }
        JointKind::Hinge if powered_hinge => {
            let motor = AngularMotor::new(MotorModel::SpringDamper {
                frequency: 8.0,
                damping_ratio: 1.0,
            })
            .with_target_position(0.0)
            .with_max_torque(10.0);
            commands.spawn((
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_hinge_axis(Vec3::Z)
                    .with_motor(motor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
            commands.entity(parent_entity).insert(SleepingDisabled);
            commands.entity(child_entity).insert(SleepingDisabled);
        }
        JointKind::Hinge => {
            commands.spawn((
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_hinge_axis(Vec3::Z),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
        }
        JointKind::Slider if powered_slider => {
            let motor = LinearMotor::new(MotorModel::SpringDamper {
                frequency: 8.0,
                damping_ratio: 1.0,
            })
            .with_target_position(0.0)
            .with_max_force(20.0);
            commands.spawn((
                PrismaticJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_slider_axis(Vec3::X)
                    .with_motor(motor),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
            commands.entity(parent_entity).insert(SleepingDisabled);
            commands.entity(child_entity).insert(SleepingDisabled);
        }
        JointKind::Slider => {
            commands.spawn((
                PrismaticJoint::new(parent_entity, child_entity)
                    .with_anchor(world_anchor)
                    .with_slider_axis(Vec3::X),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
        }
        JointKind::Distance => {
            commands.spawn((
                DistanceJoint::new(parent_entity, child_entity)
                    .with_limits(center_distance.max(0.001), center_distance.max(0.001)),
                RuntimeJointTag {
                    id: joint_id.clone(),
                },
            ));
        }
    }

    project.next_joint_id += 1;

    let (anchor_parent, anchor_child, rest_length_m) = if kind == JointKind::Distance {
        (Vec3::ZERO, Vec3::ZERO, Some(center_distance.max(0.001)))
    } else {
        (
            parent_transform
                .affine()
                .inverse()
                .transform_point3(world_anchor),
            child_transform
                .affine()
                .inverse()
                .transform_point3(world_anchor),
            None,
        )
    };

    project.organism.joints.push(Joint {
        id: joint_id.clone(),
        name: format!("{parent_id} to {child_id} {kind:?}"),
        kind,
        parent_part_id: parent_id,
        child_part_id: child_id,
        anchor_parent_m: DataVec3::new(anchor_parent.x, anchor_parent.y, anchor_parent.z),
        anchor_child_m: DataVec3::new(anchor_child.x, anchor_child.y, anchor_child.z),
        limits: None,
        rest_length_m,
    });

    if powered_hinge || powered_slider {
        let actuator_sequence = project.next_actuator_id;
        project.next_actuator_id += 1;
        let actuator_id = format!("actuator-{actuator_sequence:04}");
        let (actuator_kind, max_force_n, max_torque_nm, default_amplitude, name) = if powered_hinge
        {
            (
                ActuatorKind::AngularMotor,
                None,
                Some(10.0),
                0.55,
                format!("Powered hinge {actuator_sequence}"),
            )
        } else {
            (
                ActuatorKind::LinearMotor,
                Some(20.0),
                None,
                0.2,
                format!("Powered slider {actuator_sequence}"),
            )
        };
        project.organism.actuators.push(Actuator {
            id: actuator_id.clone(),
            name,
            joint_id: joint_id.clone(),
            kind: actuator_kind,
            max_force_n,
            max_torque_nm,
        });
        project.motion.behaviors.push(Behavior {
            id: format!("behavior-{actuator_sequence:04}"),
            name: format!("Oscillate {joint_id}"),
            controller: ControllerKind::Oscillator,
            goals: vec![MotionGoal::OscillateActuator {
                actuator_id,
                frequency_hz: 0.5,
                amplitude: default_amplitude,
            }],
        });
    }
    info!("Created {kind:?} connection");
}

fn drive_actuator_behaviors(
    time: Res<Time>,
    project: Res<DraftProject>,
    mut angular_joints: Query<(&mut RevoluteJoint, &RuntimeJointTag)>,
    mut linear_joints: Query<(&mut PrismaticJoint, &RuntimeJointTag)>,
) {
    let elapsed = time.elapsed_secs();
    for behavior in &project.motion.behaviors {
        if behavior.controller != ControllerKind::Oscillator {
            continue;
        }
        for goal in &behavior.goals {
            let MotionGoal::OscillateActuator {
                actuator_id,
                frequency_hz,
                amplitude,
            } = goal
            else {
                continue;
            };
            let Some(actuator) = project
                .organism
                .actuators
                .iter()
                .find(|actuator| actuator.id == *actuator_id)
            else {
                continue;
            };
            let target = amplitude * (std::f32::consts::TAU * frequency_hz * elapsed).sin();

            match actuator.kind {
                ActuatorKind::AngularMotor => {
                    for (mut joint, tag) in &mut angular_joints {
                        if tag.id == actuator.joint_id {
                            if let Some(max_torque_nm) = actuator.max_torque_nm {
                                joint.motor.max_torque = max_torque_nm;
                            }
                            joint.motor.target_position = target;
                            break;
                        }
                    }
                }
                ActuatorKind::LinearMotor => {
                    for (mut joint, tag) in &mut linear_joints {
                        if tag.id == actuator.joint_id {
                            if let Some(max_force_n) = actuator.max_force_n {
                                joint.motor.max_force = max_force_n;
                            }
                            joint.motor.target_position = target;
                            break;
                        }
                    }
                }
                ActuatorKind::MuscleTendon => {}
            }
        }
    }
}

fn apply_selected_part_edit(
    selected_part_id: Option<&str>,
    project: &mut DraftProject,
    inputs: &Query<(&EditableText, &InspectorFieldInput)>,
    part_entities: &Query<(Entity, &PartTag)>,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    status: &mut Query<&mut Text, With<InspectorStatusLabel>>,
) {
    let Some(part_id) = selected_part_id else {
        set_inspector_status(status, "Select a body part before applying changes.");
        return;
    };
    let Some(part_index) = project
        .organism
        .parts
        .iter()
        .position(|part| part.id == part_id)
    else {
        set_inspector_status(status, "The selected part is no longer in the organism.");
        return;
    };
    let Some((entity, _)) = part_entities.iter().find(|(_, tag)| tag.id == part_id) else {
        set_inspector_status(status, "The selected part has no live physics entity.");
        return;
    };

    let mut updated = project.organism.parts[part_index].clone();
    if matches!(&updated.physics, BodyPhysics::Soft(_)) {
        set_inspector_status(
            status,
            "Soft-body editing is unavailable until a deformable runtime solver exists.",
        );
        return;
    }
    let Some(name) = inspector_field_text(inputs, InspectorField::Name) else {
        set_inspector_status(status, "Enter a non-empty part name.");
        return;
    };

    let geometry = match &updated.geometry {
        Geometry::Sphere { .. } => {
            let Some(radius_m) = inspector_field_number(inputs, InspectorField::DimensionA) else {
                set_inspector_status(status, "Enter a valid sphere radius in meters.");
                return;
            };
            Geometry::Sphere { radius_m }
        }
        Geometry::Cuboid { .. } => {
            let Some(x) = inspector_field_number(inputs, InspectorField::DimensionA) else {
                set_inspector_status(status, "Enter a valid X half-extent in meters.");
                return;
            };
            let Some(y) = inspector_field_number(inputs, InspectorField::DimensionB) else {
                set_inspector_status(status, "Enter a valid Y half-extent in meters.");
                return;
            };
            let Some(z) = inspector_field_number(inputs, InspectorField::DimensionC) else {
                set_inspector_status(status, "Enter a valid Z half-extent in meters.");
                return;
            };
            Geometry::Cuboid {
                half_extents_m: DataVec3::new(x, y, z),
            }
        }
        Geometry::Capsule { .. } => {
            let Some(radius_m) = inspector_field_number(inputs, InspectorField::DimensionA) else {
                set_inspector_status(status, "Enter a valid capsule radius in meters.");
                return;
            };
            let Some(half_height_m) = inspector_field_number(inputs, InspectorField::DimensionB)
            else {
                set_inspector_status(status, "Enter a valid capsule half-height in meters.");
                return;
            };
            Geometry::Capsule {
                radius_m,
                half_height_m,
            }
        }
    };

    let Some(density_kg_m3) = inspector_field_number(inputs, InspectorField::Density) else {
        set_inspector_status(status, "Enter a valid material density in kg/m┬│.");
        return;
    };
    let Some(friction) = inspector_field_number(inputs, InspectorField::Friction) else {
        set_inspector_status(status, "Enter a valid friction coefficient.");
        return;
    };
    let Some(restitution) = inspector_field_number(inputs, InspectorField::Restitution) else {
        set_inspector_status(status, "Enter restitution between zero and one.");
        return;
    };
    let Some(linear_damping) = inspector_field_number(inputs, InspectorField::LinearDamping) else {
        set_inspector_status(status, "Enter valid linear damping.");
        return;
    };
    let Some(angular_damping) = inspector_field_number(inputs, InspectorField::AngularDamping)
    else {
        set_inspector_status(status, "Enter valid angular damping.");
        return;
    };

    if let Err(error) = apply_part_edit(
        &mut updated,
        PartEdit {
            name,
            geometry,
            density_kg_m3,
            friction,
            restitution,
            linear_damping,
            angular_damping,
        },
    ) {
        set_inspector_status(status, &error.to_string());
        return;
    }

    let Some(runtime) = build_runtime_geometry(&updated.geometry) else {
        set_inspector_status(
            status,
            "The edited geometry cannot be represented at runtime.",
        );
        return;
    };
    let BodyPhysics::Rigid(properties) = &updated.physics else {
        unreachable!("the validated edit operation only supports rigid bodies");
    };
    let mass_kg = properties.mass_kg;
    let mass_properties =
        MassPropertiesBundle::from_shape(&runtime.collider, properties.density_kg_m3);

    commands.entity(entity).insert((
        Mesh3d(meshes.add(runtime.mesh)),
        runtime.collider,
        mass_properties,
        Friction::new(properties.friction),
        Restitution::new(properties.restitution),
        LinearDamping(properties.linear_damping),
        AngularDamping(properties.angular_damping),
        PartTag {
            id: part_id.to_owned(),
            pick_radius: runtime.pick_radius,
        },
    ));
    project.organism.parts[part_index] = updated;
    set_inspector_status(
        status,
        &format!("Applied part properties ┬╖ mass {mass_kg:.4} kg."),
    );
    info!("Updated {part_id}: mass {mass_kg:.4} kg");
}

fn apply_selected_joint_edit(
    selected_joint_id: Option<&str>,
    project: &mut DraftProject,
    inputs: &Query<(&EditableText, &JointInspectorFieldInput)>,
    runtime_joint_entities: &Query<(Entity, &RuntimeJointTag)>,
    fixed_joints: &Query<&FixedJoint>,
    spherical_joints: &Query<&SphericalJoint>,
    revolute_joints: &Query<&RevoluteJoint>,
    prismatic_joints: &Query<&PrismaticJoint>,
    distance_joints: &Query<&DistanceJoint>,
    commands: &mut Commands,
    status: &mut Query<&mut Text, With<JointInspectorStatusLabel>>,
) {
    let Some(joint_id) = selected_joint_id else {
        set_joint_inspector_status(status, "Select a joint link before applying changes.");
        return;
    };
    let Some(joint_index) = project
        .organism
        .joints
        .iter()
        .position(|joint| joint.id == joint_id)
    else {
        set_joint_inspector_status(status, "The selected joint is no longer in the organism.");
        return;
    };
    let Some((runtime_entity, _)) = runtime_joint_entities
        .iter()
        .find(|(_, tag)| tag.id == joint_id)
    else {
        set_joint_inspector_status(status, "The selected joint has no live physics constraint.");
        return;
    };

    let mut updated = project.organism.joints[joint_index].clone();
    let Some(name) = joint_inspector_field_text(inputs, JointInspectorField::Name) else {
        set_joint_inspector_status(status, "Enter a non-empty joint name.");
        return;
    };
    let Some(anchor_parent_m) = joint_inspector_vec3(
        inputs,
        JointInspectorField::ParentAnchorX,
        JointInspectorField::ParentAnchorY,
        JointInspectorField::ParentAnchorZ,
    ) else {
        set_joint_inspector_status(
            status,
            "Enter finite parent-local anchor coordinates in meters.",
        );
        return;
    };
    let Some(anchor_child_m) = joint_inspector_vec3(
        inputs,
        JointInspectorField::ChildAnchorX,
        JointInspectorField::ChildAnchorY,
        JointInspectorField::ChildAnchorZ,
    ) else {
        set_joint_inspector_status(
            status,
            "Enter finite child-local anchor coordinates in meters.",
        );
        return;
    };
    let rest_length_m = if updated.kind == JointKind::Distance {
        let Some(length) = joint_inspector_field_number(inputs, JointInspectorField::RestLength)
        else {
            set_joint_inspector_status(status, "Enter a positive distance rest length in meters.");
            return;
        };
        Some(length)
    } else {
        None
    };

    if let Err(error) = apply_joint_edit(
        &mut updated,
        JointEdit {
            name,
            anchor_parent_m,
            anchor_child_m,
            rest_length_m,
        },
    ) {
        set_joint_inspector_status(status, &error.to_string());
        return;
    }

    let parent_anchor = runtime_joint_anchor(updated.anchor_parent_m);
    let child_anchor = runtime_joint_anchor(updated.anchor_child_m);
    match updated.kind {
        JointKind::Fixed => {
            let Ok(current) = fixed_joints.get(runtime_entity) else {
                set_joint_inspector_status(status, "The fixed constraint is not available.");
                return;
            };
            commands.entity(runtime_entity).insert(
                current
                    .clone()
                    .with_local_anchor1(parent_anchor)
                    .with_local_anchor2(child_anchor),
            );
        }
        JointKind::Ball => {
            let Ok(current) = spherical_joints.get(runtime_entity) else {
                set_joint_inspector_status(status, "The ball constraint is not available.");
                return;
            };
            commands.entity(runtime_entity).insert(
                current
                    .clone()
                    .with_local_anchor1(parent_anchor)
                    .with_local_anchor2(child_anchor),
            );
        }
        JointKind::Hinge => {
            let Ok(current) = revolute_joints.get(runtime_entity) else {
                set_joint_inspector_status(status, "The hinge constraint is not available.");
                return;
            };
            commands.entity(runtime_entity).insert(
                current
                    .clone()
                    .with_local_anchor1(parent_anchor)
                    .with_local_anchor2(child_anchor),
            );
        }
        JointKind::Slider => {
            let Ok(current) = prismatic_joints.get(runtime_entity) else {
                set_joint_inspector_status(status, "The slider constraint is not available.");
                return;
            };
            commands.entity(runtime_entity).insert(
                current
                    .clone()
                    .with_local_anchor1(parent_anchor)
                    .with_local_anchor2(child_anchor),
            );
        }
        JointKind::Distance => {
            let Ok(current) = distance_joints.get(runtime_entity) else {
                set_joint_inspector_status(status, "The distance constraint is not available.");
                return;
            };
            let rest_length_m = updated.rest_length_m.unwrap_or_default();
            commands.entity(runtime_entity).insert(
                current
                    .clone()
                    .with_local_anchor1(parent_anchor)
                    .with_local_anchor2(child_anchor)
                    .with_limits(rest_length_m, rest_length_m),
            );
        }
    }

    project.organism.joints[joint_index] = updated;
    set_joint_inspector_status(status, "Applied joint properties to the live constraint.");
}

fn runtime_joint_anchor(anchor: DataVec3) -> Vec3 {
    Vec3::new(anchor.x, anchor.y, anchor.z)
}

fn joint_inspector_field_text(
    inputs: &Query<(&EditableText, &JointInspectorFieldInput)>,
    field: JointInspectorField,
) -> Option<String> {
    inputs
        .iter()
        .find(|(_, marker)| marker.0 == field)
        .map(|(input, _)| input.value().to_string())
}

fn joint_inspector_field_number(
    inputs: &Query<(&EditableText, &JointInspectorFieldInput)>,
    field: JointInspectorField,
) -> Option<f32> {
    joint_inspector_field_text(inputs, field)?
        .trim()
        .parse::<f32>()
        .ok()
}

fn joint_inspector_vec3(
    inputs: &Query<(&EditableText, &JointInspectorFieldInput)>,
    x: JointInspectorField,
    y: JointInspectorField,
    z: JointInspectorField,
) -> Option<DataVec3> {
    Some(DataVec3::new(
        joint_inspector_field_number(inputs, x)?,
        joint_inspector_field_number(inputs, y)?,
        joint_inspector_field_number(inputs, z)?,
    ))
}

fn inspector_field_text(
    inputs: &Query<(&EditableText, &InspectorFieldInput)>,
    field: InspectorField,
) -> Option<String> {
    inputs
        .iter()
        .find(|(_, marker)| marker.0 == field)
        .map(|(input, _)| input.value().to_string())
}

fn inspector_field_number(
    inputs: &Query<(&EditableText, &InspectorFieldInput)>,
    field: InspectorField,
) -> Option<f32> {
    inspector_field_text(inputs, field)?
        .trim()
        .parse::<f32>()
        .ok()
}

fn inspector_field_value(part: &BodyPart, field: InspectorField) -> String {
    match field {
        InspectorField::Name => part.name.clone(),
        InspectorField::DimensionA => match &part.geometry {
            Geometry::Sphere { radius_m } => format!("{radius_m:.4}"),
            Geometry::Cuboid { half_extents_m } => format!("{:.4}", half_extents_m.x),
            Geometry::Capsule { radius_m, .. } => format!("{radius_m:.4}"),
        },
        InspectorField::DimensionB => match &part.geometry {
            Geometry::Cuboid { half_extents_m } => format!("{:.4}", half_extents_m.y),
            Geometry::Capsule { half_height_m, .. } => format!("{half_height_m:.4}"),
            Geometry::Sphere { .. } => String::new(),
        },
        InspectorField::DimensionC => match &part.geometry {
            Geometry::Cuboid { half_extents_m } => format!("{:.4}", half_extents_m.z),
            Geometry::Sphere { .. } | Geometry::Capsule { .. } => String::new(),
        },
        InspectorField::Density => match &part.physics {
            BodyPhysics::Rigid(p) => format!("{:.2}", p.density_kg_m3),
            BodyPhysics::Soft(p) => format!("{:.2}", p.density_kg_m3),
        },
        InspectorField::Friction => match &part.physics {
            BodyPhysics::Rigid(p) => format!("{:.3}", p.friction),
            BodyPhysics::Soft(_) => String::new(),
        },
        InspectorField::Restitution => match &part.physics {
            BodyPhysics::Rigid(p) => format!("{:.3}", p.restitution),
            BodyPhysics::Soft(_) => String::new(),
        },
        InspectorField::LinearDamping => match &part.physics {
            BodyPhysics::Rigid(p) => format!("{:.3}", p.linear_damping),
            BodyPhysics::Soft(_) => String::new(),
        },
        InspectorField::AngularDamping => match &part.physics {
            BodyPhysics::Rigid(p) => format!("{:.3}", p.angular_damping),
            BodyPhysics::Soft(_) => String::new(),
        },
    }
}

fn inspector_field_label(part: &BodyPart, field: InspectorField) -> String {
    match field {
        InspectorField::Name => "Part name".into(),
        InspectorField::DimensionA => match &part.geometry {
            Geometry::Sphere { .. } => "Sphere radius ┬╖ meters".into(),
            Geometry::Cuboid { .. } => "Cuboid half-extent X ┬╖ meters".into(),
            Geometry::Capsule { .. } => "Capsule radius ┬╖ meters".into(),
        },
        InspectorField::DimensionB => match &part.geometry {
            Geometry::Cuboid { .. } => "Cuboid half-extent Y ┬╖ meters".into(),
            Geometry::Capsule { .. } => "Capsule cylindrical half-height ┬╖ meters".into(),
            Geometry::Sphere { .. } => "Not used by spheres".into(),
        },
        InspectorField::DimensionC => match &part.geometry {
            Geometry::Cuboid { .. } => "Cuboid half-extent Z ┬╖ meters".into(),
            Geometry::Sphere { .. } | Geometry::Capsule { .. } => {
                "Not used by this primitive".into()
            }
        },
        InspectorField::Density => "Density ┬╖ kg/m┬│".into(),
        InspectorField::Friction => "Friction coefficient".into(),
        InspectorField::Restitution => "Restitution ┬╖ 0ΓÇô1".into(),
        InspectorField::LinearDamping => "Linear damping".into(),
        InspectorField::AngularDamping => "Angular damping".into(),
    }
}

fn inspector_field_label_for_empty_selection(field: InspectorField) -> &'static str {
    match field {
        InspectorField::Name => "Part name",
        InspectorField::DimensionA => "Dimension A ┬╖ meters",
        InspectorField::DimensionB => "Dimension B ┬╖ meters",
        InspectorField::DimensionC => "Dimension C ┬╖ meters",
        InspectorField::Density => "Density ┬╖ kg/m┬│",
        InspectorField::Friction => "Friction coefficient",
        InspectorField::Restitution => "Restitution ┬╖ 0ΓÇô1",
        InspectorField::LinearDamping => "Linear damping",
        InspectorField::AngularDamping => "Angular damping",
    }
}

fn set_inspector_status(labels: &mut Query<&mut Text, With<InspectorStatusLabel>>, message: &str) {
    for mut label in labels.iter_mut() {
        label.0 = message.to_owned();
    }
}

fn update_part_inspector(
    selection: Res<PartSelection>,
    project: Res<DraftProject>,
    mut state: ResMut<PartInspectorState>,
    mut labels: Query<&mut Text, With<PartInspectorLabel>>,
    mut inputs: Query<(&mut EditableText, &InspectorFieldInput)>,
    mut field_labels: Query<(&InspectorFieldLabel, &mut Text)>,
) {
    let selected_part = selection
        .part_id
        .as_ref()
        .and_then(|id| project.organism.parts.iter().find(|part| part.id == *id));
    let selected_id = selection.part_id.clone();
    if state.selected_part_id == selected_id && state.selected_part.as_ref() == selected_part {
        return;
    }

    state.selected_part_id = selected_id;
    state.selected_part = selected_part.cloned();

    let details = selected_part.map(describe_body_part).unwrap_or_else(|| {
        "No part selected. Choose Select / inspect part, then click a body in the viewport."
            .to_owned()
    });
    for mut label in &mut labels {
        label.0.clone_from(&details);
    }
    for (mut input, marker) in &mut inputs {
        *input = EditableText::new(
            selected_part
                .map(|part| inspector_field_value(part, marker.0))
                .unwrap_or_default(),
        );
    }
    for (marker, mut label) in &mut field_labels {
        label.0 = selected_part
            .map(|part| inspector_field_label(part, marker.0))
            .unwrap_or_else(|| inspector_field_label_for_empty_selection(marker.0).to_owned());
    }
}

fn selected_actuator_controller<'a>(
    joint_id: Option<&str>,
    project: &'a DraftProject,
) -> Result<(&'a Actuator, &'a Behavior), &'static str> {
    let Some(joint_id) = joint_id else {
        return Err("Select a powered joint link to inspect its actuator and controller.");
    };
    if !project
        .organism
        .joints
        .iter()
        .any(|joint| joint.id == joint_id)
    {
        return Err("The selected joint is no longer in the organism.");
    }

    let mut actuators = project
        .organism
        .actuators
        .iter()
        .filter(|actuator| actuator.joint_id == joint_id);
    let Some(actuator) = actuators.next() else {
        return Err("This joint has no actuator. Use a powered hinge or powered slider.");
    };
    if actuators.next().is_some() {
        return Err("This joint has multiple actuators; actuator selection is not yet available.");
    }
    if actuator.kind == ActuatorKind::MuscleTendon {
        return Err("Muscle-tendon runtime actuation is not implemented yet.");
    }

    let mut behaviors = project.motion.behaviors.iter().filter(|behavior| {
        behavior.controller == ControllerKind::Oscillator
            && behavior.goals.iter().any(|goal| {
                matches!(
                    goal,
                    MotionGoal::OscillateActuator { actuator_id, .. }
                        if actuator_id == &actuator.id
                )
            })
    });
    let Some(behavior) = behaviors.next() else {
        return Err("No supported oscillator behavior is linked to this actuator.");
    };
    if behaviors.next().is_some() {
        return Err(
            "Multiple oscillator behaviors target this actuator; resolve duplicate controllers first.",
        );
    }
    let matching_goal_count = behavior
        .goals
        .iter()
        .filter(|goal| {
            matches!(
                goal,
                MotionGoal::OscillateActuator { actuator_id, .. }
                    if actuator_id == &actuator.id
            )
        })
        .count();
    if matching_goal_count != 1 {
        return Err("The selected behavior contains duplicate goals for this actuator.");
    }

    Ok((actuator, behavior))
}

fn update_actuator_inspector(
    selection: Res<JointSelection>,
    project: Res<DraftProject>,
    mut state: ResMut<ActuatorInspectorState>,
    mut descriptions: Query<&mut Text, With<ActuatorInspectorLabel>>,
    mut inputs: Query<(&mut EditableText, &ActuatorInspectorFieldInput)>,
    mut field_labels: Query<(&ActuatorInspectorFieldLabel, &mut Text)>,
) {
    let resolved = selected_actuator_controller(selection.joint_id.as_deref(), &project);
    let (selected_actuator, selected_behavior, availability_message) = match resolved {
        Ok((actuator, behavior)) => (
            Some(actuator.clone()),
            Some(behavior.clone()),
            String::new(),
        ),
        Err(message) => (None, None, message.to_owned()),
    };
    let selected_joint_id = selection.joint_id.clone();
    if state.selected_joint_id == selected_joint_id
        && state.selected_actuator == selected_actuator
        && state.selected_behavior == selected_behavior
        && state.availability_message == availability_message
    {
        return;
    }

    state.selected_joint_id = selected_joint_id;
    state.selected_actuator = selected_actuator;
    state.selected_behavior = selected_behavior;
    state.availability_message = availability_message;

    let description = match (&state.selected_actuator, &state.selected_behavior) {
        (Some(actuator), Some(behavior)) => describe_actuator_controller(actuator, behavior),
        _ => state.availability_message.clone(),
    };
    for mut label in &mut descriptions {
        label.0.clone_from(&description);
    }
    for (mut input, marker) in &mut inputs {
        *input = EditableText::new(match (&state.selected_actuator, &state.selected_behavior) {
            (Some(actuator), Some(behavior)) => {
                actuator_inspector_field_value(actuator, behavior, marker.0)
            }
            _ => String::new(),
        });
    }
    for (marker, mut label) in &mut field_labels {
        label.0 = match (&state.selected_actuator, &state.selected_behavior) {
            (Some(actuator), Some(_behavior)) => actuator_inspector_field_label(actuator, marker.0),
            _ => actuator_inspector_field_label_for_empty_selection(marker.0).to_owned(),
        };
    }
}

fn actuator_inspector_field_value(
    actuator: &Actuator,
    behavior: &Behavior,
    field: ActuatorInspectorField,
) -> String {
    match field {
        ActuatorInspectorField::ActuatorName => actuator.name.clone(),
        ActuatorInspectorField::MaxForce => actuator
            .max_force_n
            .map(|value| format!("{value:.3}"))
            .unwrap_or_default(),
        ActuatorInspectorField::MaxTorque => actuator
            .max_torque_nm
            .map(|value| format!("{value:.3}"))
            .unwrap_or_default(),
        ActuatorInspectorField::BehaviorName => behavior.name.clone(),
        ActuatorInspectorField::Frequency => oscillator_goal_values(behavior, &actuator.id)
            .map(|(frequency, _)| format!("{frequency:.3}"))
            .unwrap_or_default(),
        ActuatorInspectorField::Amplitude => oscillator_goal_values(behavior, &actuator.id)
            .map(|(_, amplitude)| format!("{amplitude:.3}"))
            .unwrap_or_default(),
    }
}

fn actuator_inspector_field_label(actuator: &Actuator, field: ActuatorInspectorField) -> String {
    match field {
        ActuatorInspectorField::ActuatorName => "Actuator name".into(),
        ActuatorInspectorField::MaxForce if actuator.kind == ActuatorKind::LinearMotor => {
            "Maximum force (N)".into()
        }
        ActuatorInspectorField::MaxForce => "Not used by angular motors".into(),
        ActuatorInspectorField::MaxTorque if actuator.kind == ActuatorKind::AngularMotor => {
            "Maximum torque (N m)".into()
        }
        ActuatorInspectorField::MaxTorque => "Not used by linear motors".into(),
        ActuatorInspectorField::BehaviorName => "Controller behavior name".into(),
        ActuatorInspectorField::Frequency => "Oscillation frequency (Hz)".into(),
        ActuatorInspectorField::Amplitude if actuator.kind == ActuatorKind::AngularMotor => {
            "Oscillation amplitude (rad)".into()
        }
        ActuatorInspectorField::Amplitude => "Oscillation amplitude (m)".into(),
    }
}

fn actuator_inspector_field_label_for_empty_selection(
    field: ActuatorInspectorField,
) -> &'static str {
    match field {
        ActuatorInspectorField::ActuatorName => "Actuator name",
        ActuatorInspectorField::MaxForce => "Maximum force (N)",
        ActuatorInspectorField::MaxTorque => "Maximum torque (N m)",
        ActuatorInspectorField::BehaviorName => "Controller behavior name",
        ActuatorInspectorField::Frequency => "Oscillation frequency (Hz)",
        ActuatorInspectorField::Amplitude => "Oscillation amplitude",
    }
}

fn oscillator_goal_values(behavior: &Behavior, actuator_id: &str) -> Option<(f32, f32)> {
    behavior.goals.iter().find_map(|goal| match goal {
        MotionGoal::OscillateActuator {
            actuator_id: goal_actuator_id,
            frequency_hz,
            amplitude,
        } if goal_actuator_id == actuator_id => Some((*frequency_hz, *amplitude)),
        _ => None,
    })
}

fn describe_actuator_controller(actuator: &Actuator, behavior: &Behavior) -> String {
    let (frequency_hz, amplitude) =
        oscillator_goal_values(behavior, &actuator.id).unwrap_or_default();
    let limit = match actuator.kind {
        ActuatorKind::AngularMotor => format!(
            "Maximum torque: {:.3} N m",
            actuator.max_torque_nm.unwrap_or_default()
        ),
        ActuatorKind::LinearMotor => {
            format!(
                "Maximum force: {:.3} N",
                actuator.max_force_n.unwrap_or_default()
            )
        }
        ActuatorKind::MuscleTendon => "Muscle-tendon runtime unavailable".into(),
    };
    let amplitude_unit = match actuator.kind {
        ActuatorKind::AngularMotor => "rad",
        ActuatorKind::LinearMotor => "m",
        ActuatorKind::MuscleTendon => "",
    };
    format!(
        "{} ({})\nActuator type: {:?}\nJoint: {}\n{}\nController: {:?}\nFrequency: {:.3} Hz\nAmplitude: {:.3} {}",
        actuator.name,
        actuator.id,
        actuator.kind,
        actuator.joint_id,
        limit,
        behavior.controller,
        frequency_hz,
        amplitude,
        amplitude_unit,
    )
}

fn apply_selected_actuator_controller_edit(
    selected_actuator_id: Option<&str>,
    selected_behavior_id: Option<&str>,
    project: &mut DraftProject,
    inputs: &Query<(&EditableText, &ActuatorInspectorFieldInput)>,
    status: &mut Query<&mut Text, With<ActuatorInspectorStatusLabel>>,
) {
    let Some(actuator_id) = selected_actuator_id else {
        set_actuator_inspector_status(
            status,
            "Select a supported powered joint before applying actuator changes.",
        );
        return;
    };
    let Some(behavior_id) = selected_behavior_id else {
        set_actuator_inspector_status(status, "No supported oscillator controller is selected.");
        return;
    };
    let Some(actuator_index) = project
        .organism
        .actuators
        .iter()
        .position(|actuator| actuator.id == actuator_id)
    else {
        set_actuator_inspector_status(status, "The selected actuator no longer exists.");
        return;
    };
    let Some(behavior_index) = project
        .motion
        .behaviors
        .iter()
        .position(|behavior| behavior.id == behavior_id)
    else {
        set_actuator_inspector_status(status, "The selected controller behavior no longer exists.");
        return;
    };

    let mut actuator = project.organism.actuators[actuator_index].clone();
    let mut behavior = project.motion.behaviors[behavior_index].clone();
    let Some(actuator_name) =
        actuator_inspector_field_text(inputs, ActuatorInspectorField::ActuatorName)
    else {
        set_actuator_inspector_status(status, "Enter a non-empty actuator name.");
        return;
    };
    let (max_force_n, max_torque_nm) = match actuator.kind {
        ActuatorKind::AngularMotor => {
            let Some(max_torque) =
                actuator_inspector_field_number(inputs, ActuatorInspectorField::MaxTorque)
            else {
                set_actuator_inspector_status(status, "Enter a positive maximum torque in N m.");
                return;
            };
            (None, Some(max_torque))
        }
        ActuatorKind::LinearMotor => {
            let Some(max_force) =
                actuator_inspector_field_number(inputs, ActuatorInspectorField::MaxForce)
            else {
                set_actuator_inspector_status(status, "Enter a positive maximum force in newtons.");
                return;
            };
            (Some(max_force), None)
        }
        ActuatorKind::MuscleTendon => {
            set_actuator_inspector_status(
                status,
                "Muscle-tendon runtime actuation is not implemented yet.",
            );
            return;
        }
    };
    let Some(behavior_name) =
        actuator_inspector_field_text(inputs, ActuatorInspectorField::BehaviorName)
    else {
        set_actuator_inspector_status(status, "Enter a non-empty controller behavior name.");
        return;
    };
    let Some(frequency_hz) =
        actuator_inspector_field_number(inputs, ActuatorInspectorField::Frequency)
    else {
        set_actuator_inspector_status(status, "Enter a positive oscillator frequency in Hz.");
        return;
    };
    let Some(amplitude) =
        actuator_inspector_field_number(inputs, ActuatorInspectorField::Amplitude)
    else {
        set_actuator_inspector_status(status, "Enter a non-negative oscillator amplitude.");
        return;
    };

    if let Err(error) = apply_actuator_controller_edit(
        &mut actuator,
        &mut behavior,
        ActuatorControllerEdit {
            actuator_name,
            max_force_n,
            max_torque_nm,
            behavior_name,
            frequency_hz,
            amplitude,
        },
    ) {
        set_actuator_inspector_status(status, &error.to_string());
        return;
    }

    project.organism.actuators[actuator_index] = actuator;
    project.motion.behaviors[behavior_index] = behavior;
    set_actuator_inspector_status(
        status,
        "Applied actuator limits and oscillator parameters to the live controller.",
    );
}

fn actuator_inspector_field_text(
    inputs: &Query<(&EditableText, &ActuatorInspectorFieldInput)>,
    field: ActuatorInspectorField,
) -> Option<String> {
    inputs
        .iter()
        .find(|(_, marker)| marker.0 == field)
        .map(|(input, _)| input.value().to_string())
}

fn actuator_inspector_field_number(
    inputs: &Query<(&EditableText, &ActuatorInspectorFieldInput)>,
    field: ActuatorInspectorField,
) -> Option<f32> {
    actuator_inspector_field_text(inputs, field)?
        .trim()
        .parse::<f32>()
        .ok()
}

fn set_actuator_inspector_status(
    labels: &mut Query<&mut Text, With<ActuatorInspectorStatusLabel>>,
    message: &str,
) {
    for mut label in labels.iter_mut() {
        label.0 = message.to_owned();
    }
}

fn update_joint_inspector(
    selection: Res<JointSelection>,
    project: Res<DraftProject>,
    mut state: ResMut<JointInspectorState>,
    mut descriptions: Query<&mut Text, With<JointInspectorLabel>>,
    mut inputs: Query<(&mut EditableText, &JointInspectorFieldInput)>,
    mut field_labels: Query<(&JointInspectorFieldLabel, &mut Text)>,
) {
    let selected_joint = selection
        .joint_id
        .as_ref()
        .and_then(|id| project.organism.joints.iter().find(|joint| joint.id == *id));
    let selected_id = selection.joint_id.clone();
    if state.selected_joint_id == selected_id && state.selected_joint.as_ref() == selected_joint {
        return;
    }

    state.selected_joint_id = selected_id;
    state.selected_joint = selected_joint.cloned();

    let description = selected_joint.map(describe_joint).unwrap_or_else(|| {
        "No joint selected. Choose Select / inspect part, then click a connection line.".to_owned()
    });
    for mut label in &mut descriptions {
        label.0.clone_from(&description);
    }
    for (mut input, marker) in &mut inputs {
        *input = EditableText::new(
            selected_joint
                .map(|joint| joint_inspector_field_value(joint, marker.0))
                .unwrap_or_default(),
        );
    }
    for (marker, mut label) in &mut field_labels {
        label.0 = selected_joint
            .map(|joint| joint_inspector_field_label(joint, marker.0))
            .unwrap_or_else(|| {
                joint_inspector_field_label_for_empty_selection(marker.0).to_owned()
            });
    }
}

fn joint_inspector_field_value(joint: &Joint, field: JointInspectorField) -> String {
    match field {
        JointInspectorField::Name => joint.name.clone(),
        JointInspectorField::ParentAnchorX => format!("{:.4}", joint.anchor_parent_m.x),
        JointInspectorField::ParentAnchorY => format!("{:.4}", joint.anchor_parent_m.y),
        JointInspectorField::ParentAnchorZ => format!("{:.4}", joint.anchor_parent_m.z),
        JointInspectorField::ChildAnchorX => format!("{:.4}", joint.anchor_child_m.x),
        JointInspectorField::ChildAnchorY => format!("{:.4}", joint.anchor_child_m.y),
        JointInspectorField::ChildAnchorZ => format!("{:.4}", joint.anchor_child_m.z),
        JointInspectorField::RestLength => joint
            .rest_length_m
            .map(|length| format!("{length:.4}"))
            .unwrap_or_default(),
    }
}

fn joint_inspector_field_label(joint: &Joint, field: JointInspectorField) -> String {
    match field {
        JointInspectorField::Name => "Joint name".into(),
        JointInspectorField::ParentAnchorX => "Parent local anchor X (m)".into(),
        JointInspectorField::ParentAnchorY => "Parent local anchor Y (m)".into(),
        JointInspectorField::ParentAnchorZ => "Parent local anchor Z (m)".into(),
        JointInspectorField::ChildAnchorX => "Child local anchor X (m)".into(),
        JointInspectorField::ChildAnchorY => "Child local anchor Y (m)".into(),
        JointInspectorField::ChildAnchorZ => "Child local anchor Z (m)".into(),
        JointInspectorField::RestLength if joint.kind == JointKind::Distance => {
            "Distance rest length (m)".into()
        }
        JointInspectorField::RestLength => "Not used by this joint".into(),
    }
}

fn joint_inspector_field_label_for_empty_selection(field: JointInspectorField) -> &'static str {
    match field {
        JointInspectorField::Name => "Joint name",
        JointInspectorField::ParentAnchorX => "Parent local anchor X (m)",
        JointInspectorField::ParentAnchorY => "Parent local anchor Y (m)",
        JointInspectorField::ParentAnchorZ => "Parent local anchor Z (m)",
        JointInspectorField::ChildAnchorX => "Child local anchor X (m)",
        JointInspectorField::ChildAnchorY => "Child local anchor Y (m)",
        JointInspectorField::ChildAnchorZ => "Child local anchor Z (m)",
        JointInspectorField::RestLength => "Distance rest length (m)",
    }
}

fn describe_joint(joint: &Joint) -> String {
    let rest_length = joint
        .rest_length_m
        .map(|length| format!("Rest length: {length:.3} m"))
        .unwrap_or_else(|| "Rest length: not applicable".into());
    format!(
        "{} ({})\nType: {:?}\nConnects: {} → {}\nParent anchor: ({:.3}, {:.3}, {:.3}) m\nChild anchor: ({:.3}, {:.3}, {:.3}) m\n{}",
        joint.name,
        joint.id,
        joint.kind,
        joint.parent_part_id,
        joint.child_part_id,
        joint.anchor_parent_m.x,
        joint.anchor_parent_m.y,
        joint.anchor_parent_m.z,
        joint.anchor_child_m.x,
        joint.anchor_child_m.y,
        joint.anchor_child_m.z,
        rest_length,
    )
}

fn set_joint_inspector_status(
    labels: &mut Query<&mut Text, With<JointInspectorStatusLabel>>,
    message: &str,
) {
    for mut label in labels.iter_mut() {
        label.0 = message.to_owned();
    }
}

fn describe_body_part(part: &BodyPart) -> String {
    let geometry = match &part.geometry {
        Geometry::Sphere { radius_m } => format!("Sphere ┬╖ radius {radius_m:.3} m"),
        Geometry::Cuboid { half_extents_m } => format!(
            "Cuboid ┬╖ half-extents ({:.3}, {:.3}, {:.3}) m",
            half_extents_m.x, half_extents_m.y, half_extents_m.z
        ),
        Geometry::Capsule {
            radius_m,
            half_height_m,
        } => format!("Capsule ┬╖ radius {radius_m:.3} m ┬╖ half-height {half_height_m:.3} m"),
    };

    let physics = match &part.physics {
        BodyPhysics::Rigid(properties) => format!(
            "Rigid body\nMass: {:.3} kg\nDensity: {:.1} kg/m┬│\nFriction: {:.2}\nRestitution: {:.2}\nLinear damping: {:.3}\nAngular damping: {:.3}",
            properties.mass_kg,
            properties.density_kg_m3,
            properties.friction,
            properties.restitution,
            properties.linear_damping,
            properties.angular_damping
        ),
        BodyPhysics::Soft(properties) => format!(
            "Soft body\nDensity: {:.1} kg/m┬│\nElastic modulus: {:.1} Pa\nDamping ratio: {:.3}\nResolution: {}",
            properties.density_kg_m3,
            properties.elastic_modulus_pa,
            properties.damping_ratio,
            properties.resolution
        ),
    };

    format!(
        "{} ({})\nGeometry: {}\nRest position: ({:.3}, {:.3}, {:.3}) m\n{}",
        part.name,
        part.id,
        geometry,
        part.transform.translation_m.x,
        part.transform.translation_m.y,
        part.transform.translation_m.z,
        physics
    )
}

fn draw_selected_part_outline(
    mut gizmos: Gizmos,
    selection: Res<PartSelection>,
    parts: Query<(&GlobalTransform, &PartTag)>,
) {
    let Some(selected_id) = selection.part_id.as_deref() else {
        return;
    };

    if let Some((transform, tag)) = parts.iter().find(|(_, tag)| tag.id == selected_id) {
        gizmos.sphere(
            transform.translation(),
            tag.pick_radius * 1.16,
            Color::srgb(1.0, 0.82, 0.28),
        );
    }
}

fn draw_joint_links(
    mut gizmos: Gizmos,
    part_query: Query<(&GlobalTransform, &PartTag)>,
    project: Res<DraftProject>,
    selection: Res<JointSelection>,
) {
    for joint in &project.organism.joints {
        let parent = part_query
            .iter()
            .find(|(_, tag)| tag.id == joint.parent_part_id)
            .map(|(transform, _)| transform.translation());
        let child = part_query
            .iter()
            .find(|(_, tag)| tag.id == joint.child_part_id)
            .map(|(transform, _)| transform.translation());

        if let (Some(parent), Some(child)) = (parent, child) {
            let color = if selection.joint_id.as_deref() == Some(joint.id.as_str()) {
                Color::srgb(1.0, 0.88, 0.3)
            } else {
                Color::srgb(0.95, 0.78, 0.32)
            };
            gizmos.line(parent, child, color);
        }
    }
}

fn ray_sphere_distance(
    ray_origin: Vec3,
    ray_direction: Vec3,
    sphere_center: Vec3,
    sphere_radius: f32,
) -> Option<f32> {
    let offset = ray_origin - sphere_center;
    let b = offset.dot(ray_direction);
    let c = offset.length_squared() - sphere_radius * sphere_radius;
    let discriminant = b * b - c;
    if discriminant < 0.0 {
        return None;
    }

    let near = -b - discriminant.sqrt();
    let far = -b + discriminant.sqrt();
    if near >= 0.0 {
        Some(near)
    } else if far >= 0.0 {
        Some(far)
    } else {
        None
    }
}

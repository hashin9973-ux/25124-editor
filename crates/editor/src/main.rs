use avian3d::prelude::*;
use bevy::prelude::*;
use organism_core::{
    BodyPart, BodyPhysics, Geometry, Joint, JointKind, MotionDefinition, Organism,
    RigidBodyProperties, Transform3, Vec3 as DataVec3,
};
use organism_serialization::save_project;
use std::path::Path;

const PART_RADIUS: f32 = 0.3;
const EXPORT_DIRECTORY: &str = "exports";

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .insert_resource(ClearColor(Color::srgb(0.025, 0.03, 0.045)))
        .init_resource::<DraftProject>()
        .init_resource::<EditorTool>()
        .add_systems(Startup, (setup_scene, setup_editor_panel))
        .add_systems(Update, (handle_editor_buttons, edit_from_viewport))
        .run();
}

#[derive(Resource)]
struct DraftProject {
    organism: Organism,
    motion: MotionDefinition,
    next_part_id: u64,
    next_joint_id: u64,
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
            pending_joint_part: None,
        }
    }
}

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq)]
enum EditorTool {
    #[default]
    PlaceSphere,
    ConnectHinge,
}

#[derive(Component)]
struct PartTag {
    id: String,
}

#[derive(Component, Clone, Copy)]
enum EditorButtonAction {
    PlaceSphere,
    ConnectHinge,
    Save,
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
        Transform::from_xyz(9.0, 8.0, 12.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

fn setup_editor_panel(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(16.0),
                left: Val::Px(16.0),
                width: Val::Px(250.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(8.0),
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
                Text::new("Place parts, connect joints, then simulate."),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(Color::srgb(0.7, 0.74, 0.8)),
            ));

            panel
                .spawn((
                    Button,
                    Node {
                        width: Val::Px(224.0),
                        height: Val::Px(38.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.12, 0.2, 0.3)),
                    EditorButtonAction::PlaceSphere,
                ))
                .with_children(|button| {
                    button.spawn((
                        Text::new("Place spheres"),
                        TextFont {
                            font_size: FontSize::Px(13.0),
                            ..default()
                        },
                        TextColor(Color::WHITE),
                    ));
                });

            panel
                .spawn((
                    Button,
                    Node {
                        width: Val::Px(224.0),
                        height: Val::Px(38.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.12, 0.2, 0.3)),
                    EditorButtonAction::ConnectHinge,
                ))
                .with_children(|button| {
                    button.spawn((
                        Text::new("Connect hinge"),
                        TextFont {
                            font_size: FontSize::Px(13.0),
                            ..default()
                        },
                        TextColor(Color::WHITE),
                    ));
                });

            panel
                .spawn((
                    Button,
                    Node {
                        width: Val::Px(224.0),
                        height: Val::Px(38.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.16, 0.34, 0.23)),
                    EditorButtonAction::Save,
                ))
                .with_children(|button| {
                    button.spawn((
                        Text::new("Save organism + motion"),
                        TextFont {
                            font_size: FontSize::Px(13.0),
                            ..default()
                        },
                        TextColor(Color::WHITE),
                    ));
                });

            panel.spawn((
                Text::new("Place: left-click the ground\nConnect: click two parts\nExport: exports/*.json"),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(Color::srgb(0.72, 0.76, 0.82)),
            ));
        });
}

fn handle_editor_buttons(
    interactions: Query<(&Interaction, &EditorButtonAction), (Changed<Interaction>, With<Button>)>,
    mut tool: ResMut<EditorTool>,
    project: Res<DraftProject>,
) {
    for (interaction, action) in &interactions {
        if *interaction != Interaction::Pressed {
            continue;
        }

        match action {
            EditorButtonAction::PlaceSphere => {
                *tool = EditorTool::PlaceSphere;
                info!("Editor tool: place spheres");
            }
            EditorButtonAction::ConnectHinge => {
                *tool = EditorTool::ConnectHinge;
                info!("Editor tool: connect hinge; click two body parts");
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
        }
    }
}

fn edit_from_viewport(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    ui_interactions: Query<&Interaction, With<Button>>,
    part_query: Query<(Entity, &Transform, &PartTag)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut project: ResMut<DraftProject>,
    tool: Res<EditorTool>,
) {
    if !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    if ui_interactions
        .iter()
        .any(|interaction| *interaction != Interaction::None)
    {
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
    let direction = ray.direction.as_vec3();

    match *tool {
        EditorTool::PlaceSphere => {
            if direction.y.abs() < 1.0e-5 {
                return;
            }
            let distance = -ray.origin.y / direction.y;
            if distance <= 0.0 {
                return;
            }
            let position = ray.origin + direction * distance + Vec3::Y * PART_RADIUS;

            let sequence = project.next_part_id;
            project.next_part_id += 1;
            let part_id = format!("part-{sequence:04}");
            let entity = commands
                .spawn((
                    Mesh3d(meshes.add(Sphere::new(PART_RADIUS))),
                    MeshMaterial3d(materials.add(Color::srgb(0.24, 0.68, 0.92))),
                    Transform::from_translation(position),
                    RigidBody::Dynamic,
                    Collider::sphere(PART_RADIUS),
                    MassPropertiesBundle::from_shape(&Sphere::new(PART_RADIUS), 1.0),
                    PartTag {
                        id: part_id.clone(),
                    },
                ))
                .id();

            project.organism.parts.push(BodyPart {
                id: part_id.clone(),
                name: format!("Part {sequence}"),
                geometry: Geometry::Sphere {
                    radius_m: PART_RADIUS,
                },
                transform: Transform3::at(DataVec3::new(position.x, position.y, position.z)),
                physics: BodyPhysics::Rigid(RigidBodyProperties {
                    mass_kg: 1.0,
                    ..default()
                }),
            });

            debug!("Placed {part_id} as entity {entity:?}");
        }
        EditorTool::ConnectHinge => {
            let picked = part_query
                .iter()
                .filter_map(|(entity, transform, tag)| {
                    ray_sphere_distance(
                        ray.origin,
                        direction,
                        transform.translation,
                        PART_RADIUS * 1.15,
                    )
                    .map(|distance| (distance, entity, tag.id.clone(), transform.translation))
                })
                .min_by(|left, right| left.0.total_cmp(&right.0));

            let Some((_, child_entity, child_id, child_position)) = picked else {
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

            let Some((parent_entity, parent_position)) = part_query
                .iter()
                .find(|(_, _, tag)| tag.id == parent_id)
                .map(|(entity, transform, _)| (entity, transform.translation))
            else {
                warn!("Selected parent part {parent_id} no longer exists");
                return;
            };

            let world_anchor = (parent_position + child_position) * 0.5;
            commands.spawn(
                RevoluteJoint::new(parent_entity, child_entity)
                    .with_hinge_axis(Vec3::Z)
                    .with_local_anchor1(world_anchor - parent_position)
                    .with_local_anchor2(world_anchor - child_position),
            );

            let joint_sequence = project.next_joint_id;
            project.next_joint_id += 1;
            project.organism.joints.push(Joint {
                id: format!("joint-{joint_sequence:04}"),
                name: format!("{parent_id} to {child_id} hinge"),
                kind: JointKind::Hinge,
                parent_part_id: parent_id,
                child_part_id: child_id,
                anchor_parent_m: DataVec3::new(
                    world_anchor.x - parent_position.x,
                    world_anchor.y - parent_position.y,
                    world_anchor.z - parent_position.z,
                ),
                anchor_child_m: DataVec3::new(
                    world_anchor.x - child_position.x,
                    world_anchor.y - child_position.y,
                    world_anchor.z - child_position.z,
                ),
                limits: None,
            });
            info!("Created hinge joint between two body parts");
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

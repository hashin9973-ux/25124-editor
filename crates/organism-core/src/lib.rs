//! Engine-independent data contracts for physical organisms.
//!
//! This crate deliberately has no Bevy or Avian dependency. Serialized IDs,
//! geometry, physical properties, constraints, and behavior remain portable.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const ORGANISM_FORMAT: &str = "organism";
pub const MOTION_FORMAT: &str = "organism_motion";
pub const SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    pub translation_m: Vec3,
    /// Quaternion stored as [x, y, z, w].
    pub rotation_xyzw: [f32; 4],
    pub scale: Vec3,
}

impl Default for Transform3 {
    fn default() -> Self {
        Self {
            translation_m: Vec3::ZERO,
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            scale: Vec3::new(1.0, 1.0, 1.0),
        }
    }
}

impl Transform3 {
    pub fn at(translation_m: Vec3) -> Self {
        Self {
            translation_m,
            ..Self::default()
        }
    }

    fn is_valid(self) -> bool {
        self.translation_m.is_finite()
            && self.scale.is_finite()
            && self.rotation_xyzw.iter().all(|value| value.is_finite())
            && self
                .rotation_xyzw
                .iter()
                .map(|value| value * value)
                .sum::<f32>()
                > f32::EPSILON
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum Geometry {
    Sphere { radius_m: f32 },
    Cuboid { half_extents_m: Vec3 },
    Capsule { radius_m: f32, half_height_m: f32 },
}

impl Geometry {
    /// Returns the primitive's volume in cubic meters.
    pub fn volume_m3(&self) -> f32 {
        match self {
            Self::Sphere { radius_m } => (4.0 / 3.0) * std::f32::consts::PI * radius_m.powi(3),
            Self::Cuboid { half_extents_m } => {
                8.0 * half_extents_m.x * half_extents_m.y * half_extents_m.z
            }
            Self::Capsule {
                radius_m,
                half_height_m,
            } => {
                let cylinder = std::f32::consts::PI * radius_m.powi(2) * (2.0 * half_height_m);
                let hemispheres = (4.0 / 3.0) * std::f32::consts::PI * radius_m.powi(3);
                cylinder + hemispheres
            }
        }
    }

    /// Estimates mass from the primitive volume and a material density in kg/m³.
    ///
    /// Returns `None` for invalid geometry, invalid density, or an unrepresentable result.
    pub fn mass_kg_at_density(&self, density_kg_m3: f32) -> Option<f32> {
        if !self.is_valid() || !density_kg_m3.is_finite() || density_kg_m3 <= 0.0 {
            return None;
        }
        let mass_kg = self.volume_m3() * density_kg_m3;
        (mass_kg.is_finite() && mass_kg > 0.0).then_some(mass_kg)
    }

    fn is_valid(&self) -> bool {
        match self {
            Self::Sphere { radius_m } => radius_m.is_finite() && *radius_m > 0.0,
            Self::Cuboid { half_extents_m } => {
                half_extents_m.is_finite()
                    && half_extents_m.x > 0.0
                    && half_extents_m.y > 0.0
                    && half_extents_m.z > 0.0
            }
            Self::Capsule {
                radius_m,
                half_height_m,
            } => {
                radius_m.is_finite()
                    && half_height_m.is_finite()
                    && *radius_m > 0.0
                    && *half_height_m >= 0.0
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyProperties {
    pub mass_kg: f32,
    /// Material density used by the editor to estimate mass from primitive volume.
    #[serde(default = "default_tissue_density_kg_m3")]
    pub density_kg_m3: f32,
    pub friction: f32,
    pub restitution: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

fn default_tissue_density_kg_m3() -> f32 {
    1000.0
}

impl Default for RigidBodyProperties {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            density_kg_m3: default_tissue_density_kg_m3(),
            friction: 0.7,
            restitution: 0.0,
            linear_damping: 0.05,
            angular_damping: 0.05,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SoftBodyProperties {
    pub density_kg_m3: f32,
    pub elastic_modulus_pa: f32,
    pub damping_ratio: f32,
    pub resolution: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", content = "properties", rename_all = "snake_case")]
pub enum BodyPhysics {
    Rigid(RigidBodyProperties),
    Soft(SoftBodyProperties),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyPart {
    pub id: String,
    pub name: String,
    pub geometry: Geometry,
    pub transform: Transform3,
    pub physics: BodyPhysics,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointKind {
    Fixed,
    Ball,
    Hinge,
    Slider,
    Distance,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointLimits {
    pub min: f32,
    pub max: f32,
    /// Radians for angular joints; meters for linear joints.
    pub unit: JointLimitUnit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointLimitUnit {
    Radians,
    Meters,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub id: String,
    pub name: String,
    pub kind: JointKind,
    pub parent_part_id: String,
    pub child_part_id: String,
    /// Attachment points expressed in each part's local coordinates, in meters.
    pub anchor_parent_m: Vec3,
    pub anchor_child_m: Vec3,
    pub limits: Option<JointLimits>,
    /// Used by distance joints. The value is measured between the two body-local anchors.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_length_m: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActuatorKind {
    AngularMotor,
    LinearMotor,
    MuscleTendon,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Actuator {
    pub id: String,
    pub name: String,
    pub joint_id: String,
    pub kind: ActuatorKind,
    pub max_force_n: Option<f32>,
    pub max_torque_nm: Option<f32>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Units {
    pub length: String,
    pub mass: String,
    pub time: String,
}

impl Default for Units {
    fn default() -> Self {
        Self {
            length: "meter".into(),
            mass: "kilogram".into(),
            time: "second".into(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Organism {
    pub format: String,
    pub schema_version: u32,
    pub id: String,
    pub name: String,
    pub units: Units,
    pub parts: Vec<BodyPart>,
    pub joints: Vec<Joint>,
    pub actuators: Vec<Actuator>,
}

impl Organism {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            format: ORGANISM_FORMAT.into(),
            schema_version: SCHEMA_VERSION,
            id: "organism-1".into(),
            name: name.into(),
            units: Units::default(),
            parts: Vec::new(),
            joints: Vec::new(),
            actuators: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.format != ORGANISM_FORMAT {
            errors.push(format!("format must be \"{ORGANISM_FORMAT}\""));
        }
        if self.schema_version != SCHEMA_VERSION {
            errors.push(format!(
                "unsupported organism schema version: {}",
                self.schema_version
            ));
        }
        if self.id.trim().is_empty() {
            errors.push("organism id cannot be empty".into());
        }
        if self.name.trim().is_empty() {
            errors.push("organism name cannot be empty".into());
        }

        let mut part_ids = BTreeSet::new();
        for part in &self.parts {
            if part.id.trim().is_empty() {
                errors.push("body part id cannot be empty".into());
            } else if !part_ids.insert(part.id.as_str()) {
                errors.push(format!("duplicate body part id: {}", part.id));
            }
            if !part.geometry.is_valid() {
                errors.push(format!("body part {} has invalid geometry", part.id));
            }
            if !part.transform.is_valid() {
                errors.push(format!("body part {} has an invalid transform", part.id));
            }
            match &part.physics {
                BodyPhysics::Rigid(properties) => {
                    if !properties.mass_kg.is_finite() || properties.mass_kg <= 0.0 {
                        errors.push(format!("body part {} has invalid mass", part.id));
                    }
                    if !properties.density_kg_m3.is_finite() || properties.density_kg_m3 <= 0.0 {
                        errors.push(format!("body part {} has invalid density", part.id));
                    }
                    if !properties.friction.is_finite() || properties.friction < 0.0 {
                        errors.push(format!("body part {} has invalid friction", part.id));
                    }
                    if !properties.restitution.is_finite()
                        || !(0.0..=1.0).contains(&properties.restitution)
                    {
                        errors.push(format!("body part {} has invalid restitution", part.id));
                    }
                    if !properties.linear_damping.is_finite()
                        || properties.linear_damping < 0.0
                        || !properties.angular_damping.is_finite()
                        || properties.angular_damping < 0.0
                    {
                        errors.push(format!("body part {} has invalid damping", part.id));
                    }
                }
                BodyPhysics::Soft(properties) => {
                    if !properties.density_kg_m3.is_finite()
                        || properties.density_kg_m3 <= 0.0
                        || !properties.elastic_modulus_pa.is_finite()
                        || properties.elastic_modulus_pa <= 0.0
                        || !properties.damping_ratio.is_finite()
                        || properties.damping_ratio < 0.0
                        || properties.resolution < 2
                    {
                        errors.push(format!(
                            "body part {} has invalid soft-body properties",
                            part.id
                        ));
                    }
                }
            }
        }

        let mut joint_ids = BTreeSet::new();
        for joint in &self.joints {
            if joint.id.trim().is_empty() {
                errors.push("joint id cannot be empty".into());
            } else if !joint_ids.insert(joint.id.as_str()) {
                errors.push(format!("duplicate joint id: {}", joint.id));
            }
            if joint.parent_part_id == joint.child_part_id {
                errors.push(format!(
                    "joint {} cannot connect a part to itself",
                    joint.id
                ));
            }
            if !part_ids.contains(joint.parent_part_id.as_str()) {
                errors.push(format!(
                    "joint {} references missing parent part {}",
                    joint.id, joint.parent_part_id
                ));
            }
            if !part_ids.contains(joint.child_part_id.as_str()) {
                errors.push(format!(
                    "joint {} references missing child part {}",
                    joint.id, joint.child_part_id
                ));
            }
            if !joint.anchor_parent_m.is_finite() || !joint.anchor_child_m.is_finite() {
                errors.push(format!("joint {} has a non-finite anchor", joint.id));
            }
            if let Some(limits) = joint.limits {
                if !limits.min.is_finite() || !limits.max.is_finite() || limits.min > limits.max {
                    errors.push(format!("joint {} has invalid limits", joint.id));
                }
            }
            if let Some(rest_length_m) = joint.rest_length_m {
                if !rest_length_m.is_finite() || rest_length_m <= 0.0 {
                    errors.push(format!("joint {} has invalid rest length", joint.id));
                }
                if joint.kind != JointKind::Distance {
                    errors.push(format!(
                        "joint {} defines a rest length but is not a distance joint",
                        joint.id
                    ));
                }
            }
            if joint.kind == JointKind::Distance && joint.rest_length_m.is_none() {
                errors.push(format!(
                    "distance joint {} is missing its rest length",
                    joint.id
                ));
            }
        }

        let mut actuator_ids = BTreeSet::new();
        for actuator in &self.actuators {
            if actuator.id.trim().is_empty() {
                errors.push("actuator id cannot be empty".into());
            } else if !actuator_ids.insert(actuator.id.as_str()) {
                errors.push(format!("duplicate actuator id: {}", actuator.id));
            }
            if !joint_ids.contains(actuator.joint_id.as_str()) {
                errors.push(format!(
                    "actuator {} references missing joint {}",
                    actuator.id, actuator.joint_id
                ));
            }
            if actuator
                .max_force_n
                .is_some_and(|value| !value.is_finite() || value <= 0.0)
            {
                errors.push(format!("actuator {} has invalid max force", actuator.id));
            }
            if actuator
                .max_torque_nm
                .is_some_and(|value| !value.is_finite() || value <= 0.0)
            {
                errors.push(format!("actuator {} has invalid max torque", actuator.id));
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControllerKind {
    Feedback,
    Oscillator,
    ContactDriven,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "goal", rename_all = "snake_case")]
pub enum MotionGoal {
    MoveAtVelocity {
        target_speed_mps: f32,
    },
    MaintainBalance {
        target_up: Vec3,
    },
    OscillateActuator {
        actuator_id: String,
        frequency_hz: f32,
        amplitude: f32,
    },
    /// Selects a generalized motor target from the current environment-contact state.
    /// Targets are radians for angular motors and meters for linear motors.
    ContactDrivenActuator {
        sensor_id: String,
        actuator_id: String,
        contact_target_position: f32,
        free_target_position: f32,
    },
}

/// A persistent declaration that a rigid body part contributes physical
/// environment-contact readings to motion controllers.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContactSensor {
    pub id: String,
    pub part_id: String,
}

/// Transient contact data extracted from the current physics step.
///
/// Vectors are expressed in world coordinates. Impulse is measured in N·s
/// (equivalently kg·m/s); approach speed is measured in m/s.
/// This runtime reading is intentionally not serialized as organism configuration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactReading {
    /// Number of physical contact points against configured environment surfaces.
    pub contact_count: u32,
    /// Mean unit contact normal pointing from the environment into the sensed body.
    pub support_normal_world: Vec3,
    /// Sum of absolute normal impulses over the current contact points, in N·s.
    pub total_normal_impulse_ns: f32,
    /// Largest positive closing speed at a contact point, in m/s.
    pub max_approach_speed_mps: f32,
}

impl ContactReading {
    pub fn is_in_contact(self) -> bool {
        self.contact_count > 0
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Behavior {
    pub id: String,
    pub name: String,
    pub controller: ControllerKind,
    pub goals: Vec<MotionGoal>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionDefinition {
    pub format: String,
    pub schema_version: u32,
    pub organism_id: String,
    /// Persistent sensor-to-body-part bindings. Runtime readings are not stored here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contact_sensors: Vec<ContactSensor>,
    pub behaviors: Vec<Behavior>,
}

impl MotionDefinition {
    pub fn new(organism_id: impl Into<String>) -> Self {
        Self {
            format: MOTION_FORMAT.into(),
            schema_version: SCHEMA_VERSION,
            organism_id: organism_id.into(),
            contact_sensors: Vec::new(),
            behaviors: Vec::new(),
        }
    }

    pub fn validate_for(&self, organism: &Organism) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.format != MOTION_FORMAT {
            errors.push(format!("format must be \"{MOTION_FORMAT}\""));
        }
        if self.schema_version != SCHEMA_VERSION {
            errors.push(format!(
                "unsupported motion schema version: {}",
                self.schema_version
            ));
        }
        if self.organism_id != organism.id {
            errors.push("motion definition references a different organism id".into());
        }

        let actuator_ids: BTreeSet<&str> = organism
            .actuators
            .iter()
            .map(|actuator| actuator.id.as_str())
            .collect();
        let part_ids: BTreeSet<&str> = organism.parts.iter().map(|part| part.id.as_str()).collect();
        let mut sensor_ids = BTreeSet::new();
        let mut sensor_part_ids = BTreeSet::new();
        for sensor in &self.contact_sensors {
            if sensor.id.trim().is_empty() {
                errors.push("contact sensor id cannot be empty".into());
            } else if !sensor_ids.insert(sensor.id.as_str()) {
                errors.push(format!("duplicate contact sensor id: {}", sensor.id));
            }
            if !sensor_part_ids.insert(sensor.part_id.as_str()) {
                errors.push(format!(
                    "multiple contact sensors reference body part {}",
                    sensor.part_id
                ));
            }
            if !part_ids.contains(sensor.part_id.as_str()) {
                errors.push(format!(
                    "contact sensor {} references missing body part {}",
                    sensor.id, sensor.part_id
                ));
            } else if let Some(part) = organism.parts.iter().find(|part| part.id == sensor.part_id)
                && !matches!(&part.physics, BodyPhysics::Rigid(_))
            {
                errors.push(format!(
                    "contact sensor {} requires a rigid body part",
                    sensor.id
                ));
            }
        }

        let mut behavior_ids = BTreeSet::new();
        let mut actuator_controller_ids = BTreeSet::new();
        for behavior in &self.behaviors {
            if behavior.id.trim().is_empty() {
                errors.push("behavior id cannot be empty".into());
            } else if !behavior_ids.insert(behavior.id.as_str()) {
                errors.push(format!("duplicate behavior id: {}", behavior.id));
            }
            for goal in &behavior.goals {
                match goal {
                    MotionGoal::MoveAtVelocity { target_speed_mps } => {
                        if !target_speed_mps.is_finite() || *target_speed_mps < 0.0 {
                            errors
                                .push(format!("behavior {} has invalid target speed", behavior.id));
                        }
                    }
                    MotionGoal::MaintainBalance { target_up } => {
                        if !target_up.is_finite() {
                            errors.push(format!(
                                "behavior {} has invalid balance direction",
                                behavior.id
                            ));
                        }
                    }
                    MotionGoal::OscillateActuator {
                        actuator_id,
                        frequency_hz,
                        amplitude,
                    } => {
                        if !actuator_ids.contains(actuator_id.as_str()) {
                            errors.push(format!(
                                "behavior {} references missing actuator {}",
                                behavior.id, actuator_id
                            ));
                        } else if !actuator_controller_ids.insert(actuator_id.as_str()) {
                            errors.push(format!(
                                "multiple controller goals target actuator {}",
                                actuator_id
                            ));
                        }
                        if !frequency_hz.is_finite() || *frequency_hz <= 0.0 {
                            errors.push(format!("behavior {} has invalid frequency", behavior.id));
                        }
                        if !amplitude.is_finite() || *amplitude < 0.0 {
                            errors.push(format!("behavior {} has invalid amplitude", behavior.id));
                        }
                    }
                    MotionGoal::ContactDrivenActuator {
                        sensor_id,
                        actuator_id,
                        contact_target_position,
                        free_target_position,
                    } => {
                        if behavior.controller != ControllerKind::ContactDriven {
                            errors.push(format!(
                                "behavior {} uses a contact-driven goal with a non-contact controller",
                                behavior.id
                            ));
                        }
                        if !sensor_ids.contains(sensor_id.as_str()) {
                            errors.push(format!(
                                "behavior {} references missing contact sensor {}",
                                behavior.id, sensor_id
                            ));
                        }
                        if !actuator_ids.contains(actuator_id.as_str()) {
                            errors.push(format!(
                                "behavior {} references missing actuator {}",
                                behavior.id, actuator_id
                            ));
                        } else if !actuator_controller_ids.insert(actuator_id.as_str()) {
                            errors.push(format!(
                                "multiple controller goals target actuator {}",
                                actuator_id
                            ));
                        } else if let Some(actuator) = organism
                            .actuators
                            .iter()
                            .find(|actuator| actuator.id == *actuator_id)
                        {
                            let joint = organism
                                .joints
                                .iter()
                                .find(|joint| joint.id == actuator.joint_id);
                            let supported_joint = match actuator.kind {
                                ActuatorKind::AngularMotor => {
                                    joint.is_some_and(|joint| joint.kind == JointKind::Hinge)
                                }
                                ActuatorKind::LinearMotor => {
                                    joint.is_some_and(|joint| joint.kind == JointKind::Slider)
                                }
                                ActuatorKind::MuscleTendon => false,
                            };
                            if !supported_joint {
                                errors.push(format!(
                                    "behavior {} references an actuator without a supported motor joint",
                                    behavior.id
                                ));
                            }
                        }
                        if !contact_target_position.is_finite() || !free_target_position.is_finite()
                        {
                            errors.push(format!(
                                "behavior {} has non-finite contact target positions",
                                behavior.id
                            ));
                        }
                    }
                }
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_part(id: &str) -> BodyPart {
        BodyPart {
            id: id.into(),
            name: id.into(),
            geometry: Geometry::Sphere { radius_m: 0.25 },
            transform: Transform3::default(),
            physics: BodyPhysics::Rigid(RigidBodyProperties::default()),
        }
    }

    #[test]
    fn primitive_volumes_match_si_formulas() {
        let sphere = Geometry::Sphere { radius_m: 1.0 };
        let cuboid = Geometry::Cuboid {
            half_extents_m: Vec3::new(1.0, 1.0, 1.0),
        };
        let capsule = Geometry::Capsule {
            radius_m: 1.0,
            half_height_m: 1.0,
        };

        let sphere_expected = (4.0 / 3.0) * std::f32::consts::PI;
        let capsule_expected = (4.0 / 3.0) * std::f32::consts::PI + 2.0 * std::f32::consts::PI;

        assert!((sphere.volume_m3() - sphere_expected).abs() < 1.0e-5);
        assert!((cuboid.volume_m3() - 8.0).abs() < 1.0e-5);
        assert!((capsule.volume_m3() - capsule_expected).abs() < 1.0e-5);
    }

    #[test]
    fn density_conversion_produces_mass_in_kilograms() {
        let sphere = Geometry::Sphere { radius_m: 0.1 };
        let estimated_mass = sphere
            .mass_kg_at_density(1000.0)
            .expect("positive geometry and density must produce mass");
        let expected_mass = (4.0 / 3.0) * std::f32::consts::PI * 0.1_f32.powi(3) * 1000.0;

        assert!((estimated_mass - expected_mass).abs() < 1.0e-5);
        assert!(sphere.mass_kg_at_density(0.0).is_none());
        assert!(sphere.mass_kg_at_density(f32::NAN).is_none());
    }

    #[test]
    fn accepts_a_valid_empty_organism() {
        assert!(Organism::new("Test organism").validate().is_ok());
    }

    #[test]
    fn rejects_duplicate_part_ids() {
        let mut organism = Organism::new("Test organism");
        organism.parts = vec![sample_part("body"), sample_part("body")];

        let errors = organism.validate().expect_err("duplicate IDs must fail");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("duplicate body part id"))
        );
    }

    #[test]
    fn rejects_distance_joints_without_a_rest_length() {
        let mut organism = Organism::new("Test organism");
        organism.parts = vec![sample_part("a"), sample_part("b")];
        organism.joints.push(Joint {
            id: "distance-1".into(),
            name: "Distance".into(),
            kind: JointKind::Distance,
            parent_part_id: "a".into(),
            child_part_id: "b".into(),
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::ZERO,
            limits: None,
            rest_length_m: None,
        });

        let errors = organism
            .validate()
            .expect_err("distance rest length is required");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("missing its rest length"))
        );
    }

    fn sample_contact_motion() -> (Organism, MotionDefinition) {
        let mut organism = Organism::new("Contact sensing test");
        organism.parts = vec![sample_part("body"), sample_part("foot")];
        organism.joints.push(Joint {
            id: "hinge-1".into(),
            name: "Foot hinge".into(),
            kind: JointKind::Hinge,
            parent_part_id: "body".into(),
            child_part_id: "foot".into(),
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::ZERO,
            limits: None,
            rest_length_m: None,
        });
        organism.actuators.push(Actuator {
            id: "motor-1".into(),
            name: "Foot motor".into(),
            joint_id: "hinge-1".into(),
            kind: ActuatorKind::AngularMotor,
            max_force_n: None,
            max_torque_nm: Some(10.0),
        });

        let mut motion = MotionDefinition::new(organism.id.clone());
        motion.contact_sensors.push(ContactSensor {
            id: "foot-contact".into(),
            part_id: "foot".into(),
        });
        motion.behaviors.push(Behavior {
            id: "contact-reflex".into(),
            name: "Contact reflex".into(),
            controller: ControllerKind::ContactDriven,
            goals: vec![MotionGoal::ContactDrivenActuator {
                sensor_id: "foot-contact".into(),
                actuator_id: "motor-1".into(),
                contact_target_position: 0.5,
                free_target_position: 0.0,
            }],
        });
        (organism, motion)
    }

    #[test]
    fn accepts_a_valid_contact_sensor_and_contact_driven_motor_goal() {
        let (organism, motion) = sample_contact_motion();
        assert!(motion.validate_for(&organism).is_ok());
    }

    #[test]
    fn rejects_duplicate_and_dangling_contact_sensors() {
        let (organism, mut motion) = sample_contact_motion();
        motion
            .contact_sensors
            .push(motion.contact_sensors[0].clone());
        let errors = motion
            .validate_for(&organism)
            .expect_err("duplicate IDs and part binding");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("duplicate contact sensor id"))
        );
        assert!(
            errors
                .iter()
                .any(|error| error.contains("multiple contact sensors"))
        );

        let (organism, mut motion) = sample_contact_motion();
        motion.contact_sensors[0].part_id = "missing-part".into();
        let errors = motion
            .validate_for(&organism)
            .expect_err("missing sensor part");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("references missing body part"))
        );
    }

    #[test]
    fn rejects_soft_body_contact_sensors() {
        let (mut organism, motion) = sample_contact_motion();
        organism.parts[1].physics = BodyPhysics::Soft(SoftBodyProperties {
            density_kg_m3: 1000.0,
            elastic_modulus_pa: 100_000.0,
            damping_ratio: 0.5,
            resolution: 4,
        });
        let errors = motion
            .validate_for(&organism)
            .expect_err("soft bodies have no runtime solver");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("requires a rigid body part"))
        );
    }

    #[test]
    fn rejects_contact_goals_with_invalid_controller_sensor_or_target() {
        let (organism, mut motion) = sample_contact_motion();
        motion.behaviors[0].controller = ControllerKind::Oscillator;
        let errors = motion
            .validate_for(&organism)
            .expect_err("controller kind must match goal");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("non-contact controller"))
        );

        let (organism, mut motion) = sample_contact_motion();
        motion.behaviors[0].controller = ControllerKind::ContactDriven;
        motion.contact_sensors.clear();
        let MotionGoal::ContactDrivenActuator {
            sensor_id,
            contact_target_position,
            ..
        } = &mut motion.behaviors[0].goals[0]
        else {
            unreachable!();
        };
        *sensor_id = "missing-sensor".into();
        *contact_target_position = f32::NAN;
        let errors = motion
            .validate_for(&organism)
            .expect_err("bad sensor and target");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("missing contact sensor"))
        );
        assert!(
            errors
                .iter()
                .any(|error| error.contains("non-finite contact target"))
        );
    }

    #[test]
    fn rejects_contact_driven_motors_without_a_matching_supported_joint() {
        let (mut organism, motion) = sample_contact_motion();
        organism.joints[0].kind = JointKind::Ball;
        let errors = motion
            .validate_for(&organism)
            .expect_err("motor needs a supported joint");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("supported motor joint"))
        );

        let (mut organism, motion) = sample_contact_motion();
        organism.actuators[0].kind = ActuatorKind::MuscleTendon;
        let errors = motion
            .validate_for(&organism)
            .expect_err("muscle tendon is not implemented");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("supported motor joint"))
        );
    }

    #[test]
    fn rejects_multiple_controller_goals_for_one_motor() {
        let (organism, mut motion) = sample_contact_motion();
        motion.behaviors.push(Behavior {
            id: "second-controller".into(),
            name: "Competing oscillator".into(),
            controller: ControllerKind::Oscillator,
            goals: vec![MotionGoal::OscillateActuator {
                actuator_id: "motor-1".into(),
                frequency_hz: 1.0,
                amplitude: 0.25,
            }],
        });

        let errors = motion
            .validate_for(&organism)
            .expect_err("one actuator cannot have competing controllers");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("multiple controller goals"))
        );
    }

    #[test]
    fn contact_reading_reports_contact_and_default_is_clear() {
        assert!(!ContactReading::default().is_in_contact());
        assert!(
            ContactReading {
                contact_count: 1,
                ..ContactReading::default()
            }
            .is_in_contact()
        );
    }

    #[test]
    fn rejects_joints_with_missing_parts() {
        let mut organism = Organism::new("Test organism");
        organism.joints.push(Joint {
            id: "joint-1".into(),
            name: "Missing connection".into(),
            kind: JointKind::Hinge,
            parent_part_id: "missing-a".into(),
            child_part_id: "missing-b".into(),
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::ZERO,
            limits: None,
            rest_length_m: None,
        });

        let errors = organism
            .validate()
            .expect_err("dangling references must fail");
        assert!(
            errors
                .iter()
                .any(|error| error.contains("missing parent part"))
        );
        assert!(
            errors
                .iter()
                .any(|error| error.contains("missing child part"))
        );
    }
}

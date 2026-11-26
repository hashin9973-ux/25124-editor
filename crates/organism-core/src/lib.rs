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
    pub friction: f32,
    pub restitution: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl Default for RigidBodyProperties {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
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
                errors.push(format!("distance joint {} is missing its rest length", joint.id));
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
    pub behaviors: Vec<Behavior>,
}

impl MotionDefinition {
    pub fn new(organism_id: impl Into<String>) -> Self {
        Self {
            format: MOTION_FORMAT.into(),
            schema_version: SCHEMA_VERSION,
            organism_id: organism_id.into(),
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
        let mut behavior_ids = BTreeSet::new();
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
                        }
                        if !frequency_hz.is_finite() || *frequency_hz <= 0.0 {
                            errors.push(format!("behavior {} has invalid frequency", behavior.id));
                        }
                        if !amplitude.is_finite() || *amplitude < 0.0 {
                            errors.push(format!("behavior {} has invalid amplitude", behavior.id));
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

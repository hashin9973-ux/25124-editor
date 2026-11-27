//! Validated, engine-independent joint editing operations.

use organism_core::{Joint, JointKind, Vec3};
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct JointEdit {
    pub name: String,
    pub anchor_parent_m: Vec3,
    pub anchor_child_m: Vec3,
    pub rest_length_m: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointEditError {
    EmptyName,
    InvalidAnchor,
    MissingRestLength,
    InvalidRestLength,
    RestLengthNotAllowed,
}

impl fmt::Display for JointEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyName => "Joint name cannot be empty.",
            Self::InvalidAnchor => "Joint anchor coordinates must be finite.",
            Self::MissingRestLength => "Distance joints require a rest length.",
            Self::InvalidRestLength => {
                "Distance rest length must be finite and greater than zero meters."
            }
            Self::RestLengthNotAllowed => "Only distance joints can define a rest length.",
        };
        f.write_str(message)
    }
}

impl std::error::Error for JointEditError {}

/// Validates and atomically applies joint naming, local attachment points, and distance length.
pub fn apply_joint_edit(joint: &mut Joint, edit: JointEdit) -> Result<(), JointEditError> {
    if edit.name.trim().is_empty() {
        return Err(JointEditError::EmptyName);
    }
    if !edit.anchor_parent_m.is_finite() || !edit.anchor_child_m.is_finite() {
        return Err(JointEditError::InvalidAnchor);
    }

    match (joint.kind, edit.rest_length_m) {
        (JointKind::Distance, None) => return Err(JointEditError::MissingRestLength),
        (JointKind::Distance, Some(length)) if !length.is_finite() || length <= 0.0 => {
            return Err(JointEditError::InvalidRestLength);
        }
        (JointKind::Distance, Some(_)) => {}
        (_, Some(_)) => return Err(JointEditError::RestLengthNotAllowed),
        (_, None) => {}
    }

    joint.name = edit.name.trim().to_owned();
    joint.anchor_parent_m = edit.anchor_parent_m;
    joint.anchor_child_m = edit.anchor_child_m;
    if joint.kind == JointKind::Distance {
        joint.rest_length_m = edit.rest_length_m;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::{JointLimitUnit, JointLimits};

    fn joint(kind: JointKind) -> Joint {
        Joint {
            id: "joint-0001".into(),
            name: "Connection".into(),
            kind,
            parent_part_id: "part-0001".into(),
            child_part_id: "part-0002".into(),
            anchor_parent_m: Vec3::new(0.0, 0.0, 0.0),
            anchor_child_m: Vec3::new(0.0, 0.0, 0.0),
            limits: Some(JointLimits {
                min: -0.5,
                max: 0.5,
                unit: JointLimitUnit::Radians,
            }),
            rest_length_m: (kind == JointKind::Distance).then_some(0.75),
        }
    }

    fn edit(kind: JointKind) -> JointEdit {
        JointEdit {
            name: "  Edited connection  ".into(),
            anchor_parent_m: Vec3::new(0.1, -0.2, 0.3),
            anchor_child_m: Vec3::new(-0.4, 0.5, -0.6),
            rest_length_m: (kind == JointKind::Distance).then_some(1.25),
        }
    }

    #[test]
    fn hinge_edit_updates_name_and_local_anchors_without_changing_limits() {
        let mut value = joint(JointKind::Hinge);
        let original_limits = value.limits;
        apply_joint_edit(&mut value, edit(JointKind::Hinge)).expect("valid hinge edit");
        assert_eq!(value.name, "Edited connection");
        assert_eq!(value.anchor_parent_m, Vec3::new(0.1, -0.2, 0.3));
        assert_eq!(value.anchor_child_m, Vec3::new(-0.4, 0.5, -0.6));
        assert_eq!(value.limits, original_limits);
        assert_eq!(value.rest_length_m, None);
    }

    #[test]
    fn distance_edit_updates_anchors_and_rest_length() {
        let mut value = joint(JointKind::Distance);
        apply_joint_edit(&mut value, edit(JointKind::Distance)).expect("valid distance edit");
        assert_eq!(value.rest_length_m, Some(1.25));
        assert_eq!(value.anchor_parent_m, Vec3::new(0.1, -0.2, 0.3));
        assert_eq!(value.anchor_child_m, Vec3::new(-0.4, 0.5, -0.6));
    }

    #[test]
    fn invalid_edits_leave_the_joint_unchanged() {
        let cases = [
            (JointEditError::EmptyName, 0),
            (JointEditError::InvalidAnchor, 1),
            (JointEditError::InvalidRestLength, 2),
            (JointEditError::MissingRestLength, 3),
        ];
        for (expected, case) in cases {
            let mut value = joint(JointKind::Distance);
            let original = value.clone();
            let mut proposed = edit(JointKind::Distance);
            match case {
                0 => proposed.name = " ".into(),
                1 => proposed.anchor_parent_m.x = f32::NAN,
                2 => proposed.rest_length_m = Some(f32::INFINITY),
                _ => proposed.rest_length_m = None,
            }
            assert_eq!(apply_joint_edit(&mut value, proposed), Err(expected));
            assert_eq!(value, original);
        }
    }

    #[test]
    fn non_distance_joints_reject_rest_length_edits() {
        let mut value = joint(JointKind::Hinge);
        let original = value.clone();
        let mut proposed = edit(JointKind::Hinge);
        proposed.rest_length_m = Some(0.5);
        assert_eq!(
            apply_joint_edit(&mut value, proposed),
            Err(JointEditError::RestLengthNotAllowed)
        );
        assert_eq!(value, original);
    }
}

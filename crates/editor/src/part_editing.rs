//! Validated, engine-independent body-part editing operations.

use organism_core::{BodyPart, BodyPhysics, Geometry};
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct PartEdit {
    pub name: String,
    pub geometry: Geometry,
    pub density_kg_m3: f32,
    pub friction: f32,
    pub restitution: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartEditError {
    EmptyName,
    GeometryTypeMismatch,
    InvalidGeometry,
    InvalidDensity,
    InvalidFriction,
    InvalidRestitution,
    InvalidDamping,
    UnsupportedPhysics,
}

impl fmt::Display for PartEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyName => "Part name cannot be empty.",
            Self::GeometryTypeMismatch => {
                "A part's primitive type cannot be changed through dimension fields."
            }
            Self::InvalidGeometry => "Geometry dimensions must be finite and physically valid.",
            Self::InvalidDensity => "Density must be finite and greater than zero kg/m┬│.",
            Self::InvalidFriction => "Friction must be finite and non-negative.",
            Self::InvalidRestitution => "Restitution must be between zero and one.",
            Self::InvalidDamping => "Linear and angular damping must be finite and non-negative.",
            Self::UnsupportedPhysics => "Only rigid body parts can be edited with this inspector.",
        };
        f.write_str(message)
    }
}

impl std::error::Error for PartEditError {}

/// Validates and atomically applies a complete rigid-part edit. Mass is derived
/// from primitive volume and density; failed edits leave the part unchanged.
pub fn apply_part_edit(part: &mut BodyPart, edit: PartEdit) -> Result<(), PartEditError> {
    if edit.name.trim().is_empty() {
        return Err(PartEditError::EmptyName);
    }
    if !same_geometry_kind(&part.geometry, &edit.geometry) {
        return Err(PartEditError::GeometryTypeMismatch);
    }
    if !edit.density_kg_m3.is_finite() || edit.density_kg_m3 <= 0.0 {
        return Err(PartEditError::InvalidDensity);
    }
    if !edit.friction.is_finite() || edit.friction < 0.0 {
        return Err(PartEditError::InvalidFriction);
    }
    if !edit.restitution.is_finite() || !(0.0..=1.0).contains(&edit.restitution) {
        return Err(PartEditError::InvalidRestitution);
    }
    if !edit.linear_damping.is_finite()
        || edit.linear_damping < 0.0
        || !edit.angular_damping.is_finite()
        || edit.angular_damping < 0.0
    {
        return Err(PartEditError::InvalidDamping);
    }
    if !matches!(&part.physics, BodyPhysics::Rigid(_)) {
        return Err(PartEditError::UnsupportedPhysics);
    }
    let mass_kg = edit
        .geometry
        .mass_kg_at_density(edit.density_kg_m3)
        .ok_or(PartEditError::InvalidGeometry)?;

    let BodyPhysics::Rigid(properties) = &mut part.physics else {
        unreachable!("rigid-body type was validated above");
    };
    part.name = edit.name.trim().to_owned();
    part.geometry = edit.geometry;
    properties.mass_kg = mass_kg;
    properties.density_kg_m3 = edit.density_kg_m3;
    properties.friction = edit.friction;
    properties.restitution = edit.restitution;
    properties.linear_damping = edit.linear_damping;
    properties.angular_damping = edit.angular_damping;
    Ok(())
}

fn same_geometry_kind(current: &Geometry, proposed: &Geometry) -> bool {
    matches!(
        (current, proposed),
        (Geometry::Sphere { .. }, Geometry::Sphere { .. })
            | (Geometry::Cuboid { .. }, Geometry::Cuboid { .. })
            | (Geometry::Capsule { .. }, Geometry::Capsule { .. })
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::{Organism, RigidBodyProperties, SoftBodyProperties, Transform3, Vec3};

    fn rigid_part(geometry: Geometry) -> BodyPart {
        BodyPart {
            id: "part-0001".into(),
            name: "Body".into(),
            geometry,
            transform: Transform3::default(),
            physics: BodyPhysics::Rigid(RigidBodyProperties {
                density_kg_m3: 950.0,
                friction: 0.65,
                restitution: 0.1,
                linear_damping: 0.03,
                angular_damping: 0.04,
                ..Default::default()
            }),
        }
    }

    fn test_edit(geometry: Geometry) -> PartEdit {
        PartEdit {
            name: "  Edited body  ".into(),
            geometry,
            density_kg_m3: 1200.0,
            friction: 0.8,
            restitution: 0.25,
            linear_damping: 0.1,
            angular_damping: 0.2,
        }
    }

    #[test]
    fn sphere_edit_updates_name_geometry_mass_and_material_properties() {
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        apply_part_edit(&mut part, test_edit(Geometry::Sphere { radius_m: 0.4 }))
            .expect("valid sphere edit");
        assert_eq!(part.name, "Edited body");
        assert_eq!(part.geometry, Geometry::Sphere { radius_m: 0.4 });
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        let mass = (4.0 / 3.0) * std::f32::consts::PI * 0.4_f32.powi(3) * 1200.0;
        assert!((p.mass_kg - mass).abs() < 1.0e-5);
        assert_eq!(p.density_kg_m3, 1200.0);
        assert_eq!(p.friction, 0.8);
        assert_eq!(p.restitution, 0.25);
        assert_eq!(p.linear_damping, 0.1);
        assert_eq!(p.angular_damping, 0.2);
    }

    #[test]
    fn cuboid_half_extents_edit_recalculates_mass() {
        let mut part = rigid_part(Geometry::Cuboid {
            half_extents_m: Vec3::new(0.2, 0.2, 0.2),
        });
        let geometry = Geometry::Cuboid {
            half_extents_m: Vec3::new(0.3, 0.4, 0.5),
        };
        apply_part_edit(&mut part, test_edit(geometry.clone())).expect("valid cuboid edit");
        assert_eq!(part.geometry, geometry);
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        assert!((p.mass_kg - 8.0 * 0.3 * 0.4 * 0.5 * 1200.0).abs() < 1.0e-3);
    }

    #[test]
    fn capsule_radius_and_half_height_edit_recalculates_mass() {
        let mut part = rigid_part(Geometry::Capsule {
            radius_m: 0.2,
            half_height_m: 0.3,
        });
        let geometry = Geometry::Capsule {
            radius_m: 0.15,
            half_height_m: 0.45,
        };
        apply_part_edit(&mut part, test_edit(geometry.clone())).expect("valid capsule edit");
        assert_eq!(part.geometry, geometry);
        let BodyPhysics::Rigid(p) = part.physics else {
            panic!("expected rigid body");
        };
        let volume = std::f32::consts::PI * 0.15_f32.powi(2) * 0.9
            + (4.0 / 3.0) * std::f32::consts::PI * 0.15_f32.powi(3);
        assert!((p.mass_kg - volume * 1200.0).abs() < 1.0e-4);
    }

    #[test]
    fn capsule_allows_zero_cylindrical_half_height() {
        let mut part = rigid_part(Geometry::Capsule {
            radius_m: 0.2,
            half_height_m: 0.3,
        });
        apply_part_edit(
            &mut part,
            test_edit(Geometry::Capsule {
                radius_m: 0.25,
                half_height_m: 0.0,
            }),
        )
        .expect("hemisphere-only capsule is valid");
    }

    #[test]
    fn invalid_edits_do_not_partially_mutate_part() {
        let cases = [
            (PartEditError::EmptyName, 0),
            (PartEditError::InvalidDensity, 1),
            (PartEditError::InvalidFriction, 2),
            (PartEditError::InvalidRestitution, 3),
            (PartEditError::InvalidDamping, 4),
            (PartEditError::InvalidGeometry, 5),
        ];
        for (expected, case) in cases {
            let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
            let original = part.clone();
            let mut edit = test_edit(Geometry::Sphere { radius_m: 0.4 });
            match case {
                0 => edit.name = " ".into(),
                1 => edit.density_kg_m3 = f32::NAN,
                2 => edit.friction = -0.1,
                3 => edit.restitution = 1.1,
                4 => edit.angular_damping = f32::INFINITY,
                _ => {
                    edit.geometry = Geometry::Sphere {
                        radius_m: f32::INFINITY,
                    }
                }
            }
            assert_eq!(apply_part_edit(&mut part, edit), Err(expected));
            assert_eq!(part, original);
        }
    }

    #[test]
    fn rejects_geometry_type_changes_and_soft_parts() {
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        let original = part.clone();
        let mismatch = test_edit(Geometry::Cuboid {
            half_extents_m: Vec3::new(0.2, 0.2, 0.2),
        });
        assert_eq!(
            apply_part_edit(&mut part, mismatch),
            Err(PartEditError::GeometryTypeMismatch)
        );
        assert_eq!(part, original);
        part.physics = BodyPhysics::Soft(SoftBodyProperties {
            density_kg_m3: 900.0,
            elastic_modulus_pa: 100_000.0,
            damping_ratio: 0.2,
            resolution: 4,
        });
        let original = part.clone();
        let soft = PartEdit {
            name: part.name.clone(),
            geometry: part.geometry.clone(),
            density_kg_m3: 900.0,
            friction: 0.5,
            restitution: 0.0,
            linear_damping: 0.05,
            angular_damping: 0.05,
        };
        assert_eq!(
            apply_part_edit(&mut part, soft),
            Err(PartEditError::UnsupportedPhysics)
        );
        assert_eq!(part, original);
    }

    #[test]
    fn edited_part_is_encoded_in_the_organism_document() {
        let mut organism = Organism::new("Editable organism");
        let mut part = rigid_part(Geometry::Sphere { radius_m: 0.2 });
        apply_part_edit(&mut part, test_edit(Geometry::Sphere { radius_m: 0.35 }))
            .expect("valid edit");
        organism.parts.push(part);
        let json = organism_serialization::encode_organism(&organism)
            .expect("edited organism should pass validation and encode");
        assert!(json.contains("\"radius_m\": 0.35"));
        assert!(json.contains("\"name\": \"Edited body\""));
        assert!(json.contains("\"density_kg_m3\": 1200.0"));
    }
}

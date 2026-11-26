//! Validated, engine-independent body-part editing operations.

use organism_core::{BodyPart, BodyPhysics, Geometry};
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartEditError {
    InvalidRadius,
    UnsupportedGeometry,
    UnsupportedPhysics,
    InvalidMass,
}

impl fmt::Display for PartEditError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidRadius => "Radius must be a finite value greater than zero.",
            Self::UnsupportedGeometry => "The selected part is not a sphere.",
            Self::UnsupportedPhysics => "Only rigid body parts can be edited with this control.",
            Self::InvalidMass => {
                "The edited geometry cannot produce a valid mass at the current density."
            }
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for PartEditError {}

/// Updates a sphere's radius and recalculates its rigid-body mass without
/// partially mutating the part when the new geometry or mass is invalid.
pub fn set_sphere_radius(part: &mut BodyPart, radius_m: f32) -> Result<(), PartEditError> {
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(PartEditError::InvalidRadius);
    }

    if !matches!(&part.geometry, Geometry::Sphere { .. }) {
        return Err(PartEditError::UnsupportedGeometry);
    }

    let density_kg_m3 = match &part.physics {
        BodyPhysics::Rigid(properties) => properties.density_kg_m3,
        BodyPhysics::Soft(_) => return Err(PartEditError::UnsupportedPhysics),
    };

    let geometry = Geometry::Sphere { radius_m };
    let mass_kg = geometry
        .mass_kg_at_density(density_kg_m3)
        .ok_or(PartEditError::InvalidMass)?;

    part.geometry = geometry;
    if let BodyPhysics::Rigid(properties) = &mut part.physics {
        properties.mass_kg = mass_kg;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::{Organism, RigidBodyProperties, Transform3};

    fn sphere_part() -> BodyPart {
        BodyPart {
            id: "part-0001".into(),
            name: "Body".into(),
            geometry: Geometry::Sphere { radius_m: 0.2 },
            transform: Transform3::default(),
            physics: BodyPhysics::Rigid(RigidBodyProperties {
                density_kg_m3: 950.0,
                ..Default::default()
            }),
        }
    }

    #[test]
    fn sphere_radius_edit_updates_geometry_and_mass_from_density() {
        let mut part = sphere_part();

        set_sphere_radius(&mut part, 0.4).expect("valid radius edit");

        assert_eq!(part.geometry, Geometry::Sphere { radius_m: 0.4 });
        let BodyPhysics::Rigid(properties) = part.physics else {
            panic!("sphere part should remain rigid");
        };
        let expected_mass = (4.0 / 3.0) * std::f32::consts::PI * 0.4_f32.powi(3) * 950.0;
        assert!((properties.mass_kg - expected_mass).abs() < 1.0e-5);
    }

    #[test]
    fn edited_part_is_encoded_in_the_organism_document() {
        let mut organism = Organism::new("Editable organism");
        let mut part = sphere_part();
        set_sphere_radius(&mut part, 0.35).expect("valid radius edit");
        organism.parts.push(part);

        let json = organism_serialization::encode_organism(&organism)
            .expect("edited organism should pass model validation and encode");
        assert!(json.contains("\"radius_m\": 0.35"));
        assert!(json.contains("\"mass_kg\""));
    }

    #[test]
    fn invalid_radius_does_not_partially_mutate_part() {
        for radius in [0.0, -0.1, f32::NAN, f32::INFINITY] {
            let mut part = sphere_part();
            let original = part.clone();

            assert_eq!(
                set_sphere_radius(&mut part, radius),
                Err(PartEditError::InvalidRadius)
            );
            assert_eq!(part, original);
        }
    }

    #[test]
    fn rejects_non_sphere_geometry_without_mutation() {
        let mut part = sphere_part();
        part.geometry = Geometry::Cuboid {
            half_extents_m: organism_core::Vec3::new(0.2, 0.3, 0.4),
        };
        let original = part.clone();

        assert_eq!(
            set_sphere_radius(&mut part, 0.5),
            Err(PartEditError::UnsupportedGeometry)
        );
        assert_eq!(part, original);
    }

    #[test]
    fn rejects_soft_body_parts_without_mutation() {
        let mut part = sphere_part();
        part.physics = BodyPhysics::Soft(organism_core::SoftBodyProperties {
            density_kg_m3: 900.0,
            elastic_modulus_pa: 100_000.0,
            damping_ratio: 0.2,
            resolution: 4,
        });
        let original = part.clone();

        assert_eq!(
            set_sphere_radius(&mut part, 0.5),
            Err(PartEditError::UnsupportedPhysics)
        );
        assert_eq!(part, original);
    }
}

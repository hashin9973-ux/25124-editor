//! Shared primitive-to-runtime geometry conversion for construction and editing.

use avian3d::prelude::Collider;
use bevy::prelude::{Capsule3d, Color, Cuboid, Mesh, Sphere};
use organism_core::Geometry;

/// Engine geometry derived from one validated domain primitive.
pub struct RuntimeGeometry {
    pub mesh: Mesh,
    pub collider: Collider,
    pub pick_radius: f32,
    pub ground_offset: f32,
    pub color: Color,
}

/// Converts a domain primitive into matching Bevy render geometry and Avian collider data.
pub fn build_runtime_geometry(geometry: &Geometry) -> Option<RuntimeGeometry> {
    let runtime = match geometry {
        Geometry::Sphere { radius_m } if radius_m.is_finite() && *radius_m > 0.0 => {
            RuntimeGeometry {
                mesh: Sphere::new(*radius_m).into(),
                collider: Collider::sphere(*radius_m),
                pick_radius: *radius_m,
                ground_offset: *radius_m,
                color: Color::srgb(0.24, 0.68, 0.92),
            }
        }
        Geometry::Cuboid { half_extents_m }
            if half_extents_m.is_finite()
                && half_extents_m.x > 0.0
                && half_extents_m.y > 0.0
                && half_extents_m.z > 0.0 =>
        {
            let full_size = bevy::prelude::Vec3::new(
                half_extents_m.x * 2.0,
                half_extents_m.y * 2.0,
                half_extents_m.z * 2.0,
            );
            RuntimeGeometry {
                mesh: Cuboid::from_size(full_size).into(),
                // Collider::cuboid takes full side lengths, not half-extents.
                collider: Collider::cuboid(full_size.x, full_size.y, full_size.z),
                pick_radius: half_extents_m.x.hypot(half_extents_m.y).hypot(half_extents_m.z),
                ground_offset: half_extents_m.y,
                color: Color::srgb(0.32, 0.78, 0.58),
            }
        }
        Geometry::Capsule {
            radius_m,
            half_height_m,
        } if radius_m.is_finite()
            && *radius_m > 0.0
            && half_height_m.is_finite()
            && *half_height_m >= 0.0 =>
        {
            let cylinder_length = *half_height_m * 2.0;
            let outer_radius = *radius_m + *half_height_m;
            if !cylinder_length.is_finite() || !outer_radius.is_finite() {
                return None;
            }
            RuntimeGeometry {
                mesh: Capsule3d::new(*radius_m, cylinder_length).into(),
                collider: Collider::capsule(*radius_m, cylinder_length),
                pick_radius: outer_radius,
                ground_offset: outer_radius,
                color: Color::srgb(0.92, 0.63, 0.28),
            }
        }
        _ => return None,
    };

    Some(runtime)
}

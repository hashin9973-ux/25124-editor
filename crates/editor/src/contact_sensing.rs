//! Engine-independent aggregation of physics contact samples into sensor readings.

use organism_core::{ContactReading, Vec3};

const NORMAL_LENGTH_EPSILON: f32 = 1.0e-6;

/// Converts Avian's first-to-second contact normal into a normal pointing
/// from the environment surface toward the sensed body.
pub fn normal_toward_sensor(normal_world: Vec3, sensor_is_first: bool) -> Option<Vec3> {
    if !normal_world.is_finite() {
        return None;
    }

    let direction = if sensor_is_first {
        Vec3::new(-normal_world.x, -normal_world.y, -normal_world.z)
    } else {
        normal_world
    };
    let length_squared =
        direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
    if !length_squared.is_finite() || length_squared <= NORMAL_LENGTH_EPSILON.powi(2) {
        return None;
    }

    let inverse_length = length_squared.sqrt().recip();
    Some(Vec3::new(
        direction.x * inverse_length,
        direction.y * inverse_length,
        direction.z * inverse_length,
    ))
}

/// Aggregates all environment contact points for one body part during one physics step.
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactReadingAccumulator {
    contact_count: u32,
    support_normal_sum: Vec3,
    total_normal_impulse_ns: f32,
    max_approach_speed_mps: f32,
}

impl ContactReadingAccumulator {
    pub fn add_contact(
        &mut self,
        normal_toward_body_world: Vec3,
        normal_impulse_ns: f32,
        normal_speed_mps: f32,
    ) {
        let Some(normal) = normal_toward_sensor(normal_toward_body_world, false) else {
            return;
        };

        self.contact_count = self.contact_count.saturating_add(1);
        self.support_normal_sum.x += normal.x;
        self.support_normal_sum.y += normal.y;
        self.support_normal_sum.z += normal.z;

        if normal_impulse_ns.is_finite() {
            let impulse_sum = self.total_normal_impulse_ns + normal_impulse_ns.abs();
            if impulse_sum.is_finite() {
                self.total_normal_impulse_ns = impulse_sum;
            }
        }

        if normal_speed_mps.is_finite() {
            self.max_approach_speed_mps = self
                .max_approach_speed_mps
                .max((-normal_speed_mps).max(0.0));
        }
    }

    pub fn finish(self) -> ContactReading {
        let length_squared = self.support_normal_sum.x * self.support_normal_sum.x
            + self.support_normal_sum.y * self.support_normal_sum.y
            + self.support_normal_sum.z * self.support_normal_sum.z;
        let support_normal_world =
            if length_squared.is_finite() && length_squared > NORMAL_LENGTH_EPSILON.powi(2) {
                let inverse_length = length_squared.sqrt().recip();
                Vec3::new(
                    self.support_normal_sum.x * inverse_length,
                    self.support_normal_sum.y * inverse_length,
                    self.support_normal_sum.z * inverse_length,
                )
            } else {
                Vec3::ZERO
            };

        ContactReading {
            contact_count: self.contact_count,
            support_normal_world,
            total_normal_impulse_ns: self.total_normal_impulse_ns,
            max_approach_speed_mps: self.max_approach_speed_mps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contact_normal_points_toward_sensor_regardless_of_pair_order() {
        let first =
            normal_toward_sensor(Vec3::new(0.0, 1.0, 0.0), true).expect("finite unit normal");
        let second =
            normal_toward_sensor(Vec3::new(0.0, -1.0, 0.0), false).expect("finite unit normal");

        assert_eq!(first, Vec3::new(0.0, -1.0, 0.0));
        assert_eq!(second, Vec3::new(0.0, -1.0, 0.0));
    }

    #[test]
    fn aggregates_contact_points_and_only_counts_closing_speed() {
        let mut accumulator = ContactReadingAccumulator::default();
        accumulator.add_contact(Vec3::new(0.0, 2.0, 0.0), -0.4, -1.5);
        accumulator.add_contact(Vec3::new(0.0, 1.0, 0.0), 0.6, 0.25);

        let reading = accumulator.finish();
        assert_eq!(reading.contact_count, 2);
        assert_eq!(reading.support_normal_world, Vec3::new(0.0, 1.0, 0.0));
        assert!((reading.total_normal_impulse_ns - 1.0).abs() < 1.0e-6);
        assert!((reading.max_approach_speed_mps - 1.5).abs() < 1.0e-6);
        assert!(reading.is_in_contact());
    }

    #[test]
    fn invalid_normals_do_not_create_contact_and_empty_readings_are_clear() {
        let mut accumulator = ContactReadingAccumulator::default();
        accumulator.add_contact(Vec3::ZERO, 4.0, -2.0);
        accumulator.add_contact(Vec3::new(f32::NAN, 1.0, 0.0), 4.0, -2.0);

        let reading = accumulator.finish();
        assert_eq!(reading, ContactReading::default());
        assert!(!reading.is_in_contact());
    }
}

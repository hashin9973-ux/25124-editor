//! Validated, engine-independent actuator and oscillator-controller editing.

use organism_core::{Actuator, ActuatorKind, Behavior, ControllerKind, MotionGoal};
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorControllerEdit {
    pub actuator_name: String,
    pub max_force_n: Option<f32>,
    pub max_torque_nm: Option<f32>,
    pub behavior_name: String,
    pub frequency_hz: f32,
    pub amplitude: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorControllerEditError {
    EmptyActuatorName,
    EmptyBehaviorName,
    UnsupportedActuatorKind,
    InvalidForceLimit,
    InvalidTorqueLimit,
    UnsupportedController,
    MissingOscillatorGoal,
    MultipleOscillatorGoals,
    InvalidFrequency,
    InvalidAmplitude,
}

impl fmt::Display for ActuatorControllerEditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyActuatorName => "Actuator name cannot be empty.",
            Self::EmptyBehaviorName => "Controller behavior name cannot be empty.",
            Self::UnsupportedActuatorKind => {
                "Muscle-tendon actuators do not have a live runtime implementation yet."
            }
            Self::InvalidForceLimit => {
                "Linear motors require a finite, positive maximum force in newtons only."
            }
            Self::InvalidTorqueLimit => {
                "Angular motors require a finite, positive maximum torque in newton-meters only."
            }
            Self::UnsupportedController => {
                "Only oscillator controllers currently drive live actuator motors."
            }
            Self::MissingOscillatorGoal => {
                "The selected behavior has no oscillator goal for this actuator."
            }
            Self::MultipleOscillatorGoals => {
                "A behavior may contain only one oscillator goal for this actuator."
            }
            Self::InvalidFrequency => "Oscillator frequency must be finite and greater than zero Hz.",
            Self::InvalidAmplitude => "Oscillator amplitude must be finite and non-negative.",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ActuatorControllerEditError {}

/// Validates and atomically applies actuator limits and a linked oscillator goal.
/// The actuator kind, IDs, joint association, and unrelated behavior goals are preserved.
pub fn apply_actuator_controller_edit(
    actuator: &mut Actuator,
    behavior: &mut Behavior,
    edit: ActuatorControllerEdit,
) -> Result<(), ActuatorControllerEditError> {
    if edit.actuator_name.trim().is_empty() {
        return Err(ActuatorControllerEditError::EmptyActuatorName);
    }
    if edit.behavior_name.trim().is_empty() {
        return Err(ActuatorControllerEditError::EmptyBehaviorName);
    }

    match actuator.kind {
        ActuatorKind::AngularMotor => {
            if edit.max_force_n.is_some() {
                return Err(ActuatorControllerEditError::InvalidTorqueLimit);
            }
            if !edit
                .max_torque_nm
                .is_some_and(|value| value.is_finite() && value > 0.0)
            {
                return Err(ActuatorControllerEditError::InvalidTorqueLimit);
            }
        }
        ActuatorKind::LinearMotor => {
            if edit.max_torque_nm.is_some() {
                return Err(ActuatorControllerEditError::InvalidForceLimit);
            }
            if !edit
                .max_force_n
                .is_some_and(|value| value.is_finite() && value > 0.0)
            {
                return Err(ActuatorControllerEditError::InvalidForceLimit);
            }
        }
        ActuatorKind::MuscleTendon => {
            return Err(ActuatorControllerEditError::UnsupportedActuatorKind);
        }
    }

    if behavior.controller != ControllerKind::Oscillator {
        return Err(ActuatorControllerEditError::UnsupportedController);
    }
    if !edit.frequency_hz.is_finite() || edit.frequency_hz <= 0.0 {
        return Err(ActuatorControllerEditError::InvalidFrequency);
    }
    if !edit.amplitude.is_finite() || edit.amplitude < 0.0 {
        return Err(ActuatorControllerEditError::InvalidAmplitude);
    }

    let matching_goals = behavior
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
    match matching_goals {
        0 => return Err(ActuatorControllerEditError::MissingOscillatorGoal),
        1 => {}
        _ => return Err(ActuatorControllerEditError::MultipleOscillatorGoals),
    }

    actuator.name = edit.actuator_name.trim().to_owned();
    actuator.max_force_n = edit.max_force_n;
    actuator.max_torque_nm = edit.max_torque_nm;
    behavior.name = edit.behavior_name.trim().to_owned();
    for goal in &mut behavior.goals {
        if let MotionGoal::OscillateActuator {
            actuator_id,
            frequency_hz,
            amplitude,
        } = goal
            && actuator_id == &actuator.id
        {
            *frequency_hz = edit.frequency_hz;
            *amplitude = edit.amplitude;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::Vec3;

    fn angular_actuator() -> Actuator {
        Actuator {
            id: "actuator-0001".into(),
            name: "Hinge motor".into(),
            joint_id: "joint-0001".into(),
            kind: ActuatorKind::AngularMotor,
            max_force_n: None,
            max_torque_nm: Some(10.0),
        }
    }

    fn linear_actuator() -> Actuator {
        Actuator {
            id: "actuator-0002".into(),
            name: "Slider motor".into(),
            joint_id: "joint-0002".into(),
            kind: ActuatorKind::LinearMotor,
            max_force_n: Some(20.0),
            max_torque_nm: None,
        }
    }

    fn behavior(actuator_id: &str) -> Behavior {
        Behavior {
            id: "behavior-0001".into(),
            name: "Oscillate joint".into(),
            controller: ControllerKind::Oscillator,
            goals: vec![
                MotionGoal::MaintainBalance {
                    target_up: Vec3::Y,
                },
                MotionGoal::OscillateActuator {
                    actuator_id: actuator_id.into(),
                    frequency_hz: 0.5,
                    amplitude: 0.55,
                },
            ],
        }
    }

    fn angular_edit() -> ActuatorControllerEdit {
        ActuatorControllerEdit {
            actuator_name: "  Updated hinge motor  ".into(),
            max_force_n: None,
            max_torque_nm: Some(24.0),
            behavior_name: "  Faster hinge cycle  ".into(),
            frequency_hz: 1.25,
            amplitude: 0.8,
        }
    }

    #[test]
    fn angular_motor_edit_updates_torque_and_oscillator_goal() {
        let mut actuator = angular_actuator();
        let mut controller = behavior(&actuator.id);
        apply_actuator_controller_edit(&mut actuator, &mut controller, angular_edit())
            .expect("valid angular motor edit");

        assert_eq!(actuator.name, "Updated hinge motor");
        assert_eq!(actuator.max_force_n, None);
        assert_eq!(actuator.max_torque_nm, Some(24.0));
        assert_eq!(controller.name, "Faster hinge cycle");
        assert!(matches!(
            controller.goals[0],
            MotionGoal::MaintainBalance { .. }
        ));
        assert_eq!(
            controller.goals[1],
            MotionGoal::OscillateActuator {
                actuator_id: actuator.id.clone(),
                frequency_hz: 1.25,
                amplitude: 0.8,
            }
        );
    }

    #[test]
    fn linear_motor_edit_updates_force_without_a_torque_limit() {
        let mut actuator = linear_actuator();
        let mut controller = behavior(&actuator.id);
        let edit = ActuatorControllerEdit {
            actuator_name: "Updated slider".into(),
            max_force_n: Some(35.0),
            max_torque_nm: None,
            behavior_name: "Slider oscillation".into(),
            frequency_hz: 0.8,
            amplitude: 0.3,
        };
        apply_actuator_controller_edit(&mut actuator, &mut controller, edit)
            .expect("valid linear motor edit");

        assert_eq!(actuator.max_force_n, Some(35.0));
        assert_eq!(actuator.max_torque_nm, None);
        assert_eq!(
            controller.goals[1],
            MotionGoal::OscillateActuator {
                actuator_id: actuator.id.clone(),
                frequency_hz: 0.8,
                amplitude: 0.3,
            }
        );
    }

    #[test]
    fn invalid_edits_leave_actuator_and_behavior_unchanged() {
        let cases = [
            (ActuatorControllerEditError::EmptyActuatorName, 0),
            (ActuatorControllerEditError::EmptyBehaviorName, 1),
            (ActuatorControllerEditError::InvalidTorqueLimit, 2),
            (ActuatorControllerEditError::InvalidFrequency, 3),
            (ActuatorControllerEditError::InvalidAmplitude, 4),
        ];
        for (expected, case) in cases {
            let mut actuator = angular_actuator();
            let mut controller = behavior(&actuator.id);
            let original_actuator = actuator.clone();
            let original_controller = controller.clone();
            let mut edit = angular_edit();
            match case {
                0 => edit.actuator_name = " ".into(),
                1 => edit.behavior_name = " ".into(),
                2 => edit.max_torque_nm = Some(f32::INFINITY),
                3 => edit.frequency_hz = 0.0,
                _ => edit.amplitude = f32::NAN,
            }
            assert_eq!(
                apply_actuator_controller_edit(&mut actuator, &mut controller, edit),
                Err(expected)
            );
            assert_eq!(actuator, original_actuator);
            assert_eq!(controller, original_controller);
        }
    }

    #[test]
    fn unsupported_kinds_and_unlinked_goals_are_rejected_atomically() {
        let mut actuator = angular_actuator();
        let mut controller = behavior("a-different-actuator");
        let original_actuator = actuator.clone();
        let original_controller = controller.clone();
        assert_eq!(
            apply_actuator_controller_edit(
                &mut actuator,
                &mut controller,
                angular_edit()
            ),
            Err(ActuatorControllerEditError::MissingOscillatorGoal)
        );
        assert_eq!(actuator, original_actuator);
        assert_eq!(controller, original_controller);

        actuator.kind = ActuatorKind::MuscleTendon;
        assert_eq!(
            apply_actuator_controller_edit(
                &mut actuator,
                &mut controller,
                angular_edit()
            ),
            Err(ActuatorControllerEditError::UnsupportedActuatorKind)
        );
    }

    #[test]
    fn multiple_matching_goals_and_non_oscillator_controllers_are_rejected() {
        let mut actuator = angular_actuator();
        let mut controller = behavior(&actuator.id);
        controller.goals.push(controller.goals[1].clone());
        assert_eq!(
            apply_actuator_controller_edit(
                &mut actuator,
                &mut controller,
                angular_edit()
            ),
            Err(ActuatorControllerEditError::MultipleOscillatorGoals)
        );

        controller.goals.pop();
        controller.controller = ControllerKind::Feedback;
        assert_eq!(
            apply_actuator_controller_edit(
                &mut actuator,
                &mut controller,
                angular_edit()
            ),
            Err(ActuatorControllerEditError::UnsupportedController)
        );
    }
}

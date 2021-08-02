use signals::A320SignalTable;
use systems::simulation::UpdateContext;
use warnings::*;

mod signals;
mod warnings;

pub(super) struct A320FlightWarningComputer {
    new_ground_def: NewGroundActivation,
    ground_detection: GroundDetectionActivation,
    speed_detection: SpeedDetectionActivation,
    engines_not_running: EnginesNotRunning,
    both_engine_running: EngRunningActivation,
    altitude_def: AltitudeDefActivation,
    cfm_flight_phases: CfmFlightPhasesDefActivation,
    flight_phases_ground: FlightPhasesGround,
    flight_phases_air: FlightPhasesAir,
}

impl A320FlightWarningComputer {
    pub fn new() -> Self {
        Self {
            new_ground_def: NewGroundActivation::new(),
            ground_detection: GroundDetectionActivation::new(),
            speed_detection: SpeedDetectionActivation::new(),
            engines_not_running: EnginesNotRunning::new(),
            both_engine_running: EngRunningActivation::new(),
            altitude_def: AltitudeDefActivation::new(),
            cfm_flight_phases: CfmFlightPhasesDefActivation::new(),
            flight_phases_ground: FlightPhasesGround::new(),
            flight_phases_air: FlightPhasesAir::new(),
        }
    }

    fn update(&mut self, context: &UpdateContext, signals: &A320SignalTable) {
        self.new_ground_def.update(context, signals);
        self.ground_detection
            .update(context, signals, &self.new_ground_def);

        self.speed_detection.update(context, signals);
        self.engines_not_running
            .update(context, signals, &self.ground_detection);
        self.both_engine_running
            .update(context, signals, &self.engines_not_running);
        self.altitude_def.update(context, signals);
        // todo self.cfm_flight_phases.update(context, (), self.altitude_def)

        self.flight_phases_ground.update(
            context,
            signals,
            &self.ground_detection,
            &self.speed_detection,
            &self.both_engine_running,
            &self.cfm_flight_phases,
        );

        self.flight_phases_air.update(
            context,
            &self.ground_detection,
            &self.altitude_def,
            &self.cfm_flight_phases,
            &self.flight_phases_ground,
        );
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;
    use uom::si::f64::*;
    use uom::si::{
        acceleration::foot_per_second_squared, length::foot,
        thermodynamic_temperature::degree_celsius, velocity::knot,
    };

    use super::*;
    use systems::flight_warning::parameters::{Arinc429Parameter, DiscreteParameter};

    #[test]
    fn when_spawning_cold_and_dark() {
        let mut fwc = A320FlightWarningComputer::new();
        let mut signals = A320SignalTable::new();
        signals.set_ess_lh_lg_compressed(DiscreteParameter::new(true));
        signals.set_norm_lh_lg_compressed(DiscreteParameter::new(true));
        signals.set_lh_lg_compressed_1(Arinc429Parameter::new(true));
        signals.set_lh_lg_compressed_2(Arinc429Parameter::new(true));
        fwc.update(&gnd_context(Duration::from_secs(1)), &signals);
        //assert_eq!(fwc.flight_phases_ground.phase_1(), true);
    }

    fn gnd_context(delta_time: Duration) -> UpdateContext {
        UpdateContext::new(
            delta_time,
            Velocity::new::<knot>(0.),
            Length::new::<foot>(0.),
            ThermodynamicTemperature::new::<degree_celsius>(25.0),
            true,
            Acceleration::new::<foot_per_second_squared>(0.),
        )
    }
}

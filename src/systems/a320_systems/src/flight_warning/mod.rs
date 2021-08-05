use parameters::A320FWCParameterTable;
use systems::flight_warning::parameters::{Arinc429Parameter, DiscreteParameter};
use systems::simulation::UpdateContext;
use warnings::*;

mod parameters;
mod test;
mod warnings;

pub(super) struct A320FlightWarningComputer {
    index: usize,
    new_ground_def: NewGroundActivation,
    ground_detection: GroundDetectionActivation,
    speed_detection: SpeedDetectionActivation,
    engines_not_running: EnginesNotRunning,
    both_engine_running: EngRunningActivation,
    altitude_def: AltitudeDefActivation,
    eng_take_off_cfm: EngTakeOffCfmActivation,
    tla_pwr_reverse: TlaPwrReverseActivation,
    tla_at_mct_or_flex_to_cfm: TlaAtMctOrFlexToCfmActivation,
    tla_at_cl_cfm: TlaAtClCfmActivation,
    neo_ecu: NeoEcuActivation,
    cfm_flight_phases: CfmFlightPhasesDefActivation,
    flight_phases_ground: FlightPhasesGroundActivation,
    flight_phases_air: FlightPhasesAirActivation,
}

impl A320FlightWarningComputer {
    pub fn new(index: usize) -> Self {
        Self {
            index: index,
            new_ground_def: NewGroundActivation::new(),
            ground_detection: GroundDetectionActivation::new(),
            speed_detection: SpeedDetectionActivation::new(),
            engines_not_running: EnginesNotRunning::new(),
            both_engine_running: EngRunningActivation::new(),
            altitude_def: AltitudeDefActivation::new(),
            eng_take_off_cfm: EngTakeOffCfmActivation::new(),
            tla_pwr_reverse: TlaPwrReverseActivation::new(),
            tla_at_mct_or_flex_to_cfm: TlaAtMctOrFlexToCfmActivation::new(),
            tla_at_cl_cfm: TlaAtClCfmActivation::new(),
            neo_ecu: NeoEcuActivation::new(),
            cfm_flight_phases: CfmFlightPhasesDefActivation::new(),
            flight_phases_ground: FlightPhasesGroundActivation::new(),
            flight_phases_air: FlightPhasesAirActivation::new(),
        }
    }

    pub fn index(&self) -> usize {
        self.index
    }

    /// Acquires the parameter table for further processing by the FWC. In future this method may
    /// acquire the data through the opposite FWC and the SDACs.
    fn acquire(&self, _context: &UpdateContext) -> A320FWCParameterTable {
        A320FWCParameterTable::new()
    }

    fn update_flight_phase(&mut self, context: &UpdateContext, parameters: &A320FWCParameterTable) {
        self.new_ground_def.update(context, parameters);
        self.ground_detection
            .update(context, parameters, &self.new_ground_def);

        self.speed_detection.update(context, parameters);
        self.engines_not_running
            .update(context, parameters, &self.ground_detection);
        self.both_engine_running
            .update(context, parameters, &self.engines_not_running);
        self.altitude_def.update(context, parameters);

        self.eng_take_off_cfm.update(context, parameters);
        self.tla_pwr_reverse
            .update(context, parameters, &self.eng_take_off_cfm);
        self.cfm_flight_phases.update(
            context,
            parameters,
            &self.neo_ecu,
            &self.tla_at_mct_or_flex_to_cfm,
            &self.tla_pwr_reverse,
            &self.altitude_def,
            &self.tla_at_cl_cfm,
        );

        self.flight_phases_ground.update(
            context,
            parameters,
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

    pub fn update(&mut self, context: &UpdateContext, parameters: &A320FWCParameterTable) {
        //let parameters = self.acquire(context);
        self.update_flight_phase(context, parameters);
    }

    /// This method can be called to assert that exactly and only the supplied flight phase is
    /// currently active.
    pub fn assert_exact_flight_phase(self, flight_phase: usize) {
        assert!(
            !((flight_phase == 1) ^ self.flight_phases_ground.phase_1()),
            if flight_phase == 1 {
                "Flight phase 1 wasn't active"
            } else {
                "Flight phase 1 was active"
            }
        );
        assert!(
            !((flight_phase == 2) ^ self.flight_phases_ground.phase_2()),
            if flight_phase == 2 {
                "Flight phase 2 wasn't active"
            } else {
                "Flight phase 2 was active"
            }
        );
        assert!(
            !((flight_phase == 3) ^ self.flight_phases_ground.phase_3()),
            if flight_phase == 3 {
                "Flight phase 3 wasn't active"
            } else {
                "Flight phase 3 was active"
            }
        );
        assert!(
            !((flight_phase == 4) ^ self.flight_phases_ground.phase_4()),
            if flight_phase == 4 {
                "Flight phase 4 wasn't active"
            } else {
                "Flight phase 4 was active"
            }
        );
        assert!(
            !((flight_phase == 5) ^ self.flight_phases_air.phase_5()),
            if flight_phase == 5 {
                "Flight phase 5 wasn't active"
            } else {
                "Flight phase 5 was active"
            }
        );
        assert!(
            !((flight_phase == 6) ^ self.flight_phases_air.phase_6()),
            if flight_phase == 6 {
                "Flight phase 6 wasn't active"
            } else {
                "Flight phase 6 was active"
            }
        );
        assert!(
            !((flight_phase == 7) ^ self.flight_phases_air.phase_7()),
            if flight_phase == 7 {
                "Flight phase 7 wasn't active"
            } else {
                "Flight phase 7 was active"
            }
        );
        assert!(
            !((flight_phase == 8) ^ self.flight_phases_ground.phase_8()),
            if flight_phase == 8 {
                "Flight phase 8 wasn't active"
            } else {
                "Flight phase 8 was active"
            }
        );
        assert!(
            !((flight_phase == 9) ^ self.flight_phases_ground.phase_9()),
            if flight_phase == 9 {
                "Flight phase 9 wasn't active"
            } else {
                "Flight phase 9 was active"
            }
        );
        assert!(
            !((flight_phase == 10) ^ self.flight_phases_ground.phase_10()),
            if flight_phase == 10 {
                "Flight phase 10 wasn't active"
            } else {
                "Flight phase 10 was active"
            }
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

    #[cfg(test)]
    mod flight_warning_computer_tests {
        use super::*;
        use crate::flight_warning::test::test_bed;
        use uom::si::angle::radian;

        #[test]
        fn when_spawning_cold_and_dark_is_phase_1() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(1)),
                &test_bed().on_ground().parameters(),
            );
            fwc.assert_exact_flight_phase(1);
        }

        #[test]
        fn when_first_engine_running_for_30_sec_is_phase_2() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed().on_ground().one_engine_running().parameters(),
            );
            fwc.assert_exact_flight_phase(2);
        }

        #[test]
        fn when_engines_at_takeoff_power_is_phase_3() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed()
                    .on_ground()
                    .engines_running()
                    .engines_at_takeoff_power()
                    .parameters(),
            );
            fwc.assert_exact_flight_phase(3);
        }

        #[test]
        fn when_above_80_knots_is_phase_4() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed()
                    .on_ground()
                    .engines_running()
                    .engines_at_takeoff_power()
                    .computed_speeds(
                        Velocity::new::<knot>(85.0),
                        Velocity::new::<knot>(85.0),
                        Velocity::new::<knot>(85.0),
                    )
                    .parameters(),
            );
            fwc.assert_exact_flight_phase(4);
        }

        #[test]
        fn when_airborne_is_phase_5() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed()
                    .engines_running()
                    .engines_at_takeoff_power()
                    .radio_heights(Length::new::<foot>(10.0), Length::new::<foot>(10.0))
                    .computed_speeds(
                        Velocity::new::<knot>(157.0),
                        Velocity::new::<knot>(157.0),
                        Velocity::new::<knot>(157.0),
                    )
                    .parameters(),
            );
            fwc.assert_exact_flight_phase(5);
        }

        #[test]
        fn when_above_1500ft_is_phase_6() {
            let mut fwc = A320FlightWarningComputer::new(1);
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed()
                    .engines_running()
                    .engines_at_takeoff_power()
                    .radio_heights(Length::new::<foot>(1550.0), Length::new::<foot>(1550.0))
                    .computed_speeds(
                        Velocity::new::<knot>(180.0),
                        Velocity::new::<knot>(180.0),
                        Velocity::new::<knot>(180.0),
                    )
                    .parameters(),
            );
            fwc.assert_exact_flight_phase(6);
        }

        #[test]
        fn when_below_800ft_is_phase_7() {
            let mut fwc = A320FlightWarningComputer::new(1);
            let mut test_bed = test_bed()
                .engines_running()
                .engines_at_takeoff_power()
                .radio_heights(Length::new::<foot>(1550.0), Length::new::<foot>(1550.0))
                .computed_speeds(
                    Velocity::new::<knot>(180.0),
                    Velocity::new::<knot>(180.0),
                    Velocity::new::<knot>(180.0),
                );
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed.parameters(),
            );
            test_bed = test_bed
                .engines_at_idle()
                .radio_heights(Length::new::<foot>(750.0), Length::new::<foot>(750.0));
            fwc.update(
                &gnd_context(Duration::from_secs(30)),
                &test_bed.parameters(),
            );
            fwc.assert_exact_flight_phase(7);
        }

        fn gnd_context(delta_time: Duration) -> UpdateContext {
            UpdateContext::new(
                delta_time,
                Velocity::new::<knot>(0.),
                Length::new::<foot>(0.),
                ThermodynamicTemperature::new::<degree_celsius>(25.0),
                true,
                Acceleration::new::<foot_per_second_squared>(0.),
                Acceleration::new::<foot_per_second_squared>(0.),
                Acceleration::new::<foot_per_second_squared>(0.),
                Angle::new::<radian>(0.),
                Angle::new::<radian>(0.),
            )
        }
    }
}

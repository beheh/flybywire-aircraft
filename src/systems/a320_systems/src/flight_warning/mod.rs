use super::hydraulic::A320Hydraulic;
use parameters::A320FWCParameterTable;
use std::time::Duration;
use systems::failures::{Failure, FailureType};
use systems::shared::{ElectricalBusType, ElectricalBuses};
use systems::simulation::{
    InitContext, SimulationElement, SimulationElementVisitor, SimulatorReader, SimulatorWriter,
    UpdateContext, VariableIdentifier, Write, Writer,
};
use warnings::*;

mod parameters;
mod test;
mod warnings;

/// This struct represents a simulation of the software runtime that is executed on an A320
/// Flight Warning Computer. It's task is to acquire data, run warning logic and generate
/// appropriate warnings.
struct A320FlightWarningComputerRuntime {
    ready: bool,
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
    lg_downlocked: LgDownlockedActivation,
    audio_attenuation: AudioAttenuationActivation,
    ap_off_voluntarily: AutoFlightAutopilotOffVoluntaryActivation,
    ap_off_unvoluntarily: AutoFlightAutopilotOffUnvoluntaryActivation,
    auto_flight_baro_altitude: AutoFlightBaroAltitudeActivation,
    altitude_alert: AltitudeAlertActivation,
    altitude_alert_thresholds: AltitudeAlertThresholdsActivation,
    altitude_alert_inhibit: AltitudeAlertGeneralInhibitActivation,
    altitude_alert_slats: AltitudeAlertSlatInhibitActivation,
    altitude_alert_fmgc: AltitudeAlertFmgcInhibitActivation,
    altitude_alert_ap_tcas: AltitudeAlertApTcasInhibitActivation,
    to_memo: ToMemoActivation,
    ldg_memo: LdgMemoActivation,
}

impl A320FlightWarningComputerRuntime {
    fn new() -> Self {
        Self::default()
    }

    fn update(&mut self, delta: Duration, parameters: &A320FWCParameterTable) {
        self.update_flight_phase(delta, parameters);
    }

    fn ready(&self) -> bool {
        self.ready
    }

    fn update_flight_phase(&mut self, delta: Duration, parameters: &A320FWCParameterTable) {
        // Flight Phases
        self.new_ground_def.update(delta, parameters);
        self.ground_detection
            .update(delta, parameters, &self.new_ground_def);

        self.speed_detection.update(delta, parameters);
        self.engines_not_running
            .update(delta, parameters, &self.ground_detection);
        self.both_engine_running
            .update(delta, parameters, &self.engines_not_running);
        self.altitude_def.update(delta, parameters);

        self.neo_ecu.update(parameters);
        self.tla_at_mct_or_flex_to_cfm.update(parameters);
        self.eng_take_off_cfm.update(parameters);
        self.tla_pwr_reverse
            .update(delta, parameters, &self.eng_take_off_cfm);
        self.cfm_flight_phases.update(
            delta,
            parameters,
            &self.neo_ecu,
            &self.tla_at_mct_or_flex_to_cfm,
            &self.tla_pwr_reverse,
            &self.altitude_def,
            &self.tla_at_cl_cfm,
        );

        self.flight_phases_ground.update(
            delta,
            parameters,
            &self.ground_detection,
            &self.speed_detection,
            &self.both_engine_running,
            &self.cfm_flight_phases,
        );

        self.flight_phases_air.update(
            delta,
            &self.ground_detection,
            &self.altitude_def,
            &self.cfm_flight_phases,
            &self.flight_phases_ground,
        );

        // Callouts

        self.lg_downlocked.update(parameters);

        let cavalry_charge_emitted = self.cavalry_charge(); // TODO feedback from monitor

        self.ap_off_voluntarily
            .update(delta, parameters, cavalry_charge_emitted);

        self.ap_off_unvoluntarily.update(
            delta,
            parameters,
            &self.ap_off_voluntarily,
            &self.flight_phases_ground,
            cavalry_charge_emitted,
        );

        self.auto_flight_baro_altitude.update(parameters);

        self.altitude_alert_thresholds
            .update(parameters, &self.auto_flight_baro_altitude);

        self.altitude_alert_slats.update(&self.lg_downlocked);
        self.altitude_alert_fmgc.update(parameters);
        self.altitude_alert_inhibit.update(
            parameters,
            &self.altitude_alert_slats,
            &self.altitude_alert_fmgc,
        );

        self.altitude_alert_ap_tcas.update(
            delta,
            parameters,
            &self.lg_downlocked,
            &self.altitude_alert_thresholds,
            &self.altitude_alert_inhibit,
        );

        self.altitude_alert.update(
            delta,
            parameters,
            &self.ground_detection,
            &self.ap_off_voluntarily,
            &self.altitude_alert_ap_tcas,
            &self.altitude_alert_thresholds,
            &self.altitude_alert_inhibit,
            &self.lg_downlocked,
        );

        self.audio_attenuation
            .update(&self.ground_detection, &self.engines_not_running);

        // Other

        self.to_memo.update(
            delta,
            parameters,
            &self.engines_not_running,
            &self.flight_phases_ground,
            &self.flight_phases_air,
        );
        self.ldg_memo.update(
            delta,
            parameters,
            &self.flight_phases_ground,
            &self.flight_phases_air,
            &self.lg_downlocked,
            &self.to_memo,
        );
    }

    /// This method queries the corresponding activations to find the first phase that is currently
    /// active. It defaults to phase 6 if no phase appears active due to an unusual set of
    /// parameters.
    pub fn flight_phase(&self) -> u8 {
        if self.flight_phases_ground.phase_1() {
            1
        } else if self.flight_phases_ground.phase_2() {
            2
        } else if self.flight_phases_ground.phase_3() {
            3
        } else if self.flight_phases_ground.phase_4() {
            4
        } else if self.flight_phases_air.phase_5() {
            5
        } else if self.flight_phases_air.phase_6() {
            6
        } else if self.flight_phases_air.phase_7() {
            7
        } else if self.flight_phases_ground.phase_8() {
            8
        } else if self.flight_phases_ground.phase_9() {
            9
        } else if self.flight_phases_ground.phase_10() {
            10
        } else {
            6 // default to 6 for now
        }
    }

    pub fn audio_attenuation(&self) -> bool {
        self.audio_attenuation.audio_attenuation()
    }

    pub fn c_chord(&self) -> bool {
        self.altitude_alert.c_chord()
    }

    pub fn altitude_alert_steady_light(&self) -> bool {
        self.altitude_alert.steady_light()
    }

    pub fn altitude_alert_flashing(&self) -> bool {
        self.altitude_alert.flashing_light()
    }

    pub fn cavalry_charge(&self) -> bool {
        self.ap_off_voluntarily.ap_off_audio() || self.ap_off_unvoluntarily.ap_off_audio()
    }
}

impl Default for A320FlightWarningComputerRuntime {
    fn default() -> Self {
        Self {
            ready: false,
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
            audio_attenuation: Default::default(),
            ap_off_voluntarily: Default::default(),
            ap_off_unvoluntarily: Default::default(),
            auto_flight_baro_altitude: Default::default(),
            altitude_alert: Default::default(),
            altitude_alert_thresholds: Default::default(),
            altitude_alert_inhibit: Default::default(),
            altitude_alert_slats: Default::default(),
            altitude_alert_fmgc: Default::default(),
            lg_downlocked: Default::default(),
            to_memo: Default::default(),
            ldg_memo: Default::default(),
            altitude_alert_ap_tcas: Default::default(),
        }
    }
}

/// This struct represents a physical flight warning computer, as installed on an A320.
///
pub(super) struct A320FlightWarningComputer {
    powered_by: ElectricalBusType,
    is_powered: bool,

    // how long the FWC can tolerate a power transient before losing it's internal state
    transient_power_tolerance: Duration,

    unpowered_for: Duration,

    failure: Failure,

    runtime: Option<A320FlightWarningComputerRuntime>,
}

impl A320FlightWarningComputer {
    pub fn new(context: &mut InitContext, number: usize, powered_by: ElectricalBusType) -> Self {
        Self {
            powered_by,
            transient_power_tolerance: Duration::from_millis(500),
            unpowered_for: Duration::ZERO,
            is_powered: false,
            failure: Failure::new(FailureType::FlightWarningComputer(number)),
            runtime: None,
        }
    }

    /// Acquires the parameter table for further processing by the FWC. In future this method may
    /// acquire the data through the opposite FWC and the SDACs.
    fn acquire(&self, _context: &UpdateContext) -> A320FWCParameterTable {
        A320FWCParameterTable::new()
    }

    pub fn update(&mut self, context: &UpdateContext, parameters: &A320FWCParameterTable) {
        // Check for power and tick power capacitor holdover otherwise
        if self.is_powered {
            self.unpowered_for = Duration::ZERO;
        } else {
            self.unpowered_for += context.delta();
        }

        if self.unpowered_for < self.transient_power_tolerance && !self.failure.is_active() {
            // Either initialize and run or continue running the existing runtime
            let runtime = self
                .runtime
                .get_or_insert_with(|| A320FlightWarningComputerRuntime::default());
            runtime.update(context.delta(), parameters);
        } else {
            // Throw away the simulated software runtime
            self.runtime = None;
        }
    }

    fn runtime(&self) -> &Option<A320FlightWarningComputerRuntime> {
        &self.runtime
    }
}

impl SimulationElement for A320FlightWarningComputer {
    /*fn write(&self, writer: &mut SimulatorWriter) {
        match self.runtime() {
            Some(runtime) => writer.write(&self.fwc_flight_phase_id, runtime.flight_phase()),
            None => writer.write(&self.fwc_flight_phase_id, 0),
        }
    }*/

    fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        self.is_powered = buses.is_powered(self.powered_by);
    }
}

pub(super) struct A320FlightWarningSystem {
    fwc1: A320FlightWarningComputer,
    fwc2: A320FlightWarningComputer,

    flight_phase_id: VariableIdentifier,
    to_memo_id: VariableIdentifier,
    ldg_memo_id: VariableIdentifier,
    audio_attenuation_id: VariableIdentifier,
}

impl A320FlightWarningSystem {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            fwc1: A320FlightWarningComputer::new(
                context,
                1,
                ElectricalBusType::AlternatingCurrentEssential,
            ),
            fwc2: A320FlightWarningComputer::new(
                context,
                2,
                ElectricalBusType::AlternatingCurrent(2),
            ),
            flight_phase_id: context.get_identifier("FWC_FLIGHT_PHASE".to_owned()),
            to_memo_id: context.get_identifier("FWC_TOMEMO".to_owned()),
            ldg_memo_id: context.get_identifier("FWC_TOMEMO".to_owned()),
            audio_attenuation_id: context.get_identifier("FWC_AUDIO_ATTENUATION".to_owned()),
        }
    }

    pub fn update(&mut self, context: &UpdateContext, hydraulic: &A320Hydraulic) {
        self.fwc1.update(context, &A320FWCParameterTable::new());
        self.fwc2.update(context, &A320FWCParameterTable::new());
    }
}

impl SimulationElement for A320FlightWarningSystem {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where
        Self: Sized,
    {
        self.fwc1.accept(visitor);
        self.fwc2.accept(visitor);

        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.flight_phase_id, 0);
        writer.write(&self.to_memo_id, false);
        writer.write(&self.ldg_memo_id, false);
        writer.write(&self.audio_attenuation_id, false);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;
    use uom::si::f64::*;
    use uom::si::{length::foot, velocity::knot};

    use super::*;

    #[cfg(test)]
    mod flight_warning_computer_runtime_tests {
        use super::*;
        use crate::flight_warning::test::test_bed;

        mod flight_phase_tests {
            use super::*;

            trait FlightPhaseAssertions {
                /// This method can be called to assert that exactly and only the supplied flight phase is
                /// currently active.
                fn assert_exact_flight_phase(self, flight_phase: usize);
            }

            impl FlightPhaseAssertions for A320FlightWarningComputerRuntime {
                fn assert_exact_flight_phase(self, flight_phase: usize) {
                    assert!(
                        !((flight_phase == 1) ^ self.flight_phases_ground.phase_1()),
                        "{}",
                        if flight_phase == 1 {
                            "Flight phase 1 wasn't active"
                        } else {
                            "Flight phase 1 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 2) ^ self.flight_phases_ground.phase_2()),
                        "{}",
                        if flight_phase == 2 {
                            "Flight phase 2 wasn't active"
                        } else {
                            "Flight phase 2 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 3) ^ self.flight_phases_ground.phase_3()),
                        "{}",
                        if flight_phase == 3 {
                            "Flight phase 3 wasn't active"
                        } else {
                            "Flight phase 3 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 4) ^ self.flight_phases_ground.phase_4()),
                        "{}",
                        if flight_phase == 4 {
                            "Flight phase 4 wasn't active"
                        } else {
                            "Flight phase 4 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 5) ^ self.flight_phases_air.phase_5()),
                        "{}",
                        if flight_phase == 5 {
                            "Flight phase 5 wasn't active"
                        } else {
                            "Flight phase 5 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 6) ^ self.flight_phases_air.phase_6()),
                        "{}",
                        if flight_phase == 6 {
                            "Flight phase 6 wasn't active"
                        } else {
                            "Flight phase 6 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 7) ^ self.flight_phases_air.phase_7()),
                        "{}",
                        if flight_phase == 7 {
                            "Flight phase 7 wasn't active"
                        } else {
                            "Flight phase 7 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 8) ^ self.flight_phases_ground.phase_8()),
                        "{}",
                        if flight_phase == 8 {
                            "Flight phase 8 wasn't active"
                        } else {
                            "Flight phase 8 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 9) ^ self.flight_phases_ground.phase_9()),
                        "{}",
                        if flight_phase == 9 {
                            "Flight phase 9 wasn't active"
                        } else {
                            "Flight phase 9 was active"
                        }
                    );
                    assert!(
                        !((flight_phase == 10) ^ self.flight_phases_ground.phase_10()),
                        "{}",
                        if flight_phase == 10 {
                            "Flight phase 10 wasn't active"
                        } else {
                            "Flight phase 10 was active"
                        }
                    );
                }
            }

            #[test]
            fn when_spawning_cold_and_dark_is_phase_1() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(Duration::from_secs(1), &test_bed().on_ground().parameters());
                runtime.assert_exact_flight_phase(1);
            }

            #[test]
            fn when_first_engine_running_for_30_sec_is_phase_2() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
                    &test_bed().on_ground().one_engine_running().parameters(),
                );
                runtime.assert_exact_flight_phase(2);
            }

            #[test]
            fn when_engines_at_takeoff_power_is_phase_3() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
                    &test_bed()
                        .on_ground()
                        .engines_running()
                        .engines_at_takeoff_power()
                        .parameters(),
                );
                runtime.assert_exact_flight_phase(3);
            }

            #[test]
            fn when_above_80_knots_is_phase_4() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
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
                runtime.assert_exact_flight_phase(4);
            }

            #[test]
            fn when_airborne_is_phase_5() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
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
                runtime.assert_exact_flight_phase(5);
            }

            #[test]
            fn when_above_1500ft_is_phase_6() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
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
                runtime.assert_exact_flight_phase(6);
            }

            #[test]
            fn when_below_800ft_is_phase_7() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                let mut test_bed = test_bed()
                    .engines_running()
                    .engines_at_takeoff_power()
                    .radio_heights(Length::new::<foot>(1550.0), Length::new::<foot>(1550.0))
                    .computed_speeds(
                        Velocity::new::<knot>(180.0),
                        Velocity::new::<knot>(180.0),
                        Velocity::new::<knot>(180.0),
                    );
                runtime.update(Duration::from_secs(30), &test_bed.parameters());
                test_bed = test_bed
                    .engines_at_idle()
                    .radio_heights(Length::new::<foot>(750.0), Length::new::<foot>(750.0));
                runtime.update(Duration::from_secs(30), &test_bed.parameters());
                runtime.assert_exact_flight_phase(7);
            }
        }

        mod audio_attenuation_tests {
            use super::*;

            #[test]
            fn when_both_engines_off_and_on_ground_audio_is_attenuated() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                let mut test_bed = test_bed().on_ground();
                runtime.update(Duration::from_secs(30), &test_bed.parameters());
                assert!(runtime.audio_attenuation());
            }

            #[test]
            fn when_both_engines_off_and_in_air_audio_is_not_attenuated() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                let mut test_bed = test_bed();
                runtime.update(Duration::from_secs(30), &test_bed.parameters());
                assert!(!runtime.audio_attenuation());
            }

            #[test]
            fn when_both_engines_running_and_on_ground_audio_is_not_attenuated() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                let mut test_bed = test_bed().on_ground().with().engines_running();
                runtime.update(Duration::from_secs(30), &test_bed.parameters());
                assert!(!runtime.audio_attenuation());
            }
        }
    }
}

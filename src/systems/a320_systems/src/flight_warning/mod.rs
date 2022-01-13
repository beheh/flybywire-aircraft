use super::hydraulic::A320Hydraulic;
use parameters::A320FWCParameterTable;
use std::time::Duration;
use systems::engine::leap_engine::LeapEngine;
use systems::engine::EngineFireOverheadPanel;
use systems::failures::{Failure, FailureType};
use systems::flight_warning::parameters::{Arinc429Parameter, DiscreteParameter};
use systems::landing_gear::LandingGearControlInterfaceUnit;
use systems::navigation::adirs::AirDataInertialReferenceSystem;
use systems::shared::arinc429::SignStatus;
use systems::shared::{
    ConsumePower, ElectricalBusType, ElectricalBuses, EngineCorrectedN1, EngineFirePushButtons,
    HydraulicSysLowPressure, LgciuGearExtension, LgciuWeightOnWheels, PowerConsumptionReport,
};
use systems::simulation::{
    InitContext, Read, Reader, SimulationElement, SimulationElementVisitor, SimulatorReader,
    SimulatorWriter, UpdateContext, VariableIdentifier, Write, Writer,
};
use uom::si::angle::degree;
use uom::si::{f64::*, length::foot, ratio::percent};

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
    altitude_alert_c_chord: AltitudeAlertCChordActivation,
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
        self.tla_at_cl_cfm.update(delta, parameters);
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

        self.altitude_alert_c_chord
            .update(parameters, &self.altitude_alert);

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

    pub fn show_to_memo(&self) -> bool {
        self.to_memo.to_memo_computed()
    }

    pub fn show_ldg_memo(&self) -> bool {
        self.ldg_memo.ldg_memo()
    }

    pub fn audio_attenuation(&self) -> bool {
        self.audio_attenuation.audio_attenuation()
    }

    pub fn c_chord(&self) -> bool {
        self.altitude_alert_c_chord.c_chord()
    }

    pub fn alt_alert_light_on(&self) -> bool {
        // TODO this should be happening in a new sheet in the runtime
        self.altitude_alert.steady_light() || self.altitude_alert.flashing_light()
    }

    pub fn alt_alert_flashing_light(&self) -> bool {
        // TODO this should be happening in a new sheet in the runtime
        self.altitude_alert.flashing_light() && !self.altitude_alert.steady_light()
    }

    pub fn cavalry_charge(&self) -> bool {
        self.ap_off_voluntarily.ap_off_audio() || self.ap_off_unvoluntarily.ap_off_audio()
    }

    pub fn ap_off_text(&self) -> bool {
        self.ap_off_voluntarily.ap_off_text()
    }

    pub fn ap_off_warning(&self) -> bool {
        self.ap_off_unvoluntarily.ap_off_warning()
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
            altitude_alert_c_chord: Default::default(),
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
    eng1_tla_id: VariableIdentifier,
    eng2_tla_id: VariableIdentifier,
    eng1_master_lever_id: VariableIdentifier,
    eng2_master_lever_id: VariableIdentifier,
    ap1_active_id: VariableIdentifier,
    ap2_active_id: VariableIdentifier,
    ap_altitude_lock_id: VariableIdentifier,
    fma_active_vertical_mode_id: VariableIdentifier,
    mw_cancel_on_capt_id: VariableIdentifier,
    mw_cancel_on_fo_id: VariableIdentifier,

    fwc1: A320FlightWarningComputer,
    fwc2: A320FlightWarningComputer,

    eng1_tla: Angle,
    eng2_tla: Angle,
    eng1_master_lever: u8,
    eng2_master_lever: u8,
    ap1_active: bool,
    ap2_active: bool,
    ap_altitude_lock: Length,
    ap_altitude_lock_changed: bool,
    active_vertical_mode: u8,
    mw_cancel_on_capt: bool,
    mw_cancel_on_fo: bool,

    fwc1_normal_id: VariableIdentifier,
    fwc2_normal_id: VariableIdentifier,
    flight_phase_id: VariableIdentifier,
    to_memo_id: VariableIdentifier,
    ldg_memo_id: VariableIdentifier,
    audio_attenuation_id: VariableIdentifier,
    ap_off_text_id: VariableIdentifier,
    ap_off_warning_id: VariableIdentifier,
    alt_deviation_id: VariableIdentifier,
    alt_alert_steady_light_id: VariableIdentifier,
    alt_alert_flashing_light_id: VariableIdentifier,
}

impl A320FlightWarningSystem {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            eng1_tla_id: context.get_identifier("AUTOTHRUST_TLA:1".to_owned()),
            eng2_tla_id: context.get_identifier("AUTOTHRUST_TLA:2".to_owned()),
            eng1_master_lever_id: context.get_identifier("TURB ENG IGNITION SWITCH:1".to_owned()),
            eng2_master_lever_id: context.get_identifier("TURB ENG IGNITION SWITCH:2".to_owned()),
            ap1_active_id: context.get_identifier("AUTOPILOT_1_ACTIVE".to_owned()),
            ap2_active_id: context.get_identifier("AUTOPILOT_2_ACTIVE".to_owned()),
            ap_altitude_lock_id: context.get_identifier("AUTOPILOT ALTITUDE LOCK VAR:3".to_owned()),
            fma_active_vertical_mode_id: context.get_identifier("FMA_VERTICAL_MODE".to_owned()),
            mw_cancel_on_capt_id: context.get_identifier("FWS_MW_CANCEL_ON_CAPT".to_owned()),
            mw_cancel_on_fo_id: context.get_identifier("FWS_MW_CANCEL_ON_FO".to_owned()),
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
            eng1_master_lever: 0,
            eng2_master_lever: 0,
            eng1_tla: Angle::new::<degree>(0.),
            eng2_tla: Angle::new::<degree>(0.),
            ap1_active: false,
            ap2_active: false,
            ap_altitude_lock: Length::new::<foot>(0.),
            ap_altitude_lock_changed: false,
            active_vertical_mode: 0,
            mw_cancel_on_capt: false,
            mw_cancel_on_fo: false,
            fwc1_normal_id: context.get_identifier("FWC_1_NORMAL".to_owned()),
            fwc2_normal_id: context.get_identifier("FWC_2_NORMAL".to_owned()),
            flight_phase_id: context.get_identifier("FWC_FLIGHT_PHASE".to_owned()),
            to_memo_id: context.get_identifier("FWS_TOMEMO".to_owned()),
            ldg_memo_id: context.get_identifier("FWS_LDGMEMO".to_owned()),
            audio_attenuation_id: context.get_identifier("FWS_AUDIO_ATTENUATION".to_owned()),
            ap_off_text_id: context.get_identifier("FWS_AP_OFF".to_owned()),
            ap_off_warning_id: context.get_identifier("FWS_AP_OFF_WARNING".to_owned()),
            alt_deviation_id: context.get_identifier("FWS_SOUND_ALT_DEVIATION".to_owned()),
            alt_alert_steady_light_id: context
                .get_identifier("FWS_ALT_ALERT_STEADY_LIGHT".to_owned()),
            alt_alert_flashing_light_id: context
                .get_identifier("FWS_ALT_ALERT_FLASHING_LIGHT".to_owned()),
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        lgciu1: &LandingGearControlInterfaceUnit,
        lgciu2: &LandingGearControlInterfaceUnit,
        adirs: &AirDataInertialReferenceSystem,
        engine1: &LeapEngine,
        engine2: &LeapEngine,
        engine_fire_overhead: &EngineFireOverheadPanel,
        hydraulic: &A320Hydraulic,
    ) {
        let parameters = self.acquire_parameters(
            context,
            lgciu1,
            lgciu2,
            adirs,
            engine1,
            engine2,
            engine_fire_overhead,
            hydraulic,
        );
        self.fwc1.update(context, &parameters);
        self.fwc2.update(context, &parameters);
    }

    pub fn acquire_parameters(
        &self,
        context: &UpdateContext,
        lgciu1: &LandingGearControlInterfaceUnit,
        lgciu2: &LandingGearControlInterfaceUnit,
        adirs: &AirDataInertialReferenceSystem,
        engine1: &LeapEngine,
        engine2: &LeapEngine,
        engine_fire_overhead: &EngineFireOverheadPanel,
        hydraulic: &A320Hydraulic,
    ) -> A320FWCParameterTable {
        let mut parameters = A320FWCParameterTable::new();

        // Landing Gear
        parameters
            .set_lh_lg_compressed_1(Arinc429Parameter::new(lgciu1.left_gear_compressed(false)));
        parameters
            .set_lh_lg_compressed_2(Arinc429Parameter::new(lgciu2.left_gear_compressed(false)));
        parameters
            .set_ess_lh_lg_compressed(DiscreteParameter::new(lgciu1.left_gear_compressed(true)));
        parameters
            .set_norm_lh_lg_compressed(DiscreteParameter::new(lgciu2.left_gear_compressed(true)));

        parameters
            .set_lh_gear_down_lock_1(Arinc429Parameter::new(lgciu1.left_gear_down_and_locked()));
        parameters
            .set_lh_gear_down_lock_2(Arinc429Parameter::new(lgciu2.left_gear_down_and_locked()));

        parameters
            .set_rh_gear_down_lock_1(Arinc429Parameter::new(lgciu1.right_gear_down_and_locked()));
        parameters
            .set_rh_gear_down_lock_2(Arinc429Parameter::new(lgciu2.right_gear_down_and_locked()));

        parameters
            .set_nose_gear_down_lock_1(Arinc429Parameter::new(lgciu1.nose_gear_down_and_locked()));
        parameters
            .set_nose_gear_down_lock_2(Arinc429Parameter::new(lgciu2.nose_gear_down_and_locked()));

        // Air Data
        let computed_speed_1 = adirs.computed_speed(1);
        parameters.set_computed_speed_1(if computed_speed_1.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(computed_speed_1.value())
        } else {
            Arinc429Parameter::new(computed_speed_1.value())
        });

        let computed_speed_2 = adirs.computed_speed(2);
        parameters.set_computed_speed_2(if computed_speed_2.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(computed_speed_2.value())
        } else {
            Arinc429Parameter::new(computed_speed_2.value())
        });

        let computed_speed_3 = adirs.computed_speed(3);
        parameters.set_computed_speed_3(if computed_speed_3.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(computed_speed_3.value())
        } else {
            Arinc429Parameter::new(computed_speed_3.value())
        });

        let altitude_1 = adirs.altitude(1);
        parameters.set_altitude_1(if altitude_1.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(altitude_1.value())
        } else {
            Arinc429Parameter::new(altitude_1.value())
        });

        let altitude_2 = adirs.altitude(2);
        parameters.set_altitude_2(if altitude_2.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(altitude_2.value())
        } else {
            Arinc429Parameter::new(altitude_2.value())
        });

        let altitude_3 = adirs.altitude(3);
        parameters.set_altitude_3(if altitude_3.ssm() != SignStatus::NormalOperation {
            Arinc429Parameter::new_inv(altitude_3.value())
        } else {
            Arinc429Parameter::new(altitude_3.value())
        });

        // Radio Altimeters
        let height_above_ground = context.height_above_ground();
        let radio_altitude = if height_above_ground <= Length::new::<foot>(3000.0) {
            Arinc429Parameter::new(height_above_ground)
        } else {
            Arinc429Parameter::new_ncd(Length::new::<foot>(0.0))
        };
        parameters.set_radio_height_1(radio_altitude.clone());
        parameters.set_radio_height_2(radio_altitude);

        // Engine data
        parameters.set_eng1_channel_a_in_control(Arinc429Parameter::new(true));
        parameters
            .set_eng1_master_lever_select_on(Arinc429Parameter::new(self.eng1_master_lever != 0));
        parameters.set_eng1_tla_a(Arinc429Parameter::new(self.eng1_tla));
        parameters.set_eng1_tla_b(Arinc429Parameter::new(self.eng1_tla));
        parameters.set_eng1_core_speed_at_or_above_idle_a(Arinc429Parameter::new(
            engine1.corrected_n1() > Ratio::new::<percent>(15.),
        ));
        parameters.set_eng1_core_speed_at_or_above_idle_b(Arinc429Parameter::new(
            engine1.corrected_n1() > Ratio::new::<percent>(15.),
        ));

        parameters.set_eng2_channel_a_in_control(Arinc429Parameter::new(true));
        parameters
            .set_eng2_master_lever_select_on(Arinc429Parameter::new(self.eng2_master_lever != 0));
        parameters.set_eng2_tla_a(Arinc429Parameter::new(self.eng2_tla));
        parameters.set_eng2_tla_b(Arinc429Parameter::new(self.eng2_tla));
        parameters.set_eng2_core_speed_at_or_above_idle_a(Arinc429Parameter::new(
            engine2.corrected_n1() > Ratio::new::<percent>(15.),
        ));
        parameters.set_eng2_core_speed_at_or_above_idle_b(Arinc429Parameter::new(
            engine2.corrected_n1() > Ratio::new::<percent>(15.),
        ));

        // Engine Fire
        parameters
            .set_eng_1_fire_pb_out(DiscreteParameter::new(engine_fire_overhead.is_released(1)));
        parameters
            .set_eng_2_fire_pb_out(DiscreteParameter::new(engine_fire_overhead.is_released(2)));

        // Autopilot
        parameters.set_ap1_engd(
            DiscreteParameter::new(self.ap1_active),
            DiscreteParameter::new(self.ap1_active),
        );
        parameters.set_ap2_engd(
            DiscreteParameter::new(self.ap2_active),
            DiscreteParameter::new(self.ap2_active),
        );

        parameters.set_alti_select(Arinc429Parameter::new(self.ap_altitude_lock));
        parameters.set_alti_select_chg(Arinc429Parameter::new(self.ap_altitude_lock_changed));

        let gs_mode_on = self.active_vertical_mode >= 30 && self.active_vertical_mode <= 34;
        parameters.set_gs_mode_on_1(Arinc429Parameter::new(gs_mode_on));
        parameters.set_gs_mode_on_2(Arinc429Parameter::new(gs_mode_on));

        parameters.set_tcas_engaged(Arinc429Parameter::new(self.active_vertical_mode == 50));

        // Hydraulics
        parameters.set_yellow_sys_lo_pr(DiscreteParameter::new(hydraulic.is_yellow_sys_lo_pr()));
        parameters.set_green_sys_lo_pr(DiscreteParameter::new(hydraulic.is_green_sys_lo_pr()));
        parameters.set_blue_sys_lo_pr(DiscreteParameter::new(hydraulic.is_blue_sys_lo_pr()));

        // Misc
        parameters.set_capt_mw_cancel_on(DiscreteParameter::new(self.mw_cancel_on_capt));
        parameters.set_fo_mw_cancel_on(DiscreteParameter::new(self.mw_cancel_on_fo));

        parameters
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

    fn read(&mut self, reader: &mut SimulatorReader) {
        self.eng1_master_lever = reader.read(&self.eng1_master_lever_id);
        self.eng2_master_lever = reader.read(&self.eng2_master_lever_id);
        self.eng1_tla = Angle::new::<degree>(reader.read(&self.eng1_tla_id));
        self.eng2_tla = Angle::new::<degree>(reader.read(&self.eng2_tla_id));
        self.ap1_active = reader.read(&self.ap1_active_id);
        self.ap2_active = reader.read(&self.ap2_active_id);
        let ap_altitude_lock = Length::new::<foot>(reader.read(&self.ap_altitude_lock_id));
        self.ap_altitude_lock_changed =
            (ap_altitude_lock.get::<foot>() - self.ap_altitude_lock.get::<foot>()).abs()
                > f64::EPSILON;
        self.ap_altitude_lock = ap_altitude_lock;
        self.active_vertical_mode = reader.read(&self.fma_active_vertical_mode_id);
        self.mw_cancel_on_capt = reader.read(&self.mw_cancel_on_capt_id);
        self.mw_cancel_on_fo = reader.read(&self.mw_cancel_on_fo_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        let runtime1 = &self.fwc1.runtime;
        let runtime2 = &self.fwc2.runtime;
        let runtime = match runtime1 {
            Some(r) => runtime1,
            None => runtime2,
        };

        let mut flight_phase = 0;
        let mut show_to_memo = false;
        let mut show_ldg_memo = false;
        let mut audio_attenuation = false;
        let mut ap_off_text = false;
        let mut ap_off_warning = false;
        let mut cchord = false;
        let mut alt_alert_steady_light = false;
        let mut alt_alert_flashing_light = false;

        match runtime {
            Some(runtime) => {
                flight_phase = runtime.flight_phase();
                show_to_memo = runtime.show_to_memo();
                show_ldg_memo = runtime.show_ldg_memo();
                audio_attenuation = runtime.audio_attenuation();
                ap_off_text = runtime.ap_off_text();
                ap_off_warning = runtime.ap_off_warning();
                cchord = runtime.c_chord();
                alt_alert_steady_light = runtime.alt_alert_light_on();
                alt_alert_flashing_light = runtime.alt_alert_flashing_light();
            }
            None => {}
        };

        writer.write(&self.fwc1_normal_id, runtime1.is_some());
        writer.write(&self.fwc2_normal_id, runtime2.is_some());
        writer.write(&self.flight_phase_id, flight_phase);
        writer.write(&self.to_memo_id, show_to_memo);
        writer.write(&self.ldg_memo_id, show_ldg_memo);
        writer.write(&self.audio_attenuation_id, audio_attenuation);
        writer.write(&self.ap_off_text_id, ap_off_text);
        writer.write(&self.ap_off_warning_id, ap_off_warning);
        writer.write(&self.alt_deviation_id, cchord);
        writer.write(&self.alt_alert_steady_light_id, alt_alert_steady_light);
        writer.write(&self.alt_alert_flashing_light_id, alt_alert_flashing_light);
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

use super::hydraulic::A320Hydraulic;
use parameters::A320FWCParameterTable;
use runtime::A320FlightWarningComputerRuntime;
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
    HydraulicSysLowPressure, LgciuGearExtension, LgciuWeightOnWheels,
};
use systems::simulation::{
    InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
    SimulatorWriter, UpdateContext, VariableIdentifier, Write,
};
use uom::si::angle::degree;
use uom::si::{f64::*, length::foot, ratio::percent};

mod parameters;
mod runtime;
mod test;

/// This struct represents a physical flight warning computer, as installed on an A320.
///
pub(super) struct A320FlightWarningComputer {
    powered_by: ElectricalBusType,
    is_powered: bool,

    // how long the FWC can tolerate a power transient before losing it's internal state
    transient_power_tolerance: Duration,

    // how long the FWC takes to fully boot up and report useful outputs
    boot_duration: Duration,

    unpowered_for: Duration,

    failure: Failure,

    runtime: Option<A320FlightWarningComputerRuntime>,
}

impl A320FlightWarningComputer {
    pub fn new(context: &mut InitContext, number: usize, powered_by: ElectricalBusType) -> Self {
        Self {
            powered_by,
            transient_power_tolerance: Duration::from_millis(500),
            boot_duration: Duration::from_secs(50),
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

    fn valid(&self) -> bool {
        if let Some(runtime) = &self.runtime {
            return runtime.ready();
        }
        false
    }
}

impl SimulationElement for A320FlightWarningComputer {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.failure.accept(visitor);

        visitor.visit(self);
    }

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
    mc_cancel_on_capt_id: VariableIdentifier,
    mc_cancel_on_fo_id: VariableIdentifier,
    decision_height_id: VariableIdentifier,
    minimum_descent_altitude_id: VariableIdentifier,
    autothrust_status_id: VariableIdentifier,

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
    mc_cancel_on_capt: bool,
    mc_cancel_on_fo: bool,
    decision_height: Option<Length>,
    minimum_descent_altitude: Option<Length>,
    autothrust_status: u8,

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
    synthetic_callout_id: VariableIdentifier,
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
            mc_cancel_on_capt_id: context.get_identifier("FWS_MC_CANCEL_ON_CAPT".to_owned()),
            mc_cancel_on_fo_id: context.get_identifier("FWS_MC_CANCEL_ON_FO".to_owned()),
            decision_height_id: context.get_identifier("DECISION_HEIGHT".to_owned()),
            minimum_descent_altitude_id: context
                .get_identifier("MINIMUM_DESCENT_ALTITUDE".to_owned()),
            autothrust_status_id: context.get_identifier("AUTOTHRUST_STATUS".to_owned()),
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
            mc_cancel_on_capt: false,
            mc_cancel_on_fo: false,
            decision_height: None,
            minimum_descent_altitude: None,
            autothrust_status: 0,
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
            synthetic_callout_id: context.get_identifier("FWS_SYNTHETIC_VOICE".to_owned()),
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
        let radio_altitude = if height_above_ground <= Length::new::<foot>(5000.0) {
            Arinc429Parameter::new(Length::new::<foot>(
                (height_above_ground.get::<foot>() * 8.0).round() / 8.0, // encoded as multiple of 1/8
            ))
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
        // TODO engine in exactly FLEX thrust mode
        //parameters.set_eng1_tla_fto_a(engine1.corrected_n1() > Ratio::new::<percent>(15.));
        //parameters.set_eng1_tla_fto_b(engine1.corrected_n1() > Ratio::new::<percent>(15.));

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

        // DMC data

        parameters.set_decision_height_1(if let Some(dh) = self.decision_height {
            Arinc429Parameter::new(dh)
        } else {
            Arinc429Parameter::new_ncd(Length::new::<foot>(0.))
        });
        parameters.set_decision_height_2(if let Some(dh) = self.decision_height {
            Arinc429Parameter::new(dh)
        } else {
            Arinc429Parameter::new_ncd(Length::new::<foot>(0.))
        });

        parameters.set_hundred_above_for_mda_mdh_request_1(DiscreteParameter::new(
            if let Some(mda) = self.minimum_descent_altitude {
                altitude_1.ssm() == SignStatus::NormalOperation
                    && altitude_1.value() <= mda + Length::new::<foot>(100.0)
            } else {
                false
            },
        ));
        parameters.set_hundred_above_for_mda_mdh_request_2(DiscreteParameter::new(
            if let Some(mda) = self.minimum_descent_altitude {
                altitude_2.ssm() == SignStatus::NormalOperation
                    && altitude_2.value() <= mda + Length::new::<foot>(100.0)
            } else {
                false
            },
        ));

        parameters.set_minimum_for_mda_mdh_request_1(DiscreteParameter::new(
            if let Some(mda) = self.minimum_descent_altitude {
                altitude_1.ssm() == SignStatus::NormalOperation && altitude_1.value() <= mda
            } else {
                false
            },
        ));
        parameters.set_minimum_for_mda_mdh_request_2(DiscreteParameter::new(
            if let Some(mda) = self.minimum_descent_altitude {
                altitude_2.ssm() == SignStatus::NormalOperation && altitude_2.value() <= mda
            } else {
                false
            },
        ));

        parameters.set_athr_engaged(Arinc429Parameter::new(self.autothrust_status == 2));

        let is_land_trk_mode = self.active_vertical_mode >= 32 && self.active_vertical_mode <= 33;
        parameters.set_land_trk_mode_on_1(Arinc429Parameter::new(is_land_trk_mode));
        parameters.set_land_trk_mode_on_2(Arinc429Parameter::new(is_land_trk_mode));

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
        self.ap_altitude_lock_changed = (ap_altitude_lock - self.ap_altitude_lock)
            .abs()
            .get::<foot>()
            > f64::EPSILON;
        self.ap_altitude_lock = ap_altitude_lock;
        self.active_vertical_mode = reader.read(&self.fma_active_vertical_mode_id);
        self.mw_cancel_on_capt = reader.read(&self.mw_cancel_on_capt_id);
        self.mw_cancel_on_fo = reader.read(&self.mw_cancel_on_fo_id);
        self.mc_cancel_on_capt = reader.read(&self.mc_cancel_on_capt_id);
        self.mc_cancel_on_fo = reader.read(&self.mc_cancel_on_fo_id);

        let dh: i32 = reader.read(&self.decision_height_id);
        self.decision_height = if dh > 0 {
            Some(Length::new::<foot>(f64::from(dh)))
        } else {
            None
        };

        let mda: i32 = reader.read(&self.minimum_descent_altitude_id);
        self.minimum_descent_altitude = if mda > 0 {
            Some(Length::new::<foot>(f64::from(mda)))
        } else {
            None
        };

        self.autothrust_status = reader.read(&self.autothrust_status_id);
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
        let mut synthetic_voice: u8 = 0;

        match runtime {
            Some(runtime) => {
                flight_phase = if let Some(fp) = runtime.flight_phase() {
                    fp
                } else {
                    0
                };
                show_to_memo = runtime.show_to_memo();
                show_ldg_memo = runtime.show_ldg_memo();
                audio_attenuation = runtime.audio_attenuation();
                ap_off_text = runtime.ap_off_text();
                ap_off_warning = runtime.ap_off_warning();
                cchord = runtime.c_chord();
                alt_alert_steady_light = runtime.alt_alert_light_on();
                alt_alert_flashing_light = runtime.alt_alert_flashing_light();
                synthetic_voice = if let Some(voice_index) = runtime.synthetic_voice_index() {
                    voice_index
                } else {
                    0
                };
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
        writer.write(&self.synthetic_callout_id, synthetic_voice);
    }
}

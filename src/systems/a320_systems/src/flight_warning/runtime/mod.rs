use super::parameters::A320FWCParameterTable;
use crate::flight_warning::runtime::monitor::{
    A320FWCMonitor, A320FWCMonitorFeedback, A320MonitorParameters,
};
use std::time::Duration;
use systems::flight_warning::warnings::WarningCode;
use warnings::*;

mod audio;
mod monitor;
mod warnings;

/// This struct represents a simulation of the software runtime that is executed on an A320
/// Flight Warning Computer. It's task is to acquire data, run warning logic and generate
/// appropriate warnings.
pub(super) struct A320FlightWarningComputerRuntime {
    ready: bool,
    monitor: A320FWCMonitor,
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
    dh_dt_positive: GeneralDhDtPositiveActivation,
    general_cancel: GeneralCancelActivation,
    lg_downlocked: LgDownlockedActivation,
    eng_1_start_sequence: Eng1StartSequenceActivation,
    eng_2_start_sequence: Eng2StartSequenceActivation,
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
    hoisted_gpws_inhibition: HoistedGpwsInhibitionActivation,
    audio_generated: AudioGeneratedActivation,
    decision_height_val: DecisionHeightValActivation,
    mda_mdh_inhib: MdaMdhInbitionActivation,
    hundred_above: HundredAboveActivation,
    minimum: MinimumActivation,
    altitude_callout_threshold1: AltitudeThreshold1Activation,
    altitude_callout_threshold2: AltitudeThreshold2Activation,
    altitude_callout_threshold3: AltitudeThreshold3Activation,
    altitude_callout_inhibit: AutomaticCallOutInhibitionActivation,
    altitude_callout_triggers1: AltitudeThresholdTriggers1Activation,
    altitude_callout_triggers2: AltitudeThresholdTriggers2Activation,
    altitude_callout_triggers3: AltitudeThresholdTriggers3Activation,
    tla_at_idle_retard: TlaAtIdleRetardActivation,
    retard_toga_inhibition: RetardTogaInhibitionActivation,
    retard_tla_inhibition: RetardTlaInhibitionActivation,
    retard_callout: AutoCallOutRetardAnnounceActivation,
    altitude_callout_5_ft: AltitudeCallout5FtAnnounceActivation,
    altitude_callout_10_ft: AltitudeCallout10FtAnnounceActivation,
    altitude_callout_20_ft: AltitudeCallout20FtAnnounceActivation,
    altitude_callout_30_ft: AltitudeCallout30FtAnnounceActivation,
    altitude_callout_40_ft: AltitudeCallout40FtAnnounceActivation,
    altitude_callout_50_ft: AltitudeCallout50FtAnnounceActivation,
    altitude_callout_100_ft: AltitudeCallout100FtAnnounceActivation,
    altitude_callout_200_ft: AltitudeCallout200FtAnnounceActivation,
    altitude_callout_300_ft: AltitudeCallout300FtAnnounceActivation,
    altitude_callout_400_ft: AltitudeCallout400FtAnnounceActivation,
    altitude_callout_500_ft: AltitudeCallout500FtAnnounceActivation,
    altitude_callout_1000_ft: AltitudeCallout1000FtAnnounceActivation,
    altitude_callout_2000_ft: AltitudeCallout2000FtAnnounceActivation,
    altitude_callout_2500_ft: AltitudeCallout2500FtAnnounceActivation,
    altitude_callout_2500b_ft: AltitudeCallout2500BFtAnnounceActivation,
    twenty_retard_callout: AutoCallOutTwentyRetardAnnounceActivation,
    ten_retard_callout: AutoCallOutTenRetardAnnounceActivation,
    altitude_callout_threshold_detection: AltitudeCalloutThresholdDetectionActivation,
    altitude_callout_intermediate_audio: IntermediateAudioActivation,
    to_memo: ToMemoActivation,
    ldg_memo: LdgMemoActivation,
}

impl Default for A320FlightWarningComputerRuntime {
    fn default() -> Self {
        Self {
            ready: false,
            monitor: A320FWCMonitor::default(),
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
            dh_dt_positive: Default::default(),
            general_cancel: Default::default(),
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
            eng_1_start_sequence: Default::default(),
            eng_2_start_sequence: Default::default(),
            to_memo: Default::default(),
            ldg_memo: Default::default(),
            altitude_alert_ap_tcas: Default::default(),
            hoisted_gpws_inhibition: Default::default(),
            audio_generated: Default::default(),
            decision_height_val: Default::default(),
            mda_mdh_inhib: Default::default(),
            hundred_above: Default::default(),
            minimum: Default::default(),
            altitude_callout_threshold1: Default::default(),
            altitude_callout_threshold2: Default::default(),
            altitude_callout_threshold3: Default::default(),
            altitude_callout_inhibit: Default::default(),
            altitude_callout_triggers1: Default::default(),
            altitude_callout_triggers2: Default::default(),
            altitude_callout_triggers3: Default::default(),
            tla_at_idle_retard: Default::default(),
            retard_toga_inhibition: Default::default(),
            retard_tla_inhibition: Default::default(),
            retard_callout: Default::default(),
            altitude_callout_5_ft: Default::default(),
            altitude_callout_10_ft: Default::default(),
            altitude_callout_20_ft: Default::default(),
            altitude_callout_30_ft: Default::default(),
            altitude_callout_40_ft: Default::default(),
            altitude_callout_50_ft: Default::default(),
            altitude_callout_100_ft: Default::default(),
            altitude_callout_200_ft: Default::default(),
            altitude_callout_300_ft: Default::default(),
            altitude_callout_400_ft: Default::default(),
            altitude_callout_500_ft: Default::default(),
            altitude_callout_1000_ft: Default::default(),
            altitude_callout_2000_ft: Default::default(),
            altitude_callout_2500_ft: Default::default(),
            altitude_callout_2500b_ft: Default::default(),
            twenty_retard_callout: Default::default(),
            ten_retard_callout: Default::default(),
            altitude_callout_threshold_detection: Default::default(),
            altitude_callout_intermediate_audio: Default::default(),
        }
    }
}

impl A320FlightWarningComputerRuntime {
    fn new() -> Self {
        Self::default()
    }

    pub(super) fn update(&mut self, delta: Duration, parameters: &A320FWCParameterTable) {
        self.update_warnings(delta, parameters);
        self.update_monitor(delta);
    }

    pub(super) fn ready(&self) -> bool {
        self.ready
    }

    fn update_warnings(&mut self, delta: Duration, parameters: &A320FWCParameterTable) {
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

        // Misc

        self.dh_dt_positive.update(parameters);
        self.general_cancel.update(parameters);

        // Audio Setup

        self.lg_downlocked.update(parameters);

        self.eng_1_start_sequence.update(delta, parameters);
        self.eng_2_start_sequence.update(
            delta,
            parameters,
            &self.flight_phases_ground,
            &self.flight_phases_air,
        );

        let cavalry_charge_emitted = self.cavalry_charge(); // TODO feedback from monitor

        // Audio Autopilot

        self.ap_off_voluntarily
            .update(delta, parameters, cavalry_charge_emitted);

        self.ap_off_unvoluntarily.update(
            delta,
            parameters,
            &self.ap_off_voluntarily,
            &self.flight_phases_ground,
            cavalry_charge_emitted,
        );

        // Audio Altitude Alert

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

        self.altitude_alert_c_chord.update(&self.altitude_alert);

        // Auto Call Out Setup
        self.hoisted_gpws_inhibition.update(delta, parameters);
        self.audio_generated.update(
            self.monitor.minimum_generated(),
            self.monitor.hundred_above_generated(),
        );

        self.altitude_callout_threshold1.update(parameters);
        self.altitude_callout_inhibit.update(
            parameters,
            &self.altitude_callout_threshold1,
            &self.ground_detection,
            &self.cfm_flight_phases,
            &self.flight_phases_ground,
            &self.eng_1_start_sequence,
            &self.eng_2_start_sequence,
        );

        // Hundred Above & Minimum Callout

        self.decision_height_val.update(parameters);
        self.mda_mdh_inhib.update(
            delta,
            parameters,
            &self.hoisted_gpws_inhibition,
            &self.decision_height_val,
            &self.altitude_callout_inhibit,
        );
        self.hundred_above.update(
            delta,
            parameters,
            &self.audio_generated,
            &self.decision_height_val,
            &self.mda_mdh_inhib,
        );
        self.minimum.update(
            delta,
            parameters,
            &self.hundred_above,
            &self.audio_generated,
            &self.decision_height_val,
            &self.mda_mdh_inhib,
        );

        // Audio Altitude Callout

        self.altitude_callout_threshold2
            .update(delta, &self.altitude_callout_threshold1);
        self.altitude_callout_threshold3.update(
            &self.hoisted_gpws_inhibition,
            &self.dh_dt_positive,
            &self.altitude_callout_threshold1,
            &self.altitude_callout_threshold2,
            &self.minimum,
        );

        self.altitude_callout_triggers1.update(
            delta,
            parameters,
            &self.altitude_callout_threshold1,
            &self.altitude_callout_threshold3,
        );
        self.altitude_callout_triggers2.update(
            parameters,
            &self.altitude_callout_threshold1,
            &self.altitude_callout_threshold3,
        );
        self.altitude_callout_triggers3.update(
            parameters,
            &self.altitude_callout_threshold1,
            &self.altitude_callout_threshold2,
            &self.altitude_callout_threshold3,
        );

        self.twenty_retard_callout.update(
            delta,
            parameters,
            &self.altitude_callout_inhibit,
            &self.tla_at_mct_or_flex_to_cfm,
            &self.flight_phases_ground,
            &self.altitude_callout_triggers3,
            &self.ap_off_voluntarily,
        );
        self.ten_retard_callout.update(
            delta,
            parameters,
            &self.altitude_callout_inhibit,
            &self.twenty_retard_callout,
            &self.altitude_callout_triggers3,
            &self.ap_off_voluntarily,
        );
        self.tla_at_idle_retard.update(parameters);
        self.retard_toga_inhibition.update(
            &self.tla_at_idle_retard,
            &self.flight_phases_ground,
            &self.flight_phases_air,
        );
        self.retard_tla_inhibition.update(
            &self.engines_not_running,
            &self.tla_pwr_reverse,
            &self.tla_at_idle_retard,
            &self.retard_toga_inhibition,
        );
        self.retard_callout.update(
            delta,
            parameters,
            &self.retard_tla_inhibition,
            &self.altitude_callout_threshold2,
            &self.cfm_flight_phases,
            &self.flight_phases_air,
            &self.flight_phases_ground,
            &self.twenty_retard_callout,
            &self.ap_off_voluntarily,
        );

        self.altitude_callout_5_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers3,
            &self.retard_callout,
        );
        self.altitude_callout_10_ft.update(
            delta,
            parameters,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers3,
            &self.ap_off_voluntarily,
            &self.altitude_callout_5_ft,
            &self.retard_callout,
        );
        self.altitude_callout_20_ft.update(
            delta,
            parameters,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers3,
            &self.ap_off_voluntarily,
            &self.altitude_callout_10_ft,
        );
        self.altitude_callout_30_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers3,
            &self.altitude_callout_20_ft,
        );
        self.altitude_callout_40_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers3,
            &self.altitude_callout_30_ft,
        );
        self.altitude_callout_50_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers2,
            &self.altitude_callout_40_ft,
        );
        self.altitude_callout_100_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers2,
        );
        self.altitude_callout_200_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers2,
        );
        self.altitude_callout_300_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers2,
        );
        self.altitude_callout_400_ft.update(
            delta,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers2,
        );
        self.altitude_callout_500_ft.update(
            delta,
            parameters,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers1,
        );
        self.altitude_callout_1000_ft.update(
            &self.altitude_callout_threshold1,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers1,
        );
        self.altitude_callout_2000_ft.update(
            &self.altitude_callout_threshold1,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers1,
        );
        self.altitude_callout_2500_ft.update(
            &self.altitude_callout_threshold1,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers1,
        );
        self.altitude_callout_2500b_ft.update(
            &self.altitude_callout_threshold1,
            &self.altitude_callout_inhibit,
            &self.altitude_callout_triggers1,
        );
        self.altitude_callout_threshold_detection.update(
            &self.altitude_callout_triggers2,
            &self.altitude_callout_triggers3,
        );
        self.altitude_callout_intermediate_audio.update(
            delta,
            &self.altitude_callout_threshold1,
            &self.altitude_callout_threshold3,
            &self.altitude_callout_threshold_detection,
            &self.minimum,
            self.monitor.auto_call_out_generated(),
            self.monitor.inter_audio(),
        );

        // Audio Attenuation

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

    fn update_monitor(&mut self, delta: Duration) {
        let mut warnings: Vec<WarningCode> = Default::default();

        if self.ten_retard_callout.audio() {
            warnings.push(WarningCode::new(34, 00, 360));
        }
        if self.twenty_retard_callout.audio() {
            warnings.push(WarningCode::new(34, 00, 350));
        }
        if self.retard_callout.audio() {
            warnings.push(WarningCode::new(34, 00, 370));
        }
        if self.hundred_above.audio() {
            warnings.push(WarningCode::new(22, 00, 060));
        }
        if self.minimum.audio() {
            warnings.push(WarningCode::new(22, 00, 070));
        }
        if self.altitude_callout_5_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 340));
        }
        if self.altitude_callout_10_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 330));
        }
        if self.altitude_callout_20_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 320));
        }
        if self.altitude_callout_30_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 310));
        }
        if self.altitude_callout_40_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 300));
        }
        if self.altitude_callout_50_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 290));
        }
        if self.altitude_callout_100_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 280));
        }
        if self.altitude_callout_200_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 270));
        }
        if self.altitude_callout_300_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 260));
        }
        if self.altitude_callout_400_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 255));
        }
        if self.altitude_callout_500_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 380));
        }
        if self.altitude_callout_1000_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 390));
        }
        if self.altitude_callout_2000_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 410));
        }
        if self.altitude_callout_2500_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 420));
        }
        if self.altitude_callout_2500b_ft.audio() {
            warnings.push(WarningCode::new(34, 00, 400));
        }

        if self.altitude_alert_c_chord.audio() {
            warnings.push(WarningCode::new(22, 00, 050))
        }

        self.monitor.update(
            delta,
            A320MonitorParameters {
                mw_cancel_pulse_up: self.general_cancel.mw_cancel_pulse_up(),
                mc_cancel_pulse_up: self.general_cancel.mc_cancel_pulse_up(),
                radio_height: self.altitude_callout_threshold1.radio_height(),
                retard_inhibition: false,
                auto_call_out_inhib: self.altitude_callout_inhibit.auto_call_out_inhib(),
                intermediate_call_out: self
                    .altitude_callout_intermediate_audio
                    .intermediate_call_out(),
            },
            warnings,
        );
    }

    /// This method queries the corresponding activations to find the first phase that is currently
    /// active. It defaults to phase 6 if no phase appears active due to an unusual set of
    /// parameters.
    pub fn flight_phase(&self) -> Option<u8> {
        if self.flight_phases_ground.phase_1() {
            Some(1)
        } else if self.flight_phases_ground.phase_2() {
            Some(2)
        } else if self.flight_phases_ground.phase_3() {
            Some(3)
        } else if self.flight_phases_ground.phase_4() {
            Some(4)
        } else if self.flight_phases_air.phase_5() {
            Some(5)
        } else if self.flight_phases_air.phase_6() {
            Some(6)
        } else if self.flight_phases_air.phase_7() {
            Some(7)
        } else if self.flight_phases_ground.phase_8() {
            Some(8)
        } else if self.flight_phases_ground.phase_9() {
            Some(9)
        } else if self.flight_phases_ground.phase_10() {
            Some(10)
        } else {
            None
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
        self.monitor.c_chord()
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

    pub fn synthetic_voice_index(&self) -> Option<u8> {
        self.monitor.synthetic_voice_index()
    }
}

#[cfg(test)]
mod tests {
    use crate::flight_warning::test::*;
    use std::time::Duration;
    use uom::si::f64::*;
    use uom::si::{length::foot, velocity::knot};

    use super::*;

    #[cfg(test)]
    mod flight_warning_computer_runtime_tests {
        use super::*;

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
                runtime.update(
                    Duration::from_secs(30),
                    &test_bed().on_ground().parameters(),
                );
                assert!(runtime.audio_attenuation());
            }

            #[test]
            fn when_both_engines_off_and_in_air_audio_is_not_attenuated() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(Duration::from_secs(30), &test_bed().parameters());
                assert!(!runtime.audio_attenuation());
            }

            #[test]
            fn when_both_engines_running_and_on_ground_audio_is_not_attenuated() {
                let mut runtime = A320FlightWarningComputerRuntime::new();
                runtime.update(
                    Duration::from_secs(30),
                    &test_bed().on_ground().with().engines_running().parameters(),
                );
                assert!(!runtime.audio_attenuation());
            }
        }
    }
}

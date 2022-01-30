use std::time::Duration;

use super::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::{SignStatusMatrix, Value};
use systems::flight_warning::utils::FwcSsm;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::{percent, ratio};
use uom::si::velocity::knot;

pub(in crate::flight_warning::runtime) trait GeneralCancel {
    fn mw_cancel_pulse_up(&self) -> bool;
    fn mc_cancel_pulse_up(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct GeneralCancelActivation {
    capt_mw_pulse: PulseNode,
    fo_mw_pulse: PulseNode,
    capt_mc_pulse: PulseNode,
    fo_mc_pulse: PulseNode,
    mw_cancel_pulse_up: bool,
    mc_cancel_pulse_up: bool,
}

impl Default for GeneralCancelActivation {
    fn default() -> Self {
        Self {
            capt_mw_pulse: PulseNode::new(true),
            fo_mw_pulse: PulseNode::new(true),
            capt_mc_pulse: PulseNode::new(true),
            fo_mc_pulse: PulseNode::new(true),
            mw_cancel_pulse_up: false,
            mc_cancel_pulse_up: false,
        }
    }
}

impl GeneralCancelActivation {
    pub fn update(
        &mut self,
        signals: &(impl CaptMwCancelOn + FoMwCancelOn + CaptMcCancelOn + FoMcCancelOn),
    ) {
        self.mw_cancel_pulse_up = self
            .capt_mw_pulse
            .update(signals.capt_mw_cancel_on().value())
            || self.fo_mw_pulse.update(signals.fo_mw_cancel_on().value());
        self.mc_cancel_pulse_up = self
            .capt_mc_pulse
            .update(signals.capt_mc_cancel_on().value())
            || self.fo_mc_pulse.update(signals.fo_mc_cancel_on().value());
    }
}

impl GeneralCancel for GeneralCancelActivation {
    fn mw_cancel_pulse_up(&self) -> bool {
        self.mw_cancel_pulse_up
    }

    fn mc_cancel_pulse_up(&self) -> bool {
        self.mc_cancel_pulse_up
    }
}

pub(in crate::flight_warning::runtime) trait AudioAttenuation {
    fn audio_attenuation(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AudioAttenuationActivation {
    audio_attenuation: bool,
}

impl AudioAttenuationActivation {
    pub fn update(
        &mut self,
        ground_sheet: &impl GroundDetection,
        engine_not_running_sheet: &impl EngineNotRunning,
    ) {
        self.audio_attenuation = ground_sheet.ground()
            && engine_not_running_sheet.eng_1_not_running()
            && engine_not_running_sheet.eng_2_not_running();
    }
}

impl AudioAttenuation for AudioAttenuationActivation {
    fn audio_attenuation(&self) -> bool {
        self.audio_attenuation
    }
}

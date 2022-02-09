use std::time::Duration;

use super::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::{SignStatusMatrix, Value};
use systems::flight_warning::utils::FwcSsm;
use uom::si::f64::*;
use uom::si::length::foot;

trait NavStallWarn {
    fn stall_warn(&self) -> bool;
}

struct NavStallWarnActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    conf3: ConfirmationNode,
    conf4: ConfirmationNode,
    conf1_pulse: PulseNode,
    conf2_pulse: PulseNode,
    conf3_pulse: PulseNode,
    conf4_pulse: PulseNode,
    prec1: PreceedingValueNode,
    prec2: PreceedingValueNode,
    prec3: PreceedingValueNode,
    prec4: PreceedingValueNode,
    stall_warn: bool,
}

impl Default for NavStallWarnActivation {
    fn default() -> Self {
        Self {
            conf1: ConfirmationNode::new(true, Duration::from_secs(3)),
            conf2: ConfirmationNode::new(true, Duration::from_secs(3)),
            conf3: ConfirmationNode::new(true, Duration::from_secs(3)),
            conf4: ConfirmationNode::new(true, Duration::from_secs(3)),
            conf1_pulse: PulseNode::new(true),
            conf2_pulse: PulseNode::new(true),
            conf3_pulse: PulseNode::new(true),
            conf4_pulse: PulseNode::new(true),
            prec1: Default::default(),
            prec2: Default::default(),
            prec3: Default::default(),
            prec4: Default::default(),
            stall_warn: false,
        }
    }
}

impl NavStallWarnActivation {
    pub fn update(&mut self, signals: &(impl EcpEmerCancelOn + EcpRecall)) {}
}

impl NavStallWarn for NavStallWarnActivation {
    fn stall_warn(&self) -> bool {
        self.stall_warn
    }
}

trait NavStallWarning {
    fn stall_warn_on(&self) -> bool;
    fn stall_on(&self) -> bool;

    // This signal is used to inhibit the GPWS.
    fn stall_steady_on(&self) -> bool;
}

struct NavStallWarningActivation {
    pulse1: PulseNode,
    pulse2: PulseNode,
    pulse3: PulseNode,
    pulse4: PulseNode,
    mem: MemoryNode,
    prec: PreceedingValueNode,
    stall_warn_on: bool,
    stall_on: bool,
    stall_steady_on: bool,
}

impl Default for NavStallWarningActivation {
    fn default() -> Self {
        Self {
            pulse1: PulseNode::new(false),
            pulse2: PulseNode::new(false),
            pulse3: PulseNode::new(false),
            pulse4: PulseNode::new(true),
            mem: MemoryNode::new(false),
            prec: Default::default(),
            stall_warn_on: false,
            stall_on: false,
            stall_steady_on: false,
        }
    }
}

impl NavStallWarningActivation {
    pub fn update(
        &mut self,
        signals: &(impl EcpEmerCancelOn + RadioHeight + PitchLawCode1),
        stall_warn: &impl NavStallWarn,
        flight_phases_gnd: &impl FlightPhasesGround,
        flight_phases_air: &impl FlightPhasesAir,
    ) {
        let rh1 = signals.radio_height(1);
        let rh2 = signals.radio_height(2);

        let phase_5 = flight_phases_air.phase_5();
        let phase_6 = flight_phases_air.phase_6();
        let phase_7 = flight_phases_air.phase_7();

        let end_of_phase_8 = self.pulse1.update(flight_phases_gnd.phase_8());
        let entered_phase_7 = end_of_phase_8 && flight_phases_air.phase_7();
        let end_of_phase_4 = self.pulse2.update(flight_phases_gnd.phase_4());
        let entered_phase_5 = end_of_phase_4 && phase_5;
        let end_of_phase_5 = self.pulse3.update(phase_5);

        let mem_out = self.mem.update(
            entered_phase_5 || entered_phase_7,
            end_of_phase_5 || phase_6,
        );

        let air_phase = phase_5 || phase_6 || phase_7;

        let is_normal_law =
            signals.pitch_law_code_1(1).value() || signals.pitch_law_code_1(2).value();

        let rh1_below_1500ft =
            rh1.value() <= Length::new::<foot>(1500.) && !rh1.is_ncd() && !rh1.is_inv();
        let rh2_below_1500ft =
            rh2.value() <= Length::new::<foot>(1500.) && !rh2.is_ncd() && !rh2.is_inv();

        let inhibit = is_normal_law && mem_out && rh1_below_1500ft && rh2_below_1500ft;

        let stall_warn_in = stall_warn.stall_warn();
        let cancel_pulse = self
            .pulse4
            .update(signals.ecp_emer_cancel_on().value() && self.prec.value());
        let stall_warn_or_ec = stall_warn_in || cancel_pulse;

        let stall_warn_on = air_phase && stall_warn_or_ec && !inhibit; // TODO alpha test
        self.prec.update(stall_warn_on);
        self.stall_warn_on = stall_warn_on;

        self.stall_on = self.stall_warn_on;
        self.stall_stead_on = self.stall_on;
    }
}

impl WarningActivation for NavStallWarningActivation {
    fn warning(&self) -> bool {
        self.stall_warn_on
    }
}

impl NavStallWarning for NavStallWarningActivation {
    fn stall_warn_on(&self) -> bool {
        self.stall_warn_on
    }

    fn stall_on(&self) -> bool {
        self.stall_on
    }

    fn stall_steady_on(&self) -> bool {
        self.stall_steady_on
    }
}

#[derive(Default)]
struct NavStallWarningVoiceActivation {
    stall_warn_on: bool,
}

impl NavStallWarningVoiceActivation {
    pub fn update(&mut self, stall_sheet: &impl NavStallWarning) {
        self.stall_warn_on = stall_sheet.stall_warn_on();
    }
}

impl WarningActivation for NavStallWarningVoiceActivation {
    fn warning(&self) -> bool {
        self.stall_warn_on
    }
}

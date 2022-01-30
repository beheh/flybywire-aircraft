use std::time::Duration;

use super::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::Value;

pub(in crate::flight_warning::runtime) trait Eng1StartSequence {
    fn eng_1_tempo_master_lever_1_on(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct Eng1StartSequenceActivation {
    conf: ConfirmationNode,
    eng_1_tempo_master_lever_1_on: bool,
}

impl Default for Eng1StartSequenceActivation {
    fn default() -> Self {
        Self {
            conf: ConfirmationNode::new(true, Duration::from_secs(30)),
            eng_1_tempo_master_lever_1_on: false,
        }
    }
}

impl Eng1StartSequenceActivation {
    pub fn update(&mut self, delta: Duration, signals: &impl Eng1MasterLeverSelectOn) {
        self.eng_1_tempo_master_lever_1_on = self
            .conf
            .update(signals.eng1_master_lever_select_on().value(), delta);
    }
}

impl Eng1StartSequence for Eng1StartSequenceActivation {
    fn eng_1_tempo_master_lever_1_on(&self) -> bool {
        self.eng_1_tempo_master_lever_1_on
    }
}

pub(in crate::flight_warning::runtime) trait Eng2StartSequence {
    fn eng_2_tempo_master_lever_1_on(&self) -> bool;
    fn phase_5_to_30s(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct Eng2StartSequenceActivation {
    conf: ConfirmationNode,
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    eng_2_tempo_master_lever_1_on: bool,
    phase_5_to_30s: bool,
}

impl Default for Eng2StartSequenceActivation {
    fn default() -> Self {
        Self {
            conf: ConfirmationNode::new(true, Duration::from_secs(30)),
            pulse: PulseNode::new(false),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(30)),
            eng_2_tempo_master_lever_1_on: false,
            phase_5_to_30s: false,
        }
    }
}

impl Eng2StartSequenceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &impl Eng2MasterLeverSelectOn,
        flight_phases_ground_sheet: &impl FlightPhasesGround,
        flight_phases_air_sheet: &impl FlightPhasesAir,
    ) {
        self.eng_2_tempo_master_lever_1_on = self
            .conf
            .update(signals.eng2_master_lever_select_on().value(), delta);
        let phase4 = flight_phases_ground_sheet.phase_4();
        let phase5 = flight_phases_air_sheet.phase_5();
        self.phase_5_to_30s = self
            .mtrig
            .update(self.pulse.update(phase4) && phase5, delta);
    }
}

impl Eng2StartSequence for Eng2StartSequenceActivation {
    fn eng_2_tempo_master_lever_1_on(&self) -> bool {
        self.eng_2_tempo_master_lever_1_on
    }

    fn phase_5_to_30s(&self) -> bool {
        self.phase_5_to_30s
    }
}

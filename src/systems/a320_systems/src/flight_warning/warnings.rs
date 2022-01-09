use std::time::Duration;

use crate::flight_warning::parameters::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::{SignStatusMatrix, Value};
use systems::flight_warning::utils::FwcSsm;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::percent;
use uom::si::velocity::knot;

pub(super) trait NewGround {
    fn new_ground(&self) -> bool;
    fn lgciu_12_inv(&self) -> bool;
}

pub(super) struct NewGroundActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    conf3: ConfirmationNode,
    conf4: ConfirmationNode,
    memory1: MemoryNode,
    memory2: MemoryNode,
    new_ground: bool,
    lgciu_12_inv: bool,
}

impl NewGroundActivation {
    pub fn new() -> Self {
        NewGroundActivation {
            conf1: ConfirmationNode::new_leading(Duration::from_secs_f64(1.0)),
            conf2: ConfirmationNode::new_leading(Duration::from_secs_f64(0.5)),
            conf3: ConfirmationNode::new_leading(Duration::from_secs_f64(1.0)),
            conf4: ConfirmationNode::new_leading(Duration::from_secs_f64(0.5)),
            memory1: MemoryNode::new(true),
            memory2: MemoryNode::new(true),
            new_ground: false,
            lgciu_12_inv: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl LhLgCompressed + EssLhLgCompressed + NormLhLgCompressed),
    ) {
        let xor1 = signals.lh_lg_compressed(1).value() ^ signals.ess_lh_lg_compressed().value();
        let set_memory1 = signals.lh_lg_compressed(1).is_ncd()
            || signals.lh_lg_compressed(1).is_inv()
            || self.conf1.update(xor1, delta);

        let memory1_out = self
            .memory1
            .update(set_memory1, self.conf2.update(!xor1, delta));

        let xor3 = signals.lh_lg_compressed(2).value() ^ signals.norm_lh_lg_compressed().value();
        let set_memory2 = signals.lh_lg_compressed(2).is_ncd()
            || signals.lh_lg_compressed(2).is_inv()
            || self.conf3.update(xor3, delta);
        let memory2_out = self
            .memory2
            .update(set_memory2, self.conf4.update(!xor3, delta));

        let op1 = signals.lh_lg_compressed(1).value() && signals.ess_lh_lg_compressed().value();
        let op2 = signals.lh_lg_compressed(2).value() && signals.norm_lh_lg_compressed().value();

        self.new_ground = op1 && op2;
        self.lgciu_12_inv = memory1_out || memory2_out;
    }
}

impl NewGround for NewGroundActivation {
    fn new_ground(&self) -> bool {
        self.new_ground
    }
    fn lgciu_12_inv(&self) -> bool {
        self.lgciu_12_inv
    }
}

pub(super) trait GroundDetection {
    fn ground(&self) -> bool;
    fn ground_immediate(&self) -> bool;
}

pub(super) struct GroundDetectionActivation {
    memory1: MemoryNode,
    memory2: MemoryNode,
    conf1: ConfirmationNode,
    mrtrig1: MonostableTriggerNode,
    ground_immediate: bool,
    ground: bool,
}

impl GroundDetectionActivation {
    pub fn new() -> Self {
        Self {
            memory1: MemoryNode::new(true),
            memory2: MemoryNode::new(true),
            conf1: ConfirmationNode::new(true, Duration::from_secs(1)),
            mrtrig1: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(10)),
            ground_immediate: false,
            ground: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl EssLhLgCompressed + NormLhLgCompressed + RadioHeight),
        lgciu_ground: &impl NewGround,
    ) {
        let radio_1_ncd = signals.radio_height(1).is_ncd();
        let radio_1_inv = signals.radio_height(1).is_inv();
        let radio_2_ncd = signals.radio_height(2).is_ncd();
        let radio_2_inv = signals.radio_height(2).is_inv();

        let reset_memory =
            !signals.ess_lh_lg_compressed().value() || !signals.norm_lh_lg_compressed().value();
        let set_memory_1 = signals.radio_height(1).value() < Length::new::<foot>(5.0);
        let memory1_out = self.memory1.update(set_memory_1, reset_memory);

        let set_memory_2 = signals.radio_height(2).value() < Length::new::<foot>(5.0);
        let memory2_out = self.memory2.update(set_memory_2, reset_memory);

        let radio_1_on_gnd = (memory1_out || set_memory_1) && !radio_1_ncd && !radio_1_inv;
        let radio_2_on_gnd = (memory2_out || set_memory_2) && !radio_2_ncd && !radio_2_inv;

        let ground_signals = [
            signals.ess_lh_lg_compressed().value(),
            signals.norm_lh_lg_compressed().value(),
            radio_1_on_gnd,
            radio_2_on_gnd,
        ];
        let ground_count = ground_signals.iter().filter(|&n| *n).count();
        let more_than_2 = ground_count > 2;
        let more_than_1 = ground_count > 1;

        let dual_radio_inv = radio_1_inv && radio_2_inv;
        let gnd_cond1 = more_than_2 && !dual_radio_inv;
        let gnd_cond2 = more_than_1 && dual_radio_inv;

        let mrtrig_in = radio_1_ncd && radio_2_ncd && !lgciu_ground.lgciu_12_inv();
        let trig_ground = self.mrtrig1.update(mrtrig_in, delta) && lgciu_ground.new_ground();

        self.ground_immediate = (gnd_cond1 || gnd_cond2) || trig_ground;
        self.ground = self.conf1.update(self.ground_immediate, delta);
    }
}

impl GroundDetection for GroundDetectionActivation {
    fn ground(&self) -> bool {
        self.ground
    }
    fn ground_immediate(&self) -> bool {
        self.ground_immediate
    }
}

pub(super) trait SpeedDetection {
    fn ac_speed_above_80_kt(&self) -> bool;

    fn adc_test_inhib(&self) -> bool;
}

pub(super) struct SpeedDetectionActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    conf3: ConfirmationNode,
    memory: MemoryNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    ac_speed_above_80_kt: bool,
    adc_test_inhib: bool,
}

impl SpeedDetectionActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new(true, Duration::from_secs(1)),
            conf2: ConfirmationNode::new(true, Duration::from_secs(1)),
            conf3: ConfirmationNode::new(true, Duration::from_secs(1)),
            memory: MemoryNode::new(true),
            mtrig1: MonostableTriggerNode::new_falling(Duration::from_secs_f64(0.5)),
            mtrig2: MonostableTriggerNode::new_falling(Duration::from_secs_f64(1.5)),
            ac_speed_above_80_kt: false,
            adc_test_inhib: false,
        }
    }

    pub fn update(&mut self, delta: Duration, signals: &impl ComputedSpeed) {
        let adc_1_invalid =
            signals.computed_speed(1).is_inv() || signals.computed_speed(1).is_ncd();
        let adc_2_invalid =
            signals.computed_speed(2).is_inv() || signals.computed_speed(2).is_ncd();
        let adc_3_invalid =
            signals.computed_speed(3).is_inv() || signals.computed_speed(3).is_ncd();
        let any_adc_invalid = adc_1_invalid || adc_2_invalid || adc_3_invalid;

        let conf1_out = self.conf1.update(
            signals.computed_speed(1).value() > Velocity::new::<knot>(50.0) && !adc_1_invalid,
            delta,
        );
        let conf2_out = self.conf2.update(
            signals.computed_speed(2).value() > Velocity::new::<knot>(50.0) && !adc_2_invalid,
            delta,
        );
        let conf3_out = self.conf3.update(
            signals.computed_speed(3).value() > Velocity::new::<knot>(50.0) && !adc_3_invalid,
            delta,
        );

        let adc_1_above_80_kt = conf1_out
            && !adc_1_invalid
            && signals.computed_speed(1).value() > Velocity::new::<knot>(83.0);
        let adc_2_above_80_kt = conf2_out
            && !adc_2_invalid
            && signals.computed_speed(2).value() > Velocity::new::<knot>(83.0);
        let adc_3_above_80_kt = conf3_out
            && !adc_3_invalid
            && signals.computed_speed(3).value() > Velocity::new::<knot>(83.0);
        let any_adc_above_80_kt = adc_1_above_80_kt || adc_2_above_80_kt || adc_3_above_80_kt;

        let set_signals = &[
            adc_1_above_80_kt,
            adc_2_above_80_kt,
            adc_3_above_80_kt,
            any_adc_above_80_kt && any_adc_invalid,
        ];
        let set_memory = set_signals.iter().filter(|&n| *n).count() > 1;

        let adc_1_below_77_kt =
            signals.computed_speed(1).value() < Velocity::new::<knot>(77.0) && !adc_1_invalid;
        let adc_2_below_77_kt =
            signals.computed_speed(2).value() < Velocity::new::<knot>(77.0) && !adc_2_invalid;
        let adc_3_below_77_kt =
            signals.computed_speed(3).value() < Velocity::new::<knot>(77.0) && !adc_3_invalid;
        let any_adc_below_77_kt = adc_1_below_77_kt || adc_2_below_77_kt || adc_3_below_77_kt;

        let any_adc_fault = signals.computed_speed(1).is_ft()
            || signals.computed_speed(2).is_ft()
            || signals.computed_speed(3).is_ft();

        let reset_signals = &[
            adc_1_below_77_kt,
            adc_2_below_77_kt,
            adc_3_below_77_kt,
            any_adc_below_77_kt && any_adc_invalid,
        ];
        let reset_memory = reset_signals.iter().filter(|&n| *n).count() > 1
            || self.mtrig1.update(any_adc_fault, delta);

        self.ac_speed_above_80_kt = self.memory.update(set_memory, reset_memory);
        self.adc_test_inhib = self.mtrig2.update(any_adc_fault, delta);
    }
}

impl SpeedDetection for SpeedDetectionActivation {
    fn ac_speed_above_80_kt(&self) -> bool {
        self.ac_speed_above_80_kt
    }
    fn adc_test_inhib(&self) -> bool {
        self.adc_test_inhib
    }
}

pub(super) trait EngineNotRunning {
    fn eng_1_not_running(&self) -> bool;
    fn eng_2_not_running(&self) -> bool;
}

pub(super) struct EnginesNotRunning {
    trans1: TransientDetectionNode,
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    conf3: ConfirmationNode,
    conf4: ConfirmationNode,
    conf5: ConfirmationNode,
    eng_1_not_running: bool,
    eng_2_not_running: bool,
}

impl EnginesNotRunning {
    pub fn new() -> Self {
        Self {
            trans1: TransientDetectionNode::new(true),
            conf1: ConfirmationNode::new_leading(Duration::from_secs(30)),
            conf2: ConfirmationNode::new_leading(Duration::from_secs(30)),
            conf3: ConfirmationNode::new_leading(Duration::from_secs(30)),
            conf4: ConfirmationNode::new_leading(Duration::from_secs(30)),
            conf5: ConfirmationNode::new_falling(Duration::from_secs(31)),
            eng_1_not_running: false,
            eng_2_not_running: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1MasterLeverSelectOn
              + Eng1CoreSpeedAtOrAboveIdle
              + Eng1FirePbOut
              + Eng2CoreSpeedAtOrAboveIdle
              + Eng2MasterLeverSelectOn),
        ground: &impl GroundDetection,
    ) {
        let eng1_core_speed_at_or_above_idle_a = signals.eng1_core_speed_at_or_above_idle(1);
        let eng1_core_speed_at_or_above_idle_b = signals.eng1_core_speed_at_or_above_idle(2);

        let conf5_out = self.trans1.update(signals.eng_1_fire_pb_out().value());

        let conf1_out = self
            .conf1
            .update(eng1_core_speed_at_or_above_idle_a.value(), delta);

        let conf2_out = self
            .conf2
            .update(eng1_core_speed_at_or_above_idle_b.value(), delta);

        let eng_1_core_speed_not_running_conf = !conf1_out && !conf2_out;
        let eng_1_core_speed_running_immediate = eng1_core_speed_at_or_above_idle_a.value()
            && eng1_core_speed_at_or_above_idle_b.value()
            && conf5_out
            && !ground.ground();

        let eng_1_core_speed_not_running =
            eng_1_core_speed_not_running_conf && !eng_1_core_speed_running_immediate;

        self.eng_1_not_running = (signals.eng1_master_lever_select_on().is_val()
            && !signals.eng1_master_lever_select_on().value())
            || eng_1_core_speed_not_running;

        let eng2_core_speed_at_or_above_idle_a = signals.eng2_core_speed_at_or_above_idle(1);
        let eng2_core_speed_at_or_above_idle_b = signals.eng2_core_speed_at_or_above_idle(2);

        let conf3_out = self
            .conf3
            .update(eng2_core_speed_at_or_above_idle_a.value(), delta);

        let conf4_out = self
            .conf4
            .update(eng2_core_speed_at_or_above_idle_b.value(), delta);

        let eng_2_core_speed_running_immediate = !ground.ground()
            && conf5_out
            && eng1_core_speed_at_or_above_idle_b.value()
            && eng1_core_speed_at_or_above_idle_a.value();

        let eng_2_core_speed_not_running_conf = !conf3_out && !conf4_out;

        let eng_2_core_speed_not_running =
            !eng_2_core_speed_running_immediate && eng_2_core_speed_not_running_conf;

        self.eng_2_not_running = eng_2_core_speed_not_running
            || (!signals.eng1_master_lever_select_on().value()
                && signals.eng2_master_lever_select_on().is_val());
    }
}

impl EngineNotRunning for EnginesNotRunning {
    fn eng_1_not_running(&self) -> bool {
        self.eng_1_not_running
    }

    fn eng_2_not_running(&self) -> bool {
        self.eng_2_not_running
    }
}

pub(super) trait EngRunning {
    fn eng_1_and_2_not_running(&self) -> bool;
    fn eng_1_or_2_running(&self) -> bool;
    fn one_eng_running(&self) -> bool;
}

pub(super) struct EngRunningActivation {
    conf1: ConfirmationNode,
    eng_1_and_2_not_running: bool,
    eng_1_or_2_running: bool,
    one_eng_running: bool,
}

impl EngRunningActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new(true, Duration::from_secs(30)),
            eng_1_and_2_not_running: false,
            eng_1_or_2_running: false,
            one_eng_running: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1CoreSpeedAtOrAboveIdle + Eng2CoreSpeedAtOrAboveIdle),
        engine_not_running: &impl EngineNotRunning,
    ) {
        self.eng_1_and_2_not_running =
            engine_not_running.eng_1_not_running() && engine_not_running.eng_2_not_running();
        let one_eng_running = signals.eng1_core_speed_at_or_above_idle(1).value()
            || signals.eng1_core_speed_at_or_above_idle(2).value()
            || signals.eng2_core_speed_at_or_above_idle(1).value()
            || signals.eng2_core_speed_at_or_above_idle(2).value();

        self.one_eng_running = one_eng_running;
        self.eng_1_or_2_running = self.conf1.update(one_eng_running, delta);
    }
}

impl EngRunning for EngRunningActivation {
    fn eng_1_and_2_not_running(&self) -> bool {
        self.eng_1_and_2_not_running
    }

    fn eng_1_or_2_running(&self) -> bool {
        self.eng_1_or_2_running
    }

    fn one_eng_running(&self) -> bool {
        self.one_eng_running
    }
}

pub(super) trait EngTakeOffCfm {
    fn eng1_to_cfm(&self) -> bool;
    fn eng2_to_cfm(&self) -> bool;
    fn tla1_idle_pwr_cfm(&self) -> bool;
    fn tla2_idle_pwr_cfm(&self) -> bool;
}

pub(super) struct EngTakeOffCfmActivation {
    eng1_to_cfm: bool,
    eng2_to_cfm: bool,
    tla1_idle_pwr_cfm: bool,
    tla2_idle_pwr_cfm: bool,
}

impl EngTakeOffCfmActivation {
    pub fn new() -> Self {
        Self {
            eng1_to_cfm: false,
            eng2_to_cfm: false,
            tla1_idle_pwr_cfm: false,
            tla2_idle_pwr_cfm: false,
        }
    }

    pub fn update(
        &mut self,
        signals: &(impl Eng1N1SelectedActual
              + Eng2N1SelectedActual
              + Tla1IdlePwr
              + Tla2IdlePwr
              + Eng1ChannelInControl
              + Eng2ChannelInControl),
    ) {
        let is_cfm = true;

        let eng1_n1_selected_actual_a = signals.eng1_n1_selected_actual(1);
        let eng1_n1_selected_actual_b = signals.eng1_n1_selected_actual(2);

        let eng1_n1_abv_95_a = !(eng1_n1_selected_actual_a.is_inv()
            || eng1_n1_selected_actual_a.is_ncd())
            && eng1_n1_selected_actual_a.value() > Ratio::new::<percent>(95.0);
        let eng1_n1_abv_95_b = !(eng1_n1_selected_actual_b.is_inv()
            || eng1_n1_selected_actual_b.is_ncd())
            && eng1_n1_selected_actual_b.value() > Ratio::new::<percent>(95.0);

        self.eng1_to_cfm = is_cfm && (eng1_n1_abv_95_a || eng1_n1_abv_95_b);

        let eng2_n1_selected_actual_a = signals.eng2_n1_selected_actual(1);
        let eng2_n1_selected_actual_b = signals.eng2_n1_selected_actual(2);

        let eng2_n1_abv_95_a = !(eng2_n1_selected_actual_a.is_inv()
            || eng2_n1_selected_actual_a.is_ncd())
            && eng2_n1_selected_actual_a.value() > Ratio::new::<percent>(95.0);
        let eng2_n1_abv_95_b = !(eng2_n1_selected_actual_b.is_inv()
            || eng2_n1_selected_actual_b.is_ncd())
            && eng2_n1_selected_actual_b.value() > Ratio::new::<percent>(95.0);

        self.eng2_to_cfm = is_cfm && (eng2_n1_abv_95_a || eng2_n1_abv_95_b);

        let eng1_channel_a_in_control = signals.eng1_channel_a_in_control().value();
        let eng1_channel_b_in_control = signals.eng1_channel_b_in_control().value();

        self.tla1_idle_pwr_cfm = is_cfm
            && ((eng1_channel_a_in_control && signals.tla1_idle_pwr(1).value())
                || (eng1_channel_b_in_control && signals.tla1_idle_pwr(2).value()));

        let eng2_channel_a_in_control = signals.eng2_channel_a_in_control().value();
        let eng2_channel_b_in_control = signals.eng2_channel_b_in_control().value();

        self.tla2_idle_pwr_cfm = is_cfm
            && ((eng2_channel_a_in_control && signals.tla2_idle_pwr(1).value())
                || (eng2_channel_b_in_control && signals.tla2_idle_pwr(2).value()));
    }
}

impl EngTakeOffCfm for EngTakeOffCfmActivation {
    fn eng1_to_cfm(&self) -> bool {
        self.eng1_to_cfm
    }

    fn eng2_to_cfm(&self) -> bool {
        self.eng2_to_cfm
    }

    fn tla1_idle_pwr_cfm(&self) -> bool {
        self.tla1_idle_pwr_cfm
    }

    fn tla2_idle_pwr_cfm(&self) -> bool {
        self.tla2_idle_pwr_cfm
    }
}

pub(super) trait NeoEcu {
    fn eng_1_auto_toga(&self) -> bool;
    fn eng_1_limit_mode_soft_ga(&self) -> bool;
    fn eng_2_auto_toga(&self) -> bool;
    fn eng_2_limit_mode_soft_ga(&self) -> bool;
}

pub(super) struct NeoEcuActivation {
    eng_1_auto_toga: bool,
    eng_1_limit_mode_soft_ga: bool,
    eng_2_auto_toga: bool,
    eng_2_limit_mode_soft_ga: bool,
}

impl NeoEcuActivation {
    pub fn new() -> Self {
        Self {
            eng_1_auto_toga: false,
            eng_1_limit_mode_soft_ga: false,
            eng_2_auto_toga: false,
            eng_2_limit_mode_soft_ga: false,
        }
    }

    pub fn update(
        &mut self,
        signals: &(impl Eng1AutoToga + Eng1LimitModeSoftGa + Eng2AutoToga + Eng2LimitModeSoftGa),
    ) {
        self.eng_1_auto_toga =
            signals.eng_1_auto_toga(1).value() || signals.eng_1_auto_toga(2).value();
        self.eng_1_limit_mode_soft_ga = signals.eng_1_limit_mode_soft_ga(1).value()
            || signals.eng_1_limit_mode_soft_ga(2).value();
        self.eng_2_auto_toga =
            signals.eng_2_auto_toga(1).value() || signals.eng_2_auto_toga(2).value();
        self.eng_2_limit_mode_soft_ga = signals.eng_2_limit_mode_soft_ga(1).value()
            || signals.eng_2_limit_mode_soft_ga(2).value();
    }
}

impl NeoEcu for NeoEcuActivation {
    fn eng_1_auto_toga(&self) -> bool {
        self.eng_1_auto_toga
    }

    fn eng_1_limit_mode_soft_ga(&self) -> bool {
        self.eng_1_limit_mode_soft_ga
    }

    fn eng_2_auto_toga(&self) -> bool {
        self.eng_2_auto_toga
    }

    fn eng_2_limit_mode_soft_ga(&self) -> bool {
        self.eng_2_limit_mode_soft_ga
    }
}

pub(super) trait TlaAtMctOrFlexToCfm {
    fn eng_1_tla_mct_cfm(&self) -> bool;
    fn eng_1_end_mct(&self) -> bool;
    fn eng_1_sup_mct_cfm(&self) -> bool;
    fn eng_2_tla_mct_cfm(&self) -> bool;
    fn eng_2_end_mct(&self) -> bool;
    fn eng_2_sup_mct_cfm(&self) -> bool;
}

pub(super) struct TlaAtMctOrFlexToCfmActivation {
    eng_1_tla_mct_cfm: bool,
    eng_1_end_mct: bool,
    eng_1_sup_mct_cfm: bool,
    eng_2_tla_mct_cfm: bool,
    eng_2_end_mct: bool,
    eng_2_sup_mct_cfm: bool,
}

impl TlaAtMctOrFlexToCfmActivation {
    pub fn new() -> Self {
        Self {
            eng_1_tla_mct_cfm: false,
            eng_1_end_mct: false,
            eng_1_sup_mct_cfm: false,
            eng_2_tla_mct_cfm: false,
            eng_2_end_mct: false,
            eng_2_sup_mct_cfm: false,
        }
    }

    pub fn update(&mut self, signals: &(impl Eng1Tla + Eng2Tla)) {
        let any_cfm = true;

        // Engine 1

        let eng1_a_lt_36 = signals.eng1_tla(1).value() < Angle::new::<degree>(36.7);
        let eng1_a_val = signals.eng1_tla(1).is_val();
        let eng1_a_gt_33 = signals.eng1_tla(1).value() > Angle::new::<degree>(33.3);
        let eng1_a_tla_mct_cfm = eng1_a_lt_36 && eng1_a_val && eng1_a_gt_33;

        let eng1_b_lt_36 = signals.eng1_tla(2).value() < Angle::new::<degree>(36.7);
        let eng1_b_val = signals.eng1_tla(2).is_val();
        let eng1_b_gt_33 = signals.eng1_tla(2).value() > Angle::new::<degree>(33.3);
        let eng1_b_tla_mct_cfm = eng1_b_lt_36 && eng1_b_val && eng1_b_gt_33;

        self.eng_1_tla_mct_cfm = any_cfm && (eng1_a_tla_mct_cfm || eng1_b_tla_mct_cfm);

        let eng1_a_gt_36 = signals.eng1_tla(1).value() > Angle::new::<degree>(36.6);
        let eng1_a_end_mct = eng1_a_lt_36 && eng1_a_val && eng1_a_gt_36;

        let eng1_b_gt_36 = signals.eng1_tla(2).value() > Angle::new::<degree>(36.6);
        let eng1_b_end_mct = eng1_b_lt_36 && eng1_b_val && eng1_b_gt_36;

        self.eng_1_end_mct = eng1_a_end_mct || eng1_b_end_mct;

        let eng1_a_sup_mct_cfm = !eng1_a_lt_36 && eng1_a_val;
        let eng1_b_sup_mct_cfm = !eng1_b_lt_36 && eng1_b_val;

        self.eng_1_sup_mct_cfm = any_cfm && (eng1_a_sup_mct_cfm || eng1_b_sup_mct_cfm);

        // Engine 2

        let eng2_a_lt_36 = signals.eng2_tla(1).value() < Angle::new::<degree>(36.7);
        let eng2_a_val = signals.eng2_tla(1).is_val();
        let eng2_a_gt_33 = signals.eng2_tla(1).value() > Angle::new::<degree>(33.3);
        let eng2_a_tla_mct_cfm = eng2_a_lt_36 && eng2_a_val && eng2_a_gt_33;

        let eng2_b_lt_36 = signals.eng2_tla(2).value() < Angle::new::<degree>(36.7);
        let eng2_b_val = signals.eng2_tla(2).is_val();
        let eng2_b_gt_33 = signals.eng2_tla(2).value() > Angle::new::<degree>(33.3);
        let eng2_b_tla_mct_cfm = eng2_b_lt_36 && eng2_b_val && eng2_b_gt_33;

        self.eng_2_tla_mct_cfm = any_cfm && (eng2_a_tla_mct_cfm || eng2_b_tla_mct_cfm);

        let eng2_a_gt_36 = signals.eng2_tla(1).value() > Angle::new::<degree>(36.6);
        let eng2_a_end_mct = eng2_a_lt_36 && eng2_a_val && eng2_a_gt_36;

        let eng2_b_gt_36 = signals.eng2_tla(2).value() > Angle::new::<degree>(36.6);
        let eng2_b_end_mct = eng2_b_lt_36 && eng2_b_val && eng2_b_gt_36;

        self.eng_2_end_mct = eng2_a_end_mct || eng2_b_end_mct;

        let eng2_a_sup_mct_cfm = !eng2_a_lt_36 && eng2_a_val;
        let eng2_b_sup_mct_cfm = !eng2_b_lt_36 && eng2_b_val;

        self.eng_2_sup_mct_cfm = any_cfm && (eng2_a_sup_mct_cfm || eng2_b_sup_mct_cfm);
    }
}

impl TlaAtMctOrFlexToCfm for TlaAtMctOrFlexToCfmActivation {
    fn eng_1_tla_mct_cfm(&self) -> bool {
        self.eng_1_tla_mct_cfm
    }

    fn eng_1_end_mct(&self) -> bool {
        self.eng_1_end_mct
    }

    fn eng_1_sup_mct_cfm(&self) -> bool {
        self.eng_1_sup_mct_cfm
    }

    fn eng_2_tla_mct_cfm(&self) -> bool {
        self.eng_2_tla_mct_cfm
    }

    fn eng_2_end_mct(&self) -> bool {
        self.eng_2_end_mct
    }

    fn eng_2_sup_mct_cfm(&self) -> bool {
        self.eng_2_sup_mct_cfm
    }
}

pub(super) trait TlaPwrReverse {
    fn eng_1_tla_full_pwr_cfm(&self) -> bool;
    fn eng_1_tla_reverse_cfm(&self) -> bool;
    fn eng_2_tla_full_pwr_cfm(&self) -> bool;
    fn eng_2_tla_reverse_cfm(&self) -> bool;
}

pub(super) struct TlaPwrReverseActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    eng_1_tla_full_pwr_cfm: bool,
    eng_1_tla_reverse_cfm: bool,
    eng_2_tla_full_pwr_cfm: bool,
    eng_2_tla_reverse_cfm: bool,
}

impl TlaPwrReverseActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new_falling(Duration::from_secs(10)),
            conf2: ConfirmationNode::new_falling(Duration::from_secs(10)),
            eng_1_tla_full_pwr_cfm: false,
            eng_1_tla_reverse_cfm: false,
            eng_2_tla_full_pwr_cfm: false,
            eng_2_tla_reverse_cfm: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1Tla + Eng2Tla),
        eng_take_off_cfm: &impl EngTakeOffCfm,
    ) {
        let any_cfm = true;

        // Engine 1

        let eng1_a_lt_m4 = signals.eng1_tla(1).value() < Angle::new::<degree>(-4.3);
        let eng1_b_lt_m4 = signals.eng1_tla(2).value() < Angle::new::<degree>(-4.3);

        let eng1_a_inv = signals.eng1_tla(1).is_inv() || signals.eng1_tla(1).is_ncd();
        let eng1_b_inv = signals.eng1_tla(2).is_inv() || signals.eng1_tla(2).is_ncd();

        let eng1_a_tla_reverse = eng1_a_lt_m4 && !eng1_a_inv;
        let eng1_b_tla_reverse = eng1_b_lt_m4 && !eng1_b_inv;
        let eng1_tla_reverse = eng1_a_tla_reverse || eng1_b_tla_reverse;
        self.eng_1_tla_reverse_cfm = any_cfm && eng1_tla_reverse;

        let eng1_a_gt_43 = signals.eng1_tla(1).value() > Angle::new::<degree>(43.3);
        let eng1_b_gt_43 = signals.eng1_tla(2).value() > Angle::new::<degree>(43.3);

        let conf1_in = eng1_tla_reverse;
        let conf1_out = self.conf1.update(conf1_in, delta);
        let eng_1_tla_full_pwr_cond = conf1_in || conf1_out;

        let eng1_to_conf = eng_take_off_cfm.eng1_to_cfm() && !eng_1_tla_full_pwr_cond;

        self.eng_1_tla_full_pwr_cfm = any_cfm && (eng1_a_gt_43 || eng1_to_conf || eng1_b_gt_43);

        // Engine 2

        let eng2_a_lt_m4 = signals.eng2_tla(1).value() < Angle::new::<degree>(-4.3);
        let eng2_b_lt_m4 = signals.eng2_tla(2).value() < Angle::new::<degree>(-4.3);

        let eng2_a_inv = signals.eng2_tla(1).is_inv() || signals.eng2_tla(1).is_ncd();
        let eng2_b_inv = signals.eng2_tla(2).is_inv() || signals.eng2_tla(2).is_ncd();

        let eng2_a_tla_reverse = eng2_a_lt_m4 && !eng2_a_inv;
        let eng2_b_tla_reverse = eng2_b_lt_m4 && !eng2_b_inv;
        let eng2_tla_reverse = eng2_a_tla_reverse || eng2_b_tla_reverse;
        self.eng_2_tla_reverse_cfm = any_cfm && eng2_tla_reverse;

        let eng2_a_gt_43 = signals.eng2_tla(1).value() > Angle::new::<degree>(43.3);
        let eng2_b_gt_43 = signals.eng2_tla(2).value() > Angle::new::<degree>(43.3);

        let conf2_in = eng2_tla_reverse;
        let conf2_out = self.conf2.update(conf1_in, delta);
        let eng_2_tla_full_pwr_cond = conf2_in || conf2_out;

        let eng2_to_conf = eng_take_off_cfm.eng2_to_cfm() && !eng_2_tla_full_pwr_cond;

        self.eng_2_tla_full_pwr_cfm = any_cfm && (eng2_a_gt_43 || eng2_to_conf || eng2_b_gt_43);
    }
}

impl TlaPwrReverse for TlaPwrReverseActivation {
    fn eng_1_tla_full_pwr_cfm(&self) -> bool {
        self.eng_1_tla_full_pwr_cfm
    }

    fn eng_1_tla_reverse_cfm(&self) -> bool {
        self.eng_1_tla_reverse_cfm
    }

    fn eng_2_tla_full_pwr_cfm(&self) -> bool {
        self.eng_2_tla_full_pwr_cfm
    }

    fn eng_2_tla_reverse_cfm(&self) -> bool {
        self.eng_2_tla_reverse_cfm
    }
}

pub(super) trait TlaAtClCfm {
    fn eng_1_tla_cl_cfm(&self) -> bool;
    fn eng_12_mcl_cfm(&self) -> bool;
    fn eng_2_tla_cl_cfm(&self) -> bool;
}

pub(super) struct TlaAtClCfmActivation {
    eng_1_tla_cl_cfm: bool,
    eng_12_mcl_cfm: bool,
    eng_2_tla_cl_cfm: bool,
}

impl TlaAtClCfmActivation {
    pub fn new() -> Self {
        Self {
            eng_1_tla_cl_cfm: false,
            eng_12_mcl_cfm: false,
            eng_2_tla_cl_cfm: false,
        }
    }

    pub fn update(&mut self, _delta: Duration, signals: &(impl Eng1Tla + Eng2Tla)) {
        let any_cfm = true;

        // Engine 1

        let eng1_a_lt_27 = signals.eng1_tla(1).value() < Angle::new::<degree>(27.1);
        let eng1_a_gt_22 = signals.eng1_tla(1).value() > Angle::new::<degree>(22.9);
        let eng1_a_val = signals.eng1_tla(1).is_val();
        let eng1_a_tla_cl = eng1_a_lt_27 && eng1_a_gt_22 && eng1_a_val;

        let eng1_b_lt_27 = signals.eng1_tla(2).value() < Angle::new::<degree>(27.1);
        let eng1_b_gt_22 = signals.eng1_tla(2).value() > Angle::new::<degree>(22.9);
        let eng1_b_val = signals.eng1_tla(2).is_val();
        let eng1_b_tla_cl = eng1_b_lt_27 && eng1_b_gt_22 && eng1_b_val;

        self.eng_1_tla_cl_cfm = any_cfm && (eng1_a_tla_cl || eng1_b_tla_cl);

        // Engine 2

        let eng2_a_lt_27 = signals.eng2_tla(1).value() < Angle::new::<degree>(27.1);
        let eng2_a_gt_22 = signals.eng2_tla(1).value() > Angle::new::<degree>(22.9);
        let eng2_a_val = signals.eng2_tla(1).is_val();
        let eng2_a_tla_cl = eng2_a_lt_27 && eng2_a_gt_22 && eng2_a_val;

        let eng2_b_lt_27 = signals.eng2_tla(2).value() < Angle::new::<degree>(27.1);
        let eng2_b_gt_22 = signals.eng2_tla(2).value() > Angle::new::<degree>(22.9);
        let eng2_b_val = signals.eng2_tla(2).is_val();
        let eng2_b_tla_cl = eng2_b_lt_27 && eng2_b_gt_22 && eng2_b_val;

        self.eng_2_tla_cl_cfm = any_cfm && (eng2_a_tla_cl || eng2_b_tla_cl);

        // Engine 1 + 2

        let eng1_a_mcl = eng1_a_val && eng1_a_gt_22;
        let eng1_b_mcl = eng1_b_val && eng1_b_gt_22;
        let eng1_mcl = eng1_a_mcl || eng1_b_mcl;

        let eng2_a_mcl = eng2_a_val && eng2_a_gt_22;
        let eng2_b_mcl = eng2_b_val && eng2_b_gt_22;
        let eng2_mcl = eng2_a_mcl || eng2_b_mcl;

        self.eng_12_mcl_cfm = any_cfm && eng1_mcl && eng2_mcl;
    }
}

impl TlaAtClCfm for TlaAtClCfmActivation {
    fn eng_1_tla_cl_cfm(&self) -> bool {
        self.eng_1_tla_cl_cfm
    }

    fn eng_12_mcl_cfm(&self) -> bool {
        self.eng_12_mcl_cfm
    }

    fn eng_2_tla_cl_cfm(&self) -> bool {
        self.eng_2_tla_cl_cfm
    }
}

pub(super) trait CfmFlightPhasesDef {
    fn cfm_flex(&self) -> bool;
    fn eng_1_or_2_to_pwr(&self) -> bool;
}

pub(super) struct CfmFlightPhasesDefActivation {
    conf1: ConfirmationNode,
    cfm_flex: bool,
    eng_1_or_2_to_pwr: bool,
}

impl CfmFlightPhasesDefActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new_falling(Duration::from_secs(60)),
            cfm_flex: false,
            eng_1_or_2_to_pwr: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1TlaFto + Eng2TlaFto),
        neo_def: &impl NeoEcu,
        tla_mct_or_flex_to: &impl TlaAtMctOrFlexToCfm,
        tla_pwr_reverse: &impl TlaPwrReverse,
        altitude_def: &impl AltitudeDef,
        tla_at_cl_cfm: &impl TlaAtClCfm,
    ) {
        let any_cfm = true;

        // CFM Flex

        let eng1_flex = tla_mct_or_flex_to.eng_1_tla_mct_cfm()
            && (neo_def.eng_1_auto_toga()
                || neo_def.eng_1_limit_mode_soft_ga()
                || signals.eng1_tla_fto(1).value()
                || signals.eng1_tla_fto(2).value());

        let eng2_flex = tla_mct_or_flex_to.eng_2_tla_mct_cfm()
            && (neo_def.eng_2_auto_toga()
                || neo_def.eng_2_limit_mode_soft_ga()
                || signals.eng2_tla_fto(1).value()
                || signals.eng2_tla_fto(2).value());

        self.cfm_flex = any_cfm && (eng1_flex || eng2_flex);

        // Engine 1 or 2 Takeoff Power

        let eng1_or_2_to_pwr_cond1 = eng1_flex
            || eng2_flex
            || tla_mct_or_flex_to.eng_1_sup_mct_cfm()
            || tla_mct_or_flex_to.eng_2_sup_mct_cfm()
            || tla_pwr_reverse.eng_1_tla_full_pwr_cfm()
            || tla_pwr_reverse.eng_2_tla_full_pwr_cfm();

        let conf1_in = eng1_or_2_to_pwr_cond1;
        let conf1_out = self.conf1.update(conf1_in, delta);

        let eng1_or_2_to_pwr_cond2 =
            conf1_out && !altitude_def.h_gt_1500ft() && tla_at_cl_cfm.eng_12_mcl_cfm();

        let eng1_or_2_to_pwr_cond = eng1_or_2_to_pwr_cond1 || eng1_or_2_to_pwr_cond2;

        self.eng_1_or_2_to_pwr = any_cfm && (eng1_or_2_to_pwr_cond);
    }
}

impl CfmFlightPhasesDef for CfmFlightPhasesDefActivation {
    fn cfm_flex(&self) -> bool {
        self.cfm_flex
    }

    fn eng_1_or_2_to_pwr(&self) -> bool {
        self.eng_1_or_2_to_pwr
    }
}

pub(super) trait AltitudeDef {
    fn h_gt_800ft(&self) -> bool;
    fn h_gt_1500ft(&self) -> bool;
    fn h_fail(&self) -> bool;
}

pub(super) struct AltitudeDefActivation {
    conf1: ConfirmationNode,
    memory1: MemoryNode,
    h_fail: bool,
    h_gt_800ft: bool,
    h_gt_1500ft: bool,
}

impl AltitudeDefActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new_leading(Duration::from_secs(4)),
            memory1: MemoryNode::new(false),
            h_fail: false,
            h_gt_800ft: false,
            h_gt_1500ft: false,
        }
    }

    pub fn update(&mut self, delta: Duration, signals: &impl RadioHeight) {
        let radio1_inv = signals.radio_height(1).is_inv();
        let radio2_inv = signals.radio_height(2).is_inv();
        let radio1_ncd = signals.radio_height(1).is_ncd();
        let radio2_ncd = signals.radio_height(2).is_ncd();

        let dual_radio_inv = radio1_inv && radio2_inv;
        self.h_fail = dual_radio_inv;

        let conf1_conf =
            !dual_radio_inv && (radio1_inv || radio1_ncd) && (radio2_inv || radio2_ncd);
        let conf1_out = self.conf1.update(conf1_conf, delta);

        let radio1_abv_1500ft =
            signals.radio_height(1).value() > Length::new::<foot>(1500.0) && !radio1_inv;
        let radio2_abv_1500ft =
            signals.radio_height(2).value() > Length::new::<foot>(1500.0) && !radio2_inv;

        let h_gt_1500ft = radio1_abv_1500ft || radio2_abv_1500ft || conf1_out;
        self.h_gt_1500ft = h_gt_1500ft;

        let radio1_blw_800ft = signals.radio_height(1).value() < Length::new::<foot>(800.0)
            && !radio1_inv
            && !radio1_ncd;
        let radio2_blw_800ft = signals.radio_height(2).value() < Length::new::<foot>(800.0)
            && !radio2_inv
            && !radio2_ncd;

        let memory1_set = h_gt_1500ft;
        let memory1_reset = (radio1_blw_800ft || radio2_blw_800ft) && !conf1_out;

        self.h_gt_800ft = self.memory1.update(memory1_set, memory1_reset);
    }
}

impl AltitudeDef for AltitudeDefActivation {
    fn h_gt_800ft(&self) -> bool {
        self.h_gt_800ft
    }

    fn h_gt_1500ft(&self) -> bool {
        self.h_gt_1500ft
    }

    fn h_fail(&self) -> bool {
        self.h_fail
    }
}

pub(super) trait FlightPhasesGround {
    fn phase_1(&self) -> bool;
    fn phase_2(&self) -> bool;
    fn phase_3(&self) -> bool;
    fn phase_4(&self) -> bool;
    fn phase_8(&self) -> bool;
    fn phase_9(&self) -> bool;
    fn phase_10(&self) -> bool;
}

pub(super) struct FlightPhasesGroundActivation {
    trans1: TransientDetectionNode,
    conf1: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mtrig3: MonostableTriggerNode,
    mtrig4: MonostableTriggerNode,
    mtrig5: MonostableTriggerNode,
    mtrig6: MonostableTriggerNode,
    mem_phase10: MemoryNode,
    mem_phase9: MemoryNode,
    prec_phase9: PreceedingValueNode,
    phase1: bool,
    phase2: bool,
    phase3: bool,
    phase4: bool,
    phase8: bool,
    phase9: bool,
    phase10: bool,
}

impl FlightPhasesGroundActivation {
    pub fn new() -> Self {
        Self {
            trans1: TransientDetectionNode::new(false),
            conf1: ConfirmationNode::new_leading(Duration::from_secs_f64(0.2)),
            mtrig1: MonostableTriggerNode::new_falling(Duration::from_secs_f64(1.0)),
            mtrig2: MonostableTriggerNode::new_falling(Duration::from_secs_f64(3.0)),
            mtrig3: MonostableTriggerNode::new_leading(Duration::from_secs_f64(300.0)),
            mtrig4: MonostableTriggerNode::new_leading(Duration::from_secs_f64(2.0)),
            mtrig5: MonostableTriggerNode::new_leading(Duration::from_secs_f64(2.0)),
            mtrig6: MonostableTriggerNode::new_leading(Duration::from_secs_f64(2.0)),
            mem_phase9: MemoryNode::new_nvm(true),
            mem_phase10: MemoryNode::new(false),
            prec_phase9: PreceedingValueNode::new(),
            phase1: false,
            phase2: false,
            phase3: false,
            phase4: false,
            phase8: false,
            phase9: false,
            phase10: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1FirePbOut + ToConfigTest),
        ground_sheet: &impl GroundDetection,
        ac_speed_sheet: &impl SpeedDetection,
        eng_running_sheet: &impl EngRunning,
        takeoff_power_sheet: &impl CfmFlightPhasesDef,
    ) {
        let ground = ground_sheet.ground();
        let ground_immediate = ground_sheet.ground_immediate();
        let ac_speed_above_80_kt = ac_speed_sheet.ac_speed_above_80_kt();
        let eng1_or_2_running = eng_running_sheet.eng_1_or_2_running();
        let eng1_or_2_to_pwr = takeoff_power_sheet.eng_1_or_2_to_pwr();

        // phase 1 and 10 preamble
        let trans1 = self.trans1.update(signals.eng_1_fire_pb_out().value());
        let conf1 = self.conf1.update(trans1, delta);
        let mtrig5 = self.mtrig5.update(conf1, delta);
        let reset_mem10 = ground && mtrig5;

        // phases 3 and 4

        let ground_and_to_pwr = ground && eng1_or_2_to_pwr;

        let phase3 = !ac_speed_above_80_kt && eng1_or_2_running && ground_and_to_pwr;
        self.phase3 = phase3;
        self.phase4 = ac_speed_above_80_kt && ground_and_to_pwr;

        // phase 8

        let phase8_cond1 = ground_immediate || self.mtrig6.update(ground_immediate, delta);

        let phase8 = phase8_cond1 & !eng1_or_2_to_pwr && ac_speed_above_80_kt;
        self.phase8 = phase8;

        // phases 2 and 9

        let prec_phase9 = self.prec_phase9.value();
        let mtrig1 = self.mtrig1.update(eng1_or_2_to_pwr, delta);
        let mtrig2 = self.mtrig2.update(prec_phase9, delta);
        let mtrig4 = self.mtrig4.update(!ac_speed_above_80_kt, delta);
        let phase29_cond = ground && !eng1_or_2_to_pwr && !ac_speed_above_80_kt;
        let one_eng_running = eng_running_sheet.one_eng_running();

        let reset_nvm_cond1 = ground && mtrig2;
        let reset_nvm_cond2 = reset_mem10;
        let reset_nvm_cond3 = ground && mtrig1;
        let reset_nvm = reset_nvm_cond1 || reset_nvm_cond2 || reset_nvm_cond3;

        let inhibited_reset_nvm = !mtrig4 && reset_nvm && !prec_phase9;

        let adc_test_inhib = ac_speed_sheet.adc_test_inhib();
        let to_config_test = signals.to_config_test().value();
        let to_config_reset_9 = to_config_test && phase29_cond && one_eng_running;
        let reset_mem9 = inhibited_reset_nvm || adc_test_inhib || to_config_reset_9;

        let phase9_mem = self.mem_phase9.update(phase3 || phase8, reset_mem9);

        self.phase2 = phase29_cond && !phase9_mem && eng1_or_2_running;

        let phase9 = one_eng_running && phase9_mem && phase29_cond;
        self.phase9 = phase9;
        self.prec_phase9.update(phase9);

        // phases 1 and 10

        let set_mem10 = phase9;
        let mem_phase10_out = self.mem_phase10.update(set_mem10, reset_mem10);

        let phase110_cond =
            !set_mem10 && eng_running_sheet.eng_1_and_2_not_running() && ground_immediate;
        let mtrig3 = self.mtrig3.update(mem_phase10_out && phase110_cond, delta);

        self.phase1 = phase110_cond && !mtrig3;
        self.phase10 = phase110_cond && mtrig3;
    }
}

impl FlightPhasesGround for FlightPhasesGroundActivation {
    fn phase_1(&self) -> bool {
        self.phase1
    }

    fn phase_2(&self) -> bool {
        self.phase2
    }

    fn phase_3(&self) -> bool {
        self.phase3
    }

    fn phase_4(&self) -> bool {
        self.phase4
    }

    fn phase_8(&self) -> bool {
        self.phase8
    }

    fn phase_9(&self) -> bool {
        self.phase9
    }

    fn phase_10(&self) -> bool {
        self.phase10
    }
}

pub(super) trait FlightPhasesAir {
    fn phase_5(&self) -> bool;
    fn phase_6(&self) -> bool;
    fn phase_7(&self) -> bool;
}

pub(super) struct FlightPhasesAirActivation {
    conf1: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mtrig3: MonostableTriggerNode,
    trans1: TransientDetectionNode,
    pulse1: PulseNode,
    phase5: bool,
    phase6: bool,
    phase7: bool,
}

impl FlightPhasesAirActivation {
    pub fn new() -> Self {
        Self {
            conf1: ConfirmationNode::new_leading(Duration::from_secs_f64(0.2)),
            mtrig1: MonostableTriggerNode::new_leading(Duration::from_secs(120)),
            mtrig2: MonostableTriggerNode::new_leading(Duration::from_secs(180)),
            mtrig3: MonostableTriggerNode::new_leading(Duration::from_secs(2)),
            trans1: TransientDetectionNode::new(false),
            pulse1: PulseNode::new_leading(),
            phase5: false,
            phase6: false,
            phase7: false,
        }
    }

    pub fn update(
        &mut self,
        delta: Duration,
        ground_sheet: &impl GroundDetection,
        altitude_sheet: &impl AltitudeDef,
        cfm_flight_phases_sheet: &impl CfmFlightPhasesDef,
        flight_phases_gnd_sheet: &impl FlightPhasesGround,
    ) {
        let mtrig3_in = ground_sheet.ground_immediate();
        let ground_immediate = self.mtrig3.update(mtrig3_in, delta) || mtrig3_in;

        let eng_1_or_2_to_pwr = cfm_flight_phases_sheet.eng_1_or_2_to_pwr();
        let h_fail = altitude_sheet.h_fail();
        let h_gt_1500ft = altitude_sheet.h_gt_1500ft();
        let h_gt_800ft = altitude_sheet.h_gt_800ft();

        let conf1_cond = self.trans1.update(h_gt_800ft);
        let pulse_cond = self.conf1.update(conf1_cond, delta);
        let h_gt_800ft_pulse = self.pulse1.update(pulse_cond);

        let mtrig1_in = !h_gt_1500ft && eng_1_or_2_to_pwr && !h_fail && !ground_immediate;
        let phase5_cond = self.mtrig1.update(mtrig1_in, delta) && mtrig1_in;

        let mtrig2_in = !ground_immediate
            && !h_fail
            && !eng_1_or_2_to_pwr
            && !h_gt_1500ft
            && !h_gt_800ft
            && !h_gt_800ft_pulse;

        let phase7_cond = self.mtrig2.update(mtrig2_in, delta) && mtrig2_in;

        self.phase5 = phase5_cond;
        self.phase6 = !phase5_cond && !ground_immediate && !phase7_cond;
        self.phase7 = phase7_cond && !flight_phases_gnd_sheet.phase_8();
    }
}

impl FlightPhasesAir for FlightPhasesAirActivation {
    fn phase_5(&self) -> bool {
        self.phase5
    }

    fn phase_6(&self) -> bool {
        self.phase6
    }

    fn phase_7(&self) -> bool {
        self.phase7
    }
}

pub(super) trait AudioAttenuation {
    fn audio_attenuation(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AudioAttenuationActivation {
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

pub(super) trait ToMemo {
    fn to_memo_computed(&self) -> bool;
}

pub(super) struct ToMemoActivation {
    conf: ConfirmationNode,
    mem: MemoryNode,
    to_memo_computed: bool,
}

impl Default for ToMemoActivation {
    fn default() -> Self {
        Self {
            conf: ConfirmationNode::new(true, Duration::from_secs(120)),
            mem: MemoryNode::new(false),
            to_memo_computed: false,
        }
    }
}

impl ToMemoActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &impl ToConfigTest,
        engine_not_running_sheet: &impl EngineNotRunning,
        flight_phases_gnd_sheet: &impl FlightPhasesGround,
        flight_phases_air_sheet: &impl FlightPhasesAir,
    ) {
        let phase2 = flight_phases_gnd_sheet.phase_2();
        let set_mem =
            (phase2 || flight_phases_gnd_sheet.phase_9()) && signals.to_config_test().value();
        let reset_mem = flight_phases_gnd_sheet.phase_1()
            || flight_phases_gnd_sheet.phase_3()
            || flight_phases_air_sheet.phase_6()
            || flight_phases_gnd_sheet.phase_10();
        let mem_out = self.mem.update(set_mem, reset_mem);

        let both_eng_running = !engine_not_running_sheet.eng_1_not_running()
            && !engine_not_running_sheet.eng_2_not_running();
        let conf_out = self.conf.update(both_eng_running, delta);

        self.to_memo_computed = mem_out || (phase2 && conf_out);
    }
}

impl ToMemo for ToMemoActivation {
    fn to_memo_computed(&self) -> bool {
        self.to_memo_computed
    }
}

pub(super) trait LgDownlocked {
    fn main_lg_downlocked(&self) -> bool;
    fn lg_downlocked(&self) -> bool;
}

#[derive(Default)]
pub(super) struct LgDownlockedActivation {
    main_lg_downlocked: bool,
    lg_downlocked: bool,
}

impl LgDownlockedActivation {
    pub fn update(&mut self, signals: &(impl LhGearDownLock + RhGearDownLock + NoseGearDownLock)) {
        let lh_gear_down_lock_1 = signals.lh_gear_down_lock(1).value();
        let lh_gear_down_lock_2 = signals.lh_gear_down_lock(2).value();
        let lh_gear_invalid =
            signals.lh_gear_down_lock(1).is_inv() || signals.lh_gear_down_lock(2).is_inv();
        let lh_gear_norm_down_lock = lh_gear_down_lock_1 && lh_gear_down_lock_2;
        let lh_gear_abnorm_down_lock =
            lh_gear_invalid && (lh_gear_down_lock_1 || lh_gear_down_lock_2);
        let lh_gear_downlocked = lh_gear_norm_down_lock || lh_gear_abnorm_down_lock;

        let rh_gear_down_lock_1 = signals.rh_gear_down_lock(1).value();
        let rh_gear_down_lock_2 = signals.rh_gear_down_lock(2).value();
        let rh_gear_invalid =
            signals.rh_gear_down_lock(1).is_inv() || signals.rh_gear_down_lock(2).is_inv();
        let rh_gear_norm_down_lock = rh_gear_down_lock_1 && rh_gear_down_lock_2;
        let rh_gear_abnorm_down_lock =
            rh_gear_invalid && (rh_gear_down_lock_1 || rh_gear_down_lock_2);
        let rh_gear_downlocked = rh_gear_norm_down_lock || rh_gear_abnorm_down_lock;

        self.main_lg_downlocked = lh_gear_downlocked && rh_gear_downlocked;

        let nose_gear_down_lock_1 = signals.nose_gear_down_lock(1).value();
        let nose_gear_down_lock_2 = signals.nose_gear_down_lock(2).value();
        let nose_gear_invalid =
            signals.nose_gear_down_lock(1).is_inv() || signals.nose_gear_down_lock(2).is_inv();
        let nose_gear_norm_down_lock = nose_gear_down_lock_1 && nose_gear_down_lock_2;
        let nose_gear_abnorm_down_lock =
            nose_gear_invalid && (nose_gear_down_lock_1 || nose_gear_down_lock_2);
        let nose_gear_downlocked = nose_gear_norm_down_lock || nose_gear_abnorm_down_lock;

        self.lg_downlocked = self.main_lg_downlocked && nose_gear_downlocked;
    }
}

impl LgDownlocked for LgDownlockedActivation {
    fn main_lg_downlocked(&self) -> bool {
        self.main_lg_downlocked
    }

    fn lg_downlocked(&self) -> bool {
        self.lg_downlocked
    }
}

pub(super) trait LdgMemo {
    fn ldg_memo(&self) -> bool;
    fn config_memo_computed(&self) -> bool;
}

pub(super) struct LdgMemoActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    mem_abv_2200: MemoryNode,
    mem_blw_2000: MemoryNode,
    ldg_memo: bool,
    config_memo_computed: bool,
}

impl Default for LdgMemoActivation {
    fn default() -> Self {
        Self {
            conf1: ConfirmationNode::new_leading(Duration::from_secs(1)),
            conf2: ConfirmationNode::new_leading(Duration::from_secs(10)),
            mem_blw_2000: MemoryNode::new(true),
            mem_abv_2200: MemoryNode::new(false),
            ldg_memo: false,
            config_memo_computed: false,
        }
    }
}

impl LdgMemoActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &impl RadioHeight,
        flight_phases_gnd_sheet: &impl FlightPhasesGround,
        flight_phases_air_sheet: &impl FlightPhasesAir,
        lg_downlocked_sheet: &impl LgDownlocked,
        to_memo_sheet: &impl ToMemo,
    ) {
        let rh1 = signals.radio_height(1);
        let rh1_inv = rh1.is_inv();
        let rh1_inv_or_ncd = rh1_inv || rh1.is_ncd();
        let rh1_abv_2200 = rh1.value() > Length::new::<foot>(2200.0);
        let rh1_blw_2000 = rh1.value() < Length::new::<foot>(2000.0);

        let rh2 = signals.radio_height(2);
        let rh2_inv = rh2.is_inv();
        let rh2_inv_or_ncd = rh2_inv || rh2.is_ncd();
        let rh2_abv_2200 = rh2.value() > Length::new::<foot>(2200.0);
        let rh2_blw_2000 = rh2.value() < Length::new::<foot>(2000.0);

        let dual_ra_inv_or_ncd = rh1_inv_or_ncd && rh2_inv_or_ncd;
        let dual_ra_abv_2200_or_inv_or_ncd =
            (rh1_abv_2200 || rh1_inv_or_ncd) && (rh2_abv_2200 || rh2_inv_or_ncd);
        let any_ra_below_2000 =
            (!rh1_inv_or_ncd && rh1_blw_2000) || (!rh2_inv_or_ncd && rh2_blw_2000);

        let phase6 = flight_phases_air_sheet.phase_6();
        let phase7 = flight_phases_air_sheet.phase_7();
        let phase8 = flight_phases_gnd_sheet.phase_8();

        let set_mem_abv_2200 = self
            .conf1
            .update(!dual_ra_inv_or_ncd && dual_ra_abv_2200_or_inv_or_ncd, delta);
        let abv_2200 = self
            .mem_abv_2200
            .update(set_mem_abv_2200, !(phase6 || phase7 || phase8));

        let blw_2000 = self
            .mem_blw_2000
            .update(any_ra_below_2000, dual_ra_abv_2200_or_inv_or_ncd);

        let lg_down_flight = false;
        let dual_ra_inv_lg_downlocked = self.conf2.update(
            rh1_inv && rh2_inv && lg_downlocked_sheet.lg_downlocked() && !lg_down_flight && phase6,
            delta,
        );

        self.ldg_memo =
            (abv_2200 && blw_2000 && phase6) || phase7 || phase8 || dual_ra_inv_lg_downlocked;

        self.config_memo_computed = to_memo_sheet.to_memo_computed() || self.ldg_memo;
    }
}

impl LdgMemo for LdgMemoActivation {
    fn ldg_memo(&self) -> bool {
        self.ldg_memo
    }

    fn config_memo_computed(&self) -> bool {
        self.config_memo_computed
    }
}

#[derive(Default)]
pub(super) struct AltitudeActivation {
    altitude: Length,
}

impl AltitudeActivation {
    pub fn update(&mut self, signals: &(impl Altitude)) {
        let alti1 = signals.altitude(1);
        let alti2 = signals.altitude(2);
        let alti3 = signals.altitude(3);

        self.altitude = if !alti1.is_inv() {
            alti1.value()
        } else if !alti2.is_inv() {
            alti2.value()
        } else {
            alti3.value()
        };
    }
}

pub(super) trait AutoFlightAutopilotOffVoluntary {
    /// This signal indicates that AP1 is fully engaged (command & monitor channel).
    fn ap1_engd(&self) -> bool;

    /// This signal indicates that AP2 is fully engaged (command & monitor channel).
    fn ap2_engd(&self) -> bool;

    /// This signal indicates that any one of the APs is fully engaged (command & monitor).
    fn one_ap_engd(&self) -> bool;

    /// This signal indicates that the Cavalry Charge should be playing due to one of the
    /// instinctive P/Bs being pressed.
    fn ap_off_audio(&self) -> bool;

    /// This signal indicates that the MW should start flashing due to one of the instinctive P/Bs
    /// being pressed.
    fn ap_off_mw(&self) -> bool;

    /// This signal indicates that the special line "AP OFF" in the EWD should be shown due to one
    /// of the instinctive P/Bs being pressed.
    fn ap_off_text(&self) -> bool;
}

pub(super) struct AutoFlightAutopilotOffVoluntaryActivation {
    pulse1: PulseNode,
    pulse2: PulseNode,
    pulse3: PulseNode,
    instinc_discnct_pulse: PulseNode,
    conf1: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mtrig3: MonostableTriggerNode,
    mtrig4: MonostableTriggerNode,
    mtrig5: MonostableTriggerNode,
    mtrig6: MonostableTriggerNode,
    mtrig7: MonostableTriggerNode,
    mtrig8: MonostableTriggerNode,
    mtrig9: MonostableTriggerNode,
    mtrig10: MonostableTriggerNode,
    ap_off_audio_mem: MemoryNode,
    ap1_engd: bool,
    ap2_engd: bool,
    one_ap_engd: bool,
    ap_off_audio: bool,
    ap_off_mw: bool,
    ap_off_text: bool,
}

impl Default for AutoFlightAutopilotOffVoluntaryActivation {
    fn default() -> Self {
        Self {
            pulse1: PulseNode::new(false),
            pulse2: PulseNode::new(false),
            pulse3: PulseNode::new(false),
            instinc_discnct_pulse: PulseNode::new(true),
            conf1: ConfirmationNode::new_leading(Duration::from_secs_f64(0.2)),
            mtrig1: MonostableTriggerNode::new_leading(Duration::from_secs_f64(1.3)),
            mtrig2: MonostableTriggerNode::new_leading(Duration::from_secs_f64(1.3)),
            mtrig3: MonostableTriggerNode::new_leading(Duration::from_secs_f64(5.0)),
            mtrig4: MonostableTriggerNode::new_leading(Duration::from_secs_f64(1.5)),
            mtrig5: MonostableTriggerNode::new_leading(Duration::from_secs_f64(3.0)),
            mtrig6: MonostableTriggerNode::new_leading(Duration::from_secs_f64(3.0)),
            mtrig7: MonostableTriggerNode::new_leading(Duration::from_secs_f64(9.0)),
            mtrig8: MonostableTriggerNode::new_leading(Duration::from_secs_f64(9.0)),
            mtrig9: MonostableTriggerNode::new_leading(Duration::from_secs_f64(0.5)),
            mtrig10: MonostableTriggerNode::new_falling(Duration::from_secs_f64(1.5)),
            ap_off_audio_mem: MemoryNode::new(true),
            ap1_engd: false,
            ap2_engd: false,
            one_ap_engd: false,
            ap_off_audio: false,
            ap_off_mw: false,
            ap_off_text: false,
        }
    }
}

impl AutoFlightAutopilotOffVoluntaryActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        cavalry_charge_emitted: bool,
        signals: &(impl Ap1Engd
              + Ap2Engd
              + CaptMwCancelOn
              + FoMwCancelOn
              + InstincDiscnct1ApEngd
              + InstincDiscnct2ApEngd),
    ) {
        self.ap1_engd = signals.ap1_engd_com().value() && signals.ap1_engd_mon().value();
        self.ap2_engd = signals.ap2_engd_com().value() && signals.ap2_engd_mon().value();
        let one_ap_engd = self.ap1_engd || self.ap2_engd;
        self.one_ap_engd = one_ap_engd;

        let allow_cancel = self.conf1.update(!one_ap_engd, delta);

        let instinc_discnct_1ap_engd = signals.instinc_discnct_1ap_engd().value();
        let instinc_discnct_2ap_engd = signals.instinc_discnct_2ap_engd().value();
        let instinc_discnct_pulse_out = self
            .instinc_discnct_pulse
            .update(instinc_discnct_1ap_engd || instinc_discnct_2ap_engd);

        let red_warning = false; // TODO certain red warnings should have priority over AP cancel
        let mw_cancel = (signals.capt_mw_cancel_on().value() || signals.fo_mw_cancel_on().value())
            && !red_warning;
        let do_cancel = allow_cancel && (mw_cancel || instinc_discnct_pulse_out);

        let any_instinc_discnct_mtrig = self.mtrig1.update(instinc_discnct_1ap_engd, delta)
            || self.mtrig2.update(instinc_discnct_2ap_engd, delta);
        let ap_disengage_pulse = self.pulse1.update(self.ap1_engd || self.ap2_engd);
        let instinctive_disconnect = ap_disengage_pulse && any_instinc_discnct_mtrig;

        let reset_ap_off_audio_cond_1 = self.pulse2.update(
            self.mtrig3
                .update(ap_disengage_pulse && any_instinc_discnct_mtrig, delta),
        );
        let reset_ap_off_audio_cond_2 = self
            .pulse3
            .update(self.mtrig4.update(cavalry_charge_emitted, delta));

        // these signals are true if the warning should be active in the first place
        let ap_off_audio_mem_out = self.ap_off_audio_mem.update(
            instinctive_disconnect,
            reset_ap_off_audio_cond_1 || reset_ap_off_audio_cond_2,
        );
        let ap_off_mw_mtrig_out = self.mtrig5.update(instinctive_disconnect, delta);
        let ap_of_text_mtrig_out = self.mtrig7.update(instinctive_disconnect, delta);

        // these signals are true if the warning should be cancelled
        let cancel_ap_off_audio = self
            .mtrig10
            .update(self.mtrig9.update(do_cancel, delta), delta);
        let cancel_ap_off_mw = self.mtrig6.update(do_cancel, delta);
        let cancel_ap_off_text = self.mtrig8.update(do_cancel, delta);

        self.ap_off_audio = ap_off_audio_mem_out && !one_ap_engd && !cancel_ap_off_audio;
        self.ap_off_mw = ap_off_mw_mtrig_out && !one_ap_engd && !cancel_ap_off_mw;
        self.ap_off_text = ap_of_text_mtrig_out && !one_ap_engd && !cancel_ap_off_text;
    }
}

impl AutoFlightAutopilotOffVoluntary for AutoFlightAutopilotOffVoluntaryActivation {
    fn ap1_engd(&self) -> bool {
        self.ap1_engd
    }

    fn ap2_engd(&self) -> bool {
        self.ap2_engd
    }

    fn one_ap_engd(&self) -> bool {
        self.one_ap_engd
    }

    fn ap_off_audio(&self) -> bool {
        self.ap_off_audio
    }

    fn ap_off_mw(&self) -> bool {
        self.ap_off_mw
    }

    fn ap_off_text(&self) -> bool {
        self.ap_off_text
    }
}

pub(super) trait AutoFlightAutopilotOffUnvoluntary {
    fn ap_off_warning(&self) -> bool;
    /// This signal indicates that an involuntary AP disocnnect was detected and and that the MW
    /// should start flashing.
    fn ap_unvol_off(&self) -> bool;

    /// This signal indicates that the AP condition for MW no longer exists because either an AP was
    /// reengaged or Phase 1 was reached, and that the MW related to the AP disconnect should stop
    /// flashing.
    fn ap_off_reset(&self) -> bool;

    /// This signal indicates that a MW cancel button has been pressed and that the MW related to
    /// the AP disconnect should stop flashing.
    fn ap_mw(&self) -> bool;
}

pub(super) struct AutoFlightAutopilotOffUnvoluntaryActivation {
    pulse_ap_disengage: PulseNode,
    pulse_ap_unvol_off: PulseNode,
    pulse_instinc_discnct: PulseNode,
    pulse_phase1: PulseNode,
    pulse_ap_engage: PulseNode,
    pulse_mw_cancel: PulseNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mtrig3: MonostableTriggerNode,
    mem_ap_unvol_off: MemoryNode,
    mem_warning: MemoryNode,
    ap_off_warning: bool,
    ap_unvol_off: bool,
    ap_off_reset: bool,
    ap_mw: bool,
}

impl Default for AutoFlightAutopilotOffUnvoluntaryActivation {
    fn default() -> Self {
        Self {
            pulse_ap_disengage: PulseNode::new(false),
            pulse_ap_unvol_off: PulseNode::new(true),
            pulse_instinc_discnct: PulseNode::new(true),
            pulse_phase1: PulseNode::new(true),
            pulse_ap_engage: PulseNode::new(true),
            pulse_mw_cancel: PulseNode::new(true),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.3)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.3)),
            mtrig3: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.5)),
            mem_ap_unvol_off: MemoryNode::new(false),
            mem_warning: MemoryNode::new(false),
            ap_off_warning: false,
            ap_unvol_off: false,
            ap_off_reset: false,
            ap_mw: false,
        }
    }
}

impl AutoFlightAutopilotOffUnvoluntaryActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        voluntary_sheet: &impl AutoFlightAutopilotOffVoluntary,
        flight_phases_ground: &impl FlightPhasesGround,
        cavalry_charge_emitted: bool,
        signals: &(impl Ap1Engd
              + Ap2Engd
              + CaptMwCancelOn
              + FoMwCancelOn
              + InstincDiscnct1ApEngd
              + InstincDiscnct2ApEngd),
    ) {
        let phase1 = flight_phases_ground.phase_1();
        let blue_sys_lo_pr = false; // TODO tripple low pressure should inhibit flashing MW in phase 1
        let yellow_sys_lo_pr = false;
        let green_sys_lo_pr = false;
        let inhibited_on_ground = phase1 && blue_sys_lo_pr && yellow_sys_lo_pr && green_sys_lo_pr;
        let instinc_discnct_1ap = signals.instinc_discnct_1ap_engd().value();
        let instinc_discnct_2ap = signals.instinc_discnct_2ap_engd().value();
        let mtrig1_out = self.mtrig1.update(instinc_discnct_1ap, delta);
        let mtrig2_out = self.mtrig2.update(instinc_discnct_2ap, delta);
        let recent_voluntary_disconnect = mtrig1_out || mtrig2_out;

        let ap1_engaged = signals.ap1_engd_com().value() && signals.ap1_engd_mon().value();
        let ap2_engaged = signals.ap2_engd_com().value() && signals.ap2_engd_mon().value();
        let any_ap_engaged = ap1_engaged || ap2_engaged;
        let ap_disenage_pulse = self.pulse_ap_disengage.update(any_ap_engaged);
        let phase1_pulse = self.pulse_phase1.update(phase1);
        let ap_engage_pulse = self.pulse_ap_engage.update(any_ap_engaged);

        let reset_ap_warnings = ap_engage_pulse || phase1_pulse;

        let ap_unvol_off = self.mem_ap_unvol_off.update(
            !inhibited_on_ground && !recent_voluntary_disconnect && ap_disenage_pulse,
            reset_ap_warnings,
        );
        let ap_unvol_off_pulse = self.pulse_ap_unvol_off.update(self.ap_unvol_off);
        let ap_recently_unvol_off = self.mtrig3.update(ap_unvol_off_pulse, delta);

        let any_mw_cancel = (signals.capt_mw_cancel_on().value()
            || signals.fo_mw_cancel_on().value())
            && cavalry_charge_emitted;
        let ap_mw = self
            .pulse_mw_cancel
            .update(!any_ap_engaged && any_mw_cancel);

        let any_instinc_discnct = instinc_discnct_1ap || instinc_discnct_2ap;
        let any_instinc_discnct_pulse =
            self.pulse_instinc_discnct.update(any_instinc_discnct) && !any_ap_engaged;

        let ap_off_reset =
            reset_ap_warnings || (!ap_recently_unvol_off && (any_instinc_discnct_pulse || ap_mw));

        let warning = self.mem_warning.update(ap_unvol_off_pulse, ap_off_reset); // TODO use

        self.ap_off_warning = voluntary_sheet.ap_off_text() || ap_unvol_off;
        self.ap_unvol_off = ap_unvol_off;
        self.ap_off_reset = ap_off_reset;
        self.ap_mw = ap_mw;
    }
}

impl AutoFlightAutopilotOffUnvoluntary for AutoFlightAutopilotOffUnvoluntaryActivation {
    fn ap_off_warning(&self) -> bool {
        self.ap_off_warning
    }

    fn ap_unvol_off(&self) -> bool {
        self.ap_unvol_off
    }

    fn ap_off_reset(&self) -> bool {
        self.ap_off_reset
    }

    fn ap_mw(&self) -> bool {
        self.ap_mw
    }
}

pub(super) trait AutoFlightGeneralInhibit {
    fn general_inhibit(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AutoFlightGeneralInhibitActivation {
    general_inhibit: bool,
}

impl AutoFlightGeneralInhibitActivation {
    pub fn update(&mut self, signals: &(impl AltiSelect + AltSelectChg)) {
        let alti_select = signals.alti_select();
        let bad_alti_select = alti_select.is_ncd() || alti_select.is_inv();

        self.general_inhibit = bad_alti_select || signals.alt_select_chg().value();
    }
}

impl AutoFlightGeneralInhibit for AutoFlightGeneralInhibitActivation {
    fn general_inhibit(&self) -> bool {
        self.general_inhibit
    }
}

pub(super) trait AutoFlightAltitudeThreshold {
    fn alt_200(&self) -> bool;
    fn alt_750(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AutoFlightAltitudeThresholdActivation {
    alt_200: bool,
    alt_750: bool,
}

impl AutoFlightAltitudeThresholdActivation {
    pub fn update(&mut self, signals: &impl AltiSelect, altitude_sheet: AltitudeActivation) {
        let difference = altitude_sheet.altitude - signals.alti_select().value();

        // TODO baro-corrected altitudes
        self.alt_200 = difference.abs() < Length::new::<foot>(200.0);
        self.alt_750 = difference.abs() < Length::new::<foot>(750.0);
    }
}

impl AutoFlightAltitudeThreshold for AutoFlightAltitudeThresholdActivation {
    fn alt_200(&self) -> bool {
        self.alt_200
    }

    fn alt_750(&self) -> bool {
        self.alt_750
    }
}

pub(super) trait AutopilotTcasAltitudeAlertInhibition {
    fn ap_tcas_mode_eng(&self) -> bool;
    fn alt_alert_inib(&self) -> bool;
}

pub(super) struct AutopilotTcasAltitudeAlertInhibitionActivation {
    pulse1: PulseNode,
    pulse2: PulseNode,
    pulse3: PulseNode,
    pulse4: PulseNode,
    mrtrig1: MonostableTriggerNode,
    mrtrig2: MonostableTriggerNode,
    mem_altitude_alert_inhib: MemoryNode,
    ap_tcas_mode_eng: bool,
    alt_alert_inib: bool,
}

impl Default for AutopilotTcasAltitudeAlertInhibitionActivation {
    fn default() -> Self {
        Self {
            mem_altitude_alert_inhib: MemoryNode::new(true),
            pulse1: PulseNode::new(false),
            pulse2: PulseNode::new(true),
            pulse3: PulseNode::new(false),
            pulse4: PulseNode::new(false),
            mrtrig1: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(1)),
            mrtrig2: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(1)),
            ap_tcas_mode_eng: false,
            alt_alert_inib: false,
        }
    }
}

impl AutopilotTcasAltitudeAlertInhibitionActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl FakeSignalApTcasEngaged + AltSelectChg),
        lg_downlocked_sheet: &impl LgDownlocked,
        alti_threshold_sheet: &impl AutoFlightAltitudeThreshold,
        general_inhibit_sheet: &impl AutoFlightGeneralInhibit,
    ) {
        self.ap_tcas_mode_eng = signals.ap_tcas_engaged();

        let alt_200 = alti_threshold_sheet.alt_200();
        let alt_750 = alti_threshold_sheet.alt_750();
        let inhibit = general_inhibit_sheet.general_inhibit();
        let lg_downlocked = lg_downlocked_sheet.lg_downlocked();

        let pulse1_out = self.pulse1.update(alt_200 && alt_750 && !inhibit);

        let pulse23_in = !alt_200 && !alt_750 && !inhibit;
        let pulse2_out = self.pulse2.update(pulse23_in);

        let set = self.ap_tcas_mode_eng && (pulse1_out || pulse2_out);

        let pulse3_out = self.pulse3.update(pulse23_in);
        let pulse4_out = self.pulse4.update(lg_downlocked);

        let mrtrig1_out = self.mrtrig1.update(lg_downlocked, delta);
        let mrtrig2_out = self.mrtrig2.update(signals.alt_select_chg().value(), delta);

        let reset = pulse3_out || pulse4_out || mrtrig1_out || mrtrig2_out;
        self.alt_alert_inib = self.mem_altitude_alert_inhib.update(set, reset);
    }
}

impl AutopilotTcasAltitudeAlertInhibition for AutopilotTcasAltitudeAlertInhibitionActivation {
    fn ap_tcas_mode_eng(&self) -> bool {
        self.ap_tcas_mode_eng
    }

    fn alt_alert_inib(&self) -> bool {
        self.alt_alert_inib
    }
}

pub(super) trait AutoFlightAltAlert {
    fn c_chord(&self) -> bool;
    fn steady_light(&self) -> bool;
    fn flashing_light(&self) -> bool;
}

pub(super) struct AutoFlightAltAlertActivation {
    pulse1: PulseNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mtrig3: MonostableTriggerNode,
    mtrig4: MonostableTriggerNode,
    mem_within_200: MemoryNode,
    mem_within_750: MemoryNode,
    c_chord: bool,
    steady_light: bool,
    flashing_light: bool,
}

impl Default for AutoFlightAltAlertActivation {
    fn default() -> Self {
        Self {
            pulse1: PulseNode::new(false),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs(1)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs(1)),
            mtrig3: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.5)),
            mtrig4: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.5)),
            mem_within_200: MemoryNode::new(false),
            mem_within_750: MemoryNode::new(false),
            c_chord: false,
            steady_light: false,
            flashing_light: false,
        }
    }
}

impl AutoFlightAltAlertActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &impl AltSelectChg,
        gnd_sheet: &impl GroundDetection,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
        ap_tcas_alt_inhibit_sheet: &impl AutopilotTcasAltitudeAlertInhibition,
        threshold_sheet: &impl AutoFlightAltitudeThreshold,
        inhibit_sheet: &impl AutoFlightGeneralInhibit,
        lg_sheet: &impl LgDownlocked,
    ) {
        let ap_tcas_mode_eng = ap_tcas_alt_inhibit_sheet.ap_tcas_mode_eng();
        let ground_or_ap_tcas = gnd_sheet.ground() || ap_tcas_mode_eng;

        let mtrig1_out = self.mtrig1.update(signals.alt_select_chg().value(), delta);
        let mtrig2_out = self.mtrig2.update(lg_sheet.lg_downlocked(), delta);
        let mtrig12_out = mtrig1_out || mtrig2_out;

        let alt_200 = threshold_sheet.alt_200();
        let alt_750 = threshold_sheet.alt_750();
        let general_inhibit = inhibit_sheet.general_inhibit();

        let within_750 = alt_750 && !alt_200 && !general_inhibit;

        let not_750 = !alt_200 && !alt_750 && !general_inhibit;

        let mem200_out = self.mem_within_200.update(
            alt_750 && alt_200 && !general_inhibit,
            not_750 || mtrig12_out,
        );

        let mem750_out = self.mem_within_750.update(within_750, mtrig12_out);

        let flashing_light_cond1 = not_750 && mem750_out;
        let flashing_light_cond2 = within_750 && mem200_out;
        let flashing_light_cond = flashing_light_cond1 || flashing_light_cond2;

        self.flashing_light = !ground_or_ap_tcas && flashing_light_cond;
        self.steady_light = !ground_or_ap_tcas && within_750 && !flashing_light_cond1;

        let one_ap_engd = ap_sheet.one_ap_engd();
        let mtrig3_out = self.mtrig3.update(!one_ap_engd && within_750, delta);

        let mtrig4_in = (!one_ap_engd && self.pulse1.update(ap_tcas_mode_eng) && !general_inhibit);
        let mtrig4_out = self.mtrig4.update(mtrig4_in, delta);

        self.c_chord = !ground_or_ap_tcas
            && (mtrig3_out
                || mtrig4_out
                || (!ap_tcas_alt_inhibit_sheet.alt_alert_inib() && flashing_light_cond));
    }
}

impl AutoFlightAltAlert for AutoFlightAltAlertActivation {
    fn c_chord(&self) -> bool {
        self.c_chord
    }

    fn steady_light(&self) -> bool {
        self.steady_light
    }

    fn flashing_light(&self) -> bool {
        self.flashing_light
    }
}

#[cfg(test)]
mod tests {
    use crate::flight_warning::test::test_bed_with;
    use uom::si::f64::*;
    use uom::si::{length::foot, velocity::knot};

    use super::*;

    #[cfg(test)]
    mod new_ground_activation_tests {
        use super::*;

        #[test]
        fn when_all_compressed_new_ground_and_not_inv() {
            let mut sheet = NewGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .lh_lg_compressed(1)
                    .ess_lh_lg_compressed()
                    .lh_lg_compressed(2)
                    .norm_lh_lg_compressed()
                    .parameters(),
            );
            assert_eq!(sheet.new_ground, true);
            assert_eq!(sheet.lgciu_12_inv, false);
        }

        #[test]
        fn when_none_compressed_new_ground_and_not_inv() {
            let mut sheet = NewGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .lh_lg_extended(1)
                    .lh_lg_extended(2)
                    .parameters(),
            );
            assert_eq!(sheet.new_ground, false);
            assert_eq!(sheet.lgciu_12_inv, false);
        }

        #[test]
        fn when_single_lgciu_mismatch_then_lgciu12_inv() {
            let mut sheet = NewGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .lh_lg_compressed(1)
                    .lh_lg_extended(2)
                    .parameters(),
            );
            assert_eq!(sheet.new_ground, false);
            assert_eq!(sheet.lgciu_12_inv, true);
        }
    }

    #[cfg(test)]
    mod ground_detection_activation_tests {
        use super::*;

        struct TestLgciuGroundDetection {
            new_ground: bool,
            lgciu_12_inv: bool,
        }

        impl TestLgciuGroundDetection {
            fn new(new_ground: bool, lgciu_12_inv: bool) -> Self {
                Self {
                    new_ground: new_ground,
                    lgciu_12_inv: lgciu_12_inv,
                }
            }
        }

        impl NewGround for TestLgciuGroundDetection {
            fn new_ground(&self) -> bool {
                self.new_ground
            }
            fn lgciu_12_inv(&self) -> bool {
                self.lgciu_12_inv
            }
        }

        #[test]
        fn when_on_ground_ground_immediate_and_ground() {
            let mut sheet = GroundDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .ess_lh_lg_compressed()
                    .norm_lh_lg_compressed()
                    .radio_heights(Length::new::<foot>(0.0), Length::new::<foot>(-1.0))
                    .parameters(),
                &TestLgciuGroundDetection::new(true, false),
            );
            assert_eq!(sheet.ground_immediate(), true);
            assert_eq!(sheet.ground(), true);
        }

        #[test]
        fn when_touching_down_triggers_ground_immediate_first() {
            let mut sheet = GroundDetectionActivation::new();
            sheet.update(
                Duration::from_millis(500),
                test_bed_with()
                    .ess_lh_lg_compressed()
                    .norm_lh_lg_compressed()
                    .radio_heights(Length::new::<foot>(0.0), Length::new::<foot>(-1.0))
                    .parameters(),
                &TestLgciuGroundDetection::new(true, false),
            );
            assert_eq!(sheet.ground_immediate(), true);
            assert_eq!(sheet.ground(), false);
            sheet.update(
                Duration::from_millis(500),
                test_bed_with()
                    .ess_lh_lg_compressed()
                    .norm_lh_lg_compressed()
                    .radio_heights(Length::new::<foot>(0.0), Length::new::<foot>(-1.0))
                    .parameters(),
                &TestLgciuGroundDetection::new(true, false),
            );
            assert_eq!(sheet.ground_immediate(), true);
            assert_eq!(sheet.ground(), true);
        }

        #[test]
        fn when_dual_ra_failure_on_ground() {
            let mut sheet = GroundDetectionActivation::new();
            sheet.update(
                Duration::from_millis(500),
                test_bed_with()
                    .ess_lh_lg_compressed()
                    .norm_lh_lg_compressed()
                    .parameters(),
                &TestLgciuGroundDetection::new(true, false),
            );
            assert_eq!(sheet.ground_immediate(), true);
            assert_eq!(sheet.ground(), false);
        }
    }

    mod test_tla_at_mct_or_flex_to_cfm_activation_tests {
        use super::*;

        #[test]
        fn when_at_37_0_degress_above_mct() {
            let mut sheet = TlaAtMctOrFlexToCfmActivation::new();
            sheet.update(
                test_bed_with()
                    .eng1_tla(Angle::new::<degree>(37.0))
                    .parameters(),
            );
            assert_eq!(sheet.eng_1_tla_mct_cfm(), false);
            assert_eq!(sheet.eng_1_end_mct(), false);
            assert_eq!(sheet.eng_1_sup_mct_cfm(), true);
        }

        #[test]
        fn when_at_36_55_degress_at_end_mct() {
            let mut sheet = TlaAtMctOrFlexToCfmActivation::new();
            sheet.update(
                test_bed_with()
                    .eng1_tla(Angle::new::<degree>(36.65))
                    .parameters(),
            );
            assert_eq!(sheet.eng_1_tla_mct_cfm(), true);
            assert_eq!(sheet.eng_1_end_mct(), true);
            assert_eq!(sheet.eng_1_sup_mct_cfm(), false);
        }

        #[test]
        fn when_at_33_degress_not_in_mct() {
            let mut sheet = TlaAtMctOrFlexToCfmActivation::new();
            sheet.update(
                test_bed_with()
                    .eng1_tla(Angle::new::<degree>(33.0))
                    .parameters(),
            );
            assert_eq!(sheet.eng_1_tla_mct_cfm(), false);
            assert_eq!(sheet.eng_1_end_mct(), false);
            assert_eq!(sheet.eng_1_sup_mct_cfm(), false);
        }

        #[test]
        fn when_at_35_degress_in_mct() {
            let mut sheet = TlaAtMctOrFlexToCfmActivation::new();
            sheet.update(
                test_bed_with()
                    .eng1_tla(Angle::new::<degree>(35.0))
                    .parameters(),
            );
            assert_eq!(sheet.eng_1_tla_mct_cfm(), true);
            assert_eq!(sheet.eng_1_end_mct(), false);
            assert_eq!(sheet.eng_1_sup_mct_cfm(), false);
        }
    }

    #[cfg(test)]
    mod speed_detection_activation_tests {
        use super::*;

        #[test]
        fn when_at_0_kt_not_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(0.0),
                        Velocity::new::<knot>(0.0),
                        Velocity::new::<knot>(0.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), false);
        }

        #[test]
        fn when_at_250_kt_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(250.0),
                        Velocity::new::<knot>(250.0),
                        Velocity::new::<knot>(250.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), true);
        }

        #[test]
        fn when_one_adc_at_250_kt_not_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(250.0),
                        Velocity::new::<knot>(0.0),
                        Velocity::new::<knot>(0.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), false);
        }

        #[test]
        fn when_two_at_250_kt_and_adc_failure_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speed_1(Velocity::new::<knot>(250.0))
                    .parameters(), // TODO ADC failures
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), true);
        }

        #[test]
        fn when_two_adcs_at_250_kt_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(250.0),
                        Velocity::new::<knot>(0.0),
                        Velocity::new::<knot>(250.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), true);
        }

        #[test]
        fn when_spikes_below_50_to_above_80_kt_not_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(49.0),
                        Velocity::new::<knot>(49.0),
                        Velocity::new::<knot>(49.0),
                    )
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.5),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(84.0),
                        Velocity::new::<knot>(84.0),
                        Velocity::new::<knot>(84.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), false);
        }

        #[test]
        fn when_jumps_below_50_to_above_80_kt_above_80_kt() {
            let mut sheet = SpeedDetectionActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(49.0),
                        Velocity::new::<knot>(49.0),
                        Velocity::new::<knot>(49.0),
                    )
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .computed_speeds(
                        Velocity::new::<knot>(84.0),
                        Velocity::new::<knot>(84.0),
                        Velocity::new::<knot>(84.0),
                    )
                    .parameters(),
            );
            assert_eq!(sheet.ac_speed_above_80_kt(), true);
        }
    }

    #[cfg(test)]
    mod engines_not_running_activation_tests {
        use super::*;

        #[test]
        fn when_engines_off() {
            let mut sheet = EnginesNotRunning::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
            );
            assert_eq!(sheet.eng_1_not_running(), true);
            assert_eq!(sheet.eng_2_not_running(), true);
        }

        #[test]
        fn when_engine_off_and_master_lever_on() {
            let mut sheet = EnginesNotRunning::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().eng1_master_lever_select_on().parameters(),
                &TestGroundDetection::new(true),
            );
            assert_eq!(sheet.eng_1_not_running(), true);
            assert_eq!(sheet.eng_2_not_running(), true);
        }

        #[test]
        fn when_engine_on_and_master_lever_on_after_30_secs() {
            let mut sheet = EnginesNotRunning::new();
            sheet.update(
                Duration::from_secs(30),
                test_bed_with()
                    .eng1_master_lever_select_on()
                    .eng1_at_or_above_idle()
                    .parameters(),
                &TestGroundDetection::new(true),
            );
            assert_eq!(sheet.eng_1_not_running(), false);
            assert_eq!(sheet.eng_2_not_running(), true);
        }

        #[test]
        fn when_engine_on_and_master_lever_off_after_30_secs() {
            let mut sheet = EnginesNotRunning::new();
            sheet.update(
                Duration::from_secs(30),
                test_bed_with().eng1_at_or_above_idle().parameters(),
                &TestGroundDetection::new(true),
            );
            assert_eq!(sheet.eng_1_not_running(), false);
            assert_eq!(sheet.eng_2_not_running(), true);
        }

        // TODO repeat above tests for engine 2 (macro?)

        #[test]
        fn when_engine_1_on_and_master_lever_on_in_flight_with_fire_pb() {
            let mut sheet = EnginesNotRunning::new();

            // Engine 1 just turned on, we would need to wait 30s for "off" confirmation...
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().eng1_at_or_above_idle().parameters(),
                &TestGroundDetection::new(false),
            );
            assert_eq!(sheet.eng_1_not_running(), true);

            // ...however toggling the fire p/b immediately forces it to off
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .eng1_at_or_above_idle()
                    .eng1_fire_pb_out()
                    .parameters(),
                &TestGroundDetection::new(false),
            );
            assert_eq!(sheet.eng_1_not_running(), false);
        }

        #[test]
        fn when_engine_2_on_and_master_lever_on_in_flight_with_fire_pb() {
            let mut sheet = EnginesNotRunning::new();

            // Engine 2 just turned on, we would need to wait 30s for "off" confirmation...
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().eng1_at_or_above_idle().parameters(),
                &TestGroundDetection::new(false),
            );
            assert_eq!(sheet.eng_2_not_running(), true);

            // ...however toggling engine 1 (!!) fire p/b immediately forces it to off
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .eng1_at_or_above_idle()
                    .eng1_fire_pb_out()
                    .parameters(),
                &TestGroundDetection::new(false),
            );
            assert_eq!(sheet.eng_2_not_running(), false);
        }

        // TODO: a/b channel discrepancy
    }

    #[cfg(test)]
    mod altitude_def_activation_tests {
        use super::*;

        #[test]
        fn when_at_cruise() {
            let mut sheet = AltitudeDefActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().radio_heights_at_cruise().parameters(),
            );
            assert_eq!(sheet.h_fail(), false);
            assert_eq!(sheet.h_gt_800ft, true);
            assert_eq!(sheet.h_gt_1500ft, true);
        }

        #[test]
        fn when_above_1500ft() {
            let mut sheet = AltitudeDefActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(1600.0), Length::new::<foot>(1600.0))
                    .parameters(),
            );
            assert_eq!(sheet.h_fail(), false);
            assert_eq!(sheet.h_gt_800ft, true);
            assert_eq!(sheet.h_gt_1500ft, true);
        }

        #[test]
        fn when_above_800ft_and_below_1500ft() {
            let mut sheet = AltitudeDefActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(1501.0), Length::new::<foot>(1501.0))
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(1499.0), Length::new::<foot>(1499.0))
                    .parameters(),
            );
            assert_eq!(sheet.h_fail(), false);
            assert_eq!(sheet.h_gt_800ft, true);
            assert_eq!(sheet.h_gt_1500ft, false);
        }

        #[test]
        fn when_below_800ft() {
            let mut sheet = AltitudeDefActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(1501.0), Length::new::<foot>(1501.0))
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(799.0), Length::new::<foot>(799.0))
                    .parameters(),
            );
            assert_eq!(sheet.h_fail(), false);
            assert_eq!(sheet.h_gt_800ft, false);
            assert_eq!(sheet.h_gt_1500ft, false);
        }

        #[test]
        fn when_on_ground() {
            let mut sheet = AltitudeDefActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with()
                    .radio_heights(Length::new::<foot>(0.0), Length::new::<foot>(0.0))
                    .parameters(),
            );
            assert_eq!(sheet.h_fail(), false);
            assert_eq!(sheet.h_gt_800ft, false);
            assert_eq!(sheet.h_gt_1500ft, false);
        }
    }

    #[cfg(test)]
    mod flight_phases_ground {
        use super::*;

        #[test]
        fn when_cold_and_dark_is_phase_1() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(false, false),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            assert!(sheet.phase_1());
        }

        #[test]
        fn when_one_eng_running_is_phase_2() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, false),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            assert!(sheet.phase_2());
        }

        #[test]
        fn when_at_flex_takeoff_is_phase_3() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_flex(),
            );
            assert!(sheet.phase_3());
        }

        #[test]
        fn when_at_toga_takeoff_is_phase_3() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_toga(),
            );
            assert!(sheet.phase_3());
        }

        #[test]
        fn when_at_toga_above_80_kt_is_phase_4() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(true),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_toga(),
            );
            assert!(sheet.phase_4());
        }

        #[test]
        fn when_engine_failed_above_80_kt_is_phase_4() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(true),
                &TestEngRunning::new(false, true),
                &TestCfmFlightPhasesDef::new_toga(),
            );
            assert!(sheet.phase_4());
        }

        #[test]
        fn when_below_flex_above_80_kt_is_phase_8() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(true),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            assert!(sheet.phase_8());
        }

        #[test]
        fn after_rto_below_80_knots_is_phase_9() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_toga(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            assert!(sheet.phase_9());
        }

        #[test]
        fn after_rto_below_80_knots_and_to_config_is_phase_2() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_toga(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().takeoff_config_test_pressed().parameters(),
                &TestGroundDetection::new(true),
                &TestSpeedDetection::new(false),
                &TestEngRunning::new(true, true),
                &TestCfmFlightPhasesDef::new_below_flex(),
            );
            assert!(sheet.phase_2());
        }

        /*#[test]
        fn after_engine_shutdown_reset_to_phase_1() {
            let mut sheet = FlightPhasesGroundActivation::new();
            sheet.update(Duration::from_secs(1), test_bed().signals());
            assert_eq!(sheet.get_phase(), 3);
            sheet.update(Duration::from_secs(30), test_bed().signals());
            sheet.update(Duration::from_secs(60), test_bed().signals());
            assert_eq!(sheet.get_phase(), 9);
            sheet.update(Duration::from_secs(60), test_bed().signals());
            assert_eq!(sheet.get_phase(), 10);
            sheet.update(Duration::from_secs(300), test_bed().signals());
            assert_eq!(sheet.get_phase(), 1);
            sheet.update(Duration::from_secs(1), test_bed().signals());
        }*/
    }

    #[cfg(test)]
    mod flight_phases_air {
        use super::*;

        #[test]
        fn when_at_toga_below_1500ft_is_phase_5() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_takeoff(Length::new::<foot>(1499.0)),
                &TestCfmFlightPhasesDef::new_toga(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_5());
        }

        #[test]
        fn when_at_flex_below_1500ft_is_phase_5() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_takeoff(Length::new::<foot>(1499.0)),
                &TestCfmFlightPhasesDef::new_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_5());
        }

        #[test]
        fn when_at_takeoff_power_above_1500ft_is_phase_6() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_takeoff(Length::new::<foot>(1500.1)),
                &TestCfmFlightPhasesDef::new_toga(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_6());
        }

        #[test]
        fn when_at_flex_above_1500ft_is_phase_6() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_takeoff(Length::new::<foot>(1500.1)),
                &TestCfmFlightPhasesDef::new_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_6());
        }

        #[test]
        fn when_at_below_flex_below_800ft_is_phase_7() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_landing(Length::new::<foot>(799.9)),
                &TestCfmFlightPhasesDef::new_below_flex(),
                &TestFlightPhasesGround::default(),
            );
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_landing(Length::new::<foot>(799.9)),
                &TestCfmFlightPhasesDef::new_below_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_7());
        }

        #[test]
        fn when_at_flex_below_800ft_is_phase_5() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_landing(Length::new::<foot>(799.9)),
                &TestCfmFlightPhasesDef::new_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_5());
        }

        #[test]
        fn when_at_toga_below_800ft_is_phase_5() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_landing(Length::new::<foot>(799.9)),
                &TestCfmFlightPhasesDef::new_toga(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_5());
        }

        #[test]
        fn when_ra_failed_is_phase_6() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(false),
                &TestAltitudeDef::new_failed(),
                &TestCfmFlightPhasesDef::new_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(sheet.phase_6());
        }

        #[test]
        fn when_on_ground_and_ra_failed_is_no_phase() {
            let mut sheet = FlightPhasesAirActivation::new();
            sheet.update(
                Duration::from_secs(1),
                &TestGroundDetection::new(true),
                &TestAltitudeDef::new_failed(),
                &TestCfmFlightPhasesDef::new_below_flex(),
                &TestFlightPhasesGround::default(),
            );
            assert!(!sheet.phase_5());
            assert!(!sheet.phase_6());
            assert!(!sheet.phase_7());
        }
    }

    mod voluntary_autopilot_warnings {
        use super::*;
        use crate::flight_warning::test::test_bed;

        #[test]
        fn reports_ap1_engaged() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs(1),
                false,
                test_bed_with().ap1_engaged(true).parameters(),
            );
            assert_eq!(sheet.ap1_engd(), true);
            assert_eq!(sheet.ap2_engd(), false);
            assert_eq!(sheet.one_ap_engd(), true);
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
        }

        #[test]
        fn reports_ap2_engaged() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs(1),
                false,
                test_bed_with().ap2_engaged(true).parameters(),
            );
            assert_eq!(sheet.ap1_engd(), false);
            assert_eq!(sheet.ap2_engd(), true);
            assert_eq!(sheet.one_ap_engd(), true);
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
        }

        #[test]
        fn reports_both_aps_engaged() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs(1),
                false,
                test_bed_with()
                    .ap1_engaged(true)
                    .ap2_engaged(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap1_engd(), true);
            assert_eq!(sheet.ap2_engd(), true);
            assert_eq!(sheet.one_ap_engd(), true);
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
        }

        #[test]
        fn warns_when_ap1_is_instinctively_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with().ap1_engaged(true).parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(1.5), true, test_bed().parameters());
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(1.5), true, test_bed().parameters());
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(6.0), false, test_bed().parameters());
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
        }

        #[test]
        fn stops_warning_when_instinctive_disconnect_is_pressed_again() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.001),
                false,
                test_bed_with().ap1_engaged(true).parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.001),
                false,
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(
                Duration::from_secs_f64(0.001),
                false,
                test_bed_with()
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
            );

            // another press within 0.2s is inhibited
            sheet.update(
                Duration::from_secs_f64(0.001),
                false,
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.001),
                false,
                test_bed_with()
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);

            // after 0.2s a press inhibits the mw and text signal, but audio remains
            sheet.update(
                Duration::from_secs_f64(0.2),
                false,
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);

            // once 0.5s have passed since the inhibit is pressed, the audio is also inhibited
            sheet.update(
                Duration::from_secs_f64(0.5),
                false,
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio(), false);
        }

        #[test]
        fn warns_when_ap2_is_instinctively_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with().ap2_engaged(true).parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with()
                    .ap2_engaged(false)
                    .instinc_disconnect_2ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio, true);
            assert_eq!(sheet.ap_off_mw, true);
            assert_eq!(sheet.ap_off_text, true);
        }

        #[test]
        fn warns_when_both_aps_are_instinctively_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with()
                    .ap1_engaged(true)
                    .ap2_engaged(true)
                    .parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with()
                    .ap1_engaged(false)
                    .ap2_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio, true);
            assert_eq!(sheet.ap_off_mw, true);
            assert_eq!(sheet.ap_off_text, true);
        }

        #[test]
        fn does_not_warn_when_ap1_is_involuntarily_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with().ap1_engaged(true).parameters(),
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                false,
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
            );
            assert_eq!(sheet.ap_off_audio, false);
            assert_eq!(sheet.ap_off_mw, false);
            assert_eq!(sheet.ap_off_text, false);
        }
    }

    struct TestGroundDetection {
        ground: bool,
        ground_immediate: bool,
    }

    impl TestGroundDetection {
        fn new(ground: bool) -> Self {
            Self {
                ground: ground,
                ground_immediate: ground,
            }
        }

        fn new_with_immediate(ground: bool, ground_immediate: bool) -> Self {
            Self {
                ground: ground,
                ground_immediate: ground_immediate,
            }
        }
    }

    impl GroundDetection for TestGroundDetection {
        fn ground(&self) -> bool {
            self.ground
        }

        fn ground_immediate(&self) -> bool {
            self.ground_immediate
        }
    }

    struct TestSpeedDetection {
        ac_speed_above_80_kt: bool,
        adc_test_inhib: bool,
    }

    impl TestSpeedDetection {
        fn new(ac_speed_above_80_kt: bool) -> Self {
            Self {
                ac_speed_above_80_kt,
                adc_test_inhib: false,
            }
        }

        fn new_adc_test(ac_speed_above_80_kt: bool) -> Self {
            Self {
                ac_speed_above_80_kt,
                adc_test_inhib: true,
            }
        }
    }

    impl SpeedDetection for TestSpeedDetection {
        fn ac_speed_above_80_kt(&self) -> bool {
            self.ac_speed_above_80_kt
        }

        fn adc_test_inhib(&self) -> bool {
            self.adc_test_inhib
        }
    }

    struct TestAltitudeDef {
        h_gt_800ft: bool,
        h_gt_1500ft: bool,
        h_fail: bool,
    }

    impl TestAltitudeDef {
        fn new_takeoff(height: Length) -> Self {
            Self {
                h_gt_800ft: height > Length::new::<foot>(1500.0),
                h_gt_1500ft: height > Length::new::<foot>(1500.0),
                h_fail: false,
            }
        }

        fn new_landing(height: Length) -> Self {
            Self {
                h_gt_800ft: height > Length::new::<foot>(800.0),
                h_gt_1500ft: height > Length::new::<foot>(1500.0),
                h_fail: false,
            }
        }

        fn new_failed() -> Self {
            Self {
                h_gt_800ft: false,
                h_gt_1500ft: false,
                h_fail: true,
            }
        }
    }

    impl AltitudeDef for TestAltitudeDef {
        fn h_gt_800ft(&self) -> bool {
            self.h_gt_800ft
        }

        fn h_gt_1500ft(&self) -> bool {
            self.h_gt_1500ft
        }

        fn h_fail(&self) -> bool {
            self.h_fail
        }
    }

    struct TestEngRunning {
        eng_1_and_2_not_running: bool,
        eng_1_or_2_running: bool,
        one_eng_running: bool,
    }

    impl TestEngRunning {
        fn new(eng_1_running: bool, eng_2_running: bool) -> Self {
            Self {
                eng_1_and_2_not_running: !eng_1_running && !eng_2_running,
                eng_1_or_2_running: eng_1_running || eng_2_running,
                one_eng_running: eng_1_running || eng_2_running,
            }
        }

        fn new_unconfirmed_eng_running(eng_1_running: bool, eng_2_running: bool) -> Self {
            Self {
                eng_1_and_2_not_running: !eng_1_running && !eng_2_running,
                eng_1_or_2_running: false,
                one_eng_running: eng_1_running || eng_2_running,
            }
        }
    }

    impl EngRunning for TestEngRunning {
        fn eng_1_and_2_not_running(&self) -> bool {
            self.eng_1_and_2_not_running
        }

        fn eng_1_or_2_running(&self) -> bool {
            self.eng_1_or_2_running
        }

        fn one_eng_running(&self) -> bool {
            self.one_eng_running
        }
    }

    struct TestCfmFlightPhasesDef {
        cfm_flex: bool,
        eng_1_or_2_to_pwr: bool,
    }

    impl TestCfmFlightPhasesDef {
        fn new(cfm_flex: bool, eng_1_or_2_to_pwr: bool) -> Self {
            Self {
                cfm_flex,
                eng_1_or_2_to_pwr,
            }
        }

        fn new_below_flex() -> Self {
            Self::new(false, false)
        }

        fn new_flex() -> Self {
            Self::new(true, true)
        }

        fn new_toga() -> Self {
            Self::new(false, true)
        }
    }

    impl CfmFlightPhasesDef for TestCfmFlightPhasesDef {
        fn cfm_flex(&self) -> bool {
            self.cfm_flex
        }

        fn eng_1_or_2_to_pwr(&self) -> bool {
            self.eng_1_or_2_to_pwr
        }
    }

    struct TestFlightPhasesGround {
        phase: usize,
    }

    impl TestFlightPhasesGround {
        fn new(phase: usize) -> Self {
            Self { phase }
        }
    }

    impl FlightPhasesGround for TestFlightPhasesGround {
        fn phase_1(&self) -> bool {
            self.phase == 1
        }

        fn phase_2(&self) -> bool {
            self.phase == 2
        }

        fn phase_3(&self) -> bool {
            self.phase == 3
        }

        fn phase_4(&self) -> bool {
            self.phase == 4
        }

        fn phase_8(&self) -> bool {
            self.phase == 8
        }

        fn phase_9(&self) -> bool {
            self.phase == 9
        }

        fn phase_10(&self) -> bool {
            self.phase == 10
        }
    }

    impl Default for TestFlightPhasesGround {
        fn default() -> Self {
            Self::new(0)
        }
    }
}

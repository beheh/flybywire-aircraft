use std::time::Duration;

use super::super::parameters::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::{SignStatusMatrix, Value};
use systems::flight_warning::utils::FwcSsm;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::{percent, ratio};
use uom::si::velocity::knot;

pub(super) trait WarningActivation {
    fn audio(&self) -> bool {
        self.warning()
    }
    fn warning(&self) -> bool;
}

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
        altitude_def: &impl FlightPhasesAltitudeDef,
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

pub(super) trait FlightPhasesAltitudeDef {
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

impl FlightPhasesAltitudeDef for AltitudeDefActivation {
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
        altitude_sheet: &impl FlightPhasesAltitudeDef,
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

pub(super) trait GeneralDhDtPositive {
    fn dh_positive(&self) -> bool;
}

#[derive(Default)]
pub(super) struct GeneralDhDtPositiveActivation {
    last_rh: Length,
    dh_positive: bool,
}

impl GeneralDhDtPositiveActivation {
    pub fn update(&mut self, signals: &impl RadioHeight) {
        let rh1 = signals.radio_height(1);
        let rh1_inv_or_ncd = rh1.is_inv() || rh1.is_ncd();
        let rh2 = signals.radio_height(2);
        let rh = if rh1_inv_or_ncd { rh2 } else { rh1 };
        let rh_value = rh.value();

        let derivative = rh_value - self.last_rh;
        self.last_rh = rh_value;
        self.dh_positive = derivative > Length::new::<foot>(0.0);
    }
}

impl GeneralDhDtPositive for GeneralDhDtPositiveActivation {
    fn dh_positive(&self) -> bool {
        self.dh_positive
    }
}

pub(super) trait GeneralCancel {
    fn mw_cancel_pulse_up(&self) -> bool;
    fn mc_cancel_pulse_up(&self) -> bool;
}

pub(super) struct GeneralCancelActivation {
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

pub(super) trait Eng1StartSequence {
    fn eng_1_tempo_master_lever_1_on(&self) -> bool;
}

pub(super) struct Eng1StartSequenceActivation {
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

pub(super) trait Eng2StartSequence {
    fn eng_2_tempo_master_lever_1_on(&self) -> bool;
    fn phase_5_to_30s(&self) -> bool;
}

pub(super) struct Eng2StartSequenceActivation {
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

pub(super) trait AutoFlightBaroAltitude {
    /// This signal contains the barometric altitude that is picked from the first available ADR.
    /// It may be nonsensical if all three ADRs are unavailable.
    fn alti_basic(&self) -> Length;

    fn alti_invalid(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AutoFlightBaroAltitudeActivation {
    alti_basic: Length,
    alti_invalid: bool,
}

impl AutoFlightBaroAltitudeActivation {
    pub fn update(&mut self, signals: &impl AltitudeParameter) {
        let alti1 = signals.altitude(1);
        let alti2 = signals.altitude(2);
        let alti3 = signals.altitude(3);

        let bad_alti1 = alti1.is_ncd() || alti1.is_inv();
        let bad_alti2 = alti2.is_ncd() || alti2.is_inv();
        let bad_alti3 = alti3.is_ncd() || alti3.is_inv();

        let two_over_one = bad_alti1;
        let three_over_one_and_two = bad_alti1 && bad_alti2;

        self.alti_basic = if three_over_one_and_two {
            alti3.value()
        } else if two_over_one {
            alti2.value()
        } else {
            alti1.value()
        };

        let buss_installed = false; // TODO
        let gps_alt_used_and_invalid = false; // TODO
        self.alti_invalid =
            (bad_alti1 && bad_alti2 && bad_alti3 && !buss_installed) || gps_alt_used_and_invalid;
    }
}

impl AutoFlightBaroAltitude for AutoFlightBaroAltitudeActivation {
    fn alti_basic(&self) -> Length {
        self.alti_basic
    }

    fn alti_invalid(&self) -> bool {
        self.alti_invalid
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
        signals: &(impl Ap1Engd
              + Ap2Engd
              + CaptMwCancelOn
              + FoMwCancelOn
              + InstincDiscnct1ApEngd
              + InstincDiscnct2ApEngd),
        cavalry_charge_emitted: bool,
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
    /// This signal indicates that the textual warning part of the AP OFF warning should be visible
    /// on the EWD.
    fn ap_off_warning(&self) -> bool;

    /// This signal indicates that the aural part of the AP off warning should be active.
    /// TODO Replace with real warning block (so it can be cleared)
    fn ap_off_audio(&self) -> bool;

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
    ap_off_audio: bool,
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
            ap_off_audio: false,
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
        signals: &(impl Ap1Engd
              + Ap2Engd
              + CaptMwCancelOn
              + FoMwCancelOn
              + InstincDiscnct1ApEngd
              + InstincDiscnct2ApEngd
              + BlueSysLoPr
              + YellowSysLoPr
              + GreenSysLoPr),
        voluntary_sheet: &impl AutoFlightAutopilotOffVoluntary,
        flight_phases_ground: &impl FlightPhasesGround,
        cavalry_charge_emitted: bool,
    ) {
        let phase1 = flight_phases_ground.phase_1();
        let inhibited_on_ground = phase1
            && signals.blue_sys_lo_pr().value()
            && signals.yellow_sys_lo_pr().value()
            && signals.green_sys_lo_pr().value();
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

    fn ap_off_audio(&self) -> bool {
        self.ap_off_audio
    }
}

pub(super) trait AltitudeAlertThresholds {
    fn alt_200(&self) -> bool;
    fn alt_750(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeAlertThresholdsActivation {
    alt_200: bool,
    alt_750: bool,
}

impl AltitudeAlertThresholdsActivation {
    pub fn update(
        &mut self,
        signals: &impl AltiSelect,
        altitude_sheet: &impl AutoFlightBaroAltitude,
    ) {
        let difference = altitude_sheet.alti_basic() - signals.alti_select().value();

        // TODO baro-corrected altitudes
        self.alt_200 = difference.abs() < Length::new::<foot>(200.0);
        self.alt_750 = difference.abs() < Length::new::<foot>(750.0);
    }
}

impl AltitudeAlertThresholds for AltitudeAlertThresholdsActivation {
    fn alt_200(&self) -> bool {
        self.alt_200
    }

    fn alt_750(&self) -> bool {
        self.alt_750
    }
}

pub(super) trait AltitudeAlertSlatInhibit {
    /// This signal indicates that the altitude alerts should be inhibited because the gear is down.
    fn slat_inhibit(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeAlertSlatInhibitActivation {
    slat_inhibit: bool,
}

impl AltitudeAlertSlatInhibitActivation {
    pub fn update(&mut self, lg_downlocked_sheet: &impl LgDownlocked) {
        let lg_downlocked = lg_downlocked_sheet.lg_downlocked();
        self.slat_inhibit = lg_downlocked; // TODO gear down selection with flap setting
    }
}

impl AltitudeAlertSlatInhibit for AltitudeAlertSlatInhibitActivation {
    fn slat_inhibit(&self) -> bool {
        self.slat_inhibit
    }
}

pub(super) trait AltitudeAlertFmgcInhibit {
    /// This signal indicates that the altitude alerts should be inhibited because a descent is
    /// expected, for example because the aircraft is following a glide slope.
    fn fmgc_inhibit(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeAlertFmgcInhibitActivation {
    fmgc_inhibit: bool,
}

impl AltitudeAlertFmgcInhibitActivation {
    pub fn update(&mut self, signals: &impl GsModeOn) {
        self.fmgc_inhibit = signals.gs_mode_on(1).value(); // TODO dual FMGC, LAND mode, FINAL DESCENT
    }
}

impl AltitudeAlertFmgcInhibit for AltitudeAlertFmgcInhibitActivation {
    fn fmgc_inhibit(&self) -> bool {
        self.fmgc_inhibit
    }
}

pub(super) trait AltitudeAlertGeneralInhibit {
    /// This signal indicates that the altitude alerts should be inhibited for example because a
    /// descent is expected (on glideslope, or gear down) or because of a system failure (invalid altitude source).
    fn general_inhibit(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeAlertGeneralInhibitActivation {
    general_inhibit: bool,
}

impl AltitudeAlertGeneralInhibitActivation {
    pub fn update(
        &mut self,
        signals: &(impl AltiSelect + AltSelectChg),
        slat_sheet: &impl AltitudeAlertSlatInhibit,
        fmgc_sheet: &impl AltitudeAlertFmgcInhibit,
    ) {
        let alti_select = signals.alti_select();
        let bad_alti_select = alti_select.is_ncd() || alti_select.is_inv();

        self.general_inhibit = bad_alti_select
            || signals.alt_select_chg().value()
            || slat_sheet.slat_inhibit()
            || fmgc_sheet.fmgc_inhibit();
        // TODO baro failures
    }
}

impl AltitudeAlertGeneralInhibit for AltitudeAlertGeneralInhibitActivation {
    fn general_inhibit(&self) -> bool {
        self.general_inhibit
    }
}

pub(super) trait AltitudeAlertApTcasInhibit {
    /// This signal indicates that the Autopilot TCAS is available and engaged.
    fn ap_tcas_mode_eng(&self) -> bool;

    /// This signal indicates that the aural altitude alert should be inhibited because the
    /// deviation was initiated by AP TCAS.
    fn alt_alert_inib(&self) -> bool;
}

pub(super) struct AltitudeAlertApTcasInhibitActivation {
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

impl Default for AltitudeAlertApTcasInhibitActivation {
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

impl AltitudeAlertApTcasInhibitActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl TcasEngaged + AltSelectChg),
        lg_downlocked_sheet: &impl LgDownlocked,
        alti_threshold_sheet: &impl AltitudeAlertThresholds,
        general_inhibit_sheet: &impl AltitudeAlertGeneralInhibit,
    ) {
        self.ap_tcas_mode_eng = signals.tcas_engaged().value(); // TODO pin programming

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

impl AltitudeAlertApTcasInhibit for AltitudeAlertApTcasInhibitActivation {
    fn ap_tcas_mode_eng(&self) -> bool {
        self.ap_tcas_mode_eng
    }

    fn alt_alert_inib(&self) -> bool {
        self.alt_alert_inib
    }
}

pub(super) trait AltitudeAlert {
    /// This signal indicates that the C. Chord (Altitude Alert) should be playing.
    fn c_chord(&self) -> bool;

    /// This signal indicates that the altitude indicator on the PFDs should be slowly flashing in
    /// yellow (slight altitude deviation).
    fn steady_light(&self) -> bool;

    /// This signal indicates that the altitude indicator on the PFDs should be rapidly flashing in
    /// amber (severe altitude deviation).
    fn flashing_light(&self) -> bool;
}

pub(super) struct AltitudeAlertActivation {
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

impl Default for AltitudeAlertActivation {
    fn default() -> Self {
        Self {
            pulse1: PulseNode::new(false),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.0)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs_f64(1.0)),
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

impl AltitudeAlertActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &impl AltSelectChg,
        gnd_sheet: &impl GroundDetection,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
        ap_tcas_alt_inhibit_sheet: &impl AltitudeAlertApTcasInhibit,
        threshold_sheet: &impl AltitudeAlertThresholds,
        inhibit_sheet: &impl AltitudeAlertGeneralInhibit,
        lg_sheet: &impl LgDownlocked,
    ) {
        let ap_tcas_mode_eng = ap_tcas_alt_inhibit_sheet.ap_tcas_mode_eng();
        let ground_or_ap_tcas = gnd_sheet.ground() || ap_tcas_mode_eng;

        let mtrig1_out = self.mtrig1.update(signals.alt_select_chg().value(), delta);
        let mtrig2_out = self.mtrig2.update(lg_sheet.lg_downlocked(), delta);
        let reset_mems = mtrig1_out || mtrig2_out;

        let alt_200 = threshold_sheet.alt_200();
        let alt_750 = threshold_sheet.alt_750();
        let general_inhibit = inhibit_sheet.general_inhibit();

        let within_200 = alt_200 && alt_750 && !general_inhibit;
        let within_750 = !alt_200 && alt_750 && !general_inhibit;
        let not_750 = !alt_200 && !alt_750 && !general_inhibit;

        let mem_was_within_200 = self
            .mem_within_200
            .update(within_200, not_750 || reset_mems);

        let mem_was_within_750 = self.mem_within_750.update(within_750, reset_mems);

        let left_200 = within_750 && mem_was_within_200;
        let left_750 = not_750 && mem_was_within_750;
        let flashing_light_cond = left_200 || left_750;

        self.flashing_light = !ground_or_ap_tcas && flashing_light_cond;
        self.steady_light = !ground_or_ap_tcas && within_750 && !left_200;

        let one_ap_engd = ap_sheet.one_ap_engd();
        let mtrig3_out = self.mtrig3.update(!one_ap_engd && within_750, delta);

        let mtrig4_in = !one_ap_engd && self.pulse1.update(ap_tcas_mode_eng) && !general_inhibit;
        let approaching_alt = self.mtrig4.update(mtrig4_in, delta);

        self.c_chord = !ground_or_ap_tcas
            && (mtrig3_out
                || approaching_alt
                || (!ap_tcas_alt_inhibit_sheet.alt_alert_inib() && flashing_light_cond));
    }
}

impl AltitudeAlert for AltitudeAlertActivation {
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

pub(super) struct AltitudeAlertCChordActivation {
    c_chord: bool,
}

impl AltitudeAlertCChordActivation {
    pub fn update(&mut self, delta: Duration, altitude_alert: &impl AltitudeAlert) {
        self.c_chord = altitude_alert.c_chord();
    }
}

impl WarningActivation for AltitudeAlertCChordActivation {
    fn warning(&self) -> bool {
        self.c_chord
    }
}

impl Default for AltitudeAlertCChordActivation {
    fn default() -> Self {
        Self { c_chord: false }
    }
}

pub(super) trait AudioGenerated {
    fn minimum_generated(&self) -> bool;

    fn hundred_above_generated(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AudioGeneratedActivation {
    minimum_generated: bool,
    hundred_above_generated: bool,
}

impl AudioGeneratedActivation {
    pub fn update(&mut self, minimum_generated: bool, hunded_above_generated: bool) {
        self.minimum_generated = minimum_generated;
        self.hundred_above_generated = hunded_above_generated;
    }
}

impl AudioGenerated for AudioGeneratedActivation {
    fn minimum_generated(&self) -> bool {
        self.minimum_generated
    }

    fn hundred_above_generated(&self) -> bool {
        self.hundred_above_generated
    }
}

pub(super) trait DecisionHeightVal {
    fn radio_height_val(&self) -> Length;

    fn decision_height_val(&self) -> Length;

    fn decision_inv(&self) -> bool;
}

/// This is the final sheet to decide that the C Chord should be played. It currently contains some
/// rudimentary cancel logic, which will be replaced with a generalized system in future.
#[derive(Default)]
pub(super) struct DecisionHeightValActivation {
    radio_height_val: Length,
    decision_height_val: Length,
    decision_inv: bool,
}

impl DecisionHeightValActivation {
    pub fn update(&mut self, signals: &(impl RadioHeight + DecisionHeight)) {
        let rh1 = signals.radio_height(1);
        let rh2 = signals.radio_height(2);
        self.radio_height_val = if rh1.is_inv() || rh1.is_ncd() {
            rh2.value()
        } else {
            rh1.value()
        };

        let dh1 = signals.decision_height(1);
        let dh2 = signals.decision_height(2);

        let dh2_chosen = (dh1.is_val()
            && !dh1.is_ncd()
            && dh2.is_val()
            && !dh2.is_ncd()
            && dh1.value() > dh2.value())
            || dh1.is_ncd()
            || dh1.is_inv();

        self.decision_height_val = if dh2_chosen { dh2.value() } else { dh1.value() };

        self.decision_inv = (dh1.is_inv() || dh1.is_ncd()) && (dh2.is_inv() || dh2.is_ncd());
    }
}

impl DecisionHeightVal for DecisionHeightValActivation {
    fn radio_height_val(&self) -> Length {
        self.radio_height_val
    }

    fn decision_height_val(&self) -> Length {
        self.decision_height_val
    }

    fn decision_inv(&self) -> bool {
        self.decision_inv
    }
}

pub(super) trait MdaMdhInbition {
    fn aco_mda_mdh_inhib(&self) -> bool;

    fn aco_dh_inhib(&self) -> bool;
}

pub(super) struct MdaMdhInbitionActivation {
    aco_mda_mdh_inhib: bool,
    aco_dh_inhib: bool,
    mrtrig: MonostableTriggerNode,
}

impl Default for MdaMdhInbitionActivation {
    fn default() -> Self {
        Self {
            aco_mda_mdh_inhib: false,
            aco_dh_inhib: false,
            mrtrig: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(5)),
        }
    }
}

impl MdaMdhInbitionActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl RadioHeight + TcasAuralAdvisaryOutput),
        gpws_sheet: &impl HoistedGpwsInhibition,
        dh_sheet: &impl DecisionHeightVal,
        aco_inhib: &impl AutomaticCallOutInhibition,
    ) {
        let stall_on = false; // TODO
        let speed_on = false; // TODO
        let tcas_output = self
            .mrtrig
            .update(signals.tcas_aural_advisory_output().value(), delta);

        self.aco_mda_mdh_inhib =
            stall_on || speed_on || gpws_sheet.gpws_inhibition() || tcas_output;

        let decision_height_inf_3ft = dh_sheet.decision_height_val() <= Length::new::<foot>(3.);
        let rh1 = signals.radio_height(1);
        let rh2 = signals.radio_height(1);
        let rh1_and_2_inv = (rh1.is_inv() || rh1.is_ncd()) && (rh2.is_inv() || rh2.is_ncd());

        self.aco_dh_inhib = decision_height_inf_3ft
            || dh_sheet.decision_inv()
            || aco_inhib.auto_call_out_inhib()
            || rh1_and_2_inv;
    }
}

impl MdaMdhInbition for MdaMdhInbitionActivation {
    fn aco_mda_mdh_inhib(&self) -> bool {
        self.aco_mda_mdh_inhib
    }

    fn aco_dh_inhib(&self) -> bool {
        self.aco_dh_inhib
    }
}

pub(super) trait HundredAbove {
    fn dh_hundred_above(&self) -> bool;

    fn ha_generated(&self) -> bool;
}

pub(super) struct HundredAboveActivation {
    dh_hundred_above: bool,
    ha_generated: bool,
    conf1: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mem_dh_generated: MemoryNode,
    mem_mda_mdh_generated: MemoryNode,
}

impl Default for HundredAboveActivation {
    fn default() -> Self {
        Self {
            dh_hundred_above: false,
            ha_generated: false,
            conf1: ConfirmationNode::new(true, Duration::from_secs_f64(0.1)),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs(3)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs(3)),
            mem_dh_generated: MemoryNode::new(false),
            mem_mda_mdh_generated: MemoryNode::new(false),
        }
    }
}

impl HundredAboveActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl AutoCalloutPins + HundredAboveForMdaMdhRequest),
        audio_sheet: &impl AudioGenerated,
        dh_sheet: &impl DecisionHeightVal,
        mda_mdh_inhib_sheet: &impl MdaMdhInbition,
    ) {
        let hundred_above_generated = audio_sheet.hundred_above_generated();
        let pin_programmed = signals.decision_height_plus_100_ft_code_a().value()
            && signals.decision_height_plus_100_ft_code_b().value();

        let dh_inhib = mda_mdh_inhib_sheet.aco_dh_inhib();
        let mda_mdh_inhib = mda_mdh_inhib_sheet.aco_mda_mdh_inhib();

        // Decision Height (compared to Radio Altimeter)
        let rh = dh_sheet.radio_height_val();
        let dh = dh_sheet.decision_height_val();
        let dh_inf_90ft = dh < Length::new::<foot>(90.);

        let below_100_above = if dh_inf_90ft {
            rh < dh + Length::new::<foot>(105.)
        } else {
            rh < dh + Length::new::<foot>(115.)
        };
        let mtrig1_out = self
            .mtrig1
            .update(self.conf1.update(below_100_above, delta), delta);

        let mem_dh_out = self
            .mem_dh_generated
            .update(hundred_above_generated, !mtrig1_out);

        let dh_cond = !mem_dh_out && mtrig1_out && pin_programmed && !dh_inhib;

        // Minimum Descent Altitude / Height (triggered by DMC discrete)
        let capt_mda_mdh_raw = signals.hundred_above_for_mda_mdh_request(1);
        let capt_mda_mdh = capt_mda_mdh_raw.value()
            && capt_mda_mdh_raw.is_val()
            && !capt_mda_mdh_raw.is_ncd()
            && !capt_mda_mdh_raw.is_ft();

        let fo_mda_mdh_raw = signals.hundred_above_for_mda_mdh_request(2);
        let fo_mda_mdh = fo_mda_mdh_raw.value()
            && fo_mda_mdh_raw.is_val()
            && !fo_mda_mdh_raw.is_ncd()
            && !fo_mda_mdh_raw.is_ft();

        let mtrig2_out = self.mtrig2.update(capt_mda_mdh || fo_mda_mdh, delta);

        let mem_mda_mdh_out = self
            .mem_mda_mdh_generated
            .update(hundred_above_generated, !mtrig2_out);

        let mda_mdh_cond = !mem_mda_mdh_out && mtrig2_out && pin_programmed && !mda_mdh_inhib;

        println!(
            "dh={} hundred_above_generated={} pin_programmed={} below_100_above={} mtrig1_out={} mem_dh_out={} dh_inhib={} dh_cond={}",
            dh.get::<foot>(), hundred_above_generated, pin_programmed, below_100_above, mtrig1_out, mem_dh_out, dh_inhib, dh_cond
        );

        // Outputs

        self.ha_generated = dh_cond || mda_mdh_cond;
        self.dh_hundred_above = self.ha_generated || hundred_above_generated;
    }
}

impl HundredAbove for HundredAboveActivation {
    fn dh_hundred_above(&self) -> bool {
        self.dh_hundred_above
    }

    fn ha_generated(&self) -> bool {
        self.ha_generated
    }
}

impl WarningActivation for HundredAboveActivation {
    fn warning(&self) -> bool {
        self.ha_generated
    }
}

pub(super) trait Minimum {
    fn dh_generated(&self) -> bool;
}

pub(super) struct MinimumActivation {
    dh_generated: bool,
    dh_generated_discrete: bool,
    conf1: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    mem_dh_generated: MemoryNode,
    mem_mda_mdh_generated: MemoryNode,
}

impl Default for MinimumActivation {
    fn default() -> Self {
        Self {
            dh_generated: false,
            dh_generated_discrete: false,
            conf1: ConfirmationNode::new(true, Duration::from_secs_f64(0.1)),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs(3)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs(3)),
            mem_dh_generated: MemoryNode::new(false),
            mem_mda_mdh_generated: MemoryNode::new(false),
        }
    }
}

impl MinimumActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl AutoCalloutPins + MinimumForMdaMdhRequest),
        ha_sheet: &impl HundredAbove,
        audio_sheet: &impl AudioGenerated,
        dh_sheet: &impl DecisionHeightVal,
        mda_mdh_inhib_sheet: &impl MdaMdhInbition,
    ) {
        let minimum_generated = audio_sheet.minimum_generated();
        let pin_programmed =
            signals.decision_height_code_a().value() && signals.decision_height_code_b().value();

        let dh_inhib = mda_mdh_inhib_sheet.aco_dh_inhib();
        let mda_mdh_inhib = mda_mdh_inhib_sheet.aco_mda_mdh_inhib();

        // Decision Height (compared to Radio Altimeter)
        let rh = dh_sheet.radio_height_val();
        let dh = dh_sheet.decision_height_val();
        let dh_inf_90ft = dh < Length::new::<foot>(90.);

        let below_100_above = if dh_inf_90ft {
            rh < dh + Length::new::<foot>(5.)
        } else {
            rh < dh + Length::new::<foot>(15.)
        };
        let mtrig1_out = self
            .mtrig1
            .update(self.conf1.update(below_100_above, delta), delta);

        let mem_dh_out = self.mem_dh_generated.update(minimum_generated, !mtrig1_out);

        let dh_cond = !mem_dh_out && mtrig1_out && pin_programmed && !dh_inhib;

        // Minimum Descent Altitude / Height (triggered by DMC discrete)
        let capt_mda_mdh_raw = signals.minimum_for_mda_mdh_request(1);
        let capt_mda_mdh = capt_mda_mdh_raw.value()
            && capt_mda_mdh_raw.is_val()
            && !capt_mda_mdh_raw.is_ncd()
            && !capt_mda_mdh_raw.is_ft();

        let fo_mda_mdh_raw = signals.minimum_for_mda_mdh_request(2);
        let fo_mda_mdh = fo_mda_mdh_raw.value()
            && fo_mda_mdh_raw.is_val()
            && !fo_mda_mdh_raw.is_ncd()
            && !fo_mda_mdh_raw.is_ft();

        let mtrig2_out = self.mtrig2.update(capt_mda_mdh || fo_mda_mdh, delta);

        let mem_mda_mdh_out = self
            .mem_mda_mdh_generated
            .update(minimum_generated, !mtrig2_out);

        let mda_mdh_cond = !mem_mda_mdh_out && mtrig2_out && pin_programmed && !mda_mdh_inhib;

        // Outputs

        self.dh_generated_discrete = dh_cond || mda_mdh_cond;
        self.dh_generated = self.dh_generated || minimum_generated || ha_sheet.dh_hundred_above();
    }
}

impl Minimum for MinimumActivation {
    fn dh_generated(&self) -> bool {
        self.dh_generated
    }
}

impl WarningActivation for MinimumActivation {
    fn warning(&self) -> bool {
        self.dh_generated_discrete
    }
}

pub(super) trait AltitudeThreshold1 {
    fn ra_1_inv(&self) -> bool;
    fn alt_sup_50_ft(&self) -> bool;
    fn alt_sup_410_ft(&self) -> bool;
    fn alt_400_ft(&self) -> bool;
    fn alt_300_ft(&self) -> bool;
    fn alt_200_ft(&self) -> bool;
    fn alt_100_ft(&self) -> bool;
    fn alt_50_ft(&self) -> bool;
    fn radio_height(&self) -> Length;
    fn ra_invalid(&self) -> bool;
    fn ra_fonctionnal_test(&self) -> bool;
    fn ra_no_computed_data(&self) -> bool;
}

pub(super) struct AltitudeThreshold1Activation {
    ra_1_inv: bool,
    alt_sup_50_ft: bool,
    alt_sup_410_ft: bool,
    alt_400_ft: bool,
    alt_300_ft: bool,
    alt_200_ft: bool,
    alt_100_ft: bool,
    alt_50_ft: bool,
    radio_height: Length,
    ra_invalid: bool,
    ra_fonctionnal_test: bool,
    ra_no_computed_data: bool,
}

impl Default for AltitudeThreshold1Activation {
    fn default() -> Self {
        Self {
            ra_1_inv: false,
            alt_sup_50_ft: false,
            alt_sup_410_ft: false,
            alt_400_ft: false,
            alt_300_ft: false,
            alt_200_ft: false,
            alt_100_ft: false,
            alt_50_ft: false,
            radio_height: Length::new::<foot>(0.),
            ra_invalid: false,
            ra_fonctionnal_test: false,
            ra_no_computed_data: false,
        }
    }
}

impl AltitudeThreshold1Activation {
    pub fn update(&mut self, signals: &impl RadioHeight) {
        let rh1 = signals.radio_height(1);
        let rh2 = signals.radio_height(2);
        let rh1_ncd_or_inv = rh1.is_ncd() || rh1.is_inv();
        let rh_param = if rh1_ncd_or_inv { rh2 } else { rh1 };
        let rh = rh_param.value();

        self.ra_1_inv = rh1_ncd_or_inv;
        self.alt_sup_50_ft = rh > Length::new::<foot>(50.);
        self.alt_sup_410_ft = rh >= Length::new::<foot>(410.);
        self.alt_400_ft = Length::new::<foot>(400.) <= rh && rh < Length::new::<foot>(410.);
        self.alt_300_ft = Length::new::<foot>(300.) <= rh && rh < Length::new::<foot>(310.);
        self.alt_200_ft = Length::new::<foot>(200.) <= rh && rh < Length::new::<foot>(210.);
        self.alt_100_ft = Length::new::<foot>(100.) <= rh && rh < Length::new::<foot>(110.);
        self.alt_50_ft = Length::new::<foot>(50.) <= rh && rh < Length::new::<foot>(53.);
        self.radio_height = rh;
        self.ra_invalid = rh1_ncd_or_inv && rh2.is_inv();
        self.ra_fonctionnal_test = if rh1_ncd_or_inv {
            rh2.is_ft()
        } else {
            rh1.is_ft()
        };
        self.ra_no_computed_data = if rh1_ncd_or_inv {
            rh2.is_ncd()
        } else {
            rh1.is_ncd()
        };
    }
}

impl AltitudeThreshold1 for AltitudeThreshold1Activation {
    fn ra_1_inv(&self) -> bool {
        self.ra_1_inv
    }

    fn alt_sup_50_ft(&self) -> bool {
        self.alt_sup_50_ft
    }

    fn alt_sup_410_ft(&self) -> bool {
        self.alt_sup_410_ft
    }

    fn alt_400_ft(&self) -> bool {
        self.alt_400_ft
    }

    fn alt_300_ft(&self) -> bool {
        self.alt_300_ft
    }

    fn alt_200_ft(&self) -> bool {
        self.alt_200_ft
    }

    fn alt_100_ft(&self) -> bool {
        self.alt_100_ft
    }

    fn alt_50_ft(&self) -> bool {
        self.alt_50_ft
    }

    fn radio_height(&self) -> Length {
        self.radio_height
    }

    fn ra_invalid(&self) -> bool {
        self.ra_invalid
    }

    fn ra_fonctionnal_test(&self) -> bool {
        self.ra_fonctionnal_test
    }

    fn ra_no_computed_data(&self) -> bool {
        self.ra_no_computed_data
    }
}

pub(super) trait AltitudeThreshold2 {
    fn alt_40_ft(&self) -> bool;
    fn alt_30_ft(&self) -> bool;
    fn alt_20_ft(&self) -> bool;
    fn alt_10_ft(&self) -> bool;
    fn alt_5_ft(&self) -> bool;
    fn alt_inf_3_ft(&self) -> bool;
    fn dh_inhibition(&self) -> bool;
}

pub(super) struct AltitudeThreshold2Activation {
    last_rh: Length,
    conf_node: ConfirmationNode,
    alt_40_ft: bool,
    alt_30_ft: bool,
    alt_20_ft: bool,
    alt_inf_20_ft: bool,
    alt_10_ft: bool,
    alt_inf_10_ft: bool,
    alt_5_ft: bool,
    alt_inf_3_ft: bool,
    dh_inhibition: bool,
}

impl Default for AltitudeThreshold2Activation {
    fn default() -> Self {
        Self {
            last_rh: Length::new::<foot>(0.0),
            conf_node: ConfirmationNode::new(false, Duration::from_secs_f64(0.3)),
            alt_40_ft: false,
            alt_30_ft: false,
            alt_20_ft: false,
            alt_inf_20_ft: false,
            alt_10_ft: false,
            alt_inf_10_ft: false,
            alt_5_ft: false,
            alt_inf_3_ft: false,
            dh_inhibition: false,
        }
    }
}

impl AltitudeThreshold2Activation {
    pub fn update(&mut self, delta: Duration, threshold_sheet: &impl AltitudeThreshold1) {
        let rh = threshold_sheet.radio_height();
        let ra_invalid = threshold_sheet.ra_invalid();
        self.alt_40_ft = Length::new::<foot>(40.) <= rh && rh < Length::new::<foot>(42.);
        self.alt_30_ft = Length::new::<foot>(30.) <= rh && rh < Length::new::<foot>(32.);
        self.alt_20_ft = Length::new::<foot>(20.) <= rh && rh < Length::new::<foot>(22.);
        self.alt_inf_20_ft =
            !ra_invalid && Length::new::<foot>(-5.) <= rh && rh < Length::new::<foot>(22.);
        self.alt_10_ft = Length::new::<foot>(10.) <= rh && rh < Length::new::<foot>(12.);
        self.alt_inf_10_ft =
            !ra_invalid && Length::new::<foot>(-5.) <= rh && rh < Length::new::<foot>(12.);
        self.alt_5_ft = Length::new::<foot>(5.) <= rh && rh < Length::new::<foot>(6.);
        self.alt_inf_3_ft = !ra_invalid && rh <= Length::new::<foot>(3.);
        let derivative = rh - self.last_rh; // while we should be dividing by delta, we don't really mind as we simply check for > 0
        self.last_rh = rh;
        self.dh_inhibition = self
            .conf_node
            .update(derivative > Length::new::<foot>(0.0), delta);
    }
}

impl AltitudeThreshold2 for AltitudeThreshold2Activation {
    fn alt_40_ft(&self) -> bool {
        self.alt_40_ft
    }

    fn alt_30_ft(&self) -> bool {
        self.alt_30_ft
    }

    fn alt_20_ft(&self) -> bool {
        self.alt_20_ft
    }

    fn alt_10_ft(&self) -> bool {
        self.alt_10_ft
    }

    fn alt_5_ft(&self) -> bool {
        self.alt_5_ft
    }

    fn alt_inf_3_ft(&self) -> bool {
        self.alt_inf_3_ft
    }

    fn dh_inhibition(&self) -> bool {
        self.dh_inhibition
    }
}

pub(super) trait HoistedGpwsInhibition {
    fn gpws_inhibition(&self) -> bool;
}

pub(super) struct HoistedGpwsInhibitionActivation {
    mtrig: MonostableTriggerNode,
    gpws_inhibition: bool,
}

impl Default for HoistedGpwsInhibitionActivation {
    fn default() -> Self {
        Self {
            gpws_inhibition: false,
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
        }
    }
}

impl HoistedGpwsInhibitionActivation {
    pub fn update(&mut self, delta: Duration, signals: &(impl GpwsModesOn + GsVisualAlertOn)) {
        let any_gpws = signals.gpws_modes_on().value() || signals.gs_visual_alert_on().value();
        self.gpws_inhibition = self.mtrig.update(any_gpws, delta);
    }
}

impl HoistedGpwsInhibition for HoistedGpwsInhibitionActivation {
    fn gpws_inhibition(&self) -> bool {
        self.gpws_inhibition
    }
}

pub(super) trait AltitudeThreshold3 {
    fn threshold_detection(&self) -> bool;
    fn gpws_inhibition(&self) -> bool;
    fn to_and_ground_detection(&self) -> bool;
    fn renvoi1(&self) -> bool;
    fn renvoi2(&self) -> bool;
    fn renvoi3(&self) -> bool;
}

pub(super) struct AltitudeThreshold3Activation {
    threshold_detection: bool,
    gpws_inhibition: bool,
    to_and_ground_detection: bool,
    renvoi1: bool,
    renvoi2: bool,
    renvoi3: bool,
}

impl Default for AltitudeThreshold3Activation {
    fn default() -> Self {
        Self {
            threshold_detection: false,
            gpws_inhibition: false,
            to_and_ground_detection: false,
            renvoi1: false,
            renvoi2: false,
            renvoi3: false,
        }
    }
}

impl AltitudeThreshold3Activation {
    pub fn update(
        &mut self,
        gpws_inhib_sheet: &impl HoistedGpwsInhibition,
        dh_dt_positive: &impl GeneralDhDtPositive,
        altitude_threshold1_sheet: &impl AltitudeThreshold1,
        altitude_threshold2_sheet: &impl AltitudeThreshold2,
        minimum_sheet: &impl Minimum,
    ) {
        let alt_400_ft = altitude_threshold1_sheet.alt_400_ft();
        let alt_300_ft = altitude_threshold1_sheet.alt_300_ft();
        let alt_200_ft = altitude_threshold1_sheet.alt_200_ft();
        let alt_100_ft = altitude_threshold1_sheet.alt_100_ft();
        let alt_50_ft = altitude_threshold1_sheet.alt_50_ft();
        let alt_40_ft = altitude_threshold2_sheet.alt_40_ft();
        let alt_30_ft = altitude_threshold2_sheet.alt_30_ft();
        let alt_20_ft = altitude_threshold2_sheet.alt_20_ft();
        let alt_10_ft = altitude_threshold2_sheet.alt_10_ft();
        let alt_5_ft = altitude_threshold2_sheet.alt_5_ft();
        let alt_inf_3_ft = altitude_threshold2_sheet.alt_inf_3_ft();
        let dh_positive = dh_dt_positive.dh_positive();
        let dh_generated = minimum_sheet.dh_generated();
        let dh_inhibition = altitude_threshold2_sheet.dh_inhibition();

        self.threshold_detection = alt_400_ft
            || alt_300_ft
            || alt_200_ft
            || alt_100_ft
            || alt_50_ft
            || alt_40_ft
            || alt_30_ft
            || alt_20_ft
            || alt_10_ft
            || alt_5_ft;
        self.gpws_inhibition = gpws_inhib_sheet.gpws_inhibition();
        self.to_and_ground_detection = alt_inf_3_ft || dh_positive;
        self.renvoi1 = self.to_and_ground_detection || self.gpws_inhibition || dh_generated;
        self.renvoi2 = self.to_and_ground_detection || dh_generated;
        self.renvoi3 = self.to_and_ground_detection || alt_inf_3_ft || dh_inhibition;
    }
}

impl AltitudeThreshold3 for AltitudeThreshold3Activation {
    fn threshold_detection(&self) -> bool {
        self.threshold_detection
    }

    fn gpws_inhibition(&self) -> bool {
        self.gpws_inhibition
    }

    fn to_and_ground_detection(&self) -> bool {
        self.to_and_ground_detection
    }

    fn renvoi1(&self) -> bool {
        self.renvoi1
    }

    fn renvoi2(&self) -> bool {
        self.renvoi2
    }

    fn renvoi3(&self) -> bool {
        self.renvoi3
    }
}

pub(super) trait AutomaticCallOutInhibition {
    fn auto_call_out_inhib(&self) -> bool;
    fn retard_inhib(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AutomaticCallOutInhibitionActivation {
    auto_call_out_inhib: bool,
    retard_inhib: bool,
}

impl AutomaticCallOutInhibitionActivation {
    pub fn update(
        &mut self,
        signals: &(impl EssLhLgCompressed + NormLhLgCompressed),
        threshold1_sheet: &impl AltitudeThreshold1,
        ground_sheet: &impl GroundDetection,
        cfm_flex_sheet: &impl CfmFlightPhasesDef,
        flight_phases_ground_sheet: &impl FlightPhasesGround,
        eng1_start_sheet: &impl Eng1StartSequence,
        eng2_start_sheet: &impl Eng2StartSequence,
    ) {
        let ground_test = threshold1_sheet.ra_fonctionnal_test()
            && (signals.ess_lh_lg_compressed().value() || signals.norm_lh_lg_compressed().value());

        let stall_on = false; // TODO
        let ra_invalid = threshold1_sheet.ra_invalid();
        let ra_ncd = threshold1_sheet.ra_no_computed_data();
        let cfm_flex = cfm_flex_sheet.cfm_flex();
        let iae_flex = false; // TODO
        let speed_on = false; // TODO

        let main_inhibit = stall_on || ra_invalid || ra_ncd || cfm_flex || iae_flex || speed_on;

        let eng_1_master_lever_on = eng1_start_sheet.eng_1_tempo_master_lever_1_on();
        let eng_2_master_lever_on = eng2_start_sheet.eng_2_tempo_master_lever_1_on();
        let ground = ground_sheet.ground();
        let phase8 = flight_phases_ground_sheet.phase_8();

        let auto_call_inhib_ground = eng_1_master_lever_on && eng_2_master_lever_on && ground;
        let retard_inhib_ground =
            eng_1_master_lever_on && eng_2_master_lever_on && (ground && !phase8);

        self.auto_call_out_inhib = (main_inhibit || auto_call_inhib_ground) && !ground_test;
        self.retard_inhib = (main_inhibit || retard_inhib_ground) && !ground_test;
    }
}

impl AutomaticCallOutInhibition for AutomaticCallOutInhibitionActivation {
    fn auto_call_out_inhib(&self) -> bool {
        self.auto_call_out_inhib
    }

    fn retard_inhib(&self) -> bool {
        self.retard_inhib
    }
}

pub(super) trait AltitudeThresholdTriggers1 {
    fn seuil_2500_ft(&self) -> bool;
    fn seuil_2500b_ft(&self) -> bool;
    fn seuil_2000_ft(&self) -> bool;
    fn seuil_1000_ft(&self) -> bool;
    fn seuil_500_ft(&self) -> bool;
}

pub(super) struct AltitudeThresholdTriggers1Activation {
    seuil_2500_ft: bool,
    seuil_2500b_ft: bool,
    seuil_2000_ft: bool,
    seuil_1000_ft: bool,
    seuil_500_ft: bool,
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    conf3: ConfirmationNode,
    conf4: ConfirmationNode,
    tcas_mtrig: MonostableTriggerNode,
}

impl Default for AltitudeThresholdTriggers1Activation {
    fn default() -> Self {
        Self {
            conf1: ConfirmationNode::new(true, Duration::from_secs_f64(0.2)),
            conf2: ConfirmationNode::new(true, Duration::from_secs_f64(0.2)),
            conf3: ConfirmationNode::new(true, Duration::from_secs_f64(0.2)),
            conf4: ConfirmationNode::new(true, Duration::from_secs_f64(0.2)),
            tcas_mtrig: MonostableTriggerNode::new(true, Duration::from_secs(5)),
            seuil_2500_ft: false,
            seuil_2500b_ft: false,
            seuil_2000_ft: false,
            seuil_1000_ft: false,
            seuil_500_ft: false,
        }
    }
}

impl AltitudeThresholdTriggers1Activation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl AutoCalloutPins + TcasAuralAdvisaryOutput),
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_threshold_3_sheet: &impl AltitudeThreshold3,
    ) {
        let rh = altitude_callout_threshold_1_sheet.radio_height();
        let gpws_inhibiton = altitude_callout_threshold_3_sheet.gpws_inhibition();
        let tcas_inhibit = self
            .tcas_mtrig
            .update(signals.tcas_aural_advisory_output().value(), delta);
        let gpws_or_tcas_inhibit = gpws_inhibiton || tcas_inhibit;
        let renvoi1 = altitude_callout_threshold_3_sheet.renvoi1();
        let lower_inhibit = renvoi1 || tcas_inhibit;

        let pin_2500_ft = signals.auto_call_out_2500_ft().value();
        let pin_2500b = signals.auto_call_out_2500b().value();
        let pin_2000_ft = signals.auto_call_out_2000_ft().value();
        let pin_1000_ft = signals.auto_call_out_1000_ft().value();

        let cond_2500_ft = self.conf1.update(
            Length::new::<foot>(2500.) <= rh && rh < Length::new::<foot>(2530.),
            delta,
        );
        let cond_2000_ft = self.conf2.update(
            Length::new::<foot>(2000.) <= rh && rh < Length::new::<foot>(2020.),
            delta,
        );
        let cond_1000_ft = self.conf3.update(
            Length::new::<foot>(1000.) <= rh && rh < Length::new::<foot>(1020.),
            delta,
        );
        let cond_500_ft = self.conf4.update(
            Length::new::<foot>(500.) <= rh && rh < Length::new::<foot>(513.),
            delta,
        );

        self.seuil_2500_ft = pin_2500_ft && cond_2500_ft && !gpws_or_tcas_inhibit;
        self.seuil_2500b_ft = pin_2500b && cond_2500_ft && !gpws_or_tcas_inhibit;
        self.seuil_2000_ft = pin_2000_ft && cond_2000_ft && !gpws_or_tcas_inhibit;
        self.seuil_1000_ft = pin_1000_ft && cond_1000_ft && !lower_inhibit;
        self.seuil_500_ft = cond_500_ft && !lower_inhibit;
    }
}

impl AltitudeThresholdTriggers1 for AltitudeThresholdTriggers1Activation {
    fn seuil_2500_ft(&self) -> bool {
        self.seuil_2500_ft
    }

    fn seuil_2500b_ft(&self) -> bool {
        self.seuil_2500b_ft
    }

    fn seuil_2000_ft(&self) -> bool {
        self.seuil_2000_ft
    }

    fn seuil_1000_ft(&self) -> bool {
        self.seuil_1000_ft
    }

    fn seuil_500_ft(&self) -> bool {
        self.seuil_500_ft
    }
}

pub(super) trait AltitudeThresholdTriggers2 {
    fn seuil_400_ft(&self) -> bool;
    fn seuil_300_ft(&self) -> bool;
    fn seuil_200_ft(&self) -> bool;
    fn seuil_100_ft(&self) -> bool;
    fn seuil_50_ft(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeThresholdTriggers2Activation {
    seuil_400_ft: bool,
    seuil_300_ft: bool,
    seuil_200_ft: bool,
    seuil_100_ft: bool,
    seuil_50_ft: bool,
}

impl AltitudeThresholdTriggers2Activation {
    pub fn update(
        &mut self,
        signals: &impl AutoCalloutPins,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_threshold_3_sheet: &impl AltitudeThreshold3,
    ) {
        let renvoi1 = altitude_callout_threshold_3_sheet.renvoi1();
        let pin_400_ft = signals.auto_call_out_400_ft().value();
        let pin_300_ft = signals.auto_call_out_300_ft().value();
        let pin_200_ft = signals.auto_call_out_200_ft().value();
        let pin_100_ft = signals.auto_call_out_100_ft().value();
        let pin_50_ft = signals.auto_call_out_50_ft().value();

        let cond_400_ft = altitude_callout_threshold_1_sheet.alt_400_ft();
        let cond_300_ft = altitude_callout_threshold_1_sheet.alt_300_ft();
        let cond_200_ft = altitude_callout_threshold_1_sheet.alt_200_ft();
        let cond_100_ft = altitude_callout_threshold_1_sheet.alt_100_ft();
        let cond_50_ft = altitude_callout_threshold_1_sheet.alt_50_ft();

        self.seuil_400_ft = pin_400_ft && cond_400_ft && !renvoi1;
        self.seuil_300_ft = pin_300_ft && cond_300_ft && !renvoi1;
        self.seuil_200_ft = pin_200_ft && cond_200_ft && !renvoi1;
        self.seuil_100_ft = pin_100_ft && cond_100_ft && !renvoi1;
        self.seuil_50_ft = pin_50_ft && cond_50_ft && !renvoi1;
    }
}

impl AltitudeThresholdTriggers2 for AltitudeThresholdTriggers2Activation {
    fn seuil_400_ft(&self) -> bool {
        self.seuil_400_ft
    }

    fn seuil_300_ft(&self) -> bool {
        self.seuil_300_ft
    }

    fn seuil_200_ft(&self) -> bool {
        self.seuil_200_ft
    }

    fn seuil_100_ft(&self) -> bool {
        self.seuil_100_ft
    }

    fn seuil_50_ft(&self) -> bool {
        self.seuil_50_ft
    }
}

pub(super) trait AltitudeThresholdTriggers3 {
    fn seuil_40_ft(&self) -> bool;
    fn seuil_30_ft(&self) -> bool;
    fn seuil_20_ft(&self) -> bool;
    fn seuil_10_ft(&self) -> bool;
    fn seuil_5_ft(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeThresholdTriggers3Activation {
    seuil_40_ft: bool,
    seuil_30_ft: bool,
    seuil_20_ft: bool,
    seuil_10_ft: bool,
    seuil_5_ft: bool,
}

impl AltitudeThresholdTriggers3Activation {
    pub fn update(
        &mut self,
        signals: &impl AutoCalloutPins,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_threshold_2_sheet: &impl AltitudeThreshold2,
        altitude_callout_threshold_3_sheet: &impl AltitudeThreshold3,
    ) {
        let ra_ft = altitude_callout_threshold_1_sheet.ra_fonctionnal_test();
        let renvoi2 = altitude_callout_threshold_3_sheet.renvoi2();
        let renvoi3 = altitude_callout_threshold_3_sheet.renvoi3();

        let pin_40_ft = signals.auto_call_out_40_ft().value();
        let pin_30_ft = signals.auto_call_out_30_ft().value();
        let pin_20_ft = signals.auto_call_out_20_ft().value();
        let pin_10_ft = signals.auto_call_out_10_ft().value();
        let pin_5_ft = signals.auto_call_out_5_ft().value();

        let cond_40_ft = altitude_callout_threshold_2_sheet.alt_40_ft();
        let cond_30_ft = altitude_callout_threshold_2_sheet.alt_30_ft();
        let cond_20_ft = altitude_callout_threshold_2_sheet.alt_20_ft();
        let cond_10_ft = altitude_callout_threshold_2_sheet.alt_10_ft();
        let cond_5_ft = altitude_callout_threshold_2_sheet.alt_5_ft();

        self.seuil_40_ft = cond_40_ft && (ra_ft || (!renvoi2 && pin_40_ft));
        self.seuil_30_ft = cond_30_ft && pin_30_ft && !renvoi2;
        self.seuil_20_ft = cond_20_ft && pin_20_ft && !renvoi2;
        self.seuil_10_ft = cond_10_ft && pin_10_ft && !renvoi3;
        self.seuil_5_ft = cond_5_ft && pin_5_ft && !renvoi3;
    }
}

impl AltitudeThresholdTriggers3 for AltitudeThresholdTriggers3Activation {
    fn seuil_40_ft(&self) -> bool {
        self.seuil_40_ft
    }

    fn seuil_30_ft(&self) -> bool {
        self.seuil_30_ft
    }

    fn seuil_20_ft(&self) -> bool {
        self.seuil_20_ft
    }

    fn seuil_10_ft(&self) -> bool {
        self.seuil_10_ft
    }

    fn seuil_5_ft(&self) -> bool {
        self.seuil_5_ft
    }
}

pub(super) struct AltitudeCallout2500FtAnnounceActivation {
    hysteresis: HysteresisNode<Length>,
    mem: MemoryNode,
    active_pulse: PulseNode,
    prec_node: PreceedingValueNode,
    reset_pulse: PulseNode,
    two_thd_five_hd: bool,
}

impl Default for AltitudeCallout2500FtAnnounceActivation {
    fn default() -> Self {
        Self {
            hysteresis: HysteresisNode::new(Length::new::<foot>(2500.), Length::new::<foot>(3000.)),
            mem: MemoryNode::new(false),
            prec_node: Default::default(),
            reset_pulse: PulseNode::new(false),
            active_pulse: PulseNode::new(true),
            two_thd_five_hd: false,
        }
    }
}

impl AltitudeCallout2500FtAnnounceActivation {
    pub fn update(
        &mut self,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers1,
    ) {
        let rh = altitude_callout_threshold_1_sheet.radio_height();
        let seuil_2500_ft = altitude_trigger_sheet.seuil_2500_ft();
        let auto_call_out_inhib = altitude_callout_inhib_sheet.auto_call_out_inhib();

        let hysteresis_out = self.hysteresis.update(rh);
        let reset_pulse_out = self.reset_pulse.update(hysteresis_out);

        let mem_out = self.mem.update(self.prec_node.value(), reset_pulse_out);

        let active = hysteresis_out && seuil_2500_ft && !auto_call_out_inhib && !mem_out;

        let active_pulse_out = self.active_pulse.update(active);
        self.prec_node.update(active_pulse_out);

        self.two_thd_five_hd = active;
    }
}

impl WarningActivation for AltitudeCallout2500FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.two_thd_five_hd
    }
}

pub(super) struct AltitudeCallout2500BFtAnnounceActivation {
    hysteresis: HysteresisNode<Length>,
    mem: MemoryNode,
    active_pulse: PulseNode,
    prec_node: PreceedingValueNode,
    reset_pulse: PulseNode,
    twenty_five_hundred: bool,
}

impl Default for AltitudeCallout2500BFtAnnounceActivation {
    fn default() -> Self {
        Self {
            hysteresis: HysteresisNode::new(Length::new::<foot>(2500.), Length::new::<foot>(3000.)),
            mem: MemoryNode::new(false),
            prec_node: Default::default(),
            reset_pulse: PulseNode::new(false),
            active_pulse: PulseNode::new(true),
            twenty_five_hundred: false,
        }
    }
}

impl AltitudeCallout2500BFtAnnounceActivation {
    pub fn update(
        &mut self,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers1,
    ) {
        let rh = altitude_callout_threshold_1_sheet.radio_height();
        let seuil_2500b_ft = altitude_trigger_sheet.seuil_2500_ft();
        let auto_call_out_inhib = altitude_callout_inhib_sheet.auto_call_out_inhib();

        let hysteresis_out = self.hysteresis.update(rh);
        let reset_pulse_out = self.reset_pulse.update(hysteresis_out);

        let mem_out = self.mem.update(self.prec_node.value(), reset_pulse_out);

        let active = hysteresis_out && seuil_2500b_ft && !auto_call_out_inhib && !mem_out;

        let active_pulse_out = self.active_pulse.update(active);
        self.prec_node.update(active_pulse_out);

        self.twenty_five_hundred = active;
    }
}

impl WarningActivation for AltitudeCallout2500BFtAnnounceActivation {
    fn warning(&self) -> bool {
        self.twenty_five_hundred
    }
}

pub(super) struct AltitudeCallout2000FtAnnounceActivation {
    hysteresis: HysteresisNode<Length>,
    mem: MemoryNode,
    active_pulse: PulseNode,
    prec_node: PreceedingValueNode,
    reset_pulse: PulseNode,
    two_thousand: bool,
}

impl Default for AltitudeCallout2000FtAnnounceActivation {
    fn default() -> Self {
        Self {
            hysteresis: HysteresisNode::new(Length::new::<foot>(2000.), Length::new::<foot>(2400.)),
            mem: MemoryNode::new(false),
            prec_node: Default::default(),
            reset_pulse: PulseNode::new(false),
            active_pulse: PulseNode::new(true),
            two_thousand: false,
        }
    }
}

impl AltitudeCallout2000FtAnnounceActivation {
    pub fn update(
        &mut self,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers1,
    ) {
        let rh = altitude_callout_threshold_1_sheet.radio_height();
        let seuil_2000_ft = altitude_trigger_sheet.seuil_2000_ft();
        let auto_call_out_inhib = altitude_callout_inhib_sheet.auto_call_out_inhib();

        let hysteresis_out = self.hysteresis.update(rh);
        let reset_pulse_out = self.reset_pulse.update(hysteresis_out);

        let mem_out = self.mem.update(self.prec_node.value(), reset_pulse_out);

        let active = hysteresis_out && seuil_2000_ft && !auto_call_out_inhib && !mem_out;

        let active_pulse_out = self.active_pulse.update(active);
        self.prec_node.update(active_pulse_out);

        self.two_thousand = active;
    }
}

impl WarningActivation for AltitudeCallout2000FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.two_thousand
    }
}

pub(super) struct AltitudeCallout1000FtAnnounceActivation {
    hysteresis: HysteresisNode<Length>,
    mem: MemoryNode,
    active_pulse: PulseNode,
    prec_node: PreceedingValueNode,
    reset_pulse: PulseNode,
    one_thousand: bool,
}

impl Default for AltitudeCallout1000FtAnnounceActivation {
    fn default() -> Self {
        Self {
            hysteresis: HysteresisNode::new(Length::new::<foot>(1000.), Length::new::<foot>(1100.)),
            mem: MemoryNode::new(false),
            prec_node: Default::default(),
            reset_pulse: PulseNode::new(false),
            active_pulse: PulseNode::new(true),
            one_thousand: false,
        }
    }
}

impl AltitudeCallout1000FtAnnounceActivation {
    pub fn update(
        &mut self,
        altitude_callout_threshold_1_sheet: &impl AltitudeThreshold1,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers1,
    ) {
        let rh = altitude_callout_threshold_1_sheet.radio_height();
        let seuil_1000_ft = altitude_trigger_sheet.seuil_1000_ft();
        let auto_call_out_inhib = altitude_callout_inhib_sheet.auto_call_out_inhib();

        let hysteresis_out = self.hysteresis.update(rh);
        let reset_pulse_out = self.reset_pulse.update(hysteresis_out);

        let mem_out = self.mem.update(self.prec_node.value(), reset_pulse_out);

        let active = hysteresis_out && seuil_1000_ft && !auto_call_out_inhib && !mem_out;

        let active_pulse_out = self.active_pulse.update(active);
        self.prec_node.update(active_pulse_out);

        self.one_thousand = active;
    }
}

impl WarningActivation for AltitudeCallout1000FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.one_thousand
    }
}

pub(super) struct AltitudeCallout500FtAnnounceActivation {
    conf1: ConfirmationNode,
    conf2: ConfirmationNode,
    mtrig1: MonostableTriggerNode,
    mtrig2: MonostableTriggerNode,
    prec1: PreceedingValueNode,
    prec2: PreceedingValueNode,
    five_hundred: bool,
}

impl Default for AltitudeCallout500FtAnnounceActivation {
    fn default() -> Self {
        Self {
            conf1: ConfirmationNode::new(false, Duration::from_secs_f64(0.5)),
            conf2: ConfirmationNode::new(false, Duration::from_secs_f64(0.5)),
            mtrig1: MonostableTriggerNode::new(true, Duration::from_secs(11)),
            mtrig2: MonostableTriggerNode::new(true, Duration::from_secs(11)),
            prec1: PreceedingValueNode::new(),
            prec2: PreceedingValueNode::new(),
            five_hundred: false,
        }
    }
}

impl AltitudeCallout500FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl AutoCalloutPins + GlideDeviation),
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers1,
    ) {
        let glide_deviation_1 = signals.glide_deviation(1);
        let glide_deviation_2 = signals.glide_deviation(2);

        let pin_500_ft_glide_deviation = signals.auto_call_out_500_ft_glide_deviation().value();
        let pin_500_ft = signals.auto_call_out_500_ft().value();

        let auto_call_out_inhib = altitude_callout_inhib_sheet.auto_call_out_inhib();
        let seuil_500_ft = altitude_trigger_sheet.seuil_500_ft();

        let glide_deviation_1_inv_or_ncd = glide_deviation_1.is_inv() || glide_deviation_1.is_ncd();
        let glide_deviation_2_inv_or_ncd = glide_deviation_2.is_inv() || glide_deviation_2.is_ncd();

        let conf1_out = self.conf1.update(
            glide_deviation_1.value() < Ratio::new::<ratio>(0.175),
            delta,
        );
        let conf2_out = self.conf2.update(
            glide_deviation_2.value() < Ratio::new::<ratio>(0.175),
            delta,
        );

        let on_gs_1 = glide_deviation_1_inv_or_ncd || conf1_out;
        let on_gs_2 = glide_deviation_2_inv_or_ncd || conf2_out;
        let dual_inv_gs = glide_deviation_1_inv_or_ncd && glide_deviation_2_inv_or_ncd;

        let glide_deviation = !on_gs_1 || !on_gs_2 || dual_inv_gs;

        let callout_glide_deviation = glide_deviation
            && pin_500_ft_glide_deviation
            && seuil_500_ft
            && !auto_call_out_inhib
            && !self.prec1.value();
        self.prec1
            .update(self.mtrig1.update(callout_glide_deviation, delta));

        let callout_500_ft =
            pin_500_ft && seuil_500_ft && !auto_call_out_inhib && !self.prec2.value();
        self.prec2.update(self.mtrig2.update(callout_500_ft, delta));

        self.five_hundred = callout_glide_deviation || callout_500_ft;
    }
}

impl WarningActivation for AltitudeCallout500FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.five_hundred
    }
}

pub(super) struct AltitudeCallout400FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    four_hundred: bool,
}

impl Default for AltitudeCallout400FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(5)),
            prec: Default::default(),
            four_hundred: false,
        }
    }
}

impl AltitudeCallout400FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers2,
    ) {
        let active = self.pulse.update(altitude_trigger_sheet.seuil_400_ft())
            && !altitude_callout_inhib_sheet.auto_call_out_inhib()
            && !self.prec.value();

        self.prec.update(self.mtrig.update(active, delta));

        self.four_hundred = active;
    }
}

impl WarningActivation for AltitudeCallout400FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.four_hundred
    }
}

pub(super) struct AltitudeCallout300FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_300: bool,
}

impl Default for AltitudeCallout300FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(5)),
            prec: Default::default(),
            aco_300: false,
        }
    }
}

impl AltitudeCallout300FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers2,
    ) {
        let active = self.pulse.update(altitude_trigger_sheet.seuil_300_ft())
            && !altitude_callout_inhib_sheet.auto_call_out_inhib()
            && !self.prec.value();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_300 = active;
    }
}

impl WarningActivation for AltitudeCallout300FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_300
    }
}

pub(super) struct AltitudeCallout200FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_200: bool,
}

impl Default for AltitudeCallout200FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(5)),
            prec: Default::default(),
            aco_200: false,
        }
    }
}

impl AltitudeCallout200FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers2,
    ) {
        let active = self.pulse.update(altitude_trigger_sheet.seuil_200_ft())
            && !altitude_callout_inhib_sheet.auto_call_out_inhib()
            && !self.prec.value();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_200 = active;
    }
}

impl WarningActivation for AltitudeCallout200FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_200
    }
}

pub(super) struct AltitudeCallout100FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_100: bool,
}

impl Default for AltitudeCallout100FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(5)),
            prec: Default::default(),
            aco_100: false,
        }
    }
}

impl AltitudeCallout100FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers2,
    ) {
        let active = self.pulse.update(altitude_trigger_sheet.seuil_100_ft())
            && !altitude_callout_inhib_sheet.auto_call_out_inhib()
            && !self.prec.value();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_100 = active;
    }
}

impl WarningActivation for AltitudeCallout100FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_100
    }
}

pub(super) trait AltitudeCallout50FtAnnounce {
    fn audio_50(&self) -> bool;
}

pub(super) struct AltitudeCallout50FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_50: bool,
}

impl Default for AltitudeCallout50FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_50: false,
        }
    }
}

impl AltitudeCallout50FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers2,
        audio_40_sheet: &impl AltitudeCallout40FtAnnounce,
    ) {
        let active = self.pulse.update(
            altitude_trigger_sheet.seuil_50_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib(),
        ) && !self.prec.value()
            && !audio_40_sheet.audio_40();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_50 = active;
    }
}

impl AltitudeCallout50FtAnnounce for AltitudeCallout50FtAnnounceActivation {
    fn audio_50(&self) -> bool {
        self.aco_50
    }
}

impl WarningActivation for AltitudeCallout50FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_50
    }
}

pub(super) trait AltitudeCallout40FtAnnounce {
    fn audio_40(&self) -> bool;
}

pub(super) struct AltitudeCallout40FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_40: bool,
}

impl Default for AltitudeCallout40FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_40: false,
        }
    }
}

impl AltitudeCallout40FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers3,
        audio_30_sheet: &impl AltitudeCallout30FtAnnounce,
    ) {
        let active = self.pulse.update(
            altitude_trigger_sheet.seuil_40_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib(),
        ) && !self.prec.value()
            && !audio_30_sheet.audio_30();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_40 = active;
    }
}

impl AltitudeCallout40FtAnnounce for AltitudeCallout40FtAnnounceActivation {
    fn audio_40(&self) -> bool {
        self.aco_40
    }
}

impl WarningActivation for AltitudeCallout40FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_40
    }
}

pub(super) trait AltitudeCallout30FtAnnounce {
    fn audio_30(&self) -> bool;
}

pub(super) struct AltitudeCallout30FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_30: bool,
}

impl Default for AltitudeCallout30FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_30: false,
        }
    }
}

impl AltitudeCallout30FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers3,
        audio_20_sheet: &impl AltitudeCallout20FtAnnounce,
    ) {
        let active = self.pulse.update(
            altitude_trigger_sheet.seuil_30_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib(),
        ) && !self.prec.value()
            && !audio_20_sheet.audio_20();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_30 = active;
    }
}

impl AltitudeCallout30FtAnnounce for AltitudeCallout30FtAnnounceActivation {
    fn audio_30(&self) -> bool {
        self.aco_30
    }
}

impl WarningActivation for AltitudeCallout30FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_30
    }
}

pub(super) trait AltitudeCallout20FtAnnounce {
    fn audio_20(&self) -> bool;
}

pub(super) struct AltitudeCallout20FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_20: bool,
}

impl Default for AltitudeCallout20FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_20: false,
        }
    }
}

impl AltitudeCallout20FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl LandTrkModeOn + AThrEngaged),
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers3,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
        audio_10_sheet: &impl AltitudeCallout10FtAnnounce,
    ) {
        let ap1_in_land = ap_sheet.ap1_engd() && signals.land_trk_mode_on(1).value();
        let ap2_in_land = ap_sheet.ap2_engd() && signals.land_trk_mode_on(2).value();
        let any_ap_in_land = ap1_in_land || ap2_in_land;
        let athr_engaged = signals.athr_engaged().value();

        let pulse_out = self.pulse.update(
            altitude_trigger_sheet.seuil_20_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib(),
        );

        let active = any_ap_in_land
            && athr_engaged
            && pulse_out
            && !self.prec.value()
            && !audio_10_sheet.audio_10();

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_20 = active;
    }
}

impl AltitudeCallout20FtAnnounce for AltitudeCallout20FtAnnounceActivation {
    fn audio_20(&self) -> bool {
        self.aco_20
    }
}

impl WarningActivation for AltitudeCallout20FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_20
    }
}

pub(super) trait AltitudeCallout10FtAnnounce {
    fn audio_10(&self) -> bool;
}

pub(super) struct AltitudeCallout10FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_10: bool,
}

impl Default for AltitudeCallout10FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_10: false,
        }
    }
}

impl AltitudeCallout10FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl LandTrkModeOn + AThrEngaged),
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers3,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
        audio_5_sheet: &impl AltitudeCallout5FtAnnounce,
    ) {
        let ap1_in_land = ap_sheet.ap1_engd() && signals.land_trk_mode_on(1).value();
        let ap2_in_land = ap_sheet.ap2_engd() && signals.land_trk_mode_on(2).value();
        let any_ap_in_land = ap1_in_land || ap2_in_land;
        let athr_engaged = signals.athr_engaged().value();

        let pulse_out = self.pulse.update(
            altitude_trigger_sheet.seuil_10_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib(),
        );

        let retard_inhibtion = false; // TODO

        let saying_five = false; // TODO

        let active = ((!any_ap_in_land && athr_engaged) || !athr_engaged)
            && pulse_out
            && !self.prec.value()
            && !retard_inhibtion
            && !saying_five;

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_10 = active;
    }
}

impl AltitudeCallout10FtAnnounce for AltitudeCallout10FtAnnounceActivation {
    fn audio_10(&self) -> bool {
        self.aco_10
    }
}

impl WarningActivation for AltitudeCallout10FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_10
    }
}

pub(super) trait AltitudeCallout5FtAnnounce {
    fn audio_5(&self) -> bool;
}

pub(super) struct AltitudeCallout5FtAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_5: bool,
}

impl Default for AltitudeCallout5FtAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: Default::default(),
            aco_5: false,
        }
    }
}

impl AltitudeCallout5FtAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        altitude_callout_inhib_sheet: &impl AutomaticCallOutInhibition,
        altitude_trigger_sheet: &impl AltitudeThresholdTriggers3,
    ) {
        let retard_inhibition = false;

        let active = self.pulse.update(
            altitude_trigger_sheet.seuil_5_ft()
                && !altitude_callout_inhib_sheet.auto_call_out_inhib()
                && !self.prec.value()
                && !retard_inhibition,
        );

        self.prec.update(self.mtrig.update(active, delta));

        self.aco_5 = active;
    }
}

impl AltitudeCallout5FtAnnounce for AltitudeCallout5FtAnnounceActivation {
    fn audio_5(&self) -> bool {
        self.aco_5
    }
}

impl WarningActivation for AltitudeCallout5FtAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_5
    }
}

pub(super) trait AltitudeCalloutThresholdDetection {
    fn non_inhibited_threshold_detection(&self) -> bool;
}

#[derive(Default)]
pub(super) struct AltitudeCalloutThresholdDetectionActivation {
    non_inhibited_threshold_detection: bool,
}

impl AltitudeCalloutThresholdDetectionActivation {
    pub fn update(
        &mut self,
        triggers2_sheet: &impl AltitudeThresholdTriggers2,
        triggers3_sheet: &impl AltitudeThresholdTriggers3,
    ) {
        self.non_inhibited_threshold_detection = triggers2_sheet.seuil_400_ft()
            || triggers2_sheet.seuil_300_ft()
            || triggers2_sheet.seuil_200_ft()
            || triggers2_sheet.seuil_100_ft()
            || triggers2_sheet.seuil_50_ft()
            || triggers3_sheet.seuil_40_ft()
            || triggers3_sheet.seuil_30_ft()
            || triggers3_sheet.seuil_20_ft()
            || triggers3_sheet.seuil_10_ft();
    }
}

impl AltitudeCalloutThresholdDetection for AltitudeCalloutThresholdDetectionActivation {
    fn non_inhibited_threshold_detection(&self) -> bool {
        self.non_inhibited_threshold_detection
    }
}

pub(super) trait IntermediateAudio {
    fn intermediate_call_out(&self) -> bool;
}

pub(super) struct IntermediateAudioActivation {
    pulse1: PulseNode,
    pulse2: PulseNode,
    mrtrig1: MonostableTriggerNode,
    mrtrig2: MonostableTriggerNode,
    mem: MemoryNode,
    intermediate_call_out: bool,
}

impl Default for IntermediateAudioActivation {
    fn default() -> Self {
        Self {
            pulse1: PulseNode::new(true),
            pulse2: PulseNode::new(true),
            mrtrig1: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(4)),
            mrtrig2: MonostableTriggerNode::new_retriggerable(true, Duration::from_secs(11)),
            mem: MemoryNode::new(true),
            intermediate_call_out: false,
        }
    }
}

impl IntermediateAudioActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        threshold1_sheet: &impl AltitudeThreshold1,
        threshold3_sheet: &impl AltitudeThreshold3,
        threshold_detection_sheet: &impl AltitudeCalloutThresholdDetection,
        minimum_sheet: &impl Minimum,
        auto_call_out_generated: bool,
        inter_audio: bool,
    ) {
        let inhibited = self.mem.update(
            threshold1_sheet.alt_sup_410_ft()
                || threshold3_sheet.to_and_ground_detection()
                || threshold3_sheet.gpws_inhibition(),
            threshold_detection_sheet.non_inhibited_threshold_detection()
                && auto_call_out_generated,
        );

        let dh_generated = minimum_sheet.dh_generated();

        let pulse1_out = self.pulse1.update(threshold3_sheet.threshold_detection());
        let pulse2_out = self.pulse2.update(inter_audio);
        let threshold_or_inter_audio = pulse1_out || pulse2_out;

        let alt_sub_50_ft = threshold1_sheet.alt_sup_50_ft();

        let mrtrig1_out = self
            .mrtrig1
            .update(threshold_or_inter_audio && !alt_sub_50_ft, delta);
        let mrtrig2_out = self
            .mrtrig2
            .update(threshold_or_inter_audio && alt_sub_50_ft, delta);

        self.intermediate_call_out = inhibited || dh_generated || mrtrig1_out || mrtrig2_out;
    }
}

impl IntermediateAudio for IntermediateAudioActivation {
    fn intermediate_call_out(&self) -> bool {
        self.intermediate_call_out
    }
}

#[cfg(test)]
mod tests {
    use crate::flight_warning::test::*;
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

        // TODO a/b channel discrepancy
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

    mod landing_gear_downlock {
        use super::*;

        #[test]
        fn when_all_gears_are_downlocked_is_full_down_lock() {
            let mut sheet = LgDownlockedActivation::default();
            sheet.update(
                test_bed_with()
                    .rh_gear_downlocked(true)
                    .lh_gear_downlocked(true)
                    .nose_gear_downlocked(true)
                    .parameters(),
            );
            assert_eq!(sheet.lg_downlocked(), true);
            assert_eq!(sheet.main_lg_downlocked(), true);
        }

        #[test]
        fn when_main_gears_are_downlocked_is_not_full_down_lock() {
            let mut sheet = LgDownlockedActivation::default();
            sheet.update(
                test_bed_with()
                    .rh_gear_downlocked(true)
                    .lh_gear_downlocked(true)
                    .nose_gear_downlocked(false)
                    .parameters(),
            );
            assert_eq!(sheet.main_lg_downlocked(), true);
            assert_eq!(sheet.lg_downlocked(), false);
        }

        #[test]
        fn when_no_gear_is_downlocked_is_no_down_lock() {
            let mut sheet = LgDownlockedActivation::default();
            sheet.update(
                test_bed_with()
                    .rh_gear_downlocked(false)
                    .lh_gear_downlocked(false)
                    .nose_gear_downlocked(false)
                    .parameters(),
            );
            assert_eq!(sheet.main_lg_downlocked(), false);
            assert_eq!(sheet.lg_downlocked(), false);
        }

        #[test]
        fn when_one_main_gear_is_downlocked_is_no_down_lock() {
            let mut sheet = LgDownlockedActivation::default();
            sheet.update(
                test_bed_with()
                    .rh_gear_downlocked(false)
                    .lh_gear_downlocked(true)
                    .nose_gear_downlocked(false)
                    .parameters(),
            );
            assert_eq!(sheet.main_lg_downlocked(), false);
            assert_eq!(sheet.lg_downlocked(), false);
        }
    }

    mod voluntary_autopilot_warnings {
        use super::*;

        #[test]
        fn reports_ap1_engaged() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs(1),
                test_bed_with().ap1_engaged(true).parameters(),
                false,
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
                test_bed_with().ap2_engaged(true).parameters(),
                false,
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
                test_bed_with()
                    .ap1_engaged(true)
                    .ap2_engaged(true)
                    .parameters(),
                false,
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
                test_bed_with().ap1_engaged(true).parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(1.5), test_bed().parameters(), true);
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(1.5), test_bed().parameters(), true);
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(Duration::from_secs_f64(6.0), test_bed().parameters(), false);
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
        }

        #[test]
        fn stops_warning_when_instinctive_disconnect_is_pressed_again() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.001),
                test_bed_with().ap1_engaged(true).parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.001),
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
            sheet.update(
                Duration::from_secs_f64(0.001),
                test_bed_with()
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
                false,
            );

            // another press within 0.2s is inhibited
            sheet.update(
                Duration::from_secs_f64(0.001),
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.001),
                test_bed_with()
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);

            // after 0.2s a press inhibits the mw and text signal, but audio remains
            sheet.update(
                Duration::from_secs_f64(0.2),
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);

            // once 0.5s have passed since the inhibit is pressed, the audio is also inhibited
            sheet.update(
                Duration::from_secs_f64(0.5),
                test_bed_with()
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), false);
        }

        #[test]
        fn warns_when_ap2_is_instinctively_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with().ap2_engaged(true).parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with()
                    .ap2_engaged(false)
                    .instinc_disconnect_2ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
        }

        #[test]
        fn warns_when_both_aps_are_instinctively_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with()
                    .ap1_engaged(true)
                    .ap2_engaged(true)
                    .parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with()
                    .ap1_engaged(false)
                    .ap2_engaged(false)
                    .instinc_disconnect_1ap_engd(true)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), true);
            assert_eq!(sheet.ap_off_mw(), true);
            assert_eq!(sheet.ap_off_text(), true);
        }

        #[test]
        fn does_not_warn_when_ap1_is_involuntarily_disconnected() {
            let mut sheet = AutoFlightAutopilotOffVoluntaryActivation::default();
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with().ap1_engaged(true).parameters(),
                false,
            );
            sheet.update(
                Duration::from_secs_f64(0.1),
                test_bed_with()
                    .ap1_engaged(false)
                    .instinc_disconnect_1ap_engd(false)
                    .parameters(),
                false,
            );
            assert_eq!(sheet.ap_off_audio(), false);
            assert_eq!(sheet.ap_off_mw(), false);
            assert_eq!(sheet.ap_off_text(), false);
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

    impl FlightPhasesAltitudeDef for TestAltitudeDef {
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

use systems::flight_warning::parameters::*;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::percent;
use uom::si::velocity::knot;

pub(super) trait LhLgCompressed {
    fn lh_lg_compressed(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait EssLhLgCompressed {
    fn ess_lh_lg_compressed(&self) -> &DiscreteParameter;
}

pub(super) trait NormLhLgCompressed {
    fn norm_lh_lg_compressed(&self) -> &DiscreteParameter;
}

pub(super) trait RadioHeight {
    fn radio_height(&self, index: usize) -> &Arinc429Parameter<Length>;
}

pub(super) trait ComputedSpeed {
    fn computed_speed(&self, index: usize) -> &Arinc429Parameter<Velocity>;
}

pub(super) trait Eng1MasterLeverSelectOn {
    fn eng1_master_lever_select_on(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2MasterLeverSelectOn {
    fn eng2_master_lever_select_on(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1CoreSpeedAtOrAboveIdle {
    fn eng1_core_speed_at_or_above_idle(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2CoreSpeedAtOrAboveIdle {
    fn eng2_core_speed_at_or_above_idle(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1FirePbOut {
    fn eng_1_fire_pb_out(&self) -> &DiscreteParameter;
}

pub(super) trait ToConfigTest {
    fn to_config_test(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1Tla {
    fn eng1_tla(&self, index: usize) -> &Arinc429Parameter<Angle>;
}

pub(super) trait Eng2Tla {
    fn eng2_tla(&self, index: usize) -> &Arinc429Parameter<Angle>;
}

pub(super) trait Eng1TlaFto {
    fn eng1_tla_fto(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2TlaFto {
    fn eng2_tla_fto(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1AutoToga {
    fn eng_1_auto_toga(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1LimitModeSoftGa {
    fn eng_1_limit_mode_soft_ga(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2AutoToga {
    fn eng_2_auto_toga(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2LimitModeSoftGa {
    fn eng_2_limit_mode_soft_ga(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1N1SelectedActual {
    fn eng1_n1_selected_actual(&self, index: usize) -> &Arinc429Parameter<Ratio>;
}

pub(super) trait Eng2N1SelectedActual {
    fn eng2_n1_selected_actual(&self, index: usize) -> &Arinc429Parameter<Ratio>;
}

pub(super) trait Tla1IdlePwr {
    fn tla1_idle_pwr(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Tla2IdlePwr {
    fn tla2_idle_pwr(&self, index: usize) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1ChannelInControl {
    fn eng1_channel_a_in_control(&self) -> &Arinc429Parameter<bool>;
    fn eng1_channel_b_in_control(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2ChannelInControl {
    fn eng2_channel_a_in_control(&self) -> &Arinc429Parameter<bool>;
    fn eng2_channel_b_in_control(&self) -> &Arinc429Parameter<bool>;
}

pub(super) struct A320SignalTable {
    lh_lg_compressed_1: Arinc429Parameter<bool>,
    lh_lg_compressed_2: Arinc429Parameter<bool>,
    ess_lh_lg_compressed: DiscreteParameter,
    norm_lh_lg_compressed: DiscreteParameter,
    radio_height_1: Arinc429Parameter<Length>,
    radio_height_2: Arinc429Parameter<Length>,
    computed_speed_1: Arinc429Parameter<Velocity>,
    computed_speed_2: Arinc429Parameter<Velocity>,
    computed_speed_3: Arinc429Parameter<Velocity>,
    eng1_master_lever_select_on: Arinc429Parameter<bool>,
    eng2_master_lever_select_on: Arinc429Parameter<bool>,
    eng1_core_speed_at_or_above_idle_a: Arinc429Parameter<bool>,
    eng1_core_speed_at_or_above_idle_b: Arinc429Parameter<bool>,
    eng2_core_speed_at_or_above_idle_a: Arinc429Parameter<bool>,
    eng2_core_speed_at_or_above_idle_b: Arinc429Parameter<bool>,
    eng_1_fire_pb_out: DiscreteParameter,
    to_config_test: Arinc429Parameter<bool>,
    eng1_tla_a: Arinc429Parameter<Angle>,
    eng1_tla_b: Arinc429Parameter<Angle>,
    eng2_tla_a: Arinc429Parameter<Angle>,
    eng2_tla_b: Arinc429Parameter<Angle>,
    eng1_tla_fto_a: Arinc429Parameter<bool>,
    eng1_tla_fto_b: Arinc429Parameter<bool>,
    eng2_tla_fto_a: Arinc429Parameter<bool>,
    eng2_tla_fto_b: Arinc429Parameter<bool>,
    eng1_n1_selected_actual_a: Arinc429Parameter<Ratio>,
    eng1_n1_selected_actual_b: Arinc429Parameter<Ratio>,
    eng2_n1_selected_actual_a: Arinc429Parameter<Ratio>,
    eng2_n1_selected_actual_b: Arinc429Parameter<Ratio>,
    tla1_idle_pwr_a: Arinc429Parameter<bool>,
    tla1_idle_pwr_b: Arinc429Parameter<bool>,
    tla2_idle_pwr_a: Arinc429Parameter<bool>,
    tla2_idle_pwr_b: Arinc429Parameter<bool>,
    eng1_channel_a_in_control: Arinc429Parameter<bool>,
    eng1_channel_b_in_control: Arinc429Parameter<bool>,
    eng2_channel_a_in_control: Arinc429Parameter<bool>,
    eng2_channel_b_in_control: Arinc429Parameter<bool>,
}
impl A320SignalTable {
    pub fn new() -> Self {
        Self {
            lh_lg_compressed_1: Arinc429Parameter::new_inv(false),
            lh_lg_compressed_2: Arinc429Parameter::new_inv(false),
            ess_lh_lg_compressed: DiscreteParameter::new_inv(false),
            norm_lh_lg_compressed: DiscreteParameter::new_inv(false),
            radio_height_1: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            radio_height_2: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            computed_speed_1: Arinc429Parameter::new_inv(Velocity::new::<knot>(0.0)),
            computed_speed_2: Arinc429Parameter::new_inv(Velocity::new::<knot>(0.0)),
            computed_speed_3: Arinc429Parameter::new_inv(Velocity::new::<knot>(0.0)),
            eng1_master_lever_select_on: Arinc429Parameter::new_inv(false),
            eng2_master_lever_select_on: Arinc429Parameter::new_inv(false),
            eng1_core_speed_at_or_above_idle_a: Arinc429Parameter::new_inv(false),
            eng1_core_speed_at_or_above_idle_b: Arinc429Parameter::new_inv(false),
            eng2_core_speed_at_or_above_idle_a: Arinc429Parameter::new_inv(false),
            eng2_core_speed_at_or_above_idle_b: Arinc429Parameter::new_inv(false),
            eng_1_fire_pb_out: DiscreteParameter::new_inv(false),
            to_config_test: Arinc429Parameter::new_inv(false),
            eng1_tla_a: Arinc429Parameter::new_inv(Angle::new::<degree>(0.0)),
            eng1_tla_b: Arinc429Parameter::new_inv(Angle::new::<degree>(0.0)),
            eng2_tla_a: Arinc429Parameter::new_inv(Angle::new::<degree>(0.0)),
            eng2_tla_b: Arinc429Parameter::new_inv(Angle::new::<degree>(0.0)),
            eng1_tla_fto_a: Arinc429Parameter::new_inv(false),
            eng1_tla_fto_b: Arinc429Parameter::new_inv(false),
            eng2_tla_fto_a: Arinc429Parameter::new_inv(false),
            eng2_tla_fto_b: Arinc429Parameter::new_inv(false),
            eng1_n1_selected_actual_a: Arinc429Parameter::new_inv(Ratio::new::<percent>(0.0)),
            eng1_n1_selected_actual_b: Arinc429Parameter::new_inv(Ratio::new::<percent>(0.0)),
            eng2_n1_selected_actual_a: Arinc429Parameter::new_inv(Ratio::new::<percent>(0.0)),
            eng2_n1_selected_actual_b: Arinc429Parameter::new_inv(Ratio::new::<percent>(0.0)),
            tla1_idle_pwr_a: Arinc429Parameter::new_inv(false),
            tla1_idle_pwr_b: Arinc429Parameter::new_inv(false),
            tla2_idle_pwr_a: Arinc429Parameter::new_inv(false),
            tla2_idle_pwr_b: Arinc429Parameter::new_inv(false),
            eng1_channel_a_in_control: Arinc429Parameter::new_inv(false),
            eng1_channel_b_in_control: Arinc429Parameter::new_inv(false),
            eng2_channel_a_in_control: Arinc429Parameter::new_inv(false),
            eng2_channel_b_in_control: Arinc429Parameter::new_inv(false),
        }
    }

    pub(super) fn set_takeoff_config_test(&mut self, pressed: bool) {
        self.to_config_test = Arinc429Parameter::new(pressed)
    }

    pub(super) fn set_computed_speed_1(&mut self, speed: Arinc429Parameter<Velocity>) {
        self.computed_speed_1 = speed;
    }

    pub(super) fn set_computed_speed_2(&mut self, speed: Arinc429Parameter<Velocity>) {
        self.computed_speed_2 = speed;
    }

    pub(super) fn set_computed_speed_3(&mut self, speed: Arinc429Parameter<Velocity>) {
        self.computed_speed_3 = speed;
    }

    pub(super) fn set_lh_lg_compressed_1(&mut self, compressed: Arinc429Parameter<bool>) {
        self.lh_lg_compressed_1 = compressed;
    }

    pub(super) fn set_lh_lg_compressed_2(&mut self, compressed: Arinc429Parameter<bool>) {
        self.lh_lg_compressed_2 = compressed;
    }

    pub(super) fn set_ess_lh_lg_compressed(&mut self, compressed: DiscreteParameter) {
        self.ess_lh_lg_compressed = compressed;
    }

    pub(super) fn set_norm_lh_lg_compressed(&mut self, compressed: DiscreteParameter) {
        self.norm_lh_lg_compressed = compressed;
    }

    pub(super) fn set_radio_height_1(&mut self, height: Arinc429Parameter<Length>) {
        self.radio_height_1 = height;
    }

    pub(super) fn set_radio_height_2(&mut self, height: Arinc429Parameter<Length>) {
        self.radio_height_2 = height;
    }

    pub(super) fn set_eng_1_fire_pb_out(&mut self, fire_pb_out: DiscreteParameter) {
        self.eng_1_fire_pb_out = fire_pb_out;
    }

    pub(super) fn set_eng1_master_lever_select_on(&mut self, on: Arinc429Parameter<bool>) {
        self.eng1_master_lever_select_on = on;
    }

    pub(super) fn set_eng2_master_lever_select_on(&mut self, on: Arinc429Parameter<bool>) {
        self.eng2_master_lever_select_on = on;
    }

    pub(super) fn set_eng1_core_speed_at_or_above_idle_a(
        &mut self,
        at_or_above_idle: Arinc429Parameter<bool>,
    ) {
        self.eng1_core_speed_at_or_above_idle_a = at_or_above_idle;
    }

    pub(super) fn set_eng1_core_speed_at_or_above_idle_b(
        &mut self,
        at_or_above_idle: Arinc429Parameter<bool>,
    ) {
        self.eng1_core_speed_at_or_above_idle_b = at_or_above_idle;
    }

    pub(super) fn set_eng2_core_speed_at_or_above_idle_a(
        &mut self,
        at_or_above_idle: Arinc429Parameter<bool>,
    ) {
        self.eng2_core_speed_at_or_above_idle_a = at_or_above_idle;
    }

    pub(super) fn set_eng2_core_speed_at_or_above_idle_b(
        &mut self,
        at_or_above_idle: Arinc429Parameter<bool>,
    ) {
        self.eng2_core_speed_at_or_above_idle_b = at_or_above_idle;
    }

    pub(super) fn set_eng1_tla_a(&mut self, tla: Arinc429Parameter<Angle>) {
        self.eng1_tla_a = tla
    }

    pub(super) fn set_eng1_tla_b(&mut self, tla: Arinc429Parameter<Angle>) {
        self.eng1_tla_b = tla
    }

    pub(super) fn set_eng2_tla_a(&mut self, tla: Arinc429Parameter<Angle>) {
        self.eng2_tla_a = tla
    }

    pub(super) fn set_eng2_tla_b(&mut self, tla: Arinc429Parameter<Angle>) {
        self.eng2_tla_b = tla
    }
}
impl LhLgCompressed for A320SignalTable {
    fn lh_lg_compressed(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.lh_lg_compressed_1,
            2 => &self.lh_lg_compressed_2,
            _ => panic!(),
        }
    }
}
impl EssLhLgCompressed for A320SignalTable {
    fn ess_lh_lg_compressed(&self) -> &DiscreteParameter {
        &self.ess_lh_lg_compressed
    }
}
impl NormLhLgCompressed for A320SignalTable {
    fn norm_lh_lg_compressed(&self) -> &DiscreteParameter {
        &self.norm_lh_lg_compressed
    }
}
impl RadioHeight for A320SignalTable {
    fn radio_height(&self, index: usize) -> &Arinc429Parameter<Length> {
        match index {
            1 => &self.radio_height_1,
            2 => &self.radio_height_2,
            _ => panic!(),
        }
    }
}
impl ComputedSpeed for A320SignalTable {
    fn computed_speed(&self, index: usize) -> &Arinc429Parameter<Velocity> {
        match index {
            1 => &self.computed_speed_1,
            2 => &self.computed_speed_2,
            3 => &self.computed_speed_3,
            _ => panic!(),
        }
    }
}
impl Eng1MasterLeverSelectOn for A320SignalTable {
    fn eng1_master_lever_select_on(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_master_lever_select_on
    }
}

impl Eng2MasterLeverSelectOn for A320SignalTable {
    fn eng2_master_lever_select_on(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_master_lever_select_on
    }
}

impl Eng1CoreSpeedAtOrAboveIdle for A320SignalTable {
    fn eng1_core_speed_at_or_above_idle(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng1_core_speed_at_or_above_idle_a,
            2 => &self.eng1_core_speed_at_or_above_idle_b,
            _ => panic!(),
        }
    }
}

impl Eng2CoreSpeedAtOrAboveIdle for A320SignalTable {
    fn eng2_core_speed_at_or_above_idle(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng2_core_speed_at_or_above_idle_a,
            2 => &self.eng2_core_speed_at_or_above_idle_b,
            _ => panic!(),
        }
    }
}

impl Eng1FirePbOut for A320SignalTable {
    fn eng_1_fire_pb_out(&self) -> &DiscreteParameter {
        &self.eng_1_fire_pb_out
    }
}

impl ToConfigTest for A320SignalTable {
    fn to_config_test(&self) -> &Arinc429Parameter<bool> {
        &self.to_config_test
    }
}

impl Eng1Tla for A320SignalTable {
    fn eng1_tla(&self, index: usize) -> &Arinc429Parameter<Angle> {
        match index {
            1 => &self.eng1_tla_a,
            2 => &self.eng1_tla_b,
            _ => panic!(),
        }
    }
}

impl Eng2Tla for A320SignalTable {
    fn eng2_tla(&self, index: usize) -> &Arinc429Parameter<Angle> {
        match index {
            1 => &self.eng2_tla_a,
            2 => &self.eng2_tla_b,
            _ => panic!(),
        }
    }
}

impl Eng1TlaFto for A320SignalTable {
    fn eng1_tla_fto(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng1_tla_fto_a,
            2 => &self.eng1_tla_fto_b,
            _ => panic!(),
        }
    }
}

impl Eng2TlaFto for A320SignalTable {
    fn eng2_tla_fto(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng2_tla_fto_a,
            2 => &self.eng2_tla_fto_b,
            _ => panic!(),
        }
    }
}

impl Eng1N1SelectedActual for A320SignalTable {
    fn eng1_n1_selected_actual(&self, index: usize) -> &Arinc429Parameter<Ratio> {
        match index {
            1 => &self.eng1_n1_selected_actual_a,
            2 => &self.eng1_n1_selected_actual_b,
            _ => panic!(),
        }
    }
}
impl Eng2N1SelectedActual for A320SignalTable {
    fn eng2_n1_selected_actual(&self, index: usize) -> &Arinc429Parameter<Ratio> {
        match index {
            1 => &self.eng2_n1_selected_actual_a,
            2 => &self.eng2_n1_selected_actual_a,
            _ => panic!(),
        }
    }
}
impl Tla1IdlePwr for A320SignalTable {
    fn tla1_idle_pwr(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.tla1_idle_pwr_a,
            2 => &self.tla1_idle_pwr_b,
            _ => panic!(),
        }
    }
}
impl Tla2IdlePwr for A320SignalTable {
    fn tla2_idle_pwr(&self, index: usize) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.tla2_idle_pwr_a,
            2 => &self.tla2_idle_pwr_b,
            _ => panic!(),
        }
    }
}
impl Eng1ChannelInControl for A320SignalTable {
    fn eng1_channel_a_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_channel_a_in_control
    }

    fn eng1_channel_b_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_channel_b_in_control
    }
}
impl Eng2ChannelInControl for A320SignalTable {
    fn eng2_channel_a_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_channel_a_in_control
    }

    fn eng2_channel_b_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_channel_a_in_control
    }
}

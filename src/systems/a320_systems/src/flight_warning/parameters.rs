use systems::flight_warning::parameters::*;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::{percent, ratio};
use uom::si::velocity::knot;

pub(super) trait FwcIdentSide1 {
    /// This signal indicates that the FWC is installed as FWC 1.
    fn fwc_ident_side1(&self) -> &DiscreteParameter;
}

pub(super) trait FwcIdentSide2 {
    /// This signal indicates that the FWC is installed as FWC 2.
    fn fwc_ident_side2(&self) -> &DiscreteParameter;
}

pub(super) trait LhLgCompressed {
    /// This signal indicates that the left main landing gear is compressed.
    /// The index is either 1 or 2, corresponding to LGCIU 1 or 2 respectively.
    fn lh_lg_compressed(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait EssLhLgCompressed {
    /// This signal indicates that the left main landing gear is compressed.
    /// This discrete signal is provided by LGCIU 1, which is powered by the ESS bus.
    fn ess_lh_lg_compressed(&self) -> &DiscreteParameter;
}

pub(super) trait NormLhLgCompressed {
    /// This signal indicates that the left main landing gear is compressed.
    /// This discrete signal is provided by LGCIU 2.
    fn norm_lh_lg_compressed(&self) -> &DiscreteParameter;
}

pub(super) trait LhGearDownLock {
    /// This signal indicates that the left main landing gear is downlocked.
    /// The index is either 1 or 2, corresponding to LGCIU 1 or 2 respectively.
    fn lh_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait RhGearDownLock {
    /// This signal indicates that the right main landing gear is downlocked.
    /// The index is either 1 or 2, corresponding to LGCIU 1 or 2 respectively.
    fn rh_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait NoseGearDownLock {
    /// This signal indicates that the nose landing gear is downlocked.
    /// The index is either 1 or 2, corresponding to LGCIU 1 or 2 respectively.
    fn nose_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait RadioHeight {
    /// This signal contains the measured radio altitude.
    /// The index is either 1 or 2, corresponding to Radio Altimeter 1 or 2 respectively.
    fn radio_height(&self, index: u8) -> &Arinc429Parameter<Length>;
}

pub(super) trait ComputedSpeed {
    fn computed_speed(&self, index: u8) -> &Arinc429Parameter<Velocity>;
}

pub(super) trait Eng1MasterLeverSelectOn {
    fn eng1_master_lever_select_on(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2MasterLeverSelectOn {
    fn eng2_master_lever_select_on(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1CoreSpeedAtOrAboveIdle {
    fn eng1_core_speed_at_or_above_idle(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2CoreSpeedAtOrAboveIdle {
    fn eng2_core_speed_at_or_above_idle(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1FirePbOut {
    fn eng_1_fire_pb_out(&self) -> &DiscreteParameter;
}

pub(super) trait ToConfigTest {
    fn to_config_test(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1Tla {
    fn eng1_tla(&self, index: u8) -> &Arinc429Parameter<Angle>;
}

pub(super) trait Eng2Tla {
    fn eng2_tla(&self, index: u8) -> &Arinc429Parameter<Angle>;
}

/// This parameter indicates that the engine 1 CFM ECU has determined that the TLA is in the
/// Flex Take Off (FTO) position.
pub(super) trait Eng1TlaFto {
    fn eng1_tla_fto(&self, index: u8) -> &Arinc429Parameter<bool>;
}

/// This parameter indicates that the engine 2 CFM ECU has determined that the TLA is in the
/// Flex Take Off (FTO) position.
pub(super) trait Eng2TlaFto {
    fn eng2_tla_fto(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1AutoToga {
    fn eng_1_auto_toga(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1LimitModeSoftGa {
    fn eng_1_limit_mode_soft_ga(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2AutoToga {
    fn eng_2_auto_toga(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2LimitModeSoftGa {
    fn eng_2_limit_mode_soft_ga(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1N1SelectedActual {
    fn eng1_n1_selected_actual(&self, index: u8) -> &Arinc429Parameter<Ratio>;
}

pub(super) trait Eng2N1SelectedActual {
    fn eng2_n1_selected_actual(&self, index: u8) -> &Arinc429Parameter<Ratio>;
}

pub(super) trait Tla1IdlePwr {
    fn tla1_idle_pwr(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Tla2IdlePwr {
    fn tla2_idle_pwr(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng1ChannelInControl {
    fn eng1_channel_a_in_control(&self) -> &Arinc429Parameter<bool>;
    fn eng1_channel_b_in_control(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Eng2ChannelInControl {
    fn eng2_channel_a_in_control(&self) -> &Arinc429Parameter<bool>;
    fn eng2_channel_b_in_control(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait AltitudeParameter {
    /// This parameter contains the barometric altitude as measures by the ADR.
    /// The index is 1, 2, or 3, for ADR 1, 2, or 3 respectively.
    fn altitude(&self, index: u8) -> &Arinc429Parameter<Length>;
}

pub(super) trait AltiSelect {
    /// This parameter contains the selected altitude in the FCU.
    fn alti_select(&self) -> &Arinc429Parameter<Length>;
}

pub(super) trait AltSelectChg {
    /// This parameter indicates that the selected altitude in the FCU has been changed.
    fn alt_select_chg(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait Ap1Engd {
    fn ap1_engd_com(&self) -> &DiscreteParameter;
    fn ap1_engd_mon(&self) -> &DiscreteParameter;
}

pub(super) trait Ap2Engd {
    fn ap2_engd_com(&self) -> &DiscreteParameter;
    fn ap2_engd_mon(&self) -> &DiscreteParameter;
}

pub(super) trait InstincDiscnct1ApEngd {
    /// This signal indicates when the A/P Disconnect button on the Captain's sidestick has been
    /// pressed.
    fn instinc_discnct_1ap_engd(&self) -> &DiscreteParameter;
}

pub(super) trait InstincDiscnct2ApEngd {
    /// This signal indicates when the A/P Disconnect button on the First Officer's sidestick has
    /// been pressed.
    fn instinc_discnct_2ap_engd(&self) -> &DiscreteParameter;
}

pub(super) trait CaptMwCancelOn {
    /// This parameter indicates that the master warning cancel button on the Captain's side
    /// has been pressed.
    fn capt_mw_cancel_on(&self) -> &DiscreteParameter;
}

pub(super) trait FoMwCancelOn {
    /// This parameter indicates that the master warning cancel button on the First Officer's side
    /// has been pressed.
    fn fo_mw_cancel_on(&self) -> &DiscreteParameter;
}

pub(super) trait CaptMcCancelOn {
    /// This parameter indicates that the master caution cancel button on the Captain's side
    /// has been pressed.
    fn capt_mc_cancel_on(&self) -> &DiscreteParameter;
}

pub(super) trait FoMcCancelOn {
    /// This parameter indicates that the master caution cancel button on the First Officer's side
    /// has been pressed.
    fn fo_mc_cancel_on(&self) -> &DiscreteParameter;
}

pub(super) trait BlueSysLoPr {
    /// This parameter indicates that the pressure switch in the blue hydraulic system has detected
    /// low hydraulic pressure.
    fn blue_sys_lo_pr(&self) -> &DiscreteParameter;
}

pub(super) trait YellowSysLoPr {
    /// This parameter indicates that the pressure switch in the yellow hydraulic system has
    /// detected low hydraulic pressure.
    fn yellow_sys_lo_pr(&self) -> &DiscreteParameter;
}

pub(super) trait GreenSysLoPr {
    /// This parameter indicates that the pressure switch in the green hydraulic system has detected
    /// low hydraulic pressure.
    fn green_sys_lo_pr(&self) -> &DiscreteParameter;
}

pub(super) trait TcasEngaged {
    /// This parameter indicates that the TCAS mode (as used by AP TCAS) is engaged.
    fn tcas_engaged(&self) -> &Arinc429Parameter<bool>;
}

pub(super) trait GsModeOn {
    /// This parameter indicates that the respective autopilot is in GS mode.
    /// The index is 1 or 2, for FMGC 1 or 2 respectively.
    fn gs_mode_on(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait GpwsModesOn {
    /// This parameter indicates that one of the main GPWS alerts is active.
    fn gpws_modes_on(&self) -> &DiscreteParameter;
}

pub(super) trait GsVisualAlertOn {
    /// This parameter indicates that the GPWS glideslope alert is active.
    fn gs_visual_alert_on(&self) -> &DiscreteParameter;
}

pub(super) trait TcasAuralAdvisaryOutput {
    /// This parameter indicates that the TCAS is issuing an aural alert.
    fn tcas_aural_advisory_output(&self) -> &DiscreteParameter;
}

pub(super) trait DecisionHeight {
    fn decision_height(&self, index: u8) -> &Arinc429Parameter<Length>;
}

pub(super) trait HundredAboveForMdaMdhRequest {
    fn hundred_above_for_mda_mdh_request(&self, index: u8) -> &DiscreteParameter;
}

pub(super) trait MinimumForMdaMdhRequest {
    fn minimum_for_mda_mdh_request(&self, index: u8) -> &DiscreteParameter;
}

pub(super) trait LandTrkModeOn {
    fn land_trk_mode_on(&self, index: u8) -> &Arinc429Parameter<bool>;
}

pub(super) trait AThrEngaged {
    fn athr_engaged(&self) -> &Arinc429Parameter<bool>;
}

/// This trait represents the pins for the pin programmed auto callouts.
pub(super) trait AutoCalloutPins {
    fn decision_height_code_a(&self) -> &DiscreteParameter;
    fn decision_height_code_b(&self) -> &DiscreteParameter;
    fn decision_height_plus_100_ft_code_a(&self) -> &DiscreteParameter;
    fn decision_height_plus_100_ft_code_b(&self) -> &DiscreteParameter;
    fn auto_call_out_2500_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_2500b(&self) -> &DiscreteParameter;
    fn auto_call_out_2000_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_1000_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_500_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_500_ft_glide_deviation(&self) -> &DiscreteParameter;
    fn auto_call_out_400_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_300_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_200_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_100_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_50_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_40_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_30_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_20_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_10_ft(&self) -> &DiscreteParameter;
    fn auto_call_out_5_ft(&self) -> &DiscreteParameter;
}

pub(super) trait GlideDeviation {
    fn glide_deviation(&self, index: u8) -> &Arinc429Parameter<Ratio>;
}

/// This struct contains the parameters acquired directly by the FWC (in other words: not through an
/// SDAC). They can be received via ARINC or as discretes.
pub struct A320FwcDirectParameterTable {
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
}

impl A320FwcDirectParameterTable {}

pub struct A320FwcSdacParameterTable {}

/// This struct represents the in-memory representation of the signals used by a Flight Warning
/// Computer to determine it's activations.
pub struct A320FWCParameterTable {
    fwc_ident_side1: DiscreteParameter,
    fwc_ident_side2: DiscreteParameter,
    lh_lg_compressed_1: Arinc429Parameter<bool>,
    lh_lg_compressed_2: Arinc429Parameter<bool>,
    ess_lh_lg_compressed: DiscreteParameter,
    norm_lh_lg_compressed: DiscreteParameter,
    lh_gear_down_lock_1: Arinc429Parameter<bool>,
    lh_gear_down_lock_2: Arinc429Parameter<bool>,
    rh_gear_down_lock_1: Arinc429Parameter<bool>,
    rh_gear_down_lock_2: Arinc429Parameter<bool>,
    nose_gear_down_lock_1: Arinc429Parameter<bool>,
    nose_gear_down_lock_2: Arinc429Parameter<bool>,
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
    eng_2_fire_pb_out: DiscreteParameter,
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
    eng_1_auto_toga_a: Arinc429Parameter<bool>,
    eng_1_auto_toga_b: Arinc429Parameter<bool>,
    eng_2_auto_toga_a: Arinc429Parameter<bool>,
    eng_2_auto_toga_b: Arinc429Parameter<bool>,
    eng_1_limit_mode_soft_ga_a: Arinc429Parameter<bool>,
    eng_1_limit_mode_soft_ga_b: Arinc429Parameter<bool>,
    eng_2_limit_mode_soft_ga_a: Arinc429Parameter<bool>,
    eng_2_limit_mode_soft_ga_b: Arinc429Parameter<bool>,
    altitude_1: Arinc429Parameter<Length>,
    altitude_2: Arinc429Parameter<Length>,
    altitude_3: Arinc429Parameter<Length>,
    alti_select: Arinc429Parameter<Length>,
    alti_select_chg: Arinc429Parameter<bool>,
    ap1_engd_com: DiscreteParameter,
    ap1_engd_mon: DiscreteParameter,
    ap2_engd_com: DiscreteParameter,
    ap2_engd_mon: DiscreteParameter,
    instinc_discnct_1ap_engd: DiscreteParameter,
    instinc_discnct_2ap_engd: DiscreteParameter,
    capt_mw_cancel_on: DiscreteParameter,
    fo_mw_cancel_on: DiscreteParameter,
    capt_mc_cancel_on: DiscreteParameter,
    fo_mc_cancel_on: DiscreteParameter,
    blue_sys_lo_pr: DiscreteParameter,
    yellow_sys_lo_pr: DiscreteParameter,
    green_sys_lo_pr: DiscreteParameter,
    tcas_engaged: Arinc429Parameter<bool>,
    gs_mode_on_1: Arinc429Parameter<bool>,
    gs_mode_on_2: Arinc429Parameter<bool>,
    gpws_modes_on: DiscreteParameter,
    gs_visual_alert_on: DiscreteParameter,
    tcas_aural_advisory_output: DiscreteParameter,
    decision_height_1: Arinc429Parameter<Length>,
    decision_height_2: Arinc429Parameter<Length>,
    hundred_above_for_mda_mdh_request_1: DiscreteParameter,
    hundred_above_for_mda_mdh_request_2: DiscreteParameter,
    minimum_for_mda_mdh_request_1: DiscreteParameter,
    minimum_for_mda_mdh_request_2: DiscreteParameter,
    decision_height_code_a: DiscreteParameter,
    decision_height_code_b: DiscreteParameter,
    decision_height_plus_100_ft_code_a: DiscreteParameter,
    decision_height_plus_100_ft_code_b: DiscreteParameter,
    auto_call_out_2500_ft: DiscreteParameter,
    auto_call_out_2500b: DiscreteParameter,
    auto_call_out_2000_ft: DiscreteParameter,
    auto_call_out_1000_ft: DiscreteParameter,
    auto_call_out_500_ft: DiscreteParameter,
    auto_call_out_500_ft_glide_deviation: DiscreteParameter,
    auto_call_out_400_ft: DiscreteParameter,
    auto_call_out_300_ft: DiscreteParameter,
    auto_call_out_200_ft: DiscreteParameter,
    auto_call_out_100_ft: DiscreteParameter,
    auto_call_out_50_ft: DiscreteParameter,
    auto_call_out_40_ft: DiscreteParameter,
    auto_call_out_30_ft: DiscreteParameter,
    auto_call_out_20_ft: DiscreteParameter,
    auto_call_out_10_ft: DiscreteParameter,
    auto_call_out_5_ft: DiscreteParameter,
    glide_deviation_1: Arinc429Parameter<Ratio>,
    glide_deviation_2: Arinc429Parameter<Ratio>,
    land_trk_mode_on_1: Arinc429Parameter<bool>,
    land_trk_mode_on_2: Arinc429Parameter<bool>,
    athr_engaged: Arinc429Parameter<bool>,
}

impl A320FWCParameterTable {
    pub fn new() -> Self {
        Self {
            fwc_ident_side1: DiscreteParameter::new_inv(false),
            fwc_ident_side2: DiscreteParameter::new_inv(false),
            lh_lg_compressed_1: Arinc429Parameter::new_inv(false),
            lh_lg_compressed_2: Arinc429Parameter::new_inv(false),
            ess_lh_lg_compressed: DiscreteParameter::new_inv(false),
            norm_lh_lg_compressed: DiscreteParameter::new_inv(false),
            lh_gear_down_lock_1: Arinc429Parameter::new_inv(false),
            lh_gear_down_lock_2: Arinc429Parameter::new_inv(false),
            rh_gear_down_lock_1: Arinc429Parameter::new_inv(false),
            rh_gear_down_lock_2: Arinc429Parameter::new_inv(false),
            nose_gear_down_lock_1: Arinc429Parameter::new_inv(false),
            nose_gear_down_lock_2: Arinc429Parameter::new_inv(false),
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
            eng_2_fire_pb_out: DiscreteParameter::new_inv(false),
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
            eng_1_auto_toga_a: Arinc429Parameter::new_inv(false),
            eng_1_auto_toga_b: Arinc429Parameter::new_inv(false),
            eng_2_auto_toga_a: Arinc429Parameter::new_inv(false),
            eng_2_auto_toga_b: Arinc429Parameter::new_inv(false),
            eng_1_limit_mode_soft_ga_a: Arinc429Parameter::new_inv(false),
            eng_1_limit_mode_soft_ga_b: Arinc429Parameter::new_inv(false),
            eng_2_limit_mode_soft_ga_a: Arinc429Parameter::new_inv(false),
            eng_2_limit_mode_soft_ga_b: Arinc429Parameter::new_inv(false),
            altitude_1: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            altitude_2: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            altitude_3: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            alti_select: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            alti_select_chg: Arinc429Parameter::new_inv(false),
            ap1_engd_com: DiscreteParameter::new_inv(false),
            ap1_engd_mon: DiscreteParameter::new_inv(false),
            ap2_engd_com: DiscreteParameter::new_inv(false),
            ap2_engd_mon: DiscreteParameter::new_inv(false),
            instinc_discnct_1ap_engd: DiscreteParameter::new_inv(false),
            instinc_discnct_2ap_engd: DiscreteParameter::new_inv(false),
            capt_mw_cancel_on: DiscreteParameter::new_inv(false),
            fo_mw_cancel_on: DiscreteParameter::new_inv(false),
            capt_mc_cancel_on: DiscreteParameter::new_inv(false),
            fo_mc_cancel_on: DiscreteParameter::new_inv(false),
            blue_sys_lo_pr: DiscreteParameter::new_inv(false),
            yellow_sys_lo_pr: DiscreteParameter::new_inv(false),
            green_sys_lo_pr: DiscreteParameter::new_inv(false),
            tcas_engaged: Arinc429Parameter::new_inv(false),
            gs_mode_on_1: Arinc429Parameter::new_inv(false),
            gs_mode_on_2: Arinc429Parameter::new_inv(false),
            gpws_modes_on: DiscreteParameter::new_inv(false),
            gs_visual_alert_on: DiscreteParameter::new_inv(false),
            tcas_aural_advisory_output: DiscreteParameter::new_inv(false),
            decision_height_1: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            decision_height_2: Arinc429Parameter::new_inv(Length::new::<foot>(0.0)),
            hundred_above_for_mda_mdh_request_1: DiscreteParameter::new_inv(false),
            hundred_above_for_mda_mdh_request_2: DiscreteParameter::new_inv(false),
            minimum_for_mda_mdh_request_1: DiscreteParameter::new_inv(false),
            minimum_for_mda_mdh_request_2: DiscreteParameter::new_inv(false),
            decision_height_code_a: DiscreteParameter::new(true), // TODO
            decision_height_code_b: DiscreteParameter::new(true), // TODO
            decision_height_plus_100_ft_code_a: DiscreteParameter::new(true), // TODO
            decision_height_plus_100_ft_code_b: DiscreteParameter::new(true), // TODO
            auto_call_out_2500_ft: DiscreteParameter::new(true),  // TODO
            auto_call_out_2500b: DiscreteParameter::new(false),   // TODO
            auto_call_out_2000_ft: DiscreteParameter::new(true),  // TODO
            auto_call_out_1000_ft: DiscreteParameter::new(true),  // TODO
            auto_call_out_500_ft: DiscreteParameter::new(true),   // TODO
            auto_call_out_500_ft_glide_deviation: DiscreteParameter::new(false), // TODO
            auto_call_out_400_ft: DiscreteParameter::new(true),   // TODO
            auto_call_out_300_ft: DiscreteParameter::new(true),   // TODO
            auto_call_out_200_ft: DiscreteParameter::new(true),   // TODO
            auto_call_out_100_ft: DiscreteParameter::new(true),   // TODO
            auto_call_out_50_ft: DiscreteParameter::new(true),    // TODO
            auto_call_out_40_ft: DiscreteParameter::new(true),    // TODO
            auto_call_out_30_ft: DiscreteParameter::new(true),    // TODO
            auto_call_out_20_ft: DiscreteParameter::new(true),    // TODO
            auto_call_out_10_ft: DiscreteParameter::new(true),    // TODO
            auto_call_out_5_ft: DiscreteParameter::new(true),     // TODO
            glide_deviation_1: Arinc429Parameter::new_inv(Ratio::new::<ratio>(0.0)),
            glide_deviation_2: Arinc429Parameter::new_inv(Ratio::new::<ratio>(0.0)),
            land_trk_mode_on_1: Arinc429Parameter::new_inv(false),
            land_trk_mode_on_2: Arinc429Parameter::new_inv(false),
            athr_engaged: Arinc429Parameter::new_inv(false),
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

    pub(super) fn set_altitude_1(&mut self, altitude: Arinc429Parameter<Length>) {
        self.altitude_1 = altitude;
    }

    pub(super) fn set_altitude_2(&mut self, altitude: Arinc429Parameter<Length>) {
        self.altitude_2 = altitude;
    }

    pub(super) fn set_altitude_3(&mut self, altitude: Arinc429Parameter<Length>) {
        self.altitude_3 = altitude;
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

    pub(super) fn set_lh_gear_down_lock_1(&mut self, compressed: Arinc429Parameter<bool>) {
        self.lh_gear_down_lock_1 = compressed;
    }

    pub(super) fn set_lh_gear_down_lock_2(&mut self, compressed: Arinc429Parameter<bool>) {
        self.lh_gear_down_lock_2 = compressed;
    }

    pub(super) fn set_rh_gear_down_lock_1(&mut self, compressed: Arinc429Parameter<bool>) {
        self.rh_gear_down_lock_1 = compressed;
    }

    pub(super) fn set_rh_gear_down_lock_2(&mut self, compressed: Arinc429Parameter<bool>) {
        self.rh_gear_down_lock_2 = compressed;
    }

    pub(super) fn set_nose_gear_down_lock_1(&mut self, compressed: Arinc429Parameter<bool>) {
        self.nose_gear_down_lock_1 = compressed;
    }

    pub(super) fn set_nose_gear_down_lock_2(&mut self, compressed: Arinc429Parameter<bool>) {
        self.nose_gear_down_lock_2 = compressed;
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

    pub(super) fn set_eng_2_fire_pb_out(&mut self, fire_pb_out: DiscreteParameter) {
        self.eng_2_fire_pb_out = fire_pb_out;
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

    pub(super) fn set_eng1_tla_fto_a(&mut self, fto: Arinc429Parameter<bool>) {
        self.eng1_tla_fto_a = fto
    }

    pub(super) fn set_eng1_tla_fto_b(&mut self, fto: Arinc429Parameter<bool>) {
        self.eng1_tla_fto_b = fto
    }

    pub(super) fn set_eng2_tla_fto_a(&mut self, fto: Arinc429Parameter<bool>) {
        self.eng2_tla_fto_a = fto
    }

    pub(super) fn set_eng2_tla_fto_b(&mut self, fto: Arinc429Parameter<bool>) {
        self.eng2_tla_fto_b = fto
    }

    pub(super) fn set_eng1_channel_a_in_control(&mut self, in_control: Arinc429Parameter<bool>) {
        self.eng1_channel_a_in_control = in_control;
    }

    pub(super) fn set_eng1_channel_b_in_control(&mut self, in_control: Arinc429Parameter<bool>) {
        self.eng1_channel_b_in_control = in_control;
    }

    pub(super) fn set_eng2_channel_a_in_control(&mut self, in_control: Arinc429Parameter<bool>) {
        self.eng2_channel_a_in_control = in_control;
    }
    pub(super) fn set_eng2_channel_b_in_control(&mut self, in_control: Arinc429Parameter<bool>) {
        self.eng2_channel_b_in_control = in_control;
    }

    pub(super) fn set_ap1_engd(&mut self, com: DiscreteParameter, mon: DiscreteParameter) {
        self.ap1_engd_com = com;
        self.ap1_engd_mon = mon;
    }

    pub(super) fn set_ap2_engd(&mut self, com: DiscreteParameter, mon: DiscreteParameter) {
        self.ap2_engd_com = com;
        self.ap2_engd_mon = mon;
    }

    pub(super) fn set_alti_select(&mut self, alti_select: Arinc429Parameter<Length>) {
        self.alti_select = alti_select;
    }

    pub(super) fn set_alti_select_chg(&mut self, alti_select_chg: Arinc429Parameter<bool>) {
        self.alti_select_chg = alti_select_chg;
    }

    pub(super) fn set_instinc_discnct_1ap_engd(&mut self, engd: DiscreteParameter) {
        self.instinc_discnct_1ap_engd = engd;
    }

    pub(super) fn set_instinc_discnct_2ap_engd(&mut self, engd: DiscreteParameter) {
        self.instinc_discnct_2ap_engd = engd;
    }

    pub(super) fn set_capt_mw_cancel_on(&mut self, capt_mw_cancel_on: DiscreteParameter) {
        self.capt_mw_cancel_on = capt_mw_cancel_on;
    }

    pub(super) fn set_fo_mw_cancel_on(&mut self, fo_mw_cancel_on: DiscreteParameter) {
        self.fo_mw_cancel_on = fo_mw_cancel_on;
    }

    pub(super) fn set_capt_mc_cancel_on(&mut self, capt_mc_cancel_on: DiscreteParameter) {
        self.capt_mc_cancel_on = capt_mc_cancel_on;
    }

    pub(super) fn set_fo_mc_cancel_on(&mut self, fo_mc_cancel_on: DiscreteParameter) {
        self.fo_mc_cancel_on = fo_mc_cancel_on;
    }

    pub(super) fn set_blue_sys_lo_pr(&mut self, blue_sys_lo_pr: DiscreteParameter) {
        self.blue_sys_lo_pr = blue_sys_lo_pr;
    }

    pub(super) fn set_yellow_sys_lo_pr(&mut self, yellow_sys_lo_pr: DiscreteParameter) {
        self.yellow_sys_lo_pr = yellow_sys_lo_pr;
    }

    pub(super) fn set_green_sys_lo_pr(&mut self, green_sys_lo_pr: DiscreteParameter) {
        self.green_sys_lo_pr = green_sys_lo_pr;
    }

    pub(super) fn set_tcas_engaged(&mut self, tcas_engaged: Arinc429Parameter<bool>) {
        self.tcas_engaged = tcas_engaged;
    }

    pub(super) fn set_gs_mode_on_1(&mut self, gs_mode_on_1: Arinc429Parameter<bool>) {
        self.gs_mode_on_1 = gs_mode_on_1;
    }

    pub(super) fn set_gs_mode_on_2(&mut self, gs_mode_on_2: Arinc429Parameter<bool>) {
        self.gs_mode_on_2 = gs_mode_on_2;
    }

    pub(super) fn set_decision_height_1(&mut self, decision_height_1: Arinc429Parameter<Length>) {
        self.decision_height_1 = decision_height_1;
    }
    pub(super) fn set_decision_height_2(&mut self, decision_height_2: Arinc429Parameter<Length>) {
        self.decision_height_2 = decision_height_2;
    }
    pub(super) fn set_hundred_above_for_mda_mdh_request_1(
        &mut self,
        hundred_above_for_mda_mdh_request_1: DiscreteParameter,
    ) {
        self.hundred_above_for_mda_mdh_request_1 = hundred_above_for_mda_mdh_request_1;
    }

    pub(super) fn set_hundred_above_for_mda_mdh_request_2(
        &mut self,
        hundred_above_for_mda_mdh_request_2: DiscreteParameter,
    ) {
        self.hundred_above_for_mda_mdh_request_2 = hundred_above_for_mda_mdh_request_2;
    }

    pub(super) fn set_minimum_for_mda_mdh_request_1(
        &mut self,
        minimum_for_mda_mdh_request_1: DiscreteParameter,
    ) {
        self.minimum_for_mda_mdh_request_1 = minimum_for_mda_mdh_request_1;
    }

    pub(super) fn set_minimum_for_mda_mdh_request_2(
        &mut self,
        minimum_for_mda_mdh_request_2: DiscreteParameter,
    ) {
        self.minimum_for_mda_mdh_request_2 = minimum_for_mda_mdh_request_2;
    }

    pub(super) fn set_land_trk_mode_on_1(&mut self, land_trk_mode_on_1: Arinc429Parameter<bool>) {
        self.land_trk_mode_on_1 = land_trk_mode_on_1;
    }

    pub(super) fn set_land_trk_mode_on_2(&mut self, land_trk_mode_on_2: Arinc429Parameter<bool>) {
        self.land_trk_mode_on_2 = land_trk_mode_on_2;
    }

    pub(super) fn set_athr_engaged(&mut self, athr_engaged: Arinc429Parameter<bool>) {
        self.athr_engaged = athr_engaged;
    }
}
impl FwcIdentSide1 for A320FWCParameterTable {
    fn fwc_ident_side1(&self) -> &DiscreteParameter {
        &self.fwc_ident_side1
    }
}
impl FwcIdentSide2 for A320FWCParameterTable {
    fn fwc_ident_side2(&self) -> &DiscreteParameter {
        &self.fwc_ident_side2
    }
}
impl LhLgCompressed for A320FWCParameterTable {
    fn lh_lg_compressed(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.lh_lg_compressed_1,
            2 => &self.lh_lg_compressed_2,
            _ => panic!(),
        }
    }
}
impl EssLhLgCompressed for A320FWCParameterTable {
    fn ess_lh_lg_compressed(&self) -> &DiscreteParameter {
        &self.ess_lh_lg_compressed
    }
}
impl NormLhLgCompressed for A320FWCParameterTable {
    fn norm_lh_lg_compressed(&self) -> &DiscreteParameter {
        &self.norm_lh_lg_compressed
    }
}
impl LhGearDownLock for A320FWCParameterTable {
    fn lh_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.lh_gear_down_lock_1,
            2 => &self.lh_gear_down_lock_2,
            _ => panic!(),
        }
    }
}
impl RhGearDownLock for A320FWCParameterTable {
    fn rh_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.rh_gear_down_lock_1,
            2 => &self.rh_gear_down_lock_2,
            _ => panic!(),
        }
    }
}
impl NoseGearDownLock for A320FWCParameterTable {
    fn nose_gear_down_lock(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.nose_gear_down_lock_1,
            2 => &self.nose_gear_down_lock_2,
            _ => panic!(),
        }
    }
}
impl RadioHeight for A320FWCParameterTable {
    fn radio_height(&self, index: u8) -> &Arinc429Parameter<Length> {
        match index {
            1 => &self.radio_height_1,
            2 => &self.radio_height_2,
            _ => panic!(),
        }
    }
}
impl ComputedSpeed for A320FWCParameterTable {
    fn computed_speed(&self, index: u8) -> &Arinc429Parameter<Velocity> {
        match index {
            1 => &self.computed_speed_1,
            2 => &self.computed_speed_2,
            3 => &self.computed_speed_3,
            _ => panic!(),
        }
    }
}
impl Eng1MasterLeverSelectOn for A320FWCParameterTable {
    fn eng1_master_lever_select_on(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_master_lever_select_on
    }
}

impl Eng2MasterLeverSelectOn for A320FWCParameterTable {
    fn eng2_master_lever_select_on(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_master_lever_select_on
    }
}

impl Eng1CoreSpeedAtOrAboveIdle for A320FWCParameterTable {
    fn eng1_core_speed_at_or_above_idle(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng1_core_speed_at_or_above_idle_a,
            2 => &self.eng1_core_speed_at_or_above_idle_b,
            _ => panic!(),
        }
    }
}

impl Eng2CoreSpeedAtOrAboveIdle for A320FWCParameterTable {
    fn eng2_core_speed_at_or_above_idle(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng2_core_speed_at_or_above_idle_a,
            2 => &self.eng2_core_speed_at_or_above_idle_b,
            _ => panic!(),
        }
    }
}

impl Eng1FirePbOut for A320FWCParameterTable {
    fn eng_1_fire_pb_out(&self) -> &DiscreteParameter {
        &self.eng_1_fire_pb_out
    }
}

impl ToConfigTest for A320FWCParameterTable {
    fn to_config_test(&self) -> &Arinc429Parameter<bool> {
        &self.to_config_test
    }
}

impl Eng1Tla for A320FWCParameterTable {
    fn eng1_tla(&self, index: u8) -> &Arinc429Parameter<Angle> {
        match index {
            1 => &self.eng1_tla_a,
            2 => &self.eng1_tla_b,
            _ => panic!(),
        }
    }
}

impl Eng2Tla for A320FWCParameterTable {
    fn eng2_tla(&self, index: u8) -> &Arinc429Parameter<Angle> {
        match index {
            1 => &self.eng2_tla_a,
            2 => &self.eng2_tla_b,
            _ => panic!(),
        }
    }
}

impl Eng1TlaFto for A320FWCParameterTable {
    fn eng1_tla_fto(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng1_tla_fto_a,
            2 => &self.eng1_tla_fto_b,
            _ => panic!(),
        }
    }
}

impl Eng2TlaFto for A320FWCParameterTable {
    fn eng2_tla_fto(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng2_tla_fto_a,
            2 => &self.eng2_tla_fto_b,
            _ => panic!(),
        }
    }
}

impl Eng1N1SelectedActual for A320FWCParameterTable {
    fn eng1_n1_selected_actual(&self, index: u8) -> &Arinc429Parameter<Ratio> {
        match index {
            1 => &self.eng1_n1_selected_actual_a,
            2 => &self.eng1_n1_selected_actual_b,
            _ => panic!(),
        }
    }
}
impl Eng2N1SelectedActual for A320FWCParameterTable {
    fn eng2_n1_selected_actual(&self, index: u8) -> &Arinc429Parameter<Ratio> {
        match index {
            1 => &self.eng2_n1_selected_actual_a,
            2 => &self.eng2_n1_selected_actual_b,
            _ => panic!(),
        }
    }
}
impl Tla1IdlePwr for A320FWCParameterTable {
    fn tla1_idle_pwr(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.tla1_idle_pwr_a,
            2 => &self.tla1_idle_pwr_b,
            _ => panic!(),
        }
    }
}
impl Tla2IdlePwr for A320FWCParameterTable {
    fn tla2_idle_pwr(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.tla2_idle_pwr_a,
            2 => &self.tla2_idle_pwr_b,
            _ => panic!(),
        }
    }
}
impl Eng1ChannelInControl for A320FWCParameterTable {
    fn eng1_channel_a_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_channel_a_in_control
    }

    fn eng1_channel_b_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng1_channel_b_in_control
    }
}
impl Eng2ChannelInControl for A320FWCParameterTable {
    fn eng2_channel_a_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_channel_a_in_control
    }

    fn eng2_channel_b_in_control(&self) -> &Arinc429Parameter<bool> {
        &self.eng2_channel_a_in_control
    }
}
impl Eng1AutoToga for A320FWCParameterTable {
    fn eng_1_auto_toga(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng_1_auto_toga_a,
            2 => &self.eng_1_auto_toga_b,
            _ => panic!(),
        }
    }
}
impl Eng2AutoToga for A320FWCParameterTable {
    fn eng_2_auto_toga(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng_2_auto_toga_a,
            2 => &self.eng_2_auto_toga_b,
            _ => panic!(),
        }
    }
}
impl Eng1LimitModeSoftGa for A320FWCParameterTable {
    fn eng_1_limit_mode_soft_ga(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng_1_limit_mode_soft_ga_a,
            2 => &self.eng_1_limit_mode_soft_ga_b,
            _ => panic!(),
        }
    }
}
impl Eng2LimitModeSoftGa for A320FWCParameterTable {
    fn eng_2_limit_mode_soft_ga(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.eng_2_limit_mode_soft_ga_a,
            2 => &self.eng_2_limit_mode_soft_ga_b,
            _ => panic!(),
        }
    }
}
impl AltitudeParameter for A320FWCParameterTable {
    fn altitude(&self, index: u8) -> &Arinc429Parameter<Length> {
        match index {
            1 => &self.altitude_1,
            2 => &self.altitude_2,
            3 => &self.altitude_3,
            _ => panic!(),
        }
    }
}
impl AltiSelect for A320FWCParameterTable {
    fn alti_select(&self) -> &Arinc429Parameter<Length> {
        &self.alti_select
    }
}
impl AltSelectChg for A320FWCParameterTable {
    fn alt_select_chg(&self) -> &Arinc429Parameter<bool> {
        &self.alti_select_chg
    }
}
impl Ap1Engd for A320FWCParameterTable {
    fn ap1_engd_com(&self) -> &DiscreteParameter {
        &self.ap1_engd_com
    }

    fn ap1_engd_mon(&self) -> &DiscreteParameter {
        &self.ap1_engd_mon
    }
}
impl Ap2Engd for A320FWCParameterTable {
    fn ap2_engd_com(&self) -> &DiscreteParameter {
        &self.ap2_engd_com
    }

    fn ap2_engd_mon(&self) -> &DiscreteParameter {
        &self.ap2_engd_mon
    }
}
impl InstincDiscnct1ApEngd for A320FWCParameterTable {
    fn instinc_discnct_1ap_engd(&self) -> &DiscreteParameter {
        &self.instinc_discnct_1ap_engd
    }
}
impl InstincDiscnct2ApEngd for A320FWCParameterTable {
    fn instinc_discnct_2ap_engd(&self) -> &DiscreteParameter {
        &self.instinc_discnct_2ap_engd
    }
}

impl CaptMwCancelOn for A320FWCParameterTable {
    fn capt_mw_cancel_on(&self) -> &DiscreteParameter {
        &self.capt_mw_cancel_on
    }
}

impl FoMwCancelOn for A320FWCParameterTable {
    fn fo_mw_cancel_on(&self) -> &DiscreteParameter {
        &self.fo_mw_cancel_on
    }
}

impl CaptMcCancelOn for A320FWCParameterTable {
    fn capt_mc_cancel_on(&self) -> &DiscreteParameter {
        &self.capt_mc_cancel_on
    }
}

impl FoMcCancelOn for A320FWCParameterTable {
    fn fo_mc_cancel_on(&self) -> &DiscreteParameter {
        &self.fo_mc_cancel_on
    }
}

impl BlueSysLoPr for A320FWCParameterTable {
    fn blue_sys_lo_pr(&self) -> &DiscreteParameter {
        &self.blue_sys_lo_pr
    }
}

impl YellowSysLoPr for A320FWCParameterTable {
    fn yellow_sys_lo_pr(&self) -> &DiscreteParameter {
        &self.yellow_sys_lo_pr
    }
}

impl GreenSysLoPr for A320FWCParameterTable {
    fn green_sys_lo_pr(&self) -> &DiscreteParameter {
        &self.green_sys_lo_pr
    }
}

impl TcasEngaged for A320FWCParameterTable {
    fn tcas_engaged(&self) -> &Arinc429Parameter<bool> {
        &self.tcas_engaged
    }
}

impl GsModeOn for A320FWCParameterTable {
    fn gs_mode_on(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.gs_mode_on_1,
            2 => &self.gs_mode_on_2,
            _ => panic!(),
        }
    }
}

impl GpwsModesOn for A320FWCParameterTable {
    fn gpws_modes_on(&self) -> &DiscreteParameter {
        &self.gpws_modes_on
    }
}

impl GsVisualAlertOn for A320FWCParameterTable {
    fn gs_visual_alert_on(&self) -> &DiscreteParameter {
        &self.gs_visual_alert_on
    }
}

impl TcasAuralAdvisaryOutput for A320FWCParameterTable {
    fn tcas_aural_advisory_output(&self) -> &DiscreteParameter {
        &self.tcas_aural_advisory_output
    }
}

impl DecisionHeight for A320FWCParameterTable {
    fn decision_height(&self, index: u8) -> &Arinc429Parameter<Length> {
        match index {
            1 => &self.decision_height_1,
            2 => &self.decision_height_2,
            _ => panic!(),
        }
    }
}

impl HundredAboveForMdaMdhRequest for A320FWCParameterTable {
    fn hundred_above_for_mda_mdh_request(&self, index: u8) -> &DiscreteParameter {
        match index {
            1 => &self.hundred_above_for_mda_mdh_request_1,
            2 => &self.hundred_above_for_mda_mdh_request_2,
            _ => panic!(),
        }
    }
}

impl MinimumForMdaMdhRequest for A320FWCParameterTable {
    fn minimum_for_mda_mdh_request(&self, index: u8) -> &DiscreteParameter {
        match index {
            1 => &self.minimum_for_mda_mdh_request_1,
            2 => &self.minimum_for_mda_mdh_request_2,
            _ => panic!(),
        }
    }
}

impl AutoCalloutPins for A320FWCParameterTable {
    /// TODO move to vars

    fn decision_height_code_a(&self) -> &DiscreteParameter {
        &self.decision_height_code_a
    }

    fn decision_height_code_b(&self) -> &DiscreteParameter {
        &self.decision_height_code_b
    }

    fn decision_height_plus_100_ft_code_a(&self) -> &DiscreteParameter {
        &self.decision_height_plus_100_ft_code_a
    }

    fn decision_height_plus_100_ft_code_b(&self) -> &DiscreteParameter {
        &self.decision_height_plus_100_ft_code_b
    }

    fn auto_call_out_2500_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_2500_ft
    }

    fn auto_call_out_2500b(&self) -> &DiscreteParameter {
        &self.auto_call_out_2500b
    }

    fn auto_call_out_2000_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_2000_ft
    }

    fn auto_call_out_1000_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_1000_ft
    }

    fn auto_call_out_500_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_500_ft
    }

    fn auto_call_out_500_ft_glide_deviation(&self) -> &DiscreteParameter {
        &self.auto_call_out_500_ft_glide_deviation
    }

    fn auto_call_out_400_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_400_ft
    }

    fn auto_call_out_300_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_300_ft
    }

    fn auto_call_out_200_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_200_ft
    }

    fn auto_call_out_100_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_100_ft
    }

    fn auto_call_out_50_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_50_ft
    }

    fn auto_call_out_40_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_40_ft
    }

    fn auto_call_out_30_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_30_ft
    }

    fn auto_call_out_20_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_20_ft
    }

    fn auto_call_out_10_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_10_ft
    }

    fn auto_call_out_5_ft(&self) -> &DiscreteParameter {
        &self.auto_call_out_5_ft
    }
}

impl GlideDeviation for A320FWCParameterTable {
    fn glide_deviation(&self, index: u8) -> &Arinc429Parameter<Ratio> {
        match index {
            1 => &self.glide_deviation_1,
            2 => &self.glide_deviation_2,
            _ => panic!(),
        }
    }
}

impl LandTrkModeOn for A320FWCParameterTable {
    fn land_trk_mode_on(&self, index: u8) -> &Arinc429Parameter<bool> {
        match index {
            1 => &self.land_trk_mode_on_1,
            2 => &self.land_trk_mode_on_2,
            _ => panic!(),
        }
    }
}

impl AThrEngaged for A320FWCParameterTable {
    fn athr_engaged(&self) -> &Arinc429Parameter<bool> {
        &self.athr_engaged
    }
}

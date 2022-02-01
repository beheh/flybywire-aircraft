use std::time::Duration;
use uom::si::angle::degree;

use super::*;
use systems::flight_warning::logic::*;
use systems::flight_warning::parameters::{SignStatusMatrix, Value};
use systems::flight_warning::utils::FwcSsm;
use uom::si::f64::*;
use uom::si::length::foot;
use uom::si::ratio::ratio;

pub(in crate::flight_warning::runtime) trait GeneralDhDtPositive {
    fn dh_positive(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct GeneralDhDtPositiveActivation {
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

pub(in crate::flight_warning::runtime) trait AudioGenerated {
    fn minimum_generated(&self) -> bool;

    fn hundred_above_generated(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AudioGeneratedActivation {
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

pub(in crate::flight_warning::runtime) trait DecisionHeightVal {
    fn radio_height_val(&self) -> Length;

    fn decision_height_val(&self) -> Length;

    fn decision_inv(&self) -> bool;
}

/// This is the final sheet to decide that the C Chord should be played. It currently contains some
/// rudimentary cancel logic, which will be replaced with a generalized system in future.
#[derive(Default)]
pub(in crate::flight_warning::runtime) struct DecisionHeightValActivation {
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

pub(in crate::flight_warning::runtime) trait MdaMdhInbition {
    fn aco_mda_mdh_inhib(&self) -> bool;

    fn aco_dh_inhib(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct MdaMdhInbitionActivation {
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

pub(in crate::flight_warning::runtime) trait HundredAbove {
    fn dh_hundred_above(&self) -> bool;

    fn ha_generated(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct HundredAboveActivation {
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

pub(in crate::flight_warning::runtime) trait Minimum {
    fn dh_generated(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct MinimumActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThreshold1 {
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

pub(in crate::flight_warning::runtime) struct AltitudeThreshold1Activation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThreshold2 {
    fn alt_40_ft(&self) -> bool;
    fn alt_30_ft(&self) -> bool;
    fn alt_20_ft(&self) -> bool;
    fn alt_10_ft(&self) -> bool;
    fn alt_5_ft(&self) -> bool;
    fn alt_inf_3_ft(&self) -> bool;
    fn dh_inhibition(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeThreshold2Activation {
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

pub(in crate::flight_warning::runtime) trait HoistedGpwsInhibition {
    fn gpws_inhibition(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct HoistedGpwsInhibitionActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThreshold3 {
    fn threshold_detection(&self) -> bool;
    fn gpws_inhibition(&self) -> bool;
    fn to_and_ground_detection(&self) -> bool;
    fn renvoi1(&self) -> bool;
    fn renvoi2(&self) -> bool;
    fn renvoi3(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeThreshold3Activation {
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

pub(in crate::flight_warning::runtime) trait AutomaticCallOutInhibition {
    fn auto_call_out_inhib(&self) -> bool;
    fn retard_inhib(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AutomaticCallOutInhibitionActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThresholdTriggers1 {
    fn seuil_2500_ft(&self) -> bool;
    fn seuil_2500b_ft(&self) -> bool;
    fn seuil_2000_ft(&self) -> bool;
    fn seuil_1000_ft(&self) -> bool;
    fn seuil_500_ft(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeThresholdTriggers1Activation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThresholdTriggers2 {
    fn seuil_400_ft(&self) -> bool;
    fn seuil_300_ft(&self) -> bool;
    fn seuil_200_ft(&self) -> bool;
    fn seuil_100_ft(&self) -> bool;
    fn seuil_50_ft(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AltitudeThresholdTriggers2Activation {
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

pub(in crate::flight_warning::runtime) trait AltitudeThresholdTriggers3 {
    fn seuil_40_ft(&self) -> bool;
    fn seuil_30_ft(&self) -> bool;
    fn seuil_20_ft(&self) -> bool;
    fn seuil_10_ft(&self) -> bool;
    fn seuil_5_ft(&self) -> bool;
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AltitudeThresholdTriggers3Activation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout2500FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout2500BFtAnnounceActivation {
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
        let seuil_2500b_ft = altitude_trigger_sheet.seuil_2500b_ft();
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout2000FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout1000FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout500FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout400FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout300FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout200FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) struct AltitudeCallout100FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCallout50FtAnnounce {
    fn audio_50(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout50FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCallout40FtAnnounce {
    fn audio_40(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout40FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCallout30FtAnnounce {
    fn audio_30(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout30FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCallout20FtAnnounce {
    fn audio_20(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout20FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCallout10FtAnnounce {
    fn audio_10(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout10FtAnnounceActivation {
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

        let active = ((!any_ap_in_land && athr_engaged) || !athr_engaged)
            && pulse_out
            && !self.prec.value()
            && !retard_inhibtion
            && !audio_5_sheet.audio_5();

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

pub(in crate::flight_warning::runtime) trait AltitudeCallout5FtAnnounce {
    fn audio_5(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AltitudeCallout5FtAnnounceActivation {
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

pub(in crate::flight_warning::runtime) trait AltitudeCalloutThresholdDetection {
    fn non_inhibited_threshold_detection(&self) -> bool;
}

pub(in crate::flight_warning::runtime) trait AutoCallOutTwentyRetardAnnounce {
    fn retard_toga(&self) -> bool;
    fn toga(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct AutoCallOutTwentyRetardAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    toga: bool,
    retard_toga: bool,
    aco_twenty_retard: bool,
}

impl Default for AutoCallOutTwentyRetardAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: PreceedingValueNode::new(),
            toga: false,
            retard_toga: false,
            aco_twenty_retard: false,
        }
    }
}

impl AutoCallOutTwentyRetardAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1TlaCfm + Eng2TlaCfm + LandTrkModeOn + AThrEngaged),
        inhibit_sheet: &impl AutomaticCallOutInhibition,
        cfm_tla_sheet: &impl TlaAtMctOrFlexToCfm,
        flight_phase_sheet: &impl FlightPhasesGround,
        trigger_sheet: &impl AltitudeThresholdTriggers3,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
    ) {
        let cfm_tla_cond = signals.eng1_tla(1).value() > Angle::new::<degree>(43.3)
            || signals.eng2_tla(1).value() > Angle::new::<degree>(43.3);

        let cfm_mct_cond = flight_phase_sheet.phase_8()
            && (cfm_tla_sheet.eng_1_sup_mct_cfm() || cfm_tla_sheet.eng_2_sup_mct_cfm());

        let toga_cond = cfm_tla_cond || cfm_mct_cond;
        let retard_inhib = inhibit_sheet.retard_inhib();

        self.retard_toga = retard_inhib || toga_cond;
        self.toga = toga_cond || inhibit_sheet.auto_call_out_inhib();

        let ap1_in_land = ap_sheet.ap1_engd() && signals.land_trk_mode_on(1).value();
        let ap2_in_land = ap_sheet.ap2_engd() && signals.land_trk_mode_on(2).value();
        let any_ap_in_land = ap1_in_land || ap2_in_land;
        let athr_engaged = signals.athr_engaged().value();
        let athr_cond = !athr_engaged || !any_ap_in_land;

        let twenty_retard =
            !self.toga && trigger_sheet.seuil_20_ft() && athr_cond && !self.prec.value();

        let pulse_out = self.pulse.update(twenty_retard);
        self.aco_twenty_retard = pulse_out;
        self.prec.update(self.mtrig.update(pulse_out, delta));
    }
}

impl AutoCallOutTwentyRetardAnnounce for AutoCallOutTwentyRetardAnnounceActivation {
    fn retard_toga(&self) -> bool {
        self.retard_toga
    }

    fn toga(&self) -> bool {
        self.toga
    }
}

impl WarningActivation for AutoCallOutTwentyRetardAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_twenty_retard
    }
}

pub(in crate::flight_warning::runtime) struct AutoCallOutTenRetardAnnounceActivation {
    pulse: PulseNode,
    mtrig: MonostableTriggerNode,
    prec: PreceedingValueNode,
    aco_twenty_retard: bool,
}

impl Default for AutoCallOutTenRetardAnnounceActivation {
    fn default() -> Self {
        Self {
            pulse: PulseNode::new(true),
            mtrig: MonostableTriggerNode::new(true, Duration::from_secs(2)),
            prec: PreceedingValueNode::new(),
            aco_twenty_retard: false,
        }
    }
}

impl AutoCallOutTenRetardAnnounceActivation {
    pub fn update(
        &mut self,
        delta: Duration,
        signals: &(impl Eng1TlaCfm + Eng2TlaCfm + LandTrkModeOn + AThrEngaged),
        inhibit_sheet: &impl AutomaticCallOutInhibition,
        twenty_sheet: &impl AutoCallOutTwentyRetardAnnounce,
        trigger_sheet: &impl AltitudeThresholdTriggers3,
        ap_sheet: &impl AutoFlightAutopilotOffVoluntary,
    ) {
        let ap1_in_land = ap_sheet.ap1_engd() && signals.land_trk_mode_on(1).value();
        let ap2_in_land = ap_sheet.ap2_engd() && signals.land_trk_mode_on(2).value();
        let any_ap_in_land = ap1_in_land || ap2_in_land;
        let athr_engaged = signals.athr_engaged().value();
        let athr_cond = athr_engaged && any_ap_in_land;

        let twenty_retard = !(twenty_sheet.toga() || inhibit_sheet.auto_call_out_inhib())
            && trigger_sheet.seuil_10_ft()
            && athr_cond
            && !self.prec.value();

        let pulse_out = self.pulse.update(twenty_retard);
        self.aco_twenty_retard = pulse_out;
        self.prec.update(self.mtrig.update(pulse_out, delta));
    }
}

impl WarningActivation for AutoCallOutTenRetardAnnounceActivation {
    fn warning(&self) -> bool {
        self.aco_twenty_retard
    }
}

#[derive(Default)]
pub(in crate::flight_warning::runtime) struct AltitudeCalloutThresholdDetectionActivation {
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

pub(in crate::flight_warning::runtime) trait IntermediateAudio {
    fn intermediate_call_out(&self) -> bool;
}

pub(in crate::flight_warning::runtime) struct IntermediateAudioActivation {
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

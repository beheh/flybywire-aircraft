use crate::flight_warning::parameters::A320FWCParameterTable;
use systems::flight_warning::parameters::*;
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;

pub struct A320FWCParameterTestBed {
    parameters: A320FWCParameterTable,
}
impl A320FWCParameterTestBed {
    pub fn new() -> Self {
        Self {
            parameters: A320FWCParameterTable::new(),
        }
    }

    pub fn and(self) -> Self {
        self
    }

    pub fn parameters(&self) -> &A320FWCParameterTable {
        &self.parameters
    }

    pub fn takeoff_config_test_pressed(mut self) -> Self {
        self.parameters.set_takeoff_config_test(true);
        self
    }

    pub fn on_ground(mut self) -> Self {
        self.ess_lh_lg_compressed()
            .norm_lh_lg_compressed()
            .lh_lg_compressed(1)
            .lh_lg_compressed(2)
            .radio_heights(Length::new::<foot>(0.0), Length::new::<foot>(0.0))
    }

    pub fn one_engine_running(mut self) -> Self {
        self.eng1_master_lever_select_on().eng1_at_or_above_idle()
    }

    pub fn engines_running(mut self) -> Self {
        self.eng1_master_lever_select_on()
            .eng1_at_or_above_idle()
            .eng2_master_lever_select_on()
            .eng2_at_or_above_idle()
    }

    pub fn engines_at_takeoff_power(mut self) -> Self {
        self.eng1_tla(Angle::new::<degree>(45.0))
            .eng2_tla(Angle::new::<degree>(45.0))
    }

    pub fn engines_at_idle(mut self) -> Self {
        self.eng1_tla(Angle::new::<degree>(0.0))
            .eng2_tla(Angle::new::<degree>(0.0))
    }

    pub fn computed_speeds(mut self, speed1: Velocity, speed2: Velocity, speed3: Velocity) -> Self {
        self.parameters
            .set_computed_speed_1(Arinc429Parameter::new(speed1));
        self.parameters
            .set_computed_speed_2(Arinc429Parameter::new(speed2));
        self.parameters
            .set_computed_speed_3(Arinc429Parameter::new(speed3));
        self
    }

    pub fn computed_speed_1(mut self, speed: Velocity) -> Self {
        self.parameters
            .set_computed_speed_1(Arinc429Parameter::new(speed));
        self
    }

    pub fn computed_speed_2(mut self, speed: Velocity) -> Self {
        self.parameters
            .set_computed_speed_2(Arinc429Parameter::new(speed));
        self
    }

    pub fn computed_speed_3(mut self, speed: Velocity) -> Self {
        self.parameters
            .set_computed_speed_3(Arinc429Parameter::new(speed));
        self
    }

    pub fn lh_lg_compressed(mut self, lgciu: usize) -> Self {
        match lgciu {
            1 => self
                .parameters
                .set_lh_lg_compressed_1(Arinc429Parameter::new(true)),
            2 => self
                .parameters
                .set_lh_lg_compressed_2(Arinc429Parameter::new(true)),
            _ => panic!(),
        }
        self
    }

    pub fn lh_lg_extended(mut self, lgciu: usize) -> Self {
        match lgciu {
            1 => self
                .parameters
                .set_lh_lg_compressed_1(Arinc429Parameter::new(false)),
            2 => self
                .parameters
                .set_lh_lg_compressed_2(Arinc429Parameter::new(false)),
            _ => panic!(),
        }
        self
    }

    pub fn ess_lh_lg_compressed(mut self) -> Self {
        self.parameters
            .set_ess_lh_lg_compressed(DiscreteParameter::new(true));
        self
    }

    pub fn norm_lh_lg_compressed(mut self) -> Self {
        self.parameters
            .set_norm_lh_lg_compressed(DiscreteParameter::new(true));
        self
    }

    pub fn radio_heights(mut self, height1: Length, height2: Length) -> Self {
        self.parameters
            .set_radio_height_1(Arinc429Parameter::new(height1));
        self.parameters
            .set_radio_height_2(Arinc429Parameter::new(height2));
        self
    }

    /// Simulates a flight at cruise, where the radio altimeters will not be able to receive a
    /// valid ground return and mark their data as NCD.
    pub fn radio_heights_at_cruise(mut self) -> Self {
        self.parameters
            .set_radio_height_1(Arinc429Parameter::new_ncd(Length::new::<foot>(10000.0)));
        self.parameters
            .set_radio_height_2(Arinc429Parameter::new_ncd(Length::new::<foot>(10000.0)));
        self
    }

    pub fn eng1_fire_pb_out(mut self) -> Self {
        self.parameters
            .set_eng_1_fire_pb_out(DiscreteParameter::new(true));
        self
    }

    pub fn eng1_master_lever_select_on(mut self) -> Self {
        self.parameters
            .set_eng1_master_lever_select_on(Arinc429Parameter::new(true));
        self.parameters
            .set_eng1_channel_a_in_control(Arinc429Parameter::new(true));
        self.parameters
            .set_eng1_channel_b_in_control(Arinc429Parameter::new(false));
        self
    }

    pub fn eng2_master_lever_select_on(mut self) -> Self {
        self.parameters
            .set_eng2_master_lever_select_on(Arinc429Parameter::new(true));
        self.parameters
            .set_eng2_channel_a_in_control(Arinc429Parameter::new(true));
        self.parameters
            .set_eng2_channel_b_in_control(Arinc429Parameter::new(false));
        self
    }

    pub fn eng1_at_or_above_idle(mut self) -> Self {
        self.parameters
            .set_eng1_core_speed_at_or_above_idle_a(Arinc429Parameter::new(true));
        self.parameters
            .set_eng1_core_speed_at_or_above_idle_b(Arinc429Parameter::new(true));
        self
    }

    pub fn eng2_at_or_above_idle(mut self) -> Self {
        self.parameters
            .set_eng2_core_speed_at_or_above_idle_a(Arinc429Parameter::new(true));
        self.parameters
            .set_eng2_core_speed_at_or_above_idle_b(Arinc429Parameter::new(true));
        self
    }

    pub fn eng1_tla(mut self, tla: Angle) -> Self {
        self.parameters.set_eng1_tla_a(Arinc429Parameter::new(tla));
        self.parameters.set_eng1_tla_b(Arinc429Parameter::new(tla));
        self
    }

    pub fn eng1_tla_a(mut self, tla: Angle) -> Self {
        self.parameters.set_eng1_tla_a(Arinc429Parameter::new(tla));
        self
    }

    pub fn eng1_tla_b(mut self, tla: Angle) -> Self {
        self.parameters.set_eng1_tla_b(Arinc429Parameter::new(tla));
        self
    }

    pub fn eng2_tla(mut self, tla: Angle) -> Self {
        self.parameters.set_eng2_tla_a(Arinc429Parameter::new(tla));
        self.parameters.set_eng2_tla_b(Arinc429Parameter::new(tla));
        self
    }

    pub fn eng2_tla_a(mut self, tla: Angle) -> Self {
        self.parameters.set_eng2_tla_a(Arinc429Parameter::new(tla));
        self
    }

    pub fn eng2_tla_b(mut self, tla: Angle) -> Self {
        self.parameters.set_eng2_tla_b(Arinc429Parameter::new(tla));
        self
    }
}

pub fn test_bed() -> A320FWCParameterTestBed {
    A320FWCParameterTestBed::new()
}

pub fn test_bed_with() -> A320FWCParameterTestBed {
    test_bed()
}

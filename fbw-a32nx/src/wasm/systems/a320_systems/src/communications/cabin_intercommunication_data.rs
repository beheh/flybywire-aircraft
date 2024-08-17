use crate::hydraulic::A320SlatFlapComputerDiscretes;
use std::time::Duration;

use systems::{
    air_conditioning::AirCondToCidsInterface,
    failures::{Failure, FailureType},
    shared::{
        arinc429::{Arinc429Word, SignStatus},
        ConsumePower, ElectricalBusType, ElectricalBuses, EngineOilPressureLow, LgciuGearExtension,
        LgciuWeightOnWheels,
    },
    simulation::{
        InitContext, Read, SimulationElement, SimulationElementVisitor, SimulatorReader,
        SimulatorWriter, UpdateContext, VariableIdentifier, Write,
    },
};

use uom::si::f64::Power;
use uom::si::power::watt;

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
enum SwitchPosition {
    On = 0,
    Auto = 1,
    Off = 2,
}

impl From<u8> for SwitchPosition {
    fn from(val: u8) -> Self {
        match val {
            0 => SwitchPosition::On,
            1 => SwitchPosition::Auto,
            2 => SwitchPosition::Off,
            _ => SwitchPosition::Off,
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum NoSmokingSwitchType {
    NoSmoking = 0,
    NoPortableDevices = 1,
}

impl From<u8> for NoSmokingSwitchType {
    fn from(value: u8) -> Self {
        match value {
            1 => NoSmokingSwitchType::NoPortableDevices,
            // we might add EXIT switch here in future (together with a GSM setting for NO MOBILE)
            _ => NoSmokingSwitchType::NoSmoking,
        }
    }
}

#[derive(Debug)]
pub struct A320CabinIntercommunicationDataSystemOverheadPanel {
    no_smoking_id: VariableIdentifier,
    fasten_seat_belt_id: VariableIdentifier,
    configured_using_portable_devices_id: VariableIdentifier,

    no_smoking_auto: bool,
    no_smoking_command: bool,
    fasten_seat_belt_auto: bool,
    fasten_seat_belt_command: bool,
    no_ped_auto: bool,
    no_ped_command: bool,
}

impl A320CabinIntercommunicationDataSystemOverheadPanel {
    const FASTEN_SEAT_BELT: &'static str = "OVHD_SIGNS_SEAT_BELTS";
    const NO_SMOKING: &'static str = "OVHD_SIGNS_NO_SMOKING";
    const USING_PORTABLE_DEVICES: &'static str = "CIDS_USING_PORTABLE_DEVICES";

    pub fn new(context: &mut InitContext) -> Self {
        Self {
            no_smoking_id: context.get_identifier(Self::NO_SMOKING.to_owned()),
            fasten_seat_belt_id: context.get_identifier(Self::FASTEN_SEAT_BELT.to_owned()),
            configured_using_portable_devices_id: context
                .get_identifier(Self::USING_PORTABLE_DEVICES.to_owned()),
            no_smoking_auto: false,
            no_smoking_command: false,
            fasten_seat_belt_auto: false,
            fasten_seat_belt_command: false,
            no_ped_auto: false,
            no_ped_command: false,
        }
    }
}

impl SimulationElement for A320CabinIntercommunicationDataSystemOverheadPanel {
    fn read(&mut self, reader: &mut SimulatorReader) {
        let seat_belts_val: u8 = reader.read(&self.fasten_seat_belt_id);
        let no_smoking_val: u8 = reader.read(&self.no_smoking_id);
        let no_smoking_switch_type_val: u8 =
            reader.read(&self.configured_using_portable_devices_id);

        let no_smoking_switch_type: NoSmokingSwitchType = no_smoking_switch_type_val.into();

        let seat_belts: SwitchPosition = seat_belts_val.into();
        let no_smoking: Option<SwitchPosition> =
            if no_smoking_switch_type == NoSmokingSwitchType::NoSmoking {
                Some(no_smoking_val.into())
            } else {
                None
            };
        let no_ped: Option<SwitchPosition> =
            if no_smoking_switch_type == NoSmokingSwitchType::NoPortableDevices {
                Some(no_smoking_val.into())
            } else {
                None
            };

        self.fasten_seat_belt_auto = seat_belts == SwitchPosition::Auto;
        self.fasten_seat_belt_command = seat_belts == SwitchPosition::On;
        self.no_ped_auto = no_ped.map_or(false, |pos| pos == SwitchPosition::Auto);
        self.no_ped_command = no_ped.map_or(false, |pos| pos == SwitchPosition::On);
        self.no_smoking_auto = no_smoking.map_or(false, |pos| pos == SwitchPosition::Auto);
        self.no_smoking_command = no_smoking.map_or(false, |pos| pos == SwitchPosition::On);
    }
}

pub struct A320CabinIntercommunicationDataSystem {
    fwc_flight_phase_id: VariableIdentifier,

    configured_using_portable_devices_id: VariableIdentifier,
    cids_audio_operational_id: VariableIdentifier,
    cids_audio_chime_id: VariableIdentifier,
    cabin_fasten_seat_belt_signs_id: VariableIdentifier,
    cabin_return_to_seat_signs_id: VariableIdentifier,
    cabin_no_smoking_signs_id: VariableIdentifier,
    cabin_no_portable_devices_signs_id: VariableIdentifier,
    cabin_exit_signs_id: VariableIdentifier,

    cabin_ready_id: VariableIdentifier,

    director_1: A320CabinIntercommunicationDataDirector,
    director_2: A320CabinIntercommunicationDataDirector,

    // signs
    cabin_fasten_seat_belt_signs: bool,
    cabin_return_to_seat_signs: bool,
    cabin_no_smoking_signs: bool,
    cabin_no_portable_devices_signs: bool,
    cabin_exit_signs: bool,

    // audio
    cids_audio_operational: bool,
    cids_audio_chime: bool,

    // configuration
    no_smoking_switch_type: NoSmokingSwitchType,

    // state
    fwc_flight_phase: u8,
}

impl A320CabinIntercommunicationDataSystem {
    const FWC_FLIGHT_PHASE: &'static str = "FWC_FLIGHT_PHASE";
    const USING_PORTABLE_DEVICES: &'static str = "CIDS_USING_PORTABLE_DEVICES";
    const AUDIO_OPERATIONAL: &'static str = "CIDS_AUDIO_OPERATIONAL";
    const AUDIO_CHIME: &'static str = "CIDS_AUDIO_CHIME";
    const CABIN_FASTEN_SEAT_BELT_SIGNS: &'static str = "CIDS_CABIN_FASTEN_SEAT_BELT_SIGNS";
    const CABIN_RETURN_TO_SEAT_SIGNS: &'static str = "CIDS_CABIN_RETURN_TO_SEAT_SIGNS";
    const CABIN_NO_SMOKING_SIGNS: &'static str = "CIDS_CABIN_NO_SMOKING_SIGNS";
    const CABIN_NO_PORTABLE_DEVICES_SIGNS: &'static str = "CIDS_CABIN_NO_PORTABLE_DEVICES_SIGNS";
    const CABIN_EXIT_SIGNS: &'static str = "CIDS_CABIN_EXIT_SIGNS";

    const CABIN_READY: &'static str = "CABIN_READY";

    pub fn new(context: &mut InitContext) -> Self {
        Self {
            no_smoking_switch_type: NoSmokingSwitchType::NoSmoking,
            fwc_flight_phase_id: context.get_identifier(Self::FWC_FLIGHT_PHASE.to_owned()),
            configured_using_portable_devices_id: context
                .get_identifier(Self::USING_PORTABLE_DEVICES.to_owned()),
            cids_audio_operational_id: context.get_identifier(Self::AUDIO_OPERATIONAL.to_owned()),
            cids_audio_chime_id: context.get_identifier(Self::AUDIO_CHIME.to_owned()),
            cabin_fasten_seat_belt_signs_id: context
                .get_identifier(Self::CABIN_FASTEN_SEAT_BELT_SIGNS.to_owned()),
            cabin_return_to_seat_signs_id: context
                .get_identifier(Self::CABIN_RETURN_TO_SEAT_SIGNS.to_owned()),
            cabin_no_smoking_signs_id: context
                .get_identifier(Self::CABIN_NO_SMOKING_SIGNS.to_owned()),
            cabin_no_portable_devices_signs_id: context
                .get_identifier(Self::CABIN_NO_PORTABLE_DEVICES_SIGNS.to_owned()),
            cabin_exit_signs_id: context.get_identifier(Self::CABIN_EXIT_SIGNS.to_owned()),
            cabin_ready_id: context.get_identifier(Self::CABIN_READY.to_owned()),
            director_1: A320CabinIntercommunicationDataDirector::new(context, 1),
            director_2: A320CabinIntercommunicationDataDirector::new(context, 2),
            cids_audio_operational: false,
            cids_audio_chime: false,
            cabin_fasten_seat_belt_signs: false,
            cabin_return_to_seat_signs: false,
            cabin_no_smoking_signs: false,
            cabin_no_portable_devices_signs: false,
            cabin_exit_signs: false,
            fwc_flight_phase: 0,
        }
    }

    pub(crate) fn update(
        &mut self,
        context: &UpdateContext,
        overhead_panel: &A320CabinIntercommunicationDataSystemOverheadPanel,
        lgcius: [&(impl LgciuWeightOnWheels + LgciuGearExtension); 2],
        cpcs: [&impl AirCondToCidsInterface; 2],
        engines: [&impl EngineOilPressureLow; 2],
        sfccs: [&impl A320SlatFlapComputerDiscretes; 2],
        fap: &A320ForwardAttendantPanel,
    ) {
        let inputs = self.determine_inputs(lgcius, overhead_panel, cpcs, engines, sfccs, fap);
        let outputs1 = self.director_1.update(context.delta(), &inputs[0]);
        let outputs2 = self.director_2.update(context.delta(), &inputs[1]);

        // TODO primary outputs
        let outputs = if !outputs1.fail { outputs1 } else { outputs2 };

        self.cids_audio_operational = !outputs.fail;
        self.cids_audio_chime = outputs.audio_chime;
        self.cabin_fasten_seat_belt_signs = outputs.fasten_seat_belt_signs;
        self.cabin_return_to_seat_signs = outputs.return_to_seat_signs;
        self.cabin_no_smoking_signs = outputs.no_smoking_signs;
        self.cabin_no_portable_devices_signs = outputs.no_portable_devices_signs;
        self.cabin_exit_signs = outputs.exit_signs;
    }

    fn get_primary_director(&self) -> Option<&A320CabinIntercommunicationDataDirector> {
        // TODO
        Some(&self.director_1)
    }

    fn determine_inputs(
        &self,
        lgcius: [&(impl LgciuWeightOnWheels + LgciuGearExtension); 2],
        overhead: &A320CabinIntercommunicationDataSystemOverheadPanel,
        cpcs: [&impl AirCondToCidsInterface; 2],
        engines: [&impl EngineOilPressureLow; 2],
        sfccs: [&impl A320SlatFlapComputerDiscretes; 2],
        fap: &A320ForwardAttendantPanel,
    ) -> [A320CabinIntercommunicationDataDirectorInputs; 2] {
        let lg_compressed = lgcius[0].left_and_right_gear_compressed(true)
            || lgcius[1].left_and_right_gear_compressed(true);
        let lg_down_locked = lgcius[0].main_down_and_locked() || lgcius[1].main_down_and_locked();

        [
            A320CabinIntercommunicationDataDirectorInputs {
                no_ped_installed: self.no_smoking_switch_type
                    == NoSmokingSwitchType::NoPortableDevices,
                opp_dir_active: false,
                opp_dir_fault: false,
                fasten_seat_belt_command: overhead.fasten_seat_belt_command,
                fasten_seat_belt_auto: overhead.fasten_seat_belt_auto,
                no_smoking_command: overhead.no_smoking_command,
                no_smoking_auto: overhead.no_smoking_auto,
                no_ped_command: overhead.no_ped_command,
                no_ped_auto: overhead.no_ped_auto,
                oil_pressure_low_on_ground: lg_compressed
                    && engines[0].oil_pressure_is_low()
                    && engines[1].oil_pressure_is_low(),
                lg_down_locked,
                slats_1: sfccs[0].slats_at_or_above_17deg_discrete(),
                slats_2: sfccs[1].slats_at_or_above_17deg_discrete(),
                flaps_1: sfccs[0].flaps_at_or_above_19deg_discrete(),
                flaps_2: sfccs[0].flaps_at_or_above_19deg_discrete(),
                cpc_1_excessive_alt: cpcs[0].excessive_cabin_altitude(),
                cpc_2_excessive_alt: cpcs[1].excessive_cabin_altitude(),
                flight_phase: self.fwc_flight_phase,
                request_cabin_ready: fap.request_cabin_ready(),
            },
            A320CabinIntercommunicationDataDirectorInputs {
                no_ped_installed: self.no_smoking_switch_type
                    == NoSmokingSwitchType::NoPortableDevices,
                opp_dir_active: true,
                opp_dir_fault: false,
                fasten_seat_belt_command: overhead.fasten_seat_belt_command,
                fasten_seat_belt_auto: overhead.fasten_seat_belt_auto,
                no_smoking_command: overhead.no_smoking_command,
                no_smoking_auto: overhead.no_smoking_auto,
                no_ped_command: overhead.no_ped_command,
                no_ped_auto: overhead.no_ped_auto,
                oil_pressure_low_on_ground: lg_compressed
                    && engines[0].oil_pressure_is_low()
                    && engines[1].oil_pressure_is_low(),
                lg_down_locked,
                slats_1: sfccs[0].slats_at_or_above_17deg_discrete(),
                slats_2: sfccs[1].slats_at_or_above_17deg_discrete(),
                flaps_1: sfccs[0].flaps_at_or_above_19deg_discrete(),
                flaps_2: sfccs[0].flaps_at_or_above_19deg_discrete(),
                cpc_1_excessive_alt: cpcs[0].excessive_cabin_altitude(),
                cpc_2_excessive_alt: cpcs[1].excessive_cabin_altitude(),
                flight_phase: self.fwc_flight_phase,
                request_cabin_ready: fap.request_cabin_ready(),
            },
        ]
    }

    fn cabin_ready(&self) -> bool {
        self.get_primary_director()
            .map(|d| d.cabin_ready)
            .unwrap_or(false)
    }

    fn cabin_ready_available(&self) -> bool {
        self.get_primary_director()
            .map(|d| d.cabin_ready_available)
            .unwrap_or(false)
    }
}

impl SimulationElement for A320CabinIntercommunicationDataSystem {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.director_1.accept(visitor);
        self.director_2.accept(visitor);

        visitor.visit(self);
    }

    fn read(&mut self, reader: &mut SimulatorReader) {
        let no_smoking_switch_type_val: u8 =
            reader.read(&self.configured_using_portable_devices_id);
        self.no_smoking_switch_type = no_smoking_switch_type_val.into();

        self.fwc_flight_phase = reader.read(&self.fwc_flight_phase_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.cids_audio_operational_id, self.cids_audio_operational);
        writer.write(&self.cids_audio_chime_id, self.cids_audio_chime);
        writer.write(
            &self.cabin_fasten_seat_belt_signs_id,
            self.cabin_fasten_seat_belt_signs,
        );
        writer.write(
            &self.cabin_return_to_seat_signs_id,
            self.cabin_return_to_seat_signs,
        );
        writer.write(&self.cabin_no_smoking_signs_id, self.cabin_no_smoking_signs);
        writer.write(
            &self.cabin_no_portable_devices_signs_id,
            self.cabin_no_portable_devices_signs,
        );
        writer.write(&self.cabin_exit_signs_id, self.cabin_exit_signs);

        writer.write(
            &self.cabin_ready_id,
            self.get_primary_director()
                .map(|d| d.cabin_ready)
                .unwrap_or(false),
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd)]
enum AutoLightsSetting {
    Gear = 0,
    GearOrSlats = 1,
    EnginesAndGearOrSlats = 2,
}

impl AutoLightsSetting {
    fn satisfied_by_gear_down(self) -> bool {
        self >= Self::Gear
    }

    fn satisfied_by_slats_or_flaps(self) -> bool {
        self >= Self::GearOrSlats
    }

    fn requires_engine_running(self) -> bool {
        self == Self::EnginesAndGearOrSlats
    }

    fn satisfied_by(self, input: &A320CabinIntercommunicationDataDirectorInputs) -> bool {
        (!self.requires_engine_running() || !input.oil_pressure_low_on_ground)
            && (self.satisfied_by_gear_down() && input.lg_down_locked
                || self.satisfied_by_slats_or_flaps()
                    && (input.slats_1 || input.slats_2 || input.flaps_1 || input.flaps_2))
    }
}

struct A320CabinIntercommunicationDataDirectorInputs {
    no_ped_installed: bool,

    opp_dir_active: bool,
    opp_dir_fault: bool,

    fasten_seat_belt_command: bool,
    fasten_seat_belt_auto: bool,
    no_smoking_command: bool,
    no_smoking_auto: bool,
    no_ped_command: bool,
    no_ped_auto: bool,
    oil_pressure_low_on_ground: bool,
    lg_down_locked: bool,
    slats_1: bool,
    slats_2: bool,
    flaps_1: bool,
    flaps_2: bool,
    cpc_1_excessive_alt: bool,
    cpc_2_excessive_alt: bool,
    flight_phase: u8,
    request_cabin_ready: Option<bool>,
}

struct A320CabinIntercommunicationDataSystemOutputs {
    fail: bool,
    discrete_word_1: Arinc429Word<u32>,
    discrete_word_2: Arinc429Word<u32>,

    // Cabin Lights
    fasten_seat_belt_signs: bool,
    return_to_seat_signs: bool,
    no_smoking_signs: bool,
    no_portable_devices_signs: bool,
    exit_signs: bool,

    // Audio
    audio_chime: bool,
}

impl Default for A320CabinIntercommunicationDataSystemOutputs {
    fn default() -> Self {
        Self {
            fail: true,
            discrete_word_1: Arinc429Word::new(0, SignStatus::FailureWarning),
            discrete_word_2: Arinc429Word::new(0, SignStatus::FailureWarning),
            fasten_seat_belt_signs: false,
            return_to_seat_signs: false,
            no_smoking_signs: false,
            no_portable_devices_signs: false,
            exit_signs: false,
            audio_chime: false,
        }
    }
}

struct A320CabinIntercommunicationDataDirector {
    number: usize,
    failure: Failure,

    cids_discrete_word_1_id: VariableIdentifier,
    cids_discrete_word_2_id: VariableIdentifier,

    // power
    powered_by: Option<ElectricalBusType>,

    // settings
    fsb_auto_setting: AutoLightsSetting,
    no_ped_auto_setting: AutoLightsSetting,
    non_smoker_ac: bool,

    // state
    active: bool,
    cabin_ready: bool,
    phase_9_time_elapsed: Duration,
    is_takeoff: bool,
    slats_were_retracted_in_flight: bool,
    cabin_ready_available: bool,

    remaining_chimes: u8,
    chime_cooldown: Duration,

    // output
    output: A320CabinIntercommunicationDataSystemOutputs,
}

impl A320CabinIntercommunicationDataDirector {
    const NORMAL_POWER_SUPPLY: ElectricalBusType = ElectricalBusType::DirectCurrentGndFltService; // TODO sub-bus 601PP
    const EMERGENCY_POWER_SUPPLY: ElectricalBusType = ElectricalBusType::DirectCurrentEssential; // TODO sub-bus (401PP DC ESS)

    fn new(context: &mut InitContext, number: usize) -> Self {
        Self {
            number,
            failure: Failure::new(FailureType::CabinIntercommunicationDataSystem(number)),
            cids_discrete_word_1_id: context
                .get_identifier(format!("CIDS_{}_DISCRETE_WORD_1", number)),
            cids_discrete_word_2_id: context
                .get_identifier(format!("CIDS_{}_DISCRETE_WORD_2", number)),

            active: false,
            cabin_ready: false,
            phase_9_time_elapsed: Duration::ZERO,
            is_takeoff: true,
            slats_were_retracted_in_flight: false,
            cabin_ready_available: false,

            remaining_chimes: 0,
            chime_cooldown: Duration::ZERO,

            powered_by: if context.has_engines_running() {
                Some(Self::NORMAL_POWER_SUPPLY)
            } else {
                None
            },

            output: A320CabinIntercommunicationDataSystemOutputs::default(),
            fsb_auto_setting: AutoLightsSetting::GearOrSlats,
            no_ped_auto_setting: AutoLightsSetting::GearOrSlats,
            non_smoker_ac: true,
        }
    }

    fn update(
        &mut self,
        delta: Duration,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) -> &A320CabinIntercommunicationDataSystemOutputs {
        let fault = self.failure.is_active();
        let is_powered = self.powered_by.is_some();

        // active or passive?
        let active = if self.number == 1 {
            if self.active && !fault {
                // keep control if healthy
                true
            } else if !self.active {
                // defer to the opposite director
                !inputs.opp_dir_active
            } else {
                // fault
                // when failed, set active only if opposite has also failed
                inputs.opp_dir_fault
            }
        } else {
            // defer to the opposite director
            !inputs.opp_dir_active
        };

        if !active || !is_powered {
            self.remaining_chimes = 0;
            self.chime_cooldown = Duration::ZERO;
            self.output = A320CabinIntercommunicationDataSystemOutputs::default();
            return &self.output;
        }

        self.update_cabin_ready(delta, inputs);

        self.output = self.determine_outputs(delta, inputs);
        &self.output
    }

    fn update_cabin_ready(
        &mut self,
        delta: Duration,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) {
        self.phase_9_time_elapsed = if inputs.flight_phase == 9 {
            self.phase_9_time_elapsed + delta
        } else {
            Duration::ZERO
        };

        if inputs.flight_phase == 6 {
            self.is_takeoff = false;
        }
        if inputs.flight_phase < 4 || inputs.flight_phase > 8 {
            self.is_takeoff = true;
        }

        if inputs.flight_phase == 6 && !inputs.slats_1 && !inputs.slats_2 {
            self.slats_were_retracted_in_flight = true;
        }
        if inputs.flight_phase != 6 {
            self.slats_were_retracted_in_flight = false;
        }

        self.cabin_ready_available = match inputs.flight_phase {
            1 => !inputs.oil_pressure_low_on_ground,
            2 | 3 => true,
            4 | 5 => !self.is_takeoff,
            6 => self.slats_were_retracted_in_flight && (inputs.slats_1 || inputs.slats_2),
            7 | 8 => true,
            9 => self.phase_9_time_elapsed < Duration::from_secs(30),
            _ => false,
        };

        if !self.cabin_ready_available {
            self.cabin_ready = false;
        } else if let Some(value) = inputs.request_cabin_ready {
            self.cabin_ready = value;
        }
    }

    fn determine_outputs(
        &mut self,
        delta: Duration,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) -> A320CabinIntercommunicationDataSystemOutputs {
        // Chime cooldown
        if let Some(remaining) = self.chime_cooldown.checked_sub(delta) {
            self.chime_cooldown = remaining;
        } else {
            self.chime_cooldown = Duration::ZERO;
        }

        // Lights
        let fasten_seat_belt_signs = self.determine_fasten_seat_belt_signs(inputs);
        let no_smoking_signs_arinc = self.determine_no_smoking_signs(inputs);
        let return_to_seat_signs =
            self.determine_return_to_seat_signs(inputs, fasten_seat_belt_signs);
        let no_portable_devices_signs = self.determine_no_portable_electronic_devices_sign(inputs);
        let exit_signs = self.determine_exit_signs(inputs);
        let no_smoking_signs = self.non_smoker_ac || no_smoking_signs_arinc;

        // Discrete Word 1, Label 275
        let mut discrete_word_1 = Arinc429Word::new(0, SignStatus::NormalOperation);

        discrete_word_1.set_bit(11, false); // CIDS Class 2 Fault
        discrete_word_1.set_bit(12, false); // CIDS Fault

        discrete_word_1.set_bit(15, no_portable_devices_signs);
        discrete_word_1.set_bit(16, fasten_seat_belt_signs);
        discrete_word_1.set_bit(17, no_smoking_signs_arinc);

        discrete_word_1.set_bit(22, self.cabin_ready);
        discrete_word_1.set_bit(23, true); // Check Cabin installed (move CABIN READY from right memo to T.O/LDG memo)

        discrete_word_1.set_bit(27, inputs.no_ped_installed); // No PED installed
        discrete_word_1.set_bit(28, true); // FSB Installed
        discrete_word_1.set_bit(29, !inputs.no_ped_installed); // No Smoking installed

        // Discrete Word 2, Label 276
        let mut discrete_word_2 = Arinc429Word::new(0, SignStatus::NormalOperation);

        discrete_word_2.set_bit(11, false); // No Mobile signs
        discrete_word_2.set_bit(12, false); // No Mobile installed

        // Audio
        if fasten_seat_belt_signs != self.output.fasten_seat_belt_signs
            || return_to_seat_signs != self.output.return_to_seat_signs
            || no_smoking_signs != self.output.no_smoking_signs
            || no_portable_devices_signs != self.output.no_portable_devices_signs
            || exit_signs != self.output.exit_signs
        {
            self.remaining_chimes = self.remaining_chimes.checked_add(1).unwrap_or(u8::MAX);
        }

        let mut audio_chime = false;
        if self.remaining_chimes > 0 && self.chime_cooldown == Duration::ZERO {
            audio_chime = true;
            self.chime_cooldown = Duration::from_millis(2_700); // TODO
            self.remaining_chimes -= 1;
        }

        A320CabinIntercommunicationDataSystemOutputs {
            fail: false,
            discrete_word_1,
            discrete_word_2,
            fasten_seat_belt_signs,
            no_smoking_signs,
            return_to_seat_signs,
            no_portable_devices_signs,
            exit_signs,
            audio_chime,
        }
    }

    fn determine_fasten_seat_belt_signs(
        &self,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) -> bool {
        // The Fasten Seat Belt signs can either be ON, AUTO or OFF. If the cabin pressure is low,
        // they will always be illuminated. Otherwise they follow the switch. In the AUTO position
        // they follow a configurable setting that determines the criteria for coming on
        // automatically.
        if inputs.cpc_1_excessive_alt
            || inputs.cpc_2_excessive_alt
            || inputs.fasten_seat_belt_command
        {
            true
        } else if inputs.fasten_seat_belt_auto {
            self.fsb_auto_setting.satisfied_by(inputs)
        } else {
            false
        }
    }

    fn determine_return_to_seat_signs(
        &self,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
        fasten_seat_belt_signs: bool,
    ) -> bool {
        // The Return To Seat signs (in the restrooms) usually follow the Fasten Seatbelt signs,
        // except they are not illuminated in a low cabin pressure situation.
        !inputs.cpc_1_excessive_alt && !inputs.cpc_2_excessive_alt && fasten_seat_belt_signs
    }

    fn determine_no_smoking_signs(
        &self,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) -> bool {
        // Note that this is calculated for the purposes of the A429 word even if the aircraft is
        // configured as a non-smoker aircraft and the signs will be eventually forced on anyway.

        if inputs.no_ped_installed {
            false
        } else if inputs.no_smoking_command
            || inputs.cpc_1_excessive_alt
            || inputs.cpc_2_excessive_alt
        {
            true
        } else if inputs.no_smoking_auto {
            inputs.lg_down_locked
        } else {
            false
        }
    }

    fn determine_exit_signs(&self, inputs: &A320CabinIntercommunicationDataDirectorInputs) -> bool {
        // These are controlled together with the NO SMOKING or NO PED switch, depending on which
        // is installed.
        let exit_command = if inputs.no_ped_installed {
            inputs.no_ped_command
        } else {
            inputs.no_smoking_command
        };
        let exit_auto = if inputs.no_ped_installed {
            inputs.no_ped_auto
        } else {
            inputs.no_smoking_auto
        };

        if exit_command || inputs.cpc_1_excessive_alt || inputs.cpc_2_excessive_alt {
            true
        } else if exit_auto {
            inputs.lg_down_locked
        } else {
            false
        }
    }

    fn determine_no_portable_electronic_devices_sign(
        &self,
        inputs: &A320CabinIntercommunicationDataDirectorInputs,
    ) -> bool {
        if !inputs.no_ped_installed {
            false
        } else if inputs.no_ped_command {
            true
        } else if inputs.no_ped_auto {
            self.no_ped_auto_setting.satisfied_by(inputs)
        } else {
            false
        }
    }
}

impl SimulationElement for A320CabinIntercommunicationDataDirector {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where
        Self: Sized,
    {
        self.failure.accept(visitor);

        visitor.visit(self);
    }

    fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        fn try_receive_power(
            buses: &impl ElectricalBuses,
            bus: ElectricalBusType,
        ) -> Option<ElectricalBusType> {
            if buses.is_powered(bus) {
                Some(bus)
            } else {
                None
            }
        }

        self.powered_by = try_receive_power(buses, Self::NORMAL_POWER_SUPPLY)
            .or_else(|| try_receive_power(buses, Self::EMERGENCY_POWER_SUPPLY));
    }

    fn consume_power<T: ConsumePower>(&mut self, _context: &UpdateContext, consumption: &mut T) {
        if let Some(powered_by) = self.powered_by {
            consumption.consume_from_bus(powered_by, Power::new::<watt>(100.));
        }
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.cids_discrete_word_1_id, self.output.discrete_word_1);
        writer.write(&self.cids_discrete_word_2_id, self.output.discrete_word_2);
    }
}

pub struct A320ForwardAttendantPanel {
    cabin_ready_button_id: VariableIdentifier,
    cabin_ready_button_available_id: VariableIdentifier,

    cabin_ready_available: bool,
    cabin_ready_button: bool,
    request_cabin_ready: Option<bool>,
}

impl A320ForwardAttendantPanel {
    const CABIN_READY_BUTTON: &'static str = "FAP_CABIN_READY_BUTTON";
    const CABIN_READY_BUTTON_AVAILABLE: &'static str = "FAP_CABIN_READY_AVAILABLE";

    pub fn new(context: &mut InitContext) -> Self {
        Self {
            cabin_ready_button_id: context.get_identifier(Self::CABIN_READY_BUTTON.to_owned()),
            cabin_ready_button_available_id: context
                .get_identifier(Self::CABIN_READY_BUTTON_AVAILABLE.to_owned()),
            cabin_ready_available: false,
            cabin_ready_button: false,
            request_cabin_ready: None,
        }
    }

    fn request_cabin_ready(&self) -> Option<bool> {
        self.request_cabin_ready
    }

    pub(crate) fn update(
        &mut self,
        _context: &UpdateContext,
        cids: &A320CabinIntercommunicationDataSystem,
    ) {
        // check if CABIN READY button is enabled
        self.cabin_ready_available = cids.cabin_ready_available();

        // cabin ready
        if self.cabin_ready_available && self.cabin_ready_button != cids.cabin_ready() {
            self.request_cabin_ready = Some(self.cabin_ready_button);
        } else {
            self.request_cabin_ready = None;
        }
    }
}

impl SimulationElement for A320ForwardAttendantPanel {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.cabin_ready_button = reader.read(&self.cabin_ready_button_id);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(
            &self.cabin_ready_button_available_id,
            self.cabin_ready_available,
        );
        if !self.cabin_ready_available {
            writer.write(&self.cabin_ready_button_id, false);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use systems::air_conditioning::AirCondToCidsInterface;
    use systems::{
        electrical::{test::TestElectricitySource, ElectricalBus, Electricity},
        shared::{PotentialOrigin, PowerConsumptionReport},
        simulation::{
            test::{ReadByName, SimulationTestBed, TestBed, WriteByName},
            Aircraft, InitContext, SimulationElementVisitor, StartState,
        },
    };
    use uom::si::electric_potential::volt;
    use uom::si::f64::ElectricPotential;

    struct TestLgciu {
        compressed: bool,
        down: bool,
    }

    impl TestLgciu {
        fn set_compressed(&mut self, compressed: bool) {
            self.compressed = compressed;
        }

        fn set_down(&mut self, down: bool) {
            self.down = down;
        }
    }

    impl LgciuWeightOnWheels for TestLgciu {
        fn right_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            self.compressed
        }

        fn right_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            !self.compressed
        }

        fn left_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            self.compressed
        }

        fn left_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            !self.compressed
        }

        fn left_and_right_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            self.compressed
        }

        fn left_and_right_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            !self.compressed
        }

        fn nose_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            self.compressed
        }

        fn nose_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            !self.compressed
        }
    }

    impl LgciuGearExtension for TestLgciu {
        fn all_down_and_locked(&self) -> bool {
            self.down
        }

        fn all_up_and_locked(&self) -> bool {
            !self.down
        }

        fn main_down_and_locked(&self) -> bool {
            self.down
        }

        fn main_up_and_locked(&self) -> bool {
            !self.down
        }

        fn nose_down_and_locked(&self) -> bool {
            self.down
        }

        fn nose_up_and_locked(&self) -> bool {
            !self.down
        }
    }

    struct TestSfcc {
        slats_out: bool,
        flaps_out: bool,
    }

    impl TestSfcc {
        fn set_slats_out(&mut self, slats_out: bool) {
            self.slats_out = slats_out;
        }

        fn set_flaps_out(&mut self, flaps_out: bool) {
            self.flaps_out = flaps_out;
        }
    }

    impl A320SlatFlapComputerDiscretes for TestSfcc {
        fn flaps_at_or_above_19deg_discrete(&self) -> bool {
            self.flaps_out
        }

        fn slats_at_or_above_17deg_discrete(&self) -> bool {
            self.slats_out
        }
    }

    struct TestCpc {
        excessive_alt: bool,
    }

    impl TestCpc {
        fn set_excessive_alt(&mut self, excessive_alt: bool) {
            self.excessive_alt = excessive_alt;
        }
    }

    impl AirCondToCidsInterface for TestCpc {
        fn excessive_cabin_altitude(&self) -> bool {
            self.excessive_alt
        }
    }

    struct TestEngine {
        oil_pressure_low: bool,
    }

    impl TestEngine {
        fn set_running(&mut self, running: bool) {
            self.oil_pressure_low = !running;
        }
    }

    impl EngineOilPressureLow for TestEngine {
        fn oil_pressure_is_low(&self) -> bool {
            self.oil_pressure_low
        }
    }

    struct TestAircraft {
        electricity_source: TestElectricitySource,
        dc_service_bus: ElectricalBus,
        dc_ess_bus: ElectricalBus,
        test_lgciu: TestLgciu,
        test_sfcc: TestSfcc,
        test_cpc: TestCpc,
        test_engine: TestEngine,
        cids: A320CabinIntercommunicationDataSystem,
        cids_overhead: A320CabinIntercommunicationDataSystemOverheadPanel,
        fap: A320ForwardAttendantPanel,
        is_dc_service_bus_powered: bool,
        is_dc_ess_powered: bool,
        power_consumption: Power,
    }
    impl TestAircraft {
        fn new(context: &mut InitContext) -> Self {
            Self {
                electricity_source: TestElectricitySource::powered(
                    context,
                    PotentialOrigin::EngineGenerator(1),
                ),
                dc_service_bus: ElectricalBus::new(
                    context,
                    ElectricalBusType::DirectCurrentGndFltService,
                ),
                dc_ess_bus: ElectricalBus::new(context, ElectricalBusType::DirectCurrentEssential),
                test_lgciu: TestLgciu {
                    compressed: false,
                    down: false,
                },
                test_sfcc: TestSfcc {
                    slats_out: false,
                    flaps_out: false,
                },
                test_cpc: TestCpc {
                    excessive_alt: false,
                },
                test_engine: TestEngine {
                    oil_pressure_low: true,
                },
                cids: A320CabinIntercommunicationDataSystem::new(context),
                cids_overhead: A320CabinIntercommunicationDataSystemOverheadPanel::new(context),
                fap: A320ForwardAttendantPanel::new(context),
                is_dc_service_bus_powered: false,
                is_dc_ess_powered: false,
                power_consumption: Power::new::<watt>(0.),
            }
        }

        fn cids(&self) -> &A320CabinIntercommunicationDataSystem {
            &self.cids
        }

        fn cids_director(&self, number: usize) -> &A320CabinIntercommunicationDataDirector {
            match number {
                1 => &self.cids.director_1,
                2 => &self.cids.director_2,
                _ => {
                    panic!("The A320CabinIntercommunicationDataSystem only has two CIDS directors")
                }
            }
        }

        fn set_service_bus_power(&mut self, is_powered: bool) {
            self.is_dc_service_bus_powered = is_powered;
        }

        fn set_dc_ess_power(&mut self, is_powered: bool) {
            self.is_dc_ess_powered = is_powered;
        }

        fn power_consumption(&self) -> Power {
            self.power_consumption
        }
    }

    impl Aircraft for TestAircraft {
        fn update_before_power_distribution(
            &mut self,
            _: &UpdateContext,
            electricity: &mut Electricity,
        ) {
            self.electricity_source
                .power_with_potential(ElectricPotential::new::<volt>(115.));
            electricity.supplied_by(&self.electricity_source);

            if self.is_dc_service_bus_powered {
                electricity.flow(&self.electricity_source, &self.dc_service_bus);
            }

            if self.is_dc_ess_powered {
                electricity.flow(&self.electricity_source, &self.dc_ess_bus);
            }
        }

        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.cids.update(
                context,
                &self.cids_overhead,
                [&self.test_lgciu, &self.test_lgciu],
                [&self.test_cpc, &self.test_cpc],
                [&self.test_engine, &self.test_engine],
                [&self.test_sfcc, &self.test_sfcc],
                &self.fap,
            );
        }
    }

    impl SimulationElement for TestAircraft {
        fn process_power_consumption_report<T: PowerConsumptionReport>(
            &mut self,
            _: &UpdateContext,
            report: &T,
        ) {
            self.power_consumption =
                report.total_consumption_of(PotentialOrigin::EngineGenerator(1));
        }

        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
            self.cids_overhead.accept(visitor);
            self.cids.accept(visitor);

            visitor.visit(self);
        }
    }

    struct CabinIntercommunicationDataSystemTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }

    impl CabinIntercommunicationDataSystemTestBed {
        fn new() -> Self {
            let mut cids_test_bed = Self {
                test_bed: SimulationTestBed::new_with_start_state(
                    StartState::Cruise,
                    TestAircraft::new,
                ),
            };
            cids_test_bed = cids_test_bed.powered();

            cids_test_bed
        }

        fn no_power(mut self) -> Self {
            self.command(|a| a.set_service_bus_power(false));
            self
        }

        fn powered(mut self) -> Self {
            self.command(|a| a.set_service_bus_power(true));
            self
        }

        fn engines_running(mut self) -> Self {
            self.command(|a| a.test_engine.set_running(true));
            self
        }

        fn slats_out(mut self, out: bool) -> Self {
            self.command(|a| a.test_sfcc.set_slats_out(out));
            self
        }

        fn flaps_out(mut self, out: bool) -> Self {
            self.command(|a| a.test_sfcc.set_flaps_out(out));
            self
        }

        fn clean_config(mut self) -> Self {
            self.flaps_out(false).slats_out(false).gear_up()
        }

        fn gear_up(mut self) -> Self {
            self.command(|a| a.test_lgciu.set_down(false));
            self.command(|a| a.test_lgciu.set_compressed(false));
            self
        }

        fn gear_down(mut self) -> Self {
            self.command(|a| a.test_lgciu.set_down(true));
            self
        }

        fn gear_compressed(mut self, compressed: bool) -> Self {
            self.command(|a| a.test_lgciu.set_compressed(compressed));
            self
        }

        fn fwc_flight_phase(mut self, flight_phase: u8) -> Self {
            self.write_by_name("FWC_FLIGHT_PHASE", flight_phase as i32 as f64);
            self
        }

        fn no_smoking_switch_position(mut self, position: SwitchPosition) -> Self {
            self.write_by_name(
                A320CabinIntercommunicationDataSystemOverheadPanel::USING_PORTABLE_DEVICES,
                false,
            );
            self.write_by_name(
                A320CabinIntercommunicationDataSystemOverheadPanel::NO_SMOKING,
                position as u8,
            );
            self
        }

        fn no_ped_switch_position(mut self, position: SwitchPosition) -> Self {
            self.write_by_name(
                A320CabinIntercommunicationDataSystemOverheadPanel::USING_PORTABLE_DEVICES,
                true,
            );
            self.write_by_name(
                A320CabinIntercommunicationDataSystemOverheadPanel::NO_SMOKING,
                position as u8,
            );
            self
        }

        fn fasten_seat_belts_switch_position(mut self, position: SwitchPosition) -> Self {
            self.write_by_name(
                A320CabinIntercommunicationDataSystemOverheadPanel::FASTEN_SEAT_BELT,
                position as u8,
            );
            self
        }

        fn signs_off(self) -> Self {
            self.fasten_seat_belts_switch_position(SwitchPosition::Off)
                .no_smoking_switch_position(SwitchPosition::Off)
        }

        fn excessive_alt(mut self) -> Self {
            self.command(|a| a.test_cpc.set_excessive_alt(true));
            self
        }

        fn optionally_failed_cids_director(mut self, maybe_number: Option<usize>) -> Self {
            if let Some(number) = maybe_number {
                self.failed_cids_director(number)
            } else {
                self
            }
        }

        fn failed_cids_director(mut self, number: usize) -> Self {
            self.fail(FailureType::CabinIntercommunicationDataSystem(number));
            self
        }

        fn assert_discrete_word_1(&mut self, bit: u8, on: bool) {
            let word: Arinc429Word<u32> = self.read_arinc429_by_name("CIDS_1_DISCRETE_WORD_1");
            assert_eq!(SignStatus::NormalOperation, word.ssm());
            assert_eq!(on, word.get_bit(bit));
        }

        fn assert_discrete_word_2(&mut self, bit: u8, on: bool) {
            let word: Arinc429Word<u32> = self.read_arinc429_by_name("CIDS_1_DISCRETE_WORD_2");
            assert_eq!(SignStatus::NormalOperation, word.ssm());
            assert_eq!(on, word.get_bit(bit));
        }

        fn assert_fasten_seat_belts_sign(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_fasten_seat_belt_signs), on);
            self.assert_discrete_word_1(16, on)
        }

        fn assert_return_to_seats_sign(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_return_to_seat_signs), on);
        }

        fn assert_no_smoking_signs(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_no_smoking_signs), on);
        }

        fn assert_no_smoking_signs_arinc(&mut self, on: bool) {
            self.assert_discrete_word_1(17, on)
        }

        fn assert_no_portable_devices_signs(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_no_portable_devices_signs), on);
            self.assert_discrete_word_1(15, on);
        }

        fn assert_exit_signs(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_exit_signs), on);
        }

        fn assert_cabin_ready(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_ready()), on);
        }

        fn assert_cabin_ready_available(&mut self, on: bool) {
            assert_eq!(self.query(|a| a.cids.cabin_ready_available()), on);
        }
    }
    impl TestBed for CabinIntercommunicationDataSystemTestBed {
        type Aircraft = TestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
            &mut self.test_bed
        }
    }

    fn test_bed() -> CabinIntercommunicationDataSystemTestBed {
        CabinIntercommunicationDataSystemTestBed::new()
    }

    fn test_bed_with() -> CabinIntercommunicationDataSystemTestBed {
        test_bed()
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn auto_seat_belts_on_ground_are_on(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_down()
            .gear_compressed(true)
            .fasten_seat_belts_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn auto_seat_belts_with_slats_out_are_on(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_down()
            .gear_compressed(true)
            .fasten_seat_belts_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn auto_seat_belts_in_clean_config_are_off(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .clean_config()
            .fasten_seat_belts_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(false);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn auto_no_ped_on_ground_are_on(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_down()
            .gear_compressed(true)
            .no_ped_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_no_portable_devices_signs(true);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn auto_no_ped_slats_out_are_on(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_up()
            .no_ped_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_no_portable_devices_signs(true);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn signs_on_ground_with_engines_running(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_down()
            .gear_compressed(true)
            .fasten_seat_belts_switch_position(SwitchPosition::On)
            .no_smoking_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
        test_bed.assert_return_to_seats_sign(true);
        test_bed.assert_no_smoking_signs(true);
        test_bed.assert_no_smoking_signs_arinc(true);
        test_bed.assert_exit_signs(true);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn signs_in_the_air_after_takeoff(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .flaps_out(true)
            .slats_out(true)
            .gear_up()
            .gear_compressed(false)
            .fasten_seat_belts_switch_position(SwitchPosition::On)
            .no_smoking_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
        test_bed.assert_return_to_seats_sign(true);
        test_bed.assert_no_smoking_signs(true);
        test_bed.assert_no_smoking_signs_arinc(false);
        test_bed.assert_exit_signs(false);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn signs_in_the_air_after_clean_up(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .clean_config()
            .gear_compressed(false)
            .fasten_seat_belts_switch_position(SwitchPosition::On)
            .no_smoking_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
        test_bed.assert_return_to_seats_sign(true);
        test_bed.assert_no_smoking_signs(true);
        test_bed.assert_no_smoking_signs_arinc(false);
        test_bed.assert_exit_signs(false);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn signs_in_cruise(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .clean_config()
            .fasten_seat_belts_switch_position(SwitchPosition::Off)
            .no_smoking_switch_position(SwitchPosition::Auto);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(false);
        test_bed.assert_return_to_seats_sign(false);
        test_bed.assert_no_smoking_signs(true);
        test_bed.assert_no_smoking_signs_arinc(false);
        test_bed.assert_exit_signs(false);
    }

    #[rstest]
    #[case(None)]
    #[case(Some(1))]
    #[case(Some(2))]
    fn signs_after_decompression(#[case] failed_cids_director: Option<usize>) {
        let mut test_bed = test_bed_with()
            .optionally_failed_cids_director(failed_cids_director)
            .engines_running()
            .clean_config()
            .fasten_seat_belts_switch_position(SwitchPosition::Off)
            .no_smoking_switch_position(SwitchPosition::Auto)
            .excessive_alt();
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_fasten_seat_belts_sign(true);
        test_bed.assert_return_to_seats_sign(false);
        test_bed.assert_no_smoking_signs(true);
        test_bed.assert_exit_signs(true);
    }

    #[test]
    fn cabin_ready_not_available_before_start() {
        let mut test_bed = test_bed_with().gear_compressed(true).fwc_flight_phase(1);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_cabin_ready_available(false);
    }

    #[test]
    fn cabin_ready_available_after_start() {
        let mut test_bed = test_bed_with()
            .engines_running()
            .gear_compressed(true)
            .fwc_flight_phase(1);
        test_bed.run_with_delta(Duration::from_millis(1));

        test_bed.assert_cabin_ready_available(true);
    }
}

use std::fmt::{Debug, Display, Formatter};
use std::time::Duration;

#[derive(Eq, PartialEq, Hash, Copy, Clone)]
pub struct WarningCode {
    ata: u8,
    sub_ata: u8,
    id: u16,
}

impl WarningCode {
    pub fn new(ata: u8, sub_ata: u8, id: u16) -> Self {
        Self { ata, sub_ata, id }
    }
}

impl Display for WarningCode {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:02}-{:02}-{:03}", self.ata, self.sub_ata, self.id)
    }
}

impl Debug for WarningCode {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:02}-{:02}-{:03}", self.ata, self.sub_ata, self.id)
    }
}

pub struct WarningNode {}

enum SysPage {
    AirBleed,
    AirPress,
    Hydraulic,
    Elec,
    Fuel,
    Apu,
    AirCond,
    Door,
    FltCtl,
    Wheel,
}

pub enum Sound {
    Cchord,
    Crc,
    Click,
    CavalryCharge,
    Cricket,
    Sc,
    Stall,
    Forty, // TODO should this go in synthetic voice instead?
}

pub enum WarningType {
    Status,
    Memo,
    SpecialLine,
    Failure,
    AutoCallOut,
}

pub struct WarningNodeBuilder<'a> {
    warning_type: WarningType,
    confirmation_delay: Option<Duration>,
    active_flight_phases: Option<&'a [u8]>,
    sys_page: Option<SysPage>,
    master_warning: Option<bool>,
    master_caution: Option<bool>,
    sound: Option<Sound>,
    clear: bool,
    recall: bool,
    cancel: bool,
    status: bool,
    emergency_cancel: bool,
    level: u8,
}

impl<'a> WarningNodeBuilder<'a> {
    pub fn new(warning_type: WarningType) -> Self {
        Self {
            warning_type,
            confirmation_delay: None,
            active_flight_phases: None,
            sys_page: None,
            master_warning: None,
            master_caution: None,
            sound: None,
            clear: false,
            recall: false,
            cancel: false,
            status: false,
            emergency_cancel: false,
            level: 0,
        }
    }

    pub fn confirmation_delay(mut self, confirmation_delay: Duration) -> Self {
        self.confirmation_delay = Some(confirmation_delay);
        self
    }

    pub fn sound(mut self, sound: Sound) -> Self {
        self.sound = Some(sound);
        self
    }

    pub fn master_warning(mut self) -> Self {
        self.master_warning = Some(true);
        self
    }

    pub fn master_caution(mut self) -> Self {
        self.master_caution = Some(true);
        self
    }

    fn in_flight_phases(mut self, flight_phases: &'a [u8]) -> Self {
        self.active_flight_phases = Some(flight_phases);
        self
    }

    fn in_all_flight_phases(mut self) -> Self {
        self.active_flight_phases = Some(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        self
    }

    fn sys_page(mut self, sys_page: SysPage) -> Self {
        self.sys_page = Some(sys_page);
        self
    }

    fn clear(mut self) -> Self {
        self.clear = true;
        self
    }

    fn status(mut self) -> Self {
        self.status = true;
        self
    }

    fn recall(mut self) -> Self {
        self.recall = true;
        self
    }

    fn cancel(mut self) -> Self {
        self.cancel = true;
        self
    }

    fn emergency_cancel(mut self) -> Self {
        // TODO
        self
    }

    pub fn build(self) -> WarningNode {
        WarningNode {}
    }
}

#[cfg(test)]
mod tests {
    use crate::flight_warning::warnings::{
        Sound, SysPage, WarningNode, WarningNodeBuilder, WarningType,
    };
    use std::time::Duration;

    #[test]
    fn ap_off_unvoluntary() {
        let node = WarningNodeBuilder::new(WarningType::Failure)
            .confirmation_delay(Duration::ZERO)
            .clear()
            .recall()
            .emergency_cancel()
            .in_all_flight_phases()
            .sound(Sound::CavalryCharge)
            .build();
    }

    #[test]
    fn ap_off_mv() {
        let node = WarningNodeBuilder::new(WarningType::Failure)
            .confirmation_delay(Duration::ZERO)
            .clear()
            .recall()
            .cancel()
            .emergency_cancel()
            .in_all_flight_phases()
            .master_warning()
            .build();
    }

    #[test]
    fn change_mode_pa_dv() {
        let node = WarningNodeBuilder::new(WarningType::Failure)
            .emergency_cancel()
            .in_flight_phases(&[1, 5, 6, 7])
            .sound(Sound::Click)
            .build();
    }

    #[test]
    fn eng_eng_out() {
        let node = WarningNodeBuilder::new(WarningType::Failure)
            .clear()
            .recall()
            .sys_page(SysPage::Elec)
            .build();
    }

    #[test]
    fn ldg_inhibit() {
        let node = WarningNodeBuilder::new(WarningType::SpecialLine);
    }

    #[test]
    fn elec_bat_bus_fault() {
        let node = WarningNodeBuilder::new(WarningType::Failure)
            .clear()
            .recall()
            .cancel()
            .emergency_cancel()
            .in_flight_phases(&[1, 2, 3, 6, 9, 10])
            .sound(Sound::Sc)
            .sys_page(SysPage::Elec)
            .build();
    }

    #[test]
    fn gen_1_inop() {
        let node = WarningNodeBuilder::new(WarningType::Status)
            .clear()
            .status()
            .build();
    }

    #[test]
    fn altitude_callout() {
        let node = WarningNodeBuilder::new(WarningType::AutoCallOut)
            .emergency_cancel()
            .sound(Sound::Forty)
            .confirmation_delay(Duration::ZERO)
            .build();
    }
}

#[cfg(test)]
mod warning_code_tests {
    use super::WarningCode;

    #[test]
    fn can_be_made_human_readable() {
        assert_eq!(WarningCode::new(0, 0, 10).to_string(), "00-00-010");
        assert_eq!(WarningCode::new(21, 27, 10).to_string(), "21-27-010");
        assert_eq!(WarningCode::new(26, 0, 130).to_string(), "26-00-130");
        assert_eq!(WarningCode::new(77, 0, 80).to_string(), "77-00-080");
    }

    #[test]
    fn can_be_compared() {
        assert_eq!(WarningCode::new(0, 0, 10), WarningCode::new(0, 0, 10));
        assert_eq!(WarningCode::new(21, 27, 10), WarningCode::new(21, 27, 10));
        assert_eq!(WarningCode::new(26, 0, 130), WarningCode::new(26, 0, 130));
        assert_eq!(WarningCode::new(77, 0, 80), WarningCode::new(77, 0, 80));
    }
}

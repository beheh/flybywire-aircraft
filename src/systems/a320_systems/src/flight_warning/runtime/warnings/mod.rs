use super::super::parameters::*;

mod auto_call_outs;
mod auto_flight;
mod engine;
mod flight_phases;
mod landing_gear;
mod memo;
mod monitor;

pub(in crate::flight_warning::runtime) use auto_call_outs::*;
pub(in crate::flight_warning::runtime) use auto_flight::*;
pub(in crate::flight_warning::runtime) use engine::*;
pub(in crate::flight_warning::runtime) use flight_phases::*;
pub(in crate::flight_warning::runtime) use landing_gear::*;
pub(in crate::flight_warning::runtime) use memo::*;
pub(in crate::flight_warning::runtime) use monitor::*;

pub(super) trait WarningActivation {
    fn audio(&self) -> bool {
        self.warning()
    }
    fn warning(&self) -> bool;
}

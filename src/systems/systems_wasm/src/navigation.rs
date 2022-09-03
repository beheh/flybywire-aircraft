#[cfg(not(target_arch = "wasm32"))]
use crate::msfs::legacy::execute_calculator_code;
#[cfg(target_arch = "wasm32")]
use msfs::legacy::execute_calculator_code;

use crate::{ExecuteOn, MsfsAspectBuilder, Variable};
use std::error::Error;
use systems::shared::{to_bool, ElectricalBusType};

pub(super) fn radio_altimeter_probes(
) -> impl FnOnce(&mut MsfsAspectBuilder) -> Result<(), Box<dyn Error>> {
    move |builder: &mut MsfsAspectBuilder| {
        builder.on_change(
            ExecuteOn::PostTick,
            vec![
                is_available_variable,
                Variable::aircraft("APU SWITCH", "Bool", 0),
            ],
            Box::new(move |_, values| {
                let is_available = to_bool(values[0]);
                let msfs_apu_is_on = to_bool(values[1]);

                if is_available && !msfs_apu_is_on {
                    toggle_fuel_valve(fuel_valve_number);
                    start_apu();
                } else if !is_available && msfs_apu_is_on {
                    toggle_fuel_valve(fuel_valve_number);
                    stop_apu();
                }
            }),
        );

        Ok(())
    }
}

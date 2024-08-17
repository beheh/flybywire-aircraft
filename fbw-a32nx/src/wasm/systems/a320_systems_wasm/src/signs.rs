use std::error::Error;

use systems_wasm::aspects::{EventToVariableMapping, MsfsAspectBuilder, VariablesToObject};
use systems_wasm::Variable;

pub(super) fn signs(builder: &mut MsfsAspectBuilder) -> Result<(), Box<dyn Error>> {
    builder.event_to_variable(
        "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE",
        EventToVariableMapping::CurrentValueToValue(
            |current_value| {
                if current_value > 1.5 {
                    0.
                } else {
                    2.
                }
            },
        ),
        Variable::named("OVHD_SIGNS_SEAT_BELTS"),
        |options| options.mask(),
    )?;

    builder.event_to_variable(
        "CABIN_NO_SMOKING_ALERT_SWITCH_TOGGLE",
        EventToVariableMapping::CurrentValueToValue(
            |current_value| {
                if current_value > 1.5 {
                    0.
                } else {
                    2.
                }
            },
        ),
        Variable::named("OVHD_SIGNS_NO_SMOKING"),
        |options| options.mask(),
    )?;

    Ok(())
}

use crate::flight_warning::parameters::{
    Arinc429Parameter, DiscreteParameter, SignStatusMatrix, SynchroParameter,
};

/// The FWC SSM trait can be used to expose additional
pub trait FwcSsm: SignStatusMatrix {
    fn is_val(&self) -> bool {
        !self.is_fw()
    }
    fn is_inv(&self) -> bool {
        self.is_fw()
    }
}

impl FwcSsm for DiscreteParameter {}

impl FwcSsm for SynchroParameter {}

impl<T> FwcSsm for Arinc429Parameter<T> {}

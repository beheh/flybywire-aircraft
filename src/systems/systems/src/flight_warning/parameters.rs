use crate::navigation::adirs::AdirsData;
use crate::shared::arinc429::SignStatus;
use uom::si::angle::degree;

pub trait Value<T> {
    fn value(&self) -> T;
}

/// The sign status matrix trait (SSM) encodes the two SSM bits, which are used by parameters to
/// encode certain statuses of the accompanied value. The trait additionally provides methods to
/// interpret the SSM bits.
pub trait SignStatusMatrix {
    fn ssm1(&self) -> bool;
    fn ssm2(&self) -> bool;

    /// A parameter has "no computed data" (ncd) when no explicit failure has been detected, but no
    /// data is available. For example, the radio altimeter in cruise won't return any valid data
    /// even if it has not failed.
    fn is_ncd(&self) -> bool;

    /// A parameter is considered "normal" (no) when the data is considered valid.
    fn is_no(&self) -> bool;

    // A parameter is in "functional test" (ft) when the parameter has been artificially forced to
    // certain value.
    fn is_ft(&self) -> bool;

    // A parameter is considered to have a "failure warning" (fw) when the data is most likely
    // faulty.
    fn is_fw(&self) -> bool;
}

pub struct DiscreteParameter {
    value: bool,
    ssm1: bool,
    ssm2: bool,
}

impl DiscreteParameter {
    pub fn new(value: bool) -> Self {
        Self {
            value: value,
            ssm1: false,
            ssm2: false,
        }
    }

    pub fn new_inv(value: bool) -> Self {
        Self {
            value: value,
            ssm1: true,
            ssm2: true,
        }
    }
}

impl SignStatusMatrix for DiscreteParameter {
    fn ssm1(&self) -> bool {
        self.ssm1
    }
    fn ssm2(&self) -> bool {
        self.ssm2
    }

    fn is_ncd(&self) -> bool {
        self.ssm1() && !self.ssm2()
    }
    fn is_no(&self) -> bool {
        !self.ssm1() && !self.ssm2()
    }
    fn is_ft(&self) -> bool {
        !self.ssm1() && self.ssm2()
    }
    fn is_fw(&self) -> bool {
        self.ssm1() && self.ssm2()
    }
}

impl Default for DiscreteParameter {
    fn default() -> Self {
        Self {
            value: false,
            ssm1: true,
            ssm2: true,
        }
    }
}

impl Value<bool> for DiscreteParameter {
    fn value(&self) -> bool {
        self.value
    }
}

pub struct SynchroParameter {
    value: degree,
    ssm1: bool,
    ssm2: bool,
}

impl SynchroParameter {
    pub fn new_synchro(value: degree) -> Self {
        Self {
            value: value,
            ssm1: true, // not between 0 and 360
            ssm2: true,
        }
    }

    pub fn new_rvdt(value: degree) -> Self {
        Self {
            value: value,
            ssm1: true, // not between -35 and 35
            ssm2: true,
        }
    }
}

impl SignStatusMatrix for SynchroParameter {
    fn ssm1(&self) -> bool {
        self.ssm1
    }
    fn ssm2(&self) -> bool {
        self.ssm2
    }

    fn is_ncd(&self) -> bool {
        self.ssm1() && !self.ssm2()
    }
    fn is_no(&self) -> bool {
        self.ssm1() && self.ssm2()
    }
    fn is_ft(&self) -> bool {
        !self.ssm1() && self.ssm2()
    }
    fn is_fw(&self) -> bool {
        !self.ssm1() && !self.ssm2()
    }
}

impl Value<degree> for SynchroParameter {
    fn value(&self) -> degree {
        self.value
    }
}

#[derive(Clone)]
pub struct Arinc429Parameter<T> {
    value: T,
    ssm1: bool,
    ssm2: bool,
}

impl<T> Arinc429Parameter<T> {
    pub fn new(value: T) -> Self {
        Self {
            value: value,
            ssm1: true,
            ssm2: true,
        }
    }

    pub fn new_ncd(value: T) -> Self {
        Self {
            value: value,
            ssm1: true,
            ssm2: false,
        }
    }

    pub fn new_inv(value: T) -> Self {
        Self {
            value: value,
            ssm1: false,
            ssm2: false,
        }
    }
}

impl<T> SignStatusMatrix for Arinc429Parameter<T> {
    fn ssm1(&self) -> bool {
        self.ssm1
    }
    fn ssm2(&self) -> bool {
        self.ssm2
    }

    fn is_ncd(&self) -> bool {
        self.ssm1() && !self.ssm2()
    }
    fn is_no(&self) -> bool {
        self.ssm1() && self.ssm2()
    }
    fn is_ft(&self) -> bool {
        !self.ssm1() && self.ssm2()
    }
    fn is_fw(&self) -> bool {
        !self.ssm1() && !self.ssm2()
    }
}

impl<T: Default> Default for Arinc429Parameter<T> {
    fn default() -> Self {
        Self {
            value: T::default(),
            ssm1: false,
            ssm2: false,
        }
    }
}

impl<T> Value<T> for Arinc429Parameter<T>
where
    T: Copy,
{
    fn value(&self) -> T {
        self.value
    }
}

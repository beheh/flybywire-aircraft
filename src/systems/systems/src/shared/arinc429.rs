use crate::shared::MachNumber;
use crate::simulation::SimulationElement;
use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;
use uom::si::f64::*;

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub struct Arinc429Discretes {
    values: [bool; 19],
}

impl Arinc429Discretes {
    pub fn new(values: [bool; 19]) -> Self {
        Self { values }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Arinc429Label<T> {
    label: u8,
    sdi: u8,
    _marker: PhantomData<T>,
}

impl<T> Arinc429Label<T> {
    pub const fn new(label: u8, sdi: u8) -> Self {
        Self {
            label,
            sdi,
            _marker: PhantomData,
        }
    }

    pub fn label(&self) -> u8 {
        self.label
    }

    pub fn source_destination_identifier(&self) -> u8 {
        self.sdi
    }
}

impl<T> Hash for Arinc429Label<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.label.hash(state);
        self.sdi.hash(state);
    }
}

impl<T> PartialEq<Self> for Arinc429Label<T> {
    fn eq(&self, other: &Self) -> bool {
        self.label == other.label && self.sdi == other.sdi
    }
}

impl<T> Eq for Arinc429Label<T> {}

#[derive(PartialEq, Clone, Copy)]
pub struct Arinc429Word<T: Copy> {
    label: Arinc429Label<T>,
    value: T,
    ssm: SignStatus,
}

impl<T: Copy> Arinc429Word<T> {
    pub fn new(label: Arinc429Label<T>, value: T, ssm: SignStatus) -> Self {
        Self { label, value, ssm }
    }

    pub fn new_norm(label: Arinc429Label<T>, value: T) -> Self {
        Self {
            label,
            value,
            ssm: SignStatus::NormalOperation,
        }
    }

    pub fn value(&self) -> T {
        self.value
    }

    pub fn label(&self) -> Arinc429Label<T> {
        self.label
    }

    pub fn ssm(&self) -> SignStatus {
        self.ssm
    }

    pub fn is_normal(&self) -> bool {
        matches!(self.ssm, SignStatus::NormalOperation)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SignStatus {
    FailureWarning,
    FunctionalTest,
    NoComputedData,
    NormalOperation,
}

impl From<SignStatus> for u64 {
    fn from(value: SignStatus) -> Self {
        match value {
            SignStatus::FailureWarning => 0b00,
            SignStatus::FunctionalTest => 0b01,
            SignStatus::NoComputedData => 0b10,
            SignStatus::NormalOperation => 0b11,
        }
    }
}

impl From<u32> for SignStatus {
    fn from(value: u32) -> Self {
        match value {
            0b00 => SignStatus::FailureWarning,
            0b01 => SignStatus::FunctionalTest,
            0b10 => SignStatus::NoComputedData,
            0b11 => SignStatus::NormalOperation,
            _ => panic!("Unknown SSM value: {}.", value),
        }
    }
}

pub(crate) fn from_arinc429(value: f64) -> (f64, SignStatus) {
    let bits = value.to_bits();

    let value = (bits >> 32) as u32;
    let status = bits as u32;

    (f32::from_bits(value) as f64, status.into())
}

pub(crate) fn to_arinc429(value: f64, ssm: SignStatus) -> f64 {
    let value = value as f32;
    let status: u64 = ssm.into();

    let bits = (value.to_bits() as u64) << 32 | status;

    f64::from_bits(bits)
}

pub struct Arinc429LabelBuilder<T> {
    label: u8,
    _marker: PhantomData<T>,
}

impl<T> Arinc429LabelBuilder<T> {
    pub const fn new(label: u8) -> Self {
        Self {
            label,
            _marker: PhantomData,
        }
    }

    pub const fn build(&self, sdi: u8) -> Arinc429Label<T> {
        Arinc429Label::new(self.label, sdi)
    }
}

pub struct Arinc429DiscretesWordBuilder {
    label: Arinc429Label<Arinc429Discretes>,
    value: [bool; 19],
}

impl Arinc429DiscretesWordBuilder {
    pub fn new(label: Arinc429Label<Arinc429Discretes>) -> Self {
        Self {
            label,
            value: [false; 19],
        }
    }

    pub fn set(&mut self, bit: usize, value: bool) {
        self.value[bit - 11] = value
    }

    pub fn build(&self, ssm: SignStatus) -> Arinc429Word<Arinc429Discretes> {
        Arinc429Word::new(self.label, Arinc429Discretes::new(self.value), ssm)
    }
}

pub trait Arinc429Reader<T: Copy> {
    fn read_value(&self, label: Arinc429Label<T>) -> Option<&Arinc429Word<T>>;
}

pub trait Arinc429Writer<T: Copy> {
    fn write_value(&mut self, word: Arinc429Word<T>);
}

#[derive(Clone, Default)]
pub struct Arinc429Bus {
    read_from_a: bool,
    table_a: Arinc429Table,
    table_b: Arinc429Table,
}

impl Arinc429Bus {
    pub fn new() -> Self {
        Self {
            read_from_a: false,
            table_a: Arinc429Table::new(),
            table_b: Arinc429Table::new(),
        }
    }

    fn table_to_read(&self) -> &Arinc429Table {
        if self.read_from_a {
            &self.table_a
        } else {
            &self.table_b
        }
    }

    fn table_to_write(&mut self) -> &mut Arinc429Table {
        if !self.read_from_a {
            &mut self.table_a
        } else {
            &mut self.table_b
        }
    }
}

impl SimulationElement for Arinc429Bus {
    fn post_tick(&mut self) {
        // clear the read table, as it will become the new write table
        if self.read_from_a {
            self.table_a.clear()
        } else {
            self.table_b.clear()
        }
        // swap the read/write tables
        self.read_from_a = !self.read_from_a
    }
}

macro_rules! bus_to_table_read_write {
    ($t: ty) => {
        impl Arinc429Reader<$t> for Arinc429Bus {
            fn read_value(&self, label: Arinc429Label<$t>) -> Option<&Arinc429Word<$t>> {
                self.table_to_read().read_value(label)
            }
        }

        impl Arinc429Writer<$t> for Arinc429Bus {
            fn write_value(&mut self, word: Arinc429Word<$t>) {
                self.table_to_write().write_value(word)
            }
        }
    };
}

bus_to_table_read_write!(Arinc429Discretes);
bus_to_table_read_write!(bool);
bus_to_table_read_write!(f64);

#[derive(Clone, Default)]
pub struct Arinc429Table {
    discretes: HashMap<Arinc429Label<Arinc429Discretes>, Arinc429Word<Arinc429Discretes>>,
    bools: HashMap<Arinc429Label<bool>, Arinc429Word<bool>>,
    f64s: HashMap<Arinc429Label<f64>, Arinc429Word<f64>>,
    lengths: HashMap<Arinc429Label<Length>, Arinc429Word<Length>>,
    velocities: HashMap<Arinc429Label<Velocity>, Arinc429Word<Velocity>>,
    mach_numbers: HashMap<Arinc429Label<MachNumber>, Arinc429Word<MachNumber>>,
    thermodynamic_temperatures:
        HashMap<Arinc429Label<ThermodynamicTemperature>, Arinc429Word<ThermodynamicTemperature>>,
}

impl Arinc429Table {
    fn new() -> Self {
        Self {
            discretes: HashMap::new(),
            bools: HashMap::new(),
            f64s: HashMap::new(),
            lengths: HashMap::new(),
            velocities: HashMap::new(),
            mach_numbers: HashMap::new(),
            thermodynamic_temperatures: HashMap::new(),
        }
    }

    fn clear(&mut self) {
        self.discretes.clear();
        self.bools.clear();
        self.f64s.clear();
        self.lengths.clear();
        self.velocities.clear();
        self.mach_numbers.clear();
        self.thermodynamic_temperatures.clear();
    }
}

macro_rules! table_read_write {
    ($t: ty, $t2: tt) => {
        impl Arinc429Reader<$t> for Arinc429Table {
            fn read_value(&self, label: Arinc429Label<$t>) -> Option<&Arinc429Word<$t>> {
                self.$t2.get(&label)
            }
        }

        impl Arinc429Writer<$t> for Arinc429Table {
            fn write_value(&mut self, word: Arinc429Word<$t>) {
                self.$t2.insert(word.label(), word);
            }
        }
    };
}

table_read_write!(Arinc429Discretes, discretes);
table_read_write!(bool, bools);
table_read_write!(f64, f64s);

#[cfg(test)]
mod tests {
    use super::*;
    use rand::Rng;
    use rstest::rstest;

    #[rstest]
    #[case(SignStatus::FailureWarning)]
    #[case(SignStatus::FunctionalTest)]
    #[case(SignStatus::NoComputedData)]
    #[case(SignStatus::NormalOperation)]
    fn conversion_is_symmetric(#[case] expected_ssm: SignStatus) {
        let mut rng = rand::thread_rng();
        let expected_value: f64 = rng.gen_range(0.0..10000.0);

        let result = from_arinc429(to_arinc429(expected_value, expected_ssm));

        assert!(
            (result.0 - expected_value).abs() < 0.001,
            "Expected: {}, got: {}",
            expected_value,
            result.0
        );
        assert_eq!(expected_ssm, result.1);
    }
}

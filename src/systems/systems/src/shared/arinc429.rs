pub struct Arinc429Word<T: Copy> {
    value: T,
    ssm: SignStatus,
}
impl<T: Copy> Arinc429Word<T> {
    pub fn new(value: T, ssm: SignStatus) -> Self {
        Self { value, ssm }
    }

    pub fn value(&self) -> T {
        self.value
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

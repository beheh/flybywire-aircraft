use std::fmt::Debug;
use std::time::Duration;
use systems::flight_warning::logic::PulseNode;
use uom::si::f64::*;
use uom::si::length::foot;

#[derive(Copy, Clone, Debug, PartialEq)]
pub(super) enum SyntheticVoice {
    Minimum,
    HundredAbove,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    Ten,
    Eleven,
    Twelve,
    Thirteen,
    Fourteen,
    Fifteen,
    Sixteen,
    Seventeen,
    Eighteen,
    Nineteen,
    Twenty,
    Thirty,
    Forty,
    Fifty,
    Sixty,
    Seventy,
    Eighty,
    Ninety,
    OneHundred,
    TwoHundred,
    ThreeHundred,
    FourHundred,
    FiveHundred,
    OneThousand,
    TwoThousand,
    TwoThousandFiveHundred,
    TwentyFiveHundred,
    OneC,
    TwoC,
    ThreeC,
    FourC,
    FiveC,
    TwentyC,
    ThirtyC,
    FortyC,
    FiftyC,
    SixtyC,
    SeventyC,
    EightyC,
    NinetyC,
    Hundredand,
}

pub(super) struct SyntheticVoiceFile {
    id: u8,
    duration: Duration,
}

/// ID space:
/// 80: HundredAbove, Minimum
/// 100s: 1, 2, ..9
/// 110s: 10, 11, 12, ..19
/// 120s: 20, 30, 40, ..90
/// 130s: 100, 200, 300, 400, 500, 1000, 2000, 2500, 2_500
/// 140s: 1c, 2c, 3c, 4c, 5c
/// 150s: 20c, 30c, ...90c
/// 160: hundredand

impl SyntheticVoice {
    fn get_voice_file(&self) -> SyntheticVoiceFile {
        return match self {
            SyntheticVoice::Minimum => SyntheticVoiceFile {
                id: 80,
                duration: Duration::from_millis(670),
            },
            SyntheticVoice::HundredAbove => SyntheticVoiceFile {
                id: 81,
                duration: Duration::from_millis(720),
            },
            SyntheticVoice::One => SyntheticVoiceFile {
                id: 101,
                duration: Duration::from_millis(339),
            },
            SyntheticVoice::Two => SyntheticVoiceFile {
                id: 102,
                duration: Duration::from_millis(436),
            },
            SyntheticVoice::Three => SyntheticVoiceFile {
                id: 103,
                duration: Duration::from_millis(445),
            },
            SyntheticVoice::Four => SyntheticVoiceFile {
                id: 104,
                duration: Duration::from_millis(370),
            },
            SyntheticVoice::Five => SyntheticVoiceFile {
                id: 105,
                duration: Duration::from_millis(483),
            },
            SyntheticVoice::Six => SyntheticVoiceFile {
                id: 106,
                duration: Duration::from_millis(525),
            },
            SyntheticVoice::Seven => SyntheticVoiceFile {
                id: 107,
                duration: Duration::from_millis(538),
            },
            SyntheticVoice::Eight => SyntheticVoiceFile {
                id: 108,
                duration: Duration::from_millis(393),
            },
            SyntheticVoice::Nine => SyntheticVoiceFile {
                id: 109,
                duration: Duration::from_millis(435),
            },
            SyntheticVoice::Ten => SyntheticVoiceFile {
                id: 110,
                duration: Duration::from_millis(340),
            },
            SyntheticVoice::Eleven => SyntheticVoiceFile {
                id: 111,
                duration: Duration::from_millis(656),
            },
            SyntheticVoice::Twelve => SyntheticVoiceFile {
                id: 112,
                duration: Duration::from_millis(576),
            },
            SyntheticVoice::Thirteen => SyntheticVoiceFile {
                id: 113,
                duration: Duration::from_millis(728),
            },
            SyntheticVoice::Fourteen => SyntheticVoiceFile {
                id: 114,
                duration: Duration::from_millis(775),
            },
            SyntheticVoice::Fifteen => SyntheticVoiceFile {
                id: 115,
                duration: Duration::from_millis(715),
            },
            SyntheticVoice::Sixteen => SyntheticVoiceFile {
                id: 116,
                duration: Duration::from_millis(766),
            },
            SyntheticVoice::Seventeen => SyntheticVoiceFile {
                id: 117,
                duration: Duration::from_millis(812),
            },
            SyntheticVoice::Eighteen => SyntheticVoiceFile {
                id: 118,
                duration: Duration::from_millis(698),
            },
            SyntheticVoice::Nineteen => SyntheticVoiceFile {
                id: 119,
                duration: Duration::from_millis(830),
            },
            SyntheticVoice::Twenty => SyntheticVoiceFile {
                id: 122,
                duration: Duration::from_millis(480),
            },
            SyntheticVoice::Thirty => SyntheticVoiceFile {
                id: 123,
                duration: Duration::from_millis(499),
            },
            SyntheticVoice::Forty => SyntheticVoiceFile {
                id: 124,
                duration: Duration::from_millis(493),
            },
            SyntheticVoice::Fifty => SyntheticVoiceFile {
                id: 125,
                duration: Duration::from_millis(537),
            },
            SyntheticVoice::Sixty => SyntheticVoiceFile {
                id: 126,
                duration: Duration::from_millis(506),
            },
            SyntheticVoice::Seventy => SyntheticVoiceFile {
                id: 127,
                duration: Duration::from_millis(537),
            },
            SyntheticVoice::Eighty => SyntheticVoiceFile {
                id: 128,
                duration: Duration::from_millis(394),
            },
            SyntheticVoice::Ninety => SyntheticVoiceFile {
                id: 129,
                duration: Duration::from_millis(473),
            },
            SyntheticVoice::OneHundred => SyntheticVoiceFile {
                id: 131,
                duration: Duration::from_millis(652),
            },
            SyntheticVoice::TwoHundred => SyntheticVoiceFile {
                id: 132,
                duration: Duration::from_millis(685),
            },
            SyntheticVoice::ThreeHundred => SyntheticVoiceFile {
                id: 133,
                duration: Duration::from_millis(731),
            },
            SyntheticVoice::FourHundred => SyntheticVoiceFile {
                id: 134,
                duration: Duration::from_millis(753),
            },
            SyntheticVoice::FiveHundred => SyntheticVoiceFile {
                id: 135,
                duration: Duration::from_millis(739),
            },
            SyntheticVoice::OneThousand => SyntheticVoiceFile {
                id: 136,
                duration: Duration::from_millis(790),
            },
            SyntheticVoice::TwoThousand => SyntheticVoiceFile {
                id: 137,
                duration: Duration::from_millis(711),
            },
            SyntheticVoice::TwoThousandFiveHundred => SyntheticVoiceFile {
                id: 138,
                duration: Duration::from_millis(1_331),
            },
            SyntheticVoice::TwentyFiveHundred => SyntheticVoiceFile {
                id: 139,
                duration: Duration::from_millis(1_047),
            },
            SyntheticVoice::OneC => SyntheticVoiceFile {
                id: 141,
                duration: Duration::from_millis(228),
            },
            SyntheticVoice::TwoC => SyntheticVoiceFile {
                id: 142,
                duration: Duration::from_millis(229),
            },
            SyntheticVoice::ThreeC => SyntheticVoiceFile {
                id: 143,
                duration: Duration::from_millis(255),
            },
            SyntheticVoice::FourC => SyntheticVoiceFile {
                id: 144,
                duration: Duration::from_millis(286),
            },
            SyntheticVoice::FiveC => SyntheticVoiceFile {
                id: 145,
                duration: Duration::from_millis(309),
            },
            SyntheticVoice::TwentyC => SyntheticVoiceFile {
                id: 152,
                duration: Duration::from_millis(299),
            },
            SyntheticVoice::ThirtyC => SyntheticVoiceFile {
                id: 153,
                duration: Duration::from_millis(389),
            },
            SyntheticVoice::FortyC => SyntheticVoiceFile {
                id: 154,
                duration: Duration::from_millis(392),
            },
            SyntheticVoice::FiftyC => SyntheticVoiceFile {
                id: 155,
                duration: Duration::from_millis(314),
            },
            SyntheticVoice::SixtyC => SyntheticVoiceFile {
                id: 156,
                duration: Duration::from_millis(403),
            },
            SyntheticVoice::SeventyC => SyntheticVoiceFile {
                id: 157,
                duration: Duration::from_millis(536),
            },
            SyntheticVoice::EightyC => SyntheticVoiceFile {
                id: 158,
                duration: Duration::from_millis(295),
            },
            SyntheticVoice::NinetyC => SyntheticVoiceFile {
                id: 159,
                duration: Duration::from_millis(538),
            },
            SyntheticVoice::Hundredand => SyntheticVoiceFile {
                id: 160,
                duration: Duration::from_millis(442),
            },
        };
    }
}

struct PlayingSound {
    file: SyntheticVoiceFile,
    elapsed: Duration,
}

impl PlayingSound {
    pub fn elapse(&mut self, delta: Duration) {
        self.elapsed += delta;
    }

    fn remaining(&self) -> Duration {
        self.file.duration - self.elapsed
    }

    fn complete(&self) -> bool {
        self.elapsed >= self.file.duration
    }
}

#[derive(Default)]
pub(super) struct VoiceSynthesizer {
    playing: Option<PlayingSound>,
}

impl VoiceSynthesizer {
    pub fn update(&mut self, delta: Duration, requested_sound: Option<SyntheticVoice>) {
        if let Some(sound) = requested_sound {
            // play something new
            self.playing = Some(PlayingSound {
                file: sound.get_voice_file(),
                elapsed: Duration::ZERO,
            });
        } else if let Some(ref mut playing) = self.playing {
            // continue playing
            playing.elapse(delta);
            if playing.complete() {
                self.playing = None;
            }
        }
    }

    pub fn callout_sound_id(&self) -> Option<u8> {
        match &self.playing {
            Some(p) => Some(p.file.id),
            None => None,
        }
    }

    pub fn playing(&self) -> bool {
        self.playing.is_some()
    }

    pub fn ready_in(&self, delta: Duration) -> bool {
        if let Some(playing) = &self.playing {
            playing.remaining() <= delta
        } else {
            true
        }
    }

    pub fn ready(&self) -> bool {
        self.playing.is_none()
    }
}

#[derive(Default)]
pub struct SyntheticVoiceManager {
    sequence: Vec<SyntheticVoice>,
    next_index: usize,
    synthesizer: VoiceSynthesizer,
}

impl SyntheticVoiceManager {
    pub(super) fn update(
        &mut self,
        delta: Duration,
        parts: Option<Vec<SyntheticVoice>>,
        cancel: bool,
    ) {
        let mut requested_sound: Option<SyntheticVoice> = None;
        if self.synthesizer.ready_in(delta) || cancel {
            if self.next_index < self.sequence.len() && !cancel {
                // we have parts still to play
                requested_sound = Some(self.sequence[self.next_index].clone());
                self.next_index = self.next_index + 1;
            } else {
                // we can play something new
                if let Some(new_parts) = parts {
                    if new_parts.len() > 0 {
                        self.sequence = new_parts;
                        requested_sound = Some(self.sequence[0].clone());
                        self.next_index = 1;
                    }
                }
            }
        }
        self.synthesizer.update(delta, requested_sound);
    }

    pub fn callout_sound_id(&self) -> Option<u8> {
        self.synthesizer.callout_sound_id()
    }

    pub fn ready(&self) -> bool {
        self.synthesizer.ready()
    }

    pub fn ready_in(&self, delta: Duration) -> bool {
        self.synthesizer.ready_in(delta)
    }
}

pub(super) fn translate_height(raw_height: Length) -> Option<Vec<SyntheticVoice>> {
    if raw_height < Length::new::<foot>(1.0) || raw_height >= Length::new::<foot>(400.0) {
        return None;
    }
    let height: i32 = (if raw_height < Length::new::<foot>(100.0) {
        // round to nearest foot
        (raw_height.get::<foot>()).round()
    } else {
        // round to nearest ten feet
        ((raw_height.get::<foot>()) / 10.0).round() * 10.0
    }) as i32;
    if height > 400 {
        return None;
    }
    if height < 100 {
        if height < 20 || height % 10 == 0 {
            return match height {
                1 => Some(vec![SyntheticVoice::One]),
                2 => Some(vec![SyntheticVoice::Two]),
                3 => Some(vec![SyntheticVoice::Three]),
                4 => Some(vec![SyntheticVoice::Four]),
                5 => Some(vec![SyntheticVoice::Five]),
                6 => Some(vec![SyntheticVoice::Six]),
                7 => Some(vec![SyntheticVoice::Seven]),
                8 => Some(vec![SyntheticVoice::Eight]),
                9 => Some(vec![SyntheticVoice::Nine]),
                10 => Some(vec![SyntheticVoice::Ten]),
                11 => Some(vec![SyntheticVoice::Eleven]),
                12 => Some(vec![SyntheticVoice::Twelve]),
                13 => Some(vec![SyntheticVoice::Thirteen]),
                14 => Some(vec![SyntheticVoice::Fourteen]),
                15 => Some(vec![SyntheticVoice::Fifteen]),
                16 => Some(vec![SyntheticVoice::Sixteen]),
                17 => Some(vec![SyntheticVoice::Seventeen]),
                18 => Some(vec![SyntheticVoice::Eighteen]),
                19 => Some(vec![SyntheticVoice::Nineteen]),
                20 => Some(vec![SyntheticVoice::Twenty]),
                30 => Some(vec![SyntheticVoice::Thirty]),
                40 => Some(vec![SyntheticVoice::Forty]),
                50 => Some(vec![SyntheticVoice::Fifty]),
                60 => Some(vec![SyntheticVoice::Sixty]),
                70 => Some(vec![SyntheticVoice::Seventy]),
                80 => Some(vec![SyntheticVoice::Eighty]),
                90 => Some(vec![SyntheticVoice::Ninety]),
                _ => None,
            };
        } else {
            // prefix
            let prefix = match height / 10 {
                2 => SyntheticVoice::TwentyC,
                3 => SyntheticVoice::ThirtyC,
                4 => SyntheticVoice::FortyC,
                5 => SyntheticVoice::FiftyC,
                6 => SyntheticVoice::SixtyC,
                7 => SyntheticVoice::SeventyC,
                8 => SyntheticVoice::EightyC,
                9 => SyntheticVoice::NinetyC,
                _ => panic!("wat"),
            };
            let suffix = match height % 10 {
                1 => SyntheticVoice::One,
                2 => SyntheticVoice::Two,
                3 => SyntheticVoice::Three,
                4 => SyntheticVoice::Four,
                5 => SyntheticVoice::Five,
                6 => SyntheticVoice::Six,
                7 => SyntheticVoice::Seven,
                8 => SyntheticVoice::Eight,
                9 => SyntheticVoice::Nine,
                _ => panic!("wat"),
            };
            return Some(vec![prefix, suffix]);
        }
    } else if height >= 100 {
        if height % 100 == 0 {
            return match height / 100 {
                1 => Some(vec![SyntheticVoice::OneHundred]),
                2 => Some(vec![SyntheticVoice::TwoHundred]),
                3 => Some(vec![SyntheticVoice::ThreeHundred]),
                4 => Some(vec![SyntheticVoice::FourHundred]),
                _ => None,
            };
        } else {
            let prefix = match height / 100 {
                1 => SyntheticVoice::OneC,
                2 => SyntheticVoice::TwoC,
                3 => SyntheticVoice::ThreeC,
                4 => SyntheticVoice::FourC,
                _ => panic!("wat"),
            };
            let suffix = match (height % 100) / 10 {
                1 => SyntheticVoice::Ten,
                2 => SyntheticVoice::Twenty,
                3 => SyntheticVoice::Thirty,
                4 => SyntheticVoice::Forty,
                5 => SyntheticVoice::Fifty,
                6 => SyntheticVoice::Sixty,
                7 => SyntheticVoice::Seventy,
                8 => SyntheticVoice::Eighty,
                9 => SyntheticVoice::Ninety,
                x => panic!(format!("wat {}", x)),
            };
            return Some(vec![prefix, SyntheticVoice::Hundredand, suffix]);
        }
    }
    return None;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(test)]
    mod test_voice_synthesizer {
        use super::*;

        #[test]
        fn plays_a_file() {
            let mut synthesizer = VoiceSynthesizer::default();
            synthesizer.update(
                Duration::from_millis(1000),
                Some(SyntheticVoice::TwoThousandFiveHundred),
            );
            assert!(!synthesizer.ready());
            synthesizer.update(Duration::from_millis(1000), None);
            assert!(!synthesizer.ready());
            synthesizer.update(Duration::from_millis(2000), None);
            assert!(synthesizer.ready());
        }
    }

    #[cfg(test)]
    mod test_translate_height {
        use uom::si::f64::*;
        use uom::si::length::foot;

        use super::*;

        fn assert_callout(height: f64, expected: Vec<SyntheticVoice>) {
            assert_eq!(
                translate_height(Length::new::<foot>(height)),
                Some(expected)
            );
        }

        #[test]
        fn when_out_of_range_returns_none() {
            assert!(translate_height(Length::new::<foot>(0.)).is_none());
        }

        #[test]
        fn when_below_100ft_rounds_to_nearest_foot() {
            assert_callout(4., vec![SyntheticVoice::Four]);
            assert_callout(4.3, vec![SyntheticVoice::Four]);
            assert_callout(4.5, vec![SyntheticVoice::Five]);
            assert_callout(11., vec![SyntheticVoice::Eleven]);
            assert_callout(21., vec![SyntheticVoice::TwentyC, SyntheticVoice::One]);
            assert_callout(42., vec![SyntheticVoice::FortyC, SyntheticVoice::Two]);
            assert_callout(93., vec![SyntheticVoice::NinetyC, SyntheticVoice::Three]);
        }

        #[test]
        fn when_above_100ft_rounds_to_nearest_10ft() {
            assert_callout(
                194.,
                vec![
                    SyntheticVoice::OneC,
                    SyntheticVoice::Hundredand,
                    SyntheticVoice::Ninety,
                ],
            );
            assert_callout(195., vec![SyntheticVoice::TwoHundred]);
            assert_callout(200., vec![SyntheticVoice::TwoHundred]);
            assert_callout(204., vec![SyntheticVoice::TwoHundred]);
            assert_callout(
                209.,
                vec![
                    SyntheticVoice::TwoC,
                    SyntheticVoice::Hundredand,
                    SyntheticVoice::Ten,
                ],
            );
            assert_callout(
                210.,
                vec![
                    SyntheticVoice::TwoC,
                    SyntheticVoice::Hundredand,
                    SyntheticVoice::Ten,
                ],
            );
            assert_callout(
                210.1,
                vec![
                    SyntheticVoice::TwoC,
                    SyntheticVoice::Hundredand,
                    SyntheticVoice::Ten,
                ],
            );
        }

        #[test]
        fn resolves() {
            for i in 1..400 {
                assert!(
                    translate_height(Length::new::<foot>(i as f64)).is_some(),
                    "{} returned None",
                    i
                );
            }
        }
    }
}

use std::time::Duration;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum AudioCode {
    LowChime = 1,
    HiLowChime = 2,
}

#[derive(Copy, Clone, Debug)]
struct AudioData {
    code: Option<AudioCode>,
    duration: Duration,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Sound {
    GenericChime,
    CabinCall,
    EmergencyCall,
}

impl Sound {
    fn audio_files(&self) -> Vec<AudioData> {
        match self {
            Sound::GenericChime => vec![
                AudioData {
                    code: None,
                    duration: Duration::from_millis(1_350),
                },
                AudioData {
                    code: Some(AudioCode::LowChime),
                    duration: Duration::from_millis(/*2_700 - 1_350*/ 1_683), // 1.6826s
                },
            ],
            Sound::CabinCall => vec![
                AudioData {
                    code: None,
                    duration: Duration::from_millis(1_350),
                },
                AudioData {
                    code: Some(AudioCode::HiLowChime),
                    duration: Duration::from_millis(3_600 - 1_350), // 2.23555
                },
            ],
            Sound::EmergencyCall => vec![
                AudioData {
                    code: None,
                    duration: Duration::from_millis(1_350),
                },
                AudioData {
                    code: Some(AudioCode::HiLowChime),
                    duration: Duration::from_millis(3_600 - 1_350),
                },
                AudioData {
                    code: Some(AudioCode::HiLowChime),
                    duration: Duration::from_millis(3_600 - 1_350),
                },
                AudioData {
                    code: Some(AudioCode::HiLowChime),
                    duration: Duration::from_millis(3_600 - 1_350),
                },
            ],
        }
    }
}

#[derive(Debug)]
struct PlayingFile {
    audio_data: AudioData,
    elapsed: Duration,
}

impl PlayingFile {
    fn elapse(&mut self, delta: Duration) {
        self.elapsed += delta;
    }

    fn total_duration(&self) -> Duration {
        self.audio_data.duration
    }

    fn remaining(&self) -> Duration {
        self.total_duration()
            .checked_sub(self.elapsed)
            .unwrap_or(Duration::ZERO)
    }

    fn complete(&self) -> bool {
        self.elapsed >= self.total_duration()
    }
}

#[derive(Default)]
pub(super) struct AudioSynthesizer {
    playing: Option<PlayingFile>,
    audio_stack: Vec<AudioData>,
    started: bool,
}

impl AudioSynthesizer {
    pub fn update(&mut self, delta: Duration, new_sound: Option<Sound>) {
        if let Some(ref mut playing) = self.playing {
            // continue playing
            self.started = true;
            playing.elapse(delta);
            if playing.complete() {
                println!("Complete! {:?}", playing);
                if let Some(audio_data) = self.audio_stack.pop() {
                    println!("Queuing {:?}", audio_data);
                    self.playing = Some(PlayingFile {
                        audio_data,
                        elapsed: Duration::ZERO,
                    });
                    self.started = false;
                } else {
                    self.playing = None;
                }
            }
        }

        if self.playing.is_none() {
            if let Some(audio_data) = new_sound {
                println!("New sound! {:?}", audio_data);
                // build the queue
                self.audio_stack = audio_data.audio_files().into_iter().rev().collect();
                // play something new
                self.playing = Some(PlayingFile {
                    audio_data: self.audio_stack.pop().unwrap(),
                    elapsed: Duration::ZERO,
                });
                self.started = false;
            }
        }
    }

    pub(super) fn audio_code(&self) -> Option<u8> {
        if self.started {
            None
        } else {
            self.playing
                .as_ref()
                .and_then(|p| p.audio_data.code.map(|p| p as u8))
        }
    }

    /// Returns the PlayingSound the synthesizer will play in delta, or none if the synthesizer is
    /// not playing anything or will have finished playing the currently playing sound in delta.
    fn playing_in(&self, delta: Duration) -> Option<&PlayingFile> {
        self.playing
            .as_ref()
            .filter(|playing| playing.remaining() > delta)
    }

    /// Returns whether the synthesizer will be ready to play another sound in delta.
    /// This is either because there is currently no sound playing, the currently playing sound will
    /// have ended, or the currently playing sound is interruptible.
    /// This is useful when you want to check whether the synthesizer is ready to play a sound in
    /// the update call before calling update. For example, if a sound just completed playing but is
    /// still marked as playing in the synthesizer, you can check whether the synthesizer will be
    //     /// ready to accept a new sound by the time you call update (with delta) by passing the same
    /// delta to this function ahead of time.
    pub(super) fn ready_in(&self, delta: Duration) -> bool {
        self.playing_in(delta).is_none()
    }
}

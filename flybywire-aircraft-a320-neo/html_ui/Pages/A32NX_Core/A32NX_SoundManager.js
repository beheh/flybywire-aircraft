class PeriodicSound {
    constructor(sound, period) {
        this.sound = sound;
        this.period = period;
        this.timeSinceLastPlayed = NaN;
    }
}

class A32NX_SoundManager {
    constructor() {
        this.periodicList = [];

        this.playingSound = null;
        this.playingSoundRemaining = NaN;
    }

    addPeriodicSound(sound, period = NaN) {
        if (!sound) {
            return;
        }

        let useLengthForPeriod = false;
        if (period < sound.length) {
            console.error("A32NXSoundManager ERROR: Sound period can't be smaller than sound length. Using sound length instead.");
            useLengthForPeriod = true;
        }

        let found = false;
        this.periodicList.forEach((element) => {
            if (element.name === sound.name) {
                found = true;
            }
        });

        if (!found) {
            this.periodicList.push(new PeriodicSound(sound, useLengthForPeriod ? sound.length : period));
        }
    }

    removePeriodicSound(sound) {
        if (!sound) {
            return;
        }

        for (let i = 0; i < this.periodicList.length; i++) {
            if (this.periodicList[i].sound.name === sound.name) {
                this.periodicList.splice(i, 1);
            }
        }
    }

    tryPlaySound(sound) {
        if (this.playingSound === null) {
            this.playingSound = sound;
            this.playingSoundRemaining = sound.length;

            Coherent.call("PLAY_INSTRUMENT_SOUND", sound.name).catch(console.error);
            return true;
        }
        return false;
    }

    update(deltaTime, _core) {
        if (this.playingSoundRemaining <= 0) {
            this.playingSound = null;
            this.playingSoundRemaining = NaN;
        } else if (this.playingSoundRemaining > 0) {
            this.playingSoundRemaining -= deltaTime / 1000;
        }

        this.periodicList.forEach((element) => {
            if (isNaN(element.timeSinceLastPlayed) || element.timeSinceLastPlayed >= element.period) {
                if (this.tryPlaySound(element.sound)) {
                    element.timeSinceLastPlayed = 0;
                }
            } else {
                element.timeSinceLastPlayed += deltaTime / 1000;
            }
        });
    }
}

// many lengths are approximate until we can get them accuratly (when boris re-makes them and we have the sources)
const soundList = {
    pull_up: {
        name: "aural_pullup_new",
        length: 0.9
    },
    sink_rate: {
        name: "aural_sink_rate_new",
        length: 0.9
    },
    dont_sink:{
        name: "aural_dontsink_new",
        length: 0.9
    },
    too_low_gear:{
        name: "aural_too_low_gear",
        length: 0.8
    },
    too_low_flaps:{
        name: "aural_too_low_flaps",
        length: 0.8
    },
    too_low_terrain: {
        name: "aural_too_low_terrain",
        length: 0.9
    },
    minimums: {
        name: "aural_minimumnew",
        length: 0.67
    },
    hundred_above: {
        name: "aural_100above",
        length: 0.72
    },
    retard: {
        name: "new_retard",
        length: 0.9
    },
};

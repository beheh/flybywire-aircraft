class A32NX_FWC {
    constructor() {
        // momentary
        this.toConfigTest = null;
        this.flightPhaseEndedPulse = false;

        // persistent
        this.flightPhase = null;
        this.ldgMemo = null;
        this.toMemo = null;

        this.memoFlightPhaseInhibOvrd_memo = new NXLogic_MemoryNode(false);

        this.memoTo_conf01 = new NXLogic_ConfirmNode(120, true);
        this.memoTo_memo = new NXLogic_MemoryNode(false);

        this.memoLdgMemo_conf01 = new NXLogic_ConfirmNode(1, true);
        this.memoLdgMemo_memory1 = new NXLogic_MemoryNode(false);
        this.memoLdgMemo_conf02 = new NXLogic_ConfirmNode(10, true);
        this.memoLdgMemo_below2000ft = new NXLogic_MemoryNode(true);

        // master warning & caution buttons
        this.warningPressed = false;
        this.cautionPressed = false;

        // altitude warning
        this.previousTargetAltitude = NaN;
        this._wasBellowThreshold = false;
        this._wasAboveThreshold = false;
        this._wasInRange = false;
        this._wasReach200ft = false;
    }

    update(_deltaTime, _core) {
        this._resetPulses();

        this.flightPhase = SimVar.GetSimVarValue("L:A32NX_FWC_FLIGHT_PHASE", "Enum");

        this._updateButtons(_deltaTime);
        this._updateTakeoffMemo(_deltaTime);
        this._updateLandingMemo(_deltaTime);
        this._updateAltitudeWarning();
    }

    _resetPulses() {
        this.flightPhaseEndedPulse = false;
    }

    _updateButtons(_deltaTime) {
        if (SimVar.GetSimVarValue("L:A32NX_BTN_TOCONFIG", "Bool")) {
            SimVar.SetSimVarValue("L:A32NX_BTN_TOCONFIG", "Bool", 0);
            this.toConfigTest = true;
            SimVar.SetSimVarValue("L:A32NX_FWC_TOCONFIG", "Bool", 1);
        } else if (this.toConfigTest) {
            this.toConfigTest = false;
        }

        let recall = false;
        if (SimVar.GetSimVarValue("L:A32NX_BTN_RCL", "Bool")) {
            SimVar.SetSimVarValue("L:A32NX_BTN_RCL", "Bool", 0);
            SimVar.SetSimVarValue("L:A32NX_FWC_RECALL", "Bool", 1);
            recall = true;
        }

        const inhibOverride = this.memoFlightPhaseInhibOvrd_memo.write(recall, this.flightPhaseEndedPulse);
        SimVar.SetSimVarValue("L:A32NX_FWC_INHIBOVRD", "Bool", inhibOverride);

        if (SimVar.GetSimVarValue("L:PUSH_AUTOPILOT_MASTERAWARN_L", "Bool") || SimVar.GetSimVarValue("L:PUSH_AUTOPILOT_MASTERAWARN_R", "Bool")) {
            this.warningPressed = true;
        } else {
            this.warningPressed = false;
        }
        if (SimVar.GetSimVarValue("L:PUSH_AUTOPILOT_MASTERCAUT_L", "Bool") || SimVar.GetSimVarValue("L:PUSH_AUTOPILOT_MASTERCAUT_R", "Bool")) {
            this.cautionPressed = true;
        } else {
            this.cautionPressed = false;
        }
    }

    _updateTakeoffMemo(_deltaTime) {
        const setFlightPhaseMemo = this.flightPhase === 2 && this.toConfigTest;
        const resetFlightPhaseMemo = (
            this.flightPhase === 10 ||
            this.flightPhase === 3 ||
            this.flightPhase === 1 ||
            this.flightPhase === 6
        );
        const flightPhaseMemo = this.memoTo_memo.write(setFlightPhaseMemo, resetFlightPhaseMemo);

        const eng1NotRunning = SimVar.GetSimVarValue("ENG N1 RPM:1", "Percent") < 15;
        const eng2NotRunning = SimVar.GetSimVarValue("ENG N1 RPM:2", "Percent") < 15;
        const toTimerElapsed = this.memoTo_conf01.write(!eng1NotRunning && !eng2NotRunning, _deltaTime);

        this.toMemo = flightPhaseMemo || (this.flightPhase === 2 && toTimerElapsed);
        SimVar.SetSimVarValue("L:A32NX_FWC_TOMEMO", "Bool", this.toMemo);
    }

    _updateLandingMemo(_deltaTime) {
        const radioHeight = SimVar.GetSimVarValue("RADIO HEIGHT", "Feet");
        const radioHeightInvalid = false;
        const gearDownlocked = SimVar.GetSimVarValue("GEAR TOTAL PCT EXTENDED", "percent") > 0.95;

        const setBelow2000ft = radioHeight < 2000;
        const resetBelow2000ft = radioHeight > 2200;
        const memo2 = this.memoLdgMemo_below2000ft.write(setBelow2000ft, resetBelow2000ft);

        const setLandingMemo = this.memoLdgMemo_conf01.write(!radioHeightInvalid && resetBelow2000ft, _deltaTime);
        const resetLandingMemo = !(this.flightPhase === 7 || this.flightPhase === 8 || this.flightPhase === 6);
        const memo1 = this.memoLdgMemo_memory1.write(setLandingMemo, resetLandingMemo);

        const showInApproach = memo1 && memo2 && this.flightPhase === 6;

        const invalidRadioMemo = this.memoLdgMemo_conf02.write(radioHeightInvalid && gearDownlocked);

        this.ldgMemo = showInApproach || invalidRadioMemo || this.flightPhase === 8 || this.flightPhase === 7;
        SimVar.SetSimVarValue("L:A32NX_FWC_LDGMEMO", "Bool", this.ldgMemo);
    }

    _updateAltitudeWarning() {
        const indicatedAltitude = Simplane.getAltitude();
        const shortAlert = SimVar.GetSimVarValue("L:A32NX_ALT_DEVIATION_SHORT", "Bool");
        if (shortAlert === 1) {
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION_SHORT", "Bool", false);
        }

        if (this.warningPressed === true) {
            this._wasBellowThreshold = false;
            this._wasAboveThreshold = false;
            this._wasInRange = false;
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
            return;
        }

        if (Simplane.getIsGrounded()) {
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
        }

        // Use FCU displayed value
        const currentAltitudeConstraint = SimVar.GetSimVarValue("L:A32NX_FG_ALTITUDE_CONSTRAINT", "feet");
        const currentFCUAltitude = SimVar.GetSimVarValue("AUTOPILOT ALTITUDE LOCK VAR:3", "feet");
        const targetAltitude = currentAltitudeConstraint && !this.hasAltitudeConstraint() ? currentAltitudeConstraint : currentFCUAltitude;

        // Exit when selected altitude is being changed
        if (this.previousTargetAltitude !== targetAltitude) {
            this.previousTargetAltitude = targetAltitude;
            this._wasBellowThreshold = false;
            this._wasAboveThreshold = false;
            this._wasInRange = false;
            this._wasReach200ft = false;
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION_SHORT", "Bool", false);
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
            return;
        }

        // Exit when:
        // - Landing gear down & slats extended
        // - Glide slope captured
        // - Landing locked down

        const landingGearIsDown = SimVar.GetSimVarValue("L:A32NX_FLAPS_HANDLE_INDEX", "Enum") >= 1 && SimVar.GetSimVarValue("GEAR HANDLE POSITION", "Boolean");
        const verticalMode = SimVar.GetSimVarValue("L:A32NX_FMA_VERTICAL_MODE", "Number");
        const glideSlopeCaptured = verticalMode >= 30 && verticalMode <= 34;
        const landingGearIsLockedDown = SimVar.GetSimVarValue("GEAR POSITION:0", "Enum") > 0.9;
        const isTcasResolutionAdvisoryActive = SimVar.GetSimVarValue("L:A32NX_TCAS_STATE", "Enum") > 1;
        if (landingGearIsDown || glideSlopeCaptured || landingGearIsLockedDown || isTcasResolutionAdvisoryActive) {
            this._wasBellowThreshold = false;
            this._wasAboveThreshold = false;
            this._wasInRange = false;
            this._wasReach200ft = false;
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION_SHORT", "Bool", false);
            SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
            return;
        }

        const delta = Math.abs(indicatedAltitude - targetAltitude);

        if (delta < 200) {
            this._wasBellowThreshold = true;
            this._wasAboveThreshold = false;
            this._wasReach200ft = true;
        }
        if (750 < delta) {
            this._wasAboveThreshold = true;
            this._wasBellowThreshold = false;
        }
        if (200 <= delta && delta <= 750) {
            this._wasInRange = true;
        }

        if (this._wasBellowThreshold && this._wasReach200ft) {
            if (delta >= 200) {
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", true);
            } else if (delta < 200) {
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
            }
        } else if (this._wasAboveThreshold && delta <= 750 && !this._wasReach200ft) {
            if (!SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_1_ACTIVE", "Bool") && !SimVar.GetSimVarValue("L:A32NX_AUTOPILOT_2_ACTIVE", "Bool")) {
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION_SHORT", "Bool", true);
            }
        } else if (750 < delta && this._wasInRange && !this._wasReach200ft) {
            if (750 < delta) {
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", true);
            } else if (delta >= 750) {
                SimVar.SetSimVarValue("L:A32NX_ALT_DEVIATION", "Bool", false);
            }
        }
    }

    hasAltitudeConstraint() {
        if (this.aircraft == Aircraft.A320_NEO) {
            if (Simplane.getAutoPilotAltitudeManaged() && SimVar.GetSimVarValue("L:AP_CURRENT_TARGET_ALTITUDE_IS_CONSTRAINT", "number") != 0) {
                return false;
            }
        }
        return true;
    }
}

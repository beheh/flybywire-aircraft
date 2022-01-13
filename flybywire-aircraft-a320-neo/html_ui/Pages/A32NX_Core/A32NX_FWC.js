class A32NX_FWC {
    constructor() {
        // momentary
        this.toConfigTest = null;
        this.flightPhaseEndedPulse = false;

        this.memoFlightPhaseInhibOvrd_memo = new NXLogic_MemoryNode(false);
    }

    update(_deltaTime, _core) {
        this._resetPulses();

        this._updateButtons(_deltaTime);
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
    }
}

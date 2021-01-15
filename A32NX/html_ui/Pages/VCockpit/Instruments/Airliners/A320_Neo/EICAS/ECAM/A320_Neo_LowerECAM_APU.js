/** @type A320_Neo_LowerECAM_APU */
var A320_Neo_LowerECAM_APU;
(function (A320_Neo_LowerECAM_APU) {
    class Page extends Airliners.EICASTemplateElement {
        constructor() {
            super();
            this.isInitialised = false;
        }
        get templateID() {
            return "LowerECAMAPUTemplate";
        }
        connectedCallback() {
            super.connectedCallback();
            TemplateElement.call(this, this.init.bind(this));
        }
        init() {
            // Last state tracking inits to -1 since we don't know what the state is.
            // The first update sets it correctly for us.
            this.lastAPUBleedState = -1;

            this.APUGenInfo = this.querySelector("#APUGenInfo_On");
            this.APUGenAvailArrow = this.querySelector("#APUGenAvailArrow");
            this.APUGenLoad = this.querySelector("#APUGenLoad");
            this.APUVolts = this.querySelector("#APUGenVoltage");
            this.APUFrequency = this.querySelector("#APUGenFrequency");
            this.APUGenTitle = this.querySelector("#APUGenParams");

            this.APUAvail = this.querySelector("#APUAvail_On");

            this.APUFlapOpen = this.querySelector("#APUFlapOpen_On");

            this.APUBleedOn = this.querySelector("#APUBleed_On");
            this.APUBleedOff = this.querySelector("#APUBleed_Off");
            this.APUBleedPressure = this.querySelector("#APUBleedAirPressure");

            this.apuInfo = new APUInfo(this.querySelector("#APUGauges"));

            this.previousState = {
                available: undefined,
                adirsAligned: undefined,
                apuGenActive: undefined,
                externalPowerOn: undefined
            };

            this.isInitialised = true;
        }
        update(_deltaTime) {
            if (!this.isInitialised || !A320_Neo_EICAS.isOnBottomScreen()) {
                return;
            }

            // *******************************************************************************************************
            // APU Logic that isn't tied to the APU ECAM SCREEN belongs in A32NX/html_ui/Pages/A32NX_Core/A32NX_APU.js
            // *******************************************************************************************************

            // Bleed
            const currentAPUBleedState = SimVar.GetSimVarValue("BLEED AIR APU","Bool");

            if (currentAPUBleedState !== this.lastAPUBleedState) {
                this.lastAPUBleedState = currentAPUBleedState;

                if (currentAPUBleedState === 1) {
                    this.APUBleedOn.setAttribute("visibility", "visible");
                    this.APUBleedOff.setAttribute("visibility", "hidden");
                } else {
                    this.APUBleedOn.setAttribute("visibility", "hidden");
                    this.APUBleedOff.setAttribute("visibility", "visible");
                }
            }

            const showApuData = shouldShowApuData();
            let allParametersWithinAcceptableRange = false;
            if (showApuData) {
                const load = Math.round(SimVar.GetSimVarValue("L:APU_LOAD_PERCENT","percent"));
                this.APUGenLoad.textContent = load;
                const loadWithinAcceptableRange = load <= 100;
                this.APUGenLoad.classList.toggle("APUGenParamValue", loadWithinAcceptableRange);
                this.APUGenLoad.classList.toggle("APUGenParamValueWarn", !loadWithinAcceptableRange);

                const volts = SimVar.GetSimVarValue("L:A32NX_APU_GEN_VOLTAGE","Volts");
                this.APUVolts.textContent = volts;
                const voltsWithinAcceptableRange = 110 <= volts && volts <= 120;
                this.APUVolts.classList.toggle("APUGenParamValue", voltsWithinAcceptableRange);
                this.APUVolts.classList.toggle("APUGenParamValueWarn", !voltsWithinAcceptableRange);

                const hertz = SimVar.GetSimVarValue("L:A32NX_APU_GEN_FREQ","Hertz");
                this.APUFrequency.textContent = Math.round(hertz);
                const hertzWithinAcceptableRange = 390 <= hertz && hertz <= 410;
                this.APUFrequency.classList.toggle("APUGenParamValue", hertzWithinAcceptableRange);
                this.APUFrequency.classList.toggle("APUGenParamValueWarn", !hertzWithinAcceptableRange);

                allParametersWithinAcceptableRange = loadWithinAcceptableRange && voltsWithinAcceptableRange && hertzWithinAcceptableRange;
            }

            this.APUGenTitle.classList.toggle("APUGenTitle", showApuData && allParametersWithinAcceptableRange);
            this.APUGenTitle.classList.toggle("APUGenTitleWarn", showApuData && !allParametersWithinAcceptableRange);
            this.APUGenTitle.classList.toggle("APUGenTitleInactive", !showApuData);

            this.APUGenInfo.setAttribute("visibility", showApuData ? "visible" : "hidden");

            const adirsAligned = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Number") === 2;
            const apuGenActive = SimVar.GetSimVarValue("APU GENERATOR ACTIVE", "Bool") === 1;
            const externalPowerOn = SimVar.GetSimVarValue("EXTERNAL POWER ON", "Bool") === 0;
            const available = SimVar.GetSimVarValue("L:A32NX_APU_AVAILABLE", "Bool");

            // AVAIL indication & bleed pressure
            const doUpdateAvailAndBleed =
                available !== this.previousState.available
                || adirsAligned !== this.previousState.adirsAligned
                || apuGenActive !== this.previousState.apuGenActive
                || externalPowerOn !== this.previousState.externalPowerOn;

            if (doUpdateAvailAndBleed) {
                this.previousState.available = available;
                this.previousState.adirsAligned = adirsAligned;
                this.previousState.apuGenActive = apuGenActive;
                this.previousState.externalPowerOn = apuGenActive;

                if (available) {
                    this.APUAvail.setAttribute("visibility", "visible");

                    if (apuGenActive && externalPowerOn) {
                        this.APUGenAvailArrow.setAttribute("visibility", "visible");
                    } else {
                        this.APUGenAvailArrow.setAttribute("visibility", "hidden");
                    }

                    this.APUBleedPressure.textContent = SimVar.GetSimVarValue("L:APU_BLEED_PRESSURE","PSI");
                    this.APUBleedPressure.setAttribute("class", "APUGenParamValue");
                }

                if (!available || !adirsAligned) {
                    this.APUAvail.setAttribute("visibility", "hidden");
                    this.APUGenAvailArrow.setAttribute("visibility", "hidden");
                    this.APUBleedPressure.textContent = "XX";
                    this.APUBleedPressure.setAttribute("class", "APUGenParamValueWarn");
                }
            }

            const apuFlapOpenPercent = SimVar.GetSimVarValue("L:APU_FLAP_OPEN", "Percent");
            this.APUFlapOpen.setAttribute("visibility", apuFlapOpenPercent === 100 ? "visible" : "hidden");

            this.apuInfo.update(_deltaTime);
        }
    }
    A320_Neo_LowerECAM_APU.Page = Page;

    class APUInfo {
        constructor(_gaugeDiv) {
            //APU N Gauge
            const gaugeDef1 = new A320_Neo_ECAM_Common.GaugeDefinition();
            gaugeDef1.arcSize = 180;
            gaugeDef1.startAngle = -210;
            gaugeDef1.currentValuePrecision = 0;
            gaugeDef1.minValue = 0;
            gaugeDef1.maxValue = 110;
            gaugeDef1.minRedValue = 101;
            gaugeDef1.maxRedValue = 110;
            gaugeDef1.dangerRange[0] = 101;
            gaugeDef1.dangerRange[1] = 110;
            gaugeDef1.currentValuePos.x = 0.8;
            gaugeDef1.currentValuePos.y = 0.74;
            gaugeDef1.currentValueFunction = getN.bind(this);
            this.apuNGauge = window.document.createElement("a320-neo-ecam-gauge");
            this.apuNGauge.id = "APU_N_Gauge";
            this.apuNGauge.init(gaugeDef1);
            this.apuNGauge.addGraduation(0, true, "0");
            this.apuNGauge.addGraduation(50, true);
            this.apuNGauge.addGraduation(100, true, "10");
            this.apuNGauge.active = false;
            if (_gaugeDiv != null) {
                _gaugeDiv.appendChild(this.apuNGauge);
            }

            //APU EGT Gauge
            const gaugeDef2 = new A320_Neo_ECAM_Common.GaugeDefinition();
            gaugeDef2.arcSize = 200;
            gaugeDef2.startAngle = -210;
            gaugeDef2.currentValuePrecision = 0;
            gaugeDef2.minValue = 300;
            gaugeDef2.maxValue = 1100;
            const warningEgt = this.getWarningEgt();
            gaugeDef2.minRedValue = warningEgt;
            gaugeDef2.maxRedValue = 1100;
            gaugeDef2.warningRange[0] = this.getCautionEgt();
            gaugeDef2.warningRange[1] = warningEgt;
            gaugeDef2.dangerRange[0] = warningEgt;
            gaugeDef2.dangerRange[1] = 1100;
            gaugeDef2.currentValuePos.x = 0.8;
            gaugeDef2.currentValuePos.y = 0.74;
            gaugeDef2.currentValueFunction = this.getEgt.bind(this);
            gaugeDef2.roundDisplayValueToNearest = 5;
            gaugeDef2.outerDynamicMarkerFunction = this.getCautionEgtForDynamicMarker.bind(this, "EGTCaution");
            this.apuEGTGauge = window.document.createElement("a320-neo-ecam-gauge");
            this.apuEGTGauge.id = "APU_EGT_Gauge";
            this.apuEGTGauge.init(gaugeDef2);
            this.apuEGTGauge.addGraduation(300, true, "3");
            this.apuEGTGauge.addGraduation(700, true, "7");
            this.apuEGTGauge.addGraduation(1000, true, "10");
            this.apuEGTGauge.addGraduation(1100, false, "", true, true, "EGTCaution");
            this.apuEGTGauge.active = false;
            if (_gaugeDiv != null) {
                _gaugeDiv.appendChild(this.apuEGTGauge);
            }

            // Last state tracking inits to -1 since we don't know what the state is.
            // The first update sets it correctly for us.
            this.apuShuttingDown = false;
            this.apuInactiveTimer = -1;
        }

        update(_deltaTime) {
            const showApuData = shouldShowApuData();
            this.apuEGTGauge.active = showApuData;
            this.apuNGauge.active = showApuData;

            this.setCautionAndWarningRanges();

            this.apuNGauge.update(_deltaTime);
            this.apuEGTGauge.update(_deltaTime);
        }

        setCautionAndWarningRanges() {
            const warningEgt = this.getWarningEgt();
            this.apuEGTGauge.minRedValue = warningEgt;
            this.apuEGTGauge.dangerRange[0] = warningEgt;
            this.apuEGTGauge.warningRange[0] = this.getCautionEgt();
            this.apuEGTGauge.warningRange[1] = warningEgt;
        }

        //function accepts ID of the marker and returns an array with ID and EGT
        getCautionEgtForDynamicMarker(_id) {
            return [_id, this.getCautionEgt()];
        }

        getCautionEgt() {
            return SimVar.GetSimVarValue("L:A32NX_APU_EGT_CAUTION", "celsius");
        }

        getWarningEgt() {
            return SimVar.GetSimVarValue("L:A32NX_APU_EGT_WARNING", "celsius");
        }

        getEgt() {
            return SimVar.GetSimVarValue("L:A32NX_APU_EGT", "celsius");
        }
    }

    function shouldShowApuData() {
        const apuMasterSwitch = SimVar.GetSimVarValue("FUELSYSTEM VALVE SWITCH:8", "Bool");
        return apuMasterSwitch || getN() > 0;
    }

    function getN() {
        return SimVar.GetSimVarValue("L:A32NX_APU_N", "percent");
    }
})(A320_Neo_LowerECAM_APU || (A320_Neo_LowerECAM_APU = {}));

customElements.define("a320-neo-lower-ecam-apu", A320_Neo_LowerECAM_APU.Page);

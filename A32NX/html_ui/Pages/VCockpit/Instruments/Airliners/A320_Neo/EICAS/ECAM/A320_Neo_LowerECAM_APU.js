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

            this.updateThrottler = new UpdateThrottler(75);

            this.isInitialised = true;
        }
        update(_deltaTime) {
            if (!this.isInitialised || !A320_Neo_EICAS.isOnBottomScreen()) {
                return;
            }

            _deltaTime = this.updateThrottler.canUpdate(_deltaTime);
            if (_deltaTime === -1) {
                return;
            }

            // *******************************************************************************************************
            // APU Logic that isn't tied to the APU ECAM SCREEN belongs in A32NX/html_ui/Pages/A32NX_Core/A32NX_APU.js
            // *******************************************************************************************************

            // Bleed
            const apuBleedAirValveOpen = SimVar.GetSimVarValue("L:A32NX_APU_BLEED_AIR_VALVE_OPEN", "Bool");
            toggleVisibility(this.APUBleedOn, apuBleedAirValveOpen);
            toggleVisibility(this.APUBleedOff, !apuBleedAirValveOpen);

            const showApuData = shouldShowApuData();
            let allParametersWithinAcceptableRange = false;
            if (showApuData) {
                const load = Math.round(SimVar.GetSimVarValue("L:APU_LOAD_PERCENT","percent"));
                this.APUGenLoad.textContent = load;
                const loadWithinNormalRange = load <= 100;
                this.APUGenLoad.classList.toggle("APUGenParamValue", loadWithinNormalRange);
                this.APUGenLoad.classList.toggle("APUGenParamValueWarn", !loadWithinNormalRange);

                const volts = SimVar.GetSimVarValue("L:A32NX_APU_GEN_VOLTAGE","Volts");
                this.APUVolts.textContent = volts;
                const voltsWithinNormalRange = SimVar.GetSimVarValue("L:A32NX_APU_GEN_VOLTAGE_NORMAL", "Bool");
                this.APUVolts.classList.toggle("APUGenParamValue", voltsWithinNormalRange);
                this.APUVolts.classList.toggle("APUGenParamValueWarn", !voltsWithinNormalRange);

                const hertz = SimVar.GetSimVarValue("L:A32NX_APU_GEN_FREQ","Hertz");
                this.APUFrequency.textContent = Math.round(hertz);
                const hertzWithinNormalRange = SimVar.GetSimVarValue("L:A32NX_APU_GEN_FREQ_NORMAL", "Bool");
                this.APUFrequency.classList.toggle("APUGenParamValue", hertzWithinNormalRange);
                this.APUFrequency.classList.toggle("APUGenParamValueWarn", !hertzWithinNormalRange);

                allParametersWithinAcceptableRange = loadWithinNormalRange && voltsWithinNormalRange && hertzWithinNormalRange;
            }

            this.APUGenTitle.classList.toggle("APUGenTitle", showApuData && allParametersWithinAcceptableRange);
            this.APUGenTitle.classList.toggle("APUGenTitleWarn", showApuData && !allParametersWithinAcceptableRange);
            this.APUGenTitle.classList.toggle("APUGenTitleInactive", !showApuData);

            toggleVisibility(this.APUGenInfo, showApuData);

            const adirsAligned = SimVar.GetSimVarValue("L:A320_Neo_ADIRS_STATE", "Number") === 2;
            const apuGenOnline = SimVar.GetSimVarValue("L:APU_GEN_ONLINE", "Bool") === 1;
            const externalPowerOff = SimVar.GetSimVarValue("EXTERNAL POWER ON", "Bool") === 0;
            const available = SimVar.GetSimVarValue("L:A32NX_APU_AVAILABLE", "Bool");

            toggleVisibility(this.APUAvail, available);
            toggleVisibility(this.APUGenAvailArrow, available && apuGenOnline && externalPowerOff);

            if (available && adirsAligned) {
                this.APUBleedPressure.textContent = SimVar.GetSimVarValue("L:APU_BLEED_PRESSURE","PSI");
                this.APUBleedPressure.setAttribute("class", "APUGenParamValue");
            } else {
                this.APUBleedPressure.textContent = "XX";
                this.APUBleedPressure.setAttribute("class", "APUGenParamValueWarn");
            }

            const apuFlapOpenPercent = SimVar.GetSimVarValue("L:APU_FLAP_OPEN", "Percent");
            toggleVisibility(this.APUFlapOpen, apuFlapOpenPercent === 100);

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

    function toggleVisibility(element, condition) {
        element.setAttribute("visibility", condition ? "visible" : "hidden");
    }
})(A320_Neo_LowerECAM_APU || (A320_Neo_LowerECAM_APU = {}));

customElements.define("a320-neo-lower-ecam-apu", A320_Neo_LowerECAM_APU.Page);

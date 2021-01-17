import ReactDOM from 'react-dom';
import { useState } from 'react';
import {
    renderTarget,
    useInteractionEvent,
    useUpdate,
    getSimVar,
} from '../util.mjs';
import './style.scss';
import Ecam from './Ecam/Ecam.jsx';

// TODO: Move anything dependent on ac power change to A32NX_Core
function powerAvailable() {
    // These are inlined so they're only evaluated if prior conditions return false.
    return (
        Simplane.getEngineActive(0) === 1 || Simplane.getEngineActive(1) === 1
    ) || (
        getSimVar('L:APU_GEN_ONLINE')
    ) || (
        getSimVar('EXTERNAL POWER AVAILABLE:1') && getSimVar('EXTERNAL POWER ON')
    );
}

function SelfTest() {
    return (
        <svg className="text-wrapper">
            <text x="246" y="170">SELF TEST IN PROGRESS</text>
            <text x="246" y="210">(MAX 10 SECONDS)</text>
        </svg>
    );
}

function Idle() {
    const [inop, setInop] = useState(false);

    useInteractionEvent('A32NX_DCDU_BTN_INOP', () => {
        if (!inop) {
            setInop(true);
            setTimeout(() => {
                setInop(false);
            }, 3000);
        }
    });

    return (
        <svg className="ewd-svg" viewBox="0 0 600 600">
            <Ecam />
        </svg>
    );
}

function EWD() {
    const [status, setStatus] = useState(() => {
        if(getSimVar('L:A32NX_COLD_AND_DARK_SPAWN')) {
            return "OFF";
        }
        else {
            return "IDLE";
        }
    });

    /*useEffect(() => {
        setTimeout(() => {
            if (powerAvailable()) {
                setStatus('IDLE');
            }
        }, 8000);
    }, [status]);*/

    useUpdate((_deltaTime) => {
        if (status === 'OFF') {
            if (powerAvailable()) {
                setStatus('ON');
            }
        } else if (!powerAvailable()) {
            setStatus('OFF');
        }
    });

    if (status == "ON") {
        return <SelfTest/>;
    }

    if (status === "IDLE") {
        return <Idle />;
    }

    return null;
}

ReactDOM.render(<EWD />, renderTarget);

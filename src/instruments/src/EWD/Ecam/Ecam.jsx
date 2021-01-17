import './Ecam.scss';

const LINE_SPACING = 23.5;
const LETTER_WIDTH = 13.19;

const Ecam = () => {

    /*const message = [
        "",
        "\x1b<4m\x1b4mFWS\x1bm FWC 1+2 FAULT",
        "\x1b<5m -MONITOR SYS",
        "\x1b<5m -MONITOR OVERHEAD PANEL",
    ].join("\r");*/

    const message = [
    "\x1b<2m\x1b4mELEC\x1bm \x1b'mEMER CONFIG\x1bm",
    "\x1b<5m PROC:GRVTY FUEL FEEDING",
    "\x1b<5m -FAC 1......OFF THEN ON",
    "",
    "",
    "\x1b<5m -BUS TIE............OFF",
    "\x1b<5m -GEN 1+2... OFF THEN ON",
    ].join("\r");

    /*const message = [
        "023456789012345678901234567",
        " -2",
        " -3",
        " -4",
        " -5",
        " -6",
        " -7",
        " -8",
    ].join("\r");*/

    const lines = [];
    let spans = [];

    let color = "white";
    let underlined = false;
    let flashing = false;
    let framed = false;

    const decorations = [];

    let buffer = "";
    let startCol = 0;
    let col = 0;
    for (let i = 0; i < message.length; i++) {
        const char = message[i];
        if (char === "\x1B" || char === "\r") {
            if (buffer !== "") {
                // close current part
                spans.push(
                    <tspan
                        key={buffer}
                        className={`color-${color}`}
                    >
                        {buffer}
                    </tspan>
                );
                buffer = "";

                if (underlined) {
                    decorations.push(
                        <path
                            className={`ewd-underline color-${color}`}
                            d={`M ${startCol * LETTER_WIDTH + 1.5} ${lines.length * LINE_SPACING + 4} h ${(col - startCol) * LETTER_WIDTH - 1}`}
                            strokeLinecap="round"
                        />);
                }

                if (framed) {
                    decorations.push(
                        <path
                            className={`ewd-underline color-${color}`}
                            d={`M ${startCol * LETTER_WIDTH - 3} ${lines.length * LINE_SPACING - 17} h ${(col - startCol) * LETTER_WIDTH + 7} v 21 h ${-((col - startCol) * LETTER_WIDTH + 7)} v -21`}
                            strokeLinecap="round"
                        />
                    );
                }

                startCol = col;
            }

            if (char === "\x1B") {
                let ctrlBuffer = "";
                i++;
                for (; i < message.length; i++) {
                    ctrlBuffer += message[i];

                    let match = true;
                    switch(ctrlBuffer) {
                        case "m":
                            // Reset attribute
                            underlined = false;
                            flashing = false;
                            framed = false;
                            break;
                        case "4m":
                            // Underlined attribute
                            underlined = true;
                            break;
                        case ")m":
                            // Flashing attribute
                            flashing = true;
                            break;
                        case "'m":
                            // Characters which follow must be framed
                            framed = true;
                            break;
                        case "<1m":
                            // Select YELLOW
                            color = "yellow";
                            break;
                        case "<2m":
                            // Select RED
                            color = "red";
                            break;
                        case "<3m":
                            // Select GREEN
                            color = "green";
                            break;
                        case "<4m":
                            // Select AMBER
                            color = "amber";
                            break;
                        case "<5m":
                            // Select CYAN (blue-green)
                            color = "cyan";
                            break;
                        case "<6m":
                            // Select MAGENTA
                            color = "magenta";
                            break;
                        case "<7m":
                            // Select WHITE
                            color = "white";
                            break;
                        default:
                            match = false;
                            break;
                    }

                    if (match) {
                        break;
                    }
                }

                continue;
            }

            if (char === "\r") {
                lines.push(<text y={lines.length * LINE_SPACING}>{spans}</text>);

                spans = [];
                col = 0;
                startCol = 0;
                continue;
            }
        }

        buffer += char;
        col++;
    }

    if (buffer !== "") {
        spans.push(
            <tspan
                key={buffer}
                className={`color-${color}`}
            >
                {buffer}
            </tspan>
        );
    }

    if (spans.length) {
        lines.push(<text y={lines.length * LINE_SPACING}>{spans}</text>);
    }

    const memos = [
        <>&nbsp;NOT AVAIL</>,
        "ECAM WARN",
        "ALTI ALERT",
        "STATUS",
        "A/CALL OUT",
        "MEMO",
    ];

    return (
        <g>
            <path className="ewd-ecam-line--outer" d="M 3   406 h 342" strokeLinecap="round" />
            <path className="ewd-ecam-line" d="M 3   406 h 342" strokeLinecap="round" />

            <path className="ewd-ecam-line--outer" d="M 409 406 h 188" strokeLinecap="round" />
            <path className="ewd-ecam-line" d="M 409 406 h 188" strokeLinecap="round" />

            <path className="ewd-ecam-line--outer" d="M 378.5 422 v 146" strokeLinecap="round" />
            <path className="ewd-ecam-line" d="M 378.5 422 v 146" strokeLinecap="round" />

            <g className="ewd-warning-text-left" transform="translate(7 432)" fill="white">
                {lines}
                {decorations}
            </g>

            <text
                x="378"
                y="414"
                fill="white"
                textAnchor="middle"
                style={{fontSize: "1.30em", letterSpacing: "0.07em"}}
            >
                ADV
            </text>
            <path
                className="ewd-underline color-white"
                d="M 358 418 h 40 v -20 h -40 v 20"
                strokeLinecap="round"
            />

            {/*<text x="378.5" y="587.5" fill="white" textAnchor="middle" style={{fontSize: "1.25em"}}>
                STS
            </text>
            <path
                className="ewd-underline color-white"
                d="M 361.5 590.5 h 34 v -18 h -34 v 18"
                strokeLinecap="round"
            />*/}

            <path
                d="m 376 571 h 5 v 15 h 5 l -7.5,11 l -7.5,-11 h 5 v -15"
                style={{
                    fill: "#00ff00",
                    stroke: "none",
                    //strokeWidth: 0.2,
                }}
            />
        </g>
    );
};

export default Ecam;

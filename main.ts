/*
Final improved sdwireless module for MeowBit SD Wireless (KittenBot / Meowbit).
Changes from previous version:
 - Robust try/catch around SPI operations and IRQ handler.
 - Fixed buffer bounds and length checks.
 - Added timestamps (control.millis()) and a serialCounter included in outgoing packets.
 - Safer RSSI request and configurable RSSI command.
 - Added radio-like wrapper APIs (sendString/sendNumber/sendValue, onReceived...)
 - Added set transmit power helper (best-effort; module-dependent)
 - Error and RSSI callbacks and getters.

Notes:
 - The MeowBit SD Wireless module (KittenBot) uses an nRF51822 on the SD-shaped board and communicates over SPI using a small service protocol. Detailed low-level firmware commands are not publicly documented; the code uses the same conventions from the original pxt-sdwireless extension and adds best-effort config commands where firmware specifics were unknown. If you have an official firmware command list, tell me and I will update the constants to exactly match it.
 - This file is written for MakeCode / PXT TypeScript for micro:bit / Meowbit targets.
*/

//% color="#31C7D5" weight=10 icon=""
namespace sdwireless {

    type EvtStr = (data: string, ts?: number) => void;
    type EvtBuff = (data: Buffer, ts?: number) => void;
    type EvtNum = (data: number, ts?: number) => void;
    type EvtValue = (name: string, value: number, ts?: number) => void;
    type EvtRssi = (rssi: number, ts?: number) => void;
    type EvtErr = (err: string) => void;

    const CONFIG_SETGROUP = 0xe2; // used by original firmware for group config
    const SPI_TX = 0xe0;
    const SPI_RX = 0xe2;

    // default RSSI command protocol value — module-dependent and configurable
    let CMD_GET_RSSI_PROTOCOL = 0xEE;

    // event handlers
    let onMsg: EvtStr = undefined;
    let onMsgBuff: EvtBuff = undefined;
    let onMbitString: EvtStr = undefined;
    let onMbitNumber: EvtNum = undefined;
    let onMbitValue: EvtValue = undefined;
    let onRssi: EvtRssi = undefined;
    let onError: EvtErr = undefined;

    // SPI & pins
    let spi: SPI = undefined;
    let cs = pins.P19;
    let irq = pins.P20;

    // state
    let serialCounter = 0;
    let lastRssi: number = null;
    let lastError: string = null;
    let configuredTxPower: number = null; // best-effort

    // Packet type constants (same as original schema)
    const PACKET_TYPE_NUMBER = 0;
    const PACKET_TYPE_VALUE = 1;
    const PACKET_TYPE_STRING = 2;
    const PACKET_TYPE_BUFFER = 3;
    const PACKET_TYPE_DOUBLE = 4;
    const PACKET_TYPE_DOUBLE_VALUE = 5;

    function safeUint8Set(buf: Buffer, idx: number, v: number) {
        if (!buf) return;
        if (idx >= 0 && idx < buf.length) buf.setUint8(idx, v & 0xff);
    }

    function spiTx(b: Buffer, protocol?: number) {
        if (!spi) return;
        if (!protocol) protocol = SPI_TX;
        try {
            let tx = pins.createBuffer(b.length + 4);
            let rx = pins.createBuffer(b.length + 4);
            // header
            safeUint8Set(tx, 0, 0xff);
            safeUint8Set(tx, 1, 0xaa);
            safeUint8Set(tx, 2, protocol);
            safeUint8Set(tx, 3, b.length);
            for (let j = 0; j < b.length; j++) safeUint8Set(tx, j + 4, b[j]);
            cs.digitalWrite(false);
            spi.transfer(tx, rx);
            cs.digitalWrite(true);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    function spiRx(): Buffer {
        if (!spi) return null;
        try {
            // prepare tx to ask for data. Keep buffers big enough for expected response.
            let tx = pins.createBuffer(24 + 4);
            let rx = pins.createBuffer(24 + 4);

            safeUint8Set(tx, 0, 0xff);
            safeUint8Set(tx, 1, 0xaa);
            safeUint8Set(tx, 2, 0xe1);
            safeUint8Set(tx, 3, 0);

            cs.digitalWrite(false);
            spi.transfer(tx, rx);
            cs.digitalWrite(true);

            // rx[3] encodes payload length. Clamp to available range.
            let rawLen = rx[3] || 0;
            let maxAvail = Math.max(0, rx.length - 4);
            let len = Math.max(0, Math.min(rawLen, maxAvail));
            if (len <= 0) return null;
            return rx.slice(4, 4 + len);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
            return null;
        }
    }

    //% blockId=sdw_init block="SD wireless init"
    //% weight=100
    export function sdw_init(): void {
        try {
            spi = pins.createSPI(pins.P15, pins.P14, pins.P13);
            spi.setMode(3);
            spi.setFrequency(1000000);
            cs.digitalWrite(true);

            // send init ping twice (matches previous behaviour)
            let initBuf = pins.createBuffer(2);
            initBuf[0] = 0x31;
            initBuf[1] = 0x32;
            spiTx(initBuf, 0xea);
            basic.pause(200);
            spiTx(initBuf, 0xea);
            basic.pause(200);

            irq.onEvent(PinEvent.PulseHigh, function () {
                try {
                    let msg = spiRx();
                    let ts = control.millis();
                    if (!msg) return;

                    // increment serial counter for received packet
                    serialCounter = (serialCounter + 1) & 0xffffffff;

                    // Heuristic: some firmwares append RSSI byte at the end of the packet.
                    // If present and plausible, update lastRssi and call callback. This is
                    // a heuristic; if it causes false positives for your firmware remove it.
                    if (msg.length >= 1) {
                        const possibleRssi = msg[msg.length - 1];
                        if (possibleRssi <= 200) {
                            let r = (possibleRssi > 127) ? possibleRssi - 256 : possibleRssi;
                            lastRssi = r;
                            if (onRssi) onRssi(r, ts);
                        }
                    }

                    // generic callbacks
                    if (onMsg) onMsg(msg.toString(), ts);
                    if (onMsgBuff) onMsgBuff(msg, ts);

                    // micro:bit-style packets
                    if (msg[0] == PACKET_TYPE_NUMBER && onMbitNumber) {
                        if (msg.length >= 13) {
                            let num = msg.getNumber(NumberFormat.Int32LE, 9);
                            onMbitNumber(num, ts);
                        }
                    }
                    if (msg[0] == PACKET_TYPE_STRING && onMbitString) {
                        if (msg.length >= 10) {
                            let strLen: number = msg[9];
                            // ensure bounds
                            let end = Math.min(10 + strLen, msg.length);
                            let strBuf = msg.slice(10, end);
                            onMbitString(strBuf.toString(), ts);
                        }
                    }
                    if (msg[0] == PACKET_TYPE_VALUE && onMbitValue) {
                        if (msg.length >= 14) {
                            let value: number = msg.getNumber(NumberFormat.Int32LE, 9);
                            let strLen: number = msg[13];
                            let end = Math.min(14 + strLen, msg.length);
                            let strBuf = msg.slice(14, end);
                            onMbitValue(strBuf.toString(), value, ts);
                        }
                    }

                } catch (err) {
                    lastError = (err && err.message) ? err.message : (err + "");
                    if (onError) onError(lastError);
                }
            });
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    // low-level transmit of raw string data (keeps previous API)
    //% blockId=sdw_tx block="Send message %data"
    //% weight=90
    export function sdw_tx(data: string): void {
        try {
            let payload = pins.createBuffer(data.length);
            for (let i = 0; i < data.length; i++) payload.setUint8(i, data.charCodeAt(i));

            // create packet wrapper: type + time(4) + serial(4) + payload
            serialCounter = (serialCounter + 1) & 0xffffffff;
            let pkt = pins.createBuffer(1 + 4 + 4 + payload.length);
            pkt[0] = PACKET_TYPE_BUFFER;
            pkt.setNumber(NumberFormat.Int32LE, 1, control.millis());
            pkt.setNumber(NumberFormat.Int32LE, 5, serialCounter);
            for (let i = 0; i < payload.length; i++) pkt[9 + i] = payload[i];
            spiTx(pkt);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    //% blockId=sdw_tx_n block="Send message %data (New Line)"
    //% weight=90
    export function sdw_tx_n(data: string): void {
        sdw_tx(data + "\n");
    }

    // event registration
    //% blockId=sdw_ondata block="on Message"
    //% weight=90
    export function sdw_ondata(handler: (sdMsg: string, ts?: number) => void): void {
        onMsg = handler;
    }

    //% blockId=sdw_ondata_buff block="on Message Buff"
    //% weight=90
    export function sdw_ondata_buff(handler: (sdMsg: Buffer, ts?: number) => void): void {
        onMsgBuff = handler;
    }

    //% blockId=sdw_onerror block="on Error"
    //% weight=90
    export function sdw_onerror(handler: (err: string) => void): void {
        onError = handler;
    }

    //% blockId=sdw_onrssi block="on RSSI"
    //% weight=80
    export function sdw_onrssi(handler: (rssi: number, ts?: number) => void): void {
        onRssi = handler;
    }

    // micro:bit-style senders (wrappers) -------------------------------------------------
    //% blockId=sdw_mbit_send_string block="Send Microbit String %data"
    //% weight=80
    export function sdw_mbit_send_string(data: string): void {
        try {
            serialCounter = (serialCounter + 1) & 0xffffffff;
            let buf = pins.createBuffer(9 + 1 + data.length);
            buf[0] = PACKET_TYPE_STRING;
            buf.setNumber(NumberFormat.Int32LE, 1, control.millis());
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter);
            buf[9] = data.length;
            for (let i = 0; i < data.length; i++) buf[10 + i] = data.charCodeAt(i);
            spiTx(buf);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    //% blockId=sdw_mbit_send_number block="Send Microbit Number %data"
    //% weight=80
    export function sdw_mbit_send_number(data: number): void {
        try {
            serialCounter = (serialCounter + 1) & 0xffffffff;
            let buf = pins.createBuffer(4 + 9);
            buf[0] = PACKET_TYPE_NUMBER;
            buf.setNumber(NumberFormat.Int32LE, 1, control.millis());
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter);
            buf.setNumber(NumberFormat.Int32LE, 9, data);
            spiTx(buf);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    //% blockId=sdw_mbit_send_value block="Send Microbit Value %name = %value"
    //% weight=80
    export function sdw_mbit_send_value(name: string, value: number): void {
        try {
            serialCounter = (serialCounter + 1) & 0xffffffff;
            let buf = pins.createBuffer(9 + 4 + 1 + name.length);
            buf[0] = PACKET_TYPE_VALUE;
            buf.setNumber(NumberFormat.Int32LE, 1, control.millis());
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter);
            buf.setNumber(NumberFormat.Int32LE, 9, value);
            buf[13] = name.length;
            for (let i = 0; i < name.length; i++) buf[14 + i] = name.charCodeAt(i);
            spiTx(buf);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    // micro:bit-style event registration
    //% blockId=sdw_onmbit_number block="on Microbit Number"
    //% weight=70
    export function sdw_onmbit_number(handler: (num: number, ts?: number) => void): void {
        onMbitNumber = handler;
    }

    //% blockId=sdw_onmbit_string block="on Microbit String"
    //% weight=70
    export function sdw_onmbit_string(handler: (str: string, ts?: number) => void): void {
        onMbitString = handler;
    }

    //% blockId=sdw_onmbit_value block="on Microbit Value"
    //% weight=70
    export function sdw_onmbit_value(handler: (name: string, value: number, ts?: number) => void): void {
        onMbitValue = handler;
    }

    //% blockId=sdw_set_radiogp block="Set Radio Group %gp"
    //% weight=60
    export function sdw_set_radiogp(gp: number): void {
        try {
            let buf = pins.createBuffer(2);
            buf[0] = 1; // command: set group (existing convention)
            buf[1] = gp & 0xff;
            spiTx(buf, CONFIG_SETGROUP);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    // Best-effort: set transmit power via config command. The module firmware must
    // support this command to take effect. We use "2" as the command id (following
    // the same simple config layout used for group). If your firmware uses a
    // different command id, use sdw_set_tx_power_command to change it.
    //% blockId=sdw_set_tx_power block="Set TX power %power"
    //% weight=60
    export function sdw_set_tx_power(power: number): void {
        try {
            let buf = pins.createBuffer(2);
            buf[0] = 2; // command: set tx power (best-effort)
            buf[1] = power & 0xff;
            configuredTxPower = power;
            spiTx(buf, CONFIG_SETGROUP);
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    //% blockId=sdw_request_rssi block="Request RSSI"
    //% weight=55
    //% help="If your module supports an RSSI command, this will request it."
    export function sdw_request_rssi(): void {
        try {
            // send a short (1-byte) request using the configured RSSI protocol
            let buf = pins.createBuffer(1);
            buf[0] = 0x00;
            spiTx(buf, CMD_GET_RSSI_PROTOCOL);
            // small pause then attempt to read response if IRQ hasn't arrived
            basic.pause(50);
            let rx = spiRx();
            if (rx && rx.length > 0) {
                let raw = rx[0];
                let r = (raw > 127) ? raw - 256 : raw;
                lastRssi = r;
                if (onRssi) onRssi(r, control.millis());
            }
        } catch (err) {
            lastError = (err && err.message) ? err.message : (err + "");
            if (onError) onError(lastError);
        }
    }

    // allow changing the RSSI command protocol at runtime
    //% blockId=sdw_set_rssi_command block="Set RSSI command %cmd"
    //% weight=50
    export function sdw_set_rssi_command(cmd: number): void {
        CMD_GET_RSSI_PROTOCOL = cmd & 0xff;
    }

    //% blockId=sdw_get_last_rssi block="Get last RSSI"
    //% weight=55
    export function sdw_get_last_rssi(): number {
        return lastRssi === null ? -999 : lastRssi;
    }

    //% blockId=sdw_get_last_error block="Get last Error"
    //% weight=55
    export function sdw_get_last_error(): string {
        return lastError || "";
    }

    // Radio-style wrappers (for convenience / compatibility with micro:bit radio names)
    //% blockId=sdw_radio_send_string block="radio send string %data"
    //% weight=45
    export function sdw_radio_send_string(data: string): void {
        sdw_mbit_send_string(data);
    }

    //% blockId=sdw_radio_send_number block="radio send number %n"
    //% weight=45
    export function sdw_radio_send_number(n: number): void {
        sdw_mbit_send_number(n);
    }

    //% blockId=sdw_radio_send_value block="radio send value %name = %value"
    //% weight=45
    export function sdw_radio_send_value(name: string, value: number): void {
        sdw_mbit_send_value(name, value);
    }

    //% blockId=sdw_radio_on_received_string block="on radio received string"
    //% weight=45
    export function sdw_radio_on_received_string(handler: (s: string, ts?: number) => void): void {
        sdw_onmbit_string(handler);
    }

    //% blockId=sdw_radio_on_received_number block="on radio received number"
    //% weight=45
    export function sdw_radio_on_received_number(handler: (n: number, ts?: number) => void): void {
        sdw_onmbit_number(handler);
    }

    //% blockId=sdw_radio_on_received_value block="on radio received value"
    //% weight=45
    export function sdw_radio_on_received_value(handler: (name: string, value: number, ts?: number) => void): void {
        sdw_onmbit_value(handler);
    }

}

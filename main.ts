/*
Modified sdwireless with: try/catch error handling, timestamps, serial number,
RSSI request API (module-dependent), and error/rssi callbacks.

Note: RSSI request/response command is module-specific. See CMD_GET_RSSI_PROTOCOL
and adjust to match your module firmware if the default doesn't work.
*/

//% color="#31C7D5" weight=10 icon="\uf012"
namespace sdwireless {

    type EvtStr = (data: string, ts?: number) => void;
    type EvtBuff = (data: Buffer, ts?: number) => void;
    type EvtNum = (data: number, ts?: number) => void;
    type EvtValue = (name: string, value: number, ts?: number) => void;
    type EvtRssi = (rssi: number, ts?: number) => void;
    type EvtErr = (err: string) => void;

    const CONFIG_SETGROUP = 0xe2;
    const SPI_TX = 0xe0;
    const SPI_RX = 0xe2;

    // NOTE: this protocol value is module-specific — change if needed.
    const CMD_GET_RSSI_PROTOCOL = 0xEE;

    let onMsg: EvtStr = undefined;
    let onMsgBuff: EvtBuff = undefined;
    let onMbitString: EvtStr = undefined;
    let onMbitNumber: EvtNum = undefined;
    let onMbitValue: EvtValue = undefined;
    let onRssi: EvtRssi = undefined;
    let onError: EvtErr = undefined;

    let spi: SPI = undefined;
    let cs = pins.P19;
    let irq = pins.P20;

    let serialCounter = 0;
    let lastRssi: number = null;
    let lastError: string = null;

    // addr id: 
    // e0: tx
    // e1: rx
    // e2: config
    function safeUint8Set(buf: Buffer, idx: number, v: number) {
        // helper to avoid accidental out of range sets
        if (idx >= 0 && idx < buf.length) buf.setUint8(idx, v & 0xff);
    }

    function spiTx(b: Buffer, protocol?: number) {
        if (!spi) return;
        if (!protocol) protocol = SPI_TX;
        try {
            let tx = pins.createBuffer(b.length + 4)
            let rx = pins.createBuffer(b.length + 4)
            // check sum and service num not implement
            safeUint8Set(tx, 0, 0xff)
            safeUint8Set(tx, 1, 0xaa)
            safeUint8Set(tx, 2, protocol)
            safeUint8Set(tx, 3, b.length)
            for (let j = 0; j < b.length; j++) {
                safeUint8Set(tx, j + 4, b[j])
            }
            cs.digitalWrite(false)
            spi.transfer(tx, rx)
            cs.digitalWrite(true)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    function spiRx(): Buffer {
        if (!spi) return null;
        try {
            // max 24 bytes as microbit ble supported
            let tx = pins.createBuffer(24 + 4)
            let rx = pins.createBuffer(24 + 4)

            safeUint8Set(tx, 0, 0xff)
            safeUint8Set(tx, 1, 0xaa)
            safeUint8Set(tx, 2, 0xe1)
            safeUint8Set(tx, 3, 0)

            cs.digitalWrite(false)
            spi.transfer(tx, rx)
            cs.digitalWrite(true)

            let len = rx[3]
            if (len <= 0) return null
            // slice from 4 to 4+len (end index)
            return rx.slice(4, 4 + len)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
            return null
        }
    }

    //% blockId=sdw_init block="SD wireless init"
    //% weight=100
    export function sdw_init(): void {
        try {
            spi = pins.createSPI(pins.P15, pins.P14, pins.P13)
            spi.setMode(3)
            spi.setFrequency(1000000)
            cs.digitalWrite(true)
            let buf = pins.createBuffer(2)
            buf[0] = 0x31;
            buf[1] = 0x32;
            spiTx(buf, 0xea)
            basic.pause(200)
            spiTx(buf, 0xea)
            basic.pause(200)

            // ensure irq pin is set to pull down/up as safe default
            // (platform dependent; may be ignored if hardware controls IRQ)
            //irq.setPull(PullMode.PullNone)

            irq.onEvent(PinEvent.PulseHigh, function () {
                try {
                    let msg = spiRx()
                    let ts = input.runningTime();
                    if (!msg) return;

                    // update serial counter (packet layout reserves bytes 5..8)
                    serialCounter = (serialCounter + 1) & 0xffffffff

                    // If module sends RSSI as a trailing byte in known firmware,
                    // you can detect and update lastRssi here. This is heuristic
                    // and may not be correct for your module. Remove if it collides.
                    if (msg.length >= 2) {
                        // Some firmwares append an RSSI value as the last byte.
                        // We'll treat it as unsigned byte 0..255 and convert to signed -128..127
                        const possibleRssi = msg[msg.length - 1]
                        // treat > 200 as unlikely RSSI; simple heuristic
                        if (possibleRssi <= 200) {
                            lastRssi = (possibleRssi > 127) ? possibleRssi - 256 : possibleRssi
                            if (onRssi) onRssi(lastRssi, ts)
                        }
                    }

                    if (onMsg) onMsg(msg.toString(), ts)
                    if (onMsgBuff) onMsgBuff(msg, ts)

                    if (onMbitNumber && msg[0] == PACKET_TYPE_NUMBER) {
                        let num = msg.getNumber(NumberFormat.Int32LE, 9)
                        onMbitNumber(num, ts)
                    }
                    if (onMbitString && msg[0] == PACKET_TYPE_STRING) {
                        let strLen: number = msg[9]
                        // slice end index = 10 + strLen
                        let strBuf = msg.slice(10, 10 + strLen)
                        onMbitString(strBuf.toString(), ts)
                    }
                    if (onMbitValue && msg[0] == PACKET_TYPE_VALUE) {
                        let value: number = msg.getNumber(NumberFormat.Int32LE, 9)
                        let strLen: number = msg[13]
                        let strBuf = msg.slice(14, 14 + strLen)
                        onMbitValue(strBuf.toString(), value, ts)
                    }
                } catch (err) {
                    lastError = (err && err.message) ? err.message : err + "";
                    if (onError) onError(lastError)
                }
            })
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_tx block="Send message %data"
    //% weight=90
    export function sdw_tx(data: string): void {
        try {
            let buf = pins.createBuffer(data.length)
            for (let i = 0; i < data.length; i++) {
                buf.setUint8(i, data.charCodeAt(i))
            }
            // attach timestamp and serial in a wrapper buffer per packet layout
            let pkt = pins.createBuffer(1 + 4 + 4 + buf.length) // type + time + serial + payload
            pkt[0] = PACKET_TYPE_BUFFER
            // time (ms since start)
            pkt.setNumber(NumberFormat.Int32LE, 1, input.runningTime())
            // serial
            pkt.setNumber(NumberFormat.Int32LE, 5, serialCounter)
            // payload copy after offset 9
            for (let i = 0; i < buf.length; i++) pkt[9 + i] = buf[i]
            spiTx(pkt)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_tx_n block="Send message %data (New Line)"
    //% weight=90
    export function sdw_tx_n(data: string): void {
        sdw_tx(data + "\n")
    }

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

    /*
    Packet byte layout
    | 0              | 1 ... 4       | 5 ... 8           | 9 ... 28
    ----------------------------------------------------------------
    | packet type    | system time   | serial number     | payload
    */
    const PACKET_TYPE_NUMBER = 0;
    const PACKET_TYPE_VALUE = 1;
    const PACKET_TYPE_STRING = 2;
    const PACKET_TYPE_BUFFER = 3;
    const PACKET_TYPE_DOUBLE = 4;
    const PACKET_TYPE_DOUBLE_VALUE = 5;

    //% blockId=sdw_mbit_send_string block="Send Microbit String %data"
    //% weight=80
    export function sdw_mbit_send_string(data: string): void {
        try {
            let buf = pins.createBuffer(9 + 1 + data.length)
            buf[0] = PACKET_TYPE_STRING;
            // timestamp
            buf.setNumber(NumberFormat.Int32LE, 1, input.runningTime())
            // serial
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter)
            buf[9] = data.length
            for (let i = 0; i < data.length; i++) {
                buf[10 + i] = data.charCodeAt(i)
            }
            spiTx(buf)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_mbit_send_number block="Send Microbit Number %data"
    //% weight=80
    export function sdw_mbit_send_number(data: number): void {
        try {
            let buf = pins.createBuffer(4 + 9)
            buf[0] = PACKET_TYPE_NUMBER;
            buf.setNumber(NumberFormat.Int32LE, 1, input.runningTime())
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter)
            buf.setNumber(NumberFormat.Int32LE, 9, data);
            spiTx(buf)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_mbit_send_value block="Send Microbit Value %name = %value"
    //% weight=80
    export function sdw_mbit_send_value(name: string, value: number): void {
        try {
            let buf = pins.createBuffer(9 + 4 + 1 + name.length)
            buf[0] = PACKET_TYPE_VALUE;
            buf.setNumber(NumberFormat.Int32LE, 1, input.runningTime())
            buf.setNumber(NumberFormat.Int32LE, 5, serialCounter)
            buf.setNumber(NumberFormat.Int32LE, 9, value);
            buf[13] = name.length
            for (let i = 0; i < name.length; i++) {
                buf[14 + i] = name.charCodeAt(i)
            }
            spiTx(buf)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

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
            let buf = pins.createBuffer(2)
            buf[0] = 1;
            buf[1] = gp;
            spiTx(buf, CONFIG_SETGROUP)
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_request_rssi block="Request RSSI"
    //% weight=55
    //% help="If your module supports an RSSI command, this will request it."
    export function sdw_request_rssi(): void {
        try {
            // send an empty payload with the RSSI-protocol; module dependent
            let buf = pins.createBuffer(0)
            spiTx(buf, CMD_GET_RSSI_PROTOCOL)
            // small wait then read if IRQ hasn't fired
            basic.pause(50)
            let rx = spiRx()
            if (rx && rx.length > 0) {
                // assume first byte is RSSI signed byte
                let raw = rx[0]
                let r = (raw > 127) ? raw - 256 : raw
                lastRssi = r
                if (onRssi) onRssi(r, input.runningTime())
            }
        } catch (err) {
            lastError = (err && err.message) ? err.message : err + "";
            if (onError) onError(lastError)
        }
    }

    //% blockId=sdw_get_last_rssi block="Get last RSSI"
    //% weight=55
    export function sdw_get_last_rssi(): number {
        return lastRssi === null ? -999 : lastRssi
    }

    //% blockId=sdw_get_last_error block="Get last Error"
    //% weight=55
    export function sdw_get_last_error(): string {
        return lastError || ""
    }

}

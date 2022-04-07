import {
    avrInstruction,
    AVRTimer,
    CPU,
    timer0Config,
    AVRIOPort,
    portBConfig,
    portCConfig,
    portDConfig,
    timer1Config,
    timer2Config,
    AVRADC,
    adcConfig,
    PinState
} from "avr8js";
import { CategoryScale, Chart, LinearScale, LineController, LineElement, PointElement } from "chart.js";
Chart.register([LineController,
    CategoryScale,
    LinearScale,
    PointElement,
    LineElement]);

import program from "../main.txt";

function loadHex(source: string, target: Uint8Array) {
    for (const line of source.split('\n')) {
        if (line[0] === ':' && line.substr(7, 2) === '00') {
            const bytes = parseInt(line.substr(1, 2), 16);
            const addr = parseInt(line.substr(3, 4), 16);
            for (let i = 0; i < bytes; i++) {
                target[addr + i] = parseInt(line.substr(9 + i * 2, 2), 16);
            }
        }
    }
}

const FLASH_SIZE = 0x6000;

const ctx = document.getElementById('myChart').getContext('2d');
const myChart = new Chart(ctx, {
    type: 'line',
    data: {
        datasets: [{
            label: "VoltageA",
            borderColor: "#00F",
            backgroundColor: "#00F",
            data: [],
            pointRadius: 0
        }, {
            label: "VoltageB",
            borderColor: "#000077",
            backgroundColor: "#000077",
            data: [],
            pointRadius: 0
        }, {
            label: "Discharge Current",
            borderColor: "#FF0000",
            backgroundColor: "#FF0000",
            data: [],
            pointRadius: 0
        }, {
            label: "Discharge Power",
            borderColor: "#00AA00",
            backgroundColor: "#00AA00",
            data: [],
            pointRadius: 0
        },
        {
            label: "Avg Discharge Power",
            borderColor: "#00FF00",
            backgroundColor: "#00FF00",
            data: [],
            pointRadius: 0
        }],
        labels: []
    },
    options: {
        responsive: true,
        // scales: {
        //     y: {
        //         beginAtZero: true
        //     }
        // },
        animation: {
            duration: 0, // general animation tim
        },
    }
});



let resistance5v = 10.6;

let dischargeCurrentA = 0;
let dischargeCurrentB = 0;

let capacity = 470e-6;

function getChargeCurrent(voltage) {
    return (0.2254 * voltage ** 2 - 25.262 * voltage + 611.12) / 1000;
}

const drivingSpeedKmh = 25;

const initialCapacitorVoltage = 4;

let voltageA = initialCapacitorVoltage;
let voltageASub = initialCapacitorVoltage;
let voltageB = initialCapacitorVoltage;
let voltageBSub = initialCapacitorVoltage;

let charge_before = null;

const timestep = 1 / 12e6;

function getEquivalentResistance(voltage) {
    // let input_resistance = time < 2500 * timestep ? 1000 : 8;
    // if (voltage < 8) {
    //     return input_resistance;
    // }
    // let i = 5 / input_resistance;
    // let i_at_v = (i * 5 / voltage) * (1 / 0.77);
    // return voltage / i_at_v;
    return resistance5v;
}

let avgPowerUs = 1000e-6;

let chartupdate = 10e-6;

let avgPowerDataPoints = avgPowerUs / chartupdate;

export class Runner {
    program = new Uint16Array(FLASH_SIZE);
    cpu: CPU;
    timer1: AVRTimer;
    timer2: AVRTimer;
    readonly portB: AVRIOPort;
    readonly portC: AVRIOPort;
    readonly adc: AVRADC;
    private time = 0;
    private lastChartUpdate = 0;

    private stopped = false;

    constructor(hex: string) {
        loadHex(hex, new Uint8Array(this.program.buffer));
        this.cpu = new CPU(this.program);
        this.timer1 = new AVRTimer(this.cpu, timer1Config);
        this.timer2 = new AVRTimer(this.cpu, timer2Config);
        this.portB = new AVRIOPort(this.cpu, portBConfig);
        this.portC = new AVRIOPort(this.cpu, portCConfig);
        this.adc = new AVRADC(this.cpu, adcConfig);
        this.portB.setPin(3, false);
        this.portB.setPin(5, false);
    }

    simulateTimesteps(nr: number) {
        this.stopped = false;
        for (let i = 0; i < nr; i++) {
            avrInstruction(this.cpu);
            this.cpu.tick();
            this.simulateTimeStep();
            this.time += timestep;
            if (this.time - this.lastChartUpdate > chartupdate) {
                this.updateChart();
                this.lastChartUpdate = this.time;
            }
        }
        myChart.update();
    }

    private simulateTimeStep() {
        let chargeA = (this.portC.pinState(2)) == PinState.High;
        let chargeB = (this.portC.pinState(4)) == PinState.High;
        let dischargeA = (this.portC.pinState(3)) == PinState.Low;
        let dischargeB = (this.portC.pinState(5)) == PinState.Low;
        if (chargeA && !chargeB) {
            let chargeCurrent = getChargeCurrent(voltageA + voltageASub);
            voltageA += (timestep / capacity) * chargeCurrent;
            voltageASub += (timestep / capacity) * chargeCurrent;
        } else if (chargeB && !chargeA) {
            let chargeCurrent = getChargeCurrent(voltageB + voltageBSub);
            voltageB += (timestep / capacity) * chargeCurrent;
            voltageBSub += (timestep / capacity) * chargeCurrent;
        } else if (chargeA && chargeB) {
            console.error("charging both!");
        }
        dischargeCurrentA = 0;
        dischargeCurrentB = 0;
        if (dischargeA) {
            let r = getEquivalentResistance(voltageA);
            let discharge = Math.exp(-1 * timestep / ((r * 2) * (capacity)));
            dischargeCurrentA += 2 * (voltageA * discharge / (2 * r));
            voltageA = voltageA * discharge;
            voltageASub = voltageASub * discharge;
        }
        if (dischargeB) {
            let r = getEquivalentResistance(voltageB);
            let discharge = Math.exp(-1 * timestep / ((r * 2) * (capacity)));
            dischargeCurrentB += 2 * (voltageA * discharge / (2 * r));
            voltageB = voltageB * discharge;
            voltageBSub = voltageBSub * discharge;
        }

        this.adc.channelValues[0] = (voltageASub * (3 / (13))) / 2.56;
        this.adc.channelValues[1] = (voltageBSub * (3 / (13))) / 2.56;
    }

    private updateChart() {

        myChart.data.datasets[0].data.push(voltageA);
        myChart.data.datasets[1].data.push(voltageB);
        myChart.data.datasets[2].data.push(dischargeCurrentA + dischargeCurrentB);
        myChart.data.datasets[3].data.push((dischargeCurrentA * voltageA) + (dischargeCurrentB * voltageB));
        let avgPower = 0;
        if (myChart.data.datasets[3].data.length > avgPowerDataPoints) {
            avgPower = myChart.data.datasets[3].data.slice(-avgPowerDataPoints).reduce((a, b) => a + b, 0) / avgPowerDataPoints;
        }
        myChart.data.datasets[4].data.push(avgPower);
        // myChart.data.datasets[3].data.push(voltageBSub);

        var length = myChart.data.labels.length;
        for (let d of myChart.data.datasets) {
            d.data = d.data.slice(Math.max(0, length - 500 + 1));
        }

        myChart.data.labels.push(`${Math.round(this.time * 1e3)}ms`);
        myChart.data.labels = myChart.data.labels.slice(Math.max(0, length - 500 + 1));

    }

    getAppstate() {
        let baseAddress = 0x300;
        // let view = this.cpu.dataView;
        // // console.log(this.cpu.data.slice(baseAddress, baseAddress + 0x28));
        // return {
        //     chargeMode: view.getUint8(baseAddress + 0),
        //     dischargeA: view.getUint8(baseAddress + 1),
        //     dischargeB: view.getUint8(baseAddress + 2),
        //     lightRequested: view.getUint8(baseAddress + 3),

        //     dynamoShutoff: view.getUint8(baseAddress + 4),
        //     drivingState: view.getUint8(baseAddress + 5),
        //     cycleCount: view.getUint8(baseAddress + 6),
        //     backOff: view.getUint8(baseAddress + 7),

        //     chargeAValue: view.getUint16(baseAddress + 8, true),
        //     chargeBValue: view.getUint16(baseAddress + 10, true),

        //     maxDischargeValue: view.getUint16(baseAddress + 12, true),
        //     dynamoFrequency: view.getUint16(baseAddress + 14, true),

        //     diff: view.getInt16(baseAddress + 16, true),
        //     avg: view.getInt16(baseAddress + 18, true),

        //     isBraking: view.getUint16(baseAddress + 18, true),
        //     drivingStateTiming: view.getUint16(baseAddress + 20, true),

        //     maxChargeAValue: view.getUint16(baseAddress + 22, true),
        //     maxChargeBValue: view.getUint16(baseAddress + 24, true),
        //     turnOnLimit: view.getUint8(baseAddress + 25)
        // };
        return {
            values: this.cpu.data.slice(baseAddress, baseAddress + 10)
        }
    }

    stop() {
        this.stopped = true;
    }
}

let runner = new Runner(program);

// runner.simulateTimesteps(100);

var interval = null;


document.querySelector("#start").addEventListener("click", () => {
    if (interval == null) {
        interval = setInterval(() => {
            runner.simulateTimesteps(100000);
            document.querySelector("#state").innerText = JSON.stringify(runner.getAppstate(), null, 4);
        }, 100);
    } else {
        clearInterval(interval);
        interval = null;
    }
})

document.querySelector("#step").addEventListener("click", () => {
    runner.simulateTimesteps(400);
    document.querySelector("#state").innerText = JSON.stringify(runner.getAppstate(), null, 4);
})

document.querySelector("#resistance").addEventListener("input", e => {
    resistance5v = e.target.value;
    document.querySelector("[for=resistance]").innerText = `Load Resistance: ${resistance5v}Ω`;
});
document.querySelector("[for=resistance]").innerText = `Load Resistance: ${resistance5v}Ω`;
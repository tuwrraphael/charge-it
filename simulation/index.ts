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
import { CategoryScale, Chart, LinearScale, LineController, LineElement, PointElement, Tooltip, Legend } from "chart.js";
Chart.register([LineController,
    CategoryScale,
    LinearScale,
    PointElement,
    LineElement,
    Tooltip,
    Legend]);

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

const capacitorESR = 0.14 / (2 * Math.PI * 120 * 470e-6);

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
            label: "Capacitor Discharge Current",
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
            label: "Avg Resistor Discharge Power",
            borderColor: "#00FF00",
            backgroundColor: "#00FF00",
            data: [],
            pointRadius: 0
        },
        {
            label: "Dynamo Frequency",
            borderColor: "#000000",
            backgroundColor: "#000000",
            data: [],
            pointRadius: 0,
            hidden: true
        }, {
            label: "Charge Current",
            borderColor: "#AA0000",
            backgroundColor: "#AA0000",
            data: [],
            pointRadius: 0
        },
        {
            label: "Output Capacitor Voltage",
            borderColor: "orange",
            backgroundColor: "orange",
            data: [],
            pointRadius: 0
        },
        {
            label: "Charge Power",
            borderColor: "lightblue",
            backgroundColor: "lightblue",
            data: [],
            pointRadius: 0
        },
        {
            label: "Resistor Discharge Current",
            borderColor: "fuchsia",
            backgroundColor: "fuchsia",
            data: [],
            pointRadius: 0
        },
        {
            label: "Resistor Discharge Power",
            borderColor: "darkgreen",
            backgroundColor: "darkgreen",
            data: [],
            pointRadius: 0
        }
        ],
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

let initialSpeed = 25;

const initialCapacitorVoltage = 4;

let voltageA = initialCapacitorVoltage;
let voltageASub = initialCapacitorVoltage;
let voltageB = initialCapacitorVoltage;
let voltageBSub = initialCapacitorVoltage;

let charge_before = null;

const cpuCycleTimeStep = 1 / 12e6;

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

let avgPowerUs = 500e-6;

let chartupdate = 10e-6;

let avgPowerDataPoints = avgPowerUs / chartupdate;

let chargeCurrentABefore = 0;
let chargeCurrentBBefore = 0;

let maxChargeVoltage = 14.6;
let maxChargeCurrent = 0.2;

let outputCapacity = 100e-6;

function getChargeCapCurrent(capacitorVoltage, timestep) {
    if ((maxChargeVoltage / 2) <= capacitorVoltage) {
        return 0;
    }
    let c = (((maxChargeVoltage / 2) - capacitorVoltage) / (0.6 + capacitorESR)) * Math.exp(-timestep / capacity);
    if (c > maxChargeCurrent) {
        return maxChargeCurrent;
    }
    return c;
}

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
    private dynamoPeriod: number = 0.1;
    private dynamoPosition = 0;
    private dynamoVoltage = 0;
    dynamoFrequency = 0;
    private chargeCurrent = 0;
    private outputCapacitorVoltage = 0.001;
    private outputCapacitorCurrent = 0;
    private resistorCurrent = 0;

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
        this.setSpeed(25);
    }

    setSpeed(kmh: number) {
        this.dynamoFrequency = (16.324 * kmh - 2e-13);
        let newPeriod = (1 / this.dynamoFrequency);
        let adjustPosition = newPeriod / this.dynamoPeriod;
        this.dynamoPosition *= adjustPosition;
        this.dynamoPosition = this.dynamoPosition % newPeriod;
        this.dynamoPeriod = newPeriod;
    }

    simulateTimesteps(nr: number) {

        for (let i = 0; i < nr; i++) {
            var cycles = this.cpu.cycles;
            avrInstruction(this.cpu);
            this.cpu.tick();
            let timeAdvanced = (this.cpu.cycles - cycles) * cpuCycleTimeStep;
            this.simulateTimeStep(timeAdvanced);
            this.time += timeAdvanced;
            if (this.time - this.lastChartUpdate > chartupdate) {
                this.updateChart();
                this.lastChartUpdate = this.time;
            }
        }
        myChart.update();
    }

    private simulateTimeStep(timestep: number) {
        this.dynamoPosition += timestep;
        var dynamoAngle = this.dynamoPosition / this.dynamoPeriod * 2 * Math.PI;
        this.dynamoVoltage = Math.sin(dynamoAngle) * (voltageA + voltageASub);
        let chargeA = (this.portC.pinState(2)) == PinState.High;
        let chargeB = (this.portC.pinState(4)) == PinState.High;
        let dischargeA = (this.portC.pinState(3)) == PinState.Low;
        let dischargeB = (this.portC.pinState(5)) == PinState.Low;
        let dynamoShutoff = (this.portB.pinState(2)) == PinState.High;
        this.chargeCurrent = 0;
        if (dynamoShutoff) {

        }
        else if (chargeA && !chargeB) {

            // let chargeCurrent = getChargeCurrent(voltageA + voltageASub - ((0.6 + capacitorESR) * chargeCurrentABefore));
            // chargeCurrentABefore = chargeCurrent;

            voltageA += (timestep / capacity) * getChargeCapCurrent(voltageA, timestep);
            voltageASub += (timestep / capacity) * getChargeCapCurrent(voltageASub, timestep);
            this.chargeCurrent = getChargeCapCurrent(voltageA, timestep);
        } else if (chargeB && !chargeA) {
            // let chargeCurrent = getChargeCurrent(voltageB + voltageBSub - ((0.6 + capacitorESR) * chargeCurrentBBefore));
            // chargeCurrentBBefore = chargeCurrent;
            voltageB += (timestep / capacity) * getChargeCapCurrent(voltageB, timestep);
            voltageBSub += (timestep / capacity) * getChargeCapCurrent(voltageBSub, timestep);
            this.chargeCurrent = getChargeCapCurrent(voltageA, timestep);
        } else if (chargeA && chargeB) {
            throw "charging both!";
        }
        dischargeCurrentA = 0;
        dischargeCurrentB = 0;
        let r = getEquivalentResistance(this.outputCapacitorVoltage);
        this.resistorCurrent = this.outputCapacitorVoltage / r;
        this.outputCapacitorCurrent = -1 * this.resistorCurrent;

        let dischargeR = 0.4 + 2 * capacitorESR;
        if (dischargeA && dischargeB) {
            // throw "discharging both!";
        }
        if (dischargeA) {
            let dischargeCurrent = (voltageA - this.outputCapacitorVoltage) / (dischargeR ** 2 / (2 * dischargeR));
            voltageA -= (timestep / capacity) * (dischargeCurrent / 2);
            voltageASub -= (timestep / capacity) * (dischargeCurrent / 2);
            this.outputCapacitorCurrent += dischargeCurrent;
            dischargeCurrentA = dischargeCurrent;
        }
        if (dischargeB) {
            let dischargeCurrent = (voltageB - this.outputCapacitorVoltage) / (dischargeR ** 2 / (2 * dischargeR));
            voltageB -= (timestep / capacity) * (dischargeCurrent / 2);
            voltageBSub -= (timestep / capacity) * (dischargeCurrent / 2);
            this.outputCapacitorCurrent += dischargeCurrent;
            dischargeCurrentB = dischargeCurrent;
        }

        this.outputCapacitorVoltage += (timestep / outputCapacity) * (this.outputCapacitorCurrent);

        this.adc.channelValues[0] = ((voltageASub * (3 / (13))) / 2.56) * this.adc.referenceVoltage;
        this.adc.channelValues[1] = ((this.outputCapacitorVoltage * (3 / (13))) / 2.56) * this.adc.referenceVoltage;
        var dynamoSensePinState = Math.max(Math.min(this.dynamoVoltage, 3.6), 0) > 2.5 ? true : false;
        this.portB.setPin(0, dynamoSensePinState);
    }

    private updateChart() {

        myChart.data.datasets[0].data.push(voltageA);
        myChart.data.datasets[1].data.push(voltageB);
        myChart.data.datasets[2].data.push(dischargeCurrentA + dischargeCurrentB);
        myChart.data.datasets[3].data.push(this.outputCapacitorCurrent * this.outputCapacitorVoltage);

        // myChart.data.datasets[3].data.push(voltageBSub);
        myChart.data.datasets[5].data.push(this.dynamoVoltage);
        myChart.data.datasets[6].data.push(this.chargeCurrent);
        myChart.data.datasets[7].data.push(this.outputCapacitorVoltage);
        let chargeA = (this.portC.pinState(2)) == PinState.High;
        let chargeB = (this.portC.pinState(4)) == PinState.High;
        if (chargeA) {
            myChart.data.datasets[8].data.push((voltageA + voltageASub) * this.chargeCurrent);
        } else if (chargeB) {
            myChart.data.datasets[8].data.push((voltageB + voltageBSub) * this.chargeCurrent);
        } else {
            myChart.data.datasets[8].data.push(0);
        }
        myChart.data.datasets[9].data.push(this.resistorCurrent);
        myChart.data.datasets[10].data.push(this.outputCapacitorVoltage * this.resistorCurrent);
        let avgPower = 0;
        if (myChart.data.datasets[10].data.length > avgPowerDataPoints) {
            avgPower = myChart.data.datasets[10].data.slice(-avgPowerDataPoints).reduce((a, b) => a + b, 0) / avgPowerDataPoints;
        }
        myChart.data.datasets[4].data.push(avgPower);

        var length = myChart.data.labels.length;
        for (let d of myChart.data.datasets) {
            d.data = d.data.slice(Math.max(0, length - 500 + 1));
        }

        myChart.data.labels.push(`${Math.round(this.time * 1e3 * 100) / 100}ms`);
        myChart.data.labels = myChart.data.labels.slice(Math.max(0, length - 500 + 1));

    }

    private ocra1 = 0;

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

        let drivingState;
        switch (this.cpu.dataView.getUint8(baseAddress)) {
            case 0:
                drivingState = "starting";
                break;
            case 1:
                drivingState = "driving";
                break;
            case 2:
                drivingState = "stopped";
                break;
            case 3:
                drivingState = "stopping";
                break;
        }
        let x = {

            drivingState: drivingState,
            maxDischargeValue: this.cpu.dataView.getUint16(baseAddress + 1, true),
            dynamoFrequency: this.cpu.dataView.getUint16(baseAddress + 4, true),
            chargeMeasurementCount: this.cpu.dataView.getUint8(baseAddress + 3),
            chargeVoltageSum: this.cpu.dataView.getUint16(baseAddress + 6, true),
            chargeCurrentSum: this.cpu.dataView.getUint16(baseAddress + 8, true),
            ocr1a: this.cpu.dataView.getUint16(baseAddress + 10, true),
            tcnt1: this.timer1.debugTCNT,
            power: this.cpu.dataView.getUint16(baseAddress + 12, true),
            mpptDirection: this.cpu.dataView.getUint8(baseAddress + 14) > 0 ? "down" : "up",
            mpptStepTiming: this.cpu.dataView.getUint16(baseAddress + 15, true),
            mpptStepSize: this.cpu.dataView.getUint8(baseAddress + 17),
        }
        if (x.ocr1a != this.ocra1) {
            console.log(x.power, this.ocra1);
            this.ocra1 = x.ocr1a;
        }
        return x;
    }
}

let runner = new Runner(program);

// runner.simulateTimesteps(100);

var interval = null;

function applyState() {
    var state = runner.getAppstate();

    document.querySelector("#chargeA").innerText = runner.portC.pinState(2) == PinState.High ? "ON" : "OFF";
    document.querySelector("#chargeB").innerText = runner.portC.pinState(4) == PinState.High ? "ON" : "OFF";
    document.querySelector("#dischargeA").innerText = runner.portC.pinState(3) == PinState.Low ? "ON" : "OFF";
    document.querySelector("#dischargeB").innerText = runner.portC.pinState(5) == PinState.Low ? "ON" : "OFF";
    document.querySelector("#dynamoOff").innerText = runner.portB.pinState(2) == PinState.High ? "ON" : "OFF";
    document.querySelector("#drivingState").innerText = state.drivingState;
    document.querySelector("#maxDischargeValue").innerText = state.maxDischargeValue;
    document.querySelector("#dynamoFrequency").innerText = state.dynamoFrequency;
    document.querySelector("#chargeMeasurementCount").innerText = state.chargeMeasurementCount;
    document.querySelector("#chargeVoltageSum").innerText = state.chargeVoltageSum;
    document.querySelector("#chargeCurrentSum").innerText = state.chargeCurrentSum;
    document.querySelector("#ocr1a").innerText = state.ocr1a;
    document.querySelector("#tcnt1").innerText = state.tcnt1;
    document.querySelector("#power").innerText = state.power;
    document.querySelector("#mpptDirection").innerText = state.mpptDirection;
    document.querySelector("#mpptStepTiming").innerText = state.mpptStepTiming;
    document.querySelector("#mpptStepSize").innerText = state.mpptStepSize;
}

document.querySelector("#start").addEventListener("click", () => {
    if (interval == null) {
        interval = setInterval(() => {
            runner.simulateTimesteps(100000);
            applyState();
        }, 100);
    } else {
        clearInterval(interval);
        interval = null;
    }
})

document.querySelector("#step").addEventListener("click", () => {
    runner.simulateTimesteps(500);
    applyState();
})

document.querySelector("#resistance").addEventListener("input", e => {
    resistance5v = e.target.value;
    document.querySelector("[for=resistance]").innerText = `Load Resistance: ${resistance5v}Ω`;
});
document.querySelector("[for=resistance]").innerText = `Load Resistance: ${resistance5v}Ω`;
document.querySelector("#speed").addEventListener("input", e => {
    runner.setSpeed(e.target.value);
    document.querySelector("[for=speed]").innerText = `Driving Speed: ${e.target.value}km/h (${Math.round(runner.dynamoFrequency)}Hz)`;
});
document.querySelector("#rangeocr1a").addEventListener("input", e => {
    runner.cpu.writeData(timer1Config.OCRA + 1, e.target.value >> 8 & 0xFF);
    runner.cpu.writeData(timer1Config.OCRA, e.target.value & 0xFF);
});
runner.setSpeed(initialSpeed);
document.querySelector("[for=speed]").innerText = `Driving Speed: ${initialSpeed}km/h (${Math.round(runner.dynamoFrequency)}Hz)`;
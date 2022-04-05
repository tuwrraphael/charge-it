const initialCapacitorVoltage = 9;

let resistance5v = 10;

let capacity = 470e-6;

function getChargeCurrent(voltage) {
    return (0.2254 * voltage ** 2 - 25.262 * voltage + 611.12) / 1000;
}

const drivingSpeedKmh = 25;

let voltageA = initialCapacitorVoltage;
let voltageASub = initialCapacitorVoltage;
let voltageB = initialCapacitorVoltage;
let voltageBSub = initialCapacitorVoltage;


let max_charge = 9;

function executeProgram(va, vb) {
    let charge = null;
    if (va < max_charge || vb < max_charge) {
        if (va < vb) {
            charge = "a";
        } else {
            charge = "b";
        }
    }


    return {
        charge: charge,
        dischargeA: va > 3.5 && charge != "a",
        dischargeB: vb > 3.5 && charge != "b"
    };
}

const timestep = 1e-6;

function simulateTimeStep(programOutput) {
    if (programOutput.charge == "a") {
        let chargeCurrent = getChargeCurrent(voltageA + voltageASub);
        voltageA += (timestep / capacity) * chargeCurrent;
        voltageASub += (timestep / capacity) * chargeCurrent;
    } else if (programOutput.charge == "b") {
        let chargeCurrent = getChargeCurrent(voltageB + voltageBSub);
        voltageB += (timestep / capacity) * chargeCurrent;
        voltageBSub += (timestep / capacity) * chargeCurrent;
    }
    if (programOutput.dischargeA) {
        let discharge = Math.exp(timestep / ((-1 * resistance5v) * (2 * capacity)));
        voltageA = voltageA * discharge;
        voltageASub = voltageASub * discharge;
    }
    if (programOutput.dischargeB) {
        let discharge = Math.exp(timestep / ((-1 * resistance5v) * (2 * capacity)));
        voltageB = voltageB * discharge;
        voltageBSub = voltageBSub * discharge;
    }
}

let programRunTime = 200e-6;

for (let i = 0; i < 1000; i++) {
    let output = executeProgram(voltageASub, voltageBSub);
    for (let j = 0; j < programRunTime / timestep; j++) {
        simulateTimeStep(output);
    }
}






// let time = 0;



// function getEquivalentResistance(voltage) {
//     return resistance5v;
//     // if (voltage < 8) 
//     // return resistance5v;
//     // let i = 5 / resistance5v;
//     // let i_at_v = i * 5 / voltage;
//     // return voltage / (i_at_v  * 1/0.75);
// }



// let chargeVoltage = 0.13;



// for (var i = 0; i < 100; i++) {
//     let resistance = getEquivalentResistance(voltageA + chargeVoltage);
//     var n = voltageA / (voltageA + chargeVoltage);
//     let dischargeTime = ((-1 * resistance) * (2 * capacity) * Math.log(n));
//     console.log(dischargeTime * 1e6);

//     if (dischargeTime < minDischargeTime) {
//         let discharge = Math.exp(minDischargeTime / ((-1 * resistance) * (2 * capacity)));
//         voltageA = voltageA * discharge;
//         dischargeTime = minDischargeTime;
//     }

//     chargeVoltage = 0;
//     for (var j = 0; j < dischargeTime; j += 1e-6) {
//         chargeVoltage += (1e-6 / capacity) * getChargeCurrent(2 * voltageA + chargeVoltage);
//     }
//     console.log(resistance, voltageA + chargeVoltage, dischargeTime * 1e6, chargeVoltage, getChargeCurrent(2 * voltageA + chargeVoltage), (2 * voltageA) * getChargeCurrent(2 * voltageA + chargeVoltage));
// }






// let chargeCurrent = getChargeCurrent(voltageA*2);



// console.log(chargeCurrent, chargeVoltage);
// let chargeTime = capacity * chargeVoltage / chargeCurrent;



// console.log(chargeTime * 1e6, dischargeTime * 1e6);
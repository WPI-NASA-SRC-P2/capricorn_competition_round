
"use strict";

let BrakeRoverSrv = require('./BrakeRoverSrv.js')
let PidListSrv = require('./PidListSrv.js')
let ChargingStationSrv = require('./ChargingStationSrv.js')
let StartStopSrv = require('./StartStopSrv.js')
let LocalizationSrv = require('./LocalizationSrv.js')
let ToggleLightSrv = require('./ToggleLightSrv.js')
let SystemPowerSaveSrv = require('./SystemPowerSaveSrv.js')
let PidTuningSrv = require('./PidTuningSrv.js')

module.exports = {
  BrakeRoverSrv: BrakeRoverSrv,
  PidListSrv: PidListSrv,
  ChargingStationSrv: ChargingStationSrv,
  StartStopSrv: StartStopSrv,
  LocalizationSrv: LocalizationSrv,
  ToggleLightSrv: ToggleLightSrv,
  SystemPowerSaveSrv: SystemPowerSaveSrv,
  PidTuningSrv: PidTuningSrv,
};

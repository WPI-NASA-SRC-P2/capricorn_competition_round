
"use strict";

let BrakeRoverSrv = require('./BrakeRoverSrv.js')
let ChargingStationSrv = require('./ChargingStationSrv.js')
let StartStopSrv = require('./StartStopSrv.js')
let LocalizationSrv = require('./LocalizationSrv.js')
let ToggleLightSrv = require('./ToggleLightSrv.js')

module.exports = {
  BrakeRoverSrv: BrakeRoverSrv,
  ChargingStationSrv: ChargingStationSrv,
  StartStopSrv: StartStopSrv,
  LocalizationSrv: LocalizationSrv,
  ToggleLightSrv: ToggleLightSrv,
};

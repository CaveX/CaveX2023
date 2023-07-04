
"use strict";

let RestartController = require('./RestartController.js')
let StopController = require('./StopController.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetSpeed = require('./SetSpeed.js')
let StartController = require('./StartController.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')

module.exports = {
  RestartController: RestartController,
  StopController: StopController,
  SetTorqueLimit: SetTorqueLimit,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceSlope: SetComplianceSlope,
  SetSpeed: SetSpeed,
  StartController: StartController,
  TorqueEnable: TorqueEnable,
  SetComplianceMargin: SetComplianceMargin,
};

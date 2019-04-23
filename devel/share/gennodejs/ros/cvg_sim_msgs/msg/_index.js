
"use strict";

let AttitudeCommand = require('./AttitudeCommand.js');
let MotorCommand = require('./MotorCommand.js');
let MotorPWM = require('./MotorPWM.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let RawRC = require('./RawRC.js');
let ThrustCommand = require('./ThrustCommand.js');
let ControllerState = require('./ControllerState.js');
let Altimeter = require('./Altimeter.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let RawMagnetic = require('./RawMagnetic.js');
let Supply = require('./Supply.js');
let HeadingCommand = require('./HeadingCommand.js');
let RC = require('./RC.js');
let Altitude = require('./Altitude.js');
let MotorStatus = require('./MotorStatus.js');
let YawrateCommand = require('./YawrateCommand.js');
let HeightCommand = require('./HeightCommand.js');
let RawImu = require('./RawImu.js');
let ServoCommand = require('./ServoCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let Compass = require('./Compass.js');

module.exports = {
  AttitudeCommand: AttitudeCommand,
  MotorCommand: MotorCommand,
  MotorPWM: MotorPWM,
  VelocityZCommand: VelocityZCommand,
  RawRC: RawRC,
  ThrustCommand: ThrustCommand,
  ControllerState: ControllerState,
  Altimeter: Altimeter,
  PositionXYCommand: PositionXYCommand,
  RawMagnetic: RawMagnetic,
  Supply: Supply,
  HeadingCommand: HeadingCommand,
  RC: RC,
  Altitude: Altitude,
  MotorStatus: MotorStatus,
  YawrateCommand: YawrateCommand,
  HeightCommand: HeightCommand,
  RawImu: RawImu,
  ServoCommand: ServoCommand,
  RuddersCommand: RuddersCommand,
  VelocityXYCommand: VelocityXYCommand,
  Compass: Compass,
};

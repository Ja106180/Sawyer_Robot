
"use strict";

let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let NavigatorState = require('./NavigatorState.js');
let EndpointState = require('./EndpointState.js');
let IODataStatus = require('./IODataStatus.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let JointCommand = require('./JointCommand.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let InteractionControlState = require('./InteractionControlState.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let HomingCommand = require('./HomingCommand.js');
let HomingState = require('./HomingState.js');
let AnalogIOState = require('./AnalogIOState.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let DigitalIOState = require('./DigitalIOState.js');
let IOStatus = require('./IOStatus.js');
let JointLimits = require('./JointLimits.js');
let IONodeStatus = require('./IONodeStatus.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let NavigatorStates = require('./NavigatorStates.js');
let IONodeConfiguration = require('./IONodeConfiguration.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let SEAJointState = require('./SEAJointState.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let EndpointStates = require('./EndpointStates.js');
let IODeviceStatus = require('./IODeviceStatus.js');
let CameraSettings = require('./CameraSettings.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let CameraControl = require('./CameraControl.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let HeadState = require('./HeadState.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');

module.exports = {
  AnalogOutputCommand: AnalogOutputCommand,
  NavigatorState: NavigatorState,
  EndpointState: EndpointState,
  IODataStatus: IODataStatus,
  InteractionControlCommand: InteractionControlCommand,
  EndpointNamesArray: EndpointNamesArray,
  JointCommand: JointCommand,
  URDFConfiguration: URDFConfiguration,
  DigitalIOStates: DigitalIOStates,
  DigitalOutputCommand: DigitalOutputCommand,
  InteractionControlState: InteractionControlState,
  IOComponentStatus: IOComponentStatus,
  HomingCommand: HomingCommand,
  HomingState: HomingState,
  AnalogIOState: AnalogIOState,
  CollisionDetectionState: CollisionDetectionState,
  DigitalIOState: DigitalIOState,
  IOStatus: IOStatus,
  JointLimits: JointLimits,
  IONodeStatus: IONodeStatus,
  IOComponentCommand: IOComponentCommand,
  NavigatorStates: NavigatorStates,
  IONodeConfiguration: IONodeConfiguration,
  AnalogIOStates: AnalogIOStates,
  SEAJointState: SEAJointState,
  IODeviceConfiguration: IODeviceConfiguration,
  CollisionAvoidanceState: CollisionAvoidanceState,
  EndpointStates: EndpointStates,
  IODeviceStatus: IODeviceStatus,
  CameraSettings: CameraSettings,
  HeadPanCommand: HeadPanCommand,
  RobotAssemblyState: RobotAssemblyState,
  CameraControl: CameraControl,
  IOComponentConfiguration: IOComponentConfiguration,
  HeadState: HeadState,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandResult: CalibrationCommandResult,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
  CalibrationCommandAction: CalibrationCommandAction,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
};


"use strict";

let simRosReadDistance = require('./simRosReadDistance.js')
let simRosGetStringSignal = require('./simRosGetStringSignal.js')
let simRosGetUIButtonProperty = require('./simRosGetUIButtonProperty.js')
let simRosGetDialogInput = require('./simRosGetDialogInput.js')
let simRosReadForceSensor = require('./simRosReadForceSensor.js')
let simRosStopSimulation = require('./simRosStopSimulation.js')
let simRosTransferFile = require('./simRosTransferFile.js')
let simRosSetArrayParameter = require('./simRosSetArrayParameter.js')
let simRosSetObjectSelection = require('./simRosSetObjectSelection.js')
let simRosSetObjectPosition = require('./simRosSetObjectPosition.js')
let simRosAppendStringSignal = require('./simRosAppendStringSignal.js')
let simRosSetUIButtonProperty = require('./simRosSetUIButtonProperty.js')
let simRosSetModelProperty = require('./simRosSetModelProperty.js')
let simRosGetIntegerParameter = require('./simRosGetIntegerParameter.js')
let simRosRemoveModel = require('./simRosRemoveModel.js')
let simRosAddStatusbarMessage = require('./simRosAddStatusbarMessage.js')
let simRosSetSphericalJointMatrix = require('./simRosSetSphericalJointMatrix.js')
let simRosSetJointTargetPosition = require('./simRosSetJointTargetPosition.js')
let simRosCopyPasteObjects = require('./simRosCopyPasteObjects.js')
let simRosReadVisionSensor = require('./simRosReadVisionSensor.js')
let simRosGetObjectFloatParameter = require('./simRosGetObjectFloatParameter.js')
let simRosSetVisionSensorImage = require('./simRosSetVisionSensorImage.js')
let simRosClearIntegerSignal = require('./simRosClearIntegerSignal.js')
let simRosGetObjectPose = require('./simRosGetObjectPose.js')
let simRosAuxiliaryConsoleClose = require('./simRosAuxiliaryConsoleClose.js')
let simRosCallScriptFunction = require('./simRosCallScriptFunction.js')
let simRosAuxiliaryConsolePrint = require('./simRosAuxiliaryConsolePrint.js')
let simRosClearStringSignal = require('./simRosClearStringSignal.js')
let simRosSetObjectPose = require('./simRosSetObjectPose.js')
let simRosGetObjectParent = require('./simRosGetObjectParent.js')
let simRosSynchronousTrigger = require('./simRosSynchronousTrigger.js')
let simRosGetCollectionHandle = require('./simRosGetCollectionHandle.js')
let simRosSetObjectIntParameter = require('./simRosSetObjectIntParameter.js')
let simRosLoadUI = require('./simRosLoadUI.js')
let simRosSynchronous = require('./simRosSynchronous.js')
let simRosSetUISlider = require('./simRosSetUISlider.js')
let simRosGetObjectGroupData = require('./simRosGetObjectGroupData.js')
let simRosAuxiliaryConsoleShow = require('./simRosAuxiliaryConsoleShow.js')
let simRosCreateDummy = require('./simRosCreateDummy.js')
let simRosSetStringSignal = require('./simRosSetStringSignal.js')
let simRosSetIntegerParameter = require('./simRosSetIntegerParameter.js')
let simRosRemoveUI = require('./simRosRemoveUI.js')
let simRosGetFloatSignal = require('./simRosGetFloatSignal.js')
let simRosSetJointForce = require('./simRosSetJointForce.js')
let simRosGetAndClearStringSignal = require('./simRosGetAndClearStringSignal.js')
let simRosBreakForceSensor = require('./simRosBreakForceSensor.js')
let simRosGetVisionSensorImage = require('./simRosGetVisionSensorImage.js')
let simRosSetFloatingParameter = require('./simRosSetFloatingParameter.js')
let simRosGetObjectHandle = require('./simRosGetObjectHandle.js')
let simRosReadCollision = require('./simRosReadCollision.js')
let simRosGetIntegerSignal = require('./simRosGetIntegerSignal.js')
let simRosSetJointPosition = require('./simRosSetJointPosition.js')
let simRosGetJointMatrix = require('./simRosGetJointMatrix.js')
let simRosSetObjectFloatParameter = require('./simRosSetObjectFloatParameter.js')
let simRosGetObjects = require('./simRosGetObjects.js')
let simRosGetDistanceHandle = require('./simRosGetDistanceHandle.js')
let simRosAuxiliaryConsoleOpen = require('./simRosAuxiliaryConsoleOpen.js')
let simRosSetUIButtonLabel = require('./simRosSetUIButtonLabel.js')
let simRosSetFloatSignal = require('./simRosSetFloatSignal.js')
let simRosGetUIEventButton = require('./simRosGetUIEventButton.js')
let simRosGetUISlider = require('./simRosGetUISlider.js')
let simRosGetObjectSelection = require('./simRosGetObjectSelection.js')
let simRosCloseScene = require('./simRosCloseScene.js')
let simRosEraseFile = require('./simRosEraseFile.js')
let simRosGetCollisionHandle = require('./simRosGetCollisionHandle.js')
let simRosDisablePublisher = require('./simRosDisablePublisher.js')
let simRosClearFloatSignal = require('./simRosClearFloatSignal.js')
let simRosGetUIHandle = require('./simRosGetUIHandle.js')
let simRosSetObjectParent = require('./simRosSetObjectParent.js')
let simRosStartSimulation = require('./simRosStartSimulation.js')
let simRosDisplayDialog = require('./simRosDisplayDialog.js')
let simRosGetFloatingParameter = require('./simRosGetFloatingParameter.js')
let simRosLoadModel = require('./simRosLoadModel.js')
let simRosSetJointTargetVelocity = require('./simRosSetJointTargetVelocity.js')
let simRosPauseSimulation = require('./simRosPauseSimulation.js')
let simRosSetJointState = require('./simRosSetJointState.js')
let simRosGetArrayParameter = require('./simRosGetArrayParameter.js')
let simRosGetBooleanParameter = require('./simRosGetBooleanParameter.js')
let simRosSetIntegerSignal = require('./simRosSetIntegerSignal.js')
let simRosSetObjectQuaternion = require('./simRosSetObjectQuaternion.js')
let simRosEndDialog = require('./simRosEndDialog.js')
let simRosEnablePublisher = require('./simRosEnablePublisher.js')
let simRosRemoveObject = require('./simRosRemoveObject.js')
let simRosLoadScene = require('./simRosLoadScene.js')
let simRosGetDialogResult = require('./simRosGetDialogResult.js')
let simRosEnableSubscriber = require('./simRosEnableSubscriber.js')
let simRosGetModelProperty = require('./simRosGetModelProperty.js')
let simRosGetInfo = require('./simRosGetInfo.js')
let simRosGetStringParameter = require('./simRosGetStringParameter.js')
let simRosSetBooleanParameter = require('./simRosSetBooleanParameter.js')
let simRosGetJointState = require('./simRosGetJointState.js')
let simRosReadProximitySensor = require('./simRosReadProximitySensor.js')
let simRosGetLastErrors = require('./simRosGetLastErrors.js')
let simRosGetObjectIntParameter = require('./simRosGetObjectIntParameter.js')
let simRosDisableSubscriber = require('./simRosDisableSubscriber.js')
let simRosGetObjectChild = require('./simRosGetObjectChild.js')
let simRosGetVisionSensorDepthBuffer = require('./simRosGetVisionSensorDepthBuffer.js')

module.exports = {
  simRosReadDistance: simRosReadDistance,
  simRosGetStringSignal: simRosGetStringSignal,
  simRosGetUIButtonProperty: simRosGetUIButtonProperty,
  simRosGetDialogInput: simRosGetDialogInput,
  simRosReadForceSensor: simRosReadForceSensor,
  simRosStopSimulation: simRosStopSimulation,
  simRosTransferFile: simRosTransferFile,
  simRosSetArrayParameter: simRosSetArrayParameter,
  simRosSetObjectSelection: simRosSetObjectSelection,
  simRosSetObjectPosition: simRosSetObjectPosition,
  simRosAppendStringSignal: simRosAppendStringSignal,
  simRosSetUIButtonProperty: simRosSetUIButtonProperty,
  simRosSetModelProperty: simRosSetModelProperty,
  simRosGetIntegerParameter: simRosGetIntegerParameter,
  simRosRemoveModel: simRosRemoveModel,
  simRosAddStatusbarMessage: simRosAddStatusbarMessage,
  simRosSetSphericalJointMatrix: simRosSetSphericalJointMatrix,
  simRosSetJointTargetPosition: simRosSetJointTargetPosition,
  simRosCopyPasteObjects: simRosCopyPasteObjects,
  simRosReadVisionSensor: simRosReadVisionSensor,
  simRosGetObjectFloatParameter: simRosGetObjectFloatParameter,
  simRosSetVisionSensorImage: simRosSetVisionSensorImage,
  simRosClearIntegerSignal: simRosClearIntegerSignal,
  simRosGetObjectPose: simRosGetObjectPose,
  simRosAuxiliaryConsoleClose: simRosAuxiliaryConsoleClose,
  simRosCallScriptFunction: simRosCallScriptFunction,
  simRosAuxiliaryConsolePrint: simRosAuxiliaryConsolePrint,
  simRosClearStringSignal: simRosClearStringSignal,
  simRosSetObjectPose: simRosSetObjectPose,
  simRosGetObjectParent: simRosGetObjectParent,
  simRosSynchronousTrigger: simRosSynchronousTrigger,
  simRosGetCollectionHandle: simRosGetCollectionHandle,
  simRosSetObjectIntParameter: simRosSetObjectIntParameter,
  simRosLoadUI: simRosLoadUI,
  simRosSynchronous: simRosSynchronous,
  simRosSetUISlider: simRosSetUISlider,
  simRosGetObjectGroupData: simRosGetObjectGroupData,
  simRosAuxiliaryConsoleShow: simRosAuxiliaryConsoleShow,
  simRosCreateDummy: simRosCreateDummy,
  simRosSetStringSignal: simRosSetStringSignal,
  simRosSetIntegerParameter: simRosSetIntegerParameter,
  simRosRemoveUI: simRosRemoveUI,
  simRosGetFloatSignal: simRosGetFloatSignal,
  simRosSetJointForce: simRosSetJointForce,
  simRosGetAndClearStringSignal: simRosGetAndClearStringSignal,
  simRosBreakForceSensor: simRosBreakForceSensor,
  simRosGetVisionSensorImage: simRosGetVisionSensorImage,
  simRosSetFloatingParameter: simRosSetFloatingParameter,
  simRosGetObjectHandle: simRosGetObjectHandle,
  simRosReadCollision: simRosReadCollision,
  simRosGetIntegerSignal: simRosGetIntegerSignal,
  simRosSetJointPosition: simRosSetJointPosition,
  simRosGetJointMatrix: simRosGetJointMatrix,
  simRosSetObjectFloatParameter: simRosSetObjectFloatParameter,
  simRosGetObjects: simRosGetObjects,
  simRosGetDistanceHandle: simRosGetDistanceHandle,
  simRosAuxiliaryConsoleOpen: simRosAuxiliaryConsoleOpen,
  simRosSetUIButtonLabel: simRosSetUIButtonLabel,
  simRosSetFloatSignal: simRosSetFloatSignal,
  simRosGetUIEventButton: simRosGetUIEventButton,
  simRosGetUISlider: simRosGetUISlider,
  simRosGetObjectSelection: simRosGetObjectSelection,
  simRosCloseScene: simRosCloseScene,
  simRosEraseFile: simRosEraseFile,
  simRosGetCollisionHandle: simRosGetCollisionHandle,
  simRosDisablePublisher: simRosDisablePublisher,
  simRosClearFloatSignal: simRosClearFloatSignal,
  simRosGetUIHandle: simRosGetUIHandle,
  simRosSetObjectParent: simRosSetObjectParent,
  simRosStartSimulation: simRosStartSimulation,
  simRosDisplayDialog: simRosDisplayDialog,
  simRosGetFloatingParameter: simRosGetFloatingParameter,
  simRosLoadModel: simRosLoadModel,
  simRosSetJointTargetVelocity: simRosSetJointTargetVelocity,
  simRosPauseSimulation: simRosPauseSimulation,
  simRosSetJointState: simRosSetJointState,
  simRosGetArrayParameter: simRosGetArrayParameter,
  simRosGetBooleanParameter: simRosGetBooleanParameter,
  simRosSetIntegerSignal: simRosSetIntegerSignal,
  simRosSetObjectQuaternion: simRosSetObjectQuaternion,
  simRosEndDialog: simRosEndDialog,
  simRosEnablePublisher: simRosEnablePublisher,
  simRosRemoveObject: simRosRemoveObject,
  simRosLoadScene: simRosLoadScene,
  simRosGetDialogResult: simRosGetDialogResult,
  simRosEnableSubscriber: simRosEnableSubscriber,
  simRosGetModelProperty: simRosGetModelProperty,
  simRosGetInfo: simRosGetInfo,
  simRosGetStringParameter: simRosGetStringParameter,
  simRosSetBooleanParameter: simRosSetBooleanParameter,
  simRosGetJointState: simRosGetJointState,
  simRosReadProximitySensor: simRosReadProximitySensor,
  simRosGetLastErrors: simRosGetLastErrors,
  simRosGetObjectIntParameter: simRosGetObjectIntParameter,
  simRosDisableSubscriber: simRosDisableSubscriber,
  simRosGetObjectChild: simRosGetObjectChild,
  simRosGetVisionSensorDepthBuffer: simRosGetVisionSensorDepthBuffer,
};

# Copyright 2013 Velodyne Acoustics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import csv
import datetime
import time
import math
import sys
import paraview.simple as smp
from paraview import servermanager
from paraview import vtk

import PythonQt
from PythonQt import QtCore, QtGui

from vtk import vtkXMLPolyDataWriter
import lidarviewcore.kiwiviewerExporter as kiwiviewerExporter
import gridAdjustmentDialog
import aboutDialog
import bisect

from PythonQt.paraview import vvCalibrationDialog, vvCropReturnsDialog, vvSelectFramesDialog

# import the vtk wrapping of the Lidar Plugin
# this enable to get the specific vtkObject behind a proxy via GetClientSideObject()
# without this plugin, GetClientSideObject(), would return the first mother class known by paraview
import LidarPluginPython

import planefit


_repCache = {}

SAMPLE_PROCESSING_MODE = False

def vtkGetFileNameFromPluginName(pluginName):
  import os
  if os.name == "nt":
    return pluginName + ".dll";
  elif sys.platform == "darwin":
    return "lib" + pluginName + ".dylib";
  else:
    return "lib" + pluginName + ".so";

def cachedGetRepresentation(src, view):
    try:
        return _repCache[(src, view)]
    except KeyError:
        rep = smp.GetRepresentation(src, view)
        _repCache[(src, view)] = rep
        return rep

class AppLogic(object):

    def __init__(self):
        self.createStatusBarWidgets()

        self.mousePressed = False

        mainView = smp.GetActiveView()
        self.mainView = mainView

        self.transformMode = 0
        self.relativeTransform = False

        self.reader = None
        self.trailingFrame = None
        self.position = None
        self.sensor = None

        self.gridProperties = None

        #smp.LoadPlugin(vtkGetFileNameFromPluginName('PointCloudPlugin'))
        smp.LoadPlugin(vtkGetFileNameFromPluginName('EyeDomeLightingView'))


    def createStatusBarWidgets(self):

        self.logoLabel = QtGui.QLabel()
        self.logoLabel.setPixmap(QtGui.QPixmap(":/vvResources/SoftwareInformation/bottom_logo.png"))
        self.logoLabel.setScaledContents(True)

        self.filenameLabel = QtGui.QLabel()
        self.statusLabel = QtGui.QLabel()
        self.sensorInformationLabel = QtGui.QLabel()
        self.positionPacketInfoLabel = QtGui.QLabel()


class GridProperties:

    def __init__(self):
        self.Normal = [0, 0, 0]
        self.Origin = [0, 0, 0]
        self.Scale = 0
        self.GridNbTicks = 0
        self.LineWidth = 0
        self.Color = [0, 0, 0]
        self.Persist = False



def hasArrayName(sourceProxy, arrayName):
    '''
    Returns True if the data has non-zero points and has a point data
    attribute with the given arrayName.
    '''
    if not sourceProxy:
        return False

    info = sourceProxy.GetDataInformation().DataInformation

    if info.GetNumberOfPoints() == 0:
        return False

    info = info.GetAttributeInformation(0)
    for i in range(info.GetNumberOfArrays()):
        if info.GetArrayInformation(i).GetName() == arrayName:
            return True
    return False


def openData(filename):

    close()

    reader = smp.OpenDataFile(filename, guiName='Data')

    if not reader:
        return

    rep = smp.Show(reader)
    rep.InterpolateScalarsBeforeMapping = 0
    setDefaultLookupTables(reader)
    colorByIntensity(reader)

    showSourceInSpreadSheet(reader)

    smp.GetActiveView().ViewTime = 0.0

    app.reader = reader
    app.filenameLabel.setText('File: %s' % os.path.basename(filename))

    enableSaveActions()
    addRecentFile(filename)
    app.actions['actionSavePCAP'].setEnabled(False)
    app.actions['actionChoose_Calibration_File'].setEnabled(False)
    app.actions['actionCropReturns'].setEnabled(False)
    app.actions['actionShowRPM'].enabled = True


def planeFit():
    planefit.fitPlane()


def findPresetByName(name):
    presets = servermanager.vtkSMTransferFunctionPresets()

    numberOfPresets = presets.GetNumberOfPresets()

    for i in range(0,numberOfPresets):
        currentName = presets.GetPresetName(i)
        if currentName == name:
            return i

    return -1


def createDSRColorsPreset():

    dsrColorIndex = findPresetByName("DSR Colors")

    if dsrColorIndex == -1:
        rcolor = [0,        0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
                  0,         0,         0,         0,         0,         0,         0,         0,         0,    0.0625,    0.1250,    0.1875,    0.2500,    0.3125,    0.3750,
                  0.4375,    0.5000,    0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000 ,   1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,
                  0.6875,    0.6250,    0.5625,    0.5000]

        gcolor = [0,         0,         0,         0,         0,         0 ,        0,         0,    0.0625,    0.1250,    0.1875,    0.2500,    0.3125,    0.3750,    0.4375,
                  0.5000,    0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,   1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,    0.6875,
                  0.6250,    0.5625,    0.5000,    0.4375,    0.3750,    0.3125,    0.2500,    0.1875,    0.1250,    0.0625,         0,         0,         0,         0,         0,
                  0,         0,         0,         0]

        bcolor = [0.5625,    0.6250,    0.6875,    0.7500,    0.8125,    0.8750,    0.9375,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,
                  1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    1.0000,    0.9375,    0.8750,    0.8125,    0.7500,    0.6875,    0.6250,
                  0.5625,    0.5000,    0.4375,    0.3750,    0.3125,    0.2500,   0.1875,    0.1250,    0.0625,         0,         0,         0,         0,         0,         0,
                  0,         0 ,        0,         0,         0,         0,         0,         0,         0 ,        0 ,        0 ,        0,         0 ,        0 ,        0,
                  0,         0,         0,         0]

        intensityColor = [0] * 256

        for i in range(0,63):
            index = i/63.0*255.0

            intensityColor[i*4] = index
            intensityColor[i*4+1] = rcolor[i]
            intensityColor[i*4+2] = gcolor[i]
            intensityColor[i*4+3] = bcolor[i]
            i = i + 1

        presets = servermanager.vtkSMTransferFunctionPresets()

        intensityString = ',\n'.join(map(str, intensityColor))

        intensityJSON = "{\n\"ColorSpace\" : \"RGB\",\n\"Name\" : \"DSR\",\n\"NanColor\" : [ 1, 1, 0 ],\n\"RGBPoints\" : [\n"+ intensityString + "\n]\n}"

        presets.AddPreset("DSR Colors",intensityJSON)


def setDefaultLookupTables(sourceProxy):
    createDSRColorsPreset()

    presets = servermanager.vtkSMTransferFunctionPresets()

    dsrIndex = findPresetByName("DSR Colors")
    presetDSR = presets.GetPresetAsString(dsrIndex)

    # LUT for 'intensity'
    smp.GetLookupTableForArray(
      'intensity', 1,
      ScalarRangeInitialized=1.0,
      ColorSpace='HSV',
      RGBPoints=[0.0, 0.0, 0.0, 1.0,
               100.0, 1.0, 1.0, 0.0,
               256.0, 1.0, 0.0, 0.0])

    # LUT for 'laser_id'. This LUT is extracted from the XML calibration file
    # which doesn't exist in live stream mode
    if False and getReader() is not None:
        rgbRaw = [0] * 256
        sourceProxy.GetClientSideObject().GetXMLColorTable(rgbRaw)

        smp.GetLookupTableForArray(
          'laser_id', 1,
          ScalarRangeInitialized=1.0,
          ColorSpace='RGB',
          RGBPoints=rgbRaw)

def colorByIntensity(sourceProxy):

    if not hasArrayName(sourceProxy, 'intensity'):
        return False

    setDefaultLookupTables(sourceProxy)
    rep = smp.GetDisplayProperties(sourceProxy)
    rep.ColorArrayName = 'intensity'
    rep.LookupTable = smp.GetLookupTableForArray('intensity', 1)
    return True


def getTimeStamp():
    format = '%Y-%m-%d-%H-%M-%S'
    return datetime.datetime.now().strftime(format)


def getReaderFileName():
    filename = getReader().FileName
    return filename[0] if isinstance(filename, servermanager.FileNameProperty) else filename


def setDefaultSaveName(defaultSaveName):
  app.DefaultSaveName = defaultSaveName

def getDefaultSaveFileName(extension, suffix='', appendFrameNumber=False):

    sensor = getSensor()
    reader = getReader()

    if sensor:
        nchannels =  sensor.Interpreter.GetClientSideObject().GetNumberOfChannels()
        base = 'HDL-'
        if nchannels <= 16:
            base = 'VLP-'
        sensortype = base + str(nchannels)

        return '%s_Velodyne-%s-Data.%s' % (getTimeStamp(), sensortype, extension)

    if reader:
        basename =  os.path.splitext(os.path.basename(getReaderFileName()))[0]
        if appendFrameNumber:
            suffix = '%s (Frame %04d)' % (suffix, int(app.scene.AnimationTime))
        return '%s%s.%s' % (basename, suffix, extension)


def chooseCalibration(calibrationFilename=None):

    class Calibration(object):
        def __init__(self, dialog):
            self.calibrationFile = dialog.selectedCalibrationFile()
            self.gpsYaw = dialog.gpsYaw()
            self.gpsRoll = dialog.gpsRoll()
            self.gpsPitch = dialog.gpsPitch()
            self.lidarPort = dialog.lidarPort()
            self.gpsPort = dialog.gpsPort()
            self.gpsForwardingPort = dialog.gpsForwardingPort()
            self.lidarForwardingPort = dialog.lidarForwardingPort()
            self.isForwarding = dialog.isForwarding()
            self.ipAddressForwarding = dialog.ipAddressForwarding()
            self.sensorTransform = vtk.vtkTransform()
            self.gpsTransform = vtk.vtkTransform()
            self.isCrashAnalysing = dialog.isCrashAnalysing()
            self.isEnableInterpretGPSPackets = dialog.isEnableInterpretGPSPackets()

            qm = dialog.sensorTransform()
            vmLidar = vtk.vtkMatrix4x4()
            for row in range(4):
                vmLidar.SetElement(row, 0, qm.row(row).x())
                vmLidar.SetElement(row, 1, qm.row(row).y())
                vmLidar.SetElement(row, 2, qm.row(row).z())
                vmLidar.SetElement(row, 3, qm.row(row).w())
            self.sensorTransform.SetMatrix(vmLidar)

            qm = dialog.gpsTransform()
            vmGps = vtk.vtkMatrix4x4()
            for row in range(4):
                vmGps.SetElement(row, 0, qm.row(row).x())
                vmGps.SetElement(row, 1, qm.row(row).y())
                vmGps.SetElement(row, 2, qm.row(row).z())
                vmGps.SetElement(row, 3, qm.row(row).w())
            self.gpsTransform.SetMatrix(vmGps)


    dialog = vvCalibrationDialog(getMainWindow())
    if calibrationFilename is None:
        if not dialog.exec_():
            return None
        return Calibration(dialog)
    else:
        result = Calibration(dialog)
        result.calibrationFile = calibrationFilename
        return result


def UpdateApplogicLidar(lidarProxyName, gpsProxyName):

    sensor = smp.FindSource(lidarProxyName)

    app.grid = createGrid()

    sensor.Interpreter.UseIntraFiringAdjustment = app.actions['actionIntraFiringAdjust'].isChecked()
    sensor.Interpreter.IgnoreZeroDistances = app.actions['actionIgnoreZeroDistances'].isChecked()
    sensor.Interpreter.HideDropPoints = app.actions['actionHideDropPoints'].isChecked()
    sensor.Interpreter.IgnoreEmptyFrames = app.actions['actionIgnoreEmptyFrames'].isChecked()

    sensor.UpdatePipeline()

    if gpsProxyName:
        app.position = smp.FindSource(gpsProxyName)

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(sensor)

    smp.GetActiveView().ViewTime = 0.0

    app.sensor = sensor

    app.trailingFramesSpinBox.enabled = False
    app.colorByInitialized = False
    LidarPort = sensor.GetClientSideObject().GetListeningPort()
    app.filenameLabel.setText('Live sensor stream (Port:'+str(LidarPort)+')' )
    app.positionPacketInfoLabel.setText('')
    enableSaveActions()

    onCropReturns(False) # Dont show the dialog just restore settings

    rep = smp.Show(sensor)
#    rep.InterpolateScalarsBeforeMapping = 0
#    if app.sensor.GetClientSideObject().GetNumberOfChannels() == 128:
#        rep.Representation = 'Point Cloud'
#        rep.ColorArrayName = 'intensity'

    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    smp.Render()

    showSourceInSpreadSheet(sensor)

    app.actions['actionShowRPM'].enabled = True

    #Auto adjustment of the grid size with the distance resolution
    app.DistanceResolutionM = sensor.Interpreter.GetClientSideObject().GetDistanceResolutionM()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    updateUIwithNewLidar()


def openPCAP(filename, positionFilename=None, calibrationFilename=None, calibrationUIArgs=None):

    calibration = chooseCalibration(calibrationFilename)
    if not calibration:
        return

    if calibrationFilename is not None and calibrationUIArgs is not None and isinstance(calibrationUIArgs, dict):
        for k in calibrationUIArgs.keys():
          if hasattr(calibration, k):
            setattr(calibration, k, calibrationUIArgs[k])

    calibrationFile = calibration.calibrationFile
    sensorTransform = calibration.sensorTransform

    close()

    def onProgressEvent(o, e):
        PythonQt.QtGui.QApplication.instance().processEvents()

    progressDialog = QtGui.QProgressDialog('Reading packet file...', '', 0, 0, getMainWindow())
    progressDialog.setCancelButton(None)
    progressDialog.setModal(True)
    progressDialog.show()

    handler = servermanager.ActiveConnection.Session.GetProgressHandler()
    handler.PrepareProgress()
    interval = handler.GetProgressInterval()
    handler.SetProgressInterval(0.05)
    tag = handler.AddObserver('ProgressEvent', onProgressEvent)

    # construct the reader, this calls UpdateInformation on the
    # reader which scans the pcap file and emits progress events
    reader = smp.LidarReader(guiName='Data',
                             CalibrationFile = calibrationFile,
                             FileName = filename)

    # Change the default interpreter if the pcap is a Velodyne one
    if calibrationFile.endswith('.xml'):
        reader.Interpreter = 'Velodyne Meta Interpreter'

    reader.UpdatePipelineInformation()
    app.reader = reader
    app.trailingFramesSpinBox.enabled = True
    app.trailingFrame = smp.TrailingFrame(guiName="TrailingFrame", Input=getLidar(), NumberOfTrailingFrames=app.trailingFramesSpinBox.value)

    displayableFilename = os.path.basename(filename)
    # shorten the name to display because the status bar gives a lower bound to main window width
    shortDisplayableFilename = (displayableFilename[:20] + '...' + displayableFilename[-20:]) if len(displayableFilename) > 43 else displayableFilename
    app.filenameLabel.setText('File: %s' % shortDisplayableFilename)
    app.filenameLabel.setToolTip('File: %s' % displayableFilename)

    app.positionPacketInfoLabel.setText('') # will be updated later if possible
    onCropReturns(False) # Dont show the dialog just restore settings

    reader.Interpreter.GetClientSideObject().SetSensorTransform(sensorTransform)

    lidarPacketInterpreter = getLidarPacketInterpreter()

    if SAMPLE_PROCESSING_MODE:
        processor = smp.ProcessingSample(reader)

    handler.RemoveObserver(tag)
    handler.LocalCleanupPendingProgress()
    handler.SetProgressInterval(interval)
    progressDialog.close()

    smp.GetActiveView().ViewTime = 0.0

    if SAMPLE_PROCESSING_MODE:
        prep = smp.Show(processor)
    app.scene.UpdateAnimationUsingDataTimeSteps()

    if calibration.isEnableInterpretGPSPackets :
        posreader = smp.PositionOrientationReader(guiName='Position Orientation Data', FileName = filename)

        # wrapping not currently working for plugins:
        #posreader.GetClientSideObject().SetCalibrationTransform(calibration.gpsTransform)
        smp.Show(posreader)

        if positionFilename is None:
            # only VelodyneHDLReader provides this information
            # this information must be read after an update
            # GetTimeSyncInfo() has the side effect of showing a message in the error
            # console in the cases where the timeshift if computed
            # wrapping not currently working for plugins:
            # app.positionPacketInfoLabel.setText(posreader.GetClientSideObject().GetTimeSyncInfo())
            pass

        output0 = posreader.GetClientSideObject().GetOutput(0)
        if output0.GetNumberOfPoints() != 0:

            output1 = posreader.GetClientSideObject().GetOutputDataObject(1)
            trange = output1.GetColumnByName("time").GetRange()

            # Setup scalar bar
            rep = smp.GetDisplayProperties(posreader)
            rep.ColorArrayName = 'time'
            rgbPoints = [trange[0], 0.0, 0.0, 1.0,
                         trange[1], 1.0, 0.0, 0.0]
            rep.LookupTable = smp.GetLookupTableForArray('time', 1,
                                                         RGBPoints=rgbPoints,
                                                         ScalarRangeInitialized=1.0)
            sb = smp.CreateScalarBar(LookupTable=rep.LookupTable, Title='Time')
            sb.Orientation = 'Horizontal'
            app.position = posreader
            if not app.actions['actionShowPosition'].isChecked():
                smp.Hide(app.position)
        else:
            if positionFilename is not None:
                QtGui.QMessageBox.warning(getMainWindow(), 'Georeferencing data invalid',
                                          'File %s is empty or not supported' % positionFilename)
            smp.Delete(posreader)

    smp.Show(app.trailingFrame)
    smp.SetActiveView(app.mainView)

    colorByIntensity(app.trailingFrame)

    showSourceInSpreadSheet(app.trailingFrame)

    enableSaveActions()
    addRecentFile(filename)

    app.actions['actionShowRPM'].enabled = True

    #Auto adjustment of the grid size with the distance resolution
    app.DistanceResolutionM = reader.Interpreter.GetClientSideObject().GetDistanceResolutionM()
    app.grid = createGrid()
    app.actions['actionMeasurement_Grid'].setChecked(True)
    showMeasurementGrid()

    smp.SetActiveSource(app.trailingFrame)
    updateUIwithNewLidar()



def hideMeasurementGrid():
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 0
    smp.Render()


def showMeasurementGrid():
    rep = smp.GetDisplayProperties(app.grid)
    rep.Visibility = 1
    smp.Render()


# Start Functions related to ruler


def createRuler():
    pxm = servermanager.ProxyManager()
    distancerep = pxm.NewProxy('representations', 'DistanceWidgetRepresentation')
    distancerepeasy = servermanager._getPyProxy(distancerep)
    smp.GetActiveView().Representations.append(distancerepeasy)
    distancerepeasy.Visibility = False
    smp.Render()

    return distancerepeasy


def hideRuler():
    app.ruler.Visibility = False
    smp.Render()


def showRuler():
    app.ruler.Visibility = True
    smp.Render()

def getPointFromCoordinate(coord, midPlaneDistance = 0.5):
    assert len(coord) == 2

    windowHeight = smp.GetActiveView().ViewSize[1]

    displayPoint = [coord[0], windowHeight - coord[1], midPlaneDistance]
    renderer = smp.GetActiveView().GetRenderer()
    renderer.SetDisplayPoint(displayPoint)
    renderer.DisplayToWorld()
    world1 = renderer.GetWorldPoint()

    return world1[:3]

def toggleRulerContext():

    measurmentState = app.actions['actionMeasure'].isChecked()

    mW = getMainWindow()
    vtkW = mW.findChild('pqQVTKWidget')

    if measurmentState == True:
        vtkW.connect('mouseEvent(QMouseEvent*)', setRulerCoordinates)

    elif measurmentState == False:
        vtkW.disconnect('mouseEvent(QMouseEvent*)', setRulerCoordinates)

        app.mousePressed = False
        hideRuler()


def setRulerCoordinates(mouseEvent):

    pqView = smp.GetActiveView()
    rW = pqView.GetRenderWindow()
    windowInteractor = rW.GetInteractor()
    currentMouseState = mouseEvent.buttons()
    currentKeyboardState = mouseEvent.modifiers()

    if currentMouseState == 1:  #Left button pressed

        if app.mousePressed == False: #For the first time

            if currentKeyboardState == 67108864: #Control key pressed

                app.mousePressed = True
                app.ruler.Point1WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])

                windowInteractor.Disable()

        elif app.mousePressed == True: #Not for the first time

            app.ruler.Point2WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])
            showRuler()
            smp.Render()

    elif currentMouseState == 0: #Left button released

        windowInteractor.Enable()

        if  app.mousePressed == True: #For the first time

            app.mousePressed = False
            app.ruler.Point2WorldPosition = getPointFromCoordinate([mouseEvent.x(),mouseEvent.y()])
            showRuler()
            smp.Render()


# End Functions related to ruler


def rotateCSVFile(filename):

    # read the csv file, move the last 3 columns to the
    # front, and then overwrite the file with the result
    csvFile = open(filename, 'rt')
    reader = csv.reader(csvFile, quoting=csv.QUOTE_NONNUMERIC)
    rows = [row[-3:] + row[:-3] for row in reader]
    csvFile.close()

    writer = csv.writer(open(filename, 'wt'), quoting=csv.QUOTE_NONNUMERIC, delimiter=',', lineterminator = '\n')
    writer.writerows(rows)


def savePositionCSV(filename):
    w = smp.CreateWriter(filename, getPosition())
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)

def saveCSVCurrentFrame(filename):
    w = smp.CreateWriter(filename, getLidar())
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    rotateCSVFile(filename)

def saveCSVCurrentFrameSelection(filename):
    source = getReader()
    selection = source.GetSelectionOutput(0)
    extractSelection = smp.ExtractSelection(Input = source, Selection = selection.Selection)
    w = smp.CreateWriter(filename, extractSelection)
    w.Precision = 16
    w.FieldAssociation = 'Points'
    w.UpdatePipeline()
    smp.Delete(w)
    rotateCSVFile(filename)

# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLASFrames(filename, first, last, transform = 0):
    reader = getReader().GetClientSideObject()

    # Check that we have a position provider
    if getPosition() is not None:
        position = getPosition().GetClientSideObject().GetOutput()

        PythonQt.paraview.pqLidarViewManager.saveFramesToLAS(
            reader, position, first, last, filename, transform)

    else:
        PythonQt.paraview.pqLidarViewManager.saveFramesToLAS(
            reader, None, first, last, filename, transform)


# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLASCurrentFrame(filename, transform = 0):
    t = app.scene.AnimationTime
    saveLASFrames(filename, t, t, transform)


def saveAllFrames(filename, saveFunction):
    saveFunction(filename, getLidar.TimestepValues())


def saveFrameRange(filename, frameStart, frameStop, saveFunction):
    timesteps = range(frameStart, frameStop+1)
    saveFunction(filename, timesteps)


def saveCSV(filename, timesteps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    writer = smp.CreateWriter('tmp.csv', getLidar())
    writer.FieldAssociation = 'Points'
    writer.Precision = 16

    for t in timesteps:
        app.scene.AnimationTime = t
        writer.FileName = filenameTemplate % t
        writer.UpdatePipeline()
        rotateCSVFile(writer.FileName)

    smp.Delete(writer)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)

# transform parameter indicates the coordinates system and
# the referential for the exported points clouds:
# - 0 Sensor: sensor referential, cartesian coordinate system
# - 1: Relative Geoposition: NED base centered at the first position
#      of the sensor, cartesian coordinate system
# - 2: Absolute Geoposition: NED base centered at the corresponding
#      UTM zone, cartesian coordinate system
# - 3: Absolute Geoposition Lat/Lon: Lat / Lon coordinate system
def saveLAS(filename, timesteps, transform = 0):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    basenameWithoutExtension = os.path.splitext(os.path.basename(filename))[0]
    outDir = os.path.join(tempDir, basenameWithoutExtension)
    filenameTemplate = os.path.join(outDir, basenameWithoutExtension + ' (Frame %04d).csv')
    os.makedirs(outDir)

    for t in sorted(timesteps):
        saveLASFrames(filenameTemplate % t, t, t, transform)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)


def getTimeStamp():
    format = '%Y-%m-%d-%H-%M-%S'
    return datetime.datetime.now().strftime(format)


def getSaveFileName(title, extension, defaultFileName=None):

    settings = getPVSettings()
    defaultDir = settings.value('LidarPlugin/OpenData/DefaultDir', QtCore.QDir.homePath())
    defaultFileName = defaultDir if not defaultFileName else os.path.join(defaultDir, defaultFileName)

    nativeDialog = 0 if app.actions['actionNative_File_Dialogs'].isChecked() else QtGui.QFileDialog.DontUseNativeDialog

    filters = '%s (*.%s)' % (extension, extension)
    selectedFilter = '%s (*.%s)' % (extension, extension)
    fileName = QtGui.QFileDialog.getSaveFileName(getMainWindow(), title,
                        defaultFileName, filters, selectedFilter, nativeDialog)

    if fileName:
        settings.setValue('LidarPlugin/OpenData/DefaultDir', QtCore.QFileInfo(fileName).absoluteDir().absolutePath())
        return fileName


def restoreNativeFileDialogsAction():
    settings = getPVSettings()
    app.actions['actionNative_File_Dialogs'].setChecked(int(settings.value('LidarPlugin/NativeFileDialogs', 1)))


def onNativeFileDialogsAction():
    settings = getPVSettings()
    defaultDir = settings.setValue('LidarPlugin/NativeFileDialogs', int(app.actions['actionNative_File_Dialogs'].isChecked()))


def getFrameSelectionFromUser(frameStrideVisibility=False, framePackVisibility=False, frameTransformVisibility=False):
    class FrameOptions(object):
        pass

    dialog = PythonQt.paraview.vvSelectFramesDialog(getMainWindow())
    dialog.frameMinimum = 0
    if app.reader is None:
        dialog.frameMaximum = 0
    elif app.reader.GetClientSideObject().GetShowFirstAndLastFrame():
        dialog.frameMaximum = app.reader.GetClientSideObject().GetNumberOfFrames() - 1
    else:
        dialog.frameMaximum = app.reader.GetClientSideObject().GetNumberOfFrames() - 3
    dialog.frameStrideVisibility = frameStrideVisibility
    dialog.framePackVisibility = framePackVisibility
    dialog.frameTransformVisibility = frameTransformVisibility
    dialog.restoreState()

    if not dialog.exec_():
        return None

    frameOptions = FrameOptions()
    frameOptions.mode = dialog.frameMode
    frameOptions.start = dialog.frameStart
    frameOptions.stop = dialog.frameStop
    frameOptions.stride = dialog.frameStride
    frameOptions.pack = dialog.framePack
    frameOptions.transform = dialog.frameTransform

    dialog.setParent(None)

    return frameOptions


def onSaveCSV():

    frameOptions = getFrameSelectionFromUser()
    if frameOptions is None:
        return


    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', appendFrameNumber=True))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            saveCSVCurrentFrame(fileName)

            setTransformMode(oldTransform)

    else:
        # It is not possible to save several frames as CSV during stream
        if getSensor():
            QtGui.QMessageBox.information(getMainWindow(),
                                        'Save several frames as CSV is not available during stream',
                                        'Please use the "Record" tool, and open the resulting pcap offline to process it.')
            return

        fileName = getSaveFileName('Save CSV (to zip file)', 'zip', getDefaultSaveFileName('zip'))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            if frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
                saveAllFrames(fileName, saveCSV)
            else:
                start = frameOptions.start
                stop = frameOptions.stop
                saveFrameRange(fileName, start, stop, saveCSV)

            setTransformMode(oldTransform)


def onSavePosition():
    fileName = getSaveFileName('Save CSV', 'csv', getDefaultSaveFileName('csv', '-position'))
    if fileName:
        savePositionCSV(fileName)


def onSaveLAS():

    frameOptions = getFrameSelectionFromUser(framePackVisibility=True, frameTransformVisibility=False)
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameOptions.start = frameOptions.stop = app.scene.AnimationTime
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        frameOptions.start = int(app.scene.StartTime)
        frameOptions.stop = int(app.scene.EndTime)

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        fileName = getSaveFileName('Save LAS', 'las', getDefaultSaveFileName('las', appendFrameNumber=True))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            saveLASCurrentFrame(fileName, frameOptions.transform)

            setTransformMode(oldTransform)

    elif frameOptions.pack == vvSelectFramesDialog.FILE_PER_FRAME:
        fileName = getSaveFileName('Save LAS (to zip file)', 'zip',
                                   getDefaultSaveFileName('zip'))
        if fileName:
            oldTransform = transformMode()
            setTransformMode(1 if frameOptions.transform else 0)

            def saveTransformedLAS(filename, timesteps):
                saveLAS(filename, timesteps, frameOptions.transform)

            if frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
                saveAllFrames(fileName, saveTransformedLAS)
            else:
                start = frameOptions.start
                stop = frameOptions.stop
                saveFrameRange(fileName, start, stop, saveTransformedLAS)

            setTransformMode(oldTransform)

    else:
        suffix = ' (Frame %d to %d)' % (frameOptions.start, frameOptions.stop)
        defaultFileName = getDefaultSaveFileName('las', suffix=suffix)
        fileName = getSaveFileName('Save LAS', 'las', defaultFileName)
        if not fileName:
            return

        oldTransform = transformMode()
        setTransformMode(1 if frameOptions.transform else 0)

        saveLASFrames(fileName, frameOptions.start, frameOptions.stop,
                      frameOptions.transform)

        setTransformMode(oldTransform)


def onSavePCAP():

    frameOptions = getFrameSelectionFromUser(frameTransformVisibility=False)
    if frameOptions is None:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        frameOptions.start = frameOptions.stop = bisect.bisect_left(
          getAnimationScene().TimeKeeper.TimestepValues,
          getAnimationScene().TimeKeeper.Time)
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        frameOptions.start = 0
        frameOptions.stop = 0 if app.reader is None else app.reader.GetClientSideObject().GetNumberOfFrames() - 1

    defaultFileName = getDefaultSaveFileName('pcap', suffix=' (Frame %d to %d)' % (frameOptions.start, frameOptions.stop))
    fileName = getSaveFileName('Save PCAP', 'pcap', defaultFileName)
    if not fileName:
        return

    PythonQt.paraview.pqLidarViewManager.saveFramesToPCAP(getReader().SMProxy, frameOptions.start, frameOptions.stop, fileName)


def onSaveScreenshot():

    fileName = getSaveFileName('Save Screenshot', 'png', getDefaultSaveFileName('png', appendFrameNumber=True))
    if fileName:
        if fileName[-4:] != ".png":
            fileName += ".png"
        saveScreenshot(fileName)


def onKiwiViewerExport():

    frameOptions = getFrameSelectionFromUser(frameStrideVisibility=True,
                                             frameTransformVisibility=False)
    if frameOptions is None:
        return

    defaultFileName = getDefaultSaveFileName('zip', suffix=' (KiwiViewer)')
    fileName = getSaveFileName('Export To KiwiViewer', 'zip', defaultFileName)
    if not fileName:
        return

    if frameOptions.mode == vvSelectFramesDialog.CURRENT_FRAME:
        timesteps = [app.scene.AnimationTime]
    elif frameOptions.mode == vvSelectFramesDialog.ALL_FRAMES:
        timesteps = range(int(app.scene.StartTime), int(app.scene.EndTime) + 1, frameOptions.stride)
    else:
        timesteps = range(frameOptions.start, frameOptions.stop+1, frameOptions.stride)

    saveToKiwiViewer(fileName, timesteps)


def saveToKiwiViewer(filename, timesteps):

    tempDir = kiwiviewerExporter.tempfile.mkdtemp()
    outDir = os.path.join(tempDir, os.path.splitext(os.path.basename(filename))[0])

    os.makedirs(outDir)

    filenames = exportToDirectory(outDir, timesteps)

    kiwiviewerExporter.writeJsonData(outDir, smp.GetActiveView(), smp.GetDisplayProperties(), filenames)

    kiwiviewerExporter.zipDir(outDir, filename)
    kiwiviewerExporter.shutil.rmtree(tempDir)


def exportToDirectory(outDir, timesteps):

    filenames = []

    alg = smp.GetActiveSource().GetClientSideObject()

    writer = vtkXMLPolyDataWriter()
    writer.SetDataModeToAppended()
    writer.EncodeAppendedDataOff()
    writer.SetCompressorTypeToZLib()

    for t in timesteps:

        filename = 'frame_%04d.vtp' % t
        filenames.append(filename)

        app.scene.AnimationTime = t
        polyData = vtk.vtkPolyData()
        polyData.ShallowCopy(alg.GetOutput())

        writer.SetInputData(polyData)
        writer.SetFileName(os.path.join(outDir, filename))
        writer.Update()

    return filenames


def getVersionString():
  return " ".join(getMainWindow().windowTitle.split(" ")[1:])


def onAbout():
    aboutDialog.showDialog(getMainWindow())


def close():
    # Save grid properties for this session
    app.gridProperties.Normal = app.grid.Normal
    app.gridProperties.Origin = app.grid.Origin
    app.gridProperties.Scale = app.grid.Scale
    app.gridProperties.GridNbTicks = app.grid.GridNbTicks
    app.gridProperties.LineWidth = app.grid.LineWidth
    app.gridProperties.Color = app.grid.Color

    smp.GetAnimationScene().Stop()
    hideRuler()
    unloadData()
    app.scene.AnimationTime = 0
    app.reader = None
    app.sensor = None
    app.trailingFrame = None
    smp.Delete(app.grid)

    smp.HideUnusedScalarBars()

    resetCameraToForwardView()
    app.filenameLabel.setText('')
    app.statusLabel.setText('')
    disableSaveActions()


def _setSaveActionsEnabled(enabled):
    for action in ('SaveCSV', 'SavePCAP', 'SaveLAS', 'Export_To_KiwiViewer',
                   'Close', 'Choose_Calibration_File', 'CropReturns'):
        app.actions['action'+action].setEnabled(enabled)
    getMainWindow().findChild('QMenu', 'menuSaveAs').enabled = enabled


def enableSaveActions():
    _setSaveActionsEnabled(True)
    if getPosition():
        app.actions['actionSavePositionCSV'].setEnabled(True)


def disableSaveActions():
    _setSaveActionsEnabled(False)
    app.actions['actionSavePositionCSV'].setEnabled(False)


def startStream():

    sensor = getSensor()
    if sensor:
        sensor.Start()


def stopStream():
    sensor = getSensor()
    if sensor:
        sensor.Stop()


def getPointCloudData(attribute=None):

    if attribute is not None:
        data = getPointCloudData()
        if data:
            if attribute == 'points':
                return data.GetPoints().GetData()
            else:
                return data.GetPointData().GetArray(attribute)
    else:
        source = getSensor() or getReader()
        if source:
            return source.GetClientSideObject().GetOutput()


def getNumberOfTimesteps():
    return getTimeKeeper().getNumberOfTimeStepValues()


def unloadData():
    _repCache.clear()

    for k, src in smp.GetSources().items():
        if src != app.grid and src != smp.FindSource("RPM"):
            smp.Delete(src)

    app.reader = None
    app.trailingFrame = None
    app.position = None
    app.sensor = None

    clearSpreadSheetView()

def getReader():
    return getattr(app, 'reader', None)

def getSensor():
    return getattr(app, 'sensor', None)

def getLidar():
    return getReader() or getSensor()

def getLidarPacketInterpreter():
    lidar = getLidar()
    if lidar:
      return lidar.Interpreter
    return None

def getPosition():
    return getattr(app, 'position', None)

def onChooseCalibrationFile():

    calibration = chooseCalibration()
    if not calibration:
        return

    calibrationFile = calibration.calibrationFile
    sensorTransform = calibration.sensorTransform

    lidar = getLidar()
    if lidar:
        lidar.Interpreter.GetClientSideObject().SetSensorTransform(sensorTransform)
        lidar.CalibrationFile = calibrationFile
        updateUIwithNewLidar()


def onCropReturns(show = True):
    dialog = vvCropReturnsDialog(getMainWindow())

    cropEnabled = False
    cropOutside = False
    firstCorner = QtGui.QVector3D()
    secondCorner = QtGui.QVector3D()

    lidarInterpreter = getLidarPacketInterpreter()

    # Retrieve current values to fill the UI
    if lidarInterpreter:
        cropEnabled = lidarInterpreter.CropMode != 'None'
        cropOutside = lidarInterpreter.CropOutside
        firstCorner = QtGui.QVector3D(lidarInterpreter.CropRegion[0], lidarInterpreter.CropRegion[2], lidarInterpreter.CropRegion[4])
        secondCorner = QtGui.QVector3D(lidarInterpreter.CropRegion[1], lidarInterpreter.CropRegion[3], lidarInterpreter.CropRegion[5])

    #show the dialog box
    if show:
        dialog.cropOutside = cropOutside
        dialog.firstCorner = firstCorner
        dialog.secondCorner = secondCorner
        dialog.croppingEnabled = cropEnabled
        # Enforce the call to dialog.croppingEnabled."onChanged" even if dialog.croppingEnabled == cropEnabled
        dialog.croppingEnabled = not dialog.croppingEnabled
        dialog.croppingEnabled = not dialog.croppingEnabled

        # update the dialog configuration
        dialog.UpdateDialogWithCurrentSetting()

        if not dialog.exec_():
            return

    if lidarInterpreter:
        lidarInterpreter.CropOutside = dialog.cropOutside
        dialogCropMode = ['None', 'Cartesian', 'Spherical']
        lidarInterpreter.CropMode = dialogCropMode[dialog.GetCropMode()]
        p1 = dialog.firstCorner
        p2 = dialog.secondCorner
        lidarInterpreter.CropRegion = [p1.x(), p2.x(), p1.y(), p2.y(), p1.z(), p2.z()]
        if show:
            smp.Render()


def resetCameraToBirdsEyeView(view=None):

    view = view or smp.GetActiveView()
    view.CameraFocalPoint = [0, 0, 0]
    view.CameraViewUp = [0, 1, 0]
    view.CameraPosition = [0, 0, 40]
    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def resetCameraToForwardView(view=None):

    view = view or smp.GetActiveView()
    view.CameraFocalPoint = [0,0,0]
    view.CameraViewUp = [0, 0.27, 0.96]
    view.CameraPosition = [0, -72, 18.0]
    view.CenterOfRotation = [0, 0, 0]
    smp.Render(view)


def saveScreenshot(filename):
    smp.WriteImage(filename)

    # reload the saved screenshot as a pixmap
    screenshot = QtGui.QPixmap()
    screenshot.load(filename)

    # create a new pixmap with the status bar widget painted at the bottom
    statusBar = QtGui.QWidget.grab(getMainWindow().statusBar())
    composite = QtGui.QPixmap(screenshot.width(), screenshot.height() + statusBar.height())
    painter = QtGui.QPainter()
    painter.begin(composite)
    painter.drawPixmap(screenshot.rect(), screenshot, screenshot.rect())
    painter.drawPixmap(statusBar.rect().translated(0, screenshot.height()), statusBar, statusBar.rect())
    painter.end()

    # save final screenshot
    composite.save(filename)


def getSpreadSheetViewProxy():
    return smp.servermanager.ProxyManager().GetProxy("views", "main spreadsheet view")

def clearSpreadSheetView():
    view = getSpreadSheetViewProxy()
    if view is not None:
        view.Representations = []


def showSourceInSpreadSheet(source):

    spreadSheetView = getSpreadSheetViewProxy()
    smp.Show(source, spreadSheetView)

    # Work around a bug where the 'Showing' combobox doesn't update.
    # Calling hide and show again will trigger the refresh.
    smp.Hide(source, spreadSheetView)
    smp.Show(source, spreadSheetView)


def createGrid(view=None):

    view = view or smp.GetActiveView()
    grid = smp.GridSource(guiName='Measurement Grid')

    if app.gridProperties.Persist == False:
        grid.GridNbTicks = (int(math.ceil(50000 * app.DistanceResolutionM/ grid.Scale )))
    else:
        # Restore grid properties
        grid.Normal = app.gridProperties.Normal
        grid.Origin = app.gridProperties.Origin
        grid.Scale = app.gridProperties.Scale
        grid.GridNbTicks = app.gridProperties.GridNbTicks
        grid.LineWidth = app.gridProperties.LineWidth
        grid.Color = app.gridProperties.Color

    rep = smp.Show(grid, view)
    rep.LineWidth = grid.LineWidth
    rep.DiffuseColor = grid.Color
    rep.Pickable = 0
    rep.Visibility = 0
    smp.SetActiveSource(None)
    return grid


def hideGrid():
    smp.GetDisplayProperties(app.grid).Hide()


def showGrid():
    smp.GetDisplayProperties(app.grid).Show()


def getAnimationScene():
    '''This function is a workaround because paraview.simple.GetAnimationScene()
    has an issue where the returned proxy might not have its Cues property initialized'''
    for proxy in servermanager.ProxyManager().GetProxiesInGroup("animation").values():
        if proxy.GetXMLName() == 'AnimationScene' and len(proxy.Cues):
            return proxy


def start():

    global app
    app = AppLogic()
    app.scene = getAnimationScene()
    app.gridProperties = GridProperties()

    view = smp.GetActiveView()
    view.Background = [0.0, 0.0, 0.0]
    view.Background2 = [0.0, 0.0, 0.2]
    view.UseGradientBackground = True
    smp._DisableFirstRenderCameraReset()
    smp.GetActiveView().LODThreshold = 1e100
    app.DistanceResolutionM = 0.002
    app.grid = createGrid()
    app.ruler = createRuler()

    resetCameraToForwardView()

    setupActions()
    disableSaveActions()
    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    setupStatusBar()
    hideColorByComponent()
    restoreNativeFileDialogsAction()
    updateRecentFiles()
    createRPMBehaviour()


def findQObjectByName(widgets, name):
    for w in widgets:
        if w.objectName == name:
            return w


def getMainWindow():
    return findQObjectByName(QtGui.QApplication.topLevelWidgets(), 'vvMainWindow')


def getPVApplicationCore():
    return PythonQt.paraview.pqPVApplicationCore.instance()


def getPVSettings():
    return getPVApplicationCore().settings()


def getTimeKeeper():
    return getPVApplicationCore().getActiveServer().getTimeKeeper()


def quit():
    PythonQt.QtGui.QApplication.instance().quit()
exit = quit


def addShortcuts(keySequenceStr, function):
    shortcut = PythonQt.QtGui.QShortcut(PythonQt.QtGui.QKeySequence(keySequenceStr), getMainWindow())
    shortcut.connect("activated()", function)


def onTrailingFramesChanged(numFrames):
    tr = smp.FindSource("TrailingFrame")
    if tr:
        tr.NumberOfTrailingFrames = numFrames
        smp.Render()


def setupStatusBar():
    # by using a QScrollArea inside the statusBar it should be possible
    # to reduce the minimum main window's width

    statusBar = getMainWindow().statusBar()
    statusBar.addPermanentWidget(app.logoLabel)
    statusBar.addWidget(app.filenameLabel)
    statusBar.addWidget(app.statusLabel)
    statusBar.addWidget(app.sensorInformationLabel)
    statusBar.addWidget(app.positionPacketInfoLabel)


def onGridProperties():
    if gridAdjustmentDialog.showDialog(getMainWindow(), app.grid, app.gridProperties):
        rep = smp.Show(app.grid, None)
        rep.LineWidth = app.grid.LineWidth
        rep.DiffuseColor = app.grid.Color
        app.actions['actionMeasurement_Grid'].setChecked(True)
        smp.Render()


def hideColorByComponent():
    getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[1].hide()


def adjustScalarBarRangeLabelFormat():
    if not app.actions['actionScalarBarVisibility'].isChecked():
        return

    arrayName = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChild('QComboBox').currentText
    if arrayName != '' and hasArrayName(app.reader, arrayName):
        sb = smp.GetScalarBar(smp.GetLookupTableForArray(arrayName, []))
        sb.RangeLabelFormat = '%g'
        smp.Render()



def addRecentFile(filename):
    maxRecentFiles = 4
    recentFiles = getRecentFiles()

    # workaround to get system-locale to unicode conversion of filename
    recentFiles.insert(0, filename)
    getPVSettings().setValue('LidarPlugin/RecentFiles', recentFiles)
    unicodeFilename = getPVSettings().value('LidarPlugin/RecentFiles')[0]
    recentFiles = recentFiles[1:]

    try:
        recentFiles.remove(unicodeFilename)
    except ValueError:
        pass

    recentFiles = recentFiles[:maxRecentFiles]
    recentFiles.insert(0, unicodeFilename)

    getPVSettings().setValue('LidarPlugin/RecentFiles', recentFiles)

    updateRecentFiles()


def openRecentFile(filename):
    if not os.path.isfile(filename):
        QtGui.QMessageBox.warning(getMainWindow(), 'File not found', 'File not found: %s' % filename)
        return

    if os.path.splitext(filename)[1].lower() == '.pcap':
        openPCAP(filename)
    else:
        openData(filename)


def getRecentFiles():
    return list(getPVSettings().value('LidarPlugin/RecentFiles', []) or [])


def updateRecentFiles():
    settings = getPVSettings()
    recentFiles = getRecentFiles()
    recentFilesMenu = findQObjectByName(findQObjectByName(getMainWindow().menuBar().children(), 'menu_File').children(), 'menuRecent_Files')

    clearMenuAction = app.actions['actionClear_Menu']
    for action in recentFilesMenu.actions()[:-2]:
        recentFilesMenu.removeAction(action)

    def createActionFunction(filename):
        def f():
            openRecentFile(filename)
        return f

    actions = []
    for filename in recentFiles:
        actions.append(QtGui.QAction(os.path.basename(filename), recentFilesMenu))
        actions[-1].connect('triggered()', createActionFunction(filename))
    recentFilesMenu.insertActions(recentFilesMenu.actions()[0], actions)


def onClearMenu():
    settings = getPVSettings()
    settings.setValue('LidarPlugin/RecentFiles', [])
    updateRecentFiles()

def toggleProjectionType():

    view = smp.GetActiveView()

    view.CameraParallelProjection = not view.CameraParallelProjection
    if app.actions['actionMeasure'].isChecked():
        app.actions['actionMeasure'].trigger()
        app.actions['actionMeasure'].toggle()

    app.actions['actionMeasure'].setEnabled(view.CameraParallelProjection)
    if not view.CameraParallelProjection:
        app.actions['actionMeasure'].setChecked(False)

    smp.Render()

def toggleRPM():
    rpm = smp.FindSource("RPM")
    if rpm:
        if app.actions['actionShowRPM'].isChecked():
            smp.Show(rpm)
        else:
            smp.Hide(rpm)
        smp.Render()


def toggleCrashAnalysis():
    app.EnableCrashAnalysis = app.actions['actionEnableCrashAnalysis'].isChecked()


def transformMode():
    reader = getReader()
    if not reader:
        return None

    if reader.Interpreter.ApplyTransform:
        if app.relativeTransform:
            return 2 # relative
        else:
            return 1 # absolute
    return 0 # raw

def setTransformMode(mode):
    # 0 - raw
    # 1 - absolute
    # 2 - relative
    reader = getReader()

    if reader:
        reader.Interpreter.ApplyTransform = (mode > 0)
    app.transformMode = mode
    app.relativeTransform = (mode == 2)

def geolocationChanged(setting):
    setTransformMode(setting)
    smp.Render(view=app.mainView)

def onToogleAdvancedGUI(updateSettings = True):
  """ Switch the GUI between advanced and classic mode"""
  # hide/show Sources menu
  menuSources = getMainWindow().findChild("QMenu", "menuSources").menuAction()
  menuSources.visible = not menuSources.visible
  # hide/show Filters menu
  menuFilters = getMainWindow().findChild("QMenu", "menuFilters").menuAction()
  menuFilters.visible = not menuFilters.visible
  # hide/show Advance menu
  menuAdvance = getMainWindow().findChild("QMenu", "menuAdvance").menuAction()
  menuAdvance.visible = not menuAdvance.visible
  # hide/show view decorator
  getMainWindow().centralWidget().toggleWidgetDecoration()
  # update the UserSettings
  if updateSettings:
    # booleans must be store as int
    newValue = int(not int(getPVSettings().value("LidarPlugin/AdvanceFeature/Enable", 0)))
    getPVSettings().setValue("LidarPlugin/AdvanceFeature/Enable", newValue)

def switchVisibility(Proxy):
    """ Invert the Proxy visibility int the current view """
    ProxyRep = smp.GetRepresentation(Proxy)
    ProxyRep.Visibility = not ProxyRep.Visibility

def ShowPosition():
    if app.position:
        switchVisibility(app.position)
        smp.Render()


def setupActions():

    mW = getMainWindow()
    actions = mW.findChildren('QAction')

    app.actions = {}

    for a in actions:
        app.actions[a.objectName] = a

    app.actions['actionAdvanceFeature'].connect('triggered()', onToogleAdvancedGUI)

    app.actions['actionPlaneFit'].connect('triggered()', planeFit)

    app.actions['actionClose'].connect('triggered()', close)
    app.actions['actionSaveCSV'].connect('triggered()', onSaveCSV)
    app.actions['actionSavePositionCSV'].connect('triggered()', onSavePosition)
    app.actions['actionSaveLAS'].connect('triggered()', onSaveLAS)
    app.actions['actionSavePCAP'].connect('triggered()', onSavePCAP)
    app.actions['actionSaveScreenshot'].connect('triggered()', onSaveScreenshot)
    app.actions['actionExport_To_KiwiViewer'].connect('triggered()', onKiwiViewerExport)
    app.actions['actionGrid_Properties'].connect('triggered()', onGridProperties)
    app.actions['actionCropReturns'].connect('triggered()', onCropReturns)
    app.actions['actionNative_File_Dialogs'].connect('triggered()', onNativeFileDialogsAction)
    app.actions['actionAbout_LidarView'].connect('triggered()', onAbout)
    app.actions['actionClear_Menu'].connect('triggered()', onClearMenu)

    app.actions['actionToggleProjection'].connect('triggered()', toggleProjectionType)
    app.actions['actionMeasure'].connect('triggered()', toggleRulerContext)
    app.actions['actionShowPosition'].connect('triggered()', ShowPosition)

    app.actions['actionShowRPM'].connect('triggered()', toggleRPM)

    # Restore action states from settings
    settings = getPVSettings()


    advanceMode = int(settings.value("LidarPlugin/AdvanceFeature/Enable", 0))
    if not advanceMode:
      app.actions['actionAdvanceFeature'].checked = False
      onToogleAdvancedGUI(updateSettings=False)

    # Setup and add the geolocation toolbar
    geolocationToolBar = mW.findChild('QToolBar', 'geolocationToolbar')

    # Creating and adding the geolocation label to the geolocation toolbar
    geolocationLabel = QtGui.QLabel('Frame Mapping: ')
    geolocationToolBar.addWidget(geolocationLabel)

    # Creating the geolocation combobox
    geolocationComboBox = QtGui.QComboBox()

    #Adding the different entries
    geolocationComboBox.addItem('None (RAW Data)')
    geolocationComboBox.setItemData(0, "No mapping: Each frame is at the origin", 3)

    geolocationComboBox.addItem('Absolute Geolocation')
    geolocationComboBox.setItemData(1, "Use GPS geolocation to get each frame absolute location, the first frame is shown at origin", 3)

    geolocationComboBox.addItem('Relative Geolocation')
    geolocationComboBox.setItemData(2, "Use GPS geolocation to get each frame absolute location, the current frame is shown at origin", 3)

    geolocationComboBox.connect('currentIndexChanged(int)', geolocationChanged)
    geolocationToolBar.addWidget(geolocationComboBox)

    # Set default toolbar visibility
    geolocationToolBar.visible = False
    app.geolocationToolBar = geolocationToolBar

    # Setup and add the playback speed control toolbar
    timeToolBar = mW.findChild('QToolBar','Player Control')

    # Place the record button at the right place
    timeToolBar.addAction(app.actions['actionRecord'])

    spinBoxLabel = QtGui.QLabel('TF:')
    spinBoxLabel.toolTip = "Number of trailing frames"
    timeToolBar.addWidget(spinBoxLabel)

    spinBox = QtGui.QSpinBox()
    spinBox.toolTip = "Number of trailing frames"
    spinBox.setMinimum(0)
    spinBox.setMaximum(100)
    spinBox.connect('valueChanged(int)', onTrailingFramesChanged)
    app.trailingFramesSpinBox = spinBox

    app.actions['actionTrailingFramesSelector'] = timeToolBar.addWidget(spinBox)
    app.actions['actionTrailingFramesSelector'].setVisible(True)

    displayWidget = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget')
    displayWidget.connect('arraySelectionChanged ()',adjustScalarBarRangeLabelFormat)
    app.actions['actionScalarBarVisibility'].connect('triggered()',adjustScalarBarRangeLabelFormat)

    app.MainToolbar = getMainWindow().findChild('QToolBar','toolBar')
    app.ColorToolbar = getMainWindow().findChild('QToolBar','colorToolBar')
    app.PlaybackToolbar = timeToolBar
    app.GeolocationToolbar = getMainWindow().findChild('QToolBar','geolocationToolbar')


def createRPMBehaviour():
    # create and customize a label to display the rpm
    rpm = smp.Text(guiName="RPM", Text="No RPM")
    representation = smp.GetRepresentation(rpm)
    representation.FontSize = 8
    representation.Color = [1,1,0]
    # create an python animation cue to update the rpm value in the label
    PythonAnimationCue1 = smp.PythonAnimationCue()
    PythonAnimationCue1.Script= """
import paraview.simple as smp
def start_cue(self):
    pass

def tick(self):
    rpm = smp.FindSource("RPM")
    lidar = smp.FindSource("Data")
    if (lidar):
        value = int(lidar.Interpreter.GetClientSideObject().GetFrequency())
        rpm.Text = str(value) + " RPM"
    else:
        rpm.Text = "No RPM"

def end_cue(self):
    pass
"""
    smp.GetAnimationScene().Cues.append(PythonAnimationCue1)
    # force to be consistant with the UI
    toggleRPM()


def updateUIwithNewLidar():
    lidar = getLidar()
    if lidar:
        app.sensorInformationLabel.setText(lidar.GetClientSideObject().GetSensorInformation())
    #Remove some array to display
    ComboBox = getMainWindow().findChild('lqColorToolbar').findChild('pqDisplayColorWidget').findChildren('QComboBox')[0]
    listOfArrayToRemove = ['RotationPerMinute', 'vtkBlockColors', 'vtkCompositeIndex']
    for arrayName in listOfArrayToRemove:
        n = ComboBox.findText(arrayName)
        ComboBox.removeItem(n)

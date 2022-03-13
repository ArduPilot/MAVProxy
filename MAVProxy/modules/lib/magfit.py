#!/usr/bin/env python

'''
fit best estimate of magnetometer offsets, diagonals, off-diagonals, cmot and scaling using WMM target
'''

from MAVProxy.modules.lib import wx_processguard
from MAVProxy.modules.lib.wx_loader import wx

import sys, time, os, math, copy

from pymavlink import mavutil
from pymavlink import mavextra
from pymavlink.rotmat import Vector3
from pymavlink.rotmat import Matrix3
from MAVProxy.modules.lib import grapher
from MAVProxy.modules.lib.multiproc_util import MPDataLogChildTask

import matplotlib
import matplotlib.pyplot as pyplot
import numpy
import datetime

earth_field = None
declination = None
margs = None
mag_idx = ''

class Correction:
    def __init__(self):
        self.offsets = Vector3(0.0, 0.0, 0.0)
        self.diag = Vector3(1.0, 1.0, 1.0)
        self.offdiag = Vector3(0.0, 0.0, 0.0)
        self.cmot = Vector3(0.0, 0.0, 0.0)
        self.scaling = 1.0

    def show_parms(self):
        print("COMPASS_OFS%s_X %d" % (mag_idx, int(self.offsets.x)))
        print("COMPASS_OFS%s_Y %d" % (mag_idx, int(self.offsets.y)))
        print("COMPASS_OFS%s_Z %d" % (mag_idx, int(self.offsets.z)))
        print("COMPASS_DIA%s_X %.3f" % (mag_idx, self.diag.x))
        print("COMPASS_DIA%s_Y %.3f" % (mag_idx, self.diag.y))
        print("COMPASS_DIA%s_Z %.3f" % (mag_idx, self.diag.z))
        print("COMPASS_ODI%s_X %.3f" % (mag_idx, self.offdiag.x))
        print("COMPASS_ODI%s_Y %.3f" % (mag_idx, self.offdiag.y))
        print("COMPASS_ODI%s_Z %.3f" % (mag_idx, self.offdiag.z))
        print("COMPASS_MOT%s_X %.3f" % (mag_idx, self.cmot.x))
        print("COMPASS_MOT%s_Y %.3f" % (mag_idx, self.cmot.y))
        print("COMPASS_MOT%s_Z %.3f" % (mag_idx, self.cmot.z))
        print("COMPASS_SCALE%s %.2f" % (mag_idx, self.scaling))
        if margs['CMOT']:
            print("COMPASS_MOTCT 2")

def correct(MAG, BAT, c):
    '''correct a mag sample, returning a Vector3'''
    mag = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)

    # add the given offsets
    mag += c.offsets

    # multiply by scale factor
    mag *= c.scaling

    # apply elliptical corrections
    mat = Matrix3(
        Vector3(c.diag.x,    c.offdiag.x,  c.offdiag.y),
        Vector3(c.offdiag.x, c.diag.y,     c.offdiag.z),
        Vector3(c.offdiag.y, c.offdiag.z,  c.diag.z))

    mag = mat * mag

    # apply compassmot corrections
    if BAT is not None and hasattr(BAT, 'Curr') and not math.isnan(BAT.Curr):
        mag += c.cmot * BAT.Curr

    return mag

def get_yaw(ATT,MAG,BAT,c):
    '''calculate heading from raw magnetometer and new offsets'''

    mag = correct(MAG, BAT, c)

    # go via a DCM matrix to match the APM calculation
    dcm_matrix = mavextra.rotation_df(ATT)
    cos_pitch_sq = 1.0-(dcm_matrix.c.x*dcm_matrix.c.x)
    headY = mag.y * dcm_matrix.c.z - mag.z * dcm_matrix.c.y
    headX = mag.x * cos_pitch_sq - dcm_matrix.c.x * (mag.y * dcm_matrix.c.y + mag.z * dcm_matrix.c.z)

    global declination
    yaw = math.degrees(math.atan2(-headY,headX)) + declination
    if yaw < 0:
        yaw += 360
    return yaw

def expected_field(ATT, yaw):
    '''return expected magnetic field for attitude'''
    global earth_field

    roll = ATT.Roll
    pitch = ATT.Pitch

    rot = Matrix3()
    rot.from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))

    field = rot.transposed() * earth_field

    return field

data = None
old_corrections = Correction()

def wmm_error(p):
    '''world magnetic model error with correction fit'''
    p = list(p)
    c = copy.copy(old_corrections)

    c.offsets = Vector3(p.pop(0), p.pop(0), p.pop(0))
    c.scaling = p.pop(0)
    if margs['Elliptical']:
        c.diag = Vector3(p.pop(0), p.pop(0), p.pop(0))
        c.offdiag = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.diag = Vector3(1.0, 1.0, 1.0)
        c.offdiag = Vector3(0.0, 0.0, 0.0)

    if margs['CMOT']:
        c.cmot = Vector3(p.pop(0), p.pop(0), p.pop(0))

    ret = 0

    for (MAG,ATT,BAT) in data:
        yaw = get_yaw(ATT,MAG,BAT,c)
        expected = expected_field(ATT, yaw)
        observed = correct(MAG,BAT,c)

        error = (expected - observed).length()
        ret += error

    ret /= len(data)

    return ret

def fit_WWW():
    from scipy import optimize

    c = copy.copy(old_corrections)
    p = [c.offsets.x, c.offsets.y, c.offsets.z, c.scaling]
    if margs['Elliptical']:
        p.extend([c.diag.x, c.diag.y, c.diag.z, c.offdiag.x, c.offdiag.y, c.offdiag.z])
    if margs['CMOT']:
        p.extend([c.cmot.x, c.cmot.y, c.cmot.z])

    ofs = margs['OffsetMax']
    min_scale = margs['ScaleMin']
    max_scale = margs['ScaleMax']
    min_scale_delta = 0.00001
    bounds = [(-ofs,ofs),(-ofs,ofs),(-ofs,ofs),(min_scale,max(min_scale+min_scale_delta,max_scale))]
    if margs['CMOT NoChange']:
        bounds[0] = (c.offsets.x, c.offsets.x)
        bounds[1] = (c.offsets.y, c.offsets.y)
        bounds[2] = (c.offsets.z, c.offsets.z)

    if margs['Elliptical']:
        min_diag = margs['DiagonalMin']
        max_diag = margs['DiagonalMax']
        min_offdiag = margs['OffDiagMin']
        max_offdiag = margs['OffDiagMax']
        for i in range(3):
            bounds.append((min_diag,max_diag))
        for i in range(3):
            bounds.append((min_offdiag,max_offdiag))

    if margs['CMOT']:
        if margs['CMOT NoChange']:
            bounds.append((c.cmot.x, c.cmot.x))
            bounds.append((c.cmot.y, c.cmot.y))
            bounds.append((c.cmot.z, c.cmot.z))
        else:
            max_cmot = margs['CMOT Max']
            for i in range(3):
                bounds.append((-max_cmot,max_cmot))

    (p,err,iterations,imode,smode) = optimize.fmin_slsqp(wmm_error, p, bounds=bounds, full_output=True)
    if imode != 0:
        print("Fit failed: %s" % smode)
        sys.exit(1)
    p = list(p)

    c.offsets = Vector3(p.pop(0), p.pop(0), p.pop(0))
    c.scaling = p.pop(0)

    if margs['Elliptical']:
        c.diag = Vector3(p.pop(0), p.pop(0), p.pop(0))
        c.offdiag = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.diag = Vector3(1.0, 1.0, 1.0)
        c.offdiag = Vector3(0.0, 0.0, 0.0)

    if margs['CMOT']:
        c.cmot = Vector3(p.pop(0), p.pop(0), p.pop(0))
    else:
        c.cmot = Vector3(0.0, 0.0, 0.0)
    return c

def remove_offsets(MAG, BAT, c):
    '''remove all corrections to get raw sensor data'''
    correction_matrix = Matrix3(Vector3(c.diag.x,    c.offdiag.x, c.offdiag.y),
                                Vector3(c.offdiag.x, c.diag.y,    c.offdiag.z),
                                Vector3(c.offdiag.y, c.offdiag.z, c.diag.z))
    try:
        correction_matrix = correction_matrix.invert()
    except Exception:
        return False

    field = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)
    if BAT is not None and hasattr(BAT,'Curr') and not math.isnan(BAT.Curr):
        field -= c.cmot * BAT.Curr
    field = correction_matrix * field
    field *= 1.0 / c.scaling
    field -= Vector3(MAG.OfsX, MAG.OfsY, MAG.OfsZ)

    if math.isnan(field.x) or math.isnan(field.y) or math.isnan(field.z):
        return False
    MAG.MagX = int(field.x)
    MAG.MagY = int(field.y)
    MAG.MagZ = int(field.z)
    return True

def magfit(mlog, timestamp_in_range):
    '''find best magnetometer offset fit to a log file'''

    global earth_field, declination
    global data
    data = []

    ATT = None
    BAT = None

    mag_msg = margs['Magnetometer']
    global mag_idx
    if mag_msg[-1].isdigit():
        mag_instance = None
        mag_idx = mag_msg[-1]
    elif mag_msg.endswith('[0]'):
        mag_instance = 0
        mag_idx = ''
        mag_msg = 'MAG'
    elif mag_msg.endswith(']'):
        mag_instance = int(mag_msg[-2])
        mag_idx = str(mag_instance+1)
        mag_msg = 'MAG'
    else:
        mag_instance = None
        mag_idx = ''

    count = 0
    parameters = {}

    # get parameters
    mlog.rewind()
    while True:
        msg = mlog.recv_match(type=['PARM'])
        if msg is None:
            break
        parameters[msg.Name] = msg.Value

    mlog.rewind()

    lat = margs['Lattitude']
    lon = margs['Longitude']
    if lat != 0 and lon != 0:
        earth_field = mavextra.expected_earth_field_lat_lon(lat, lon)
        (declination,inclination,intensity) = mavextra.get_mag_field_ef(lat, lon)
        print("Earth field: %s  strength %.0f declination %.1f degrees" % (earth_field, earth_field.length(), declination))

    ATT_NAME = margs['Attitude']

    mtypes = ['GPS',mag_msg,ATT_NAME,'BAT']
    if ATT_NAME == "XKY0":
        mtypes.append('ATT')
    last_ATT = None
    print("Attitude source %s mtypes=%s" % (ATT_NAME, mtypes))

    # extract MAG data
    while True:
        msg = mlog.recv_match(type=mtypes)
        if msg is None:
            break
        in_range = timestamp_in_range(msg._timestamp)
        if in_range < 0:
            continue
        if in_range > 0:
            break
        if msg.get_type() == 'GPS' and msg.Status >= 3 and earth_field is None:
            earth_field = mavextra.expected_earth_field(msg)
            (declination,inclination,intensity) = mavextra.get_mag_field_ef(msg.Lat, msg.Lng)
            print("Earth field: %s  strength %.0f declination %.1f degrees" % (earth_field, earth_field.length(), declination))
        if msg.get_type() == 'ATT':
            # needed for XKY0 for yaw
            last_ATT = msg
        if msg.get_type() == ATT_NAME:
            if getattr(msg, 'C', 0) != 0:
                # use core zero for EKF attitude
                continue
            if ATT_NAME == 'XKY0':
                if last_ATT is None:
                    continue
                # get yaw from GSF, and roll/pitch from ATT
                ATT = last_ATT
                ATT.Yaw = math.degrees(msg.YC)
            else:
                ATT = msg
            ATT.Roll  += math.degrees(parameters['AHRS_TRIM_X'])
            ATT.Pitch += math.degrees(parameters['AHRS_TRIM_Y'])
            ATT.Yaw   += math.degrees(parameters['AHRS_TRIM_Z'])
        if msg.get_type() == 'BAT':
            if hasattr(msg,'Instance'):
                if margs['BatteryNum'] != msg.Instance+1:
                    continue
            BAT = msg
        if msg.get_type() == mag_msg and ATT is not None:
            if mag_instance is not None:
                if getattr(msg,'I',0) != mag_instance:
                    continue
            if count % margs['Reduce'] == 0:
                data.append((msg,ATT,BAT))
            count += 1

    old_corrections.offsets = Vector3(parameters.get('COMPASS_OFS%s_X' % mag_idx,0.0),
                                      parameters.get('COMPASS_OFS%s_Y' % mag_idx,0.0),
                                      parameters.get('COMPASS_OFS%s_Z' % mag_idx,0.0))
    old_corrections.diag = Vector3(parameters.get('COMPASS_DIA%s_X' % mag_idx,1.0),
                                   parameters.get('COMPASS_DIA%s_Y' % mag_idx,1.0),
                                   parameters.get('COMPASS_DIA%s_Z' % mag_idx,1.0))
    if old_corrections.diag == Vector3(0,0,0):
        old_corrections.diag = Vector3(1,1,1)
    old_corrections.offdiag = Vector3(parameters.get('COMPASS_ODI%s_X' % mag_idx,0.0),
                                      parameters.get('COMPASS_ODI%s_Y' % mag_idx,0.0),
                                      parameters.get('COMPASS_ODI%s_Z' % mag_idx,0.0))
    if parameters.get('COMPASS_MOTCT',0) == 2:
        # only support current based corrections for now
        old_corrections.cmot = Vector3(parameters.get('COMPASS_MOT%s_X' % mag_idx,0.0),
                                       parameters.get('COMPASS_MOT%s_Y' % mag_idx,0.0),
                                       parameters.get('COMPASS_MOT%s_Z' % mag_idx,0.0))
    old_corrections.scaling = parameters.get('COMPASS_SCALE%s' % mag_idx, None)
    if old_corrections.scaling is None or old_corrections.scaling < 0.1:
        force_scale = False
        old_corrections.scaling = 1.0
    else:
        force_scale = True

    # remove existing corrections
    data2 = []
    for (MAG,ATT,BAT) in data:
        if remove_offsets(MAG, BAT, old_corrections):
            data2.append((MAG,ATT,BAT))
    data = data2

    print("Extracted %u points" % len(data))
    print("Current: %s diag: %s offdiag: %s cmot: %s scale: %.2f" % (
        old_corrections.offsets, old_corrections.diag, old_corrections.offdiag, old_corrections.cmot, old_corrections.scaling))
    if len(data) == 0:
        return

    # do fit
    c = fit_WWW()

    # normalise diagonals to scale factor
    if force_scale:
        avgdiag = (c.diag.x + c.diag.y + c.diag.z)/3.0
        calc_scale = c.scaling
        c.scaling *= avgdiag
        min_scale = margs['ScaleMin']
        max_scale = margs['ScaleMax']
        if c.scaling > max_scale:
            c.scaling = max_scale
        if c.scaling < min_scale:
            c.scaling = min_scale
        scale_change = c.scaling / calc_scale
        c.diag *= 1.0/scale_change
        c.offdiag *= 1.0/scale_change

    print("New: %s diag: %s offdiag: %s cmot: %s scale: %.2f" % (
        c.offsets, c.diag, c.offdiag, c.cmot, c.scaling))

    x = []

    corrected = {}
    corrected['Yaw'] = []
    expected1 = {}
    expected2 = {}
    uncorrected = {}
    uncorrected['Yaw'] = []
    yaw_change1 = []
    yaw_change2 = []
    for i in range(len(data)):
        (MAG,ATT,BAT) = data[i]
        yaw1 = get_yaw(ATT,MAG,BAT,c)
        corrected['Yaw'].append(yaw1)
        ef1 = expected_field(ATT, yaw1)
        cf = correct(MAG, BAT, c)

        yaw2 = get_yaw(ATT,MAG,BAT,old_corrections)
        ef2 = expected_field(ATT, yaw2)
        uncorrected['Yaw'].append(yaw2)
        uf = correct(MAG, BAT, old_corrections)

        yaw_change1.append(mavextra.wrap_180(yaw1 - yaw2))
        yaw_change2.append(mavextra.wrap_180(yaw1 - ATT.Yaw))
        for axis in ['x','y','z']:
            if not axis in corrected:
                corrected[axis] = []
                uncorrected[axis] = []
                expected1[axis] = []
                expected2[axis] = []
            corrected[axis].append(getattr(cf, axis))
            uncorrected[axis].append(getattr(uf, axis))
            expected1[axis].append(getattr(ef1, axis))
            expected2[axis].append(getattr(ef2, axis))
        x.append(i)

    c.show_parms()

    fig, axs = pyplot.subplots(3, 1, sharex=True)

    for axis in ['x','y','z']:
        axs[0].plot(numpy.array(x), numpy.array(uncorrected[axis]), label='Uncorrected %s' % axis.upper() )
        axs[0].plot(numpy.array(x), numpy.array(expected2[axis]), label='Expected %s' % axis.upper() )
        axs[0].legend(loc='upper left')
        axs[0].set_title('Original')
        axs[0].set_ylabel('Field (mGauss)')

        axs[1].plot(numpy.array(x), numpy.array(corrected[axis]), label='Corrected %s' % axis.upper() )
        axs[1].plot(numpy.array(x), numpy.array(expected1[axis]), label='Expected %s' % axis.upper() )
        axs[1].legend(loc='upper left')
        axs[1].set_title('Corrected')
        axs[1].set_ylabel('Field (mGauss)')

    # show change in yaw estimate from old corrections to new
    axs[2].plot(numpy.array(x), numpy.array(yaw_change1), label='Mag Yaw Change')
    axs[2].plot(numpy.array(x), numpy.array(yaw_change2), label='ATT Yaw Change')
    axs[2].set_title('Yaw Change (degrees)')
    axs[2].legend(loc='upper left')

    pyplot.show(block=False)

class MagFit(MPDataLogChildTask):
    '''A class used to launch the MagFitUI in a child process'''

    def __init__(self, *args, **kwargs):
        '''
        Parameters
        ----------
        title : str
            The title of the application
        mlog : DFReader
            A dataflash or telemetry log
        xlimits: MAVExplorer.XLimits
            An object capturing timestamp limits
        '''

        super(MagFit, self).__init__(*args, **kwargs)

        # all attributes are implicitly passed to the child process 
        self.title = kwargs['title']
        self.xlimits = kwargs['xlimits']

    # @override
    def child_task(self):
        '''Launch the MagFitUI'''

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx

        # create wx application
        app = wx.App(False)
        app.frame = MagFitUI(title=self.title,
                             close_event=self.close_event,
                             mlog=self.mlog,
                             timestamp_in_range=self.xlimits.timestamp_in_range)

        app.frame.SetDoubleBuffered(True)
        app.frame.Show()
        app.MainLoop()

class MagFitUI(wx.Dialog):
    def __init__(self, title, close_event, mlog, timestamp_in_range):
        super(MagFitUI, self).__init__(None, title=title, size=(600, 800))

        # capture the close event, log and timestamp range function
        self.close_event = close_event
        self.mlog = mlog
        self.timestamp_in_range = timestamp_in_range

        # events
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.timer.Start(100)
        self.Bind(wx.EVT_IDLE, self.OnIdle)

        # initialise the panels etc.
        self.init_ui()

    def OnIdle(self, event):
        time.sleep(0.05)

    def OnTimer(self, event):
        '''Periodically check if the close event has been received'''

        if self.close_event.wait(0.001):
            self.timer.Stop()
            self.Destroy()
            return

    def have_msg(self, msg):
        '''see if we have a given message name in the log'''
        mid = self.mlog.name_to_id.get(msg,-1)
        if mid == -1:
            return False
        return self.mlog.counts[mid] > 0

    def init_ui(self):
        '''Initalise the UI elements'''

        if not hasattr(self.mlog, 'formats'):
            print("Must be DF log")
            return

        self.panel = wx.Panel(self)
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.AddStretchSpacer()
        self.panel.SetSizer(self.vbox)
        self.idmap = {}
        self.values = {}
        self.controls = {}
        self.callbacks = {}
        self.row = None

        msg_names = self.mlog.name_to_id.keys()
        mag_format = self.mlog.formats[self.mlog.name_to_id['MAG']]
        if 'I' in mag_format.columns:
            mag_choices = ['MAG[0]', 'MAG[1]', 'MAG[2]']
        else:
            mag_choices = ['MAG', 'MAG2', 'MAG3']

        att_choices = ['ATT']
        if self.have_msg('NKF1'):
            att_choices.append('NKF1')
        if self.have_msg('XKF1'):
            att_choices.append('XKF1')
        if self.have_msg('XKY0'):
            att_choices.append('XKY0')

        # first row, Mag and attitude source
        self.StartRow('Source Selection')
        self.AddCombo('Magnetometer', mag_choices)
        self.AddCombo('Attitude', att_choices)

        self.StartRow('Position')
        self.AddSpinFloat("Lattitude", -90, 90, 0.000001, 0, digits=8)
        self.AddSpinFloat("Longitude", -180, 180, 0.000001, 0, digits=8)

        self.StartRow()
        self.AddSpinInteger("Reduce", 1, 20, 1)

        self.StartRow('Offset Estimation')
        self.AddCheckBox("Offsets", default=True)
        self.StartRow()
        self.AddSpinInteger("OffsetMax", 500, 3000, 1500)

        self.StartRow('Scale Factor Estimation')
        self.AddSpinFloat("ScaleMin", 0.5, 2.0, 0.01, 1.0)
        self.AddSpinFloat("ScaleMax", 0.5, 2.0, 0.01, 1.0)

        self.StartRow('Elliptical Estimation')
        self.AddCheckBox("Elliptical")
        self.StartRow()
        self.AddSpinFloat("DiagonalMin", 0.8, 1.0, 0.01, 0.8)
        self.AddSpinFloat("DiagonalMax", 1.0, 1.2, 0.01, 1.2)
        self.StartRow()
        self.AddSpinFloat("OffDiagMin", -0.2, 0.0, 0.01, -0.2)
        self.AddSpinFloat("OffDiagMax", 0, 0.2, 0.01, 0.2)

        self.StartRow('Motor Interference Estimation')
        self.AddCheckBox("CMOT")
        self.AddCheckBox("CMOT NoChange")
        self.StartRow()
        self.AddSpinInteger("BatteryNum", 1, 8, 1)
        self.AddSpinFloat("CMOT Max", 1.0, 100, 0.1, 10)

        self.StartRow("Processing")
        self.AddButton('Run', callback=self.run)
        self.AddButton('Close', callback=self.close)
        self.EndRow()
        
        self.Center()

    def close(self):
        '''Set the close event'''

        self.close_event.set()

    def StartRow(self, label=None):
        if self.row:
            self.EndRow()
        if label:
            self.row = wx.BoxSizer(wx.HORIZONTAL)
            text = wx.StaticText(self.panel, label=label)
            font = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
            text.SetFont(font)
            self.row.Add(text, 0, wx.LEFT, 10)
            self.vbox.Add(self.row, 0, wx.TOP, 10)
        self.row = wx.BoxSizer(wx.HORIZONTAL)

    def EndRow(self):
        self.vbox.Add(self.row, 0, wx.TOP, 10)
        self.row = None

    def AddControl(self, c, label, default):
        self.idmap[c.GetId()] = label
        self.controls[c.GetId()] = c
        self.values[label] = default

    def AddCombo(self, label, choices):
        c = wx.ComboBox(choices=choices,
                        parent=self.panel,
                        style=0,
                        value=choices[0])
        self.AddControl(c, label, choices[0])
        c.Bind(wx.EVT_COMBOBOX, self.OnValue)
        self.row.Add(wx.StaticText(self.panel, label=label), 0, wx.LEFT, 20)
        self.row.Add(c, 0, wx.LEFT, 20)

    def AddButton(self, label, callback=None):
        c = wx.Button(self.panel, label=label)
        self.AddControl(c, label, False)
        if callback is not None:
            self.callbacks[c.GetId()] = callback
        c.Bind(wx.EVT_BUTTON, self.OnButton)
        self.row.Add(c, 0, wx.LEFT, 20)

    def AddCheckBox(self, label, default=False):
        c = wx.CheckBox(self.panel, label=label)
        c.SetValue(default)
        self.AddControl(c, label, default)
        c.Bind(wx.EVT_CHECKBOX, self.OnValue)
        self.row.Add(c, 0, wx.LEFT, 20)
        
    def AddSpinInteger(self, label, min_value, max_value, default):
        c = wx.SpinCtrl(self.panel, -1)
        c.SetRange(min_value, max_value)
        c.SetValue(default)
        self.AddControl(c, label, default)
        self.row.Add(wx.StaticText(self.panel, label=label), 0, wx.LEFT, 20)
        self.row.Add(c, 0, wx.LEFT, 20)
        self.row.Add(wx.StaticText(self.panel, label=""), 0, wx.LEFT, 20)
        self.Bind(wx.EVT_SPINCTRL, self.OnValue)

    def AddSpinFloat(self, label, min_value, max_value, increment, default, digits=2):
        c = wx.SpinCtrlDouble(self.panel, -1)
        c.SetRange(min_value, max_value)
        c.SetValue(default)
        c.SetIncrement(increment)
        c.SetDigits(digits)
        s1 = "%.*f" % (digits, min_value)
        s2 = "%.*f" % (digits, max_value)
        if len(s1) > len(s2):
            s = s1
        else:
            s = s2
        if hasattr(c, 'GetSizeFromText'):
            size = c.GetSizeFromText(s+" ")
            c.SetMinSize(size)
        self.AddControl(c, label, default)
        self.row.Add(wx.StaticText(self.panel, label=label), 0, wx.LEFT, 20)
        self.row.Add(c, 0, wx.LEFT, 20)
        self.row.Add(wx.StaticText(self.panel, label=""), 0, wx.LEFT, 20)
        self.Bind(wx.EVT_SPINCTRLDOUBLE, self.OnValue)
        
    def OnValue(self, event):
        self.values[self.idmap[event.GetId()]] = self.controls[event.GetId()].GetValue()

    def OnButton(self, event):
        self.values[self.idmap[event.GetId()]] = True
        if event.GetId() in self.callbacks:
            self.callbacks[event.GetId()]()

    def run(self):
        global margs
        margs = self.values
        magfit(self.mlog, self.timestamp_in_range)

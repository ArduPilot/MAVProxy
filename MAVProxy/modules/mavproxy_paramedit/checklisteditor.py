#!/usr/bin/env python
import wx
import wx.grid as gridlib
import math


class GridCheckListEditor(gridlib.PyGridCellEditor):
    """
    This is a custom CheckListBox editor for setting of Bitmasks
    """
    def __init__(self, choice, pvalcol, start):
        self.choices = choice
        binary = bin(int(start))[2:]
        self.startValue = [(len(binary)-ones-1) for ones in range(len(binary)) if binary[ones] == '1']
        self.pvalcol = pvalcol
        gridlib.PyGridCellEditor.__init__(self)

    def set_checked(self, selected):
        if int(selected) < int(math.pow(2, len(self.choices))):
            binary = bin(int(selected))[2:]
            self.startValue = [(len(binary)-ones-1) for ones in range(len(binary)) if binary[ones] == '1']
            try:
                self._tc.SetChecked(self.startValue)
            except Exception as e:
                print (e)

    def get_checked(self):
        return self._tc.GetChecked()

    def Create(self, parent, id, evtHandler):
        self._tc = wx.CheckListBox(parent, id, choices=self.choices)
        self._tc.SetChecked(self.startValue)
        self.SetControl(self._tc)
        if evtHandler:
            self._tc.PushEventHandler(evtHandler)

    def SetSize(self, rect):
        self._tc.SetDimensions(rect.x, rect.y, rect.width+2, 30*len(self.choices),
                               wx.SIZE_ALLOW_MINUS_ONE)

    def Show(self, show, attr):
        super(GridCheckListEditor, self).Show(show, attr)

    def BeginEdit(self, row, col, grid):
        self._tc.SetChecked(self.startValue)

    def EndEdit(self, row, col, grid, oldVal):
        val = self._tc.GetChecked()
        self.startValue = val
        if val != oldVal:
            return self._tc.GetChecked()
        else:
            return None

    def get_pvalue(self):
        s = 0
        for i in self._tc.GetChecked():
            s = s + int(math.pow(2, i))
        return str(s)

    def ApplyEdit(self, row, col, grid):
        val = ""
        for i in self._tc.GetChecked():
            val = val + str(self.choices[i]) + "\n"
        grid.GetTable().SetValue(row, col, str(val))
        grid.GetTable().SetValue(row, self.pvalcol, self.get_pvalue())

    def Reset(self):
        self._tc.SetChecked(None)

    def StartingKey(self, evt):
        evt.Skip()

    def Destroy(self):
        super(GridCheckListEditor, self).Destroy()

    def Clone(self):
        return GridCheckListEditor(self.choices, self.pvalcol, self.startValue)


class GridDropListEditor(gridlib.PyGridCellEditor):
    """
    This is a custom DropBox editor for setting of Parameter Values
    """
    def __init__(self, choice, pvalcol, start=0):
        self.choices = choice
        self.startValue = int(start)
        self.pvalcol = pvalcol
        gridlib.PyGridCellEditor.__init__(self)

    def set_checked(self, selected):
        self.startValue = int(selected)
        try:
            self._tc.SetSelection(self.startValue)
        except Exception as e:
            print (e)

    def get_checked(self):
        return self._tc.GetSelection()

    def Create(self, parent, id, evtHandler):
        self._tc = wx.ComboBox(parent, id, choices=self.choices)
        self.set_checked(self.startValue)
        self.SetControl(self._tc)
        if evtHandler:
            self._tc.PushEventHandler(evtHandler)

    def Show(self, show, attr):
        super(GridDropListEditor, self).Show(show, attr)

    def SetSize(self, rect):
        self._tc.SetDimensions(rect.x, rect.y, rect.width+2, 30,
                               wx.SIZE_ALLOW_MINUS_ONE)

    def BeginEdit(self, row, col, grid):
        self._tc.SetSelection(self.startValue)

    def EndEdit(self, row, col, grid, oldVal):
        val = self._tc.GetSelection()
        if val != oldVal and val != -1:
            self.startValue = val
            return self._tc.GetSelection()
        else:
            return None

    def ApplyEdit(self, row, col, grid):
        grid.GetTable().SetValue(row, col, self._tc.GetStringSelection())
        grid.GetTable().SetValue(row, self.pvalcol, str(self._tc.GetStringSelection()).split(':')[0])

    def Reset(self):
        self._tc.SetSelection(None)

    def StartingKey(self, evt):
        evt.Skip()

    def Destroy(self):
        super(GridDropListEditor, self).Destroy()

    def Clone(self):
        return GridDropListEditor(self.choices, self.pvalcol, self.startValue)


class GridScrollEditor(gridlib.PyGridCellEditor):
    """
    This is a custom SpinControlDouble editor for setting of float values with given range and increments
    """
    def __init__(self, Range, pvalcol, start):
        self.Range = Range
        self.startValue = float(start)
        self.pvalcol = pvalcol
        gridlib.PyGridCellEditor.__init__(self)

    def set_checked(self, selected):
        self.startValue = selected
        try:
            self._tc.SetValue(selected)
        except Exception as e:
            print (e)

    def SetSize(self, rect):
        self._tc.SetDimensions(rect.x, rect.y, rect.width+2, 30,
                               wx.SIZE_ALLOW_MINUS_ONE)

    def get_checked(self):
        return self._tc.GetValue()

    def Create(self, parent, id, evtHandler):
        self._tc = wx.SpinCtrlDouble(parent, id, min=self.Range['Min'], max=self.Range['Max'], inc=self.Range['Increment'])
        self.SetControl(self._tc)
        self.set_checked(self.startValue)
        if evtHandler:
            self._tc.PushEventHandler(evtHandler)

    def Show(self, show, attr):
        super(GridScrollEditor, self).Show(show, attr)

    def BeginEdit(self, row, col, grid):
        self._tc.SetValue(self.startValue)

    def EndEdit(self, row, col, grid, oldVal):
        val = self._tc.GetValue()
        self.startValue = val
        if val != oldVal:
            return val
        else:
            return None

    def ApplyEdit(self, row, col, grid):
        grid.GetTable().SetValue(row, col, str(self._tc.GetValue()))
        grid.GetTable().SetValue(row, self.pvalcol, str(self._tc.GetValue()))

    def Reset(self):
        self._tc.SetValue(None)

    def StartingKey(self, evt):
        evt.Skip()

    def Destroy(self):
        super(GridScrollEditor, self).Destroy()

    def Clone(self):
        return GridScrollEditor(self.Range, self.pvalcol, self.startValue)

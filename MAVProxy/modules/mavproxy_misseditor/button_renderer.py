#!/usr/bin/env python
'''
Custom button render class for use inside a wx.grid
(ported from http://forums.wxwidgets.org/viewtopic.php?t=14403 )
Michael Day
June 2014
'''

from ..lib.wx_loader import wx
from wx import grid
import copy

class ButtonRenderer(wx.grid.PyGridCellRenderer):
    def __init__(self,label,width=75,height=25):
        self.label = label
        self.width = width
        self.height = height

        wx.grid.PyGridCellRenderer.__init__(self)

    def Clone(self):
        return copy.copy(self)

    def GetBestSize(self, grid, dc, row, col):
        return wx.Size(self.width,self.height)

    def Draw(self, grid, attr, dc, rect, row, col, isSelected):
        dc.SetBrush(wx.Brush(wx.SystemSettings.GetColour(
            wx.SYS_COLOUR_BTNFACE)))
        dc.DrawRectangle( rect.GetX(), rect.GetY(), rect.GetWidth(), rect.GetHeight())
        #draw a shaded rectangle to emulate a button
        #(taken from src/generic/renderg.cpp)
        strength = 1
        pen1 = wx.Pen(wx.WHITE, strength)
        dc.SetPen(pen1)
        dc.DrawLine(rect.GetLeft()+strength-1, rect.GetTop()+strength-1,
                rect.GetLeft()+strength-1, rect.GetBottom()-strength+1)
        dc.DrawLine(rect.GetLeft()+strength-1, rect.GetTop()+strength-1,
                rect.GetRight()-strength, rect.GetTop()+strength-1)
        pen2 = wx.Pen(wx.BLACK, strength)
        dc.SetPen(pen2)
        dc.DrawLine(rect.GetRight()-strength, rect.GetTop(),
                rect.GetRight()-strength, rect.GetBottom());
        dc.DrawLine(rect.GetLeft(), rect.GetBottom(),
                rect.GetRight() - strength, rect.GetBottom());

        '''
        #another drawing routine
        #(taken from src/generic/renderg.cpp)
        #Could port this later for animating the button when clicking

  const wxCoord x = rect.x,
                y = rect.y,
                w = rect.width,
                h = rect.height;

  dc.SetBrush(*wxTRANSPARENT_BRUSH);

  wxPen pen(*wxBLACK, 1);

  dc.SetPen(pen);
  dc.DrawLine( x+w, y, x+w, y+h );            // right (outer)
  dc.DrawRectangle( x, y+h, w+1, 1 );         // bottom (outer)

  pen.SetColour(wxColour(wxT("DARK GREY")));
  dc.SetPen(pen);
  dc.DrawLine( x+w-1, y, x+w-1, y+h );        // right (inner)
  dc.DrawRectangle( x+1, y+h-1, w-2, 1 );     // bottom (inner)

  pen.SetColour(*wxWHITE);
  dc.SetPen(pen);
  dc.DrawRectangle( x, y, w, 1 );             // top (outer)
  dc.DrawRectangle( x, y, 1, h );             // left (outer)
  dc.DrawLine( x, y+h-1, x+1, y+h-1 );
  dc.DrawLine( x+w-1, y, x+w-1, y+1 );
        '''

        # draw the button-label
        dc.SetBackgroundMode(wx.TRANSPARENT )
        dc.SetTextForeground(attr.GetTextColour() )
        dc.SetFont( attr.GetFont() )
        #dc.DrawLabel( wxT("Delete"), rect,
        dc.DrawLabel( self.label, rect,
                wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_CENTER_HORIZONTAL)

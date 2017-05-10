# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version Jun 30 2011)
## http://www.wxformbuilder.org/
##
## PLEASE DO "NOT" EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc
import wx.grid

m_mniOpenId = 1000
m_mniSaveId = 1001
m_mniExitId = 1002
m_mniAboutId = 1003

###########################################################################
## Class MainFrameBase
###########################################################################

class MainFrameBase ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"MAVlink Parameter Editor", pos = wx.DefaultPosition, size = wx.Size( 640,480 ), style = wx.CAPTION|wx.CLOSE_BOX|wx.ICONIZE|wx.MINIMIZE_BOX|wx.RESIZE_BORDER|wx.SYSTEM_MENU|wx.TAB_TRAVERSAL )
		
		self.SetSizeHintsSz( wx.Size( 530,380 ), wx.DefaultSize )
		
		bSizer3 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_notebook1 = wx.Notebook( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.NB_FIXEDWIDTH )
		self.m_panelParameterEdit = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer3 = wx.FlexGridSizer( 3, 1, 0, 0 )
		fgSizer3.SetFlexibleDirection( wx.BOTH )
		fgSizer3.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		bSizer13 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.m_buttonReadParameters = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"Read Parameters", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer13.Add( self.m_buttonReadParameters, 0, wx.ALL, 5 )
		
		self.m_staticline11 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline11, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline12 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline12, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonWriteParameters = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"Write Parameters", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer13.Add( self.m_buttonWriteParameters, 0, wx.ALL, 5 )
		
		self.m_staticline13 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline13, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline14 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline14, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonWriteToFile2 = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"Write To File", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer13.Add( self.m_buttonWriteToFile2, 0, wx.ALL, 5 )
		
		self.m_staticline15 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline15, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline16 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer13.Add( self.m_staticline16, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonReadFromFile2 = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"Read From File", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer13.Add( self.m_buttonReadFromFile2, 0, wx.ALL, 5 )
		
		fgSizer3.Add( bSizer13, 1, wx.EXPAND, 5 )
		
		fgSizerParams = wx.FlexGridSizer( 1, 2, 0, 0 )
		fgSizerParams.SetFlexibleDirection( wx.BOTH )
		fgSizerParams.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_listCtrlSectionList = wx.ListCtrl( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), wx.LC_NO_HEADER|wx.LC_REPORT|wx.LC_SINGLE_SEL|wx.ALWAYS_SHOW_SB )
		self.m_listCtrlSectionList.SetMinSize( wx.Size( 150,225 ) )
		
		fgSizerParams.Add( self.m_listCtrlSectionList, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_gridParameters = wx.grid.Grid( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), 0 )
		
		# Grid
		self.m_gridParameters.CreateGrid( 5, 3 )
		self.m_gridParameters.EnableEditing( True )
		self.m_gridParameters.EnableGridLines( True )
		self.m_gridParameters.EnableDragGridSize( False )
		self.m_gridParameters.SetMargins( 0, 0 )
		
		# Columns
		self.m_gridParameters.SetColSize( 0, 118 )
		self.m_gridParameters.SetColSize( 1, 72 )
		self.m_gridParameters.SetColSize( 2, 53 )
		self.m_gridParameters.AutoSizeColumns()
		self.m_gridParameters.EnableDragColMove( False )
		self.m_gridParameters.EnableDragColSize( True )
		self.m_gridParameters.SetColLabelSize( 30 )
		self.m_gridParameters.SetColLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Rows
		self.m_gridParameters.SetRowSize( 0, 17 )
		self.m_gridParameters.SetRowSize( 1, 17 )
		self.m_gridParameters.SetRowSize( 2, 17 )
		self.m_gridParameters.SetRowSize( 3, 17 )
		self.m_gridParameters.SetRowSize( 4, 17 )
		self.m_gridParameters.EnableDragRowSize( True )
		self.m_gridParameters.SetRowLabelSize( 80 )
		self.m_gridParameters.SetRowLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Label Appearance
		
		# Cell Defaults
		self.m_gridParameters.SetDefaultCellAlignment( wx.ALIGN_LEFT, wx.ALIGN_TOP )
		self.m_gridParameters.SetMinSize( wx.Size( 325,225 ) )
		
		fgSizerParams.Add( self.m_gridParameters, 0, wx.ALL|wx.EXPAND, 5 )
		
		fgSizer3.Add( fgSizerParams, 1, wx.EXPAND, 5 )
		
		bSizer10 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.m_staticText6 = wx.StaticText( self.m_panelParameterEdit, wx.ID_ANY, u"FLASH MEMORY (by selection)", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText6.Wrap( -1 )
		bSizer10.Add( self.m_staticText6, 0, wx.ALL, 5 )
		
		self.m_buttonLoad = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"LOAD", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer10.Add( self.m_buttonLoad, 0, wx.ALL, 5 )
		
		self.m_staticline1 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer10.Add( self.m_staticline1, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline2 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer10.Add( self.m_staticline2, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonSave = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"SAVE", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer10.Add( self.m_buttonSave, 0, wx.ALL, 5 )
		
		self.m_staticline131 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer10.Add( self.m_staticline131, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline141 = wx.StaticLine( self.m_panelParameterEdit, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer10.Add( self.m_staticline141, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonClear = wx.Button( self.m_panelParameterEdit, wx.ID_ANY, u"CLEAR", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer10.Add( self.m_buttonClear, 0, wx.ALL, 5 )
		
		fgSizer3.Add( bSizer10, 1, wx.EXPAND, 5 )
		
		self.m_panelParameterEdit.SetSizer( fgSizer3 )
		self.m_panelParameterEdit.Layout()
		fgSizer3.Fit( self.m_panelParameterEdit )
		self.m_notebook1.AddPage( self.m_panelParameterEdit, u"parameters", True )
		self.m_panelCalibration = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		bSizer91 = wx.BoxSizer( wx.VERTICAL )
		
		bSizer111 = wx.BoxSizer( wx.HORIZONTAL )
		
		bSizer111.SetMinSize( wx.Size( -1,40 ) ) 
		self.m_buttonDeclareGround = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Declare Grounded", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer111.Add( self.m_buttonDeclareGround, 0, wx.ALL, 5 )
		
		self.m_staticline21 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer111.Add( self.m_staticline21, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline22 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer111.Add( self.m_staticline22, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonAckGrounded = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Aknowledge grounded", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer111.Add( self.m_buttonAckGrounded, 0, wx.ALL, 5 )
		
		bSizer91.Add( bSizer111, 0, wx.EXPAND, 5 )
		
		self.m_staticline25 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer91.Add( self.m_staticline25, 0, wx.EXPAND |wx.ALL, 5 )
		
		bSizer1111 = wx.BoxSizer( wx.HORIZONTAL )
		
		bSizer1111.SetMinSize( wx.Size( -1,40 ) ) 
		self.m_buttonDeclareActive = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Declare Active", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1111.Add( self.m_buttonDeclareActive, 0, wx.ALL, 5 )
		
		self.m_staticline211 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer1111.Add( self.m_staticline211, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_staticline221 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer1111.Add( self.m_staticline221, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonAckActive = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Aknowledge Active", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer1111.Add( self.m_buttonAckActive, 0, wx.ALL, 5 )
		
		bSizer91.Add( bSizer1111, 0, wx.EXPAND, 5 )
		
		self.m_staticline23 = wx.StaticLine( self.m_panelCalibration, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer91.Add( self.m_staticline23, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonCalIMU = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Calibrate IMU", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer91.Add( self.m_buttonCalIMU, 0, wx.ALL, 5 )
		
		self.m_buttonCalHome = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Calibrate Home", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer91.Add( self.m_buttonCalHome, 0, wx.ALL, 5 )
		
		self.m_buttonCalRadio = wx.Button( self.m_panelCalibration, wx.ID_ANY, u"Calibrate Radio", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer91.Add( self.m_buttonCalRadio, 0, wx.ALL, 5 )
		
		self.m_panelCalibration.SetSizer( bSizer91 )
		self.m_panelCalibration.Layout()
		bSizer91.Fit( self.m_panelCalibration )
		self.m_notebook1.AddPage( self.m_panelCalibration, u"Calibration", False )
		
		bSizer3.Add( self.m_notebook1, 1, wx.EXPAND |wx.ALL, 5 )
		
		self.SetSizer( bSizer3 )
		self.Layout()
		self.m_statusBar = self.CreateStatusBar( 1, wx.ST_SIZEGRIP, wx.ID_ANY )
		self.m_menubar = wx.MenuBar( 0 )
		self.m_mnFile = wx.Menu()
		self.m_mniOpen = wx.MenuItem( self.m_mnFile, m_mniOpenId, u"&Open...", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniOpen )
		
		self.m_mniSave = wx.MenuItem( self.m_mnFile, m_mniSaveId, u"&Save...", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniSave )
		
		self.m_mnFile.AppendSeparator()
		
		self.m_mniExit = wx.MenuItem( self.m_mnFile, m_mniExitId, u"&Exit", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniExit )
		
		self.m_menubar.Append( self.m_mnFile, u"&File" ) 
		
		self.m_mnHelp = wx.Menu()
		self.m_mniAbout = wx.MenuItem( self.m_mnHelp, m_mniAboutId, u"&About", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnHelp.AppendItem( self.m_mniAbout )
		
		self.m_menubar.Append( self.m_mnHelp, u"&?" ) 
		
		self.SetMenuBar( self.m_menubar )
		
		
		# Connect Events
		self.Bind( wx.EVT_CLOSE, self.OnClose )
		self.m_panelParameterEdit.Bind( wx.EVT_SIZE, self.m_panelParameterEditSize )
		self.m_buttonReadParameters.Bind( wx.EVT_BUTTON, self.m_btClick_ReadParameters )
		self.m_buttonWriteParameters.Bind( wx.EVT_BUTTON, self.m_btClick_WriteParameters )
		self.m_listCtrlSectionList.Bind( wx.EVT_LIST_ITEM_SELECTED, self.m_listCtrlSectionList_ItemSelected )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_CELL_CHANGE, self.m_gridParameters_CellChange )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_EDITOR_SHOWN, self.m_gridParameters_showEdit )
		self.m_buttonLoad.Bind( wx.EVT_BUTTON, self.m_btClick_LoadMem )
		self.m_buttonSave.Bind( wx.EVT_BUTTON, self.m_btClick_SaveMem )
		self.m_buttonClear.Bind( wx.EVT_BUTTON, self.m_btClick_ClearMem )
		self.Bind( wx.EVT_MENU, self.m_mniOpenClick, id = self.m_mniOpen.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniSaveClick, id = self.m_mniSave.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniExitClick, id = self.m_mniExit.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniAboutClick, id = self.m_mniAbout.GetId() )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def OnClose( self, event ):
		event.Skip()
	
	def m_panelParameterEditSize( self, event ):
		event.Skip()
	
	def m_btClick_ReadParameters( self, event ):
		event.Skip()
	
	def m_btClick_WriteParameters( self, event ):
		event.Skip()
	
	def m_listCtrlSectionList_ItemSelected( self, event ):
		event.Skip()
	
	def m_gridParameters_CellChange( self, event ):
		event.Skip()
	
	def m_gridParameters_showEdit( self, event ):
		event.Skip()
	
	def m_btClick_LoadMem( self, event ):
		event.Skip()
	
	def m_btClick_SaveMem( self, event ):
		event.Skip()
	
	def m_btClick_ClearMem( self, event ):
		event.Skip()
	
	def m_mniOpenClick( self, event ):
		event.Skip()
	
	def m_mniSaveClick( self, event ):
		event.Skip()
	
	def m_mniExitClick( self, event ):
		event.Skip()
	
	def m_mniAboutClick( self, event ):
		event.Skip()
	


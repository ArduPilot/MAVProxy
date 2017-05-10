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

m_mniExitId = 1000
m_mniAboutId = 1001

###########################################################################
## Class MainFrameBase
###########################################################################

class MainFrameBase ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"MAV Function editor", pos = wx.DefaultPosition, size = wx.Size( 1020,512 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizer7 = wx.BoxSizer( wx.HORIZONTAL )
		
		self.m_splitter8 = wx.SplitterWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SP_3D )
		self.m_splitter8.Bind( wx.EVT_IDLE, self.m_splitter8OnIdle )
		
		self.m_panel9 = wx.Panel( self.m_splitter8, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SIMPLE_BORDER|wx.TAB_TRAVERSAL )
		self.m_panel9.SetMinSize( wx.Size( 130,-1 ) )
		
		bSizer5 = wx.BoxSizer( wx.VERTICAL )
		
		bSizer5.SetMinSize( wx.Size( 130,-1 ) ) 
		fgSizer1 = wx.FlexGridSizer( 2, 2, 0, 0 )
		fgSizer1.SetFlexibleDirection( wx.BOTH )
		fgSizer1.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		bSizer5.Add( fgSizer1, 0, wx.EXPAND, 5 )
		
		self.m_btUpdate = wx.Button( self.m_panel9, wx.ID_ANY, u"Update", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		self.m_btUpdate.Enable( False )
		
		bSizer5.Add( self.m_btUpdate, 0, wx.ALL, 5 )
		
		self.m_checkBoxAutoUpdate = wx.CheckBox( self.m_panel9, wx.ID_ANY, u"Auto Update", wx.DefaultPosition, wx.DefaultSize, 0 )
		bSizer5.Add( self.m_checkBoxAutoUpdate, 0, wx.ALL, 5 )
		
		self.m_staticline1 = wx.StaticLine( self.m_panel9, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer5.Add( self.m_staticline1, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonGenHeaders = wx.Button( self.m_panel9, wx.ID_ANY, u"Generate C code", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		bSizer5.Add( self.m_buttonGenHeaders, 0, wx.ALL, 5 )
		
		self.m_buttonEditVirtual = wx.Button( self.m_panel9, wx.ID_ANY, u"Edit Virtual Tables", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		bSizer5.Add( self.m_buttonEditVirtual, 0, wx.ALL, 5 )
		
		self.m_staticline2 = wx.StaticLine( self.m_panel9, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer5.Add( self.m_staticline2, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonSaveNV = wx.Button( self.m_panel9, wx.ID_ANY, u"Save to NV memory", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		self.m_buttonSaveNV.Enable( False )
		
		bSizer5.Add( self.m_buttonSaveNV, 0, wx.ALL, 5 )
		
		self.m_buttonClearNVMem = wx.Button( self.m_panel9, wx.ID_ANY, u"Clear NV memory", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		self.m_buttonClearNVMem.Enable( False )
		
		bSizer5.Add( self.m_buttonClearNVMem, 0, wx.ALL, 5 )
		
		self.m_staticline11 = wx.StaticLine( self.m_panel9, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
		bSizer5.Add( self.m_staticline11, 0, wx.EXPAND |wx.ALL, 5 )
		
		self.m_buttonSave = wx.Button( self.m_panel9, wx.ID_ANY, u"Save Settings", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		bSizer5.Add( self.m_buttonSave, 0, wx.ALL, 5 )
		
		self.m_buttonSaveBackup = wx.Button( self.m_panel9, wx.ID_ANY, u"Save to backup", wx.DefaultPosition, wx.Size( 120,40 ), 0 )
		bSizer5.Add( self.m_buttonSaveBackup, 0, wx.ALL, 5 )
		
		self.m_panel9.SetSizer( bSizer5 )
		self.m_panel9.Layout()
		bSizer5.Fit( self.m_panel9 )
		self.m_panel10 = wx.Panel( self.m_splitter8, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		bSizer10 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_splitterEditor = wx.SplitterWindow( self.m_panel10, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SP_3D )
		self.m_splitterEditor.SetSashGravity( 1 )
		self.m_splitterEditor.Bind( wx.EVT_IDLE, self.m_splitterEditorOnIdle )
		self.m_splitterEditor.SetMinimumPaneSize( 300 )
		
		self.m_panelFBs = wx.Panel( self.m_splitterEditor, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SIMPLE_BORDER|wx.TAB_TRAVERSAL )
		bSizerFBs = wx.BoxSizer( wx.VERTICAL )
		
		bSizerFBs.SetMinSize( wx.Size( 300,-1 ) ) 
		self.m_gridFBs = wx.grid.Grid( self.m_panelFBs, wx.ID_ANY, wx.DefaultPosition, wx.Size( 300,-1 ), wx.HSCROLL|wx.VSCROLL )
		
		# Grid
		self.m_gridFBs.CreateGrid( 1, 1 )
		self.m_gridFBs.EnableEditing( False )
		self.m_gridFBs.EnableGridLines( True )
		self.m_gridFBs.EnableDragGridSize( False )
		self.m_gridFBs.SetMargins( 0, 0 )
		
		# Columns
		self.m_gridFBs.EnableDragColMove( False )
		self.m_gridFBs.EnableDragColSize( False )
		self.m_gridFBs.SetColLabelSize( 20 )
		self.m_gridFBs.SetColLabelValue( 0, u"NULL" )
		self.m_gridFBs.SetColLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Rows
		self.m_gridFBs.EnableDragRowSize( False )
		self.m_gridFBs.SetRowLabelSize( 0 )
		self.m_gridFBs.SetRowLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Label Appearance
		
		# Cell Defaults
		self.m_gridFBs.SetDefaultCellAlignment( wx.ALIGN_CENTRE, wx.ALIGN_TOP )
		self.m_gridFBs.SetMaxSize( wx.Size( 300,-1 ) )
		
		self.m_menuGrid = wx.Menu()
		self.m_menuItemAddRegister = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Add Register", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemAddRegister )
		
		self.m_menuItemInsertRegister = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Insert Register", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemInsertRegister )
		
		self.m_menuItemDeleteRegister = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Delete Register", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemDeleteRegister )
		
		self.m_menuGrid.AppendSeparator()
		
		self.m_menuItemAddFunction = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Add Function", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemAddFunction )
		
		self.m_menuItemInsertFunction = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Insert Function", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemInsertFunction )
		
		self.m_menuItemDeleteFunction = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Delete Function", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemDeleteFunction )
		
		self.m_menuGrid.AppendSeparator()
		
		self.m_menuItemCopyFunction = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Copy Function", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemCopyFunction )
		
		self.m_menuItemPasteFunction = wx.MenuItem( self.m_menuGrid, wx.ID_ANY, u"Paste Function", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuGrid.AppendItem( self.m_menuItemPasteFunction )
		
		self.m_gridFBs.Bind( wx.EVT_RIGHT_DOWN, self.m_gridFBsOnContextMenu ) 
		
		bSizerFBs.Add( self.m_gridFBs, 0, wx.ALL, 5 )
		
		self.m_panelFBs.SetSizer( bSizerFBs )
		self.m_panelFBs.Layout()
		bSizerFBs.Fit( self.m_panelFBs )
		self.m_scrolledWindowFuncParams = wx.ScrolledWindow( self.m_splitterEditor, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.HSCROLL|wx.SIMPLE_BORDER|wx.VSCROLL )
		self.m_scrolledWindowFuncParams.SetScrollRate( 5, 5 )
		self.m_scrolledWindowFuncParams.SetMinSize( wx.Size( 300,-1 ) )
		self.m_scrolledWindowFuncParams.SetMaxSize( wx.Size( 350,-1 ) )
		
		bSizerFuncParams = wx.BoxSizer( wx.VERTICAL )
		
		bSizerFuncParams.SetMinSize( wx.Size( 275,-1 ) ) 
		self.m_staticText7 = wx.StaticText( self.m_scrolledWindowFuncParams, wx.ID_ANY, u"Function Type", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText7.Wrap( -1 )
		bSizerFuncParams.Add( self.m_staticText7, 0, wx.ALL, 5 )
		
		m_listBoxFuncTypeChoices = []
		self.m_listBoxFuncType = wx.ListBox( self.m_scrolledWindowFuncParams, wx.ID_ANY, wx.DefaultPosition, wx.Size( 250,100 ), m_listBoxFuncTypeChoices, wx.LB_NEEDED_SB|wx.LB_SINGLE|wx.LB_SORT )
		bSizerFuncParams.Add( self.m_listBoxFuncType, 0, wx.ALL, 5 )
		
		self.m_staticText10 = wx.StaticText( self.m_scrolledWindowFuncParams, wx.ID_ANY, u"Action", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText10.Wrap( -1 )
		bSizerFuncParams.Add( self.m_staticText10, 0, wx.ALL, 5 )
		
		m_comboActionChoices = [ u"SET", u"ADD", u"CLEAR" ]
		self.m_comboAction = wx.ComboBox( self.m_scrolledWindowFuncParams, wx.ID_ANY, u"CLEAR", wx.DefaultPosition, wx.Size( 250,-1 ), m_comboActionChoices, wx.CB_DROPDOWN )
		bSizerFuncParams.Add( self.m_comboAction, 0, wx.ALL, 5 )
		
		self.m_sliderParamValue = wx.Slider( self.m_scrolledWindowFuncParams, wx.ID_ANY, 0, -150, 150, wx.DefaultPosition, wx.Size( 250,-1 ), wx.SL_AUTOTICKS|wx.SL_BOTH|wx.SL_HORIZONTAL )
		self.m_sliderParamValue.Enable( False )
		
		bSizerFuncParams.Add( self.m_sliderParamValue, 0, wx.ALL, 5 )
		
		self.m_gridParameters = wx.grid.Grid( self.m_scrolledWindowFuncParams, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, 0 )
		
		# Grid
		self.m_gridParameters.CreateGrid( 16, 4 )
		self.m_gridParameters.EnableEditing( True )
		self.m_gridParameters.EnableGridLines( True )
		self.m_gridParameters.EnableDragGridSize( False )
		self.m_gridParameters.SetMargins( 0, 0 )
		
		# Columns
		self.m_gridParameters.SetColSize( 0, 80 )
		self.m_gridParameters.SetColSize( 1, 80 )
		self.m_gridParameters.SetColSize( 2, 80 )
		self.m_gridParameters.SetColSize( 3, 350 )
		self.m_gridParameters.EnableDragColMove( False )
		self.m_gridParameters.EnableDragColSize( True )
		self.m_gridParameters.SetColLabelSize( 30 )
		self.m_gridParameters.SetColLabelValue( 0, u"Setting" )
		self.m_gridParameters.SetColLabelValue( 1, u"Value" )
		self.m_gridParameters.SetColLabelValue( 2, u"Units" )
		self.m_gridParameters.SetColLabelValue( 3, u"Description" )
		self.m_gridParameters.SetColLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Rows
		self.m_gridParameters.EnableDragRowSize( True )
		self.m_gridParameters.SetRowLabelSize( 0 )
		self.m_gridParameters.SetRowLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Label Appearance
		
		# Cell Defaults
		self.m_gridParameters.SetDefaultCellAlignment( wx.ALIGN_LEFT, wx.ALIGN_TOP )
		bSizerFuncParams.Add( self.m_gridParameters, 0, wx.ALL, 5 )
		
		self.m_scrolledWindowFuncParams.SetSizer( bSizerFuncParams )
		self.m_scrolledWindowFuncParams.Layout()
		bSizerFuncParams.Fit( self.m_scrolledWindowFuncParams )
		self.m_splitterEditor.SplitVertically( self.m_panelFBs, self.m_scrolledWindowFuncParams, 521 )
		bSizer10.Add( self.m_splitterEditor, 1, wx.EXPAND, 5 )
		
		self.m_panel10.SetSizer( bSizer10 )
		self.m_panel10.Layout()
		bSizer10.Fit( self.m_panel10 )
		self.m_splitter8.SplitVertically( self.m_panel9, self.m_panel10, 132 )
		bSizer7.Add( self.m_splitter8, 1, wx.EXPAND, 5 )
		
		self.SetSizer( bSizer7 )
		self.Layout()
		self.m_statusBar = self.CreateStatusBar( 2, wx.ST_SIZEGRIP, wx.ID_ANY )
		self.m_menubar = wx.MenuBar( 0 )
		self.m_mnFile = wx.Menu()
		self.m_mniOpenSettings = wx.MenuItem( self.m_mnFile, wx.ID_ANY, u"Open Settings", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniOpenSettings )
		
		self.m_mniSaveSettings = wx.MenuItem( self.m_mnFile, wx.ID_ANY, u"Save Settings", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniSaveSettings )
		
		self.m_mniSaveSettingsAs = wx.MenuItem( self.m_mnFile, wx.ID_ANY, u"Save Settings As Backup", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniSaveSettingsAs )
		
		self.m_mnFile.AppendSeparator()
		
		self.m_mniExit = wx.MenuItem( self.m_mnFile, m_mniExitId, u"&Exit", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnFile.AppendItem( self.m_mniExit )
		
		self.m_menubar.Append( self.m_mnFile, u"&File" ) 
		
		self.m_menuActions = wx.Menu()
		self.m_menuItemExportCHeaders = wx.MenuItem( self.m_menuActions, wx.ID_ANY, u"Export C Headers", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuActions.AppendItem( self.m_menuItemExportCHeaders )
		
		self.m_menuActions.AppendSeparator()
		
		self.m_menuItemEditVirtualisation = wx.MenuItem( self.m_menuActions, wx.ID_ANY, u"Edit Virtual Table", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuActions.AppendItem( self.m_menuItemEditVirtualisation )
		
		self.m_menuActions.AppendSeparator()
		
		self.m_menuItemCommitToNVMem = wx.MenuItem( self.m_menuActions, wx.ID_ANY, u"Commit to NV memory", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuActions.AppendItem( self.m_menuItemCommitToNVMem )
		
		self.m_menuActions.AppendSeparator()
		
		self.m_menuItemClearNVmemory = wx.MenuItem( self.m_menuActions, wx.ID_ANY, u"Clear NV mem to defaults", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_menuActions.AppendItem( self.m_menuItemClearNVmemory )
		
		self.m_menubar.Append( self.m_menuActions, u"Actions" ) 
		
		self.m_mnHelp = wx.Menu()
		self.m_mniAbout = wx.MenuItem( self.m_mnHelp, m_mniAboutId, u"&About", wx.EmptyString, wx.ITEM_NORMAL )
		self.m_mnHelp.AppendItem( self.m_mniAbout )
		
		self.m_menubar.Append( self.m_mnHelp, u"&?" ) 
		
		self.SetMenuBar( self.m_menubar )
		
		
		# Connect Events
		self.Bind( wx.EVT_CLOSE, self.m_mniExitClick )
		self.m_panel9.Bind( wx.EVT_CHAR, self.m_btClick_Disconnect )
		self.m_btUpdate.Bind( wx.EVT_BUTTON, self.m_btClick_Update )
		self.m_checkBoxAutoUpdate.Bind( wx.EVT_CHECKBOX, self.m_chkBox_autoUpdate )
		self.m_buttonGenHeaders.Bind( wx.EVT_BUTTON, self.m_btnClick_GenCCode )
		self.m_buttonEditVirtual.Bind( wx.EVT_BUTTON, self.m_btnClick_EditVirtual )
		self.m_buttonSaveNV.Bind( wx.EVT_BUTTON, self.m_btnClick_SaveNVMem )
		self.m_buttonClearNVMem.Bind( wx.EVT_BUTTON, self.m_btnClick_ClearNVMem )
		self.m_buttonSave.Bind( wx.EVT_BUTTON, self.m_btnClick_Save )
		self.m_buttonSaveBackup.Bind( wx.EVT_BUTTON, self.m_btnClick_SaveBackup )
		self.m_panelFBs.Bind( wx.EVT_SIZE, self.m_panelFBsize )
		self.m_gridFBs.Bind( wx.grid.EVT_GRID_CELL_LEFT_DCLICK, self.m_FBs_cell_dclick )
		self.m_gridFBs.Bind( wx.grid.EVT_GRID_CELL_RIGHT_CLICK, self.m_FBs_right_click )
		self.m_gridFBs.Bind( wx.grid.EVT_GRID_LABEL_LEFT_CLICK, self.m_FBs_label_click )
		self.m_gridFBs.Bind( wx.grid.EVT_GRID_LABEL_LEFT_DCLICK, self.m_FBs_regEdit )
		self.m_gridFBs.Bind( wx.grid.EVT_GRID_SELECT_CELL, self.m_FBs_cell_click )
		self.Bind( wx.EVT_MENU, self.m_menuAddRegister, id = self.m_menuItemAddRegister.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuInsertRegister, id = self.m_menuItemInsertRegister.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuDeleteRegister, id = self.m_menuItemDeleteRegister.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuAddFunction, id = self.m_menuItemAddFunction.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuInsertFunction, id = self.m_menuItemInsertFunction.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuDeleteFunction, id = self.m_menuItemDeleteFunction.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuCopyFunction, id = self.m_menuItemCopyFunction.GetId() )
		self.Bind( wx.EVT_MENU, self.m_menuPasteFunction, id = self.m_menuItemPasteFunction.GetId() )
		self.m_listBoxFuncType.Bind( wx.EVT_LISTBOX_DCLICK, self.m_listBoxFuncTypeDClick )
		self.m_comboAction.Bind( wx.EVT_COMBOBOX, self.m_comboSetAction )
		self.m_sliderParamValue.Bind( wx.EVT_SCROLL, self.m_scrollParamValue )
		self.m_sliderParamValue.Bind( wx.EVT_SCROLL_THUMBRELEASE, self.m_scrollParamRelease )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_CELL_CHANGE, self.m_ParamsCellChange )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_EDITOR_HIDDEN, self.m_ParamsEditHide )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_EDITOR_SHOWN, self.m_ParamsEditShow )
		self.m_gridParameters.Bind( wx.grid.EVT_GRID_SELECT_CELL, self.m_ParamsCellSelect )
		self.Bind( wx.EVT_MENU, self.m_mniOpenSettingsClick, id = self.m_mniOpenSettings.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniSaveSettingsClick, id = self.m_mniSaveSettings.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniSaveSettingsAsClick, id = self.m_mniSaveSettingsAs.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniExitClick, id = self.m_mniExit.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mnExportCHeaders, id = self.m_menuItemExportCHeaders.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mnEditVirtualisation, id = self.m_menuItemEditVirtualisation.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mnCommitToNV, id = self.m_menuItemCommitToNVMem.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mnClearNV, id = self.m_menuItemClearNVmemory.GetId() )
		self.Bind( wx.EVT_MENU, self.m_mniAboutClick, id = self.m_mniAbout.GetId() )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def m_mniExitClick( self, event ):
		event.Skip()
	
	def m_btClick_Disconnect( self, event ):
		event.Skip()
	
	def m_btClick_Update( self, event ):
		event.Skip()
	
	def m_chkBox_autoUpdate( self, event ):
		event.Skip()
	
	def m_btnClick_GenCCode( self, event ):
		event.Skip()
	
	def m_btnClick_EditVirtual( self, event ):
		event.Skip()
	
	def m_btnClick_SaveNVMem( self, event ):
		event.Skip()
	
	def m_btnClick_ClearNVMem( self, event ):
		event.Skip()
	
	def m_btnClick_Save( self, event ):
		event.Skip()
	
	def m_btnClick_SaveBackup( self, event ):
		event.Skip()
	
	def m_panelFBsize( self, event ):
		event.Skip()
	
	def m_FBs_cell_dclick( self, event ):
		event.Skip()
	
	def m_FBs_right_click( self, event ):
		event.Skip()
	
	def m_FBs_label_click( self, event ):
		event.Skip()
	
	def m_FBs_regEdit( self, event ):
		event.Skip()
	
	def m_FBs_cell_click( self, event ):
		event.Skip()
	
	def m_menuAddRegister( self, event ):
		event.Skip()
	
	def m_menuInsertRegister( self, event ):
		event.Skip()
	
	def m_menuDeleteRegister( self, event ):
		event.Skip()
	
	def m_menuAddFunction( self, event ):
		event.Skip()
	
	def m_menuInsertFunction( self, event ):
		event.Skip()
	
	def m_menuDeleteFunction( self, event ):
		event.Skip()
	
	def m_menuCopyFunction( self, event ):
		event.Skip()
	
	def m_menuPasteFunction( self, event ):
		event.Skip()
	
	def m_listBoxFuncTypeDClick( self, event ):
		event.Skip()
	
	def m_comboSetAction( self, event ):
		event.Skip()
	
	def m_scrollParamValue( self, event ):
		event.Skip()
	
	def m_scrollParamRelease( self, event ):
		event.Skip()
	
	def m_ParamsCellChange( self, event ):
		event.Skip()
	
	def m_ParamsEditHide( self, event ):
		event.Skip()
	
	def m_ParamsEditShow( self, event ):
		event.Skip()
	
	def m_ParamsCellSelect( self, event ):
		event.Skip()
	
	def m_mniOpenSettingsClick( self, event ):
		event.Skip()
	
	def m_mniSaveSettingsClick( self, event ):
		event.Skip()
	
	def m_mniSaveSettingsAsClick( self, event ):
		event.Skip()
	
	
	def m_mnExportCHeaders( self, event ):
		event.Skip()
	
	def m_mnEditVirtualisation( self, event ):
		event.Skip()
	
	def m_mnCommitToNV( self, event ):
		event.Skip()
	
	def m_mnClearNV( self, event ):
		event.Skip()
	
	def m_mniAboutClick( self, event ):
		event.Skip()
	
	def m_splitter8OnIdle( self, event ):
		self.m_splitter8.SetSashPosition( 132 )
		self.m_splitter8.Unbind( wx.EVT_IDLE )
	
	def m_splitterEditorOnIdle( self, event ):
		self.m_splitterEditor.SetSashPosition( 521 )
		self.m_splitterEditor.Unbind( wx.EVT_IDLE )
	
	def m_gridFBsOnContextMenu( self, event ):
		self.m_gridFBs.PopupMenu( self.m_menuGrid, event.GetPosition() )
		

###########################################################################
## Class EditDialogBase
###########################################################################

class EditDialogBase ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Edit Register Name", pos = wx.DefaultPosition, size = wx.Size( 271,64 ), style = wx.DEFAULT_DIALOG_STYLE )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizer6 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_textCtrlRegNameEdit = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_PROCESS_ENTER )
		self.m_textCtrlRegNameEdit.SetMinSize( wx.Size( 250,-1 ) )
		
		bSizer6.Add( self.m_textCtrlRegNameEdit, 0, wx.ALL, 5 )
		
		self.SetSizer( bSizer6 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.m_textCtrlRegNameEdit.Bind( wx.EVT_TEXT_ENTER, self.m_FinishEdit )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def m_FinishEdit( self, event ):
		event.Skip()
	

###########################################################################
## Class VirtualDialogBase
###########################################################################

class VirtualDialogBase ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Virtual Table Editor", pos = wx.DefaultPosition, size = wx.Size( 457,617 ), style = wx.DEFAULT_DIALOG_STYLE|wx.ALWAYS_SHOW_SB|wx.VSCROLL )
		
		self.SetSizeHintsSz( wx.Size( 300,-1 ), wx.DefaultSize )
		
		bSizer7 = wx.BoxSizer( wx.VERTICAL )
		
		self.m_gridVirtual = wx.grid.Grid( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.VSCROLL )
		
		# Grid
		self.m_gridVirtual.CreateGrid( 32, 2 )
		self.m_gridVirtual.EnableEditing( True )
		self.m_gridVirtual.EnableGridLines( True )
		self.m_gridVirtual.EnableDragGridSize( False )
		self.m_gridVirtual.SetMargins( 0, 0 )
		
		# Columns
		self.m_gridVirtual.SetColSize( 0, 158 )
		self.m_gridVirtual.SetColSize( 1, 171 )
		self.m_gridVirtual.EnableDragColMove( False )
		self.m_gridVirtual.EnableDragColSize( True )
		self.m_gridVirtual.SetColLabelSize( 30 )
		self.m_gridVirtual.SetColLabelValue( 0, u"INPUT REGISTER" )
		self.m_gridVirtual.SetColLabelValue( 1, u"OUTPUT REGISTER" )
		self.m_gridVirtual.SetColLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Rows
		self.m_gridVirtual.EnableDragRowSize( False )
		self.m_gridVirtual.SetRowLabelSize( 80 )
		self.m_gridVirtual.SetRowLabelAlignment( wx.ALIGN_CENTRE, wx.ALIGN_CENTRE )
		
		# Label Appearance
		
		# Cell Defaults
		self.m_gridVirtual.SetDefaultCellAlignment( wx.ALIGN_LEFT, wx.ALIGN_TOP )
		bSizer7.Add( self.m_gridVirtual, 0, wx.ALL, 5 )
		
		self.SetSizer( bSizer7 )
		self.Layout()
		
		self.Centre( wx.VERTICAL )
		
		# Connect Events
		self.Bind( wx.EVT_CLOSE, self.m_closeDialog )
		self.Bind( wx.EVT_INIT_DIALOG, self.m_initDialog )
		self.Bind( wx.EVT_SIZE, self.m_OnSize )
		self.m_gridVirtual.Bind( wx.grid.EVT_GRID_CELL_CHANGE, self.on_gridCellChange )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def m_closeDialog( self, event ):
		event.Skip()
	
	def m_initDialog( self, event ):
		event.Skip()
	
	def m_OnSize( self, event ):
		event.Skip()
	
	def on_gridCellChange( self, event ):
		event.Skip()
	


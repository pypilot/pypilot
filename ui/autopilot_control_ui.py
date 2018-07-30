# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version Feb 20 2018)
## http://www.wxformbuilder.org/
##
## PLEASE DO *NOT* EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc
import wx.glcanvas

###########################################################################
## Class AutopilotControlBase
###########################################################################

class AutopilotControlBase ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Autopilot Control", pos = wx.DefaultPosition, size = wx.Size( -1,400 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		#self.SetSizeHints( -1, -1, wx.DefaultSize )
		
		fgSizer5 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer5.AddGrowableCol( 0 )
		fgSizer5.AddGrowableRow( 3 )
		fgSizer5.SetFlexibleDirection( wx.BOTH )
		fgSizer5.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer4 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer4.AddGrowableCol( 1 )
		fgSizer4.SetFlexibleDirection( wx.BOTH )
		fgSizer4.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.tbAP = wx.ToggleButton( self, wx.ID_ANY, u"AP", wx.DefaultPosition, wx.Size( 80,80 ), 0 )
		self.tbAP.SetFont( wx.Font( 36, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, "Sans" ) )
		
		fgSizer4.Add( self.tbAP, 0, wx.ALL, 5 )
		
		fgSizer20 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer20.AddGrowableCol( 0 )
		fgSizer20.SetFlexibleDirection( wx.BOTH )
		fgSizer20.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer21 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer21.AddGrowableCol( 0 )
		fgSizer21.AddGrowableCol( 1 )
		fgSizer21.AddGrowableCol( 2 )
		fgSizer21.AddGrowableCol( 3 )
		fgSizer21.SetFlexibleDirection( wx.BOTH )
		fgSizer21.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.stController = wx.StaticText( self, wx.ID_ANY, u"N/A", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stController.Wrap( -1 )
		fgSizer21.Add( self.stController, 0, wx.ALL, 5 )
		
		self.stEngaged = wx.StaticText( self, wx.ID_ANY, u"        N/A        ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stEngaged.Wrap( -1 )
		fgSizer21.Add( self.stEngaged, 0, wx.ALL, 5 )
		
		self.stStatus = wx.StaticText( self, wx.ID_ANY, u"N/A", wx.DefaultPosition, wx.Size( 100,-1 ), 0 )
		self.stStatus.Wrap( -1 )
		fgSizer21.Add( self.stStatus, 0, wx.ALIGN_CENTER_HORIZONTAL|wx.ALL|wx.EXPAND, 5 )
		
		self.stMode = wx.StaticText( self, wx.ID_ANY, u"        N/A        ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stMode.Wrap( -1 )
		fgSizer21.Add( self.stMode, 0, wx.ALIGN_RIGHT|wx.ALL, 5 )
		
		
		fgSizer20.Add( fgSizer21, 1, wx.EXPAND, 5 )
		
		fgSizer8 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer8.AddGrowableCol( 0 )
		fgSizer8.AddGrowableCol( 1 )
		fgSizer8.AddGrowableRow( 0 )
		fgSizer8.SetFlexibleDirection( wx.BOTH )
		fgSizer8.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.stHeadingCommand = wx.StaticText( self, wx.ID_ANY, u" N/A ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stHeadingCommand.Wrap( -1 )
		self.stHeadingCommand.SetFont( wx.Font( 36, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, "Sans" ) )
		
		fgSizer8.Add( self.stHeadingCommand, 0, wx.ALIGN_RIGHT|wx.ALL|wx.EXPAND, 5 )
		
		self.stHeading = wx.StaticText( self, wx.ID_ANY, u" N/A ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stHeading.Wrap( -1 )
		self.stHeading.SetFont( wx.Font( 36, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, False, "Sans" ) )
		
		fgSizer8.Add( self.stHeading, 0, wx.ALIGN_RIGHT|wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer20.Add( fgSizer8, 1, wx.EXPAND, 5 )
		
		
		fgSizer4.Add( fgSizer20, 1, wx.EXPAND, 5 )
		
		
		fgSizer5.Add( fgSizer4, 1, wx.EXPAND, 5 )
		
		fgSizer26 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer26.SetFlexibleDirection( wx.BOTH )
		fgSizer26.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.rbCompass = wx.RadioButton( self, wx.ID_ANY, u"Compass", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer26.Add( self.rbCompass, 0, wx.ALL, 5 )
		
		self.rbGPS = wx.RadioButton( self, wx.ID_ANY, u"GPS", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.rbGPS.Enable( False )
		
		fgSizer26.Add( self.rbGPS, 0, wx.ALL, 5 )
		
		self.rbWind = wx.RadioButton( self, wx.ID_ANY, u"Wind", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.rbWind.Enable( False )
		
		fgSizer26.Add( self.rbWind, 0, wx.ALL, 5 )
		
		self.rbTrueWind = wx.RadioButton( self, wx.ID_ANY, u"True Wind", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.rbTrueWind.Enable( False )
		
		fgSizer26.Add( self.rbTrueWind, 0, wx.ALL, 5 )
		
		
		fgSizer5.Add( fgSizer26, 1, wx.EXPAND, 5 )
		
		self.sCommand = wx.Slider( self, wx.ID_ANY, 0, -250, 250, wx.DefaultPosition, wx.Size( -1,-1 ), wx.SL_AUTOTICKS|wx.SL_HORIZONTAL )
		fgSizer5.Add( self.sCommand, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.swGains = wx.ScrolledWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.Size( -1,-1 ), wx.VSCROLL )
		self.swGains.SetScrollRate( 5, 5 )
		fgSizer23 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer23.AddGrowableRow( 0 )
		fgSizer23.SetFlexibleDirection( wx.BOTH )
		fgSizer23.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		
		self.swGains.SetSizer( fgSizer23 )
		self.swGains.Layout()
		fgSizer23.Fit( self.swGains )
		fgSizer5.Add( self.swGains, 1, wx.EXPAND |wx.ALL, 5 )
		
		fgSizer10 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer10.AddGrowableCol( 2 )
		fgSizer10.SetFlexibleDirection( wx.BOTH )
		fgSizer10.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.bScope = wx.Button( self, wx.ID_ANY, u"&Scope", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer10.Add( self.bScope, 0, wx.ALL, 5 )
		
		self.bClient = wx.Button( self, wx.ID_ANY, u"&Client", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer10.Add( self.bClient, 0, wx.ALL, 5 )
		
		self.bCalibration = wx.Button( self, wx.ID_ANY, u"C&alibration", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer10.Add( self.bCalibration, 0, wx.ALL, 5 )
		
		self.bClose = wx.Button( self, wx.ID_ANY, u"Close", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer10.Add( self.bClose, 0, wx.ALIGN_RIGHT|wx.ALL, 5 )
		
		
		fgSizer5.Add( fgSizer10, 1, wx.EXPAND, 5 )
		
		
		self.SetSizer( fgSizer5 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.tbAP.Bind( wx.EVT_TOGGLEBUTTON, self.onAP )
		self.rbCompass.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbGPS.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbWind.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbTrueWind.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.sCommand.Bind( wx.EVT_SCROLL, self.onCommand )
		self.sCommand.Bind( wx.EVT_UPDATE_UI, self.onPaintControlSlider )
		self.bScope.Bind( wx.EVT_BUTTON, self.onScope )
		self.bClient.Bind( wx.EVT_BUTTON, self.onClient )
		self.bCalibration.Bind( wx.EVT_BUTTON, self.onCalibration )
		self.bClose.Bind( wx.EVT_BUTTON, self.onClose )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def onAP( self, event ):
		event.Skip()
	
	def onMode( self, event ):
		event.Skip()
	
	
	
	
	def onCommand( self, event ):
		event.Skip()
	
	def onPaintControlSlider( self, event ):
		event.Skip()
	
	def onScope( self, event ):
		event.Skip()
	
	def onClient( self, event ):
		event.Skip()
	
	def onCalibration( self, event ):
		event.Skip()
	
	def onClose( self, event ):
		event.Skip()
	

###########################################################################
## Class CalibrationDialogBase
###########################################################################

class CalibrationDialogBase ( wx.Dialog ):
	
	def __init__( self, parent ):
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Calibration", pos = wx.DefaultPosition, size = wx.Size( 640,800 ), style = wx.DEFAULT_DIALOG_STYLE|wx.RESIZE_BORDER )
		
		self.SetSizeHints( -1, -1 )
		
		fgSizer7 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer7.AddGrowableCol( 0 )
		fgSizer7.AddGrowableRow( 0 )
		fgSizer7.SetFlexibleDirection( wx.BOTH )
		fgSizer7.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_notebook1 = wx.Notebook( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_panel7 = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer81 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer81.AddGrowableCol( 0 )
		fgSizer81.AddGrowableRow( 0 )
		fgSizer81.SetFlexibleDirection( wx.BOTH )
		fgSizer81.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.AccelCalibration = wx.glcanvas.GLCanvas(self.m_panel7, attribList=[ wx.glcanvas.WX_GL_RGBA, wx.glcanvas.WX_GL_DOUBLEBUFFER, wx.glcanvas.WX_GL_DEPTH_SIZE, 16, wx.glcanvas.WX_GL_STENCIL_SIZE, 8, 0 ])
		fgSizer81.Add( self.AccelCalibration, 0, wx.ALL|wx.EXPAND, 5 )
		
		fgSizer1911 = wx.FlexGridSizer( 0, 3, 0, 0 )
		fgSizer1911.AddGrowableCol( 0 )
		fgSizer1911.SetFlexibleDirection( wx.BOTH )
		fgSizer1911.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer101 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer101.AddGrowableCol( 1 )
		fgSizer101.SetFlexibleDirection( wx.BOTH )
		fgSizer101.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText101 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"Calibration", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText101.Wrap( -1 )
		fgSizer101.Add( self.m_staticText101, 0, wx.ALL, 5 )
		
		self.stAccelCal = wx.StaticText( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stAccelCal.Wrap( -1 )
		fgSizer101.Add( self.stAccelCal, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText141 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"Calibration Age", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText141.Wrap( -1 )
		fgSizer101.Add( self.m_staticText141, 0, wx.ALL, 5 )
		
		self.stAccelCalAge = wx.StaticText( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stAccelCalAge.Wrap( -1 )
		fgSizer101.Add( self.stAccelCalAge, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer1911.Add( fgSizer101, 1, wx.EXPAND, 5 )
		
		fgSizer271 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer271.SetFlexibleDirection( wx.BOTH )
		fgSizer271.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_button101 = wx.Button( self.m_panel7, wx.ID_ANY, u"Clear", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer271.Add( self.m_button101, 0, wx.ALL, 5 )
		
		self.cbAccelCalibrationLocked = wx.CheckBox( self.m_panel7, wx.ID_ANY, u"calibratiion locked", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer271.Add( self.cbAccelCalibrationLocked, 0, wx.ALL, 5 )
		
		
		fgSizer1911.Add( fgSizer271, 1, wx.EXPAND, 5 )
		
		m_sdbSizer11 = wx.StdDialogButtonSizer()
		self.m_sdbSizer11OK = wx.Button( self.m_panel7, wx.ID_OK )
		m_sdbSizer11.AddButton( self.m_sdbSizer11OK )
		m_sdbSizer11.Realize();
		
		fgSizer1911.Add( m_sdbSizer11, 1, wx.EXPAND, 5 )
		
		
		fgSizer81.Add( fgSizer1911, 1, wx.EXPAND, 5 )
		
		
		self.m_panel7.SetSizer( fgSizer81 )
		self.m_panel7.Layout()
		fgSizer81.Fit( self.m_panel7 )
		self.m_notebook1.AddPage( self.m_panel7, u"accel", True )
		self.m_panel1 = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer8 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer8.AddGrowableCol( 0 )
		fgSizer8.AddGrowableRow( 0 )
		fgSizer8.SetFlexibleDirection( wx.BOTH )
		fgSizer8.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.CompassCalibration = wx.glcanvas.GLCanvas(self.m_panel1, attribList=[ wx.glcanvas.WX_GL_RGBA, wx.glcanvas.WX_GL_DOUBLEBUFFER, wx.glcanvas.WX_GL_DEPTH_SIZE, 16, wx.glcanvas.WX_GL_STENCIL_SIZE, 8, 0 ])
		fgSizer8.Add( self.CompassCalibration, 0, wx.ALL|wx.EXPAND, 5 )
		
		fgSizer191 = wx.FlexGridSizer( 0, 3, 0, 0 )
		fgSizer191.AddGrowableCol( 0 )
		fgSizer191.SetFlexibleDirection( wx.BOTH )
		fgSizer191.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer10 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer10.AddGrowableCol( 1 )
		fgSizer10.SetFlexibleDirection( wx.BOTH )
		fgSizer10.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText10 = wx.StaticText( self.m_panel1, wx.ID_ANY, u"Calibration", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText10.Wrap( -1 )
		fgSizer10.Add( self.m_staticText10, 0, wx.ALL, 5 )
		
		self.stCompassCal = wx.StaticText( self.m_panel1, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stCompassCal.Wrap( -1 )
		fgSizer10.Add( self.stCompassCal, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText14 = wx.StaticText( self.m_panel1, wx.ID_ANY, u"Calibration Age", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText14.Wrap( -1 )
		fgSizer10.Add( self.m_staticText14, 0, wx.ALL, 5 )
		
		self.stCompassCalAge = wx.StaticText( self.m_panel1, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stCompassCalAge.Wrap( -1 )
		fgSizer10.Add( self.stCompassCalAge, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer191.Add( fgSizer10, 1, wx.EXPAND, 5 )
		
		fgSizer27 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer27.SetFlexibleDirection( wx.BOTH )
		fgSizer27.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_button10 = wx.Button( self.m_panel1, wx.ID_ANY, u"Clear", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer27.Add( self.m_button10, 0, wx.ALL, 5 )
		
		self.cbCompassCalibrationLocked = wx.CheckBox( self.m_panel1, wx.ID_ANY, u"calibratiion locked", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer27.Add( self.cbCompassCalibrationLocked, 0, wx.ALL, 5 )
		
		
		fgSizer191.Add( fgSizer27, 1, wx.EXPAND, 5 )
		
		m_sdbSizer1 = wx.StdDialogButtonSizer()
		self.m_sdbSizer1OK = wx.Button( self.m_panel1, wx.ID_OK )
		m_sdbSizer1.AddButton( self.m_sdbSizer1OK )
		m_sdbSizer1.Realize();
		
		fgSizer191.Add( m_sdbSizer1, 1, wx.EXPAND, 5 )
		
		
		fgSizer8.Add( fgSizer191, 1, wx.EXPAND, 5 )
		
		
		self.m_panel1.SetSizer( fgSizer8 )
		self.m_panel1.Layout()
		fgSizer8.Fit( self.m_panel1 )
		self.m_notebook1.AddPage( self.m_panel1, u"compass", False )
		self.m_panel3 = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer12 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer12.AddGrowableCol( 0 )
		fgSizer12.AddGrowableRow( 0 )
		fgSizer12.SetFlexibleDirection( wx.BOTH )
		fgSizer12.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_splitter1 = wx.SplitterWindow( self.m_panel3, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SP_3D|wx.SP_LIVE_UPDATE )
		self.m_splitter1.SetSashGravity( 1 )
		self.m_splitter1.SetMinimumPaneSize( 220 )
		
		self.m_panel4 = wx.Panel( self.m_splitter1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer22 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer22.AddGrowableCol( 0 )
		fgSizer22.AddGrowableRow( 0 )
		fgSizer22.SetFlexibleDirection( wx.BOTH )
		fgSizer22.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.BoatPlot = wx.glcanvas.GLCanvas(self.m_panel3, attribList=[ wx.glcanvas.WX_GL_RGBA, wx.glcanvas.WX_GL_DOUBLEBUFFER, wx.glcanvas.WX_GL_DEPTH_SIZE, 16, wx.glcanvas.WX_GL_STENCIL_SIZE, 8, 0 ])
		fgSizer22.Add( self.BoatPlot, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		self.m_panel4.SetSizer( fgSizer22 )
		self.m_panel4.Layout()
		fgSizer22.Fit( self.m_panel4 )
		self.m_panel5 = wx.Panel( self.m_splitter1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer31 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer31.AddGrowableCol( 0 )
		fgSizer31.AddGrowableRow( 0 )
		fgSizer31.SetFlexibleDirection( wx.BOTH )
		fgSizer31.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer29 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer29.AddGrowableCol( 0 )
		fgSizer29.SetFlexibleDirection( wx.BOTH )
		fgSizer29.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer18 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer18.SetFlexibleDirection( wx.BOTH )
		fgSizer18.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText16 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Alignment", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText16.Wrap( -1 )
		fgSizer18.Add( self.m_staticText16, 0, wx.ALL, 5 )
		
		self.bReset = wx.Button( self.m_panel5, wx.ID_ANY, u"Reset", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer18.Add( self.bReset, 0, wx.ALL, 5 )
		
		self.stAlignment = wx.StaticText( self.m_panel5, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stAlignment.Wrap( -1 )
		fgSizer18.Add( self.stAlignment, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer29.Add( fgSizer18, 1, wx.EXPAND, 5 )
		
		fgSizer19 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer19.AddGrowableCol( 2 )
		fgSizer19.SetFlexibleDirection( wx.BOTH )
		fgSizer19.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText19 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Pitch", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText19.Wrap( -1 )
		fgSizer19.Add( self.m_staticText19, 0, wx.ALL, 5 )
		
		self.stPitch = wx.StaticText( self.m_panel5, wx.ID_ANY, u"N/A", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stPitch.Wrap( -1 )
		fgSizer19.Add( self.stPitch, 0, wx.ALL, 5 )
		
		self.gAlignment = wx.Gauge( self.m_panel5, wx.ID_ANY, 100, wx.DefaultPosition, wx.DefaultSize, wx.GA_HORIZONTAL )
		self.gAlignment.SetValue( 0 ) 
		fgSizer19.Add( self.gAlignment, 0, wx.ALIGN_RIGHT|wx.ALL, 5 )
		
		
		fgSizer29.Add( fgSizer19, 1, wx.EXPAND, 5 )
		
		fgSizer13 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer13.SetFlexibleDirection( wx.BOTH )
		fgSizer13.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText34 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Roll", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText34.Wrap( -1 )
		fgSizer13.Add( self.m_staticText34, 0, wx.ALL, 5 )
		
		self.stRoll = wx.StaticText( self.m_panel5, wx.ID_ANY, u"     N/A     ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRoll.Wrap( -1 )
		fgSizer13.Add( self.stRoll, 0, wx.ALL, 5 )
		
		self.m_staticText18 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Heel", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText18.Wrap( -1 )
		fgSizer13.Add( self.m_staticText18, 0, wx.ALL, 5 )
		
		self.stHeel = wx.StaticText( self.m_panel5, wx.ID_ANY, u"     N/A     ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stHeel.Wrap( -1 )
		fgSizer13.Add( self.stHeel, 0, wx.ALL, 5 )
		
		self.bLevel = wx.Button( self.m_panel5, wx.ID_ANY, u"Boat is level", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer13.Add( self.bLevel, 0, wx.ALL, 5 )
		
		
		fgSizer29.Add( fgSizer13, 1, wx.EXPAND, 5 )
		
		fgSizer14 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer14.SetFlexibleDirection( wx.BOTH )
		fgSizer14.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText22 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Heading", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText22.Wrap( -1 )
		fgSizer14.Add( self.m_staticText22, 0, wx.ALL, 5 )
		
		self.stHeading = wx.StaticText( self.m_panel5, wx.ID_ANY, u"     N/A     ", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stHeading.Wrap( -1 )
		fgSizer14.Add( self.stHeading, 0, wx.ALL, 5 )
		
		self.m_staticText25 = wx.StaticText( self.m_panel5, wx.ID_ANY, u"Offset", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText25.Wrap( -1 )
		fgSizer14.Add( self.m_staticText25, 0, wx.ALL, 5 )
		
		self.sHeadingOffset = wx.SpinCtrl( self.m_panel5, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.SP_ARROW_KEYS, -180, 180, -3 )
		fgSizer14.Add( self.sHeadingOffset, 0, wx.ALL, 5 )
		
		
		fgSizer29.Add( fgSizer14, 1, wx.EXPAND, 5 )
		
		
		fgSizer31.Add( fgSizer29, 1, wx.EXPAND, 5 )
		
		fgSizer23 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer23.SetFlexibleDirection( wx.BOTH )
		fgSizer23.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		sbSizer4 = wx.StaticBoxSizer( wx.StaticBox( self.m_panel5, wx.ID_ANY, u"Coords" ), wx.VERTICAL )
		
		cCoordsChoices = [ u"Sea", u"Boat", u"Compass" ]
		self.cCoords = wx.Choice( sbSizer4.GetStaticBox(), wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, cCoordsChoices, 0 )
		self.cCoords.SetSelection( 2 )
		sbSizer4.Add( self.cCoords, 0, wx.ALL, 5 )
		
		
		fgSizer23.Add( sbSizer4, 1, wx.EXPAND, 5 )
		
		self.cbTextureCompass = wx.CheckBox( self.m_panel5, wx.ID_ANY, u"tex compass", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.cbTextureCompass.SetValue(True) 
		fgSizer23.Add( self.cbTextureCompass, 0, wx.ALL, 5 )
		
		
		fgSizer31.Add( fgSizer23, 1, wx.EXPAND, 5 )
		
		
		self.m_panel5.SetSizer( fgSizer31 )
		self.m_panel5.Layout()
		fgSizer31.Fit( self.m_panel5 )
		self.m_splitter1.SplitHorizontally( self.m_panel4, self.m_panel5, -1 )
		fgSizer12.Add( self.m_splitter1, 1, wx.EXPAND, 5 )
		
		fgSizer181 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer181.AddGrowableCol( 1 )
		fgSizer181.SetFlexibleDirection( wx.BOTH )
		fgSizer181.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.bIMUScope = wx.Button( self.m_panel3, wx.ID_ANY, u"Scope", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer181.Add( self.bIMUScope, 0, wx.ALL, 5 )
		
		m_sdbSizer2 = wx.StdDialogButtonSizer()
		self.m_sdbSizer2OK = wx.Button( self.m_panel3, wx.ID_OK )
		m_sdbSizer2.AddButton( self.m_sdbSizer2OK )
		m_sdbSizer2.Realize();
		
		fgSizer181.Add( m_sdbSizer2, 1, wx.ALIGN_RIGHT|wx.EXPAND, 5 )
		
		
		fgSizer12.Add( fgSizer181, 1, wx.EXPAND, 5 )
		
		
		self.m_panel3.SetSizer( fgSizer12 )
		self.m_panel3.Layout()
		fgSizer12.Fit( self.m_panel3 )
		self.m_notebook1.AddPage( self.m_panel3, u"imu", False )
		self.m_panel2 = wx.Panel( self.m_notebook1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer9 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer9.AddGrowableCol( 0 )
		fgSizer9.AddGrowableRow( 1 )
		fgSizer9.SetFlexibleDirection( wx.BOTH )
		fgSizer9.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		sbSizer1 = wx.StaticBoxSizer( wx.StaticBox( self.m_panel2, wx.ID_ANY, u"Calibration" ), wx.VERTICAL )
		
		self.stServoCalibration = wx.StaticText( sbSizer1.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stServoCalibration.Wrap( 400 )
		sbSizer1.Add( self.stServoCalibration, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer9.Add( sbSizer1, 1, wx.EXPAND, 5 )
		
		self.m_scrolledWindow1 = wx.ScrolledWindow( self.m_panel2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.HSCROLL|wx.VSCROLL )
		self.m_scrolledWindow1.SetScrollRate( 5, 5 )
		fgSizer16 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer16.AddGrowableCol( 0 )
		fgSizer16.AddGrowableRow( 0 )
		fgSizer16.SetFlexibleDirection( wx.BOTH )
		fgSizer16.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.stServoCalibrationConsole = wx.StaticText( self.m_scrolledWindow1, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stServoCalibrationConsole.Wrap( -1 )
		fgSizer16.Add( self.stServoCalibrationConsole, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		self.m_scrolledWindow1.SetSizer( fgSizer16 )
		self.m_scrolledWindow1.Layout()
		fgSizer16.Fit( self.m_scrolledWindow1 )
		fgSizer9.Add( self.m_scrolledWindow1, 1, wx.EXPAND |wx.ALL, 5 )
		
		fgSizer17 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer17.AddGrowableCol( 1 )
		fgSizer17.SetFlexibleDirection( wx.BOTH )
		fgSizer17.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText36 = wx.StaticText( self.m_panel2, wx.ID_ANY, u"Calibration Mode", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText36.Wrap( -1 )
		fgSizer17.Add( self.m_staticText36, 0, wx.ALL, 5 )
		
		self.stServoCalibrationMode = wx.StaticText( self.m_panel2, wx.ID_ANY, u"N/A", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stServoCalibrationMode.Wrap( -1 )
		fgSizer17.Add( self.stServoCalibrationMode, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer9.Add( fgSizer17, 1, wx.EXPAND, 5 )
		
		fgSizer15 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer15.SetFlexibleDirection( wx.BOTH )
		fgSizer15.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.bCalibrateServo = wx.Button( self.m_panel2, wx.ID_ANY, u"Calibrate Servo", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer15.Add( self.bCalibrateServo, 0, wx.ALL, 5 )
		
		self.m_staticText30 = wx.StaticText( self.m_panel2, wx.ID_ANY, u"Max Current", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText30.Wrap( -1 )
		fgSizer15.Add( self.m_staticText30, 0, wx.ALL, 5 )
		
		self.dsServoMaxCurrent = wx.SpinCtrlDouble(self.m_panel2)
		self.dsServoMaxCurrent.SetMinSize( wx.Size( 60,30 ) )
		
		fgSizer15.Add( self.dsServoMaxCurrent, 0, wx.ALL, 5 )
		
		self.m_staticText32 = wx.StaticText( self.m_panel2, wx.ID_ANY, u"Amps", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText32.Wrap( -1 )
		fgSizer15.Add( self.m_staticText32, 0, wx.ALL, 5 )
		
		
		fgSizer9.Add( fgSizer15, 1, wx.EXPAND, 5 )
		
		m_sdbSizer3 = wx.StdDialogButtonSizer()
		self.m_sdbSizer3OK = wx.Button( self.m_panel2, wx.ID_OK )
		m_sdbSizer3.AddButton( self.m_sdbSizer3OK )
		m_sdbSizer3.Realize();
		
		fgSizer9.Add( m_sdbSizer3, 1, wx.EXPAND, 5 )
		
		
		self.m_panel2.SetSizer( fgSizer9 )
		self.m_panel2.Layout()
		fgSizer9.Fit( self.m_panel2 )
		self.m_notebook1.AddPage( self.m_panel2, u"servo", False )
		
		fgSizer7.Add( self.m_notebook1, 1, wx.EXPAND |wx.ALL, 5 )
		
		
		self.SetSizer( fgSizer7 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.AccelCalibration.Bind( wx.EVT_KEY_DOWN, self.onKeyPressAccel )
		self.AccelCalibration.Bind( wx.EVT_LEFT_DOWN, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_LEFT_UP, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_MIDDLE_DOWN, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_MIDDLE_UP, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_RIGHT_DOWN, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_RIGHT_UP, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_MOTION, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_LEFT_DCLICK, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_MIDDLE_DCLICK, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_RIGHT_DCLICK, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_LEAVE_WINDOW, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_ENTER_WINDOW, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_MOUSEWHEEL, self.onMouseEventsAccel )
		self.AccelCalibration.Bind( wx.EVT_PAINT, self.onPaintGLAccel )
		self.AccelCalibration.Bind( wx.EVT_SIZE, self.onSizeGLAccel )
		self.m_button101.Bind( wx.EVT_BUTTON, self.onClearAccel )
		self.cbAccelCalibrationLocked.Bind( wx.EVT_CHECKBOX, self.onAccelCalibrationLocked )
		self.CompassCalibration.Bind( wx.EVT_KEY_DOWN, self.onKeyPressCompass )
		self.CompassCalibration.Bind( wx.EVT_LEFT_DOWN, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_LEFT_UP, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_MIDDLE_DOWN, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_MIDDLE_UP, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_RIGHT_DOWN, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_RIGHT_UP, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_MOTION, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_LEFT_DCLICK, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_MIDDLE_DCLICK, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_RIGHT_DCLICK, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_LEAVE_WINDOW, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_ENTER_WINDOW, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_MOUSEWHEEL, self.onMouseEventsCompass )
		self.CompassCalibration.Bind( wx.EVT_PAINT, self.onPaintGLCompass )
		self.CompassCalibration.Bind( wx.EVT_SIZE, self.onSizeGLCompass )
		self.m_button10.Bind( wx.EVT_BUTTON, self.onClearCompass )
		self.cbCompassCalibrationLocked.Bind( wx.EVT_CHECKBOX, self.onCompassCalibrationLocked )
		self.BoatPlot.Bind( wx.EVT_KEY_DOWN, self.onKeyPressBoatPlot )
		self.BoatPlot.Bind( wx.EVT_LEFT_DOWN, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_LEFT_UP, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_MIDDLE_DOWN, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_MIDDLE_UP, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_RIGHT_DOWN, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_RIGHT_UP, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_MOTION, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_LEFT_DCLICK, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_MIDDLE_DCLICK, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_RIGHT_DCLICK, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_LEAVE_WINDOW, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_ENTER_WINDOW, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_MOUSEWHEEL, self.onMouseEventsBoatPlot )
		self.BoatPlot.Bind( wx.EVT_PAINT, self.onPaintGLBoatPlot )
		self.BoatPlot.Bind( wx.EVT_SIZE, self.onSizeGLBoatPlot )
		self.bReset.Bind( wx.EVT_BUTTON, self.onResetAlignment )
		self.bLevel.Bind( wx.EVT_BUTTON, self.onLevel )
		self.sHeadingOffset.Bind( wx.EVT_SPINCTRL, self.onIMUHeadingOffset )
		self.cbTextureCompass.Bind( wx.EVT_CHECKBOX, self.onTextureCompass )
		self.bIMUScope.Bind( wx.EVT_BUTTON, self.onIMUScope )
		self.bCalibrateServo.Bind( wx.EVT_BUTTON, self.onCalibrateServo )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def onKeyPressAccel( self, event ):
		event.Skip()
	
	def onMouseEventsAccel( self, event ):
		event.Skip()
	
	def onPaintGLAccel( self, event ):
		event.Skip()
	
	def onSizeGLAccel( self, event ):
		event.Skip()
	
	def onClearAccel( self, event ):
		event.Skip()
	
	def onAccelCalibrationLocked( self, event ):
		event.Skip()
	
	def onKeyPressCompass( self, event ):
		event.Skip()
	
	def onMouseEventsCompass( self, event ):
		event.Skip()
	
	def onPaintGLCompass( self, event ):
		event.Skip()
	
	def onSizeGLCompass( self, event ):
		event.Skip()
	
	def onClearCompass( self, event ):
		event.Skip()
	
	def onCompassCalibrationLocked( self, event ):
		event.Skip()
	
	def onKeyPressBoatPlot( self, event ):
		event.Skip()
	
	def onMouseEventsBoatPlot( self, event ):
		event.Skip()
	
	def onPaintGLBoatPlot( self, event ):
		event.Skip()
	
	def onSizeGLBoatPlot( self, event ):
		event.Skip()
	
	def onResetAlignment( self, event ):
		event.Skip()
	
	def onLevel( self, event ):
		event.Skip()
	
	def onIMUHeadingOffset( self, event ):
		event.Skip()
	
	def onTextureCompass( self, event ):
		event.Skip()
	
	def onIMUScope( self, event ):
		event.Skip()
	
	def onCalibrateServo( self, event ):
		event.Skip()
	


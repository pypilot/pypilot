# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version May 19 2018)
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
		
		#( wx.Size( -1,-1 ), wx.DefaultSize )
		
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
		
		self.stRudder = wx.StaticText( self, wx.ID_ANY, u"----", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudder.Wrap( -1 )
		fgSizer21.Add( self.stRudder, 0, wx.ALL, 5 )
		
		
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
		
		self.m_staticText52 = wx.StaticText( self, wx.ID_ANY, u"Pilot", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText52.Wrap( -1 )
		fgSizer26.Add( self.m_staticText52, 0, wx.ALL, 5 )
		
		cPilotChoices = []
		self.cPilot = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, cPilotChoices, 0 )
		self.cPilot.SetSelection( 0 )
		fgSizer26.Add( self.cPilot, 0, wx.ALL, 5 )
		
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
		
		fgSizer39 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer39.AddGrowableCol( 0 )
		fgSizer39.SetFlexibleDirection( wx.BOTH )
		fgSizer39.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.sCommand = wx.Slider( self, wx.ID_ANY, 0, -250, 250, wx.DefaultPosition, wx.Size( -1,-1 ), wx.SL_AUTOTICKS|wx.SL_HORIZONTAL )
		fgSizer39.Add( self.sCommand, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.bCenter = wx.Button( self, wx.ID_ANY, u"Center", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer39.Add( self.bCenter, 0, wx.ALL, 5 )
		
		
		fgSizer5.Add( fgSizer39, 1, wx.EXPAND, 5 )
		
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
		self.cPilot.Bind( wx.EVT_CHOICE, self.onPilot )
		self.rbCompass.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbGPS.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbWind.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.rbTrueWind.Bind( wx.EVT_RADIOBUTTON, self.onMode )
		self.sCommand.Bind( wx.EVT_SCROLL, self.onCommand )
		self.sCommand.Bind( wx.EVT_UPDATE_UI, self.onPaintControlSlider )
		self.bCenter.Bind( wx.EVT_BUTTON, self.onCenter )
		self.bScope.Bind( wx.EVT_BUTTON, self.onScope )
		self.bClient.Bind( wx.EVT_BUTTON, self.onClient )
		self.bCalibration.Bind( wx.EVT_BUTTON, self.onCalibration )
		self.bClose.Bind( wx.EVT_BUTTON, self.onClose )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def onAP( self, event ):
		event.Skip()
	
	def onPilot( self, event ):
		event.Skip()
	
	def onMode( self, event ):
		event.Skip()
	
	
	
	
	def onCommand( self, event ):
		event.Skip()
	
	def onPaintControlSlider( self, event ):
		event.Skip()
	
	def onCenter( self, event ):
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
		wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Calibration", pos = wx.DefaultPosition, size = wx.Size( 640,850 ), style = wx.DEFAULT_DIALOG_STYLE|wx.RESIZE_BORDER )
		
		#( wx.DefaultSize, wx.DefaultSize )
		
		fgSizer7 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer7.AddGrowableCol( 0 )
		fgSizer7.AddGrowableRow( 0 )
		fgSizer7.SetFlexibleDirection( wx.BOTH )
		fgSizer7.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_notebook = wx.Notebook( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_panel3 = wx.Panel( self.m_notebook, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
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
		self.m_notebook.AddPage( self.m_panel3, u"imu", False )
		self.m_panel7 = wx.Panel( self.m_notebook, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
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
		self.m_notebook.AddPage( self.m_panel7, u"accel", False )
		self.m_panel1 = wx.Panel( self.m_notebook, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
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
		self.m_notebook.AddPage( self.m_panel1, u"compass", False )
		self.m_panel2 = wx.Panel( self.m_notebook, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer9 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer9.AddGrowableCol( 0 )
		fgSizer9.AddGrowableRow( 0 )
		fgSizer9.SetFlexibleDirection( wx.BOTH )
		fgSizer9.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_swservo = wx.ScrolledWindow( self.m_panel2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.HSCROLL|wx.VSCROLL )
		self.m_swservo.SetScrollRate( 5, 5 )
		fgSizer351 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer351.AddGrowableCol( 0 )
		fgSizer351.AddGrowableRow( 0 )
		fgSizer351.AddGrowableRow( 1 )
		fgSizer351.AddGrowableRow( 2 )
		fgSizer351.SetFlexibleDirection( wx.BOTH )
		fgSizer351.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		sbSizer1 = wx.StaticBoxSizer( wx.StaticBox( self.m_swservo, wx.ID_ANY, u"Calibration" ), wx.VERTICAL )
		
		self.stServoCalibration = wx.StaticText( sbSizer1.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stServoCalibration.Wrap( 400 )
		sbSizer1.Add( self.stServoCalibration, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		fgSizer351.Add( sbSizer1, 1, wx.EXPAND, 5 )
		
		sbSizer3 = wx.StaticBoxSizer( wx.StaticBox( self.m_swservo, wx.ID_ANY, u"Rudder Feedback" ), wx.VERTICAL )
		
		fgSizer35 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer35.SetFlexibleDirection( wx.BOTH )
		fgSizer35.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		fgSizer36 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer36.SetFlexibleDirection( wx.BOTH )
		fgSizer36.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText41 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"To calibrate the rudder feedback, move the rudder and press each button.", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText41.Wrap( 550 )
		fgSizer36.Add( self.m_staticText41, 0, wx.ALL, 5 )
		
		
		fgSizer35.Add( fgSizer36, 1, wx.EXPAND, 5 )
		
		fgSizer32 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer32.AddGrowableCol( 1 )
		fgSizer32.AddGrowableCol( 2 )
		fgSizer32.SetFlexibleDirection( wx.BOTH )
		fgSizer32.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText38 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Rudder", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText38.Wrap( -1 )
		fgSizer32.Add( self.m_staticText38, 0, wx.ALL, 5 )
		
		self.stRudderAngle = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"N/A", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudderAngle.Wrap( -1 )
		fgSizer32.Add( self.stRudderAngle, 0, wx.ALL, 5 )
		
		self.stServoFlags = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stServoFlags.Wrap( -1 )
		fgSizer32.Add( self.stServoFlags, 0, wx.ALL, 5 )
		
		
		fgSizer35.Add( fgSizer32, 1, wx.EXPAND, 5 )
		
		fgSizer33 = wx.FlexGridSizer( 0, 3, 0, 0 )
		fgSizer33.SetFlexibleDirection( wx.BOTH )
		fgSizer33.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_button11 = wx.Button( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Rudder Is Centered", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer33.Add( self.m_button11, 0, wx.ALL, 5 )
		
		self.m_staticText341 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Offset", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText341.Wrap( -1 )
		fgSizer33.Add( self.m_staticText341, 0, wx.ALL, 5 )
		
		self.stRudderOffset = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"------", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudderOffset.Wrap( -1 )
		fgSizer33.Add( self.stRudderOffset, 0, wx.ALL, 5 )
		
		self.m_button12 = wx.Button( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Rudder is Starboard Range", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer33.Add( self.m_button12, 0, wx.ALL, 5 )
		
		self.m_staticText56 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Scale", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText56.Wrap( -1 )
		fgSizer33.Add( self.m_staticText56, 0, wx.ALL, 5 )
		
		self.stRudderScale = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"------", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudderScale.Wrap( -1 )
		fgSizer33.Add( self.stRudderScale, 0, wx.ALL, 5 )
		
		self.m_button121 = wx.Button( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Rudder is Port Range", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer33.Add( self.m_button121, 0, wx.ALL, 5 )
		
		self.m_staticText58 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"Non Linearity", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText58.Wrap( -1 )
		fgSizer33.Add( self.m_staticText58, 0, wx.ALL, 5 )
		
		self.stRudderNonlinearity = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"------", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudderNonlinearity.Wrap( -1 )
		fgSizer33.Add( self.stRudderNonlinearity, 0, wx.ALL, 5 )
		
		
		fgSizer35.Add( fgSizer33, 1, wx.EXPAND, 5 )
		
		fgSizer34 = wx.FlexGridSizer( 0, 4, 0, 0 )
		fgSizer34.SetFlexibleDirection( wx.BOTH )
		fgSizer34.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.stRudderRange = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u" range +-", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.stRudderRange.Wrap( -1 )
		fgSizer34.Add( self.stRudderRange, 0, wx.ALL, 5 )
		
		self.sRudderRange = wx.SpinCtrl( sbSizer3.GetStaticBox(), wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.SP_ARROW_KEYS, 10, 90, 30 )
		fgSizer34.Add( self.sRudderRange, 0, wx.ALL, 5 )
		
		self.m_staticText40 = wx.StaticText( sbSizer3.GetStaticBox(), wx.ID_ANY, u"degrees", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText40.Wrap( -1 )
		fgSizer34.Add( self.m_staticText40, 0, wx.ALL, 5 )
		
		
		fgSizer35.Add( fgSizer34, 1, wx.EXPAND, 5 )
		
		
		sbSizer3.Add( fgSizer35, 1, wx.EXPAND, 5 )
		
		
		fgSizer351.Add( sbSizer3, 1, wx.EXPAND, 5 )
		
		fgSizer15 = wx.FlexGridSizer( 0, 3, 0, 0 )
		fgSizer15.AddGrowableCol( 1 )
		fgSizer15.SetFlexibleDirection( wx.BOTH )
		fgSizer15.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText30 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Max Current", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText30.Wrap( -1 )
		fgSizer15.Add( self.m_staticText30, 0, wx.ALL, 5 )
		
		self.dsServoMaxCurrent = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoMaxCurrent.SetMinSize( wx.Size( 60,30 ) )
		
		fgSizer15.Add( self.dsServoMaxCurrent, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText32 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Amps", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText32.Wrap( -1 )
		fgSizer15.Add( self.m_staticText32, 0, wx.ALL, 5 )
		
		self.m_staticText305 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Max Controller Temperature", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText305.Wrap( -1 )
		fgSizer15.Add( self.m_staticText305, 0, wx.ALL, 5 )
		
		self.dsServoMaxControllerTemp = wx.SpinCtrl(self.m_swservo)
		self.dsServoMaxControllerTemp.SetMinSize( wx.Size( 60,30 ) )
		
		fgSizer15.Add( self.dsServoMaxControllerTemp, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText325 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"°C", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText325.Wrap( -1 )
		fgSizer15.Add( self.m_staticText325, 0, wx.ALL, 5 )
		
		self.m_staticText3051 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Max Motor Temperature", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText3051.Wrap( -1 )
		fgSizer15.Add( self.m_staticText3051, 0, wx.ALL, 5 )
		
		self.dsServoMaxMotorTemp = wx.SpinCtrl(self.m_swservo)
		self.dsServoMaxMotorTemp.SetMinSize( wx.Size( 60,30 ) )
		
		fgSizer15.Add( self.dsServoMaxMotorTemp, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText3251 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"°C", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText3251.Wrap( -1 )
		fgSizer15.Add( self.m_staticText3251, 0, wx.ALL, 5 )
		
		self.m_staticText304 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Max Slew Speed", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText304.Wrap( -1 )
		fgSizer15.Add( self.m_staticText304, 0, wx.ALL, 5 )
		
		self.dsServoMaxSlewSpeed = wx.SpinCtrl(self.m_swservo)
		self.dsServoMaxSlewSpeed.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer15.Add( self.dsServoMaxSlewSpeed, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText324 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"steps", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText324.Wrap( -1 )
		fgSizer15.Add( self.m_staticText324, 0, wx.ALL, 5 )
		
		self.m_staticText3041 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Max Slew Slow", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText3041.Wrap( -1 )
		fgSizer15.Add( self.m_staticText3041, 0, wx.ALL, 5 )
		
		self.dsServoMaxSlewSlow = wx.SpinCtrl(self.m_swservo)
		self.dsServoMaxSlewSlow.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer15.Add( self.dsServoMaxSlewSlow, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_staticText3241 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"steps", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText3241.Wrap( -1 )
		fgSizer15.Add( self.m_staticText3241, 0, wx.ALL, 5 )
		
		self.m_staticText53 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Gain", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText53.Wrap( -1 )
		fgSizer15.Add( self.m_staticText53, 0, wx.ALL, 5 )
		
		fgSizer38 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer38.AddGrowableCol( 0 )
		fgSizer38.SetFlexibleDirection( wx.BOTH )
		fgSizer38.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.dsServoGain = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoGain.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer38.Add( self.dsServoGain, 0, wx.ALL|wx.EXPAND, 5 )
		
		self.m_button16 = wx.Button( self.m_swservo, wx.ID_ANY, u"Auto Gain", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer38.Add( self.m_button16, 0, wx.ALL, 5 )
		
		
		fgSizer15.Add( fgSizer38, 1, wx.EXPAND, 5 )
		
		self.m_staticText32411 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"factor", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText32411.Wrap( -1 )
		fgSizer15.Add( self.m_staticText32411, 0, wx.ALL, 5 )
		
		
		fgSizer351.Add( fgSizer15, 1, wx.EXPAND, 5 )
		
		fgSizer361 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer361.SetFlexibleDirection( wx.BOTH )
		fgSizer361.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText52 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Voltage", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText52.Wrap( -1 )
		fgSizer361.Add( self.m_staticText52, 0, wx.ALL, 5 )
		
		self.m_stServoVoltage = wx.StaticText( self.m_swservo, wx.ID_ANY, u"---", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_stServoVoltage.Wrap( -1 )
		fgSizer361.Add( self.m_stServoVoltage, 0, wx.ALL, 5 )
		
		self.m_staticText30411 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Voltage Correction Offset", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText30411.Wrap( -1 )
		fgSizer361.Add( self.m_staticText30411, 0, wx.ALL, 5 )
		
		self.dsServoVoltageOffset = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoVoltageOffset.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer361.Add( self.dsServoVoltageOffset, 0, wx.ALL, 5 )
		
		self.m_staticText303 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Voltage Correction Factor", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText303.Wrap( -1 )
		fgSizer361.Add( self.m_staticText303, 0, wx.ALL, 5 )
		
		self.dsServoVoltageFactor = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoVoltageFactor.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer361.Add( self.dsServoVoltageFactor, 0, wx.ALL, 5 )
		
		
		fgSizer351.Add( fgSizer361, 1, wx.EXPAND, 5 )
		
		fgSizer37 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer37.SetFlexibleDirection( wx.BOTH )
		fgSizer37.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_staticText55 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Current", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText55.Wrap( -1 )
		fgSizer37.Add( self.m_staticText55, 0, wx.ALL, 5 )
		
		self.m_stServoCurrent = wx.StaticText( self.m_swservo, wx.ID_ANY, u"---", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_stServoCurrent.Wrap( -1 )
		fgSizer37.Add( self.m_stServoCurrent, 0, wx.ALL, 5 )
		
		self.m_staticText302 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Current Correction Offset", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText302.Wrap( -1 )
		fgSizer37.Add( self.m_staticText302, 0, wx.ALL, 5 )
		
		self.dsServoCurrentOffset = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoCurrentOffset.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer37.Add( self.dsServoCurrentOffset, 0, wx.ALL, 5 )
		
		self.m_staticText301 = wx.StaticText( self.m_swservo, wx.ID_ANY, u"Current Correction Factor", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText301.Wrap( -1 )
		fgSizer37.Add( self.m_staticText301, 0, wx.ALL, 5 )
		
		self.dsServoCurrentFactor = wx.SpinCtrlDouble(self.m_swservo)
		self.dsServoCurrentFactor.SetMinSize( wx.Size( 100,30 ) )
		
		fgSizer37.Add( self.dsServoCurrentFactor, 0, wx.ALL, 5 )
		
		
		fgSizer351.Add( fgSizer37, 1, wx.EXPAND, 5 )
		
		
		self.m_swservo.SetSizer( fgSizer351 )
		self.m_swservo.Layout()
		fgSizer351.Fit( self.m_swservo )
		fgSizer9.Add( self.m_swservo, 1, wx.EXPAND |wx.ALL, 5 )
		
		m_sdbSizer3 = wx.StdDialogButtonSizer()
		self.m_sdbSizer3OK = wx.Button( self.m_panel2, wx.ID_OK )
		m_sdbSizer3.AddButton( self.m_sdbSizer3OK )
		m_sdbSizer3.Realize();
		
		fgSizer9.Add( m_sdbSizer3, 1, wx.EXPAND, 5 )
		
		
		self.m_panel2.SetSizer( fgSizer9 )
		self.m_panel2.Layout()
		fgSizer9.Fit( self.m_panel2 )
		self.m_notebook.AddPage( self.m_panel2, u"servo", True )
		
		fgSizer7.Add( self.m_notebook, 1, wx.EXPAND |wx.ALL, 5 )
		
		
		self.SetSizer( fgSizer7 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.m_notebook.Bind( wx.EVT_NOTEBOOK_PAGE_CHANGED, self.PageChanged )
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
		self.m_button11.Bind( wx.EVT_BUTTON, self.onRudderCentered )
		self.m_button12.Bind( wx.EVT_BUTTON, self.onRudderStarboardRange )
		self.m_button121.Bind( wx.EVT_BUTTON, self.onRudderPortRange )
		self.sRudderRange.Bind( wx.EVT_SPINCTRL, self.onRudderRange )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def PageChanged( self, event ):
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
	
	def onRudderCentered( self, event ):
		event.Skip()
	
	def onRudderStarboardRange( self, event ):
		event.Skip()
	
	def onRudderPortRange( self, event ):
		event.Skip()
	
	def onRudderRange( self, event ):
		event.Skip()
	


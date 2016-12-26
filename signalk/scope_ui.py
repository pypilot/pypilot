# -*- coding: utf-8 -*- 

###########################################################################
## Python code generated with wxFormBuilder (version Jul 15 2016)
## http://www.wxformbuilder.org/
##
## PLEASE DO "NOT" EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc
import wx.glcanvas

###########################################################################
## Class SignalKScopeBase
###########################################################################

class SignalKScopeBase ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"SignalK Scope", pos = wx.DefaultPosition, size = wx.Size( 1024,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		fgSizer3 = wx.FlexGridSizer( 0, 1, 0, 0 )
		fgSizer3.AddGrowableCol( 0 )
		fgSizer3.AddGrowableRow( 0 )
		fgSizer3.SetFlexibleDirection( wx.BOTH )
		fgSizer3.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.m_splitter1 = wx.SplitterWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.SP_3D )
		self.m_splitter1.Bind( wx.EVT_IDLE, self.m_splitter1OnIdle )
		
		self.m_panel2 = wx.Panel( self.m_splitter1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer51 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer51.AddGrowableCol( 0 )
		fgSizer51.AddGrowableRow( 0 )
		fgSizer51.SetFlexibleDirection( wx.BOTH )
		fgSizer51.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		clValuesChoices = []
		self.clValues = wx.CheckListBox( self.m_panel2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, clValuesChoices, 0 )
		fgSizer51.Add( self.clValues, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		self.m_panel2.SetSizer( fgSizer51 )
		self.m_panel2.Layout()
		fgSizer51.Fit( self.m_panel2 )
		self.glpanel = wx.Panel( self.m_splitter1, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		fgSizer41 = wx.FlexGridSizer( 0, 2, 0, 0 )
		fgSizer41.AddGrowableCol( 0 )
		fgSizer41.AddGrowableRow( 0 )
		fgSizer41.SetFlexibleDirection( wx.BOTH )
		fgSizer41.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.glArea = wx.glcanvas.GLCanvas(self.glpanel)
		fgSizer41.Add( self.glArea, 0, wx.ALL|wx.EXPAND, 5 )
		
		
		self.glpanel.SetSizer( fgSizer41 )
		self.glpanel.Layout()
		fgSizer41.Fit( self.glpanel )
		self.m_splitter1.SplitVertically( self.m_panel2, self.glpanel, 250 )
		fgSizer3.Add( self.m_splitter1, 1, wx.EXPAND, 5 )
		
		fgSizer5 = wx.FlexGridSizer( 1, 0, 0, 0 )
		fgSizer5.SetFlexibleDirection( wx.BOTH )
		fgSizer5.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )
		
		self.bZero = wx.Button( self, wx.ID_ANY, u"Zero", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bZero, 0, wx.ALL, 5 )
		
		self.bCenter = wx.Button( self, wx.ID_ANY, u"Center", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bCenter, 0, wx.ALL, 5 )
		
		self.bScalePlus = wx.Button( self, wx.ID_ANY, u"Scale +", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bScalePlus, 0, wx.ALL, 5 )
		
		self.bScaleMinus = wx.Button( self, wx.ID_ANY, u"Scale -", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bScaleMinus, 0, wx.ALL, 5 )
		
		self.bOffsetPlus = wx.Button( self, wx.ID_ANY, u"Offset /\\", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bOffsetPlus, 0, wx.ALL, 5 )
		
		self.bOffsetMinus = wx.Button( self, wx.ID_ANY, u"Offset \\/", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bOffsetMinus, 0, wx.ALL, 5 )
		
		self.tbFreeze = wx.ToggleButton( self, wx.ID_ANY, u"Freeze", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.tbFreeze, 0, wx.ALL, 5 )
		
		self.bReset = wx.Button( self, wx.ID_ANY, u"Reset", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bReset, 0, wx.ALL, 5 )
		
		self.cbfftw = wx.CheckBox( self, wx.ID_ANY, u"fftw", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.cbfftw, 0, wx.ALL, 5 )
		
		self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"Time", wx.DefaultPosition, wx.DefaultSize, 0 )
		self.m_staticText1.Wrap( -1 )
		fgSizer5.Add( self.m_staticText1, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )
		
		self.sTime = wx.SpinCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.Size( 60,-1 ), wx.SP_ARROW_KEYS, 1, 3600, 0 )
		fgSizer5.Add( self.sTime, 0, wx.ALL, 5 )
		
		self.bClose = wx.Button( self, wx.ID_ANY, u"Close", wx.DefaultPosition, wx.DefaultSize, 0 )
		fgSizer5.Add( self.bClose, 0, wx.ALL, 5 )
		
		
		fgSizer3.Add( fgSizer5, 1, wx.EXPAND, 5 )
		
		
		self.SetSizer( fgSizer3 )
		self.Layout()
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.clValues.Bind( wx.EVT_LISTBOX, self.onValueSelected )
		self.clValues.Bind( wx.EVT_CHECKLISTBOX, self.onValueToggled )
		self.glArea.Bind( wx.EVT_KEY_DOWN, self.onKeyPress )
		self.glArea.Bind( wx.EVT_LEFT_DOWN, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_LEFT_UP, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_MIDDLE_DOWN, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_MIDDLE_UP, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_RIGHT_DOWN, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_RIGHT_UP, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_MOTION, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_LEFT_DCLICK, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_MIDDLE_DCLICK, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_RIGHT_DCLICK, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_LEAVE_WINDOW, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_ENTER_WINDOW, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_MOUSEWHEEL, self.onMouseEvents )
		self.glArea.Bind( wx.EVT_PAINT, self.onPaintGL )
		self.glArea.Bind( wx.EVT_SIZE, self.onSizeGL )
		self.bZero.Bind( wx.EVT_BUTTON, self.onZero )
		self.bCenter.Bind( wx.EVT_BUTTON, self.onCenter )
		self.bScalePlus.Bind( wx.EVT_BUTTON, self.onScalePlus )
		self.bScaleMinus.Bind( wx.EVT_BUTTON, self.onScaleMinus )
		self.bOffsetPlus.Bind( wx.EVT_BUTTON, self.onOffsetPlus )
		self.bOffsetMinus.Bind( wx.EVT_BUTTON, self.onOffsetMinus )
		self.tbFreeze.Bind( wx.EVT_TOGGLEBUTTON, self.onFreeze )
		self.bReset.Bind( wx.EVT_BUTTON, self.onReset )
		self.sTime.Bind( wx.EVT_SPINCTRL, self.onTime )
		self.bClose.Bind( wx.EVT_BUTTON, self.onClose )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def onValueSelected( self, event ):
		pass
	
	def onValueToggled( self, event ):
		pass
	
	def onKeyPress( self, event ):
		pass
	
	def onMouseEvents( self, event ):
		pass
	
	def onPaintGL( self, event ):
		pass
	
	def onSizeGL( self, event ):
		pass
	
	def onZero( self, event ):
		pass
	
	def onCenter( self, event ):
		pass
	
	def onScalePlus( self, event ):
		pass
	
	def onScaleMinus( self, event ):
		pass
	
	def onOffsetPlus( self, event ):
		pass
	
	def onOffsetMinus( self, event ):
		pass
	
	def onFreeze( self, event ):
		pass
	
	def onReset( self, event ):
		pass
	
	def onTime( self, event ):
		pass
	
	def onClose( self, event ):
		pass
	
	def m_splitter1OnIdle( self, event ):
		self.m_splitter1.SetSashPosition( 250 )
		self.m_splitter1.Unbind( wx.EVT_IDLE )
	


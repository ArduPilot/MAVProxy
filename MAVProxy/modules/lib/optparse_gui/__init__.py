'''
A drop-in replacement for optparse ( "import optparse_gui as optparse" )
Provides an identical interface to optparse(.OptionParser),
But displays an automatically generated wx dialog in order to enter the
options/args, instead of parsing command line arguments
'''

import os
import sys
import re
import optparse
from ..wx_loader import wx
from MAVProxy.modules.lib import multiproc

__version__ = 0.1
__revision__ = '$Id$'

class OptparseDialog( wx.Dialog ):
    '''The dialog presented to the user with dynamically generated controls,
    to fill in the required options.
    Based on the wx.Dialog sample from wx Docs & Demos'''
    def __init__(
            self,
            option_parser,
            parent=None,
            ID=0,
            title='Optparse Dialog',
            pos=wx.DefaultPosition,
            size=wx.DefaultSize,
            style=wx.DEFAULT_DIALOG_STYLE,
            name='OptparseDialog',
            *args, **kwargs):

        super(OptparseDialog, self).__init__(self, *args, **kwargs)
        provider = wx.SimpleHelpProvider()
        wx.HelpProvider_Set(provider)

        pre = wx.PreDialog()
        pre.SetExtraStyle(wx.DIALOG_EX_CONTEXTHELP)
        pre.Create(parent, ID, title, pos, size, style)

        self.PostCreate(pre)

        sizer = wx.BoxSizer(wx.VERTICAL)

        self.option_controls = {}

        top_label_text = '%s %s' % ( option_parser.get_prog_name(),
                                     option_parser.get_version() )
        label = wx.StaticText(self, -1, top_label_text)
        sizer.Add(label, 0, wx.ALIGN_CENTRE|wx.ALL, 5)

        self.browse_option_map = {}

        # Add controls for all the options
        for option in option_parser.option_list:
            if option.dest is None:
                continue

            if option.help is None:
                option.help = u''

            box = wx.BoxSizer(wx.HORIZONTAL)
            if 'store' == option.action:
                label = wx.StaticText(self, -1, option.dest )
                label.SetHelpText( option.help )
                box.Add( label, 0, wx.ALIGN_CENTRE|wx.ALL, 5 )

                if 'choice' == option.type:
                    if optparse.NO_DEFAULT == option.default:
                        option.default = option.choices[0]
                    ctrl = wx.ComboBox(
                        self, -1, choices = option.choices,
                        value = option.default,
                        style = wx.CB_DROPDOWN | wx.CB_READONLY | wx.CB_SORT
                    )
                else:
                    if 'MULTILINE' in option.help:
                        ctrl = wx.TextCtrl( self, -1, "", size=(300,100), style = wx.TE_MULTILINE|wx.TE_PROCESS_ENTER )
                    else:
                        ctrl = wx.TextCtrl( self, -1, "", size=(300,-1) )

                    if ( option.default != optparse.NO_DEFAULT ) and \
                       ( option.default is not None ):
                        ctrl.Value = unicode( option.default )

                box.Add( ctrl, 1, wx.ALIGN_CENTRE|wx.ALL, 5 )

                if option.type in ['file', 'directory']:
                    browse = wx.Button(self, label='...')
                    browse.SetHelpText( 'Click to open %s browser' % (option.type) )
                    self.browse_option_map[browse.GetId()] = option, ctrl
                    wx.EVT_BUTTON(self, browse.GetId(), self.OnSelectPath)
                    box.Add( browse, 0, wx.ALIGN_CENTRE|wx.ALL, 5 )

            elif option.action in ( 'store_true', 'store_false' ):
                ctrl = wx.CheckBox( self, -1, option.dest, size = ( 300, -1 ) )
                box.Add( ctrl, 0, wx.ALIGN_CENTRE|wx.ALL, 5 )
            else:
                raise NotImplementedError ('Unknown option action: %s' % repr( option.action ) )

            ctrl.SetHelpText( option.help )
            sizer.Add(box, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5)

            self.option_controls[ option ] = ctrl

        # Add a text control for entering args
        box = wx.BoxSizer( wx.HORIZONTAL )
        label = wx.StaticText(self, -1, 'args' )
        label.SetHelpText( 'This is the place to enter the args' )

        self.args_ctrl = wx.TextCtrl( self, -1, '', size = ( -1, 100 ),
                            style=wx.TE_MULTILINE | wx.TE_PROCESS_ENTER )
        self.args_ctrl.SetHelpText(
'''Args can either be separated by a space or a newline
Args the contain spaces must be entered like so: "arg with sapce"
'''
        )
        box.Add( label, 0, wx.ALIGN_CENTRE | wx.ALL, 5 )
        box.Add( self.args_ctrl, 1, wx.ALIGN_CENTRE | wx.ALL, 5 )

        sizer.Add( box , 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.RIGHT|wx.TOP, 5)

        line = wx.StaticLine(self, -1, size=(20,-1), style=wx.LI_HORIZONTAL)
        sizer.Add(line, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.RIGHT|wx.TOP, 5)

        btnsizer = wx.StdDialogButtonSizer()

        if wx.Platform != "__WXMSW__":
            btn = wx.ContextHelpButton(self)
            btnsizer.AddButton(btn)

        btn = wx.Button(self, wx.ID_OK)
        btn.SetHelpText("The OK button completes the dialog")
        btn.SetDefault()
        btnsizer.AddButton(btn)

        btn = wx.Button(self, wx.ID_CANCEL)
        btn.SetHelpText("The Cancel button cancels the dialog. (Cool, huh?)")
        btnsizer.AddButton(btn)
        btnsizer.Realize()

        sizer.Add(btnsizer, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5)

        self.SetSizer(sizer)
        sizer.Fit(self)

    def OnSelectPath(self, event):
        option, ctrl = self.browse_option_map[event.GetId()]
        path = os.path.abspath(ctrl.Value)
        if option.type == 'file':
            dlg = wx.FileDialog(self,
                                message = 'Select file for %s' % (option.dest),
                                defaultDir = os.path.dirname(path),
                                defaultFile = path)
        elif option.type == 'directory':
            if os.path.isfile (path):
                path = os.path.dirname (path)
            dlg = wx.DirDialog(self,
                               message = 'Select directory for %s' % (option.dest),
                               defaultPath = path)
        else:
            raise NotImplementedError('option.type')
        dlg_result = dlg.ShowModal()
        if wx.ID_OK != dlg_result:
            return
        ctrl.Value = dlg.GetPath()
##        import open_py_shell;open_py_shell.open_py_shell( locals() )

    def _getOptions( self ):
        option_values = {}
        for option, ctrl in self.option_controls.iteritems():
            option_values[option] = ctrl.Value

        return option_values

    def _getArgs( self ):
        args_buff = self.args_ctrl.Value
        args = re.findall( r'(?:((?:(?:\w|\d)+)|".*?"))\s*', args_buff )
        return args

    def getOptionsAndArgs( self ):
        '''Returns the tuple ( options, args )
        options -  a dictionary of option names and values
        args - a sequence of args'''

        option_values = self._getOptions()
        args = self._getArgs()
        return option_values, args

class UserCancelledError( Exception ):
    pass

class Option (optparse.Option):
    SUPER = optparse.Option
    TYPES = SUPER.TYPES + ('file', 'directory')

class OptionParser( optparse.OptionParser ):
    SUPER = optparse.OptionParser

    def __init__( self, *args, **kwargs ):
        if 'option_class' not in kwargs:
            kwargs['option_class'] = Option
        self.SUPER.__init__( self, *args, **kwargs )

    def parse_args( self, args = None, values = None ):
        '''
        multiprocessing wrapper around _parse_args
        '''
        q = multiproc.Queue()
        p = multiproc.Process(target=self._parse_args, args=(q, args, values))
        p.start()
        ret = q.get()
        p.join()
        return ret

    def _parse_args( self, q, args, values):
        '''
        This is the heart of it all - overrides optparse.OptionParser.parse_args
        @param arg is irrelevant and thus ignored,
               it is here only for interface compatibility
        '''
        if wx.GetApp() is None:
            self.app = wx.App( False )

        # preprocess command line arguments and set to defaults
        option_values, args = self.SUPER.parse_args(self, args, values)
        for option in self.option_list:
            if option.dest and hasattr(option_values, option.dest):
                default = getattr(option_values, option.dest)
                if default is not None:
                    option.default = default

        dlg = OptparseDialog( option_parser = self, title=self.get_description() )

        if args:
            dlg.args_ctrl.Value = ' '.join(args)

        dlg_result = dlg.ShowModal()
        if wx.ID_OK != dlg_result:
            raise UserCancelledError( 'User has canceled' )

        if values is None:
            values = self.get_default_values()

        option_values, args = dlg.getOptionsAndArgs()

        for option, value in option_values.iteritems():
            if ( 'store_true' == option.action ) and ( value is False ):
                setattr( values, option.dest, False )
                continue
            if ( 'store_false' == option.action ) and ( value is True ):
                setattr( values, option.dest, False )
                continue

            if option.takes_value() is False:
                value = None

            option.process( option, value, values, self )

        q.put((values, args))

    def error( self, msg ):
        wx.MessageDialog( None, msg, 'Error!', wx.ICON_ERROR ).ShowModal()
        return self.SUPER.error( self, msg )


################################################################################

def sample_parse_args():
    usage = "usage: %prog [options] args"
    if 1 == len( sys.argv ):
        option_parser_class = OptionParser
    else:
        option_parser_class = optparse.OptionParser

    parser = option_parser_class( usage = usage, version='0.1' )
    parser.add_option("-f", "--file", dest="filename", default = r'c:\1.txt',
                      help="read data from FILENAME")
    parser.add_option("-t", "--text", dest="text", default = r'c:\1.txt',
                      help="MULTILINE text field")
    parser.add_option("-a", "--action", dest="action",
                      choices = ['delete', 'copy', 'move'],
                      help="Which action do you wish to take?!")
    parser.add_option("-n", "--number", dest="number", default = 23,
                      type = 'int',
                      help="Just a number")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose",
                      help = 'To be or not to be? ( verbose )' )

    (options, args) = parser.parse_args()
    return options, args

def sample_parse_args_issue1():
    usage = "usage: %prog [options] args"
    option_parser_class = OptionParser

    parser = option_parser_class( usage = usage, version='0.1', description='Demo' )
    parser.add_option("-f", "--file", dest="filename", default = r'c:\1.txt',
                      type = 'file',
                      help="read data from FILENAME")
    parser.add_option("-t", "--text", dest="text", default = r'c:\1.txt',
                      help="MULTILINE text field")
    parser.add_option("-a", "--action", dest="action",
                      choices = ['delete', 'copy', 'move'],
                      help="Which action do you wish to take?!")
    parser.add_option("-n", "--number", dest="number", default = 23,
                      type = 'int',
                      help="Just a number")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose",
                      help = 'To be or not to be? ( verbose )' )

    (options, args) = parser.parse_args()
    return options, args

def main():
    options, args = sample_parse_args_issue1()
    print('args: %s' % repr( args ))
    print('options: %s' % repr( options ))

if '__main__' == __name__:
    main()

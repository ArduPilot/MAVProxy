'''
http://stackoverflow.com/questions/7524307/pyqt-signals-slots-with-qtdesigner
hint: extending Qt designer with extra widgets looks neat too: http://doc.qt.nokia.com/qq/qq26-pyqtdesigner.html
hint signals and slots with arguments: http://www.saltycrane.com/blog/2008/01/pyqt-how-to-pass-arguments-while/
'''

from PyQt4.QtCore import *
from PyQt4.QtGui import *

class BuzzGraphicsView(QGraphicsView):
    #moved = pyqtSignal(QMouseEvent)

    def __init__(self, parent = None):
        super(BuzzGraphicsView, self).__init__(parent)

    #mousePressEvent? 
    def mouseMoveEvent(self, event):
        # call the base method to be sure the events are forwarded to the scene
        super(BuzzGraphicsView, self).mouseMoveEvent(event)

        #print "Mouse Pointer is currently hovering at: ", event.pos()
        #self.moved.emit(event)
        
        self.emit(SIGNAL("moved"), event.pos())

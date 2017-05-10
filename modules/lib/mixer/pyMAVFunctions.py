import xml.etree.ElementTree as etree

class MAVFunctionParam(object):
    def __init__(self, name, id):
        

class MAVFunction(object):
    def __init__(self, name, id):
        self.name = name
        self.id = int(id)
        self.description = ""
        self.fields = []
        self.fieldnames = []
        self.fmtstr = ">"

class Functions(object):
        
	def loadFile(self):
	    tree = etree.parse('FunctionBlocks.xml')
	    root = tree.getroot()

	    
	

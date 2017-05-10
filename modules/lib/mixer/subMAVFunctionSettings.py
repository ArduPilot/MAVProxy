#!/usr/bin/env python

#
# Generated Wed Mar 28 20:22:14 2012 by generateDS.py version 2.7b.
#

import sys

import MAVFunctionSettings as supermod

etree_ = None
Verbose_import_ = False
(   XMLParser_import_none, XMLParser_import_lxml,
    XMLParser_import_elementtree
    ) = range(3)
XMLParser_import_library = None
try:
    # lxml
    from lxml import etree as etree_
    XMLParser_import_library = XMLParser_import_lxml
    if Verbose_import_:
        print("running with lxml.etree")
except ImportError:
    try:
        # cElementTree from Python 2.5+
        import xml.etree.cElementTree as etree_
        XMLParser_import_library = XMLParser_import_elementtree
        if Verbose_import_:
            print("running with cElementTree on Python 2.5+")
    except ImportError:
        try:
            # ElementTree from Python 2.5+
            import xml.etree.ElementTree as etree_
            XMLParser_import_library = XMLParser_import_elementtree
            if Verbose_import_:
                print("running with ElementTree on Python 2.5+")
        except ImportError:
            try:
                # normal cElementTree install
                import cElementTree as etree_
                XMLParser_import_library = XMLParser_import_elementtree
                if Verbose_import_:
                    print("running with cElementTree")
            except ImportError:
                try:
                    # normal ElementTree install
                    import elementtree.ElementTree as etree_
                    XMLParser_import_library = XMLParser_import_elementtree
                    if Verbose_import_:
                        print("running with ElementTree")
                except ImportError:
                    raise ImportError("Failed to import ElementTree from any known place")

def parsexml_(*args, **kwargs):
    if (XMLParser_import_library == XMLParser_import_lxml and
        'parser' not in kwargs):
        # Use the lxml ElementTree compatible parser so that, e.g.,
        #   we ignore comments.
        kwargs['parser'] = etree_.ETCompatXMLParser()
    doc = etree_.parse(*args, **kwargs)
    return doc

#
# Globals
#

ExternalEncoding = 'ascii'

#
# Data representation classes
#

class ComponentFunctionSettingsSub(supermod.ComponentFunctionSettings):
    def __init__(self, registers=None, functions=None, inputRegs=None, outputRegs=None):
        super(ComponentFunctionSettingsSub, self).__init__(registers, functions, inputRegs, outputRegs, )
supermod.ComponentFunctionSettings.subclass = ComponentFunctionSettingsSub
# end class ComponentFunctionSettingsSub


class registersSub(supermod.registers):
    def __init__(self, register=None):
        super(registersSub, self).__init__(register, )
supermod.registers.subclass = registersSub
# end class registersSub


class registerSub(supermod.register):
    def __init__(self, identifier=None, description=None):
        super(registerSub, self).__init__(identifier, description, )
supermod.register.subclass = registerSub
# end class registerSub


class functionsSub(supermod.functions):
    def __init__(self, function=None):
        super(functionsSub, self).__init__(function, )
supermod.functions.subclass = functionsSub
# end class functionsSub


class functionBlockDataSub(supermod.functionBlockData):
    def __init__(self, name=None, value=None):
        super(functionBlockDataSub, self).__init__(name, value, )
supermod.functionBlockData.subclass = functionBlockDataSub
# end class functionBlockDataSub


class functionBlockHeaderSub(supermod.functionBlockHeader):
    def __init__(self, functionType=None, destReg=None, action=None, description=None):
        super(functionBlockHeaderSub, self).__init__(functionType, destReg, action, description, )
supermod.functionBlockHeader.subclass = functionBlockHeaderSub
# end class functionBlockHeaderSub


class functionSub(supermod.function):
    def __init__(self, header=None, setting=None):
        super(functionSub, self).__init__(header, setting, )
supermod.function.subclass = functionSub
# end class functionSub


class inputSub(supermod.input):
    def __init__(self, register=None):
        super(inputSub, self).__init__(register, )
supermod.input.subclass = inputSub
# end class inputSub


class outputSub(supermod.output):
    def __init__(self, register=None):
        super(outputSub, self).__init__(register, )
supermod.output.subclass = outputSub
# end class outputSub


class outputsSub(supermod.outputs):
    def __init__(self, output=None):
        super(outputsSub, self).__init__(output, )
supermod.outputs.subclass = outputsSub
# end class outputsSub


class inputsSub(supermod.inputs):
    def __init__(self, input=None):
        super(inputsSub, self).__init__(input, )
supermod.inputs.subclass = inputsSub
# end class inputsSub



def get_root_tag(node):
    tag = supermod.Tag_pattern_.match(node.tag).groups()[-1]
    rootClass = None
    if hasattr(supermod, tag):
        rootClass = getattr(supermod, tag)
    return tag, rootClass


def parse(inFilename):
    doc = parsexml_(inFilename)
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'ComponentFunctionSettings'
        rootClass = supermod.ComponentFunctionSettings
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('<?xml version="1.0" ?>\n')
##     rootObj.export(sys.stdout, 0, name_=rootTag,
##         namespacedef_='')
    doc = None
    return rootObj


def parseString(inString):
    from StringIO import StringIO
    doc = parsexml_(StringIO(inString))
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'ComponentFunctionSettings'
        rootClass = supermod.ComponentFunctionSettings
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('<?xml version="1.0" ?>\n')
##     rootObj.export(sys.stdout, 0, name_=rootTag,
##         namespacedef_='')
    return rootObj


def parseLiteral(inFilename):
    doc = parsexml_(inFilename)
    rootNode = doc.getroot()
    rootTag, rootClass = get_root_tag(rootNode)
    if rootClass is None:
        rootTag = 'ComponentFunctionSettings'
        rootClass = supermod.ComponentFunctionSettings
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('#from MAVFunctionSettings import *\n\n')
##     sys.stdout.write('import MAVFunctionSettings as model_\n\n')
##     sys.stdout.write('rootObj = model_.ComponentFunctionSettings(\n')
##     rootObj.exportLiteral(sys.stdout, 0, name_="ComponentFunctionSettings")
##     sys.stdout.write(')\n')
    return rootObj


USAGE_TEXT = """
Usage: python ???.py <infilename>
"""

def usage():
    print USAGE_TEXT
    sys.exit(1)


def main():
    args = sys.argv[1:]
    if len(args) != 1:
        usage()
    infilename = args[0]
    root = parse(infilename)


if __name__ == '__main__':
    #import pdb; pdb.set_trace()
    main()



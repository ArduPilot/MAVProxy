#!/usr/bin/env python

#
# Generated Fri Sep  9 07:07:10 2011 by generateDS.py version 2.6a.
#

import sys

import FunctionBlocks as supermod

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

class FunctionBlocksSub(supermod.FunctionBlocks):
    def __init__(self, functionBlock=None):
        super(FunctionBlocksSub, self).__init__(functionBlock, )
supermod.FunctionBlocks.subclass = FunctionBlocksSub
# end class FunctionBlocksSub


class functionBlockDataSub(supermod.functionBlockData):
    def __init__(self, name=None, type_=None, default=None, description='no description'):
        super(functionBlockDataSub, self).__init__(name, type_, default, description, )
supermod.functionBlockData.subclass = functionBlockDataSub
# end class functionBlockDataSub


class functionBlockHeaderSub(supermod.functionBlockHeader):
    def __init__(self, name=None, description='no description', MAVFunction='functionNULL'):
        super(functionBlockHeaderSub, self).__init__(name, description, MAVFunction, )
supermod.functionBlockHeader.subclass = functionBlockHeaderSub
# end class functionBlockHeaderSub


class functionBlockSub(supermod.functionBlock):
    def __init__(self, header=None, setting=None):
        super(functionBlockSub, self).__init__(header, setting, )
supermod.functionBlock.subclass = functionBlockSub
# end class functionBlockSub



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
        rootTag = 'FunctionBlocks'
        rootClass = supermod.FunctionBlocks
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
        rootTag = 'FunctionBlocks'
        rootClass = supermod.FunctionBlocks
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
        rootTag = 'FunctionBlocks'
        rootClass = supermod.FunctionBlocks
    rootObj = rootClass.factory()
    rootObj.build(rootNode)
    # Enable Python to collect the space used by the DOM.
    doc = None
##     sys.stdout.write('#from FunctionBlocks import *\n\n')
##     sys.stdout.write('import FunctionBlocks as model_\n\n')
##     sys.stdout.write('rootObj = model_.FunctionBlocks(\n')
##     rootObj.exportLiteral(sys.stdout, 0, name_="FunctionBlocks")
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



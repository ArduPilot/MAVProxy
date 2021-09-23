'''
GraphDefinition class
'''

class GraphDefinition(object):
    '''a pre-defined graph'''
    def __init__(self, name, expression, description, expressions, filename, label_or=None):
        self.name = name
        self.expression = expression
        self.description = description
        self.expressions = expressions
        self.filename = filename
        self.label_override = label_or

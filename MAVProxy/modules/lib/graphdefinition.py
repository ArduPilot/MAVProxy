'''
GraphDefinition class
'''

class GraphDefinition(object):
    '''a pre-defined graph'''
    def __init__(self, name, expression, description, expressions, filename):
        self.name = name
        self.expression = expression
        self.description = description
        self.expressions = expressions
        self.filename = filename

from behave import *

@given('we have behave installed')
def step(context):
    # we got this far didn't we 
    pass

@when('we implement a test')
def step(context):
    # I test, therefor I am
    assert True

@then('behave will test it for us!')
def step(context):
    assert context.failed is False

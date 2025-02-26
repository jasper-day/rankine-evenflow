import typer
from pyparsing import *
import xml.etree.ElementTree as ET
from tabulate import tabulate

OPS = {
    '+': 'sum',
    '-': 'difference',
    '*': 'product',
    '/': 'quotient',
    'pow': 'pow',
    'exp': 'exp',
    'abs': 'abs',
    'sin': 'sin',
    'cos': 'cos',
    'tan': 'tan',
    'asin': 'asin',
    'acos': 'acos',
    'atan': 'atan',
    'atan2': 'atan2',
    'min': 'min',
    'max': 'max',
    'avg': 'avg',
    'fraction': 'fraction',
    'mod': 'mod',
    'random': 'random',
    'integer': 'integer'
}

property_set = set()
property_defined = set()

LPAR = Literal("(").suppress()
RPAR = Literal(")").suppress()
LBRA = Literal("[").suppress()
RBRA = Literal("]").suppress()


comment = Combine(Literal(";") + rest_of_line)
def comment_xml(toks: ParseResults) -> ET.Element:
    return ET.Comment(toks[0][1:].strip())  # Remove leading semicolon
comment.set_parse_action(comment_xml)

property = Word(alphanums + "/[]-_")
property_notag = Word(alphanums + "/[]-_")
def property_xml(toks: ParseResults) -> ET.Element:
    property_set.add(toks[0])
    property_element = ET.Element("property")
    property_element.text = f" {toks[0]} "
    return property_element
property.set_parse_action(property_xml)

value = common.sci_real | common.integer
value_notag = common.sci_real | common.integer
def value_xml(toks: ParseResults) -> ET.Element:
    value_element = ET.Element("value")
    value_element.text = f" {toks[0]} "
    return value_element
value.set_parse_action(value_xml)

table_entry = value_notag | '""'
table_row = Forward()
def define_col_number(toks):
    nbcol = len(toks[0])
    table_row << Group(table_entry * nbcol + Optional(Literal(",").suppress()))
first_row = Group(OneOrMore(table_entry) + Literal(",").suppress()).set_parse_action(define_col_number)

table_data = (LBRA
                + first_row
                + ZeroOrMore(table_row)
                + RBRA)

def table_data_xml(str, loc, toks):
    table_data_element = ET.Element("tableData")
    array = toks.as_list()
    if array[0][0] == '""':
        array[0][0] = None
    tabular = tabulate(array, tablefmt="plain", disable_numparse=True )
    table_data_element.text = f" \n{tabular}\n "
    return table_data_element
table_data.set_parse_action(table_data_xml)

table_index_id = one_of("row column")
table_index = (LPAR
                + table_index_id("index")
                + property_notag("property")
                + RPAR)
def table_index_xml(toks):
    index_element = ET.Element("independentVar")
    index_element.set("lookup", toks.index)
    property_set.add(toks.property)
    index_element.text = f" {toks.property} "
    return index_element
table_index.set_parse_action(table_index_xml)

table = (LPAR
            + "table"
            + Optional(property_notag("name"))
            + table_index[1,2]("index")
            + table_data("data")
            + RPAR)
def table_xml(str, loc, toks):
    table_element = ET.Element("table")
    if getattr(toks, "name"):
        if toks.name in property_defined:
            raise ParseFatalException(str, loc, toks.name + " already defined")
        property_defined.add(toks.name)
        property_set.add(toks.name)
        table_element.set("name", toks.name)
    for index_element in toks.index:
        table_element.append(index_element)
    table_element.append(toks.data)
    return table_element
table.set_parse_action(table_xml)


string = comment | table | value | property

def handle_operation(toks: ParseResults) -> ET.Element:
    op_name = toks[0]
    e = ET.Element(op_name)
    # Add all children to the operation element
    for child in toks[1:]:
        if isinstance(child, ET.Element):
            e.append(child)
    return e

op = one_of(OPS.keys()).set_parse_action(lambda toks: OPS[toks[0]])

sexp = Forward()
def sexpList_xml(toks: ParseResults) -> ET.Element:
    op_element = handle_operation(toks)
    return op_element
sexpList = (LPAR + op + sexp[...] + RPAR).set_parse_action(sexpList_xml)
sexp <<= string | sexpList

docstring = dbl_quoted_string.set_parse_action(removeQuotes)

def function_xml(str, loc, toks: ParseResults) -> ET.Element:
    fn_element = ET.Element("function")
    if toks.name in property_defined:
        raise ParseFatalException(str, loc, toks.name + " already defined")
    property_defined.add(toks.name)
    property_set.add(toks.name)
    fn_element.set('name', toks.name)
    
    if getattr(toks, "docstring"):
        # Add docstring as comment
        description_element = ET.Element("description")
        description_element.text = toks.docstring
        fn_element.append(description_element)
        
    for child in toks.body:
        if isinstance(child, ET.Element):
            fn_element.append(child)
    
    return fn_element

function = (LPAR 
           + "def" 
           + Optional(docstring("docstring")) 
           + property_notag("name") 
           + sexp[1,...]("body") 
           + RPAR).set_parse_action(function_xml)

axis_title = one_of("X Y Z AXIAL NORMAL SIDE LIFT DRAG ROLL PITCH YAW")

def axis_xml(toks: ParseResults) -> ET.Element:
    axis_element = ET.Element("axis")
    axis_element.set('name', toks.name)
    axis_element.set('frame', "BODY")
    
    for child in toks.body:
        if isinstance(child, ET.Element):
            axis_element.append(child)
            
    return axis_element

axis = (LPAR 
        + "axis" 
        + axis_title("name") 
        + (function | comment)[...]("body") 
        + RPAR).set_parse_action(axis_xml)

spec = (function | axis | comment)[...]

app = typer.Typer()

@app.command()
def compile(file: str, parse_all: bool=True):
    # Parse the file
    parsed = spec.parse_file(file, parse_all=parse_all)
    
    # Create the root element
    root = ET.Element("aerodynamics")
    
    # Add all parsed elements to the root
    for element in parsed:
        if isinstance(element, ET.Element):
            root.append(element)
    
    # Pretty print the XML
    ET.indent(root, space="    ")
    print(ET.tostring(root, encoding='unicode'))
    
@app.command()
def properties(file: str, output: str | None = None):
    try:
        _ = spec.parse_file(file, parse_all=False)
    except ParseFatalException as e:
        with open(output, 'w') as f:
            f.write(e.msg)
        return
    p_defined = "\n".join(sorted(property_defined))
    p_undefined = "\n".join(sorted(property_set - property_defined))
    p = "DEFINED:\n" + p_defined + "\nUNDEFINED:\n" + p_undefined
    if output:
        with open(output, 'w') as f:
            f.write(p)
    else:
        print(p)
    


if __name__ == "__main__":
    app()
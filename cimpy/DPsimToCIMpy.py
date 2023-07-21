import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy
import cimpy
import numpy
from cimpy import utils

def node_of_comp(self, comp_name):
    """
    node_of_comp returns the name of the node of the component comp_name
    
    :param comp_name:
    """ 
    
    comp_ptr = None
    for comp in self.components:
        if comp.name()== comp_name:
            comp_ptr = comp

    obj_dict = self.components_at_node # Attribute not found
    node_name = None
    for node, comp in obj_dict.items():
        if (comp_ptr in comp):
            node_name = node.name()

    return node_name


def DPsimToCIMpy ( DPsim_system ):
    # network is a Dictionary for CIM Objects and the output of this function
    network = {'meta_info': {
                'namespaces': {
                'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                'cim': 'http://iec.ch/TC57/2013/CIM-schema-cim16#',
                'md': 'http://iec.ch/TC57/61970-552/ModelDescription/1#',
                'entsoe': 'http://entsoe.eu/CIM/SchemaExtension/3/1#'},            
                'urls': {}, 
                'author': 'DPsim'},
                'topology': {}
                }
    
    







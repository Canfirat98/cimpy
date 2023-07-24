import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy
import cimpy
from math import sqrt, pi
from cimpy import utils



frequency = 60

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
    
    B1 = utils.create_BaseVoltage(24000)

    for node in DPsim_system.nodes:
        network = utils.add_TopologicalNode(network, "cgmes_v2_4_15", B1,0,0, node.name())
        

    for comp in DPsim_system.components:
        if "PiLine" in str(type(comp)):
            # create ACLineSegment
            # PiLine Parameters
            name = comp.name()
            r = float(str(comp.attr("R_series")))
            x= float(str(comp.attr("L_series"))) * (2*pi*frequency)
            bch= float(str(comp.attr("C_parallel"))) * (2*pi*frequency)
            gch = float(str(comp.attr("G_parallel")))
            
            network = utils.add_ACLineSegment(network, "cgmes_v2_4_15", "n1_pf", "n2_pf", r, x, bch, gch, B1, name)

        elif "NetworkInjection" in str(type(comp)):
            network = utils.add_external_network_injection(network, "cgmes_v2_4_15", "n2_pf", 1)
        
        elif "SynchronGenerator" in str(type(comp)):
            # Generator Parameters
            p = float(str(comp.attr("P_set")))
            q = float(str(comp.attr("Q_set")))
            ratedS = sqrt(p*p+q*q)
            ratedU = float(str(comp.attr("base_Voltage")))
            targetValue = float(str(comp.attr("V_set")))
            initialP = float(str(comp.attr("P_set_pu")))
            name = comp.name()

            network = utils.add_SynchronousMachine(network, "cgmes_v2_4_15", "n1_pf", p, q, ratedS, ratedU, targetValue, initialP, name)
         

    return network









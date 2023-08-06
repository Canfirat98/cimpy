import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy
import cimpy
import numpy as np
import cmath
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

    obj_dict = self.components_at_node
    node_name = []
    for node, comp in obj_dict.items():
        if (comp_ptr in comp):
            node_name = node.name()

    return node_name

def nodes_of_comp(self, comp_name):
    """
    nodes_of_comp returns a list of the names of the nodes of the component comp_name, it is used for Components with more 
    than one Node.
    
    :param comp_name:
    """ 
    
    comp_ptr = None
    for comp in self.components:
        if comp.name()== comp_name:
            comp_ptr = comp

    obj_dict = self.components_at_node
    node_list = []
    for node, comp in obj_dict.items():
        if (comp_ptr in comp):
            node_list.append(node.name())

    return node_list


def DPsimToCIMpy ( DPsim_system , DPsim_simulation):
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
        v = np.linalg.norm( DPsim_simulation.get_idobj_attr(node.name(), 'v').get()[0] )                        # Betrag SvVoltage
        angle = cmath.phase( DPsim_simulation.get_idobj_attr(node.name(), 'v').get()[0] )        #Phase von SvVoltage
        network = utils.add_TopologicalNode(network, "cgmes_v2_4_15", B1, v, angle, node.name())
        

    for comp in DPsim_system.components:
        if "PiLine" in str(type(comp)):
            # create ACLineSegment
            # PiLine Parameters
            name = comp.name()
            r = float(str(comp.attr("R_series")))
            x= float(str(comp.attr("L_series"))) * (2*pi*frequency)
            bch= float(str(comp.attr("C_parallel"))) * (2*pi*frequency)
            gch = float(str(comp.attr("G_parallel")))
            
            # determine the connected Nodes
            Node_List = nodes_of_comp(DPsim_system, name)
            network = utils.add_ACLineSegment(network, "cgmes_v2_4_15", Node_List[0], Node_List[1], r, x, bch, gch, B1, name)

        elif "NetworkInjection" in str(type(comp)):
            # determine the connected Node
            Node_name = node_of_comp(DPsim_system, comp.name())
            network = utils.add_external_network_injection(network, "cgmes_v2_4_15", Node_name, 1)
        
        elif isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator):
            # Generator Parameters
            p = float(str(comp.attr("P_set")))
            q = float(str(comp.attr("Q_set")))
            ratedS = float(str(sqrt(p*p+q*q)))
            ratedU = float(str(comp.attr("base_Voltage")))
            targetValue = float(str(comp.attr("V_set")))
            initialP = float(str(comp.attr("P_set_pu")))
            name = comp.name()

            #determine the connected Node
            Node_name = node_of_comp(DPsim_system, name)
            network = utils.add_SynchronousMachine(network, "cgmes_v2_4_15", Node_name, p, q, ratedS, ratedU, targetValue, initialP, name)

        elif "Load" in str(type(comp)):
            # Load Parameters
            name = comp.name()

            #determine the connected Node
            Node_name = node_of_comp(DPsim_system, name)
            network = utils.add_EnergyConsumer(network, "cgmes_v2_4_15", Node_name, p, q, B1, name)

        elif "Transformer" in str(type(comp)):
            # Transfomer Parameters
            name = comp.name()
            r = float(str(comp.attr("R")))                               # Widerstand
            x = float(str(comp.attr("L")))                               # Induktivität
            mNominalVoltageEnd1 = float(str(comp.attr("nominal_voltage_end1")))      # Spannung Hochspannungsseite   
            mNominalVoltageEnd2 = float(str(comp.attr("nominal_voltage_end2")))      # Spannung Niederspannungsseite   

            #determine the connected Node
            Node_List = nodes_of_comp(DPsim_system, name)
            network = utils.add_PowerTransfomer(network, "cgmes_v2_4_15", Node_List[0], Node_List[1], r, x, mNominalVoltageEnd1, mNominalVoltageEnd2, name)

        elif "SynchronGenerator" and "Order" in str(type(comp)):
            # Synchronous Machine Parameters
            p = float(str(comp.attr("initElecPower").derive_real()))
            q = float(str(comp.attr("initElecPower").derive_imag()))
            ratedS = float(str(comp.attr("initElecPower").derive_mag()))
            ratedU = float(str(getattr(comp, "Vnom", 0)))
            targetValue = float(str( np.linalg.norm(comp.InitVoltage) / comp.Vnom )) # ??? InitVoltage ist komplex, soll ich den Betrag nehmen ???
            initialP = float(str( comp.attr("initElecPower").derive_real() ))
            Sync_generator_name = comp.name()

            #determine the connected Node
            Node_name = node_of_comp(DPsim_system, Sync_generator_name)
            
            # Add Synchronous Machine to the network
            network = utils.add_SynchronousMachine(network, "cgmes_v2_4_15", Node_name, p, q, ratedS, ratedU, targetValue, initialP, Sync_generator_name)


            # Synchronous Machine TimeConstantReactance Parameters
            inertia = float(str(comp.attr("H")))
            statorResistance = 0   # because it is not modelled in DPsim
            statorLeakageReactance = float(str(comp.attr("Td0_t")))
            tpdo = float(str(comp.attr("Td0_t")))
            tpqo = float(str(comp.attr("Tq0_t")))
            tppdo = float(str(comp.attr("Td0_s")))
            tppqo = float(str(comp.attr("Tq0_s")))
            xDirectSubtrans = float(str(comp.attr("Ld_s")))
            xDirectSync = float(str(comp.attr("Ld")))
            xDirectTrans = float(str(comp.attr("Ld_t")))
            xQuadSubtrans = float(str(comp.attr("Lq_s")))
            xQuadSync = float(str(comp.attr("Lq")))
            xQuadTrans = float(str(comp.attr("Lq_t")))
            # Extend SynchronousMachine with dynamic Parameters
            network= utils.extend_SynchronousMachineTimeConstantReactance(network, "cgmes_v2_4_15", network["topology"][Sync_generator_name], 0, inertia,
                        statorResistance, statorLeakageReactance, tpdo, tpqo, tppdo, tppqo, xDirectSubtrans, xDirectSync, xDirectTrans, xQuadSubtrans, xQuadSync, xQuadTrans)
                
    return network









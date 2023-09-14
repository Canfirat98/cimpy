from enum import Enum
import cmath
import numpy as np
import logging
from cimpy import utils
import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy

# configure logging
logging.basicConfig(filename='CIM2Dpsim.log', encoding='utf-8', level=logging.DEBUG)

class Multiplier(Enum):
    p = 1
    n = 2
    micro = 3
    m = 4
    c = 5
    d = 6
    k = 7
    M = 8
    G = 9
    T = 10
    
def unitValue(value, multiplier=Multiplier.k):
    if (multiplier==Multiplier.p):
        return value * 1e-12
    elif (multiplier==Multiplier.n):
        return value * 1e-9
    elif (multiplier==Multiplier.micro):
        return value * 1e-6
    elif (multiplier==Multiplier.m):
        return value * 1e-3
    elif (multiplier==Multiplier.c):
        return value * 1e-2
    elif (multiplier==Multiplier.d):
        return value * 1e-1
    elif (multiplier==Multiplier.k):
        return value * 1e3
    elif (multiplier==Multiplier.M):
        return value * 1e6
    elif (multiplier==Multiplier.G):
        return value * 1e9
    elif (multiplier==Multiplier.T):
        return value * 1e12
    
def node_of_comp(system, comp_name):
    """
    node_of_comp returns  a list containing the nodes connected with this component
    
    :param comp_name:
    """ 
    
    comp_ptr = None
    for comp in system.components:
        if comp.name== comp_name:
            comp_ptr = comp

    node_list = []
    for node, comp in system.components_at_node.items():
        if (comp_ptr in comp):
            node_list.append(node)
            
    return node_list

def DPsimToCIMpy (DPsim_system_PF, DPsim_simulation, DPsim_system_dyn=None, frequency=60):
    # network is a Dictionary for CIM Objects and the output of this function
    cim_topology = {'meta_info': {
                'namespaces': {
                    'rdf': 'http://www.w3.org/1999/02/22-rdf-syntax-ns#',
                    'cim': 'http://iec.ch/TC57/2013/CIM-schema-cim16#',
                    'md': 'http://iec.ch/TC57/61970-552/ModelDescription/1#',
                    'entsoe': 'http://entsoe.eu/CIM/SchemaExtension/3/1#'},            
                'urls': {}, 
                'author': 'DPsim'},
                'topology': {}
                }
    
    for node in DPsim_system_PF.nodes:
        v = np.linalg.norm(DPsim_simulation.get_idobj_attr(node.uid, 'v').get()[0])
        v = unitValue(v, Multiplier.m) 
        angle = cmath.phase(DPsim_simulation.get_idobj_attr(node.uid, 'v').get()[0])
        baseVoltage = unitValue(get_node_base_voltage(DPsim_system_PF, node), Multiplier.m) 
        cim_topology = utils.add_TopologicalNode(cim_topology=cim_topology, version="cgmes_v2_4_15", 
                                                 name=node.name, mRID=node.uid, baseVoltage=baseVoltage, v=v, angle=angle)
        logging.debug('Added TopologicalNode: \n\tname={}\n\tbaseVoltage={}\n\tv={}\n\tangle={}"'.format(node.name, baseVoltage, v, angle))

    for comp in DPsim_system_PF.components:
        if isinstance(comp, dpsimpy.sp.ph1.PiLine):
            omega = 2*np.pi*frequency
            r = comp.attr("R_series")
            x = comp.attr("L_series") * omega
            bch = comp.attr("C_parallel") * omega
            gch = comp.attr("G_parallel")
            base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m) 
            
            # determine the connected Nodes
            node_list =  node_of_comp(DPsim_system_PF, comp.name)
            cim_topology = utils.add_ACLineSegment(cim_topology=cim_topology, version="cgmes_v2_4_15", name=comp.name, 
                                                   name_node1=node_list[0].mRID, name_node1=node_list[1].mRID, r=r, x=x, bch=bch, gch=gch, baseVoltage=baseVoltage)        
            logging.debug('Added PiLine: \n\tname={}, \n\tL={}, \n\tR={}, \n\tC={}, \n\tG={}, \n\tbaseVoltage={}, \n\tnode1 name={}, \n\tnode 2 name={}'.format(
                    comp.name, r, x, bch, gch, base_voltage, node_list[0].name, node_list[1].name))

        elif "Load" in str(type(comp)):
            p = unitValue(comp.attr("P"), Multiplier.micro) 
            q = unitValue(comp.attr("Q"), Multiplier.micro) 
            Vnom = unitValue(comp.attr("V_nom"), Multiplier.m) 
            
            #determine the connected Node
            node = node_of_comp(DPsim_system_PF, comp.name)
            network = utils.add_EnergyConsumer(cim_topologie=network, version="cgmes_v2_4_15", name=comp.name, 
                             node_name=node.name, p_nom=p, q_nom=q, p_init=p, q_init=q, baseVoltage=Vnom)
            logging.debug('Added Load: \n\tname={}, \n\tp={}, \n\tq={}, \n\tbaseVoltage={}, \n\tnode name={}'.format(
                            comp.name, p, q, Vnom, node.name))

        elif "Transformer" in str(type(comp)):
            r = comp.attr("R")
            x = comp.attr("L")
            mNominalVoltageEnd1 = unitValue(comp.attr("nominal_voltage_end1"), Multiplier.m) 
            mNominalVoltageEnd2 = unitValue(comp.attr("nominal_voltage_end2"), Multiplier.m) 
            base_voltage =  unitValue(comp.attr("base_Voltage"), Multiplier.m) 
            
            #determine the connected Node
            node_list = node_of_comp(DPsim_system_PF, comp.name)
            base_voltage_n1 = get_node_base_voltage(DPsim_system_PF, node_list[0])
            base_voltage_n2 = get_node_base_voltage(DPsim_system_PF, node_list[1])
            if (base_voltage_n1 >= unitValue(base_voltage_n2), Multiplier.m) :
                node1_name = node_list[0].name
                node2_name = node_list[1].name
                mNominalVoltageEnd1 = unitValue(base_voltage_n1, Multiplier.m)
                mNominalVoltageEnd2 = unitValue(base_voltage_n2, Multiplier.m)
            else:
                node2_name = node_list[0].name
                node1_name = node_list[1].name
                mNominalVoltageEnd2 = unitValue(base_voltage_n1, Multiplier.m)
                mNominalVoltageEnd1 = unitValue(base_voltage_n2, Multiplier.m)
                
            network = utils.add_PowerTransfomer(cim_topologie=network, version="cgmes_v2_4_15", name=comp.name,
                                                node1_name=node1_name, node2_name=node2_name, 
                                                r=r, x=x, nominal_voltage_end1=mNominalVoltageEnd1, nominal_voltage_end2=mNominalVoltageEnd2)
            logging.debug('Added Transformer: \n\tname={}, \n\tr={}, \n\tx={}, \n\tbaseVoltage={}, \n\tnode1 name={}, \n\tnode2 name={}'.format(
                            comp.name, r, x, base_voltage, node1_name, node2_name))
    
        elif "SynchronGenerator":
            p = unitValue(comp.attr("initElecPower").derive_real(), Multiplier.micro)
            q = unitValue(comp.attr("initElecPower").derive_imag(), Multiplier.micro)
            ratedS = unitValue(comp.attr("initElecPower").derive_mag(), Multiplier.micro)   # TODO: CHECK! MAYBE USE mAppparentPower?
            ratedU = unitValue(getattr(comp, "Vnom", 0), Multiplier.m)
            targetValue = unitValue(comp.attr("V_set"), Multiplier.m)
            initialP = unitValue(comp.attr("initElecPower").derive_real(), Multiplier.micro)

            #determine the connected Node
            node = node_of_comp(DPsim_system_PF, comp.name)
            
            # Add Synchronous Machine to the network
            network = utils.add_SynchronousMachine(cim_topologie=network, version="cgmes_v2_4_15", name=comp.name, node_name=node.name, 
                                                   ratedS=ratedS, ratedU=ratedU, p=p, q=q, targetValue=targetValue, initialP=initialP)
            logging.debug('Added SynchronGenerator: \n\tname={}, \n\tp={}, \n\tq={}, \n\tratedU={}, \n\tratedS={}, \n\ttargetValue={}, \n\tinitial={}, \n\tnode name={}'.format(
                    comp.name, p, q, ratedU, ratedS, targetValue, initialP, node.name))
            
            if (DPsim_system_dyn is not None):
                dyn_comp = DPsim_system_PF.component(comp.name)
                # Synchronous Machine TimeConstantReactance Parameters
                inertia = float(str(dyn_comp.attr("H")))
                statorResistance = 0   # TODO FIX IN DPSIM
                statorLeakageReactance = 0      # TODO FIX IN DPSIM
                tpdo = float(str(dyn_comp.attr("Td0_t")))
                tpqo = float(str(dyn_comp.attr("Tq0_t")))
                tppdo = float(str(dyn_comp.attr("Td0_s")))
                tppqo = float(str(dyn_comp.attr("Tq0_s")))
                xDirectSubtrans = float(str(dyn_comp.attr("Ld_s")))
                xDirectSync = float(str(dyn_comp.attr("Ld")))
                xDirectTrans = float(str(dyn_comp.attr("Ld_t")))
                xQuadSubtrans = float(str(dyn_comp.attr("Lq_s")))
                xQuadSync = float(str(dyn_comp.attr("Lq")))
                xQuadTrans = float(str(dyn_comp.attr("Lq_t")))
                
                #
                sg_mRID = ""
                for cim_comp in network:
                    if cim_comp.name == comp.name:
                        sg_mRID=cim_comp.mRID
                    
                # Extend SynchronousMachine with dynamic Parameters
                network = utils.extend_SynchronousMachineTimeConstantReactance(cim_topologie=network, version="cgmes_v2_4_15", SynchronousMachine_mRID=sg_mRID, 
                                                                                damping=0, inertia=inertia, statorResistance=statorResistance, statorLeakageReactance=statorLeakageReactance, 
                                                                                tpdo=tpdo, tpqo=tpqo, tppdo=tppdo, tppqo=tppqo, xDirectSubtrans=xDirectSubtrans, xDirectSync=xDirectSync, 
                                                                                xDirectTrans=xDirectTrans, xQuadSubtrans=xQuadSubtrans, xQuadSync=xQuadSync, xQuadTrans=xQuadTrans)
        """            
        elif "NetworkInjection" in str(type(comp)):
            # determine the connected Node
            Node_name = node_of_comp(DPsim_system, comp.uid)
            network = utils.add_external_network_injection(network, "cgmes_v2_4_15", Node_name, 1)
        """
        
    return network


def get_node_base_voltage(DPsim_system_PF, node):
    base_voltage = 0

    for comp in DPsim_system_PF.components_at_node[node]:
        if isinstance(comp, dpsimpy.sp.ph1.AvVoltageSourceInverterDQ):
            base_voltage = unitValue(np.mag(comp.attr("vnom")), Multiplier.m) 
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break
        elif isinstance(comp, dpsimpy.sp.ph1.PiLine):
            base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break
        elif isinstance(comp, dpsimpy.sp.ph1.Transformer):
            if comp.get_terminal(0).node().name == node.name:
                base_voltage = unitValue(comp.attr("nominal_voltage_end1") Multiplier.m)
                logging.info('Choose base voltage {}kV of {} for node "{}", TransformerEnd: 1', base_voltage, comp.name, node.name)
                break
            elif comp.get_terminal(1).node().name == node.name:
                base_voltage = unitValue(comp.attr("nominal_voltage_end2"), Multiplier.m)
                logging.info('Choose base voltage {}kV of {} for node "{}", TransformerEnd: 2', base_voltage, comp.name, node.name)
                break
        elif isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator):
            base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break
        elif isinstance(comp, dpsimpy.sp.ph1.SynchronGenerator):
            base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break  
        elif isinstance(comp, dpsimpy.sp.ph1.Load):
            base_voltage = unitValue(comp.attr("base_Voltage"), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break
        elif isinstance(comp, dpsimpy.sp.ph1.NetworkInjection):
            base_voltage = unitValue(comp.attr("V_nom"), Multiplier.m)   
            logging.info('Choose base voltage {}kV of {} for node "{}"', base_voltage, comp.name, node.name)
            break
        else:
             logging.info('Unable to get base voltage at {}"', node.name)
                     
    return base_voltage
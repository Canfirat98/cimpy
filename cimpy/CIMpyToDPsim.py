import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy
from enum import Enum
import numpy

#Nodes
Nodes = dict()
Components = dict()


# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4

frequency = 60

# Domains hinzufügen
def CIMpyToDPsim(CIM_network, domain):

    res = CIM_network["topology"]

    # get the correct module to be used (dpsimpy.sp, dpsimpy.dp or dpsimpy.emt) 
    dpsimpy_components = None
    if (domain == 1):
        dpsimpy_components = dpsimpy.sp.ph1
    elif (domain == 2):
        dpsimpy_components = dpsimpy.sp.ph1
    elif (domain == 3):
        dpsimpy_components = dpsimpy.dp.ph1
    elif (domain == 4):
        dpsimpy_components = dpsimpy.emt.ph3
    else:
        raise Exception('ERROR: domain {} is not supported in dpsimpy'.format(domain))

    
    for i in res:
        ### Components 
        if 'ACLineSegment' in str(type(res[i])):
            # PiLine
            pi_line = dpsimpy_components.PiLine(res[i].mRID, dpsimpy.LogLevel.debug)
            pi_line.set_parameters(R= res[i].r,
                                L=res[i].x/(2*numpy.pi*frequency),
                                C=res[i].bch/(2*numpy.pi*frequency),
                                G=res[i].gch)
            Components[pi_line.name()] = pi_line

        elif 'ExternalNetworkInjection' in str(type(res[i])):
            # Slack
            slack = dpsimpy_components.NetworkInjection(res[i].mRID, dpsimpy.LogLevel.debug)
            #slack.set_parameters(V_ref=nominal_voltage_mv)
            Components[slack.name()] = slack

        elif 'SynchronousMachineTimeConstantReactance' in str(type(res[i])):
            # Synchron Generator
            gen = dpsimpy_components.SynchronGenerator3OrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
            gen.set_operational_parameters_per_unit(H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync,
                                                    Ld_t=res[i].xDirectTrans, Td0_t=res[i].tpdo)
            Components[gen.name()] = gen

    for j in res:
        ### Nodes
        if 'TopologicalNode' in str(type(res[j])): 
            n1 = dpsimpy.sp.SimNode(res[j].mRID, dpsimpy.PhaseType.Single)
            Nodes[n1.name()] = n1
            Terminals = res[j].Terminal
            for terminal in Terminals:
                component_mRID = terminal.ConductingEquipment.mRID
                # Hier auf ACLineSegment achten, mehrere Nodes möglich !
                Components[component_mRID].connect([n1])

        
    
    node_list = list(Nodes.values())
    component_list =list(Components.values())
    system = dpsimpy.SystemTopology(frequency, node_list, component_list)

    return system

import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy
import cimpy
from enum import Enum
import numpy

# Nodes
Nodes = dict()
# Components
Components_Dict = dict()
# Synchronous Machines
SynchronousMachineTCR_Dict = dict()


# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4

frequency = 60

# Domains hinzufügen
def CIMpyToDPsim(CIM_network, domain, gen_model="3Order"):

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
            pi_line.set_parameters(R= res[i].r,                             #line resistance                         
                                L=res[i].x/(2*numpy.pi*frequency),          #line inductance
                                C=res[i].bch/(2*numpy.pi*frequency),        #line capacitance
                                G=res[i].gch)                               #line conductance
            if (domain == 1):
                # Set BaseVoltage of ACLineSegment to PiLine
                baseVoltage = res[i].BaseVoltage.nominalVoltage
                pi_line.set_base_voltage(baseVoltage)

            Components_Dict[pi_line.name()] = {"Element": pi_line, "Nodes": []}

        elif 'ExternalNetworkInjection' in str(type(res[i])):
            # Slack
            slack = dpsimpy_components.NetworkInjection(res[i].mRID, dpsimpy.LogLevel.debug)
            
            if (domain == 1):
                baseVoltage = 0
                for obj in res.values():
                    if isinstance(obj , cimpy.cgmes_v2_4_15.BaseVoltage):
                        if obj.ConductingEquipment != 'list':
                            for comp in obj.ConductingEquipment:
                                if comp.mRID == slack.name():
                                    baseVoltage = obj.BaseVoltage.nominalVoltage
                                    break

                if (baseVoltage == 0):
                    # Take baseVoltage of topologicalNode where equipment is connected to
                    for obj in res.values():
                        if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                                for term in obj.Terminal:
                                    if term.ConductingEquipment.mRID == slack.name():
                                        baseVoltage = obj.BaseVoltage.nominalVoltage
                                        break
                slack.set_base_voltage(baseVoltage)

                if res[i].RegulatingControl != None:
                    voltageSetPoint = res[i].RegulatingControl.targetValue
                else:
                    voltageSetPoint = baseVoltage
                slack.set_parameters(voltage_set_point = voltageSetPoint)
            
            
            Components_Dict[slack.name()] = {"Element": slack, "Nodes": []}

        elif isinstance(res[i], cimpy.cgmes_v2_4_15.SynchronousMachine) and (domain == 1):
            gen_pf = dpsimpy.sp.ph1.SynchronGenerator(res[i].mRID, dpsimpy.LogLevel.debug)
            try:
                res[i].GeneratingUnit.RotatingMachine.ratedS
            except:
                gen_baseS= res[i].ratedS
            else:
                gen_baseS= res[i].GeneratingUnit.RotatingMachine.ratedS
            
            try:
                res[i].GeneratingUnit.RotatingMachine.ratedU
            except:
                gen_baseV= res[i].ratedU
            else:
                gen_baseV= res[i].GeneratingUnit.RotatingMachine.ratedU          
            
            try:
                res[i].GeneratingUnit.initialP
            except AttributeError:
                try:
                    res[i].GeneratingUnit.RotatingMachine.p
                except AttributeError:
                    print("Fehler")
                else:
                    gen_p = res[i].GeneratingUnit.RotatingMachine.p
            else:
                gen_p= res[i].GeneratingUnit.initialP  

            # 
            try:
                res[i].GeneratingUnit.RotatingMachine.RegulatingControl.targetValue
            except AttributeError:
                try:
                    res[i].RegulatingControl.targetValue
                except:
                    gen_v = 1
                else:
                    gen_v= res[i].RegulatingControl.targetValue
            else:
                gen_v= res[i].GeneratingUnit.RotatingMachine.RegulatingControl.targetValue            
            
            # Blindleistung Q
            try:
                res[i].q
            except AttributeError:
                print("Fehler")
            else:
                gen_q= res[i].q  
             

            gen_pf.set_parameters(rated_apparent_power= gen_baseS, rated_voltage=gen_baseV, 
                      set_point_active_power=gen_p, set_point_voltage=gen_v, 
                      set_point_reactive_power=gen_q, 
                      powerflow_bus_type=dpsimpy.PowerflowBusType.PV)
            #gen_pf.set_base_voltage(nominal_voltage_mv)
            gen_pf.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PV)

            Components_Dict[gen_pf.name()] = {"Element": gen_pf, "Nodes": []}


        elif 'SynchronousMachineTimeConstantReactance' in str(type(res[i])):
            # Synchron Generator
            if (gen_model=="3Order"):
                gen = dpsimpy_components.SynchronGenerator3OrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.set_operational_parameters_per_unit(H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync,
                                                    Ld_t=res[i].xDirectTrans, Td0_t=res[i].tpdo)
            elif (gen_model=="4Order"):
                gen = dpsimpy_components.SynchronGenerator4OrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.set_operational_parameters_per_unit(H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo)		
            elif (gen_model=="6aOrder"):
                gen = dpsimpy_components.SynchronGenerator6aOrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.set_operational_parameters_per_unit(H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo,
                                                        Ld_s=res[i].xDirectSubtrans, Lq_s=res[i].xQuadSubtrans, Td0_s=res[i].tppdo, Tq0_s=res[i].tppqo)	
            elif (gen_model=="6bOrder"):
                gen = dpsimpy_components.SynchronGenerator6bOrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.set_operational_parameters_per_unit(H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo,
                                                        Ld_s=res[i].xDirectSubtrans, Lq_s=res[i].xQuadSubtrans, Td0_s=res[i].tppdo, Tq0_s=res[i].tppqo)	
          
            Components_Dict[gen.name()] = {"Element": gen, "Nodes": [], "Sync_Machine": res[i].SynchronousMachine}
            SynchronousMachineTCR_Dict[gen.name()] = {res[i].SynchronousMachine}            # Saves the connented SynchronousMachine

        elif 'EnergyConsumer' in str(type(res[i])):
            # Energy Consumer
            load = dpsimpy_components.Load(res[i].mRID, dpsimpy.LogLevel.debug)
            Components_Dict[load.name()] = {"Element": load, "Nodes": []}

        elif isinstance(res[i], cimpy.cgmes_v2_4_15.PowerTransformer):
            transformer = dpsimpy_components.Transformer(res[i].mRID, dpsimpy.LogLevel.debug)
            if (domain == 1):
                # Take baseVoltage of HighVoltage Side (PowerTransformerEnd[0])
                baseVoltage = res[i].PowerTransformerEnd[0].BaseVoltage.nominalVoltage
                transformer.set_base_voltage(baseVoltage)

            Components_Dict[transformer.name()] = {"Element": transformer, "Nodes": []}

    

    for j in res:
        ### Nodes
        if 'TopologicalNode' in str(type(res[j])): 
            n1 = dpsimpy.sp.SimNode(res[j].mRID, dpsimpy.PhaseType.Single)
            Nodes[n1.name()] = n1
            Terminals = res[j].Terminal
            for terminal in Terminals:                         # search for connected Components via Terminals
                component_mRID = terminal.ConductingEquipment.mRID
                if isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.SynchronousMachine) and (len(SynchronousMachineTCR_Dict) != 0):      # Match the Nodes from SyncMachine to SynchMachineTCR
                    for syn_machine_tcr_mRID in SynchronousMachineTCR_Dict:
                        if terminal.ConductingEquipment == SynchronousMachineTCR_Dict[syn_machine_tcr_mRID]:
                            component_mRID = syn_machine_tcr_mRID
                            break
                if component_mRID in Components_Dict:
                    Components_Dict[component_mRID]["Nodes"].append(n1)
                   
             


    component_list = []                                             # for SystemTopology: Components have to be in list structure
    for comp_ID in Components_Dict:
        Components_Dict[comp_ID]["Element"].connect(Components_Dict[comp_ID]["Nodes"])                  # Connect the Components with there Nodes
        component_list.append(Components_Dict[comp_ID]["Element"])
   

    node_list = list(Nodes.values())
    system = dpsimpy.SystemTopology(frequency, node_list, component_list)

    return system

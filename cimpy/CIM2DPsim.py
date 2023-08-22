import sys
sys.path.insert(0,'/home/mmo/git/Can/dpsim/build')
import dpsimpy
import cimpy
from enum import Enum
import numpy
import cmath
import logging

# configure logging
logging.basicConfig(filename='CIM2Dpsim.log', encoding='utf-8', level=logging.DEBUG)

# define dpsimpy domains
class Domain(Enum):
    PF = 1
    SP = 2
    DP = 3
    EMT = 4

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


# Domains hinzufügen
def CIM2DPsim(CIM_network, domain, frequency = 60, log_level=dpsimpy.LogLevel.info, gen_model="3Order"):

    # Nodes
    nodes = dict()
    
    # Components
    components_dict = dict()
    
    # Synchronous Machines
    SynchronousMachineTCR_Dict = dict()

    res = CIM_network["topology"]

    # get the correct module to be used (dpsimpy.sp, dpsimpy.dp or dpsimpy.emt) 
    dpsimpy_components = None
    if (domain == Domain.PF):
        dpsimpy_components = dpsimpy.sp.ph1
    elif (domain == Domain.SP):
        dpsimpy_components = dpsimpy.sp.ph1
    elif (domain == Domain.DP):
        dpsimpy_components = dpsimpy.dp.ph1
    elif (domain == Domain.EMT):
        dpsimpy_components = dpsimpy.emt.ph3
    else:
        raise Exception('ERROR: domain {} is not supported in dpsimpy'.format(domain))

    
    for i in res:
        ### Components 
        if 'ACLineSegment' in str(type(res[i])):
            # PiLine
            pi_line = dpsimpy_components.PiLine(res[i].mRID, log_level)
            pi_line.name = res[i].name
            pi_line.set_parameters(R=res[i].r,                             #line resistance                         
                                   L=res[i].x/(2*numpy.pi*frequency),          #line inductance
                                   C=res[i].bch/(2*numpy.pi*frequency),        #line capacitance
                                   G=res[i].gch)                               #line conductance
            if (domain == Domain.PF):
                # Set BaseVoltage of ACLineSegment to PiLine
                baseVoltage = unitValue(res[i].BaseVoltage.nominalVoltage, Multiplier.k)
                pi_line.set_base_voltage(baseVoltage)

            components_dict[pi_line.uid] = {"Element": pi_line, "Nodes": []}
            logging.debug('Created ACLineSegment: name={}, L={}, R={}, C={}, G={}'.format(
                pi_line.name, res[i].r, res[i].x/(2*numpy.pi*frequency), res[i].bch/(2*numpy.pi*frequency), res[i].gch))
            

        elif 'ExternalNetworkInjection' in str(type(res[i])):
            # Slack
            slack = dpsimpy_components.NetworkInjection(res[i].mRID, log_level)
            slack.name = res[i].name
            if (domain == Domain.PF):
                baseVoltage = 0
                for obj in res.values():
                    if isinstance(obj , cimpy.cgmes_v2_4_15.BaseVoltage):
                        if obj.ConductingEquipment != 'list':
                            for comp in obj.ConductingEquipment:
                                if comp.mRID == slack.uid:
                                    baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                    break

                if (baseVoltage == 0):
                    # Take baseVoltage of topologicalNode where equipment is connected to
                    for obj in res.values():
                        if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                                for term in obj.Terminal:
                                    if term.ConductingEquipment.mRID == slack.uid:
                                        baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                        break
                if (baseVoltage == 0):
                    baseVoltage = 1
                slack.set_base_voltage(baseVoltage)

                if res[i].RegulatingControl != None:
                    voltageSetPoint = res[i].RegulatingControl.targetValue * baseVoltage
                else:
                    voltageSetPoint = baseVoltage
                
                slack.set_parameters(voltage_set_point = voltageSetPoint)
                logging.debug('Created ExternalNetworkInjection: name={}, baseVoltage={}, voltageSetPoint={}'.format(slack.name, baseVoltage, voltageSetPoint))
            else:
                baseVoltage = 1
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                        for term in obj.Terminal:
                            if term.ConductingEquipment.mRID == slack.uid:
                                baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                                break
                try:
                    res[i].RegulatingControl.targetValue
                except:
                    voltageRef = 0
                else:
                    voltageRef = res[i].RegulatingControl.targetValue * baseVoltage

                slack.set_parameters(V_ref = voltageRef)
                logging.debug('Created ExternalNetworkInjection: name={}, baseVoltage={}, voltageRef={}'.format(slack.name, baseVoltage, voltageRef))
                       
            components_dict[slack.uid] = {"Element": slack, "Nodes": []}

        elif isinstance(res[i], cimpy.cgmes_v2_4_15.SynchronousMachine) and domain == Domain.PF:
            gen_pf = dpsimpy.sp.ph1.SynchronGenerator(res[i].mRID, log_level)
            gen_pf.name = res[i].name
            try:
                gen_baseS=res[i].ratedS
            except:
                gen_baseS= res[i].GeneratingUnit.RotatingMachine.ratedS
                

            
            try:
                res[i].ratedU
            except:
                gen_baseV= res[i].GeneratingUnit.RotatingMachine.ratedU
            else:
                gen_baseV= res[i].ratedU
        
            
            try:
                res[i].p
            except AttributeError:
                try:
                    res[i].GeneratingUnit.initialP
                except AttributeError:
                    raise Exception('initialP of SG {} was not found'.format(res[i].mRID))
                else:
                    gen_p = res[i].GeneratingUnit.initialP
            else:
                gen_p= res[i].p

            # 
            try:
                res[i].RegulatingControl.targetValue
            except AttributeError:
                try:
                    res[i].GeneratingUnit.RotatingMachine.RegulatingControl.targetValue
                except:
                    gen_v = 1
                else:
                    gen_v= res[i].GeneratingUnit.RotatingMachine.RegulatingControl.targetValue 
            else:
                gen_v= res[i].RegulatingControl.targetValue         
            
            # Blindleistung Q
            try:
                res[i].q
            except AttributeError:
                raise Exception('q of SG {} was not found'.format(res[i].mRID))
            else:
                gen_q= res[i].q  
            # Type cast to float
            gen_baseS = float(str(gen_baseS))
            gen_baseV = float(str(gen_baseV))
            gen_p = float(str(gen_p))
            gen_v = float(str(gen_v))
            gen_q = float(str(gen_q))
            # Set Parameters
            gen_pf.set_parameters(rated_apparent_power=gen_baseS, rated_voltage=gen_baseV, 
                      set_point_active_power=gen_p, set_point_voltage=gen_v, 
                      set_point_reactive_power=gen_q, 
                      powerflow_bus_type=dpsimpy.PowerflowBusType.PV)
            # Set BaseVoltage
            baseVoltage = 1
            for obj in res.values():
                if isinstance(obj, cimpy.cgmes_v2_4_15.TopologicalNode):
                    for term in obj.Terminal:
                        if term.ConductingEquipment.mRID == gen_pf.uid:
                            baseVoltage = unitValue(obj.BaseVoltage.nominalVoltage, Multiplier.k)
                            break
            gen_pf.set_base_voltage(baseVoltage)
            gen_pf.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PV)

            components_dict[gen_pf.uid] = {"Element": gen_pf, "Nodes": []}
            logging.debug('Created SynchronGenerator: name={}, rated_apparent_power={}, rated_voltage={}, set_point_active_power={}, set_point_voltage={}, set_point_reactive_power={}'.format(
                gen_pf.name, gen_baseS, gen_baseV, gen_p, gen_v, gen_q))
        
        elif 'SynchronousMachineTimeConstantReactance' in str(type(res[i])):
            # Synchron Generator
            nom_power = float(str(res[i].SynchronousMachine.ratedS))
            nom_voltage = float(str(res[i].SynchronousMachine.ratedU))
            L0 = 0.15
            H=float(str(res[i].inertia))
            Ld=float(str(res[i].xDirectSync))
            Lq=float(str(res[i].xQuadSync))
            Ld_t=float(str(res[i].xDirectTrans))
            Td0_t=float(str(res[i].tpdo))
            if (gen_model=="3Order"):      
                gen = dpsimpy_components.SynchronGenerator3OrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.name = res[i].name
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=H,
                                                        Ld=Ld, Lq=Lq, L0=L0, Ld_t=Ld_t, Td0_t=Td0_t)
            elif (gen_model=="4Order"):
                gen = dpsimpy_components.SynchronGenerator4OrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.name = res[i].name
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=res[i].inertia,
                                                        Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, L0=L0, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo)		
            elif (gen_model=="6aOrder"):
                gen = dpsimpy_components.SynchronGenerator6aOrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.name = res[i].name
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, L0=L0, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo,
                                                        Ld_s=res[i].xDirectSubtrans, Lq_s=res[i].xQuadSubtrans, Td0_s=res[i].tppdo, Tq0_s=res[i].tppqo)	
            elif (gen_model=="6bOrder"):
                gen = dpsimpy_components.SynchronGenerator6bOrderVBR(res[i].mRID, dpsimpy.LogLevel.debug)
                gen.name = res[i].name
                gen.set_operational_parameters_per_unit(nom_power=nom_power, nom_voltage=nom_voltage, nom_frequency=frequency, H=res[i].inertia, Ld=res[i].xDirectSync, Lq=res[i].xQuadSync, L0=L0, Ld_t=res[i].xDirectTrans, Lq_t=res[i].xQuadTrans, Td0_t=res[i].tpdo, Tq0_t=res[i].tpqo,
                                                        Ld_s=res[i].xDirectSubtrans, Lq_s=res[i].xQuadSubtrans, Td0_s=res[i].tppdo, Tq0_s=res[i].tppqo)	
            
            init_electrical_power = complex( float(str(res[i].SynchronousMachine.p )), float(str(res[i].SynchronousMachine.q )))
            init_mechanical_power = res[i].SynchronousMachine.p
            for obj in res.values():
                if isinstance(obj, cimpy.cgmes_v2_4_15.SvVoltage):
                    for term in obj.TopologicalNode.Terminal:
                        if term.ConductingEquipment.mRID == res[i].SynchronousMachine.mRID:
                            betrag = getattr(obj, "v", 0)
                            phase = getattr(obj, "angle", 0)
                            init_complex_terminal_voltage = betrag * cmath.exp(1j * phase)
                            break

            gen.set_initial_values(init_complex_electrical_power=init_electrical_power, init_mechanical_power=init_mechanical_power, 
                           init_complex_terminal_voltage=init_complex_terminal_voltage)
            
            components_dict[gen.uid] = {"Element": gen, "Nodes": [], "Sync_Machine": res[i].SynchronousMachine}
            SynchronousMachineTCR_Dict[gen.uid] = res[i].SynchronousMachine            # Saves the connented SynchronousMachine
            logging.debug('Created SynchronGenerator: name={}, init_complex_electrical_power={}, init_mechanical_power={}, init_complex_terminal_voltage={}'.format(
                gen.name, init_electrical_power, init_mechanical_power, init_complex_terminal_voltage))
            
        elif isinstance(res[i], cimpy.cgmes_v2_4_15.EnergyConsumer) or isinstance(res[i], cimpy.cgmes_v2_4_15.ConformLoad):
            # Energy Consumer
            if (domain == Domain.PF):
                load = dpsimpy_components.Load(res[i].mRID, log_level)
                load.name = res[i].name
                #load.modify_power_flow_bus_type(dpsimpy.PowerflowBusType.PV)
                p = getattr(res[i], "p", 0)
                q = getattr(res[i], "q", 0)
                if p == 0 and q == 0:
                    for obj in res.values():
                        if isinstance(obj, cimpy.cgmes_v2_4_15.SvPowerFlow):
                            if obj.Terminal.ConductingEquipment.mRID == load.uid:
                                p = getattr(obj, "p", 0)
                                q = getattr(obj, "q", 0)
                                break
                    if p == 0 and q == 0:
                        raise Exception("ERROR: Fehler mit p und q")
                    
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.SvVoltage):
                        for term in obj.TopologicalNode.Terminal:
                            if term.ConductingEquipment.mRID == load.uid:
                                nom_voltage = getattr(obj, "v", 0)
                                break
                if nom_voltage == 0:
                    nom_voltage = unitValue(res[i].BaseVoltage.nominalVoltage, Multiplier.k)         

                load.set_parameters(p, q, nom_voltage)
                #load.set_parameters(p, q)
            elif (domain == Domain.SP):
                load = dpsimpy_components.Load(res[i].mRID, log_level)
                load.name = res[i].name
            else:
                load = dpsimpy_components.RXLoad(res[i].mRID, log_level)
                load.name = res[i].name


            if (domain != Domain.PF):                   # Only for domains: SP, DP and EMT
                p = getattr(res[i], "p", 0)
                q = getattr(res[i], "q", 0)
                if p == 0 and q == 0:
                    for obj in res.values():
                        if isinstance(obj, cimpy.cgmes_v2_4_15.SvPowerFlow):
                            if obj.Terminal.ConductingEquipment.mRID == load.uid:
                                p = getattr(obj, "p", 0)
                                q = getattr(obj, "q", 0)
                                break
                    if p == 0 and q == 0:
                        raise Exception("ERROR: Fehler mit p und q")
                    
                for obj in res.values():
                    if isinstance(obj, cimpy.cgmes_v2_4_15.SvVoltage):
                        for term in obj.TopologicalNode.Terminal:
                            if term.ConductingEquipment.mRID == load.uid:
                                nom_voltage = getattr(obj, "v", 0)
                                break
                if nom_voltage == 0:
                    baseVoltage = unitValue(res[i].BaseVoltage.nominalVoltage, Multiplier.k)
                load.set_parameters(p, q, nom_voltage)

            components_dict[load.uid] = {"Element": load, "Nodes": []}
            logging.debug('Created EnergyConsumer: name={}, p={}, q={}, nom_voltage={}'.format(
                load.name, p, q, nom_voltage))

        elif isinstance(res[i], cimpy.cgmes_v2_4_15.PowerTransformer):
            transformer = dpsimpy_components.Transformer(res[i].mRID, log_level)
            transformer.name = res[i].name
            if (domain == Domain.PF) or (domain == Domain.SP):
                # Take baseVoltage of HighVoltage Side (PowerTransformerEnd[0])
                baseVoltage = res[i].PowerTransformerEnd[0].ratedU
                transformer.set_base_voltage(baseVoltage)

                nv1 = float(str(res[i].PowerTransformerEnd[0].ratedU))
                nv2 = float(str(res[i].PowerTransformerEnd[1].ratedU))
                r = float(str(res[i].PowerTransformerEnd[0].r))
                i = float(str(res[i].PowerTransformerEnd[0].x))
                transformer.set_parameters(nom_voltage_end_1 = nv1, nom_voltage_end_2 = nv2, ratio_abs = nv1/nv2, ratio_phase = 0, resistance = r, inductance = i)

            components_dict[transformer.uid] = {"Element": transformer, "Nodes": []}
            logging.debug('Created PowerTransformer: name={}, nom_voltage_end_1={}, nom_voltage_end_2={}, ratio_abs={}, ratio_phase={}, resistance={}, inductance={}'.format(
                transformer.name, nv1, nv2, nv1/nv2, 0, r, i))

    

    for j in res:
        ### Nodes
        if 'TopologicalNode' in str(type(res[j])): 
            n1 = dpsimpy.sp.SimNode(res[j].mRID, dpsimpy.PhaseType.Single)
            n1.name = res[j].name
            nodes[n1.uid] = n1
            Terminals = res[j].Terminal
            for terminal in Terminals:                         # search for connected Components via Terminals
                component_mRID = terminal.ConductingEquipment.mRID
                if isinstance(terminal.ConductingEquipment, cimpy.cgmes_v2_4_15.SynchronousMachine) and (len(SynchronousMachineTCR_Dict) != 0):      # Match the Nodes from SyncMachine to SynchMachineTCR
                    for syn_machine_tcr_mRID in SynchronousMachineTCR_Dict:
                        if terminal.ConductingEquipment.mRID == SynchronousMachineTCR_Dict[syn_machine_tcr_mRID].mRID:
                            component_mRID = syn_machine_tcr_mRID
                            break
                if component_mRID in components_dict:
                    components_dict[component_mRID]["Nodes"].append(n1)
                   
             


    component_list = []                                             # for SystemTopology: Components have to be in list structure
    for comp_ID in components_dict:
        components_dict[comp_ID]["Element"].connect(components_dict[comp_ID]["Nodes"])                  # Connect the Components with there Nodes
        component_list.append(components_dict[comp_ID]["Element"])
   

    node_list = list(nodes.values())
    system = dpsimpy.SystemTopology(frequency, node_list, component_list)

    return system, components_dict

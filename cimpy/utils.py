import importlib
import uuid
import cimpy.cgmes_v2_4_15.TopologicalNode

def node_breaker_to_bus_branch(import_result):
    """TODO: Add documentation
    """
    res = import_result['topology']
    breaker_list = []
    terminals_list = []
    operational_limit_set_list = []
    voltage_limit_list = []
    diagram_objects_list = []
    diagram_object_points_list = []
    connect_nodes = []
    for mRID in res.keys():
        class_name = res[mRID].__class__.__name__
        if class_name == "Breaker":
            breaker_list.append(mRID)
        elif class_name == "OperationalLimitSet":
            operational_limit_set_list.append(mRID)
        elif class_name == "Terminal":
            terminals_list.append(mRID)
        elif class_name == "VoltageLimit":
            voltage_limit_list.append(mRID)
        elif class_name == "DiagramObject":
            diagram_objects_list.append(mRID)
        elif class_name == "DiagramObjectPoint":
            diagram_object_points_list.append(mRID)
        elif class_name == 'ConnectivityNode':
            connect_nodes.append(mRID)

    # search for open breakers
    open_breakers = []
    for breaker in breaker_list:
        if res[breaker].open:
            if not res[breaker].retained:
                open_breakers.append(breaker)

    # check terminals for reference to open breakers and delete references to Connectivity Nodes
    del_terminals_list = []
    for terminal in terminals_list:
        cond_eq = res[terminal].ConductingEquipment
        if cond_eq.mRID in open_breakers:
            del_terminals_list.append(terminal)
        else:
            res[terminal].ConnectivityNode = None

    # check for OperationalLimitSet with references to deleted Terminals
    del_operationallimitset = []
    for operational_limit in operational_limit_set_list:
        if res[operational_limit].Terminal.mRID in del_terminals_list:
            del_operationallimitset.append(operational_limit)

    del_voltage_limit = []
    for voltage in voltage_limit_list:
        if res[voltage].OperationalLimitSet.mRID in del_operationallimitset:
            del_voltage_limit.append(voltage)

    del_diagram_object = []

    for diagram_object in diagram_objects_list:
        keep_diagram = []
        if res[diagram_object].IdentifiedObject is None:
            continue
        if 'ConnectivityNode' in str(type(res[diagram_object].IdentifiedObject)):
            if res[diagram_object].IdentifiedObject.TopologicalNode is not None:
                keep_diagram.append(res[diagram_object].IdentifiedObject.TopologicalNode)
        elif res[diagram_object].IdentifiedObject.mRID in (open_breakers + del_terminals_list):
            del_diagram_object.append(diagram_object)

    del_diagram_object_points = []
    for diagram_point in diagram_object_points_list:
        if res[diagram_point].DiagramObject is None:
            continue
        if res[diagram_point].DiagramObject.mRID in del_diagram_object:
            del_diagram_object_points.append(diagram_point)

    del_list = open_breakers + del_diagram_object_points + del_diagram_object + del_voltage_limit + \
               del_operationallimitset + del_terminals_list + connect_nodes

    for key in del_list:
        del res[key]

    import_result['topology'] = res

    return import_result


def add_external_network_injection(import_result, version, mRID, voltage_set_point):
    """TODO: Add documentation
    """
    res = import_result['topology']
    TopologicalNode = ''
    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode.mRID

    if TopologicalNode != '':
        i = 1
        while 'Injection ' + str(i) in res.keys():
            i += 1
        inj_name = 'Injection ' + str(i)
        reg_name = 'Regulating Control ' + str(i)
        terminal_name = 'Terminal Injection ' + str(i)

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)

        regulating_control_module = importlib.import_module(module_name + 'RegulatingControl')
        regulating_control_class = getattr(regulating_control_module, 'RegulatingControl')
        res[reg_name] = regulating_control_class(mRID=reg_name,
                                                 name=reg_name,
                                                 targetValue=voltage_set_point,
                                                 Terminal=res[terminal_name])

        network_injection_module = importlib.import_module(module_name + 'ExternalNetworkInjection')
        network_injection_class = getattr(network_injection_module, 'ExternalNetworkInjection')
        res[inj_name] = network_injection_class(mRID=inj_name,
                                                name=inj_name,
                                                RegulatingControl=res[reg_name],
                                                Terminals=[res[terminal_name]])

        res[reg_name].RegulatingCondEq = res[inj_name]
        res[terminal_name].ConductingEquipment = res[inj_name]
        res[terminal_name].RegulatingControl = res[reg_name]

        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])
    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')

    import_result['topology'] = res

    return import_result


def add_ACLineSegment(import_result, version, mRID_1, mRID_2, r, x, bch, gch, BaseVoltage, name = 'ACLineSegment'):
    res = import_result['topology']
    
    if name in res:
        print(name, "is already included, ... create ACLineSegment with new mRID")
        name = name + str(list(res.keys()).count("ACLineSegment") + 1)
    

    TopologicalNode1 = ''
    TopologicalNode2 = ''
    if mRID_1 in res:
        if 'TopologicalNode' in str(type(res[mRID_1])):
            TopologicalNode1 = res[mRID_1]
        elif 'ConnectivityNode' in str(type(res[mRID_1])):
            TopologicalNode1 = res[mRID_1].TopologicalNode.mRID

    if mRID_2 in res:
        if 'TopologicalNode' in str(type(res[mRID_2])):
            TopologicalNode2 = res[mRID_2]
        elif 'ConnectivityNode' in str(type(res[mRID_2])):
            TopologicalNode2 = res[mRID_2].TopologicalNode.mRID
    

    if TopologicalNode1 != '' and TopologicalNode2 != '':
        terminal_name1 = 'Terminal_1_' + name 
        terminal_name2 = 'Terminal_2_' + name 

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."

        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name1] = terminal_class(mRID=terminal_name1,
                                            name=terminal_name1,
                                            TopologicalNode=TopologicalNode1)
        
        res[terminal_name2] = terminal_class(mRID=terminal_name2,
                                            name=terminal_name2,
                                            TopologicalNode=TopologicalNode2)

        ACLineSegment_module = importlib.import_module((module_name + 'ACLineSegment'))
        ACLineSegment_class = getattr(ACLineSegment_module, 'ACLineSegment')
        res[name] = ACLineSegment_class(mRID= name,
                                        name= name,
                                        r = r,
                                        x = x,
                                        bch = bch,
                                        gch = gch,
                                        BaseVoltage = BaseVoltage,
                                        Terminals = [res[terminal_name1], res[terminal_name2]])
        # Connect Equipment to Terminal
        res[terminal_name1].ConductingEquipment = res[name]
        res[terminal_name2].ConductingEquipment = res[name]

        # Append the Terminal List of the TopologicalNodes
        res[mRID_1].Terminal.append(res[terminal_name1])
        res[mRID_2].Terminal.append(res[terminal_name2])

    if TopologicalNode1 == '':
        print('No Terminal with first mRID ', mRID_1, ' found in object list!')
        return 0

    if TopologicalNode2 == '':
        print('No Terminal with second mRID ', mRID_2, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result


def add_Terminal(import_result, version, TopologicalNode, ConductingEquipment, pInjection = 0, qInjection = 0, mRID = ''):
    res = import_result['topology']
    
    if mRID == "":
        mRID = uuid.uuid1()

    #module_name = "cimpy." + version + ".Equipment."
    module_name = "cimpy." + version + "."

    terminal_module = importlib.import_module((module_name + 'Terminal'))
    terminal_class = getattr(terminal_module, 'Terminal')
    res[mRID] = terminal_class(mRID='Terminal',
                                name='Terminal',
                                TopologicalNode=TopologicalNode,
                                ConductingEquipment= ConductingEquipment,
                                pInjection= pInjection,
                                qInjection= qInjection)


    import_result['topology'] = res

    return import_result
        
def add_TopologicalNode(import_result, version, BaseVoltage, v, angle, mRID = ""):
    res = import_result['topology']
    if mRID in res:
        print("mRID", mRID, "is already included")
        return import_result

    # Creating random mRID using uuid1()
    if mRID == "":
        mRID = uuid.uuid1()

    #module_name = "cimpy." + version + ".Equipment."
    module_name = "cimpy." + version + "."

    TopologicalNode_module = importlib.import_module((module_name + 'TopologicalNode'))
    TopologicalNode_class = getattr(TopologicalNode_module, 'TopologicalNode')
    res[mRID] = TopologicalNode_class(mRID= mRID,
                                name= mRID,
                                BaseVoltage= BaseVoltage,
                                Terminal = [])
    
    SvVoltage_name = mRID + "-sv"
    
    SvVoltage_module = importlib.import_module((module_name + 'SvVoltage'))
    SvVoltage_class = getattr(SvVoltage_module, 'SvVoltage')
    res[SvVoltage_name] = SvVoltage_class(v=v,
                                        angle=angle,
                                        TopologicalNode=res[mRID])


    import_result['topology'] = res

    return import_result

    
def add_SynchronousMachine(import_result, version, mRID, p, q, ratedS, ratedU, targetValue, initialP, name = 'SynchronousMachine'):          
# RotatingMachine without paramameters? 
    res = import_result['topology']
    TopologicalNode = ''
    
    if name in res:
        print(name, "is already included, ... create SynchronousMachine with new mRID")
        name = name + str(list(res.keys()).count("SynchronousMachine") + 1)
    

    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode.mRID

    if TopologicalNode != '':
        terminal_name = 'Terminal_' + name 
        GeneratingUnit_name = 'GeneratingUnit_' + name
        RegulatingControl_name = 'RegulatingControl' + name

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)
        
        RegulatingControl_module = importlib.import_module((module_name + 'RegulatingControl'))
        RegulatingControl_class = getattr(RegulatingControl_module, 'RegulatingControl')
        res[RegulatingControl_name] = RegulatingControl_class(mRID=RegulatingControl_name,
                                            name=RegulatingControl_name,
                                            targetValue=targetValue)
        
        GeneratingUnit_module = importlib.import_module((module_name + 'GeneratingUnit'))
        GeneratingUnit_class = getattr(GeneratingUnit_module, 'GeneratingUnit')
        res[GeneratingUnit_name] = GeneratingUnit_class(mRID=GeneratingUnit_name,
                                            name=GeneratingUnit_name,
                                            initialP  = initialP)

        SynchronousMachine_module = importlib.import_module((module_name + 'SynchronousMachine'))
        SynchronousMachine_class = getattr(SynchronousMachine_module, 'SynchronousMachine')
        res[name] = SynchronousMachine_class(mRID= name,
                                        name= name,
                                        p= p,
                                        q= q,
                                        ratedS= ratedS,
                                        ratedU= ratedU,
                                        RegulatingControl= res[RegulatingControl_name],
                                        GeneratingUnit= res[GeneratingUnit_name],
                                        Terminals= [res[terminal_name]])
        
        res[terminal_name].ConductingEquipment = res[name]

        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])

    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result


def extend_SynchronousMachineTimeConstantReactance(import_result, version, SynchronousMachineID, damping, inertia, statorResistance, statorLeakageReactance, tpdo, tpqo, tppdo, tppqo, xDirectSubtrans, xDirectSync, xDirectTrans, xQuadSubtrans, xQuadSync, xQuadTrans):
    res = import_result['topology']
    name = SynchronousMachineID.mRID + "_TimeConstantReactance"
    
    #module_name = "cimpy." + version + ".Equipment."
    module_name = "cimpy." + version + "."

    
    SynchronousMachineTCR_module = importlib.import_module((module_name + 'SynchronousMachineTimeConstantReactance'))
    SynchronousMachineTCR_class = getattr(SynchronousMachineTCR_module, 'SynchronousMachineTimeConstantReactance')
    res[name] = SynchronousMachineTCR_class(mRID= name,
                                    name= name,
                                    SynchronousMachine = SynchronousMachineID,
                                    damping= damping,
                                    inertia= inertia,
                                    statorResistance= statorResistance,
                                    statorLeakageReactance= statorLeakageReactance,
                                    tpdo= tpdo,
                                    tpqo= tpqo,
                                    tppdo= tppdo,
                                    tppqo= tppqo,
                                    xDirectSubtrans= xDirectSubtrans,
                                    xDirectSync= xDirectSync,
                                    xDirectTrans= xDirectTrans,
                                    xQuadSubtrans= xQuadSubtrans,
                                    xQuadSync= xQuadSync,
                                    xQuadTrans= xQuadTrans)
    import_result['topology'] = res

    return import_result


def add_EnergyConsumer(import_result, version, mRID, p, q, BaseVoltage, name = "EnergyConsumer"):
    res = import_result['topology']
    TopologicalNode = ''
    
    if name in res:
        print(name, "is already included, ... create EnergyConsumer with new mRID")
        name = name + str(list(res.keys()).count("EnergyConsumer") + 1)

    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode.mRID

    if TopologicalNode != '':
        terminal_name = 'Terminal_' + name 
        PowerFlow_name =  name + "-sv"

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)

        EnergyConsumer_module = importlib.import_module((module_name + 'EnergyConsumer'))
        EnergyConsumer_class = getattr(EnergyConsumer_module, 'EnergyConsumer')
        res[name] = EnergyConsumer_class(mRID= name,
                                        name= name,
                                        p= p,
                                        q= q,
                                        BaseVoltage= BaseVoltage,
                                        Terminals = [res[terminal_name]])
         
        SvPowerFlow_module = importlib.import_module((module_name + 'SvPowerFlow'))
        SvPowerFlow_class = getattr(SvPowerFlow_module, 'SvPowerFlow')
        res[PowerFlow_name] = SvPowerFlow_class(p= p,
                                        q= q,
                                        Terminal = res[terminal_name])
        
        res[terminal_name].ConductingEquipment = res[name]

        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])
   
    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result    


def add_PowerTransfomer(import_result, version, mRID, r, x, BaseVoltage, mRatioAbs, name = "PowerTransformer"):
    res = import_result['topology']
    TopologicalNode = ''
    
    if name in res:
        print(name, "is already included, ... create PowerTransformer with new mRID")
        name = name + str(list(res.keys()).count("PowerTransformer") + 1)

    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode.mRID

    if TopologicalNode != '':
        terminal_name = 'Terminal_' + name 
        HVSide_name = name + "-HVSide"
        LVSide_name = name + "-LVSide"
        

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)
        
        PowerTransformer_module = importlib.import_module((module_name + 'PowerTransformer'))
        PowerTransformer_class = getattr(PowerTransformer_module, 'PowerTransformer')
        res[name] = PowerTransformer_class(mRID=name,
                                name=name)
        
        HVSide_module = importlib.import_module((module_name + 'PowerTransformerEnd'))
        HVSide_class = getattr(HVSide_module, 'PowerTransformerEnd')
        res[HVSide_name] = HVSide_class(mRID=HVSide_name,
                                        name=HVSide_name,
                                        ratedU=BaseVoltage,
                                        r=r,
                                        x=x,
                                        endNumber=1,
                                        Terminal=res[terminal_name],
                                        PowerTransformer=res[name] )
        
        LVSide_module = importlib.import_module((module_name + 'PowerTransformerEnd'))
        LVSide_class = getattr(LVSide_module, 'PowerTransformerEnd')
        res[LVSide_name] = LVSide_class(mRID=LVSide_name,
                                        name=LVSide_name,
                                        ratedU=BaseVoltage/mRatioAbs,
                                        endNumber=2,
                                        Terminal=res[terminal_name],
                                        PowerTransformer=res[name] )
        
        res[name].PowerTransformerEnd = [res[HVSide_name], res[LVSide_name]]

        res[terminal_name].ConductingEquipment = res[name]
   
        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])
    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result    

def create_BaseVoltage(nominalVoltage):
        
    #module_name = "cimpy." + version + ".Equipment."
    module_name = "cimpy." + "cgmes_v2_4_15" + "."

    BaseVoltage_module = importlib.import_module((module_name + 'BaseVoltage'))
    BaseVoltage_class = getattr(BaseVoltage_module, 'BaseVoltage')
    mBaseVoltage = BaseVoltage_class(nominalVoltage=nominalVoltage )

    return mBaseVoltage
    
        






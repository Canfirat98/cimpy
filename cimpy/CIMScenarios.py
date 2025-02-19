import importlib
import uuid
import datetime


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

    res[mRID].SvVoltage = res[SvVoltage_name]
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


def add_PowerTransfomer(import_result, version, mRID_1, mRID_2, r, x, mNominalVoltageEnd1, mNominalVoltageEnd2, name = "PowerTransformer"):
    res = import_result['topology']
    
    if name in res:
        print(name, "is already included, ... create PowerTransformer with new mRID")
        name = name + str(list(res.keys()).count("PowerTransformer") + 1)

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
        HVSide_name = name + "-HVSide"
        LVSide_name = name + "-LVSide"
        

        #module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name1] = terminal_class(mRID=terminal_name1,
                                            name=terminal_name1,
                                            TopologicalNode=TopologicalNode1)
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name2] = terminal_class(mRID=terminal_name2,
                                            name=terminal_name2,
                                            TopologicalNode=TopologicalNode2)
        
        PowerTransformer_module = importlib.import_module((module_name + 'PowerTransformer'))
        PowerTransformer_class = getattr(PowerTransformer_module, 'PowerTransformer')
        res[name] = PowerTransformer_class(mRID=name,
                                name=name,
                                Terminals = [res[terminal_name1], res[terminal_name2]])
        
        HVSide_module = importlib.import_module((module_name + 'PowerTransformerEnd'))
        HVSide_class = getattr(HVSide_module, 'PowerTransformerEnd')
        res[HVSide_name] = HVSide_class(mRID=HVSide_name,
                                        name=HVSide_name,
                                        ratedU=mNominalVoltageEnd1,
                                        r=r,
                                        x=x,
                                        endNumber=1,
                                        Terminal=res[terminal_name1],
                                        PowerTransformer=res[name] )
        
        LVSide_module = importlib.import_module((module_name + 'PowerTransformerEnd'))
        LVSide_class = getattr(LVSide_module, 'PowerTransformerEnd')
        res[LVSide_name] = LVSide_class(mRID=LVSide_name,
                                        name=LVSide_name,
                                        ratedU=mNominalVoltageEnd2,
                                        endNumber=2,
                                        Terminal=res[terminal_name2],
                                        PowerTransformer=res[name] )
        
        res[name].PowerTransformerEnd = [res[HVSide_name], res[LVSide_name]]

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

def create_BaseVoltage(nominalVoltage):
        
    #module_name = "cimpy." + version + ".Equipment."
    module_name = "cimpy." + "cgmes_v2_4_15" + "."

    BaseVoltage_module = importlib.import_module((module_name + 'BaseVoltage'))
    BaseVoltage_class = getattr(BaseVoltage_module, 'BaseVoltage')
    mBaseVoltage = BaseVoltage_class(nominalVoltage=nominalVoltage )

    return mBaseVoltage

# Switch Module with infinity Conductance to simulate a short circuit in network "import_result" at the node with name "mRID"
def add_ShortCircuit_Switch(import_result, version, mRID, name = "ShortCircuit_Switch"):
    res = import_result['topology']
    TopologicalNode = ''
    
    if name in res:
        print(name, "is already included, ... create ShortCircuit_Switch with new mRID")
        name = name + str(list(res.keys()).count("ShortCircuit_Switch") + 1)

    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode

    if TopologicalNode != '':
        # Add two Topological Nodes
        BaseVoltage = TopologicalNode.BaseVoltage
        v = TopologicalNode.SvVoltage.v
        angle = TopologicalNode.SvVoltage.angle
        Node_name = "Node_" + name
        import_result = add_TopologicalNode(import_result, "cgmes_v2_4_15", BaseVoltage, v, angle, Node_name)

        terminal_name = 'Terminal_' + name
        SwitchSchedule_name = 'Schedule_' + name 
        GroundingImpedance_name = 'GroundingImpedance_' + name

        # module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        # Create Terminals
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name + "1"] = terminal_class(mRID=terminal_name + "1",
                                            name=terminal_name + "1",
                                            TopologicalNode=res[Node_name])
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name + "2"] = terminal_class(mRID=terminal_name + "2",
                                            name=terminal_name + "2",
                                            TopologicalNode=res[Node_name])
        

        # Create Components
        SwitchSchedule_module = importlib.import_module((module_name + 'SwitchSchedule'))
        SwitchSchedule_class = getattr(SwitchSchedule_module, 'SwitchSchedule')
        res[SwitchSchedule_name] = SwitchSchedule_class(mRID=SwitchSchedule_name,
                                name=SwitchSchedule_name,
                                startTime = datetime.datetime.now(),
                               # timeStep = 0.5,    # The time between each pair of subsequent regular time points in sequence order.
                                endTime = datetime.datetime.now() + datetime.timedelta(seconds=10))       # The time for the last time point
        
        switch_module = importlib.import_module((module_name + 'Switch'))
        switch_class = getattr(switch_module, 'Switch')
        res[name] = switch_class(mRID=name,
                                name=name,
                                SwitchSchedules = [res[SwitchSchedule_name]],
                                Terminals = [res[terminal_name], res[terminal_name + "1"]])
        
        GroundingImpedance_module = importlib.import_module((module_name + 'GroundingImpedance'))
        GroundingImpedance_class = getattr(GroundingImpedance_module, 'GroundingImpedance')
        res[GroundingImpedance_name] = GroundingImpedance_class(mRID=GroundingImpedance_name,
                                name=GroundingImpedance_name,
                                r = 10e-10,
                                Terminals = [res[terminal_name + "2"]])
        
        
        res[terminal_name].ConductingEquipment = res[name]
        res[terminal_name + "1"].ConductingEquipment = res[name]
        res[terminal_name + "2"].ConductingEquipment = res[GroundingImpedance_name] 

        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])
        res[Node_name].Terminal.append(res[terminal_name + "1"])
        res[Node_name].Terminal.append(res[terminal_name + "2"])

    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result

        


# Switch Module with zero Conductance to simulate an open clamp in network "import_result" at the node with name "mRID"
def add_OpenClamp_Switch(import_result, version, mRID, name = "OpenClamp_Switch"):
    res = import_result['topology']
    TopologicalNode = ''
    
    if name in res:
        print(name, "is already included, ... create OpenClamp_Switch with new mRID")
        name = name + str(list(res.keys()).count("OpenClamp_Switch") + 1)

    if mRID in res:
        if 'TopologicalNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID]
        elif 'ConnectivityNode' in str(type(res[mRID])):
            TopologicalNode = res[mRID].TopologicalNode

    if TopologicalNode != '':
        # Add two Topological Nodes
        BaseVoltage = TopologicalNode.BaseVoltage
        v = TopologicalNode.SvVoltage.v
        angle = TopologicalNode.SvVoltage.angle
        Node_name = "Node_" + name
        import_result = add_TopologicalNode(import_result, "cgmes_v2_4_15", BaseVoltage, v, angle, Node_name)

        terminal_name = 'Terminal_' + name
        SwitchSchedule_name = 'Schedule_' + name 
        GroundingImpedance_name = 'GroundingImpedance_' + name

        # module_name = "cimpy." + version + ".Equipment."
        module_name = "cimpy." + version + "."
        
        # Create Terminals
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name] = terminal_class(mRID=terminal_name,
                                            name=terminal_name,
                                            TopologicalNode=TopologicalNode)
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name + "1"] = terminal_class(mRID=terminal_name + "1",
                                            name=terminal_name + "1",
                                            TopologicalNode=res[Node_name])
        
        terminal_module = importlib.import_module((module_name + 'Terminal'))
        terminal_class = getattr(terminal_module, 'Terminal')
        res[terminal_name + "2"] = terminal_class(mRID=terminal_name + "2",
                                            name=terminal_name + "2",
                                            TopologicalNode=res[Node_name])
        

        # Create Components
        SwitchSchedule_module = importlib.import_module((module_name + 'SwitchSchedule'))
        SwitchSchedule_class = getattr(SwitchSchedule_module, 'SwitchSchedule')
        res[SwitchSchedule_name] = SwitchSchedule_class(mRID=SwitchSchedule_name,
                                name=SwitchSchedule_name,
                                timeStep = 0.5,    # The time between each pair of subsequent regular time points in sequence order.
                                endTime = 1)       # The time for the last time point
        
        switch_module = importlib.import_module((module_name + 'Switch'))
        switch_class = getattr(switch_module, 'Switch')
        res[name] = switch_class(mRID=name,
                                name=name,
                                SwitchSchedules = [res[SwitchSchedule_name]],
                                Terminals = [res[terminal_name], res[terminal_name + "1"]])
        
        GroundingImpedance_module = importlib.import_module((module_name + 'GroundingImpedance'))
        GroundingImpedance_class = getattr(GroundingImpedance_module, 'GroundingImpedance')
        res[GroundingImpedance_name] = GroundingImpedance_class(mRID=GroundingImpedance_name,
                                name=GroundingImpedance_name,
                                r = 10e10,
                                Terminals = [res[terminal_name + "2"]])
        
        
        res[terminal_name].ConductingEquipment = res[name]
        res[terminal_name + "1"].ConductingEquipment = res[name]
        res[terminal_name + "2"].ConductingEquipment = res[GroundingImpedance_name] 

        # Append the Terminal List of the TopologicalNode
        res[mRID].Terminal.append(res[terminal_name])
        res[Node_name].Terminal.append(res[terminal_name + "1"])
        res[Node_name].Terminal.append(res[terminal_name + "2"])

    else:
        print('No Terminal with mRID ', mRID, ' found in object list!')
        return 0

    import_result['topology'] = res

    return import_result
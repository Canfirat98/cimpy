import sys
sys.path.insert(0,'/home/mmo-cya/dpsim/build')
import dpsimpy

#Nodes
Nodes = []
PiLines = []
Slacks = []

def CIMpyToDPsim(CIM_network):

    sim_name_pf = "SP_ReducedOrderSG_SMIB_" + "PF_"
    sim_pf = dpsimpy.Simulation(sim_name_pf, dpsimpy.LogLevel.debug)

    res = CIM_network["topology"]
    gnd = dpsimpy.sp.SimNode.gnd
    
    for i in res:
        if 'TopologicalNode' in str(type(res[i])):
            ### Nodes
            n1 = dpsimpy.sp.SimNode(res[i].mRID, dpsimpy.PhaseType.Single)
            Nodes.append(n1)
        
        ### Components 

        if 'ACLineSegment' in str(type(res[i])):
            pi_line = dpsimpy.sp.ph1.PiLine(res[i].mRID, dpsimpy.LogLevel.debug)
            pi_line.set_parameters(R= res[i].r,
                                #L=line_inductance,
                                #C=line_capacitance,
                                G=res[i].gch)
            PiLines.append(pi_line)

        if 'ExternalNetworkInjection' in str(type(res[i])):
            # Slack
            slack = dpsimpy.sp.ph1.NetworkInjection(res[i].mRID, dpsimpy.LogLevel.debug)
            #slack.set_parameters(V_ref=nominal_voltage_mv)
            Slacks.append(slack)

        


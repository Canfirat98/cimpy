from enum import Enum
import numpy as np
import pandas as pd
import villas.dataprocessing.readtools as rt
from villas.dataprocessing.timeseries import TimeSeries as ts
import dpsimpy
#import anm_control

kw_w  = 1e3
kw_mw = 1e-3
mw_w  = 1e6
w_kw  = 1e-3
w_mw  = 1e-6
mw_kw = 1e3

class LogLevel(Enum):
    DEBUG = dpsimpy.LogLevel.debug
    INFO  = dpsimpy.LogLevel.info
    WARN  = dpsimpy.LogLevel.warn
    ERROR = dpsimpy.LogLevel.err
    OFF   = dpsimpy.LogLevel.off
    
class GeneratorType(Enum):
    PVNode             = dpsimpy.GeneratorType.PVNode
    IdealVoltageSource = dpsimpy.GeneratorType.IdealVoltageSource
    TransientStability = dpsimpy.GeneratorType.TransientStability

                
class SystemTopology():
    """
    This class represents the system topology class of DPSim. 
    It allows to read CIM files, add and modify components of the topology
    and retrieve information of the topology. At this moment it only works
    with the SP solver of DPSim.
    """
    
    def __init__(self, name):
        self.system = None
        self.name = name
        self.frequency = None
        self.reader = None
        self.sim = None
        self.logger = None
        self.logger_name = None
        self.ts_dpsimpy = None
        self.ts_dpsimpy = None
        self.base_voltage = None
     
    def read_topology(self, system_frequency, files, gen_typ=GeneratorType.PVNode):
        """
        Parses data from CIM files into the CPS system object
        
        :param system_frequency: 
        :param files: describe about parameter load_id
        :param V_nom: describe about parameter V_nom
        :param p_load: describe about parameter p_load
        :param q_load: describe about parameter q_load
        :param log_level: describe about parameter log_level
        """ 
        self.frequency = system_frequency
        self.reader = dpsimpy.CIMReader(self.name)
        self.system = self.reader.loadCIM(self.frequency, files, dpsimpy.Domain.SP, dpsimpy.PhaseType.Single, gen_typ.value)

    def set_simulation(self, step_size=1, final_time=1, log_level=LogLevel.INFO):
        """
        set_simulation does blah blah blah.

        :param step_size: describe about parameter step_size
        :param final_time: describe about parameter final_time
        :param log_level: describe about parameter log_level
        """ 
    
        self.sim = dpsimpy.Simulation(self.name, log_level.value)
        self.sim.set_system(self.system)
        self.sim.set_domain(dpsimpy.Domain.SP)
        self.sim.set_solver(dpsimpy.Solver.NRP)
        self.sim.set_time_step(step_size)
        self.sim.set_final_time(final_time)


    ### Functions to log and read results ###

    def get_logger(self, name=""):
        """
        get_logger does blah blah blah.

        :param name: describe about parameter log_dir
        :param log_dir: describe about parameter log_dir
        """ 
    
        if name=="":
            self.logger_name = self.name
            
        else:
            self.logger_name = self.name
                   
        self.logger = dpsimpy.Logger(self.logger_name)
        
    def add_logger(self):
        """
        add_logger does blah blah blah.
        """
        self.sim.add_logger(self.logger)

    def log_nodes(self, node_list=[]):
        """
        log_nodes adds nodes to logger. If node_list is empty, all nodes
        of system will be added to logger. Otherwise, only the node ids
        contained in node_list will be added to logger

        :param node_list: list containing the name of the nodes which have
                          to be logged
        """ 
    
        if node_list:
            for node in self.system.nodes:
                if node.name() in node_list:
                    self.logger.log_attribute(node.name()+'.V', 'v', node)
        else:
            for node in self.system.nodes:    
                self.logger.log_attribute(node.name()+'.V', 'v', node)

    def log_components(self, comp_class='SP::Ph1::Load', ids=[], attributes=['P', 'Q'],):
        """
        log_components does blah blah blah.

        :param comp_class: describe about parameter comp_class
            Possible values:
                - 'SP::Ph1::Load'
                - 'SP::Ph1::PiLine'
                - 'SP::Ph1::Transformer'
                - ...
        :param ids: describe about parameter ids
        :param attributes: describe about parameter attributes
        """    
    
        # get list of components to be filtered
        component_list = self.get_components_by_class(comp_class, ids)
    
        for comp in component_list:
            for attr in attributes:
                self.logger.log_attribute(comp.name() + '.' + attr, attr, comp)
    
    def read_results(self):
        """
        Read simulation results
        """
        work_dir = 'logs/'
        print(work_dir + self.logger_name + '.csv')
        self.ts_dpsimpy = rt.read_timeseries_dpsim(work_dir + self.logger_name + '.csv')
    
    def get_node_voltages(self):
        """
        return node voltage magnitude of all nodes (p.u.)
        """ 
        voltage_results_df = pd.DataFrame()
        voltage_results_dict = {}

        for node in self.system.nodes:
            voltage_results_dict[node.name()] = self.ts_dpsimpy[node.name() + '.V'].abs().values / self.base_voltage
            voltage_results_dict[node.name()] = self.ts_dpsimpy[node.name() + '.V'].abs().values / self.base_voltage
    
        voltage_results_df = pd.DataFrame.from_dict(voltage_results_dict, orient='columns')
        return voltage_results_df
    
    def get_loads_power(self):
        """
        return p and q of all loads
        """
        pq_results_df = pd.DataFrame()
        pq_results_dict = {}
        
        obj_list = self.system.list_idobjects()
        loads = {k: v for k, v in obj_list.items() if v == 'SP::Ph1::Load'}
        for load in loads.keys():
            print(load)
            pq_results_dict[load] = self.ts_dpsimpy[load+'.P'].values
            pq_results_dict[load] = self.ts_dpsimpy[load+'.Q'].values
        pq_results_df=  pd.DataFrame.from_dict(pq_results_dict, orient='columns')

        return pq_results_df
        
        
    ### Functions to edit system components and to retrieve data from dpsim topology
    
    def get_components_by_class(self, comp_class='SP::Ph1::Load', ids=[]):
        """
        get_components_class returns a list containg all objects of the class comp_class which are
        part of system. If ids is not empty, then only the components with the names indicated in 
        ids will be returned in the list.

        :param comp_class: describe about parameter logger
            Possible values:
                - 'SP::Ph1::Load'
                - 'SP::Ph1::PiLine'
                - 'SP::Ph1::Transformer'
                - ...
        :param ids: describe about parameter comp_class
        """ 
        # get components of system
        map_components = self.system.list_idobjects()    #map: (name, class)
    
        # filter components of interest
        components = []
        for key, elem in map_components.items():
            if (comp_class == elem):
                if not ids:
                    components.append(self.system.component(key))
                elif key in ids:
                    components.append(self.system.component(key))
                
        return components
    
    def get_component_names(self, comp_class):
        """
        get_component_names returns a list containg the name of all 
        componentes type comp_class presented in system

        :param system: SystemTopology object describing a DPSim topology
        :param comp_class: type of components of interest. 
            Possible values:
                 - 'SP::Ph1::Load'
                 - 'SP::Ph1::PiLine'
                 - 'SP::Ph1::Transformer'
                 - ...
        """ 
        # get components of system
        map_components = self.system.list_idobjects()    #map: (name, class)
    
        # filter components of interest
        names = []
        for key, elem in map_components.items():
            if (comp_class == elem):
                names.append(key)
    
        return names
     
    def add_load(self, load_id, V_nom, p_load, q_load, node_id=None, log_level=LogLevel.INFO):
        """
        add_load adds a new load to the topology
        
        :param load_id: load identifier
        :param V_nom: nominal voltage
        :param p_load: active power
        :param q_load: reactive power
        :param node_id: node id with which the new load has to be connected. If node is not found
                        the load will be not connected
        :param log_level: log level to be used in the new load
        """ 

        new_load = dpsimpy.sp.ph1.Load(load_id, log_level.value)
        new_load.set_parameters(- p_load, - q_load, V_nom)
        self.system.add(new_load)
        
        if node_id:
            self.connect_component_to_node(new_load, node_id)
    
   
    def connect_component_to_node(self, comp, node_id):
        """
        connect a component to node
            
        :param comp: component to be connected
        :param node_id: id of node to which the component must be connected
        """ 
        node = None
        for obj in self.system.nodes:
            if obj.name() == node_id:
                node=obj
            
        if (node==None):
            raise ValueError('System does not contain node "%s"' % (node_id))
    
        self.system.connect_component(comp, [node])
        
    def node_of_comp(self, comp_name):
        """
        node_of_comp returns the name of the node of the component comp_name
        
        :param comp_name:
        """ 
        
        comp_ptr = None
        for comp in self.system.components:
            if comp.name()== comp_name:
                comp_ptr = comp

        obj_dict = self.system.components_at_node
        node_name = None
        for node, comp in obj_dict.items():
            if (comp_ptr in comp):
                node_name = node.name()
    
        return node_name
    
    def get_active_power(self, comp_name):
        """
        return active power of component named comp_name in MW
        """ 
        
        return self.sim.get_idobj_attr(comp_name, 'P').get() * w_mw
    
    def get_reactive_power(self, comp_name):
        """
        returns reactive power of component named comp_name in MVar
        """ 
        
        return self.sim.get_idobj_attr(comp_name, 'Q').get()*w_mw
    
    def get_voltage(self, comp_name):
        """
        returns node voltage magnitude of component named comp_name in V
        """ 
        node_name = self.node_of_comp(comp_name)
        return abs(self.sim.get_idobj_attr(node_name, 'v').get())

    
   
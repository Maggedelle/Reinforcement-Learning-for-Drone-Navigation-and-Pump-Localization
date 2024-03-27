import strategoutil as sutil
from strategoutil import StrategoController

class QueueLengthController(StrategoController):
    def __init__(self, templatefile, state_names):
        super().__init__(templatefile, state_names)
        # variables to insert into the simulation *.xml file
        self.state_names = state_names
         # tag left in model_template.xml
        self.tagRule = "//TAG_{}"

    def insert_state(self, state_dict):
        """
        Uses tag rule to insert state values of [E, S, phase]
        at the appropriate position in the simulation *.xml file
        """
        for name, value in state_dict.items():
            tag = self.tagRule.format(name)
            value = str(value)
            sutil.insert_to_modelfile(
                self.simulation_file, tag, value)
            
    def generate_query_file(self, optimize, learning_param, state_vars, point_vars, observables):
        strategy = self.generate_strategy_query(optimize, learning_param, state_vars, point_vars)
        simulate = self.generate_simulate_query(observables)
        f = open("query.q", "w")
        f.write(strategy +"\n \n" + simulate)
        f.close()


    def generate_strategy_query(self,optimize, learning_param, state_vars, point_vars):

        time_to_reach_stop_condition = "1000"
        
        stop_condition = "(DroneController.target || time >= 10)"

        strategy_string = "strategy opt = {}({}) [<={}]".format(optimize, learning_param, time_to_reach_stop_condition)
        strategy_string += "{" + ",".join(state_vars) + "}"
        strategy_string += "->"
        strategy_string += "{" + ",".join(point_vars) + "}"
        strategy_string += " : <> " + stop_condition

        return strategy_string
    

    def generate_simulate_query(self, observables) :
        simulate_length = "1000"
        stop_condition = "(DroneController.target || time >= 10)"

        simulate_string = "simulate [<={};1]".format(simulate_length)
        simulate_string += " {" + ",".join(observables) + "}"
        simulate_string += " : " + stop_condition
        simulate_string += " under opt"

        return simulate_string

    def run(self, queryfile="", learning_args={}, verifyta_path="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta"):
        output = super().run(queryfile, learning_args, verifyta_path)
        # parse output
        tpls = sutil.get_float_tuples(output)
     

        values = []
        last_zero_value = 0
        previous_was_zero = False
        for i in range(len(tpls)):
            if(tpls[i][0] == 0):
                last_zero_value = tpls[i][1]
                previous_was_zero = True
            
            if(tpls[i][0] != 0 and previous_was_zero):
                previous_was_zero = False
                values.append(last_zero_value)
                
        return values
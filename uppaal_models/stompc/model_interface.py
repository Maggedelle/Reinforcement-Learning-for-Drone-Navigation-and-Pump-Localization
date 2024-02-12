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

    def run(self, queryfile="", learning_args={}, verifyta_path="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta"):
        output = super().run(queryfile, learning_args, verifyta_path)

        # parse output
        tpls = sutil.get_int_tuples(output)
        result = sutil.get_duration_action(tpls, max_time=1000)
        durations, actions = list(zip(*result)) 
        return durations, actions
import random
import argparse
import os

from model_interface import QueueLengthController


def get_random_x_and_y(x,y):
    x += 1
    y += 1

    return x, y

def run(template_file, query_file, verifyta_path):
    MIN_GREEN = 4

    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y"])

    # initial plant state
    x = 0
    y = 0
    L = 30 # simulation length
    K = 4  # every K we will do MPC

    for k in range(L):
        # run plant
        x,y = get_random_x_and_y(x,y)

        # report
        #print("Step: {}, x: {} cars, y: {} cars".format(k, x, y))

        if k % K == 0:
            # at each MPC step we want a clean template copy
            # to insert variables
            controller.init_simfile()
            
            # insert current state into simulation template
            state = {
                "x": x,
                "y": y,
            }
            controller.insert_state(state)

            # to debug errors from verifyta one can save intermediate simulation file
            # controller.debug_copy(templatePath.replace(".xml", "_debug.xml"))

            # run a verifyta querry to simulate optimal strategy
            durations, phase_seq = controller.run(
                queryfile=query_file,
                verifyta_path=verifyta_path)
            
            #print(durations)
            print(phase_seq)

            # switch phases if optimal solution changes phase
            # after minimum green time, stay otherwise
            next_duration, next_phase = durations[0], phase_seq[0]
            if next_duration == MIN_GREEN and len(phase_seq) > 1:
                next_duration, next_phase = durations[1], phase_seq[1]

            #print("  Decison: phases {} to {} for {}s".format(
             #   1, next_phase, next_duration))
            #phase = next_phase

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--template-file", default="drone_model_stompc.xml", 
        help="Path to Stratego .xml file model template")
    ap.add_argument("-q", "--query-file", default="query.q",
        help="Path to Stratego .q query file")
    ap.add_argument("-v", "--verifyta-path", default="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta", help=
        "Path to verifyta executable")
    args = ap.parse_args()

    base_path = os.path.dirname(os.path.realpath(__file__)) 
    template_file = os.path.join(base_path, args.template_file)
    query_file = os.path.join(base_path, args.query_file)

    run(template_file, query_file, args.verifyta_path)
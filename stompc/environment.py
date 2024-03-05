
#DIMENSIONS IN METERS
width = 8.0
height = 13.0

#GRANUALITY
granuality = 1

def generate_environment():
    environment = []
    
    width_start = width
    height_start = height

    width_index = 0

    while width_start > 0:
        height_start = height
        environment.append([])
        while height_start > 0:
            environment[width_index].append(0)
            height_start = height_start - granuality
        width_index = width_index + 1
        width_start -= granuality

    return environment


def build_uppaal_environment_array_string(environment):
    environment_array_string = "int environment[{}][{}] = ".format(len(environment), len(environment[0]))
    environment_array_string += "{\n"

    lst_strings = []
    for lst_ele in environment:
        arr_string = "  {"
        arr_string += ','.join([str(x) for x in lst_ele])
        arr_string += "}"
        lst_strings.append(arr_string)
    environment_array_string += ',\n'.join(lst_strings) + "\n}"


    return environment_array_string

    
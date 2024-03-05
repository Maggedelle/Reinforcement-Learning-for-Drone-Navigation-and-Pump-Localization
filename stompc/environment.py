
#DIMENSIONS IN METERS
width = 8.0
height = 13.0

#GRANUALITY
granuality = 0.5

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

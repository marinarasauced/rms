
import numpy as np


def get_viewpoints(read_path):
    """
    
    """
    with open(read_path, "r") as file:
        lines = file.readlines()
    data = []
    for line in lines:
        values = [float(x) for x in line.strip.split(",")]
        data.append(values)
    return np.asarray(data)
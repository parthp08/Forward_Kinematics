import numpy as np
import pandas as pd

def DH_df2arr(data_frame):
    """
    TODO
    """
    a_arr = np.array(data_frame['a(i-1)'], dtype=float)
    alpha_arr = np.array(data_frame['alpha(i-1)'], dtype=float)
    d_arr = np.array(data_frame['d(i)'], dtype=float)
    theta_arr = np.array(data_frame['theta(i)'], dtype=float)
    return a_arr, alpha_arr, d_arr, theta_arr
    
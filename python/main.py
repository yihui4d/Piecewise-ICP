# This is a sample Python script.
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import ctypes
import os

if __name__ == '__main__':
    # Load the C++ DLL
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "PCL_DLL/PwICP_x64R.dll")  # PwICP_x64R_withAccuracyAnalysis
    mydll = ctypes.cdll.LoadLibrary(dllPath)

    # Define function signatures
    mydll.PiecewiseICP_pair_call.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    mydll.PiecewiseICP_pair_call.restype = ctypes.c_bool
    mydll.PiecewiseICP_4D_call.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float]
    mydll.PiecewiseICP_4D_call.restype = ctypes.c_bool


    # --- Example 1: Pairwise registration ---
    conf_path_pair = "configuration_files/configuration_pair.txt"  # Chenge here! Path to config file
    output_prefix = "results/PairReg/"                             # Chenge here! The prefix of output files
    conf_bytes = ctypes.create_string_buffer(bytes(conf_path_pair, encoding='utf8'))
    out_bytes = ctypes.create_string_buffer(bytes(output_prefix, encoding='utf8'))
    success1 = mydll.PiecewiseICP_pair_call(conf_bytes, out_bytes)
    print("\nPairwise registration success:", success1)


    # --- Example 2: 4D point cloud registration ---
    conf_path_4d = "configuration_files/configuration_4d.txt"  # Chenge here! Path to config file
    start_epoch = 0          # Reference epoch index in the scan file list (0 as the first scan).
    num_epochs = 20          # Total number of scans
    pair_mode = -1           # Mode of pair sequence determination:
                                #  0: All scans to reference
                                # >0: Fixed interval
                                # <0: Adaptive interval
    overlap_thd = 0.75       # Overlap ratio threshold for adaptive registration pairs (default: 75%)
    conf_bytes_4d = ctypes.create_string_buffer(bytes(conf_path_4d, encoding='utf8'))
    success2 = mydll.PiecewiseICP_4D_call(conf_bytes_4d, start_epoch, num_epochs, pair_mode, overlap_thd)
    print("\n4DPC registration success:", success2)



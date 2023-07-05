import os
import pickle
import datetime
import numpy as np


class Logger():
    def __init__(self, folder_path=None):
        """Log timing and performances information inside a dictionnary.

        Args:
            folder_path (str, optional): folder path. Defaults to "".
        """
        # Save information in a dictionnary
        self._profiler = {
            "potential_number": [],
            "timing_potential": 0,
            "timing_MIP": 0,
            "timing_configuration": 0,
            "timing_processing": [],
            "processing_number": []
        }
        if folder_path is None:
            folder_path = os.getcwd()
        # Remove any trailing slahses from the folder path
        folder_path = folder_path.rstrip(os.path.sep)

        # Create the file path
        file_name = os.path.join(folder_path,
                                 "planner_" + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%fZ") + ".bin")
        self._file = open(file_name, "ab")
        self._counter = 0
        self._flushFrequency = 2

    def update_logger(self, planner_profiler):
        """Update the logger's profiler with the values from the planner.
        (Subset dict of the main logger)

        Args:
            planner_profiler (dict): Surface planner profiler.
        """
        self._profiler.update(planner_profiler)

    def reset_data(self):
        """Reset the profiler.
        """
        self._profiler["potential_number"] = []
        self._profiler["timing_potential"] = 0
        self._profiler["timing_MIP"] = 0
        self._profiler["timing_configuration"] = 0
        self._profiler["timing_processing"] = []
        self._profiler["processing_number"] = []

    def write_data(self):
        """ Push data inside the binary file.
        """
        # Point to end of the file
        self._file.seek(0, 2)
        # Seriallize data as binary file
        binary_data = pickle.dumps(self._profiler)
        # Write the binary data to file buffer
        self._file.write(binary_data)
        # Flush the buffer to disk every n iterations
        # if self._counter % self._flushFrequency == 0:
        self._file.flush()
        self._counter += 1

    def close_file(self):
        self._file.close()


def process_data(filename):
    """Create a large dictionnary to analyse the datas.

    Args:
        filename (str): filename of the binary file.
    """
    profiler = {
        'potential_number': [],
        "processing_number": [],
        'timing_MIP': [],
        'timing_configuration': [],
        'timing_potential': [],
        'timing_processing': [],
        "timing_pre_selection_surfaces": [],
        "max_potential": []
    }

    with open(filename, mode='rb') as file:  # b is important -> binary
        while True:
            try:
                profiler_tmp = pickle.load(file)
                if profiler_tmp["timing_MIP"] > 0.3:  # Problem
                    print("problem")
                else:
                    profiler["potential_number"].append(np.mean(profiler_tmp["potential_number"]))
                    profiler["max_potential"].append(np.max(profiler_tmp["potential_number"]))
                    profiler["timing_MIP"].append(profiler_tmp["timing_MIP"])
                    profiler["timing_configuration"].append(profiler_tmp["timing_configuration"])
                    profiler["timing_potential"].append(profiler_tmp["timing_potential"])
                    profiler["timing_pre_selection_surfaces"].append(profiler_tmp["timing_potential"] +
                                                                     profiler_tmp["timing_configuration"])
                    if len(profiler_tmp["timing_processing"]) > 0:
                        profiler["timing_processing"].append(np.mean(profiler_tmp["timing_processing"]))
                        profiler["processing_number"].append(np.mean(profiler_tmp["processing_number"]))
                    # else:
                    #     profiler["timing_processing"].append(np.nan)
                    #     profiler["processing_number"].append(np.nan)
            except EOFError:
                # End of file reached
                break

    return profiler


def print_stats(filename):

    profiler = process_data(filename)
    np.set_printoptions(precision=3)

    print("Number of MIP cycles : ", len(profiler["timing_MIP"]))
    print("Number of post-processing : ", len(profiler["timing_processing"]))

    print("\n")
    print("potential_number : ")
    print("Mean : ", np.mean(profiler["potential_number"]))
    print("Min : ", np.min(profiler["potential_number"]))
    print("Max : ", np.max(profiler["max_potential"]))
    print("std : ", np.std(profiler["potential_number"]))

    print("\n")
    print("timing_MIP : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["timing_MIP"]))
    print("Min [ms]: ", 1000 * np.min(profiler["timing_MIP"]))
    print("Max [ms]: ", 1000 * np.max(profiler["timing_MIP"]))
    print("std [ms]: ", 1000 * np.std(profiler["timing_MIP"]))

    print("\n")
    print("timing_configuration : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["timing_configuration"]))
    print("Min [ms]: ", 1000 * np.min(profiler["timing_configuration"]))
    print("Max [ms]: ", 1000 * np.max(profiler["timing_configuration"]))
    print("std [ms]: ", 1000 * np.std(profiler["timing_configuration"]))

    print("\n")
    print("timing_potential : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["timing_potential"]))
    print("Min [ms]: ", 1000 * np.min(profiler["timing_potential"]))
    print("Max [ms]: ", 1000 * np.max(profiler["timing_potential"]))
    print("std [ms]: ", 1000 * np.std(profiler["timing_potential"]))

    print("\n")
    print("timing_pre_selection_surfaces : ")
    print("Mean [ms]: ", 1000 * np.mean(profiler["timing_pre_selection_surfaces"]))
    print("Min [ms]: ", 1000 * np.min(profiler["timing_pre_selection_surfaces"]))
    print("Max [ms]: ", 1000 * np.max(profiler["timing_pre_selection_surfaces"]))
    print("std [ms]: ", 1000 * np.std(profiler["timing_pre_selection_surfaces"]))

    print("\n")
    print("timing_processing : ")
    print("Mean [ms]: ", 1000 * np.nanmean(profiler["timing_processing"]))
    print("Min [ms]: ", 1000 * np.nanmin(profiler["timing_processing"]))
    print("Max [ms]: ", 1000 * np.nanmax(profiler["timing_processing"]))
    print("std [ms]: ", 1000 * np.nanstd(profiler["timing_processing"]))

    print("\n")
    print("Processing number of surfaces : ")
    print("Mean : ", np.nanmean(profiler["processing_number"]))
    print("Min : ", np.nanmin(profiler["processing_number"]))
    print("Max : ", np.nanmax(profiler["processing_number"]))
    print("std : ", np.nanstd(profiler["processing_number"]))


def plot_stats(filename):
    import matplotlib.pyplot as plt
    profiler = process_data(filename)
    plt.figure()
    keys = ["timing_pre_selection_surfaces", "timing_potential", "timing_configuration", "timing_MIP"]
    for key in keys:
        if not key == "processing_number":
            x = np.arange(0, len(profiler.get(key)))
            plt.plot(x, profiler.get(key), label=key)
    plt.legend()
    plt.title("Timing MIP")

    plt.figure()
    x = np.arange(0, len(profiler.get("timing_processing")))
    plt.plot(x, profiler.get("timing_processing"), label="timing_processing")
    plt.legend()
    plt.title("Timing processing surfaces")

    plt.show()

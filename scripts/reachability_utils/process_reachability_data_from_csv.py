import argparse
import numpy as np
import skfmm

import os
import rospkg
import pandas
import time
import tqdm


def create_binary_reachability_space(reachability_data_filepath):
    data = pandas.read_csv(reachability_data_filepath, header=None, delimiter=' ')
    data = data.values
    print "csv data loaded."

    print "creating ND grid with reachabiity data full ..."

    data = data[:, 1:]
    unique_elements = map(np.unique, data.transpose())
    unique_elements = unique_elements[:-1]

    grid_size = [e.shape[0] for e in unique_elements]
    binary_reachability_space = np.zeros(shape=grid_size)
    # binary_reachability_space = np.zeros(shape=grid_size, dtype=np.int8)

    for i, row in enumerate(tqdm.tqdm(data)):
        idx = [np.argwhere(u_elems == val)[0][0] for val, u_elems in zip(row, unique_elements)]
        binary_reachability_space[tuple(idx)] = int(data[i, -1])

    print "created binary reachability space of dimension: \t{}".format(binary_reachability_space.shape)

    step_size = map(lambda x: x[1] - x[0] if len(x) > 1 else 1, unique_elements)
    step_size = np.array(step_size)

    mins = map(lambda x: x[0], unique_elements)
    mins = np.array(mins)

    dims = np.array(binary_reachability_space.shape)

    return binary_reachability_space, mins, step_size, dims


def generate_SDF_from_binary_space(binary_reachability_space,
                                   periodic=[False, False, False, True, True, True],
                                   dx=[1] * 6):
    print "now generating sdf ..."
    # Generate sdf
    binary_reachability_space -= 0.5
    sdf_reachability_space = skfmm.distance(binary_reachability_space, periodic=periodic, dx=dx)
    binary_reachability_space += 0.5  # undo previous operation

    return sdf_reachability_space


def save_reachability_space_to_dir(binary_reachability_space, mins, step_size, dims,
                                    sdf_reachability_space, reach_data_dir):
    if not os.path.exists(reach_data_dir):
        os.makedirs(reach_data_dir)

    print "now saving data to file..."

    processed_file_path = os.path.join(reach_data_dir, 'reach_data')
    binary_reachability_space.tofile(open(processed_file_path + '.full', 'w'))
    sdf_reachability_space.tofile(open(processed_file_path + '.sdf', 'w'))

    # save dimension to file
    def save_array(filename, array):
        with open(filename, 'w') as f:
            f.write(', '.join(str(x) for x in array))

    save_array(processed_file_path + '.step', step_size)
    save_array(processed_file_path + '.mins', mins)
    save_array(processed_file_path + '.dims', dims)


def load_reachability_data_from_dir(reach_data_dir):
    processed_file_path = os.path.join(reach_data_dir, 'reach_data')

    mins = np.loadtxt(processed_file_path + '.mins', delimiter=',', dtype=float)
    step_size = np.loadtxt(processed_file_path + '.step', delimiter=',', dtype=float)
    dims = np.loadtxt(processed_file_path + '.dims', delimiter=',', dtype=int)

    binary_reachability_space = np.fromfile(processed_file_path + '.full', dtype=float)
    binary_reachability_space = binary_reachability_space.reshape(dims)
    sdf_reachability_space = np.fromfile(processed_file_path + '.sdf', dtype=float)
    sdf_reachability_space = sdf_reachability_space.reshape(dims)

    return binary_reachability_space, mins, step_size, dims, sdf_reachability_space


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Processes reachability data into sdf space.')

    DATA_ROOT_DEFAULT = rospkg.RosPack().get_path('reachability_utils') + '/data'
    parser.add_argument('--DATA_ROOT',
                        default=DATA_ROOT_DEFAULT,
                        type=str,
                        help='Directory containing generated reachability data .')

    parser.add_argument('DATA_FILENAME',
                        type=str,
                        help='Name of csv file that contains reachability data ')

    args = parser.parse_args()

    reachability_data_filepath = os.path.join(args.DATA_ROOT, args.DATA_FILENAME)
    t0 = time.time()
    binary_reachability_space, mins, step_size, dims = create_binary_reachability_space(reachability_data_filepath)
    t1 = time.time()
    print "Loading and creating binary_reachability_space took\t {} secs".format(t1 - t0)

    # generate SDF
    t0 = time.time()
    sdf_reachability_space = generate_SDF_from_binary_space(binary_reachability_space)
    t1 = time.time()
    print "Generating SDF from binary_reachability_space took\t {} secs".format(t1 - t0)

    reach_data_dir_name = args.DATA_FILENAME.replace('.csv', "_".join(map(str, dims)))
    reach_data_dir = os.path.join(args.DATA_ROOT, reach_data_dir_name)
    save_reachability_space_to_dir(binary_reachability_space, mins, step_size, dims, sdf_reachability_space,
                                    reach_data_dir)
    #
    # import IPython
    #
    # IPython.embed()
    # data = load_reachability_data_from_dir(reach_data_dir)
    # binary_reachability_space_test, mins_test, step_size_test, dims_test, sdf_reachability_space_test = data


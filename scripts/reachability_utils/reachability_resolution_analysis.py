import argparse
import numpy as np
import skfmm

import os
import rospkg
import pandas
import time
import yaml

from process_reachability_data_from_csv import load_reachability_data_from_dir, generate_SDF_from_binary_space

# create 6D reachability space from csv

# create SDF

# write to file

# interpolate function


# Experiments

# Subsampled reachspace

# Interpolation in ordinary 6D space versus SDF

# vary resolution

# save time for generating sdf

def downsample_steps_binary_reachability_space(binary_reachability_space, mins, step_size, dims,
                                               downsample_steps):
    binary_reachability_space_downsampled = binary_reachability_space[::downsample_steps[0],
                                            ::downsample_steps[1],
                                            ::downsample_steps[2],
                                            ::downsample_steps[3],
                                            ::downsample_steps[4],
                                            ::downsample_steps[5]]

    step_size_downsampled = step_size * downsample_steps
    dims_downsampled = dims / downsample_steps
    return binary_reachability_space_downsampled, mins, step_size_downsampled, dims_downsampled


def interpolate_pose_in_reachability_space_grid(reachability_space_grid, mins, step_size, dims, query_pose):

    ndims = dims.size
    num_bins_from_origin = np.zeros(ndims, dtype=float)
    num_corners = 1 << ndims

    # determine the bin_index
    for i in range(ndims):
        num_bins_from_origin[i] = (query_pose[i] - mins[i]) / step_size[i]

        if num_bins_from_origin[i] < 0:
            num_bins_from_origin[i] = 0
        if num_bins_from_origin[i] >= dims[i]:
            num_bins_from_origin[i] = dims[i] - 1

    corner_values = np.zeros(num_corners)
    corner_weights = np.zeros(num_corners, dtype=float)
    corner_indices = np.zeros((num_corners, ndims), dtype=int)

    for i in range(num_corners):
        weight_per_dim = np.zeros(ndims)
        index_per_dim = np.zeros(ndims, dtype=int)
        for dim in range(ndims):
            is_after = (i >> dim) & 1       # 000100
            dist_to_corner = num_bins_from_origin[dim] - int(num_bins_from_origin[dim])       # value between 0 and 1.

            weight_per_dim[dim] = ((1.0 - is_after) * (1.0 - dist_to_corner) + (is_after) * (dist_to_corner))
            index_per_dim[dim] = min(int(num_bins_from_origin[dim]) + is_after, dims[dim]-1)

        weight = weight_per_dim.prod()
        corner_weights[i] = weight
        corner_indices[i, :] = index_per_dim
        # corner_values[i] = reachability_space_grid.flatten()[np.ravel_multi_index(index_per_dim, reachability_space_grid.shape)]
        # corner_values[i] = reachability_space_grid[*index_per_dim.tolist()]
    corner_values = reachability_space_grid.flatten()[
        np.ravel_multi_index(corner_indices.T, reachability_space_grid.shape)]
    value = np.multiply(corner_values, corner_weights).sum()
    return value


def get_reachability_fitness_score(reachability_space_grid, mins, step_size, dims, query_poses):

    reachability_prediction = np.zeros(shape=(query_poses.shape[0]), dtype=bool)

    for i, query in enumerate(query_poses):
        # interpolate value in space
        value = interpolate_pose_in_reachability_space_grid(reachability_space_grid, mins, step_size, dims, query[1:7])

        if value > 0:
            reachability = 1
        else:
            reachability = 0
        reachability_prediction[i] = reachability
    ground_truth = query_poses[:, -1]
    fitness = {}
    fitness['true_positive'] = np.sum(np.logical_and(reachability_prediction, ground_truth)) * 1. / np.sum(ground_truth)
    fitness['true_negative'] = np.sum(np.logical_and(1 - reachability_prediction, 1-ground_truth)) * 1. / np.sum(1-ground_truth)
    fitness['false_positive'] = 1 - fitness['true_positive']
    fitness['false_negative'] = 1 - fitness['true_negative']
    fitness['accuracy'] = 1 - np.mean(np.logical_xor(reachability_prediction, ground_truth.astype(bool)))
    fitness['precision'] = fitness['true_positive'] / (fitness['true_positive'] + fitness['false_positive'])

    return fitness


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Processes reachability data into sdf space.')

    DATA_ROOT_DEFAULT = rospkg.RosPack().get_path('reachability_utils') + '/data'
    parser.add_argument('--DATA_ROOT',
                        default=DATA_ROOT_DEFAULT,
                        type=str,
                        help='Directory containing raw reachability data .')

    parser.add_argument('--config_file',
                        default='reachability_resolution_analysis.yaml',
                        type=str)

    args = parser.parse_args()

    config = yaml.load(open(args.config_file))
    for k, v in config.items():
        args.__dict__[k] = v

    # load dense reachability space
    reach_data_dir = os.path.join(args.DATA_ROOT, args.dense_reachability_space_dir)
    data = load_reachability_data_from_dir(reach_data_dir)
    binary_reachability_space, mins, step_size, dims, sdf_reachability_space = data

    # Load randomly generated test/query poses
    args.query_pose_data_file_path = os.path.join(args.DATA_ROOT, args.query_pose_data_file_name)
    data = pandas.read_csv(args.query_pose_data_file_path, header=None, delimiter=' ')
    query_poses_data = data.values

    for downsample_steps in args.downsample_steps_options:
        space_gen_stats = {}

        downsampled_space_data = downsample_steps_binary_reachability_space(
            binary_reachability_space,
            mins, step_size, dims, downsample_steps)
        binary_reachability_space_downsampled, mins, step_size_downsampled, dims_downsampled = downsampled_space_data

        # generate SDF
        t0 = time.time()
        sdf_reachability_space_downsampled = generate_SDF_from_binary_space(binary_reachability_space_downsampled)
        t1 = time.time()
        print "creating sdf took\t {} secs".format(t1 - t0)
        space_gen_stats['sdf_generation_time'] = (t1 - t0)

        # get reachability test score
        fitness_stats = get_reachability_fitness_score(sdf_reachability_space_downsampled,
                                                       mins, step_size_downsampled, dims_downsampled, query_poses_data)

        print "\nstep_size: \t{} \n {}".format(step_size_downsampled, fitness_stats)
        space_gen_stats.update(fitness_stats)
        space_gen_stats['mins'] = mins
        space_gen_stats['step_size'] = step_size_downsampled
        space_gen_stats['dims'] = dims_downsampled
    import IPython

    IPython.embed()
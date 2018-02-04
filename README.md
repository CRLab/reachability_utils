# reachability_utils

To generate, unipolar, bipolar or multi-polar reachability space, check out *reach_data_gen.py* and run with:
```bash
 rosrun reachability_utils reach_data_gen.py
```


To generate reachability SDF space from a csv reachability data gotten from ik checks, create a data directory at the top level of the reachability_utils ros package and put the csv file in it.
This will put the generated SDF space and necessary info in folder beside the input csv file and named similar to the csv file.

```bash
python process_reachability_data_from_csv.py fetch_dense_reachability.csv
```


To do reachability resolution analysis,
- generate random query poses using the reachability_space_generation package and put the resulting csv file in the data folder of this (reachability_utils) package.
- Confirm that you have the dense reachability space in the data folder.
- In the yaml file: *reachability_resolution_analysis.yaml*, enter the necessary folder/filenames and the resolution levels (downsample factors) .
- Run the following:
```bash
python reachability_resolution_analysis.py reachability_resolution_analysis.yaml
```
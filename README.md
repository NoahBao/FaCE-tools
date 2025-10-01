# hdf5-scan-converter

Tool for converting HDF5 point cloud files into OBJ files.

## Setup

Run:
`pip install -r requirements.txt`

to install all necessary Python libraries.

## Usage

Run:
`python converter.py <hdf5 filename> <output obj filename> [trajectory filename]`

where the HDF5 input filename and OBJ output filename are required, and the trajectory filename is optional in case each point cloud scan must be transformed according to a camera position before being combined with other scans.

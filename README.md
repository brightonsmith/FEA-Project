# Finite Element Analysis Project

Author: Brighton Smith

## Description

This is a basic program to perform finite element analysis on a beam.

## Requirements

- A standard C++ environment with a g++ compiler is required.
- The non-standard library in use, Eigen/Dense, has been included in the `FEA-Project` directory. The full version of Eigen is available to download ([here](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)), but it's not necessary to run this program.

## Usage

### Windows

1. Open Windows PowerShell.
2. Navigate to the directory `FEA-Project/src` using `cd "Your/path/to/src"`.
3. Type `./fea.bat` and press Enter to compile and run the program.
4. The output file `outputFile.txt` will be generated in the `FEA-Project/src` directory.

### Linux

1. Open Bash.
2. Navigate to the directory `FEA-Project/src` using `cd "Your/path/to/FEA-Project/src"`.
3. Type `make` and press Enter to compile the program.
4. Type `./fea` and press Enter to run the program.
5. The output file `outputFile.txt` will be generated in the `FEA-Project/src` directory.

## Input File Format

The input file follows this format:

- `number of nodes`   `number of elements`
- `node#`   `x-coordinate`

- `element#`   `connectivity of element`

- `Young's modulus`   `height`   `width`

- `number of constraints`
- `list of degrees of freedom (DOFs) specified to be zero`

- `number of point loads`
- `DOF`   `load`

Edit `inputFile.txt` in the `FEA-Project/Build` directory to modify the beam problem.

## Notes

- This project assumes simple two-node cubic elements for the beam.
- This project assumes a rectangular cross-sectional area.
- Height, width, and nodal coordinates are assumed to be given in inches.
- Forces are assumed to be given in lbf, while moments are assumed to be given in lb-in.

## Additional Updates for Future Versions

- Add support for distributed/thermal loads.
- Add support for labels in the input file.
- Calculate exact forces at element boundaries using the variationally consistent method.
- Display plots of deflection, moment, and shear force.
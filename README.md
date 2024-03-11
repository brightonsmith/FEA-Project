# Finite Element Analysis Project

Author: Brighton Smith

## Description

This is a basic program to perform finite element analysis on a beam.

## Usage

1. Open Windows PowerShell.
2. Navigate to the directory `FEA-Project/Build` using `cd "Your/path/to/FEA-Project/Build"`.
3. Type `./fea.bat` and press Enter to compile and run the program.
4. The output file `outputFile.txt` will be generated in the `FEA-Project/Build` directory.

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

The zip folders represent the stages this project was submitted in. The final product is contained in the `FEA-Project/Build` directory.
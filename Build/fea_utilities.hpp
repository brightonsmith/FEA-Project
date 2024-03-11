#ifndef FEA_UTILITIES_HPP
#define FEA_UTILITIES_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace std;

struct Node {
    int number; 
    double x; // Global coordinate (from left)
    int dof1; 
    int dof2;
    
    Node();
    Node(int num);
};

Node::Node() : number(0), x(0.0), dof1(0), dof2(0) {} // Default constructor

Node::Node(int num) : number(num), x(0.0), dof1((num - 1)*2 + 1), dof2((num - 1)*2 + 2) {} // Number of DOFs are predetermined for beams (2 per node)

struct Element {
    int number; 
    vector<int> connectivity;
};

struct Properties {
    double youngsModulus;
    double height; // in
    double width; // in
};

struct Constraints {
    int number;
    vector<int> zeroDOFs;
};

struct Load {
    int dof;
    double magnitude; // lbf for force and lb-in for moment
};

// Prototypes
void readMesh(ifstream& inputFile, ofstream& outputFile, vector<Node>& nodes, vector<Element>& elements); // Reads nodal coordinates and element connectivity from input file
void readProperties(ifstream& inputFile, ofstream& outputFile, Properties& properties); // Reads beam properties from input file
void readConstraints(ifstream& inputFile, ofstream& outputFile, Constraints& constraints); // Reads list of dof specified to be zero from input file
void readLoads(ifstream& inputFile, ofstream& outputFile, vector<Load>& loads); // Reads load magnitudes and corresponding DOFs
void assembleGlobalStiffnessMatrix();
void imposeConstraints();
void solver();
void reportResults();

/////////////////////////////////////////////////////////////Part 1/////////////////////////////////////////////////////////////////////////
void readMesh(ifstream& inputFile, ofstream& outputFile, vector<Node>& nodes, vector<Element>& elements) {
    cout << "MESH:" << endl;
    outputFile << "MESH:" << endl;

    int numNodes, numElements;
    inputFile >> numNodes >> numElements;

    // Echo print the number of nodes and elements
    cout << "Number of nodes: " << numNodes << endl;
    cout << "Number of elements: " << numElements << endl;

    outputFile << "Number of nodes: " << numNodes << endl;
    outputFile << "Number of elements: " << numElements << endl;

    nodes.resize(numNodes);
    for (int i = 0; i < numNodes; i++) {
        int nodeNumber;
        double x;
        inputFile >> nodeNumber >> x;
        nodes[i] = Node(nodeNumber);
        nodes[i].x = x;

        cout << "Node " << nodes[i].number << ": x = " << nodes[i].x << endl; // Echo print nodal coordinates
        outputFile << "Node " << nodes[i].number << ": x = " << nodes[i].x << endl;
    }

    elements.resize(numElements);
    for (int i = 0; i < numElements; i++) {
        inputFile >> elements[i].number;
        cout << "Element " << elements[i].number << " connectivity: "; // Echo print element number
        outputFile << "Element " << elements[i].number << " connectivity: ";

        int nodeNumber;
        while (inputFile >> nodeNumber) {
            elements[i].connectivity.push_back(nodeNumber);
            cout << nodeNumber << " "; // Echo print node number
            outputFile << nodeNumber << " ";

            if (inputFile.peek() == '\n') {
                break;
            }
        }
        cout << endl;
        outputFile << endl;
    }
    cout << endl;
    outputFile << endl;
}

void readProperties(ifstream& inputFile, ofstream& outputFile, Properties& properties) {
    cout << "PROPERTIES: " << endl;
    outputFile << "PROPERTIES: " << endl;

    inputFile >> properties.youngsModulus >> properties.height >> properties.width;

    // Echo print properties
    cout << "Young's Modulus: " << properties.youngsModulus << endl;
    cout << "Height: " << properties.height << " in" << endl;
    cout << "Width: " << properties.width << " in" << endl << endl;

    outputFile << "Young's Modulus: " << properties.youngsModulus << endl;
    outputFile << "Height: " << properties.height << " in" << endl;
    outputFile << "Width: " << properties.width << " in" << endl << endl;
}

void readConstraints(ifstream& inputFile, ofstream& outputFile, Constraints& constraints) {
    cout << "CONSTRAINTS: " << endl;
    outputFile << "CONSTRAINTS: " << endl;

    inputFile >> constraints.number;

    cout << "Number of DOFs specified to be zero: " << constraints.number << endl; // Echo print number of constraints
    cout << "DOFs specified to be zero: ";

    outputFile << "Number of DOFs specified to be zero: " << constraints.number << endl;
    outputFile << "DOFs specified to be zero: ";

    int DOF_num;
    while (inputFile >> DOF_num) {
        constraints.zeroDOFs.push_back(DOF_num);
        cout << DOF_num << " "; // Echo print DOF number
        outputFile << DOF_num << " ";

        if (inputFile.peek() == '\n') {
            break;
        }
    }
    cout << endl << endl;
    outputFile << endl << endl;
}

void readLoads(ifstream& inputFile, ofstream& outputFile, vector<Load>& loads) {
    cout << "LOADS: " << endl;
    outputFile << "LOADS: " << endl;

    int numLoads;
    inputFile >> numLoads;

    cout << "Number of point loads: " << numLoads << endl; // Echo print number of loads
    outputFile << "Number of point loads: " << numLoads << endl;

    loads.resize(numLoads);
    for (int i = 0; i < numLoads; i++) {
        inputFile >> loads[i].dof >> loads[i].magnitude;

        // Echo print load DOFs and magnitudes
        if (loads[i].dof % 2 == 0) {
            cout << "DOF " << loads[i].dof << ": magnitude = " << loads[i].magnitude << " lb-in" << endl; // Moment
            outputFile << "DOF " << loads[i].dof << ": magnitude = " << loads[i].magnitude << " lb-in" << endl;
        } else {
            cout << "DOF " << loads[i].dof << ": magnitude = " << loads[i].magnitude << " lbf" << endl; // Force
            outputFile << "DOF " << loads[i].dof << ": magnitude = " << loads[i].magnitude << " lbf" << endl;
        }
    }
}

/////////////////////////////////////////////////////////////Part 2a////////////////////////////////////////////////////////////////////////
void assembleGlobalStiffnessMatrix() {}

void imposeConstraints() {}

/////////////////////////////////////////////////////////////Part 2b////////////////////////////////////////////////////////////////////////
void solver() {}

void reportResults() {}

#endif
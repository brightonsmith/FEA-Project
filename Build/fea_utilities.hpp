#ifndef FEA_UTILITIES_HPP
#define FEA_UTILITIES_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <utility>

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
    int length;
    pair<Node*, Node*> connectivity; // Use pair rather than vector since we assume two-node elements
};

struct Properties {
    double youngsModulus;
    double height; // in
    double width; // in
};

// No Constraint struct because constraints can be represented as a single vector 

struct Load {
    int dof;
    double magnitude; // lbf for force and lb-in for moment
};

// Prototypes
void readMesh(ifstream& inputFile, ofstream& outputFile, vector<Node>& nodes, vector<Element>& elements); // Reads nodal coordinates and element connectivity from input file
void readProperties(ifstream& inputFile, ofstream& outputFile, Properties& properties); // Reads beam properties from input file
void readConstraints(ifstream& inputFile, ofstream& outputFile, vector<int> constraints); // Reads list of DOFs specified to be zero from input file
void readLoads(ifstream& inputFile, ofstream& outputFile, vector<Load>& loads); // Reads load magnitudes and corresponding DOFs
vector<int> getElementDOFs(Element& element); // Get the assembly vector for the given element
void getElementK(Element& element); // Assemble the element stiffness matrix K_local
void assembleGlobalStiffnessMatrix(); // Assemble the global stiffness matrix K_global
void imposeConstraints(); // Use penalty method to impose kinematic BC's
void solver(); // Solve linear algebraic equations
void reportResults(); // Output to console and output file

///////////////////////////////////////////////////////////////////////////Part 1/////////////////////////////////////////////////////////////////////////////////////////////// 
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

        int leftNodeNum, rightNodeNum;
        inputFile >> leftNodeNum >> rightNodeNum;

        // This block assigns the nodal pair contained in the current element to pointers of actual nodes
        Node* leftNode = nullptr;
        Node* rightNode = nullptr;
        for (int i = 0; i < numNodes; i++) { // Use size_t when using size() method in loop
            if (nodes[i].number == leftNodeNum)
                leftNode = &nodes[i];
            if (nodes[i].number == rightNodeNum)
                rightNode = &nodes[i];
        }
        elements[i].connectivity = make_pair(leftNode, rightNode);

        cout << leftNode->number << " " << rightNode->number << endl; // Echo print left and right node numbers
        outputFile << leftNode->number << " " << rightNode->number << endl;

        if (leftNode && rightNode) {
            elements[i].length = rightNode->x - leftNode->x;
        }
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

void readConstraints(ifstream& inputFile, ofstream& outputFile, vector<int> constraints) {
    cout << "CONSTRAINTS: " << endl;
    outputFile << "CONSTRAINTS: " << endl;

    int numConstraints;
    inputFile >> numConstraints;

    cout << "Number of DOFs specified to be zero: " << numConstraints << endl; // Echo print number of constraints
    cout << "DOFs specified to be zero: ";

    outputFile << "Number of DOFs specified to be zero: " << numConstraints << endl;
    outputFile << "DOFs specified to be zero: ";


    constraints.resize(numConstraints);
    for (int i = 0; i < numConstraints; i++) {
        inputFile >> constraints[i];
        cout << constraints[i] << " "; // Echo print constraint number
        outputFile << constraints[i] << " ";
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

///////////////////////////////////////////////////////////////////////////Part 2a///////////////////////////////////////////////////////////////////////////////////////////////
vector<int> getElementDOFs(Element& element) {
    Node* leftNode = element.connectivity.first;
    Node* rightNode = element.connectivity.second;

    vector<int> assemblyVector = {leftNode->dof1, leftNode->dof2, rightNode->dof1, rightNode->dof2};

    return assemblyVector;
}

void getElementK(Element& element) {
    vector <int> assemblyVector = getElementDOFs(element);
}

void assembleGlobalStiffnessMatrix() {}

void imposeConstraints() {}

///////////////////////////////////////////////////////////////////////////Part 2b///////////////////////////////////////////////////////////////////////////////////////////////
void solver() {}

void reportResults() {}

#endif
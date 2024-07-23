#ifndef FEA_UTILITIES_HPP
#define FEA_UTILITIES_HPP

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense> // Used for matrix instantiations and linear algebraic operations (dependencies included in project directory)

using namespace std;
using namespace Eigen; 
typedef std::vector<std::vector<std::pair<int, int>>> ImageMatrix; // Custom type for image matrix (see ElementStiffnessData)

struct Node {
    int number; 
    double x; // Global coordinate (from left)
    int dof1; 
    int dof2;
    pair<double, double> displacements;
    
    Node();
    Node(int num);
};

Node::Node() : number(0), x(0.0), dof1(0), dof2(0) , displacements(make_pair(0.0, 0.0)) {} // Default constructor

Node::Node(int num) : number(num), x(0.0), dof1((num - 1)*2 + 1), dof2((num - 1)*2 + 2), displacements(make_pair(0.0, 0.0)) {} // DOF numberings are predetermined for each node

struct Element {
    int number; 
    double length;
    pair<Node*, Node*> connectivity; // Use pair rather than vector since we assume two-node elements
};

struct Properties {
    double youngsModulus;
    double height; // in
    double width; // in
};

// No Constraint struct because constraints can be represented as a simple vector 

struct Load {
    int dof;
    double magnitude; // lbf for force and lb-in for moment
};

struct ElementStiffnessData {
    MatrixXd K_local;
    ImageMatrix image; // Corresponding indices for global assembly (based on assembly vector)
};

// Prototypes
void readMesh(ifstream& inputFile, ofstream& outputFile, vector<Node>& nodes, vector<Element>& elements); // Reads nodal coordinates and element connectivity from input file
void readProperties(ifstream& inputFile, ofstream& outputFile, Properties& properties); // Reads beam properties from input file
void readConstraints(ifstream& inputFile, ofstream& outputFile, vector<int>& constraints); // Reads list of DOFs specified to be zero from input file
void readLoads(ifstream& inputFile, ofstream& outputFile, vector<Load>& loads); // Reads load magnitudes and corresponding DOFs
vector<int> getElementDOFs(const Element& element); // Get the assembly vector for the given element
ElementStiffnessData getElementK(ofstream& outputFile, const Element& element, const Properties& properties); // Assemble the element stiffness matrix K_local
ImageMatrix getImageMatrix(const vector<int>& assemblyVector); // Helper function for getElementK that returns a matrix of integer pairs that represent the global positions of each item in the element stiffness matrix K_local (a mapping mechanism)
void assembleGlobalStiffnessMatrix(ofstream& outputFile, const vector<ElementStiffnessData>& elementStiffnessDataVector); // Assemble the global stiffness matrix K_global
void imposeConstraints(ofstream& outputFile, MatrixXd& K_global, const vector<int>& constraints); // Use penalty method to impose kinematic BC's
void outputGlobalStiffnessMatrix(ofstream& outputFile, const MatrixXd& K_global); // Helper function to print K_global to console and output file
void assembleGlobalLoadVector(ofstream& outputFile, VectorXd& F_global, const vector<Load>& loads); // Assemble the global load vector F_global
void solver(const MatrixXd& K_global, const VectorXd& F_global, vector<Node>& nodes); // Solve linear algebraic equations
void reportResults(ofstream& outputFile, const vector<Node>& nodes); // Output to console and output file

///////////////////////////////////////////////////////////////////////////Part 1/////////////////////////////////////////////////////////////////////////////////////////////// 
void readMesh(ifstream& inputFile, ofstream& outputFile, vector<Node>& nodes, vector<Element>& elements) {
    cout << "MESH:" << endl;
    outputFile << "MESH:" << endl;

    int numNodes, numElements;
    inputFile >> numNodes >> numElements;

    cout << "Number of nodes: " << numNodes << endl;
    cout << "Number of elements: " << numElements << endl;

    outputFile << "Number of nodes: " << numNodes << endl;
    outputFile << "Number of elements: " << numElements << endl;

    // Build nodes
    nodes.resize(numNodes);
    for (int i = 0; i < numNodes; i++) {
        int nodeNumber;
        double x;
        inputFile >> nodeNumber >> x;
        nodes[i] = Node(nodeNumber);
        nodes[i].x = x;

        cout << "Node " << nodes[i].number << ": x = " << nodes[i].x << endl; 
        outputFile << "Node " << nodes[i].number << ": x = " << nodes[i].x << endl; 
    }

    // Build elements
    elements.resize(numElements);
    for (int i = 0; i < numElements; i++) {
        inputFile >> elements[i].number;
        cout << "Element " << elements[i].number << " connectivity: "; 
        outputFile << "Element " << elements[i].number << " connectivity: "; 

        int leftNodeNum, rightNodeNum;
        inputFile >> leftNodeNum >> rightNodeNum;

        // This block assigns the nodal pair contained in the current element to pointers of actual nodes
        Node* leftNode = nullptr;
        Node* rightNode = nullptr;
        for (int i = 0; i < numNodes; i++) {
            if (nodes[i].number == leftNodeNum)
                leftNode = &nodes[i];
            if (nodes[i].number == rightNodeNum)
                rightNode = &nodes[i];
        }
        elements[i].connectivity = make_pair(leftNode, rightNode);

        cout << leftNode->number << " " << rightNode->number << endl; 
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

    // Build properties
    inputFile >> properties.youngsModulus >> properties.height >> properties.width;

    cout << "Young's Modulus: " << properties.youngsModulus << endl;
    cout << "Height: " << properties.height << " in" << endl;
    cout << "Width: " << properties.width << " in" << endl << endl;

    outputFile << "Young's Modulus: " << properties.youngsModulus << endl;
    outputFile << "Height: " << properties.height << " in" << endl;
    outputFile << "Width: " << properties.width << " in" << endl << endl;
}

void readConstraints(ifstream& inputFile, ofstream& outputFile, vector<int>& constraints) {
    cout << "CONSTRAINTS: " << endl;
    outputFile << "CONSTRAINTS: " << endl;

    int numConstraints;
    inputFile >> numConstraints;

    cout << "Number of DOFs specified to be zero: " << numConstraints << endl; 
    cout << "DOFs specified to be zero: ";

    outputFile << "Number of DOFs specified to be zero: " << numConstraints << endl; 
    outputFile << "DOFs specified to be zero: ";

    // Build constraints
    constraints.resize(numConstraints);
    for (int i = 0; i < numConstraints; i++) {
        inputFile >> constraints[i];
        cout << constraints[i] << " "; 
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

    cout << "Number of point loads: " << numLoads << endl; 
    outputFile << "Number of point loads: " << numLoads << endl;

    // Build loads
    loads.resize(numLoads);
    for (int i = 0; i < numLoads; i++) {
        inputFile >> loads[i].dof >> loads[i].magnitude;

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
vector<int> getElementDOFs(const Element& element) {
    Node* leftNode = element.connectivity.first;
    Node* rightNode = element.connectivity.second;

    vector<int> assemblyVector = {leftNode->dof1, leftNode->dof2, rightNode->dof1, rightNode->dof2}; // Global DOF numberings for K_local (to keep track when K_global is built)

    return assemblyVector;
}

ElementStiffnessData getElementK(ofstream& outputFile, const Element& element, const Properties& properties) {
    vector <int> assemblyVector = getElementDOFs(element);
    double L = element.length;
    double L_2 = pow(L, 2);
    double L_3 = pow(L, 3);
    double E = properties.youngsModulus;
    double I_z = (properties.width * pow(properties.height, 3))/12;

    // Assuming cubic interpolation functions, K_local is pre-determined
    MatrixXd K_local(4, 4);
    K_local << 12/L_3, 6/L_2, -12/L_3, 6/L_2,
            6/L_2, 4/L, -6/L_2, 2/L,
            -12/L_3, -6/L_2, 12/L_3, -6/L_2,
            6/L_2, 2/L, -6/L_2, 4/L;

    for (size_t i = 0; i < assemblyVector.size(); i++) {
        for (size_t j = 0; j < assemblyVector.size(); j++) {
            K_local(i, j) *= (E*I_z);
        }
    }

    ImageMatrix image = getImageMatrix(assemblyVector);

    // Package data
    ElementStiffnessData result;
    result.K_local = K_local;
    result.image = image;

    // Pretty printing
    cout << setw(0) << "";
    for (int dof : assemblyVector) {
        cout << setw(13) << dof;
    }
    cout << endl;
    cout << string((assemblyVector.size())*14, '-') << endl;

    for (size_t i = 0; i < assemblyVector.size(); i++) {
        cout << setw(2) << assemblyVector[i] << " | ";
        for (size_t j = 0; j < assemblyVector.size(); j++) {
            cout << scientific << setw(12) << setprecision(4) << K_local(i, j) << " ";
        }
        cout << endl;
    }

    outputFile << setw(0) << "";
    for (int dof : assemblyVector) {
        outputFile << setw(13) << dof;
    }
    outputFile << endl;
    outputFile << string((assemblyVector.size())*14, '-') << endl;

    for (size_t i = 0; i < assemblyVector.size(); i++) {
        outputFile << setw(2) << assemblyVector[i] << " | ";
        for (size_t j = 0; j < assemblyVector.size(); j++) {
            outputFile << scientific << setw(12) << setprecision(4) << K_local(i, j) << " ";
        }
        outputFile << endl;
    }

    return result;
}

ImageMatrix getImageMatrix(const vector<int>& assemblyVector) {
    ImageMatrix image(4, vector<pair<int, int>>(4, make_pair(0 ,0)));

    for (size_t i = 0; i < assemblyVector.size(); i++) {
        for (size_t j = 0; j < assemblyVector.size(); j++) {
            image[i][j] = make_pair(assemblyVector[i] - 1, assemblyVector[j] - 1); // Each pair corresponds to the coordinates in K_global
        }
    }
    return image;
}

void assembleGlobalStiffnessMatrix(ofstream& outputFile, MatrixXd& K_global, const vector<ElementStiffnessData>& elementStiffnessDataVector) {
    for (size_t i = 0; i < elementStiffnessDataVector.size(); i++) {
        MatrixXd K_local = elementStiffnessDataVector[i].K_local;
        ImageMatrix image = elementStiffnessDataVector[i].image; // image is used to map K_local entries to their proper places in K_global
        for (int j = 0; j < K_local.rows(); j++) {
            for (int k = 0; k < K_local.cols(); k++) {
                int row = image[j][k].first;
                int col = image[j][k].second;
                K_global(row, col) += K_local(j, k);
            }
        }
    }

    cout << endl;
    cout << "Global stiffness matrix:" << endl;

    outputFile << endl;
    outputFile << "Global stiffness matrix:" << endl;
    
    outputGlobalStiffnessMatrix(outputFile, K_global);
}

void imposeConstraints(ofstream& outputFile, MatrixXd& K_global, const vector<int>& constraints) {
    double penalty = 1e30; // pre-determined
    for (size_t i = 0; i < constraints.size(); i++) {
        int index = constraints[i] - 1;
        K_global(index, index) += penalty; // This has the same effect as removing a row and column for a zero DOF
    }

    cout << endl;
    cout << "Global stiffness matrix with constraints imposed:" << endl;

    outputFile << endl;
    outputFile << "Global stiffness matrix with constraints imposed:" << endl;

    outputGlobalStiffnessMatrix(outputFile, K_global);
}

void outputGlobalStiffnessMatrix(ofstream& outputFile, const MatrixXd& K_global) {
    // Pretty printing
    cout << setw(0) << "";
    for (int i = 1; i <= K_global.cols(); i++) {
        cout << setw(13) << i;
    }
    cout << endl;
    cout << string((K_global.cols())*14, '-') << endl;

    for (int i = 0; i < K_global.rows(); i++) {
        cout << setw(2) << i + 1 << " | ";
        for (int j = 0; j < K_global.cols(); j++) {
            cout << scientific << setw(12) << setprecision(4) << K_global(i, j) << " ";
        }
        cout << endl;
    }

    outputFile << setw(0) << "";
    for (int i = 1; i <= K_global.cols(); i++) {
        outputFile << setw(13) << i;
    }
    outputFile << endl;
    outputFile << string((K_global.cols())*14, '-') << endl;

    for (int i = 0; i < K_global.rows(); i++) {
        outputFile << setw(2) << i + 1 << " | ";
        for (int j = 0; j < K_global.cols(); j++) {
            outputFile << scientific << setw(12) << setprecision(4) << K_global(i, j) << " ";
        }
        outputFile << endl;
    }
}

void assembleGlobalLoadVector(ofstream& outputFile, VectorXd& F_global, const vector<Load>& loads) {
    for (size_t i = 0; i < loads.size(); i++) {
        int dof = loads[i].dof;
        double magnitude = loads[i].magnitude;
        F_global(dof - 1) += magnitude; // Assume loads at any dof are zero if they are not specified (this is not necessarily true, but the penalty method will overrule whatever actual load there may be)
    }

    // Pretty printing
    cout << endl;
    cout << "Global load vector:" << endl;
    for (int i = 0; i < F_global.size(); i++) {
        cout << setw(2) << i + 1 << " | ";
        cout << scientific << setw(12) << setprecision(4) << F_global(i) << " " << endl;
    }

    outputFile << endl;
    outputFile << "Global load vector:" << endl;
    for (int i = 0; i < F_global.size(); i++) {
        outputFile << setw(2) << i + 1 << " | ";
        outputFile << scientific << setw(12) << setprecision(4) << F_global(i) << " " << endl;
    }
}

///////////////////////////////////////////////////////////////////////////Part 2b///////////////////////////////////////////////////////////////////////////////////////////////
void solver(const MatrixXd& K_global, const VectorXd& F_global, vector<Node>& nodes) {
    MatrixXd K_global_inverse = K_global.inverse(); // The only reason to use Eigen/Dense haha
    VectorXd displacements = K_global_inverse*F_global;

    for (int i = 0; i < displacements.size(); i += 2) {
        nodes[i/2].displacements = make_pair(displacements[i], displacements[i+1]); // Assign displacements to each dof at each node
    }
}

void reportResults(ofstream& outputFile, const vector<Node>& nodes) {
    // Pretty printing
    cout << endl;
    cout << "Solving Kq = F..." << endl;
    cout << endl;
    cout << "Nodal Displacements:" << endl;
    for (size_t i = 0; i < nodes.size(); i++) {
        cout << setw(2) << i + 1 << " | ";
        cout << scientific << setw(12) << setprecision(4) << nodes[i].displacements.first << " ";
        cout << scientific << setw(12) << setprecision(4) << nodes[i].displacements.second << endl;
    }

    outputFile << endl;
    outputFile << "Solving Kq = F..." << endl;
    outputFile << endl;
    outputFile << "Nodal Displacements:" << endl;
    for (size_t i = 0; i < nodes.size(); i++) {
        outputFile << setw(2) << i + 1 << " | ";
        outputFile << scientific << setw(12) << setprecision(4) << nodes[i].displacements.first << " ";
        outputFile << scientific << setw(12) << setprecision(4) << nodes[i].displacements.second << endl;
    }
}

#endif
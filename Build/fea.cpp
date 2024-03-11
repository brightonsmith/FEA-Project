#include "fea_utilities.hpp"

int main() {
    ifstream inputFile("inputFile.txt");

    if (!inputFile.is_open()) {
        cerr << "Error: Unable to open inputFile.txt for reading." << endl;
        return 1;
    }

    ofstream outputFile("outputFile.txt");

    if (!outputFile.is_open()) {
        cerr << "Error: Unable to open outputFile.txt for writing." << endl;
        return 1;
    }

    vector<Node> nodes;
    vector<Element> elements;
    vector<Load> loads;
    vector<int> constraints;
    Properties properties;

    readMesh(inputFile, outputFile, nodes, elements);
    readProperties(inputFile, outputFile, properties);
    readConstraints(inputFile, outputFile, constraints);
    readLoads(inputFile, outputFile, loads);

    inputFile.close();

    vector<ElementStiffnessData> elementStiffnessDataVector;

    for (size_t i = 0; i < elements.size(); i++) {
        cout << endl;
        outputFile << endl;
        cout << "Element " << i + 1 << " stiffness matrix: " << endl;
        outputFile << "Element " << i + 1 << " stiffness matrix: " << endl;
        ElementStiffnessData elementStiffnessData = getElementK(outputFile, elements[i], properties);
        elementStiffnessDataVector.push_back(elementStiffnessData);
    }
    outputFile.close();
    
    return 0;
}
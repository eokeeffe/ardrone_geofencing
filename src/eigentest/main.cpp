#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

int main()
{
    vector<Triplet<double> > trips(4);
    trips.push_back(Triplet<double>(0, 0, .435));
    trips.push_back(Triplet<double>(1, 1, .435));
    trips.push_back(Triplet<double>(2, 2, .435));
    trips.push_back(Triplet<double>(3, 3, .435));

    SparseMatrix<double> A;
    A.resize(4,4);
    A.setFromTriplets(trips.begin(), trips.end());
    SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solverA;
    solverA.compute(A);

    VectorXd B;
    B.resize(4);
    B(0) = .435;
    B(1) = .435;
    B(2) = .435;
    B(3) = .435;

    double t = B(0);
    cout << "Worked:" << t << endl;

    VectorXd X = solverA.solve(B);

    for (int i = 0; i < 4; i++)
        cout << X(i) << endl;
}

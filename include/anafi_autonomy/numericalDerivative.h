#include <iostream>
#include <cfloat>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// https://web.media.mit.edu/~crtaylor/calculator.html
# define FINITE_DIFFERENCE_COEFFICIENTS_2POINTS -1.0, 1.0
# define FINITE_DIFFERENCE_COEFFICIENTS_3POINTS 1.0/2, -4.0/2, 3.0/2
# define FINITE_DIFFERENCE_COEFFICIENTS_4POINTS -2.0/6, 9.0/6, -18.0/6, 11.0/6
# define FINITE_DIFFERENCE_COEFFICIENTS_5POINTS 3.0/12, -16.0/12, 36.0/12, -48.0/12, 25.0/12

class NumericalDerivative{
	public:
		// Constructor
		NumericalDerivative(int s, int d){
			size = s;
			dimension = d;
		
			measurements = MatrixXd::Zero(size, dimension);
			d_measurements = MatrixXd::Zero(size, dimension);
			d_ts = DBL_MAX * VectorXd::Ones(size);

			finiteDifferenceCoefficients.row(0) << 0, 0, 0, FINITE_DIFFERENCE_COEFFICIENTS_2POINTS;
			finiteDifferenceCoefficients.row(1) << 0, 0, FINITE_DIFFERENCE_COEFFICIENTS_3POINTS;
			finiteDifferenceCoefficients.row(2) << 0, FINITE_DIFFERENCE_COEFFICIENTS_4POINTS;
			finiteDifferenceCoefficients.row(3) << FINITE_DIFFERENCE_COEFFICIENTS_5POINTS;
		}
		
		// Functions
		void updateMeasurements(Ref<VectorXd> measurement, double d_t){
			measurements.topRows(size - 1) = measurements.bottomRows(size - 1);
			measurements.row(size - 1) = measurement;

			d_measurements.topRows(size - 1) = d_measurements.bottomRows(size - 1);
			//d_measurements.row(size - 1) = (measurements.row(size - 1) - measurements.row(size - 2)) / d_t;
			d_measurements.row(size - 1) = 
				(finiteDifferenceCoefficients.row(size - 2).tail(size) * measurements).array() 
					/ d_ts.transpose().array();
			
			d_ts.topRows(size - 1) = d_ts.bottomRows(size - 1);
			d_ts(size - 1) = d_t;
		}

		VectorXd getDerivative(){
			/*VectorXd derivative = VectorXd(dimension);		
			derivative = 
				(finiteDifferenceCoefficients.row(size - 2).tail(size) * measurements).array() 
					/ d_ts.transpose().array();
			return derivative;*/
			return d_measurements.colwise().mean();
		}

	private:			
		// Variables
		int size;
		int dimension;
		MatrixXd measurements;
		MatrixXd d_measurements;
		VectorXd d_ts;
		
		// Constant
		MatrixXd finiteDifferenceCoefficients = MatrixXd::Zero(5, 5);
};

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
			d_times = DBL_MAX * VectorXd::Ones(size);
			time_old = 0;

			finiteDifferenceCoefficients.row(0) << 0, 0, 0, FINITE_DIFFERENCE_COEFFICIENTS_2POINTS;
			finiteDifferenceCoefficients.row(1) << 0, 0, FINITE_DIFFERENCE_COEFFICIENTS_3POINTS;
			finiteDifferenceCoefficients.row(2) << 0, FINITE_DIFFERENCE_COEFFICIENTS_4POINTS;
			finiteDifferenceCoefficients.row(3) << FINITE_DIFFERENCE_COEFFICIENTS_5POINTS;
		}
		
		// Functions
		void updateMeasurements(Ref<VectorXd> m, double t){
			measurements.topRows(size - 1) = measurements.bottomRows(size - 1);
			measurements.row(size - 1) = m;
			
			d_times.topRows(size - 1) = d_times.bottomRows(size - 1);
			d_times(size - 1) = t - time_old;
			time_old = t;

			d_measurements.topRows(size - 1) = d_measurements.bottomRows(size - 1);
			d_measurements.row(size - 1) = 
				finiteDifferenceCoefficients.row(size - 2).tail(size) * measurements / d_times.mean();
		}

		double getTime(){
			return time_old;
		}

		VectorXd getDerivative(){
			return d_measurements.colwise().mean();
		}

		VectorXd getSecondDerivative(){
			return VectorXd::Zero(3);
		}

		VectorXd getUnfiltertedDerivative(){
			return (measurements.row(size - 1) - measurements.row(size - 2))/d_times(size - 1);
		}

		VectorXd getUnfiltertedSecondDerivative(){
			return ((measurements.row(size - 1) - measurements.row(size - 2))/d_times(size - 1) - 
				(measurements.row(size - 2) - measurements.row(size - 3))/d_times(size - 2))/d_times(size - 1);
		}

	private:			
		// Variables
		int size;
		int dimension;
		MatrixXd measurements;
		MatrixXd d_measurements;
		VectorXd d_times;
		double time_old;
		
		// Constant
		MatrixXd finiteDifferenceCoefficients = MatrixXd::Zero(5, 5);
};

class NumericalDerivativeQuaternion{
	public:
		// Constructor
		NumericalDerivativeQuaternion(){}
		
		// Functions
		void updateQuaternion(Quaterniond q, double t){
			quaternion_old_old = quaternion_old;
			quaternion_old = quaternion_new;
			quaternion_new = q;
			
			time_old_old = time_old;
			time_old = time_new;
			time_new = t;
		}

		double getTime(){
			return time_new;
		}

		Vector3d getAngularVelocity(){
			double dt = time_new - time_old;

			Vector4d q1(quaternion_old.w(), quaternion_old.x(), quaternion_old.y(), quaternion_old.z());
			Vector4d q2(quaternion_new.w(), quaternion_new.x(), quaternion_new.y(), quaternion_new.z());

			Vector3d angular_velocity;
			// https://mariogc.com/post/angular-velocity-quaternions/
        	angular_velocity(0) = 2.0/dt*(q1(0)*q2(1) - q1(1)*q2(0) - q1(2)*q2(3) + q1(3)*q2(2));
			angular_velocity(1) = 2.0/dt*(q1(0)*q2(2) + q1(1)*q2(3) - q1(2)*q2(0) - q1(3)*q2(1));
			angular_velocity(2) = 2.0/dt*(q1(0)*q2(3) - q1(1)*q2(2) + q1(2)*q2(1) - q1(3)*q2(0));

			return angular_velocity;
		}

		Vector3d getAngularAcceleration(){
			double dt_old = time_old - time_old_old;
			
			Vector4d q1(quaternion_old_old.w(), quaternion_old_old.x(), quaternion_old_old.y(), quaternion_old_old.z());
			Vector4d q2(quaternion_old.w(), quaternion_old.x(), quaternion_old.y(), quaternion_old.z());

			Vector3d angular_velocity_old;
			// https://mariogc.com/post/angular-velocity-quaternions/
        	angular_velocity_old(0) = 2.0/dt_old*(q1(0)*q2(1) - q1(1)*q2(0) - q1(2)*q2(3) + q1(3)*q2(2));
			angular_velocity_old(1) = 2.0/dt_old*(q1(0)*q2(2) + q1(1)*q2(3) - q1(2)*q2(0) - q1(3)*q2(1));
			angular_velocity_old(2) = 2.0/dt_old*(q1(0)*q2(3) - q1(1)*q2(2) + q1(2)*q2(1) - q1(3)*q2(0));

			Vector3d angular_velocity_new = getAngularVelocity();

			return (angular_velocity_new - angular_velocity_old)/(time_new - time_old);
		}

	private:			
		// Variables
		Quaterniond quaternion_old_old = Quaterniond(1, 0, 0, 0);
		Quaterniond quaternion_old = Quaterniond(1, 0, 0, 0);
		Quaterniond quaternion_new = Quaterniond(1, 0, 0, 0);
		double time_old_old = 0;
		double time_old = DBL_MAX/2;
		double time_new = DBL_MAX;
};

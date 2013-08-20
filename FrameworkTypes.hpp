#ifndef FRAMEWORK_TYPES_H
#define FRAMEWORK_TYPES_H

#include <vector>
#include <base/time.h>
#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>

namespace rover_localization
{

    /********************************/
    /** Data struct for properties **/
    /********************************/


    //Data type to know the location
    struct LocationConfiguration
    {
        double latitude;//Latitude in radians
        double longitude;//Longitude in radians
        double altitude;//Altitude in meters
        double magnetic_declination;//Declination in radians
        int magnetic_declination_mode;//The declination is positive when the magnetic north is east of true north
                                    //1 is EAST, which means positive declination. 2 is WEST, which means negative declination.
        double dip_angle;//Dip angle in radians
    };

    //Data type for the Proprioceptive sensor characteristic
    struct ProprioceptiveSensorProperties
    {
        /********************************/
        /** Inertial Sensor Properties **/
        /********************************/

        double bandwidth; //Inertial sensors bandwidth in Hertz.
        //This is characteristic of the sensor and should be equal
        //or smaller than the sampling rate.

        /** Gyroscope Noise **/
        base::Vector3d gbiasoff;//bias offset in static regimen for the Gyroscopes
        base::Vector3d gyrorw;//angle random walk for gyroscopes (rad/sqrt(s))
        base::Vector3d gyrorrw;//rate random walk for gyroscopes (rad/s/sqrt(s))
        base::Vector3d gbiasins; //gyros bias instability (rad/s)

        /** Accelerometers Noise**/
        base::Vector3d abiasoff;//bias offset in static regimen for the Accelerometers
        base::Vector3d accrw;//velocity random walk for accelerometers (m/s/sqrt(s))
        base::Vector3d accrrw;//acceleration random walk for accelerometers (m/s^2/sqrt(s))
        base::Vector3d abiasins;//acc bias instability (m/s^2)

        /** Magnetometers Noise**/
        base::Vector3d magrw; //random walk for magnetometers"

        /************************************/
        /** Robot Joint Encoder Properties **/
        /************************************/

        /** Encoder Noise **/
        std::vector<double> encodersrw;//encoders position uncertainty (modeled as zero mean white noise pdf)
    };

    /** Framework Configuration **/
    struct FrameworkConfiguration
    {
        double frontend_frequency;//Desired frequency in Hertz to run the Front-End.
                                //It cannot be higher that the sensor values of the aggregator (transformer).

        double backend_frequency;//Desired frequency in Hertz to trigger the BackEnd part of the framework.
                                //It cannot be higher that frontend_frequency.

        double visualization_frequency;//Desired frequency for the visualization part of the framework.
                                //It cannot be higher that frontend_frequency.

        bool use_inclinometers_leveling;//Some IMU provide inclinometers as fast and more accurate solution for initial leveling.
                                //Set True or False to use inclinometers values or not.
                                //Note: Check if the IMU inport has inclinometers information (right now coded in mag).

        double init_leveling_time;//Time to compute the initial leveling of the robot in order to find the gravity vector.
    };

    /** Adaptive Measurement Configuration. Variables for the attitude estimation inside the algorithm **/
    struct AdaptiveMeasurementProperties
    {
        unsigned int M1; /** Parameter for adaptive algorithm (to estimate Uk with is not directly observale) */
        unsigned int M2; /** Parameter for adaptive algorithm (to prevent falsering entering in no-external acc mode) */
        double gamma; /** Parameter for adaptive algorithm. Only entering when Qstart (adaptive cov. matrix) is greater than RHR'+Ra */
        unsigned int r2count; /** Parameter for adaptive algorithm */

        void reset()
        {
            M1 = 0;
            M2 = 0;
            gamma = 0.0;
            r2count = 0;
            return;
        }

    };

    /**************************************/
    /** Data struct for (internal) ports **/
    /**************************************/

    //Data Type about robot kinematics chain status including contact points
    struct RobotContactPoints
    {
        base::Time time;//timestamp
        std::vector< double > modelPositions;//Robot joints and model positions values
        std::vector< int > contactPoints; //Points index in contact per each robot Tree
        std::vector< base::Matrix4d > chain;//Transformation matrix of the points wrt body
        std::vector< base::Matrix6d > cov;//Covariance matrix of the pose wrt body
    };

    //BackEnd optimization problem information
    struct BackEndEstimation
    {
	
	base::Time time;
	base::VectorXd statek_i;
	base::VectorXd estatek_i;
	base::MatrixXd Pki;
	base::MatrixXd K;
	base::MatrixXd Qk;
	base::MatrixXd Rk;
	base::VectorXd innovation;
	base::Matrix3d Hellinger;
        base::Vector3d abias;
        base::Vector3d gbias;
    };

    //Local gravity information (theory and computed at initialization time)
    struct InertialState
    {
        /** Time stamp */
        base::Time time;

         /** Theoretical gravity value computed from model */
        double theoretical_g;

        /** Experimental gravity value computed at init time (no-moving robot) */
        double estimated_g;

        /** Increment in orientation (raw gyros integration) **/
        base::Quaterniond delta_orientation;

        /** Increment in velocity (bounded integration) */
        base::Vector3d delta_vel;

        /** Corrected accelerometer readings */
        base::Vector3d acc;

        /** Corrected inclinometer gyro reading*/
        base::Vector3d gyro;

        /** Raw inclinometer reading*/
        base::Vector3d incl;

        /** On/Off initial acc bias */
        base::Vector3d abias_onoff;

        /** On/Off initial gyro bias */
        base::Vector3d gbias_onoff;

    };

    /***********************************/
    /** Data struct for visualization **/
    /***********************************/

    //To visualize the chain forward kinematics(end effector or contact point)
    struct RobotContactPointsRbs
    {
        base::Time time; //timestamp
        std::vector< base::samples::RigidBodyState > rbsChain; //Rbs with the orientation and position of the contact point
    };

}
#endif

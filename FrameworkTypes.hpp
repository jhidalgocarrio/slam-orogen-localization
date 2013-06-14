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
        base::Vector3d gbiasof;//bias offset in static regimen for the Gyroscopes
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
        std::vector<double> encodersrw;//encoders velocity white noise
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

    /**************************************/
    /** Data struct for (internal) ports **/
    /**************************************/

    //Data Type to send to the Visualization component
    struct RobotContactPoints
    {
        base::Time time;//timestamp
        std::vector<base::Matrix4d> points;//Transformation matrix of the points wrt body
        std::vector<base::Matrix6d> cov;//Covariance matrix of the pose wrt body
    };

}
#endif

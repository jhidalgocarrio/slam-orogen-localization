#ifndef FRONT_END_TYPES_H
#define FRONT_END_TYPES_H

#include <vector>
#include <base/time.h>
#include <base/eigen.h>

namespace rover_localization
{

    /****************/
    /** Properties **/
    /****************/

     /** Processing Configuration **/
    struct Configuration
    {
        double output_frequency;//It cannot be higher that the sensor values of the aggregator (transformer).

        bool use_inclinometers_as_theoretical_gravity;//Inclinometers are more stable than accelerometers at initial time.
                                                    //They cloud be use as theoretical local gravity value instead of using
                                                    //some models as WGS-84 ellipsoid Earth.
        bool use_inclinometers_leveling;//Some IMU provide inclinometers as fast and more accurate solution for initial leveling.
                                //Set True or False to use inclinometers values or not.
                                //Note: Check if the IMU inport has inclinometers information (right now coded in mag).

        double init_leveling_time;//Time to compute the initial leveling of the robot in order to find the gravity vector.

        std::vector<std::string> jointsNames; //complete vector of names for the joints of the complete robot (passive and active).
    };

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

    /** Configuration parameters for the synchronization of a sweeping camera **/
    struct CameraSynchConfiguration
    {
        bool synchOn; /** True if active for camera synchronization **/
        bool zeroMark; /** True if sweeping unit passed the zero mark (set false by default) **/
        base::Quaterniond body2lcamera; /** Desired orientation of the left camera w.r.t the body **/
        base::Quaterniond quatError; /** Orientation error in the body2lcamera orientation**/
    };

    /**************************************/
    /** Data struct for (internal) ports **/
    /**************************************/

    //Inertial sensor and extra information
    struct InertialState
    {
        /** Time stamp */
        base::Time time;

         /** Theoretical gravity value computed from model */
        double theoretical_g;

        /** Experimental gravity value computed at initialization time (no-moving robot) */
        double estimated_g;

        /** On/Off initial accelerometers bias */
        base::Vector3d abias_onoff;

        /** On/Off initial gyro bias */
        base::Vector3d gbias_onoff;

    };

}
#endif

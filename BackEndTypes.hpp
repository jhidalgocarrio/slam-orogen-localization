#ifndef BACK_END_TYPES_H
#define BACK_END_TYPES_H

#include <base/time.h>
#include <base/eigen.h>

namespace rover_localization
{
    //Data type for the Inertial measurement characteristic
    struct InertialNoiseParameters
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

        /** Accelerometers Noise **/
        base::Vector3d abiasoff;//bias offset in static regimen for the Accelerometers
        base::Vector3d accrw;//velocity random walk for accelerometers (m/s/sqrt(s))
        base::Vector3d accrrw;//acceleration random walk for accelerometers (m/s^2/sqrt(s))
        base::Vector3d abiasins;//accelerometers bias instability (m/s^2)
        base::Vector3d accresolut;//minimum accelerometers resolution (m/s^2)

        /** Magnetometers Noise **/
        base::Vector3d magrw; //random walk for magnetometers

        /** Inclinometers Noise **/
        base::Vector3d incrw; //random walk for inclinometers (m/s/sqrt(s))
        base::Vector3d incresolut;//minimum accelerometers resolution (m/s^2)

    };

    /** Adaptive Measurement Configuration. Variables for the attitude estimation inside the algorithm **/
    struct AdaptiveAttitudeConfig
    {
        unsigned int M1; /** Parameter for adaptive algorithm (to estimate Uk with is not directly observable) */
        unsigned int M2; /** Parameter for adaptive algorithm (to prevent false entering in no-external acc mode) */
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
}
#endif



/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_STATE_OPTIMIZE_TASK_HPP
#define ROVER_LOCALIZATION_STATE_OPTIMIZE_TASK_HPP

#include "rover_localization/StateOptimizeBase.hpp"

/** Framework Library dependencies includes **/
#include <localization/Configuration.hpp> /** Constant values of the library */
#include <localization/core/DataModel.hpp> /** Simple Data Model with uncertainty */
#include <localization/tools/Util.hpp> /** Util class library **/
#include <localization/filters/Usckf.hpp> /** USCKF class with Manifolds */
#include <localization/filters/MtkWrap.hpp> /** USCKF wrapper for the state vector */
#include <localization/filters/State.hpp> /** Filters State */
#include <localization/filters/ProcessModels.hpp> /** Filters Process Models */
#include <localization/filters/MeasurementModels.hpp> /** Filters Measurement Models */

/** Eigen **/
#include <Eigen/Core> /** Core */
#include <Eigen/StdVector> /** For STL container with Eigen types **/
#include <Eigen/Geometry> /** For quaternion and angle-axis representations **/

/** Standard libs **/
#include <iostream>
#include <vector>

/** Boost **/
#include <boost/shared_ptr.hpp> /** For shared pointers **/
#include <boost/algorithm/string.hpp> /** case insensitive string comapre **/

namespace rover_localization {


    /** Wrap the Augmented and Single State **/
    typedef localization::MtkWrap<localization::State> WSingleState;
    typedef localization::MtkWrap<localization::AugmentedState> WAugmentedState;
    typedef localization::Usckf<WAugmentedState, WSingleState> StateOptimizeFilter;

    /*! \class StateOptimize
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Additional exteroceptive update samples to correct the prediction.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','rover_localization::StateOptimize')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class StateOptimize : public StateOptimizeBase
    {
	friend class StateOptimizeBase;

    protected:
        static const int  DEFAULT_CIRCULAR_BUFFER_SIZE = 2; /** Default number of objects to store regarding the inputs port **/

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/
        bool initFilter;
        bool absoluteFrame;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Delay state period **/
        int delayIterations;

        /** Inertial noise parameters **/
        InertialNoiseParameters inertialnoise;

        /** Adaptive Measurement Configuration **/
        AdaptiveAttitudeConfig adaptiveconfig;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** The filter uses by the Back-End **/
        boost::shared_ptr<StateOptimizeFilter> filter;

        /** Object of Class for Adaptive Measurement of Attitude Covariance Matrix **/
        boost::shared_ptr<localization::AdaptiveAttitudeCov> adapAtt;

        /** Variable in DataModel form for the differences in velocities **/
        localization::DataModel<double, 3> accModel, accInertial;

        /** SlipVector in Eigen class **/
        localization::DataModel <double, 3> slipVector;

        /** Delay delta pose from accelerometers integration **/
        Eigen::Vector3d deltaPosMeasurement;
        Eigen::Matrix3d deltaPosMeasurementCov;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Pose estimation **/
        ::base::samples::RigidBodyState deltaPoseSamples;

        /** Inertial values **/
        ::base::samples::IMUSensors inertialSamples;
        ::base::samples::IMUSensors meanInertialSamples;

        /** Inertial sensor state **/
        localization::InertialState inertialState;

        /***************************/
        /** Output port variables **/
        /***************************/

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

        virtual void inertial_stateTransformerCallback(const base::Time &ts, const ::localization::InertialState &inertial_state_sample);

        virtual void exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample);

    public:
        /** TaskContext constructor for Back-End
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        StateOptimize(std::string const& name = "rover_localization::StateOptimize");

        /** TaskContext constructor for StateOptimize 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        StateOptimize(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of StateOptimize
         */
	~StateOptimize();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /**@brief Initialize the filter used in the Back-End
         */
        void initStateOptimizeFilter(boost::shared_ptr<StateOptimizeFilter> &filter, localization::InertialState &inertialState, Eigen::Affine3d &tf);

        /**@brief Method for the delay update
         */
        inline void delayUpdate(const Eigen::Vector3d &measurement, const double &delta_t);

        /**@brief Method to perform the attitude update
         */
        inline void attitudeUpdate(const double &delta_t, const Eigen::Quaterniond &world2nav);

        /**@brief Calculate the delta of the state over the delta interval
         */
//      inline WSingleState deltaState (const double delta_t, const WSingleState &currentState, base::samples::RigidBodyState &pose,
//                                        base::samples::RigidBodyState &delta_pose, base::samples::IMUSensors &inertialSamples);

//        /**@brief Method to encapsulate the filter predict step
//         */
//        inline void statePredict(const double delta_t, const WSingleState &statek_i, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose, boost::circular_buffer<rover_localization::InertialState> &inertialState);
//
//        /**@brief Method to encapsulate the filter update (attitude and velocity)
//         */
//        inline void attitudeAndVelocityUpdate(const double delta_t, const WSingleState &statek_i,
//                                    boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//                                    boost::circular_buffer<rover_localization::InertialState> &inertialState);
//
//        /**@brief Calculates relative position
//         */
//        localization::DataModel<double, 3> relativePosition(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose);
//
//
//        /**@brief Calculates velocity error
//         */
//        localization::DataModel<double, 3> velocityError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
//                                                        const boost::shared_ptr< StateOptimizeFilter > filter);
//
//
//        /** \brief Store the variables in the Output ports
//         */
//        void outputPortSamples (const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter,
//                                const localization::DataModel<double, 3> &deltaVeloModel, const localization::DataModel<double, 3> &deltaVeloInertial);

        /** @brief Port out the values
        */
        void outputPortSamples(const base::Time &timestamp);

        /** @brief Port out the values
        */
        void outputDebugPortSamples(base::samples::IMUSensors &inertialSamples, const base::Time &timestamp);


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:

        inline Eigen::Quaterniond attitudeMeasurement (const Eigen::Quaterniond &orient, const Eigen::Vector3d &acc);

    };
}

#endif


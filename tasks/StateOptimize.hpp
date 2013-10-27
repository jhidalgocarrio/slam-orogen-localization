/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_STATE_OPTIMIZE_TASK_HPP
#define ROVER_LOCALIZATION_STATE_OPTIMIZE_TASK_HPP

#include "rover_localization/StateOptimizeBase.hpp"

/** Framework Library dependencies includes **/
#include <rover_localization/filters/Usckf.hpp> /** USCKF class with Manifolds */
#include <rover_localization/filters/MtkWrap.hpp> /** USCKF wrapper for the state vector */
#include <rover_localization/filters/State.hpp> /** Filters State */
#include <rover_localization/filters/ProcessModels.hpp> /** Filters Process Models */
#include <rover_localization/filters/MeasurementModels.hpp> /** Filters Measurement Models */
#include <rover_localization/Configuration.hpp> /** Constant values of the library */
#include <rover_localization/DataModel.hpp> /** Simple Data Model with uncertainty */
#include <rover_localization/Util.hpp> /** Helper Util class library **/

/** Eigen **/
#include <Eigen/Core> /** Core */
#include <Eigen/StdVector> /** For STL container with Eigen types **/
#include <Eigen/Geometry> /** For quaternion and angle-axis representations **/

/** Standard libs **/
#include <iostream>
#include <vector>

/** Boost **/
#include <boost/shared_ptr.hpp> /** For shared pointers **/

namespace rover_localization {


    /** Wrap the Augmented and Single State **/
    typedef localization::MtkWrap<localization::AugmentedState> WAugmentedState;
    typedef localization::MtkWrap<localization::State> WSingleState;
    typedef localization::Usckf<WAugmentedState, WSingleState> StateOptimizeFilter;

    /** Current counter of samples arrived to each port **/
    struct CounterInputPortsStateOptimize
    {
        void reset()
        {
            poseSamples = 0;
            inertialSamples = 0;
            inertialState = 0;
            return;
        }

	unsigned int poseSamples; /** counter of rover pose **/
 	unsigned int inertialSamples; /** counter of of inertial measurements **/
 	unsigned int inertialState; /** counter of inertial state **/
    };

    /** Number of samples to process in the callback function **/
    struct NumberInputPortsStateOptimize
    {
        void reset()
        {
            poseSamples = 0;
            inertialSamples = 0;
            inertialState = 0;
            return;
        }

	unsigned int poseSamples; /**  rover pose **/
 	unsigned int inertialSamples; /**  inertial measurements **/
 	unsigned int inertialState; /** inertial state **/
    };

    /** Input port samples arrived ON/OFF flags **/
    struct FlagInputPortsStateOptimize
    {
        void reset()
        {
            poseSamples = false;
            inertialSamples = false;
            inertialState = false;
            return;
        }

	unsigned int poseSamples;
        unsigned int inertialSamples;
 	unsigned int inertialState;
    };

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

	/** Filter Initialization **/
	bool initFilter;

        /** Number of samples to process in the input ports callback function **/
        NumberInputPortsStateOptimize number;

        /** Current counter of samples arrived to each input port **/
        CounterInputPortsStateOptimize counter;

        /** Data arrived ON/OFF Flag **/
        FlagInputPortsStateOptimize flag;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Inertial noise parameters **/
        InertialNoiseParameters inertialNoise;

        /** Framework configuration values **/
        StateOptimizeConfig config;

        /** Adaptive Measurement Configuration **/
        AdaptiveAttitudeConfig adaptiveConfig;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** The filter uses by the Back-End **/
        boost::shared_ptr<StateOptimizeFilter> filter;

        /** Object of Class for Adaptive Measurement of Attitude Covariance Matrix **/
        boost::shared_ptr<localization::AdaptiveAttitudeCov> adapAtt;

        /** Object of Class for Bumps in Z-Axis Velocity of Accelerometers Covariance Matrix **/
        boost::shared_ptr<localization::AdaptiveAttitudeCov> adapAcc;

        /** Variable in DataModel form for the differences in velocities **/
        localization::DataModel<double, 3> accModel, accInertial;

        /** SlipVector in Eigen class **/
        localization::DataModel <double, 3> slipVector;

        /**************************/
        /** Input port variables **/
        /**************************/

         /** Pose estimation from Front-End  **/
        boost::circular_buffer<::base::samples::RigidBodyState> poseSamples;

        /** Inertial values **/
        boost::circular_buffer<::base::samples::IMUSensors> inertialSamples;

        /** Inertial sensor state **/
        boost::circular_buffer<rover_localization::InertialState> inertialState;

        /***************************/
        /** Output port variables **/
        /***************************/

    protected:

        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

        virtual void inertial_stateTransformerCallback(const base::Time &ts, const ::rover_localization::InertialState &inertial_state_sample);

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

        /**@brief Get the values from the input port samples
         */
        void inputPortSamples(boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                            boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Initialize the filter used in the Back-End
         */
        void initStateOptimizeFilter(boost::shared_ptr<StateOptimizeFilter> &filter, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Calculate the delta of the state over the delta interval
         */
        inline WSingleState deltaState (const double delta_t, const WSingleState &currentState, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose, boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Method to encapsulate the filter predict step
         */
        inline void statePredict(const double delta_t, const WSingleState &statek_i, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose, boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Method to encapsulate the filter update (attitude and velocity)
         */
        inline void attitudeAndVelocityUpdate(const double delta_t, const WSingleState &statek_i,
                                    boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                                    boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Calculates relative position
         */
        localization::DataModel<double, 3> relativePosition(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose);


        /**@brief Calculates velocity error
         */
        localization::DataModel<double, 3> velocityError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                                                        const boost::shared_ptr< StateOptimizeFilter > filter);


        /** \brief Store the variables in the Output ports
         */
        void outputPortSamples (const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter,
                                const localization::DataModel<double, 3> &deltaVeloModel, const localization::DataModel<double, 3> &deltaVeloInertial);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif


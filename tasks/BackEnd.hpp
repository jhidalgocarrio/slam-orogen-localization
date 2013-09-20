/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_BACKEND_TASK_HPP
#define ROVER_LOCALIZATION_BACKEND_TASK_HPP

#include "rover_localization/BackEndBase.hpp"

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
    typedef localization::Usckf<WAugmentedState, WSingleState> BackEndFilter;

    /** Current counter of samples arrived to each port **/
    struct CounterInputPortsBackEnd
    {
        void reset()
        {
            frontEndPoseSamples = 0;
            inertialStateSamples = 0;
            return;
        }

	unsigned int frontEndPoseSamples; /** counter of rover pose comming from the FrontEnd **/
 	unsigned int inertialStateSamples; /** counter of of inertial measurements comming from the FrontEnd **/
    };

    /** Number of samples to process in the callback function **/
    struct NumberInputPortsBackEnd
    {
        void reset()
        {
            frontEndPoseSamples = 0;
            inertialStateSamples = 0;
            return;
        }

	unsigned int frontEndPoseSamples; /** number of rover pose comming from teh FrontEnd **/
 	unsigned int inertialStateSamples; /** number of inertial measurements comming from the FrontEnd **/
    };

    /** Inport samples arrived ON/OFF flags **/
    struct FlagInputPortsBackEnd
    {
        void reset()
        {
            frontEndPoseSamples = false;
            inertialStateSamples = false;
            return;
        }

	bool frontEndPoseSamples; /** number of rover pose comming from teh FrontEnd **/
 	bool inertialStateSamples; /** number of inertial measurements comming from the FrontEnd **/
    };

    /*! \class BackEnd 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Additional exteroceptive update samples to correct the prediction.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','rover_localization::BackEnd')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class BackEnd : public BackEndBase
    {
	friend class BackEndBase;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        static const int  DEFAULT_CIRCULAR_BUFFER_SIZE = 2; /** Default number of objects to store regarding the inputs port **/

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

	/** Init filter **/
	bool initFilter;

        /** Delta time for the noise coeff **/
        double delta_noise;

        /** Number of samples to process in the inports callback function **/
        NumberInputPortsBackEnd number;

        /** Current counter of samples arrived to each inport **/
        CounterInputPortsBackEnd counter;

        /** Data arrived ON/OFF Flag **/
        FlagInputPortsBackEnd flag;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Propioceptive sensors configuration variables **/
        ProprioceptiveSensorProperties sensornoise;

        /** Framework configuration values **/
        FrameworkConfiguration framework;

        /** Adaptive Measurement Configuration **/
        AdaptiveMeasurementProperties adapValues;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** The filter uses by the BackEnd **/
        boost::shared_ptr<BackEndFilter> filter;

        /** Pose estimation from Front-End  **/
        boost::circular_buffer<base::samples::RigidBodyState> frontEndPose;

        /** Inertial sensor from Front-End **/
        boost::circular_buffer<rover_localization::InertialState> inertialState;

        /** Object of Class for Adaptive Measurement of Attitude Covariance Matrix **/
        boost::shared_ptr<localization::AdaptiveAttitudeCov> adapAtt;

        /** Variable in DataModel form for the differences in velocities **/
        localization::DataModel<double, 3> accModel, accInertial;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Buffer for input ports samples comming from the front end **/
        boost::circular_buffer<base::samples::RigidBodyState> frontEndPoseSamples;
        boost::circular_buffer<rover_localization::InertialState> inertialStateSamples;

        /***************************/
        /** Output port variables **/
        /***************************/

    protected:

        virtual void exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample);

        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::rover_localization::InertialState &inertial_samples_sample);

        virtual void update_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &update_samples_sample);

        virtual void visual_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &visual_samples_sample);


    public:
        /** TaskContext constructor for BackEnd
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        BackEnd(std::string const& name = "rover_localization::BackEnd");

        /** TaskContext constructor for BackEnd 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        BackEnd(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of BackEnd
         */
	~BackEnd();

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
        void initBackEndFilter(boost::shared_ptr<BackEndFilter> &filter, boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                boost::circular_buffer<rover_localization::InertialState> &inertialState);

        /**@brief Calculates position error
         */
        localization::DataModel<double, 3> positionError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                                                        const boost::shared_ptr< BackEndFilter > filter);


        /**@brief Calculates velocity error
         */
        localization::DataModel<double, 3> velocityError(const boost::circular_buffer<base::samples::RigidBodyState> &frontEndPose,
                                                        const boost::shared_ptr< BackEndFilter > filter);


        /** \brief Store the variables in the Output ports
         */
        void outputPortSamples (const boost::shared_ptr< localization::Usckf<WAugmentedState, WSingleState> > filter,
                                const WAugmentedState &errorAugmentedState, const localization::DataModel<double, 3> &deltaVeloModel, const localization::DataModel<double, 3> &deltaVeloInertial);


    };
}

#endif


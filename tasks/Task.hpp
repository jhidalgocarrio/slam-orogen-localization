/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_TASK_TASK_HPP
#define LOCALIZATION_TASK_TASK_HPP

#include "localization/TaskBase.hpp"

/** Framework Library dependencies includes **/
#include <localization/Configuration.hpp> /** Constant values of the library */
//#include <localization/core/DataModel.hpp> /** Simple Data Model with uncertainty */
#include <localization/tools/Util.hpp> /** Util class library **/
#include <localization/filters/Usckf.hpp> /** USCKF class with Manifolds */
#include <localization/filters/MtkWrap.hpp> /** USCKF wrapper for the state vector */
#include <localization/filters/State.hpp> /** Filter State */
#include <localization/filters/ProcessModels.hpp> /** Filter Process Models */
//#include <localization/filters/MeasurementModels.hpp> /** Filters Measurement Models */

/** Boost **/
#include <boost/shared_ptr.hpp> /** For shared pointers **/
#include <boost/algorithm/string.hpp> /** case insensitive string compare **/

namespace localization {

    /** Wrap the Augmented and Single State **/
    typedef localization::MtkWrap<localization::State> WSingleState;
    typedef localization::MtkWrap<localization::AugmentedState> WAugmentedState;
    typedef localization::Usckf<WAugmentedState, WSingleState> StateFilter;

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Exteroceptive update samples to correct the prediction (Visual, ICP, etc..).
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:
        typedef RTT::InputPort<localization::ExteroceptiveSample> InputPortExtero;

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/
        bool initFilter;


        /**************************/
        /*** Property Variables ***/
        /**************************/

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** The filter uses by the Back-End **/
        boost::shared_ptr<StateFilter> filter;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Pose estimation **/
        ::base::samples::RigidBodyState pose_sample;

        /* Input ports variables for Exteroceptive **/
        std::vector<InputPortExtero*> mInputExtero;


        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState pose_out;

    protected:

        /** Deletes all defined input and output ports */
        void clearPorts();

        virtual void pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample);

        //virtual void exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample);

        //virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "localization::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

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
        void initStateFilter(boost::shared_ptr<StateFilter> &filter, Eigen::Affine3d &tf);

        /** @brief Port out the values
        */
        void outputPortSamples(const base::Time &timestamp);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };
}

#endif


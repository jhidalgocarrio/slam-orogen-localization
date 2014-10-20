/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_DISPATCHER_TASK_HPP
#define LOCALIZATION_DISPATCHER_TASK_HPP

#include "localization/DispatcherBase.hpp"

namespace localization {

    struct DispatcherNamedVector
    {
        void clear()
        {
            delta_pose.clear();
            point_cloud.clear();
            covariance.clear();
            jacobian_k.clear();
            jacobian_k_m.clear();
            return;
        }

        void erase( std::string const& delta_pose_name,
                    std::string const& pointcloud_name,
                    std::string const& covariance_name,
                    std::string const& jacobian_k_name,
                    std::string const& jacobian_k_m_name)
        {
            delta_pose.elements.erase(delta_pose.elements.begin() + delta_pose.mapNameToIndex(delta_pose_name));
            delta_pose.names.erase(delta_pose.names.begin() + delta_pose.mapNameToIndex(delta_pose_name));

            point_cloud.elements.erase(point_cloud.elements.begin() + point_cloud.mapNameToIndex(pointcloud_name));
            point_cloud.names.erase(point_cloud.names.begin() + point_cloud.mapNameToIndex(pointcloud_name));

            covariance.elements.erase(covariance.elements.begin() + covariance.mapNameToIndex(covariance_name));
            covariance.names.erase(covariance.names.begin() + covariance.mapNameToIndex(covariance_name));

            jacobian_k.elements.erase(jacobian_k.elements.begin() + jacobian_k.mapNameToIndex(jacobian_k_name));
            jacobian_k.names.erase(jacobian_k.names.begin() + jacobian_k.mapNameToIndex(jacobian_k_name));

            jacobian_k_m.elements.erase(jacobian_k_m.elements.begin() + jacobian_k_m.mapNameToIndex(jacobian_k_m_name));
            jacobian_k_m.names.erase(jacobian_k_m.names.begin() + jacobian_k_m.mapNameToIndex(jacobian_k_m_name));

            return;
        }

        base::NamedVector<base::samples::RigidBodyState> delta_pose;
        base::NamedVector<base::samples::Pointcloud> point_cloud;
        base::NamedVector<std::vector<base::Matrix3d> > covariance;
        base::NamedVector<base::MatrixXd> jacobian_k;
        base::NamedVector<base::MatrixXd> jacobian_k_m;
    };

    /*! \class Dispatcher 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare the State Optimization class
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','localization::Dispatcher')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Dispatcher : public DispatcherBase
    {
	friend class DispatcherBase;
    protected:

        typedef RTT::InputPort<base::samples::RigidBodyState> InputPortPose;
        typedef RTT::InputPort<base::samples::Pointcloud> InputPortPointcloud;
        typedef RTT::InputPort< std::vector<base::Matrix3d> > InputPortCov;
        typedef RTT::InputPort<base::MatrixXd> InputPortJacob;
        typedef RTT::OutputPort<localization::ExteroceptiveSample> OutputPort;

        /** Dispatcher Configuration **/
        std::vector<OutputPortsConfiguration> config;

        /** Input ports variables **/
        std::vector<InputPortPose*> mInputPose;
        std::vector<InputPortPointcloud*> mInputPointcloud;
        std::vector<InputPortCov*> mInputCov;
        std::vector<InputPortJacob*> mInputJacobk;
        std::vector<InputPortJacob*> mInputJacobk_m;

        /** Internal storage variables **/
        DispatcherNamedVector dispatcher;

        /** Output ports variables **/
        std::vector<OutputPort*> mOutputPorts;

    protected:

        /** Deletes all defined input and output ports */
        void clearPorts();

    public:
        /** TaskContext constructor for Dispatcher
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Dispatcher(std::string const& name = "localization::Dispatcher", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Dispatcher 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Dispatcher(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Dispatcher
         */
	~Dispatcher();

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
    };
}

#endif

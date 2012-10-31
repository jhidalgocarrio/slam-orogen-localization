/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ASGUARD_LOCALIZATION_TASK_TASK_HPP
#define ASGUARD_LOCALIZATION_TASK_TASK_HPP

#include <asguard/KinematicModel.hpp>
#include <asguard/BodyState.hpp>

#include <Eigen/Core>
#include <Eigen/Dense> /** for the algebra and transformation matrices **/


#include "asguard_localization/TaskBase.hpp"

namespace asguard_localization {
    
    
     /** General defines **/
    #ifndef OK
    #define OK	0  /** Integer value in order to return when everything is all right. */
    #endif
    #ifndef ERROR
    #define ERROR -1  /** Integer value in order to return when an error occured. */
    #endif
    
    #ifndef D2R
    #define D2R M_PI/180.00 /** Convert degree to radian **/
    #endif
    #ifndef R2D
    #define R2D 180.00/M_PI /** Convert radian to degree **/
    #endif
    
    #ifndef NUMAXIS
    #define NUMAXIS 3 /**< Number of axis sensed by the sensors **/
    #endif

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Otherwise, the linear acceleration is given by the inclinometers which has a range between +1 and -1 g.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','asguard_localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	
	/** Wheel kinematics structures **/
	asguard::KinematicModel wheelFL;
	asguard::KinematicModel wheelFR;
	asguard::KinematicModel wheelRL;
	asguard::KinematicModel wheelRR;
	
	/** Integration step for the dead-reckoning **/
	base::Time delta_t;
	
	/** Data arrived **/
	bool imuValues, hbridgeValues, asguardValues, orientationValues;
	
	/** Inputs port samples **/
	int current[NUMBER_WHEELS]; /** Array to perform the current mean value **/
	float pwm[NUMBER_WHEELS]; /** Array to perform the PWM mean value **/
	base::samples::IMUSensors imuSamples; /** IMU samples **/
	base::actuators::Status hbridgeStatus; /** Hbridge Status information  **/
	sysmon::SystemStatus asguardStatus; /** Asguard status information **/
	std::vector<int> contactPoints; /** Number between 0 and 4 of the feet in contact **/
	base::samples::RigidBodyState orientation; /** Orientation information (debug)**/
	
	/** Status information replica to compute the velocity **/
	base::actuators::Status prevHbridgeStatus; /** Hbridge Status information **/
	base::samples::RigidBodyState prevOrientation; /** Orientation information (debug)**/
	sysmon::SystemStatus prevAsguardStatus; /** Asguard status information **/
	
	
	/** Body Center w.r.t the World Coordinate system **/
	base::samples::RigidBodyState rbsBC;

        virtual void calibrated_sensorsTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &calibrated_sensors_sample);
        virtual void ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample);
        virtual void hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample);
        virtual void systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample);
        virtual void torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample);
	virtual void orientation_debugTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_debug_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "asguard_localization::Task");

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
    };
}

#endif


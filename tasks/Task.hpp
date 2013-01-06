/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_TASK_TASK_HPP
#define ROVER_LOCALIZATION_TASK_TASK_HPP

#include <asguard/KinematicModel.hpp>
#include <asguard/BodyState.hpp>
#include <rover_localization/Sckf.hpp>
#include <rover_localization/DeadReckon.hpp>
#include <rover_localization/DataTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Dense> /** for the algebra and transformation matrices **/


#include "rover_localization/TaskBase.hpp"

namespace rover_localization {
    
//     #ifndef EIGEN_NO_AUTOMATIC_RESIZING
//     #define EIGEN_NO_AUTOMATIC_RESIZING
//     #endif
    
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
    #define NUMAXIS 3 /** Number of axis sensed by the sensors **/
    #endif
    
    #ifndef NUMBER_INIT_ACC
    #define NUMBER_INIT_ACC 1000 /** Number acc samples to compute initial pitch and roll considering not init_attitude provided by pose_init **/
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
         task('custom_task_name','rover_localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	
	/** SCKF structure **/
	localization::Sckf mysckf;
	
	/** Measurement Generation pointer **/
	localization::Measurement *mymeasure;
	
	/** Init pose **/
	bool initPosition, initAttitude;
	
	/** Index for acc mean value for init attitude (Pose init process) **/
	int accidx; 	
	
	/** Integration step for the filter in seconds **/
	double delta_t;
	
	/** Error in the Least-Squares computation **/
	double leastSquaresError;
	
	/** Number of samples to process in the callback function **/
	unsigned int numberIMUSamples; /** number of Ground Force info samples **/
	unsigned int numberHbridgeSamples; /** number of Hbridge samples for the resampling**/
 	unsigned int numberAsguardStatusSamples; /** number of  Asguard status samples for the resampling **/
 	unsigned int numberTorqueSamples; /** number of  Torque info samples for the resampling **/
 	unsigned int numberForceSamples; /** number of Ground Force info samples for the resampling **/
 	
 	/** Current counter of samples arrived to each port **/
 	unsigned int counterHbridgeSamples; /** counter for Hbridge samples**/
 	unsigned int counterAsguardStatusSamples; /** counter for Asguard status samples  **/
 	unsigned int counterTorqueSamples; /** counter for  Torque info samples **/
 	unsigned int counterForceSamples; /** number of Ground Force info samples **/
 	unsigned int counterIMUSamples; /** number of Ground Force info samples **/
 	
	/** Wheel kinematics structures **/
	asguard::KinematicModel wheelFL;
	asguard::KinematicModel wheelFR;
	asguard::KinematicModel wheelRL;
	asguard::KinematicModel wheelRR;
	
	/** Data arrived **/
	bool imuValues, hbridgeValues, asguardValues, poseInitValues;
	
	/** Accelerometers eccentricity **/
	Eigen::Matrix<double, NUMAXIS,1> eccx, eccy, eccz;
	
	/** Inputs port samples **/
	base::actuators::Status hbridgeStatus; /** Hbridge Status information  **/
	sysmon::SystemStatus asguardStatus; /** Asguard status information **/
	base::samples::IMUSensors imuSamples; /** IMU samples **/
	base::samples::RigidBodyState poseInit; /** Orientation information (init and debug)**/
	
	/** Status information replica to compute the velocity **/
	base::actuators::Status prevHbridgeStatus; /** Hbridge Status information **/
	base::samples::RigidBodyState prevPoseInit; /** Orientation information (init and debug)**/
	sysmon::SystemStatus prevAsguardStatus; /** Asguard status information **/
	base::samples::IMUSensors prevImuSamples; /** IMU samples **/
	
	/** Input ports timing **/
	base::Time outTimeHbridge, outTimeStatus, outTimeTorque, outTimeForce, outTimeIMU, outTimeOrientation;
	
	std::vector<int> contactPoints; /** Number between 0 and 4 of the feet in contact (identification) **/
	std::vector<double> contactAngle; /** Current contact angle for the foot in contact (angle in radians) **/
	
	/** Initial values of Acceleremeters for Picth and Roll calculation */
	Eigen::Matrix <double,NUMAXIS, NUMBER_INIT_ACC> init_acc;
	
	/** Joint encoders velocities (order is 0 -> PassiveJoint, 1-> RL, 2 -> RR, 3 -> FR, 4 -> FL, ) **/
	Eigen::Matrix< double, Eigen::Dynamic, 1  > vjoints;
	
	/** Jacobian matrix for the rover Eu = Jp **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> E; /** Sparse matrix (24 x 6) **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> J; /** Sparse Wheels Jacobian matrix (24 x 21) **/
	
	/** Matrices for the navigation kinematics **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> Anav; /** Not-sensed matrix **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> Bnav; /** Sensed matrix **/
	
	/** Matrices for the slip kinematics **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> Aslip; /** Not-sensed matrix **/
	Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> Bslip; /** Sensed matrix **/
	
	/** Body Center w.r.t the World Coordinate system **/
	base::samples::RigidBodyState rbsBC;
	
	/** Variable to perform the dead-reckoning process **/
	localization::DeadReckon drPose;
	
	/** Feet for FL wheel **/
	base::samples::RigidBodyState rbsC0FL2body;
	base::samples::RigidBodyState rbsC1FL2body;
	base::samples::RigidBodyState rbsC2FL2body;
	base::samples::RigidBodyState rbsC3FL2body;
	base::samples::RigidBodyState rbsC4FL2body;
	
	/** Feet for FR wheel **/
	base::samples::RigidBodyState rbsC0FR2body;
	base::samples::RigidBodyState rbsC1FR2body;
	base::samples::RigidBodyState rbsC2FR2body;
	base::samples::RigidBodyState rbsC3FR2body;
	base::samples::RigidBodyState rbsC4FR2body;
	
	/** Feet for RL wheel **/
	base::samples::RigidBodyState rbsC0RL2body;
	base::samples::RigidBodyState rbsC1RL2body;
	base::samples::RigidBodyState rbsC2RL2body;
	base::samples::RigidBodyState rbsC3RL2body;
	base::samples::RigidBodyState rbsC4RL2body;
	
	/** Feet for RR wheel **/
	base::samples::RigidBodyState rbsC0RR2body;
	base::samples::RigidBodyState rbsC1RR2body;
	base::samples::RigidBodyState rbsC2RR2body;
	base::samples::RigidBodyState rbsC3RR2body;
	base::samples::RigidBodyState rbsC4RR2body;
	
	/** Wheel Jacobian Matrices **/
	Eigen::Matrix <double, 2*NUMAXIS, Eigen::Dynamic> jacobFL;
	Eigen::Matrix <double, 2*NUMAXIS, Eigen::Dynamic> jacobFR;
	Eigen::Matrix <double, 2*NUMAXIS, Eigen::Dynamic> jacobRL;
	Eigen::Matrix <double, 2*NUMAXIS, Eigen::Dynamic> jacobRR;

        virtual void calibrated_sensorsTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &calibrated_sensors_sample);
        virtual void ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample);
        virtual void hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample);
        virtual void systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample);
        virtual void torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample);
	virtual void pose_initTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_init_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "rover_localization::Task");

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
	
	/** \brief Select the Foot in contact among all the Foot Points
	 */
	void selectContactPoints (std::vector<int> &contactPoints);
	
	/** \brief Select the Foot in contact among all the Foot Points
	 */
	void calculateFootPoints ();
	
	/** \brief Compute Joint velocities 
	 */
	void calculateEncodersVelocities ();
	
	/** \brief It forms the composite matrices for the
	 * slip kinematics and the filter observation.
	 */
	void compositeMotionJacobians ();
	
	/** \Brief Fill the Asguard BodyState structure
	 * 
	 * All the produced information relevant to Asguard BodyState class
	 * is store in it in order to port out to the correspondent port.
	 * 
	 * @return void
	 */
	void toAsguardBodyState();
	
	/** \Brief Write debug info in the ports
	 * 
	 * @return void
	 */
	void toDebugPorts();
	
    };
}

#endif


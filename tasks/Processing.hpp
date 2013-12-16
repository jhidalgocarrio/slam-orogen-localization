/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_PROCESSING_TASK_HPP
#define ROVER_LOCALIZATION_PROCESSING_TASK_HPP

#include "rover_localization/ProcessingBase.hpp"

/** Localization library **/
#include <localization/tools/Util.hpp> /** Collection of methods */

/** General Libraries **/
#include <math.h> /** math library (for natural Log among others) **/
#include <vector> /** std vector **/

/** Eigen **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/StdVector> /** For STL container with Eigen types **/
#include <Eigen/Dense> /** Algebra and transformation matrices **/

/** Boost **/
#include <boost/circular_buffer.hpp> /** For circular buffers **/

/** Rock libraries **/
#include "frame_helper/FrameHelper.h" /** Rock lib for manipulate frames **/


namespace rover_localization {

    /** Current counter of samples arrived to each port **/
    struct CounterInputPorts
    {
        void reset()
        {
            encoderSamples = 0;
            asguardStatusSamples = 0;
            imuSamples = 0;
            orientSamples = 0;
            torqueSamples = 0;
            forceSamples = 0;
            referencePoseSamples = 0;
            return;
        }

       	unsigned int encoderSamples; /** counter for encoders samples**/
 	unsigned int asguardStatusSamples; /** counter for Asguard status samples  **/
 	unsigned int imuSamples; /** counter of inertial sensors samples **/
 	unsigned int orientSamples; /** counter of orientation samples **/
 	unsigned int torqueSamples; /** counter for  Torque info samples **/
 	unsigned int forceSamples; /** counter of Ground Force info samples for the re-sampling **/
 	unsigned int referencePoseSamples; /** counter of pose information coming from external measurement **/

    };

    /** Number of samples to process in the callback function **/
    struct NumberInputPorts
    {
        void reset()
        {
            encoderSamples = 0;
            asguardStatusSamples = 0;
            imuSamples = 0;
            orientSamples = 0;
            torqueSamples = 0;
            forceSamples = 0;
            referencePoseSamples = 0;
            return;
        }

	unsigned int encoderSamples; /** number of encoders samples for the re-sampling**/
 	unsigned int asguardStatusSamples; /** number of  Asguard status samples for the re-sampling **/
 	unsigned int imuSamples; /** number of inertial sensors samples **/
 	unsigned int orientSamples; /** number of orientation samples **/
 	unsigned int torqueSamples; /** number of  Torque info samples for the re-sampling **/
 	unsigned int forceSamples; /** number of Ground Force info samples for the re-sampling **/
 	unsigned int referencePoseSamples; /** number of pose information coming from external measurement **/
    };

    /** Input port samples arrived ON/OFF flags **/
    struct FlagInputPorts
    {
        void reset()
        {
            encoderSamples = false;
            asguardStatusSamples = false;
            imuSamples = false;
            orientSamples = false;
            torqueSamples = false;
            forceSamples = false;
            referencePoseSamples = false;
            return;
        }

        bool encoderSamples;//Encoders
        bool asguardStatusSamples;//Passive joint
        bool imuSamples;//Inertial sensors
        bool orientSamples;//Orientation
        bool torqueSamples;//Torque
        bool forceSamples;//Ground Force
        bool referencePoseSamples;//Initial pose
    };

    class Processing : public ProcessingBase
    {
	friend class ProcessingBase;

    protected:
        static const int  DEFAULT_INIT_LEVELING_SIZE =  1000; /** Default number of the initLeveling **/
        static const int  DEFAULT_CIRCULAR_BUFFER_SIZE = 2; /** Default number of objects to store regarding the inputs port **/

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

	/** Initial pose for the world to navigation transform **/
	bool initPosition, initAttitude;

        /** Index for acceleration mean value for initializing attitude (Pose init process) **/
	int init_leveling_accidx;

        /** Number acceleration samples to compute initial pitch and roll considering not init_attitude provided by pose_init **/
        int init_leveling_size;

        /** Number of samples to process in the input ports callback function **/
        NumberInputPorts number;

 	/** Current counter of samples arrived to each input port **/
        CounterInputPorts counter;

        /** Data arrived ON/OFF Flag **/
        FlagInputPorts flag;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Framework configuration values **/
        Configuration config;

        /** Camera Synch configuration **/
        CameraSynchConfiguration cameraSynch;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

	/** Initial values of Accelerometers for Pitch and Roll calculation */
	Eigen::Matrix <double, 3, Eigen::Dynamic> init_leveling_acc;
	
	/** Initial values of Accelerometers (Inclinometers) for Pitch and Roll calculation */
	Eigen::Matrix <double, 3, Eigen::Dynamic> init_leveling_incl;

        /** Initial values of Gyroscopes (sanity check) */
	Eigen::Matrix <double, 3, Eigen::Dynamic> init_gyroscopes;

        /** Align of world to navigation (in case of true in the properties) **/
        base::samples::RigidBodyState alignWorld2Navigation;

        /** Body to Left camera transformation **/
        base::samples::RigidBodyState body2lcameraRbs;

        /** Frame helper **/
        frame_helper::FrameHelper frameHelperLeft, frameHelperRight;

        /***********************************/
        /** Input ports dependent buffers **/
        /***********************************/

        /** Buffer for raw inputs port samples (the desired filter frequency) **/
 	boost::circular_buffer<base::actuators::Status> cbEncoderSamples;
	boost::circular_buffer<sysmon::SystemStatus> cbAsguardStatusSamples;
	boost::circular_buffer<base::samples::IMUSensors> cbImuSamples;
	boost::circular_buffer<base::samples::RigidBodyState> cbOrientSamples;
	
 	/** Buffer for filtered Inputs port samples (Store the samples and compute the velocities) **/
	boost::circular_buffer<base::actuators::Status> encoderSamples; /** Encoder Status information  **/
	boost::circular_buffer<sysmon::SystemStatus> asguardStatusSamples; /** Asguard status information **/
	boost::circular_buffer<base::samples::IMUSensors> imuSamples; /** IMU samples **/
	boost::circular_buffer<base::samples::RigidBodyState> orientSamples; /** IMU samples **/
	boost::circular_buffer<base::samples::RigidBodyState> referencePoseSamples; /** Pose information (init and debug)**/

        /** State information **/
        base::samples::RigidBodyState poseEstimationSamples;

        /** Gyros and Accelerometers Bias **/
        base::Vector3d accbias, gyrobias;

        /***************************/
        /** Output port variables **/
        /***************************/

        /** Joints state of the robot **/
        base::samples::Joints jointSamples;

        /** Calibrated and compensated inertial values **/
        base::samples::IMUSensors inertialSamples;

        /** Ground truth out coming for an external system (if available like Vicon or GPS) */
        base::samples::RigidBodyState referenceOut;

        /** Calculated initial navigation frame pose expressed in world frame */
        base::samples::RigidBodyState world2navigationRbs;

        /** Undistorted camera images **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> rightFrame;

        /************************/
        /** Callback functions **/
        /************************/

        virtual void reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

        virtual void torque_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_samples_sample);

        virtual void ground_forces_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_samples_sample);

        virtual void systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample);

        virtual void encoder_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &encoder_samples_sample);

        virtual void left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);

        virtual void right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample);

        virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);

    public:
        /** TaskContext constructor for Processing
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Processing(std::string const& name = "rover_localization::Processing");

        /** TaskContext constructor for Processing
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Processing(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Processing
         */
	~Processing();

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

        /** @brief Get the correct value from the input ports buffers
	 */
	void inputPortSamples();

        /** @brief Compute Cartesian and Model velocities 
	 */
	void calculateVelocities();

        /** @brief Camera with Servo Synchronization
         */
        void cameraWithDynamixelSynchro(const base::Time &ts, const Eigen::Affine3d &tf,
                        const RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> &leftFrame,
                        const RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> &rightFrame);

        /** @brief Port out the values
	 */
        void outputPortSamples();

     public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif


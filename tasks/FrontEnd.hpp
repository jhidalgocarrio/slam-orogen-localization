/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROVER_LOCALIZATION_FRONTEND_TASK_HPP
#define ROVER_LOCALIZATION_FRONTEND_TASK_HPP

#include "rover_localization/FrontEndBase.hpp"

/** Asguard dependencies includes **/
#include <asguard/AsguardKinematicModel.hpp> /** Analytical model **/
#include <asguard/AsguardKinematicKDL.hpp> /** KDL model **/
#include <asguard/Configuration.hpp> /** For dedicated Asguard variables and const **/
#include <asguard/BodyState.hpp> /** BodyState representation and constants **/

/** Framework Library includes **/
#include <rover_localization/Util.hpp>
#include <rover_localization/Analysis.hpp>
#include <rover_localization/DeadReckon.hpp>
#include <rover_localization/DataTypes.hpp>
#include <rover_localization/filters/IIR.hpp>

/** Odometry include for the Motion Model **/
#include <odometry/MotionModel.hpp>

/** General Libraries **/
#include <math.h> /** For natural Log **/
#include <vector> /** std vector **/

/** Eigen **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/StdVector> /** For STL container with Eigen types **/
#include <Eigen/Dense> /** Algebra and transformation matrices **/

/** Boost **/
#include <boost/circular_buffer.hpp> /** For circular buffers **/
#include <boost/shared_ptr.hpp> /** For shared pointers **/

namespace rover_localization {

    /** Current counter of samples arrived to each port **/
    struct CounterInputPortsFrontEnd
    {
        void reset()
        {
            encoderSamples = 0;
            asguardStatusSamples = 0;
            imuSamples = 0;
            torqueSamples = 0;
            forceSamples = 0;
            referencePoseSamples = 0;
            return;
        }

       	unsigned int encoderSamples; /** counter for encoders samples**/
 	unsigned int asguardStatusSamples; /** counter for Asguard status samples  **/
 	unsigned int imuSamples; /** counter of inertial sensors samples **/
 	unsigned int torqueSamples; /** counter for  Torque info samples **/
 	unsigned int forceSamples; /** counter of Ground Force info samples for the re-sampling **/
 	unsigned int referencePoseSamples; /** counter of pose information coming from external measurement **/

    };

    /** Number of samples to process in the callback function **/
    struct NumberInputPortsFrontEnd
    {
        void reset()
        {
            encoderSamples = 0;
            asguardStatusSamples = 0;
            imuSamples = 0;
            torqueSamples = 0;
            forceSamples = 0;
            referencePoseSamples = 0;
            return;
        }

	unsigned int encoderSamples; /** number of encoders samples for the re-sampling**/
 	unsigned int asguardStatusSamples; /** number of  Asguard status samples for the re-sampling **/
 	unsigned int imuSamples; /** number of inertial sensors samples **/
 	unsigned int torqueSamples; /** number of  Torque info samples for the re-sampling **/
 	unsigned int forceSamples; /** number of Ground Force info samples for the re-sampling **/
 	unsigned int referencePoseSamples; /** number of pose information coming from external measurement **/
    };

    /** Inport samples arrived ON/OFF flags **/
    struct FlagInputPortsFrontEnd
    {
        void reset()
        {
            encoderSamples = false;
            asguardStatusSamples = false;
            imuSamples = false;
            torqueSamples = false;
            forceSamples = false;
            referencePoseSamples = false;
            return;
        }

        bool encoderSamples;//Encoders
        bool asguardStatusSamples;//Passive joint
        bool imuSamples;//Inertial sensors
        bool torqueSamples;//Torque
        bool forceSamples;//Ground Force
        bool referencePoseSamples;//Initial pose
    };

    /** Data types definition **/
    typedef odometry::KinematicModel< double, asguard::NUMBER_OF_WHEELS, asguard::ASGUARD_JOINT_DOF, asguard::SLIP_VECTOR_SIZE, asguard::CONTACT_POINT_DOF > frontEndKinematicModel;
    typedef odometry::MotionModel< double, asguard::NUMBER_OF_WHEELS, asguard::ASGUARD_JOINT_DOF, asguard::SLIP_VECTOR_SIZE, asguard::CONTACT_POINT_DOF > frontEndMotionModel;
    typedef Eigen::Matrix<double, 6*asguard::NUMBER_OF_WHEELS, 6*asguard::NUMBER_OF_WHEELS> WeightingMatrix;

    /*! \class FrontEnd 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Note: Check if the IMU inport has inclinometers information (right now coded in mag).
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','rover_localization::FrontEnd')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class FrontEnd : public FrontEndBase
    {
	friend class FrontEndBase;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        static const int  DEFAULT_INIT_LEVELING_SIZE =  1000; /** Default number of the initLeveling **/
        static const int  DEFAULT_CIRCULAR_BUFFER_SIZE = 2; /** Default number of objects to store regarding the inputs port **/

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

	/** Init pose **/
	bool initPosition, initAttitude;

        /** Index for acc mean value for init attitude (Pose init process) **/
	int init_leveling_accidx;

        /** Number acceleration samples to compute initial pitch and roll considering not init_attitude provided by pose_init **/
        int init_leveling_size;

        /** Number of samples to process in the inports callback function **/
        NumberInputPortsFrontEnd number;

 	/** Current counter of samples arrived to each inport **/
        CounterInputPortsFrontEnd counter;

        /** Data arrived ON/OFF Flag **/
        FlagInputPortsFrontEnd flag;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Location configuration variables **/
        LocationConfiguration location;

        /** Framework configuration values **/
        FrameworkConfiguration framework;

        /** Proprioceptive sensors configuration variables **/
        ProprioceptiveSensorProperties propriosensor;

        /** IIR filter configuration structure **/
        IIRCoefficients iirConfig;

        /** Center of Mass location of the robot **/
        CenterOfMassConfiguration centerOfMass;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

	/** Initial values of Accelerometers for Pitch and Roll calculation */
	Eigen::Matrix <double,localization::NUMAXIS, Eigen::Dynamic> init_leveling_acc;
	
	/** Initial values of Accelerometers (Inclinometers) for Pitch and Roll calculation */
	Eigen::Matrix <double,localization::NUMAXIS, Eigen::Dynamic> init_leveling_incl;

        /** Accelerometers eccentricity **/
	Eigen::Matrix<double, localization::NUMAXIS,1> eccx, eccy, eccz;

        /** Robot Kinematic Model **/
        boost::shared_ptr< frontEndKinematicModel > robotKinematics;

        /** Robot Motion Model **/
        frontEndMotionModel  motionModel;

        /** Joint encoders, Slip and Contact Angle velocities NOTE: The order of the storage needs to be coincident if used as input for the motionModel **/
        Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, frontEndMotionModel::MODEL_DOF > modelVelCov;

        /** Linear and Angular velocities NOTE: The order of the storage needs to be coincident to be used as input for the motionModel **/
        Eigen::Matrix< double, 6, 6  > cartesianVelCov;

        /** Buffer for the storage of cartesianVelocities variables  (for integration assuming constant accelartion) **/
        std::vector< Eigen::Matrix <double, 2*localization::NUMAXIS, 1> , Eigen::aligned_allocator < Eigen::Matrix <double, 2*localization::NUMAXIS, 1> > > vectorCartesianVelocities;

        /** Bessel Low-pass IIR filter for the Motion Model velocities
         * Specification of the Order and Data dimension is required */
        boost::shared_ptr< localization::IIR<localization::NORDER_BESSEL_FILTER, localization::NUMAXIS> > bessel;

        /** Sensitivity analysis **/
        localization::Analysis <3, 3+asguard::ASGUARD_JOINT_DOF> modelAnalysis; //! DoF of the analysis is 8

        /** Weighting Matrix for the Motion Model  **/
        WeightingMatrix WeightMatrix;

        /***********************************/
        /** Input ports dependent buffers **/
        /***********************************/

        /** Buffer for raw inputs port samples (Front-End to the desired filter frequency) **/
 	boost::circular_buffer<base::actuators::Status> cbEncoderSamples;
	boost::circular_buffer<sysmon::SystemStatus> cbAsguardStatusSamples;
	boost::circular_buffer<base::samples::IMUSensors> cbImuSamples;
	
 	/** Buffer for filtered Inputs port samples (Store the samples for the Front-End and compute the velocities) **/
	boost::circular_buffer<base::actuators::Status> encoderSamples; /** Encoder Status information  **/
	boost::circular_buffer<sysmon::SystemStatus> asguardStatusSamples; /** Asguard status information **/
	boost::circular_buffer<base::samples::IMUSensors> imuSamples; /** IMU samples **/
	boost::circular_buffer<base::samples::RigidBodyState> poseSamples; /** Pose information (init and debug)**/

        /** Back-End information (feedback) **/
	rover_localization::BackEndEstimation backEndEstimationSamples;

        /***************************/
        /** Output port variables **/
        /***************************/

        /** Body Center w.r.t the World Coordinate system (using statistical Motion Model and IMU orientation) */
	base::samples::RigidBodyState poseOut;

        /** Ground truth out coming for an external system (if available like Vicon or GPS) */
        base::samples::RigidBodyState referenceOut;

        /** Corrected inertial values **/
        rover_localization::InertialState inertialState;

        /************************/
        /** Callback functions **/
        /************************/

        virtual void reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample);

        virtual void inertial_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample);

        virtual void torque_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_samples_sample);

        virtual void ground_forces_samplesTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_samples_sample);

        virtual void systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample);

        virtual void encoder_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &encoder_samples_sample);

        /** Weight matrix for the Asguard Robot **/
        WeightingMatrix dynamicWeightMatrix (CenterOfMassConfiguration &centerOfMass, base::Orientation &orientation);

    public:
        /** TaskContext constructor for FrontEnd
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        FrontEnd(std::string const& name = "rover_localization::FrontEnd");

        /** TaskContext constructor for FrontEnd 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        FrontEnd(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of FrontEnd
         */
	~FrontEnd();

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

        /** \brief Get the correct value from the input ports buffers
	 */
	void inputPortSamples(Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelPositions);

        /** \brief Compute Cartesian and Model velocities 
	 */
	void calculateVelocities(Eigen::Matrix< double, 6, 1  > &cartesianVelocities, Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelVelocities);

        /** \brief Store the variables in the Output ports
         */
        void outputPortSamples(const Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelPositions,
                                const Eigen::Matrix< double, 6, 1  > &cartesianVelocities,
                                const Eigen::Matrix< double, frontEndMotionModel::MODEL_DOF, 1  > &modelVelocities,
                                const base::samples::RigidBodyState &deltaPose,
                                const rover_localization::SensitivityAnalysis &sensitivityAnalysis);


    };
}

#endif


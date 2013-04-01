/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#define DEBUG_PRINTS 1

using namespace rover_localization;
using namespace localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    
    /** Booleans values to false **/
    imuValues = false;
    hbridgeValues = false;
    asguardValues = false;
    poseInitValues = false;
    initPosition = false;
    initAttitude = false;
    useInclinometers = false;
    
    /** Set index values **/
    accidx = 0;
    samplesidx = 0;
    filteridx = 0;
    
    /** Default size for initial acceleration leveling **/
    init_acc.resize (NUMAXIS, NUMBER_INIT_LEVELING);
    
    /** Default size for initial acceleration (inclinometers) leveling **/
    init_incl.resize (NUMAXIS, NUMBER_INIT_LEVELING);
    
    /** Default size for the circular_buffer **/
    cbHbridges = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbAsguard = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    cbIMU = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    
    /** Default size for the circular_buffer for the filtered samples **/
    hbridgeStatus = boost::circular_buffer<base::actuators::Status>(DEFAULT_CIRCULAR_BUFFER_SIZE);
    asguardStatus = boost::circular_buffer<sysmon::SystemStatus> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    imuSamples = boost::circular_buffer<base::samples::IMUSensors> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    poseInit = boost::circular_buffer<base::samples::RigidBodyState> (DEFAULT_CIRCULAR_BUFFER_SIZE);
    
    /** Size the Feet per wheel **/
    rbsCiFL2body = boost::circular_buffer<base::samples::RigidBodyState>(FEET_PER_WHEEL);
    rbsCiFR2body = boost::circular_buffer<base::samples::RigidBodyState>(FEET_PER_WHEEL);
    rbsCiRL2body = boost::circular_buffer<base::samples::RigidBodyState>(FEET_PER_WHEEL);
    rbsCiRR2body = boost::circular_buffer<base::samples::RigidBodyState>(FEET_PER_WHEEL);
    
    /** Wheel kinematics model objects **/
    wheelFL = asguard::KinematicModel(3,2);
    wheelFR = asguard::KinematicModel(2,2);
    wheelRR = asguard::KinematicModel(1,2);
    wheelRL = asguard::KinematicModel(0,2);
    
    /** Set eccentricity to NaN **/
    eccx[0] = base::NaN<double>();
    eccx[1] = base::NaN<double>();
    eccx[2] = base::NaN<double>();
    eccy = eccx;
    eccz = eccx;
    
    /** Set the correct size for the joint velocities vector **/
    vjoints.resize (1 + NUMBER_WHEELS, 1);
    
    /** Contact angle to zero **/
    contactAngle.resize(NUMBER_WHEELS);
    std::fill(contactAngle.begin(), contactAngle.end(), 0.00);
    
    /** Set also here the default Foot in Contact **/
    contactPoints.resize(4);
    contactPoints[0] = 2; contactPoints[1] = 2;
    contactPoints[2] = 2; contactPoints[3] = 2;
    
    /** Point the measurement pointer to the one in the filter object **/
    mymeasure = &(mysckf.measurement);
    
    slipmaxnorm = 0.00;
    
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    /** Measurement pointer to null **/
    mymeasure = NULL;
}

void Task::calibrated_sensorsTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &calibrated_sensors_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/
    
    /** Get the transformation **/
    if (!_imu2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (tf.rotation());//!Quaternion from IMU to Body
    
    #ifdef DEBUG_PRINTS
    std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
    #endif
	
    /** Check if the eccentricities are not defined **/
    if (base::isNaN(eccx[0]) && base::isNaN(eccy[0]) && base::isNaN(eccz[0]))
    {
	#ifdef DEBUG_PRINTS
	std::cout<<"Eccentricities are NaN\n";
	std::cout<<"Translation:\n"<<tf.translation()<<"\n";
	std::cout<<"Eccx :\n"<<tf.rotation() * _eccx.value()<<"\n";
	std::cout<<"Eccy :\n"<<tf.rotation() * _eccy.value()<<"\n";
	std::cout<<"Eccz :\n"<<tf.rotation() * _eccz.value()<<"\n";
	#endif
	
	/** Store the excentricity **/
	eccx = tf.translation() + tf.rotation() *_eccx.value();
	eccy = tf.translation() + tf.rotation() *_eccy.value();
	eccz = tf.translation() + tf.rotation() *_eccz.value();
	
	/** Set the eccentricity to the filter **/
	mysckf.measurement.setEccentricity(eccx, eccy, eccz);
    }
    
    /** If the initial attitude is not defined and the orientation port is not connected **/
    if (!initAttitude)
    {
	#ifdef DEBUG_PRINTS
	std::cout<< "Calculating initial level position since Init orientation is not provided." <<"\n";
	#endif
	
	/** Add one acc sample to the buffer **/
	init_acc.col(accidx) = calibrated_sensors_sample.acc;
	
	/** Add one acc(inclinometer) sample to the buffer **/
	init_incl.col(accidx) = calibrated_sensors_sample.mag; //!TO-DO: still incl values come into the mag field of the class
	
	accidx++;
	
	/** Chekc if the number of stores acceleration corresponds to the initial leveling **/
	if (accidx == this->initLeveling)
	{
	    Eigen::Matrix <double,NUMAXIS,1> meanacc; /** Mean value of accelerometers **/
	    Eigen::Matrix <double,NUMAXIS,1> meanincl; /** Mean value of inclinometer **/
	    Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude; /** Initial attitude in case no port in orientation is connected **/
	    
	    /** Mean inclinometers value **/
	    meanincl[0] = init_incl.row(0).mean();
	    meanincl[1] = init_incl.row(1).mean();
	    meanincl[2] = init_incl.row(2).mean();
	    
	    /** Mean accelerometers value **/
	    meanacc[0] = init_acc.row(0).mean();
	    meanacc[1] = init_acc.row(1).mean();
	    meanacc[2] = init_acc.row(2).mean();

	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"*** Computed mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<"\n";
	    std::cout<<"*** Computed gravity (acc): "<<meanacc.norm()<<"\n";
	    if (useInclinometers)
	    {
		std::cout<<"*** Computed gravity (incl): "<<meanincl.norm()<<"\n";
	    }
	    #endif
	    
	    if (useInclinometers)
	    {
		euler[0] = (double) asin((double)meanincl[1]/ (double)meanincl.norm()); // Roll
		euler[1] = (double) -atan(meanincl[0]/meanincl[2]); //Pitch
		euler[2] = - M_PI;
	    }
	    else
	    {
		euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
		euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
		euler[2] = - M_PI;
	    }
	    
	    /** Set the initial attitude when no initial IMU orientation is provided **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<< "******** Initial Attitude (STIM300 frame)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	    
	    /** Set the quaternion world to imu frame **/
	    q_world2imu = attitude;
	    
	    /** This attitude is in the IMU frame. It needs to be expressed in body with the help of the transformer **/
	    attitude = attitude * qtf; //! world2body = world2imu * imu2body
	    
	    #ifdef DEBUG_PRINTS
	    euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	    std::cout<< "******** Initial Attitude (after applying qtf)  *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	    
	    attitude.normalize();
	    
	    /** Check if there is initial pose connected **/
	    if (_pose_init.connected() && initPosition)
	    {
		/** Alternative method: Align the yaw from the poseInit Yaw **/
		attitude = Eigen::Quaternion <double>(Eigen::AngleAxisd(poseInit[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0], Eigen::Vector3d::UnitZ())) * attitude;
		
		attitude.normalize();
		
		/** Set the initial attitude quaternion of the filter **/
		mysckf.setAttitude (attitude);
		
		initAttitude = true;
		
	    }
	    else if (!_pose_init.connected() || !initPosition)
	    {
		/** Set the initial attitude quaternion of the filter **/
		mysckf.setAttitude (attitude);
		initAttitude = true;
	    }
    	    
	    /** Set the initial attitude to the rover rbs **/
	    if (initAttitude)
	    {
		/**Store the value as the initial one for the rbsBC **/
		rbsBC.orientation = attitude;
		rbsBC.angular_velocity.setZero();
		
		/** Assume very well know initial attitude **/
		rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
		rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
		/** Set the initial pose in the dead reckoning process **/
		drPose.setInitPose(rbsBC);
		
		/** Set the gravity to the filter to the mean computed **/
// 		mysckf.setGravity(meanacc.norm());
		
		/** Gravity error between theoretical gravity and estimated **/
		Eigen::Matrix <double,NUMAXIS,1> g_error;
		g_error << 0.00, 0.00, (mysckf.getGravity()-meanacc.norm());
		
		#ifdef DEBUG_PRINTS
		std::cout<< "G_error in world\n"<<g_error <<"\n";
		#endif
		
		/** Gravity error in IMU frame **/
		g_error = q_world2imu.inverse() * g_error;
		
		#ifdef DEBUG_PRINTS
		std::cout<< "G_error in imu\n"<<g_error <<"\n";
		#endif
		
		g_error << 0.00, 0.00, (mysckf.getGravity()-meanacc.norm());
		
		g_error = attitude.inverse() * g_error;
		
		#ifdef DEBUG_PRINTS
		std::cout<< "G_error in body\n"<<g_error <<"\n";
		#endif
		
		/** Update Bias offset with the estimated error theoretical and computed gravity **/
 		mysckf.setBiasOffset(_gbiasof.value(), _abiasof.value()-g_error);
		
		#ifdef DEBUG_PRINTS
		euler = mysckf.getEuler();
		std::cout<< "******** Initial Attitude *******"<<"\n";
		std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
		#endif
	    }
	    
	}
    }
    else
    {
	base::samples::IMUSensors imusample;
	
	/** A new sample arrived to the port**/
	
	#ifdef DEBUG_PRINTS
	std::cout<<"** Received IMU Samples **\n";
	std::cout<<"acc(imu_frame):\n"<<calibrated_sensors_sample.acc<<"\n";
	std::cout<<"acc(quat):\n"<<qtf * calibrated_sensors_sample.acc<<"\n";
	std::cout<<"acc(Rot):\n"<< tf.rotation() * calibrated_sensors_sample.acc<<"\n";
	std::cout<<"gyro(imu_frame):\n"<<calibrated_sensors_sample.gyro<<"\n";
	std::cout<<"mag(imu_frame):\n"<<calibrated_sensors_sample.mag<<"\n";
	#endif
	
	/** Convert the IMU values in the body orientation **/
	imusample.time = calibrated_sensors_sample.time;
	imusample.acc = qtf * calibrated_sensors_sample.acc;
	imusample.gyro = qtf * calibrated_sensors_sample.gyro;
	imusample.mag = qtf * calibrated_sensors_sample.mag;
	
	/** Push the IMU sensor into the buffer **/
	cbIMU.push_front(imusample);
	
	/** Set the flag of IMU values valid to true **/
	if (!imuValues && (cbIMU.size() == cbIMU.capacity()))
	    imuValues = true;
	
	counterIMUSamples++;
    }
}

void Task::hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample)
{
    /** A new sample arrived to the inport **/
    cbHbridges.push_front(hbridge_samples_sample);
    
    counterHbridgeSamples++;
    
    if (counterHbridgeSamples == numberHbridgeSamples)
    {
	hbridgeValues = true;
	counterHbridgeSamples = 0;
    }
    else
    {
	hbridgeValues = false;
    }
	
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** counterHbridgeSamples("<<counterHbridgeSamples<<") at ("<<hbridge_samples_sample.time.toMicroseconds()
	<<") received FR ("<<hbridge_samples_sample.states[0].positionExtern<<")**\n";
    #endif
       
    #ifdef DEBUG_PRINTS
    std::cout<<"hbridgeCounter ("<<counterHbridgeSamples<<") asguardCounter("<<counterAsguardStatusSamples<<") imuCounter("<<counterIMUSamples<<")\n";
    #endif
    
    /** Start the measurement and indirect filter if everything is alrigth **/
    if (initAttitude && initPosition && hbridgeValues && imuValues && asguardValues)
    {
	
	#ifdef DEBUG_PRINTS
	std::cout<<"********** ("<<hbridgeStatus[0].time.toMicroseconds()<<") **** OUT_IDX("<<this->filteridx<<")****\n";
	#endif
	
	/** Get the correct values from the input ports buffers **/
	this->getInputPortValues();
    
	/** Calculate the position and orientation of all asguard feets with the new encoders information **/
	this->calculateFootPoints();
	
	/** Select the Contact Point among the Foot Points **/
	this->selectContactPoints(contactPoints);
	
	/** Set the Contact Point in the KinematicModel **/
	wheelFL.setContactFootID(contactPoints[3]);
	wheelFR.setContactFootID(contactPoints[2]);
	wheelRR.setContactFootID(contactPoints[1]);
	wheelRL.setContactFootID(contactPoints[0]);
	
	#ifdef DEBUG_PRINTS
	std::cout<<"*********** BODY2CONTACTPOINT ***********\n";
	#endif
	
	/** Compute the wheel Contact Point **/
	wheelRL.Body2ContactPoint(hbridgeStatus[0].states[0].positionExtern, asguardStatus[0].asguardJointEncoder, contactAngle[0]);    
	wheelRR.Body2ContactPoint(hbridgeStatus[0].states[1].positionExtern, asguardStatus[0].asguardJointEncoder, contactAngle[1]);
	wheelFR.Body2ContactPoint(hbridgeStatus[0].states[2].positionExtern, asguardStatus[0].asguardJointEncoder, contactAngle[2]);
	wheelFL.Body2ContactPoint(hbridgeStatus[0].states[3].positionExtern, asguardStatus[0].asguardJointEncoder, contactAngle[3]);
	
	/** Compute the wheel Jacobians for the Selected Foot Point in Contact **/
	Eigen::Matrix< double, NUMAXIS , 1> slipvector = Eigen::Matrix< double, 3 , 1>::Zero();
	jacobRL = wheelRL.getWheelJacobian (slipvector);
	jacobRR = wheelRR.getWheelJacobian (slipvector);
	jacobFR = wheelFR.getWheelJacobian (slipvector);
	jacobFL = wheelFL.getWheelJacobian (slipvector);
	
	/** Calculate encoders joints velocities **/
	this->calculateEncodersVelocities();
	
	/** Composite Slip-Kinematics and Observation matrix for the SCKF **/
	this->compositeMotionJacobians();
	
	/** Inertial sensor information **/
	Eigen::Matrix<double, NUMAXIS, 1> angular_velocity = imuSamples[0].gyro;
	Eigen::Matrix<double, NUMAXIS, 1> acc = imuSamples[0].acc;
	Eigen::Matrix<double, NUMAXIS, 1> mag = imuSamples[0].mag;
	
	/** Substract Earth rotation from gyros **/
	Eigen::Quaternion <double> currentq = mysckf.getAttitude();
	Measurement::SubstractEarthRotation(&angular_velocity, &currentq, _latitude.value());
	
	#ifdef DEBUG_PRINTS
	mysckf.getEuler();
	std::cout<<"********** PREDICT ***** IN_IDX("<<this->filteridx<<") ***\n";
	#endif
	
	/** Filter: Predict the state of the filter **/
	mysckf.predict(angular_velocity, acc, propagation_delta_t);
	    
	#ifdef DEBUG_PRINTS
	std::cout<<"********** MOTION MODEL *****************\n";
	#endif
	
	/** Measurement: compute the motion model (nav. kinematics) **/
	mysckf.motionModel (Anav, Bnav, vjoints, propagation_delta_t);
	
	/** Increment in one the filter index **/
	this->filteridx++;
	
	/** Compare with the measurement number of iterations **/
	if (this->filteridx == this->measurementiterations)
	{
	    
	    /** Get the inertial values from the measurement **/
	    mysckf.measurement.getStepInertialValues (acc, angular_velocity);
	    
	    /** Using the incremental velocity as measurement **/
	    Eigen::Matrix<double,NUMAXIS,NUMAXIS> Hme;
	    Eigen::Matrix<double,NUMAXIS,NUMAXIS> Rme;
	    Eigen::Matrix<double,NUMAXIS,1> vel_error;
	    Hme.setIdentity();Rme.setZero();vel_error.setZero();
	    
	    /** Velocity error measurement **/
	    mysckf.velocityError (vel_error, Rme, update_delta_t);
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<< "[Measurement] Hme is of size "<<Hme.rows()<<"x"<<Hme.cols()<<"\n";
	    std::cout<< "[Measurement] Hme:\n"<<Hme<<"\n";
	    if (mysckf.measurement.getCurrentVeloModel()[0] > 0.5)
		std::cout<< "[Measurement] BIGGER THAN 0.5 at "<<hbridgeStatus[0].time.toMicroseconds()<<"\n";
	    #endif
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"********** UPDATE *****************************\n";
	    #endif
	    
	    /** Update the state of the filter **/
	    mysckf.update(Hme, Rme, vel_error, acc, mag, update_delta_t, false);

	    /** Compute Slip vector **/
	    Eigen::Matrix<double, localization::SLIP_VECTOR_SIZE, 1> roverslipvector;
	    Eigen::Matrix<double, localization::SLIP_VECTOR_SIZE, localization::SLIP_VECTOR_SIZE> roverslipvectorCov;
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"********** To Rover Slip Vector **********\n";
	    #endif
    // 	mysckf.computeSlipVector(Aslip, Bslip, roverslipvector, roverslipvectorCov, update_delta_t);
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"********** To Asguard Body **********\n";
	    #endif
	    
	    /** Body pose to asguard for the vizkit visualization **/
	    this->toAsguardBodyState();
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"********** To Envire Representation **********\n";
	    #endif
	    
	    /** Foot contact and slip vector to envire representation **/
	// 	this->toMLSGrid(roverslipvector, roverslipvectorCov);
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"********** To Deburg Ports **********\n";
	    #endif
	    
	    /** To Debug ports **/
	    this->toDebugPorts();
	    
	    /** Reset the state vector **/
	    mysckf.resetStateVector();
	    
	    /** Envire outport **/
	    this->sendEnvireEnvironment();
	    
	    /** Reset the filter index to Zero **/
	    this->filteridx = 0;
	    
	}
	
	/** Set the variables to perform the dead-reckoning **/
	Eigen::Matrix<double, NUMAXIS, 1> linvelo = mysckf.measurement.getCurrentVeloModel();//mysckf.getVelocity();
	Eigen::Quaternion <double> delta_q = mysckf.deltaQuaternion();
	Eigen::Matrix<double, NUMAXIS, NUMAXIS> covlinvelo = Eigen::Matrix<double, NUMAXIS, NUMAXIS>::Zero();
	Eigen::Matrix<double, NUMAXIS, NUMAXIS> covdelta_q = Eigen::Matrix<double, NUMAXIS, NUMAXIS>::Zero();
	
	/** Dead-reckoning and save into rbsBC **/
	rbsBC = drPose.updatePose (linvelo, delta_q, covlinvelo, covdelta_q, hbridgeStatus[0].time, propagation_delta_t);
	
	/** Write new pose to the output port **/
	_pose_samples_out.write(rbsBC);

	#ifdef DEBUG_PRINTS
	std::cout<<"********** END **********\n";
	#endif
	
    }

}
void Task::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
    /** A new sample arrived to the port **/
    cbAsguard.push_front(systemstate_samples_sample);
    
    /** Set the flag of Asguard Status values valid to true **/
    if (!asguardValues && (cbAsguard.size() == cbAsguard.capacity()))
	asguardValues = true;
    
    counterAsguardStatusSamples ++;
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** counterAsguardStatusSamples("<<counterAsguardStatusSamples<<") at ("<<systemstate_samples_sample.time.toMicroseconds()<< ")**\n";
    std::cout<<"** passive joint value: "<< systemstate_samples_sample.asguardJointEncoder<<"\n";
    #endif
}

void Task::torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for torque_estimated not implemented");
}

void Task::ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for ground_forces_estimated not implemented");
}

void Task::pose_initTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& pose_init_sample)
{
    
    poseInit.push_front(pose_init_sample);
    
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Rotation in quaternion form **/
    
    /** Get the transformation **/
    if (!_vicon2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (tf.rotation());
    
    #ifdef DEBUG_PRINTS
    std::cout<<"** received poseInit sample at("<<pose_init_sample.time.toMicroseconds()<<") **\n";
    #endif
    
    /** Apply the transformer pose offset **/
    poseInit[0].position += qtf * tf.translation();
    poseInit[0].orientation = poseInit[0].orientation * qtf;
//     poseInit.velocity = qtf * poseInit.velocity;
//     poseInit.cov_velocity = tf.rotation() * poseInit.cov_velocity;
//     poseInit.cov_angular_velocity = tf.rotation() * poseInit.cov_angular_velocity;
    
    if (!initPosition)
    {
	rbsBC.position = poseInit[0].position;
	rbsBC.velocity.setZero();
	
	/** Assume well known starting position **/
	rbsBC.cov_position = Eigen::Matrix <double, NUMAXIS , NUMAXIS>::Zero();
	rbsBC.cov_velocity = Eigen::Matrix <double, NUMAXIS , NUMAXIS>::Zero();
	
	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	euler[2] = poseInit[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	euler[1] = poseInit[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	euler[0] = poseInit[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
 	std::cout<<"** poseInit at ("<<poseInit[0].time.toMicroseconds()<< ")**\n";
	std::cout<<"** position offset\n"<<tf.translation()<<"\n";
	std::cout<<"** rotation offset\n"<<tf.rotation()<<"\n";
	std::cout<<"** position\n"<< poseInit[0].position<<"\n";
	std::cout<<"** Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
	#endif

	/** Initial attitude from IMU acceleration has been already calculated **/
	if (initAttitude)
	{
	    Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude = mysckf.getAttitude(); /** Initial attitude in case no port in orientation is connected **/
	    
	    /** Get the initial Yaw from the initialPose **/
	    euler[2] = poseInit[0].orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	    euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	    euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	    
	    /** Set the initial attitude with the Yaw provided from the initial pose **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	    
	    /** Set the initial attitude quaternion of the filter **/
	    mysckf.setAttitude (attitude);
	    
	    /**Store the value as the initial one for the rbsBC **/
	    rbsBC.orientation = attitude;
	    
	    #ifdef DEBUG_PRINTS
	    euler = mysckf.getEuler();
	    std::cout<< "******** Initial Attitude in poseInit *******"<<"\n";
	    std::cout<< "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]*R2D<<"\n";
	    #endif
	}
	
	/** Initial angular velocity **/
	rbsBC.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
		
	/** Set the initial pose in the uncertainty variable **/
	drPose.setInitPose(rbsBC);
	
	initPosition = true;
    }
    
    poseInitValues = true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    double delta_bandwidth = std::numeric_limits<double>::quiet_NaN();/** Delta (bandwidth) time of inertial sensors **/
    double theoretical_g; /** Ideal gravity value **/
    int buffer_size; /** Buffer size of the propriocesing measurement **/
    Eigen::Matrix< double, Eigen::Dynamic,1> x_0; /** Initial vector state **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra_predict; /** Measurement noise convariance matrix for acc (predict) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra_update; /** Measurement noise convariance matrix for acc (update) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rat; /** Measurement noise convariance matrix for attitude correction (gravity vector noise) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rv; /** Measurement noise convariance matrix for velocity (accelerometers integration) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /** Measurement noise convariance matrix for gyros */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /** Measurement noise convariance matrix for mag */    
    Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic> P_0; /** Initial covariance matrix **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qbg; /** Process noise matrix of gyros bias (bias instability) **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qba; /** Process noise matric of acc bias (bias instability) **/
    Eigen::Matrix <double,localization::ENCODERS_VECTOR_SIZE,localization::ENCODERS_VECTOR_SIZE> Ren; /** Measurement noise of encoders velocity **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rangvelo; /** Measurement noise for angular velocity in the measurement */
    
    if (! TaskBase::configureHook())
        return false;
    
    /** Set the initial BC to the Geographic frame **/
    rbsBC.invalidate();
    rbsBC.sourceFrame = "Body Frame";
    rbsBC.targetFrame = "Geographic_Frame (North-West-Up)";
        
    if (!_pose_init.connected())
    {
	/** set zero position **/
	rbsBC.position.setZero();
	rbsBC.velocity.setZero();
	rbsBC.angular_velocity.setZero();
	
	/** Assume very well know initial attitude **/
	rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_angular_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	/** Assume well known starting position **/
	rbsBC.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
	rbsBC.cov_velocity = Eigen::Matrix <double, 3 , 3>::Zero();
	
	initPosition = true;
    }
    
    /** Get the flag to use or not the inclinometers fopr initial leveling **/
    useInclinometers = _use_inclinometers_leveling.value();
    
    /** Set the initial number of samples for the attitude **/
    this->initLeveling = static_cast<int>(15.0/_calibrated_sensors_period.value());
    
    /** Init size for initial acceleration leveling **/
    init_acc.resize (NUMAXIS, this->initLeveling);
    
    /** Init size for initial acceleration leveling **/
    init_incl.resize (NUMAXIS, this->initLeveling);
    
    /** Set the counters to Zero **/
    counterHbridgeSamples = 0;
    counterAsguardStatusSamples = 0;
    counterTorqueSamples = 0;
    counterForceSamples = 0;
    counterIMUSamples = 0;
    counterPose = 0;
    
    /** Set the number of samples between each sensor input (if there are not comming at the same sampling rate) */
    if (_propagation_frequency.value() != 0.00)
    {
	numberIMUSamples = (1.0/_calibrated_sensors_period.value())/_propagation_frequency.value();
	numberHbridgeSamples = (1.0/_hbridge_samples_period.value())/_propagation_frequency.value();
	numberAsguardStatusSamples = (1.0/_systemstate_samples_period.value())/_propagation_frequency.value();
	numberForceSamples = (1.0/_ground_forces_estimated_period.value())/_propagation_frequency.value();
	numberTorqueSamples = (1.0/_torque_estimated_period.value())/_propagation_frequency.value();
	numberPose = (1.0/_pose_init_period.value())/_propagation_frequency.value();
    }
    
    #ifdef DEBUG_PRINTS
    std::cout<<"[Task configure] cbHbridges has capacity "<<cbHbridges.capacity()<<" and size "<<cbHbridges.size()<<"\n";
    std::cout<<"[Task configure] cbAsguard has capacity "<<cbAsguard.capacity()<<" and size "<<cbAsguard.size()<<"\n";
    std::cout<<"[Task configure] cbIMU has capacity "<<cbIMU.capacity()<<" and size "<<cbIMU.size()<<"\n";
    #endif
    
    /** Set the capacity of the circular_buffer according to the sampling rate **/
    cbHbridges.set_capacity(numberHbridgeSamples);
    cbAsguard.set_capacity(numberAsguardStatusSamples);
    cbIMU.set_capacity(numberIMUSamples);
    
    for(register unsigned int i=0;i<cbHbridges.size();i++)
    {
	cbHbridges[i].resize(NUMBER_WHEELS);
    }
    
    
    #ifdef DEBUG_PRINTS
    std::cout<<"[Task configure] cbHbridges has capacity "<<cbHbridges.capacity()<<" and size "<<cbHbridges.size()<<"\n";
    std::cout<<"[Task configure] cbAsguard has capacity "<<cbAsguard.capacity()<<" and size "<<cbAsguard.size()<<"\n";
    std::cout<<"[Task configure] cbIMU has capacity "<<cbIMU.capacity()<<" and size "<<cbIMU.size()<<"\n";
    #endif
    
    /** Initialize the samples for the filtered buffer hbridge values **/
    for(register unsigned int i=0;i<hbridgeStatus.size();i++)
    {
	/** Sizing hbridgeStatus **/
	hbridgeStatus[i].resize(NUMBER_WHEELS);
    
	/** Set to a NaN index **/
	hbridgeStatus[i].index = base::NaN<unsigned int>();
    }
    
    /** Initialize the samples for the filtered buffer asguardStatus values **/
    for(register unsigned int i=0;i<asguardStatus.size();i++)
    {
	/** Set to NaN passiveJoint **/
	asguardStatus[i].asguardJointEncoder = base::NaN<double>();
    }
    
    /** Initialize the samples for the filtered buffer imuSamples values **/
    for(register unsigned int i=0;i<imuSamples.size();i++)
    {
	/** IMU Samples **/
	imuSamples[i].acc[0] = base::NaN<double>();
	imuSamples[i].acc[1] = base::NaN<double>();
	imuSamples[i].acc[2] = base::NaN<double>();
	imuSamples[i].gyro = imuSamples[0].acc;
	imuSamples[i].mag = imuSamples[0].acc;
    }
    
    /** Initialize the samples for the filtered buffer poseInit values **/
    for(register unsigned int i=0;i<poseInit.size();i++)
    {
	/** Pose Init **/
	poseInit[i].invalidate();
    }
    
    /** Initialize the Feet per wheel **/
    base::samples::RigidBodyState initFoot;
    initFoot.invalidate();
    
    for(register unsigned int i=0;i<FEET_PER_WHEEL;i++)
    {
	rbsCiFL2body.push_front(initFoot);
	rbsCiFR2body.push_front(initFoot);
	rbsCiRL2body.push_front(initFoot);
	rbsCiRR2body.push_front(initFoot);
    }
    
    
    #ifdef DEBUG_PRINTS
    std::cout<<"[Task configure] hbridgeStatus has capacity "<<hbridgeStatus.capacity()<<" and size "<<hbridgeStatus.size()<<"\n";
    std::cout<<"[Task configure] asguardStatus has capacity "<<asguardStatus.capacity()<<" and size "<<asguardStatus.size()<<"\n";
    std::cout<<"[Task configure] imuSamples has capacity "<<imuSamples.capacity()<<" and size "<<imuSamples.size()<<"\n";
    std::cout<<"[Task configure] poseInit has capacity "<<poseInit.capacity()<<" and size "<<poseInit.size()<<"\n";
    #endif
    
    
    /** Propagation delta time step in seconds **/
    if (_propagation_frequency.value() != 0.00)
	propagation_delta_t = 1.0/_propagation_frequency.value();    
    
    /** Indirect filter update delta time step in seconds **/
    if (_update_frequency.value() != 0.00)
	update_delta_t = 1.0/_update_frequency.value(); 
    
    /** Set the Indirect Filter Bandwidth delta time to the sensors bandwidth **/
    if (_sensors_bandwidth.value() != 0.00)
	delta_bandwidth = 1.0/_sensors_bandwidth.value();
    
    /** Buffer size - difference between frequency in proprioceptive measurement and filter **/
    if (update_delta_t > propagation_delta_t)
	buffer_size = static_cast<int> (update_delta_t/propagation_delta_t); 
    else
    {
	update_delta_t = propagation_delta_t;
	buffer_size = 1;
    }
    
    /** Set in the global variable **/
    this->measurementiterations = buffer_size;
    
    
    /** Check if the indirect filter (propagation) is running at lower frequency
     *  than inertial sensor bandwidth. The Filter "bandwidth" cannot
     *  be higher than the filter sampling rate **/
    if (delta_bandwidth < propagation_delta_t)
	delta_bandwidth = propagation_delta_t;
    
    /** Fill the filter initial matrices **/
    /** Accelerometers covariance matrix **/
    Ra_predict = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Ra_predict(0,0) = pow(_accrw.get()[0]/sqrt(delta_bandwidth),2);
    Ra_predict(1,1) = pow(_accrw.get()[1]/sqrt(delta_bandwidth),2);
    Ra_predict(2,2) = pow(_accrw.get()[2]/sqrt(delta_bandwidth),2);
    
    if (update_delta_t > delta_bandwidth)
    {
	/** Accelerometers covariance matrix **/
	Ra_update = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
	Ra_update(0,0) = pow(_accrw.get()[0]/sqrt(update_delta_t),2);
	Ra_update(1,1) = pow(_accrw.get()[1]/sqrt(update_delta_t),2);
	Ra_update(2,2) = pow(_accrw.get()[2]/sqrt(update_delta_t),2);
    }
    else
    {
	/** Accelerometers covariance matrix **/
	Ra_update = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
	Ra_update(0,0) = pow(_accrw.get()[0]/sqrt(delta_bandwidth),2);
	Ra_update(1,1) = pow(_accrw.get()[1]/sqrt(delta_bandwidth),2);
	Ra_update(2,2) = pow(_accrw.get()[2]/sqrt(delta_bandwidth),2);
    }
    
    /** Gravity vector covariance matrix **/
    Rat = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rat(0,0) = NUMAXIS * Ra_update(0,0);//0.0054785914701378034;
    Rat(1,1) = NUMAXIS * Ra_update(1,1);//0.0061094546837916494;
    Rat(2,2) = NUMAXIS * Ra_update(2,2);//0.0063186020143245212;
    
    /** Gyroscopes covariance matrix **/
    Rg = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rg(0,0) = pow(_gyrorw.get()[0]/sqrt(delta_bandwidth),2);
    Rg(1,1) = pow(_gyrorw.get()[1]/sqrt(delta_bandwidth),2);
    Rg(2,2) = pow(_gyrorw.get()[2]/sqrt(delta_bandwidth),2);

    /** Magnetometers covariance matrix **/
    Rm = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rm(0,0) = pow(_magrw.get()[0]/sqrt(delta_bandwidth),2);
    Rm(1,1) = pow(_magrw.get()[1]/sqrt(delta_bandwidth),2);
    Rm(2,2) = pow(_magrw.get()[2]/sqrt(delta_bandwidth),2);
    
    /** Encoders velocity errors **/
    Ren = 0.000001 * Eigen::Matrix <double,localization::ENCODERS_VECTOR_SIZE, localization::ENCODERS_VECTOR_SIZE>::Identity();
    
    /** Angular Velocity error for the proprioceptive measurements **/
    Rangvelo = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rangvelo(0,0) = pow(_gyrorw.get()[0]/sqrt(delta_bandwidth),2);
    Rangvelo(1,1) = pow(_gyrorw.get()[1]/sqrt(delta_bandwidth),2);
    Rangvelo(2,2) = pow(_gyrorw.get()[2]/sqrt(delta_bandwidth),2);
    
    /** Initial error covariance **/
    P_0.resize(Sckf::X_STATE_VECTOR_SIZE, Sckf::X_STATE_VECTOR_SIZE);
    P_0 = Eigen::Matrix <double,Sckf::X_STATE_VECTOR_SIZE,Sckf::X_STATE_VECTOR_SIZE>::Zero();
    P_0.block <2*NUMAXIS, 2*NUMAXIS> (0, 0) = 1e-06 * Eigen::Matrix <double,2*NUMAXIS,2*NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (2*NUMAXIS,2*NUMAXIS) = 1e-06 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (3*NUMAXIS,3*NUMAXIS) = 1e-10 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (4*NUMAXIS,4*NUMAXIS) = 1e-10 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Process noise matrices **/
    /** Gyroscopes bias instability **/    
    Qbg.setZero();
    Qbg(0,0) = pow(_gbiasins.get()[0],2);
    Qbg(1,1) = pow(_gbiasins.get()[1],2);
    Qbg(2,2) = pow(_gbiasins.get()[2],2);
    
    /** Accelerometers bias instability **/    
    Qba.setZero();
    Qba(0,0) = pow(_abiasins.get()[0],2);
    Qba(1,1) = pow(_abiasins.get()[1],2);
    Qba(2,2) = pow(_abiasins.get()[2],2);
        
    /** Gravitational value according to the location **/
    theoretical_g = Measurement::GravityModel (_latitude.value(), _altitude.value());
    
    /** Initialization of the measurement generation **/
    mysckf.measurement.Init(Ren, Rangvelo, _q_weight_distribution.value(), buffer_size);
    
    /** Initialization of the filter **/
    mysckf.Init(P_0, Rg, Qbg, Qba, Ra_predict, Ra_update, Rat, Rm, theoretical_g, (double)_dip_angle.value());
     
    /** Initialization set the vector state and bias offset to zero but they can be changed here **/
    x_0.resize(Sckf::X_STATE_VECTOR_SIZE,1);
    x_0 = Eigen::Matrix<double,Sckf::X_STATE_VECTOR_SIZE,1>::Zero();
    mysckf.setStatex(x_0);
    mysckf.setBiasOffset(_gbiasof.value(), _abiasof.value());
    
    /** Info and Warnings about the Task **/
    if (_calibrated_sensors.connected())
    {
	RTT::log(RTT::Warning) << "IMU Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_hbridge_samples.connected())
    {
	RTT::log(RTT::Warning) << "Hbridge Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Hbridge samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_systemstate_samples.connected())
    {
	RTT::log(RTT::Warning) << "System State Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "System State samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_torque_estimated.connected())
    {
	RTT::log(RTT::Warning) << "Wheel Torque estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel Torque estimation samples NO connected." << RTT::endlog();
    }
    
    if (_ground_forces_estimated.connected())
    {
	RTT::log(RTT::Warning) << "Wheel ground force estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel ground force estimation samples NO connected." << RTT::endlog();
    }
    
    if (_pose_init.connected())
    {
	RTT::log(RTT::Warning) << "Initial pose connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Initial orientation NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "Pitch and Roll are taken from accelerometers assuming static body at Initial phase of this task." << RTT::endlog();
    }
    
    RTT::log(RTT::Warning)<<"[Info] Frequency of IMU samples[Hertz]: "<<(1.0/_calibrated_sensors_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Hbridge Status[Hertz]: "<<(1.0/_hbridge_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Asguard Status[Hertz]: "<<(1.0/_systemstate_samples_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Torque Estimator[Hertz]: "<<(1.0/_torque_estimated_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Frequency of Ground Force Estimator[Hertz]: "<<(1.0/_ground_forces_estimated_period.value())<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Predict step running at Frequency[Hertz]: "<<_propagation_frequency.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Update step running at Frequency[Hertz]: "<<_update_frequency.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberIMUSamples: "<<numberIMUSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberHbridgeSamples: "<<numberHbridgeSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberAsguardStatusSamples: "<<numberAsguardStatusSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberForceSamples: "<<numberForceSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] numberTorqueSamples: "<<numberTorqueSamples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Proprioceptive measurements array size: "<<buffer_size<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[Info] Proprioceptive measurements iterations: "<<this->measurementiterations<<RTT::endlog();
    
    
    /** envire **/
    mpSlip = new envire::MLSGrid(_grid_resolution.get(), _grid_resolution.get(), _patch_size.get(), _patch_size.get(),
            _grid_center_x.get(), _grid_center_y.get());
   
    mEnv.attachItem(mpSlip);
   
    /** Create envire components. **/
    envire::FrameNode* p_fn = NULL;
   
    p_fn = new envire::FrameNode();
    mEnv.getRootNode()->addChild(p_fn);
    mpSlip->setFrameNode(p_fn);
    mpSlip->setHasCellColor(true);
    mpSlip->initIndex();
    
    /** Emitter **/
    mEmitter = new envire::OrocosEmitter(_envire_environment_out);
    mEmitter->useContextUpdates(&mEnv); // Without this line the parents are not set(?)
    mEmitter->useEventQueue( true );
    mEmitter->attach(&mEnv);
    
//     /** Set a flat surface **/    
//     base::Vector3d color;
//     color<<0.0, 1.0, 0.0;
//     
//     for(register int i=0; i<100; i++ )
//     {
// 	for(register int j=0; j<100; j++ )
// 	{
// 	    double h = 0.0;
// 	    envire::MLSGrid::SurfacePatch p(h, 0.05);
// 	    if (i<50 || j<50)
// 		p.setColor(color);
// 	    else
// 	    {
// 		color <<1.0, 0.0, 0.0;
// 		p.setColor(color);
// 	    }
// 		
// 	    
// 	    mpSlip->insertTail( i, j, p);
// 	}
//     }
//     mpSlip->itemModified();
    
    /** World to MLS transformation **/
    base::Vector3d trans(_grid_center_x.get(), _grid_center_y.get(), 0.00);
    world2mls =  Eigen::Affine3d::Identity();
    world2mls.translation() = trans;
    
    /** Number of cells of the foot print **/
    numberCellsFootx = ceil(_foot_size_x.get()/_patch_size.get());
    numberCellsFooty = ceil(_foot_size_y.get()/_patch_size.get());
    #ifdef DEBUG_PRINTS
    std::cout<< "[Task configure] world2mls:\n"<<world2mls.matrix()<<"\n";
    std::cout<< "[Task configure] numberCellsFootx: "<<numberCellsFootx<<" numberCellsFooty: "<<numberCellsFooty<<"\n";
    #endif
    

    /** Envire outport **/
    this->sendEnvireEnvironment();


    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::getInputPortValues()
{
    unsigned int cbHbridgesize = cbHbridges.size();
    unsigned int cbAsguardsize = cbAsguard.size();
    unsigned int cbIMUsize = cbIMU.size();
    
    /** Local variable of the ports **/
    base::actuators::Status hbridge;
    sysmon::SystemStatus asguardS;
    base::samples::IMUSensors imu;
    
    /** sizing hbridge **/
    hbridge.resize(NUMBER_WHEELS);
    
    #ifdef DEBUG_PRINTS
    std::cout<<"[GetInportValue] cbHbridges has capacity "<<cbHbridges.capacity()<<" and size "<<cbHbridges.size()<<"\n";
    std::cout<<"[GetInportValue] cbAsguard has capacity "<<cbAsguard.capacity()<<" and size "<<cbAsguard.size()<<"\n";
    std::cout<<"[GetInportValue] cbIMU has capacity "<<cbIMU.capacity()<<" and size "<<cbIMU.size()<<"\n";
    #endif
    
    /** ********* **/
    /** Hbridges **/
    /** ********* **/
    for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
    {
	hbridge.states[i].current = 0.0;
	hbridge.states[i].position = 0.0;
	hbridge.states[i].positionExtern = 0.0;
	hbridge.states[i].pwm = 0.0;
    }
	
    /** Process the buffer **/
    for (register unsigned int i = 0; i<cbHbridgesize; i++)
    {
	for (register unsigned int j = 0; j<asguard::NUMBER_OF_WHEELS; j++)
	{
	    hbridge.states[j].current += cbHbridges[i].states[j].current;
	    hbridge.states[j].pwm += cbHbridges[i].states[j].pwm;
	}
    }
    
    /** Set the time **/
    hbridge.time = (cbHbridges[cbHbridgesize-1].time + cbHbridges[0].time)/2.0;
    
    hbridge.index = cbHbridges[0].index;
    
    for (register unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
    {
	hbridge.states[i].current /= cbHbridgesize; //the mean current in mA
	hbridge.states[i].position = cbHbridges[0].states[i].position;
	hbridge.states[i].positionExtern = cbHbridges[0].states[i].positionExtern;
	hbridge.states[i].pwm /= cbHbridgesize; //the mean PWM signal duty cicle [-1,1]
    }
    
    /** Push the result in the buffer **/
    hbridgeStatus.push_front(hbridge);
	
    /** ************** **/
    /** Asguard status **/
    /** ************** **/
    asguardS.asguardVoltage = 0.00;
    
    /** Process the buffer **/
    for (register unsigned int i=0; i<cbAsguardsize; i++)
    {
	asguardS.asguardVoltage += cbAsguard[i].asguardVoltage;
    }
    
    asguardS.asguardJointEncoder = cbAsguard[0].asguardJointEncoder;
    asguardS.systemState = cbAsguard[0].systemState;
    asguardS.packetsPerSec = cbAsguard[0].packetsPerSec;
    asguardS.controlPacketsPerSec = cbAsguard[0].controlPacketsPerSec;
    
    /** Set the time **/
    asguardS.time = (cbAsguard[cbAsguardsize-1].time + cbAsguard[0].time)/2.0;
    
    /** Set the voltage **/
    asguardS.asguardVoltage /= cbAsguardsize; //the mean voltage
    
    /** Push the result into the buffer **/
    asguardStatus.push_front(asguardS);
    
    /** ************ **/
    /** IMU samples **/
    /** ************ **/
    imu.acc.setZero();
    imu.gyro.setZero();
    imu.mag.setZero();
    
    /** Process the buffer **/
    for (register unsigned int i=0; i<cbIMUsize; i++)
    {
	imu.acc += cbIMU[i].acc;
	imu.gyro += cbIMU[i].gyro;
	imu.mag += cbIMU[i].mag;
    }
    
    /** Set the time **/
    imu.time = (cbIMU[cbIMUsize-1].time + cbIMU[0].time)/2.0;
    
    /** Set the mean of this time interval **/
    imu.acc /= cbIMUsize;
    imu.gyro /= cbIMUsize;
    imu.mag /= cbIMUsize;
    
    /** Push the result into the buffer **/
    imuSamples.push_front(imu);
    
    /** Increase the index = **/
    if (samplesidx < DEFAULT_CIRCULAR_BUFFER_SIZE)
	samplesidx++;
    
    /** Set all counters to zero **/
    counterHbridgeSamples = 0;
    counterAsguardStatusSamples = 0;
    counterIMUSamples = 0;
    counterTorqueSamples = 0;
    counterForceSamples = 0;
    
    return;
}

void Task::calculateFootPoints()
{
    /** For the Wheel 0 Rear Left **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	wheelRL.Body2FootPoint (i, hbridgeStatus[0].states[0].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
	rbsCiRL2body[i] = wheelRL.getBody2FootPoint();
    }
    
//     wheelRL.Body2ContactPoint (hbridgeStatus[0].states[0].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
//     rbsC2RL2body = wheelRL.getBody2ContactPoint();
    
    /** For the Wheel 1 Rear Right **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	wheelRR.Body2FootPoint (i, hbridgeStatus[0].states[1].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
	rbsCiRR2body[i] = wheelRR.getBody2FootPoint();
    }
    
//     wheelRR.Body2ContactPoint (hbridgeStatus[0].states[1].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
//     rbsC2RR2body = wheelRR.getBody2ContactPoint();
    
    /** For the Wheel 2 Forward Right **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	wheelFR.Body2FootPoint (i, hbridgeStatus[0].states[2].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
	rbsCiFR2body[i] = wheelFR.getBody2FootPoint();
    }
    	
    /** For the Wheel 3 Forward Left **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	wheelFL.Body2FootPoint (i, hbridgeStatus[0].states[3].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
	rbsCiFL2body[i] = wheelFL.getBody2FootPoint();
    }
        
//     wheelFL.Body2ContactPoint (hbridgeStatus[0].states[3].positionExtern, asguardStatus[0].asguardJointEncoder, 0.00);
//     rbsC2FL2body = wheelFL.getBody2ContactPoint();
    
    
    return;

}

void Task::selectContactPoints(std::vector<int> &contactPoints)
{
    std::vector<double> footFR;
    std::vector<double> footFL;
    std::vector<double> footRR;
    std::vector<double> footRL;

    /** For the FL wheel **/
    footFL.resize(5);
    
    /** Z-coordinate of the foot **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	footFL[i] = rbsCiFL2body[i].position(2);
    }
    
    contactPoints[3] = std::distance(footFL.begin(), std::min_element(footFL.begin(), footFL.end()));
    
    #ifdef DEBUG_PRINTS
    std::cout<<"FL position: "<< rbsCiFL2body[0].position(2)<<"\n";
    std::cout<<"footFL: "<<footFL[0]<<" "<<footFL[1]<<" "<<footFL[2]<<" "<<footFL[3]<<" "<<footFL[4]<<"\n";
    std::cout<<"contactPoints[3]: "<< contactPoints[3]<<"\n";
    #endif
    
    /** For the FR wheel **/
    footFR.resize(5);
    
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	footFR[i] = rbsCiFR2body[i].position(2);
    }
    
    contactPoints[2] = std::distance(footFR.begin(), std::min_element(footFR.begin(), footFR.end()));
    
    /** For the RR wheel **/
    footRR.resize(5);
    
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	footRR[i] = rbsCiRR2body[i].position(2);
    }

    contactPoints[1] = std::distance(footRR.begin(), std::min_element(footRR.begin(), footRR.end()));
    
    /** For the RL wheel **/
    footRL.resize(5);
    
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	footRL[i] = rbsCiRL2body[i].position(2);
    }
    
    contactPoints[0] = std::distance(footRL.begin(), std::min_element(footRL.begin(), footRL.end()));
    
    return;
    
}

void Task::calculateEncodersVelocities()
{
    base::Time hbridgeDelta_t = hbridgeStatus[0].time - hbridgeStatus[1].time;
    base::Time passiveJointDelta_t = asguardStatus[0].time - asguardStatus[1].time;
    base::Time imuDelta_t = imuSamples[0].time - imuSamples[1].time;
    Eigen::Matrix<double, Eigen::Dynamic, 1> passiveJoint, motorsJoint;
    
    
    /** At least two values to perform the derivative **/
    if (samplesidx < 2)
    {
	vjoints.setZero();
    }
    else
    {
	/** Set the correct size for the derivative data vectors **/
	passiveJoint.resize(samplesidx,1);
	motorsJoint.resize(samplesidx,1);
	
	#ifdef DEBUG_PRINTS
	std::cout<<"samplesidx"<<samplesidx<<"\n";
	std::cout<<"Timestamp New: "<< asguardStatus[0].time.toMicroseconds() <<" Timestamp Prev: "<<asguardStatus[1].time.toMicroseconds()<<"\n";
	std::cout<<"Delta time(passiveJoint): "<< passiveJointDelta_t.toSeconds()<<"\n";
	std::cout<<"Timestamp New: "<< imuSamples[0].time.toMicroseconds() <<" Timestamp Prev: "<<imuSamples[1].time.toMicroseconds()<<"\n";
	std::cout<<"Delta time(IMU): "<< imuDelta_t.toSeconds()<<"\n";
	std::cout<<"New: "<< asguardStatus[0].asguardJointEncoder <<" Prev: "<<asguardStatus[1].asguardJointEncoder<<"\n";
	#endif
	
	passiveJoint.setZero(); //Set to zero

	/** Fill the derivative vector **/
	for (register int i=0; i<samplesidx; ++i)
	{
	    passiveJoint[i] = asguardStatus[i].asguardJointEncoder; 
	}
	
	#ifdef DEBUG_PRINTS
	std::cout<<"PassiveJoint old velocity: "<<(asguardStatus[0].asguardJointEncoder - asguardStatus[1].asguardJointEncoder)/propagation_delta_t<<"\n";
	#endif
	
	
	/** Passive joint velocity **/
	vjoints(0) = Measurement::finiteDifference (passiveJoint, propagation_delta_t); //passive joints speed
	
	#ifdef DEBUG_PRINTS
	std::cout<<"PassiveJoint new velocity: "<<vjoints(0)<<"\n";
	#endif
	
	/** Velocities for the vector **/
	for (int i = 1; i< (NUMBER_WHEELS + 1); i++)
	{
	    #ifdef DEBUG_PRINTS
	    std::cout<<"Timestamp New: "<< hbridgeStatus[0].time.toMicroseconds() <<" Timestamp Prev: "<<hbridgeStatus[1].time.toMicroseconds()<<"\n";
	    std::cout<<"Delta time: "<< hbridgeDelta_t.toSeconds()<<"\n";
	    std::cout<<"New: "<< hbridgeStatus[0].states[i-1].positionExtern <<" Prev: "<<hbridgeStatus[1].states[i-1].positionExtern<<"\n";
	    #endif
	    
	    motorsJoint.setZero(); //Set to zero
	    
	    /** Fill the derivative vector **/
	    for (register int j=0; j<samplesidx; ++j)
	    {
		motorsJoint[j] = hbridgeStatus[j].states[i-1].positionExtern; 
	    }
	    
	    if ((i==4)&&((motorsJoint[0] < 1.25) && (motorsJoint[0]>1.02)))
		std::cout << "[Difference] Region!!\n";
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"MotorJoint old velocity: "<<(hbridgeStatus[0].states[i-1].positionExtern - hbridgeStatus[1].states[i-1].positionExtern)/propagation_delta_t<<"\n";
	    #endif
	    
	    /** Motor joint velocity **/
	    vjoints(i) = Measurement::finiteDifference(motorsJoint, propagation_delta_t); //wheel rotation speed
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"MotorJoint new velocity: "<<vjoints(i)<<"\n";
	    #endif

	}
	
	_angular_position.write(hbridgeStatus[0].states[3].positionExtern);
	_angular_rate.write(vjoints(3));
	_angular_rate_old.write((hbridgeStatus[0].states[3].positionExtern - hbridgeStatus[1].states[3].positionExtern)/propagation_delta_t);
    }
    
    #ifdef DEBUG_PRINTS
    std::cout<<"Encoders velocities:\n"<<vjoints<<"\n";
    #endif

    return;
}

void Task::compositeMotionJacobians()
{
    /** Set the proper size of the matrices **/
    E.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS);
    J.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE + localization::SLIP_VECTOR_SIZE + localization::NUMBER_OF_WHEELS);
    
    Anav.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS+2*localization::NUMBER_OF_WHEELS);
    Bnav.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS+localization::ENCODERS_VECTOR_SIZE);
    Aslip.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE);
    Bslip.resize(localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS + ENCODERS_VECTOR_SIZE + NUMBER_OF_WHEELS);
    J.setZero(); Anav.setZero(); Bnav.setZero(); Aslip.setZero(); Bslip.setZero();
    
    /** Compute the composite rover equation matrix E **/
    E.block<2*NUMAXIS, 2*NUMAXIS>(0,0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*NUMAXIS, 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(3*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    
    #ifdef DEBUG_PRINTS
    std::cout<< "Composite matrices\n";
    std::cout<< "E is of size "<<E.rows()<<"x"<<E.cols()<<"\n";
    std::cout << "The E matrix \n" << E << std::endl;
    #endif
    
    /** Compute the rover Jacobian matrix **/
    
    /** Twist passive joint **/
    J.col(0).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(1);
    J.col(0).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(1);
    
    /** Motor encoders **/
    J.col(1).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(0);
    
    J.col(2).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(0);
        
    J.col(3).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(0);
    
    J.col(4).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(0);
    
    /** Slip vector x for RL wheel **/
    J.col(5).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(2);
    
    /** Slip vector y for RL wheel **/
    J.col(6).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(3);
    
    /** Slip vector z for RL wheel **/
    J.col(7).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(4);
    
    /** Slip vector x for RR wheel **/
    J.col(8).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(2);
    
    /** Slip vector y for RR wheel **/
    J.col(9).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(3);
    
    /** Slip vector z for RR wheel **/
    J.col(10).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(4);
    
    /** Slip vector x for FR wheel **/
    J.col(11).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(1);
    
    /** Slip vector y for FR wheel **/
    J.col(12).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(2);
    
    /** Slip vector z for FR wheel **/
    J.col(13).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(3);
    
    /** Slip vector x for FL wheel **/
    J.col(14).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(1);
    
    /** Slip vector y for FL wheel **/
    J.col(15).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(2);
    
    /** Slip vector z for FL wheel **/
    J.col(16).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(3);
    
    /** Contact angle for RL wheel **/
    J.col(17).block<2*NUMAXIS,1>(0, 0) = jacobRL.col(5);
    
    /** Contact angle for RR wheel **/
    J.col(18).block<2*NUMAXIS,1>(2*NUMAXIS, 0) = jacobRR.col(5);
    
    /** Contact angle for FR wheel **/
    J.col(19).block<2*NUMAXIS,1>(2*(2*NUMAXIS), 0) = jacobFR.col(4);
    
    /** Contact angle for FL wheel **/
    J.col(20).block<2*NUMAXIS,1>(3*(2*NUMAXIS), 0) = jacobFL.col(4);
    
    #ifdef DEBUG_PRINTS
    std::cout<< "J is of size "<<J.rows()<<"x"<<J.cols()<<"\n";
    std::cout << "The J matrix \n" << J << std::endl;
    #endif
    
    
    /** Form the matrices for the Navigation Kinematics **/
    Anav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS>(0,0) = E.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,0);
    Anav.col(3) = -J.col(7); Anav.col(4) = -J.col(10); Anav.col(5) = -J.col(13); Anav.col(6) = -J.col(16);
    Anav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS>(0,NUMAXIS+localization::NUMBER_OF_WHEELS) =
	-J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS>(0,localization::ENCODERS_VECTOR_SIZE+localization::SLIP_VECTOR_SIZE);
    Bnav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,0) = -E.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), NUMAXIS> (0,NUMAXIS);
    Bnav.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,NUMAXIS) = J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,0);
    
    /** Form the matrix for the slip kinematics **/
    Aslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE>(0,0) =
	-J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::SLIP_VECTOR_SIZE>(0,localization::ENCODERS_VECTOR_SIZE);
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS> (0,0) = -E;
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,2*NUMAXIS) =
	J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::ENCODERS_VECTOR_SIZE> (0,0);
    Bslip.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS> (0,(2*NUMAXIS)+localization::ENCODERS_VECTOR_SIZE) =
	J.block<localization::NUMBER_OF_WHEELS*(2*NUMAXIS), localization::NUMBER_OF_WHEELS> (0,localization::ENCODERS_VECTOR_SIZE+localization::SLIP_VECTOR_SIZE);


    #ifdef DEBUG_PRINTS
    std::cout<< "Anav is of size "<<Anav.rows()<<"x"<<Anav.cols()<<"\n";
    std::cout<< "The Anav matrix \n" << Anav << std::endl;
    std::cout<< "Bnav is of size "<<Bnav.rows()<<"x"<<Bnav.cols()<<"\n";
    std::cout<< "The Bnav matrix \n" << Bnav << std::endl;
    std::cout<< "Aslip is of size "<<Aslip.rows()<<"x"<<Aslip.cols()<<"\n";
    std::cout<< "The Aslip matrix \n" << Aslip << std::endl;
    std::cout<< "Bslip is of size "<<Bslip.rows()<<"x"<<Bslip.cols()<<"\n";
    std::cout<< "The Bslip matrix \n" << Bslip << std::endl;
    #endif
    
    return;
}


void Task::toAsguardBodyState()
{
    register int i;
    
    /** Asguard BodyState (for visualization) **/
    asguard::BodyState asguardBodyState;
    
    asguardBodyState.time = hbridgeStatus[0].time;
    asguardBodyState.twistAngle = asguardStatus[0].asguardJointEncoder;
    
    /** For wheel FL (index 3) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[3])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelFL.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelFL.getWheelIdx(), hbridgeStatus[0].states[3].positionExtern);
    }
    
    
    /** For wheel FR (index 2) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[2])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelFR.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelFR.getWheelIdx(), hbridgeStatus[0].states[2].positionExtern);
    }
    
    /** For wheel RR (index 1) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[1])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelRR.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelRR.getWheelIdx(), hbridgeStatus[0].states[1].positionExtern);
    }
    
    /** For wheel RL (index 0) **/
    for (i = 0; i< FEET_PER_WHEEL; i++)
    {
	asguard::WheelContact mContact;
	mContact.angle = 0.00;
	
	if (i == this->contactPoints[0])
	    mContact.contact = 1.0;
	else
	    mContact.contact = 0.0;
	
	asguardBodyState.setWheelContact(wheelRL.getWheelIdx(), i, mContact);
	asguardBodyState.setWheelPos(wheelRL.getWheelIdx(), hbridgeStatus[0].states[0].positionExtern);
    }
    
    _bodystate_samples.write(asguardBodyState);
    
    
    /** For the movement of the points with the BC **/
    for (register int i=0; i<FEET_PER_WHEEL; i++)
    {
	rbsCiFL2body[i].setTransform(rbsBC.getTransform()*rbsCiFL2body[i].getTransform());
	rbsCiFR2body[i].setTransform(rbsBC.getTransform()*rbsCiFR2body[i].getTransform());
	rbsCiRL2body[i].setTransform(rbsBC.getTransform()*rbsCiRL2body[i].getTransform());
	rbsCiRR2body[i].setTransform(rbsBC.getTransform()*rbsCiRR2body[i].getTransform());
    }
    
    /** Write values in the output ports **/
    _C0FL2body_out.write(rbsCiFL2body[0]);
    _C1FL2body_out.write(rbsCiFL2body[1]);
    _C2FL2body_out.write(rbsCiFL2body[2]);
    _C3FL2body_out.write(rbsCiFL2body[3]);
    _C4FL2body_out.write(rbsCiFL2body[4]);
    
    _C0FR2body_out.write(rbsCiFR2body[0]);
    _C1FR2body_out.write(rbsCiFR2body[1]);
    _C2FR2body_out.write(rbsCiFR2body[2]);
    _C3FR2body_out.write(rbsCiFR2body[3]);
    _C4FR2body_out.write(rbsCiFR2body[4]);
    
    _C0RR2body_out.write(rbsCiRR2body[0]);
    _C1RR2body_out.write(rbsCiRR2body[1]);
    _C2RR2body_out.write(rbsCiRR2body[2]);
    _C3RR2body_out.write(rbsCiRR2body[3]);
    _C4RR2body_out.write(rbsCiRR2body[4]);
    
    _C0RL2body_out.write(rbsCiRL2body[0]);
    _C1RL2body_out.write(rbsCiRL2body[1]);
    _C2RL2body_out.write(rbsCiRL2body[2]);
    _C3RL2body_out.write(rbsCiRL2body[3]);
    _C4RL2body_out.write(rbsCiRL2body[4]);

}

void Task::toMLSGrid(Eigen::Matrix<double, localization::SLIP_VECTOR_SIZE, 1> roverslipvector,
		Eigen::Matrix<double, localization::SLIP_VECTOR_SIZE, localization::SLIP_VECTOR_SIZE> roverslipvectorCov)
{
    boost::circular_buffer<Eigen::Affine3d> world2foot(localization::NUMBER_OF_WHEELS);
    
    for (register unsigned int i = 0; i< localization::NUMBER_OF_WHEELS; i++)
    {
	world2foot.push_front(Eigen::Affine3d::Identity());
    }
    
    #ifdef DEBUG_PRINTS
    std::cout<< "[ENVIRE] world2foot has capacity "<<world2foot.capacity()<<" and size "<<world2foot.size()<<"\n";
    #endif
    
    /** Get the feet in contact **/
    for (register unsigned int j = 0; j< localization::NUMBER_OF_WHEELS; j++)
    {
	for (register unsigned int i = 0; i< FEET_PER_WHEEL; i++)
	{
	    /** For wheel FL (j index 3), FR (j index 2), RR (j index1) RL (j index 0) **/
	    if (i == static_cast<unsigned int>(this->contactPoints[j]))
	    {
		if (j==0)
		    world2foot[j]= rbsCiRL2body[i].getTransform();
		else if (j==1)
		    world2foot[j]= rbsCiRR2body[i].getTransform();
		else if (j==2)
		    world2foot[j]= rbsCiFR2body[i].getTransform();
		else
		    world2foot[j]= rbsCiFL2body[i].getTransform();		
	    }
	}
    }    
    
    /** Update the slip map according to the position and slip vector **/
    
    Eigen::Matrix<double, 3, 1> footPrint;
    footPrint<<round(numberCellsFootx/2.0), round(numberCellsFooty/2.0), 0.0;
    double stdZ = 0.1;
		
    for (register unsigned int k = 0; k< localization::NUMBER_OF_WHEELS; k++)
    {
	/** Foot position in World frame **/
	Eigen::Vector3d footPosition;
	footPosition << world2foot[k].translation()[0], world2foot[k].translation()[1], world2foot[k].translation()[2];
	
	/** Foot position in cells units in MLS frame **/
	size_t xi, yi;
	mpSlip->toGrid(footPosition.x(), footPosition.y(), xi, yi);
	
	/** Create the Patch **/
	envire::MLSGrid::SurfacePatch p(footPosition.z(), stdZ);
	
	/** Get existing Patch if any **/
	envire::MLSGrid::SurfacePatch * prevP = NULL;
	envire::GridBase::Position envirePos (footPosition[0],footPosition[1]);
	prevP = mpSlip->get(envirePos, p, stdZ, true);
	
	/** Slip vector to color **/
	base::Vector3d color;
	base::Vector3d slip = roverslipvector.block<NUMAXIS,1> (k*NUMAXIS,0);
	double slipcolor = 0.00;
	#ifdef DEBUG_PRINTS
	std::cout<< "[ENVIRE] slip\n"<< slip<<"\n";
	#endif
	
// 	slip[2] *= R2D; //! Radians to degrees in the z slip
	slipcolor = std::max(fabs(slip[0]*100), fabs(slip[1]*100));
// 	slipcolor = std::max(fabs(slip[2]*180), slipcolor);
	
	if(slipcolor > slipmaxnorm)
	    slipmaxnorm = slipcolor;
	
	color = mymeasure->valueToColor(slipcolor, 10.0, 0.0);
	p.setColor(color);
	
	#ifdef DEBUG_PRINTS
	std::cout<< "[ENVIRE] Wheel["<< k<<"]\n";
	std::cout<< "[ENVIRE] world2foot(x,y): "<<world2foot[k].translation()[0]<<","<<world2foot[k].translation()[1]<<"\n";
	std::cout<< "[ENVIRE] footi: "<< xi <<" footj: "<< yi<<"\n";
	std::cout<< "[ENVIRE] slip\n"<< slip<<"\n";
	std::cout<< "[ENVIRE] slip.norm: "<< slip.norm()<<"\n";
	std::cout<< "[ENVIRE] slipmaxnorm: "<< slipmaxnorm<<"\n";
	std::cout<< "[ENVIRE] color:\n"<< color<<"\n";
	#endif
	
// 	for(register int i=0;i<numberCellsFootx; i++)
// 	{
// 	    for(register int j=0;j<numberCellsFooty; j++)
// 	    {
// 		Eigen::Matrix<double, 3, 1> offset;
// 		offset<<i*footPrint[0],j*footPrint[1],0;
// 		
// 		offset = world2foot[k].rotation() * offset;//!offset in MLS frame;
// 		footinMLSFrame += offset;
	
	/** First look if there is already a patch in the cell **/
	double zvalue, stDev;
	if (mpSlip->get(footPosition, zvalue, stDev))
	{
	    #ifdef DEBUG_PRINTS
	    std::cout<< "[ENVIRE] THERE is a Patch in cell at ("<< footPosition[0]<<","<< footPosition[1]<<")\n";
	    #endif
	    mpSlip->updateCell (envirePos, p);
	}
	else
	{
	    #ifdef DEBUG_PRINTS
	    std::cout<< "[ENVIRE] No cell at ("<< footPosition[0]<<","<< footPosition[1]<<")\n";
	    #endif
	    mpSlip->insertTail(xi, yi, p);
	}
// 	    }
// 	}
    }
    
    mpSlip->itemModified();
    
    return;
}

void Task::toDebugPorts()
{
    base::samples::RigidBodyState rbsIMU;
    base::samples::RigidBodyState rbsModel;
    base::samples::RigidBodyState rbsCorrected;
    base::samples::RigidBodyState rbsVicon;
    localization::FilterInfo finfo;
    localization::SlipInfo sinfo;
    localization::MeasurementGenerationInfo mInfo;
    
    sinfo.slip_vector.resize(localization::SLIP_VECTOR_SIZE, 1);
    sinfo.Cov.resize(localization::SLIP_VECTOR_SIZE, localization::SLIP_VECTOR_SIZE);

    /** Port out the slip vectors **/
    mymeasure->toSlipInfo(sinfo);
    sinfo.time = rbsBC.time;
    _slip_vector.write(sinfo);
    
    /** Port out filter info **/
    mysckf.toFilterInfo(finfo, propagation_delta_t);
    finfo.time = rbsBC.time;
    _filter_info.write(finfo);
    
    /** Port out the contact angle **/
//     _contact_angle.write(mysckf.measurement.getContactAnglesVelocity()*this->measurement_delta_t);
    
    /** Port out the imu velocities in body frame **/
    rbsIMU.invalidate();
    rbsIMU.time = rbsBC.time;
    Eigen::Matrix<double, NUMAXIS, 1> acc, angvelo;
    mysckf.measurement.getStepInertialValues(acc, angvelo);
    rbsIMU.position = acc;
    rbsIMU.angular_velocity = angvelo;
    rbsIMU.velocity = mysckf.measurement.getLinearVelocities(update_delta_t);
    rbsIMU.cov_velocity = mysckf.getLinearCovarianceVelocities(update_delta_t);
    _incre_velocities_imu.write(rbsIMU);
    
    /** Port out the incremental velocity computed in the model (body frame) **/
    rbsModel.invalidate();
    rbsModel.time = rbsBC.time;
    rbsModel.velocity = mysckf.measurement.getIncrementalVeloModel();
    rbsModel.cov_velocity = mysckf.measurement.getIncrementalVeloModelCovariance();
    _incre_velocities_model.write(rbsModel);
    
    /** Port out the velocity computed in the model (body frame) **/
    rbsModel.velocity = mysckf.measurement.getCurrentVeloModel();
    rbsModel.cov_velocity = mysckf.measurement.getCurrentVeloModelCovariance();
    _velocities_model.write(rbsModel);
    
    rbsCorrected.invalidate();
    rbsCorrected.time = rbsBC.time;
    rbsCorrected.velocity =  mysckf.getVelocity();//truth = estimated + error
    _velocities_corrected.write(rbsCorrected);
    
    /** Port out the incremental velocity error **/
    base::samples::RigidBodyState rbsVelError;
    rbsVelError.time = rbsBC.time;
    rbsVelError.velocity = mysckf.getVeloError(); //error = truth - estimated
    rbsVelError.cov_velocity = finfo.evelocity_cov;
    _velocities_error.write(rbsVelError);
    
//     rbsVelError.velocity = mysckf.getIncreVeloError();
//     _incre_velocities_error.write(rbsVelError);

    if (_pose_init.connected())
    {
	/** Port out the info comming from vicon **/
	rbsVicon = poseInit[0];
	rbsVicon.velocity = mysckf.getAttitude().inverse() * poseInit[0].velocity; // velocity in body frame
	rbsVicon.time = rbsBC.time;
	_rbsVicon.write(rbsVicon);
	
	/** Port out the delta info comming from vicon **/
	rbsVicon.position = poseInit[0].position - poseInit[1].position;
	rbsVicon.cov_position = poseInit[0].cov_position - poseInit[1].cov_position;
	rbsVicon.velocity = mysckf.getAttitude().inverse() * (poseInit[0].velocity - poseInit[1].velocity);//in body frame
	rbsVicon.cov_velocity = mysckf.getAttitude().inverse()* (poseInit[0].cov_velocity - poseInit[1].cov_velocity);
	_incre_rbsVicon.write(rbsVicon);
	
	/** Port out the model velocities error wrt the vicon system **/
	rbsVicon.invalidate();
	rbsVicon.time = rbsBC.time;
	rbsVicon.velocity = (mysckf.getAttitude().inverse() * poseInit[0].velocity) - mysckf.measurement.getCurrentVeloModel();	
	_velocities_error_truth.write(rbsVicon);
    }
    
    /** IMU outport **/
    _imu_sensors_out.write(imuSamples[0]);
    
    /** Port out measurement generation info **/
    mInfo.time = rbsBC.time;
    mymeasure->toMeasurementGenerationInfo(mInfo);
    _measurement_info.write(mInfo);
     
    
}

bool Task::sendEnvireEnvironment()
{
    mEmitter->setTime(rbsBC.time);
    mEmitter->flush();

    return true;
}



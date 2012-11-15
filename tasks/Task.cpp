/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#define DEBUG_PRINTS 1

using namespace asguard_localization;
using namespace localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    
    /** Booleans values to false **/
    imuValues = false;
    hbridgeValues = false;
    asguardValues = false;
    orientationValues = false;
    
    /** Sizing hbridgeStatus **/
    hbridgeStatus.resize(NUMBER_WHEELS);
    prevHbridgeStatus.resize(NUMBER_WHEELS);
    
    /** Set to a NaN index **/
    hbridgeStatus.index = base::NaN<unsigned int>();
    prevHbridgeStatus.index = base::NaN<unsigned int>();
    
    /** Set to NaN passiveJoint **/
    asguardStatus.asguardJointEncoder = base::NaN<double>();
    prevAsguardStatus.asguardJointEncoder = base::NaN<double>();
    
    wheelFL = asguard::KinematicModel(3,2);
    wheelFR = asguard::KinematicModel(2,2);
    wheelRR = asguard::KinematicModel(1,2);
    wheelRL = asguard::KinematicModel(0,2);
    
    /** Imu Samples **/
    imuSamples.acc[0] = base::NaN<double>();
    imuSamples.acc[1] = base::NaN<double>();
    imuSamples.acc[2] = base::NaN<double>();
    imuSamples.gyro = imuSamples.acc;
    imuSamples.mag = imuSamples.acc;
    prevImuSamples = imuSamples;
    
    /** Set eccentricity to NaN **/
    eccx[0] = base::NaN<double>();
    eccx[1] = base::NaN<double>();
    eccx[2] = base::NaN<double>();
    eccy = eccx;
    eccz = eccx;
    
    /** Orientation **/
    orientation.invalidate();
    prevOrientation = orientation;
    
    /** Set also here the default Foot in Contact **/
    contactPoints.resize(4);
    contactPoints[0] = 2; contactPoints[1] = 2;
    contactPoints[2] = 2; contactPoints[3] = 2;
    
    /** Set output times **/
    outTimeHbridge.fromMilliseconds(0.00);
    outTimeForce.fromMilliseconds(0.00);
    outTimeIMU.fromMilliseconds(0.00);
    outTimeStatus.fromMilliseconds(0.00);
    outTimeTorque.fromMilliseconds(0.00);
    
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::calibrated_sensorsTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &calibrated_sensors_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    
    /** Get the transformation **/
    if (!_imu2body.get(ts, tf, false))
	return;
    
    #ifdef DEBUG_PRINTS
    std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
    #endif
	
    /** Check if the eccentricities are not defined **/
    if (base::isNaN(eccx) && base::isNaN(eccy) && base::isNaN(eccz))
    {
	/** Store the excentricity **/
	eccx = tf.translation() + tf *_eccx.value();
	eccy = tf.translation() + tf *_eccy.value();
	eccz = tf.translation() + tf *_eccz.value();
	
	/** Set the eccentricity to the filter **/
	mysckf.setEccentricity(eccx, eccy, eccz);
    }
    
    /** If the initial attitude is not defined and the orientation port is not connected **/
    if (base::isNaN(mysckf.getAttitude()) && (!_orientation_init.connected()))
    {
	
	RTT::log(RTT::Info) << "Calculating initial level position since Init orientation is not provided." << RTT::endlog();
	
	/** Add one acc sample to the buffer **/
	init_acc.col(accidx) = calibrated_sensors_sample.acc;
	accidx++;

	if (accidx == NUMBER_INIT_ACC)
	{
	    Eigen::Matrix <double,NUMAXIS,1> meanacc; /** In euler angles **/
	    Eigen::Matrix <double,NUMAXIS,1> euler; /** In euler angles **/
	    Eigen::Quaternion <double> attitude; /** Initial attitude in case no port in orientation is connected **/
	    
	    meanacc[0] = init_acc.row(0).mean();
	    meanacc[1] = init_acc.row(1).mean();
	    meanacc[2] = init_acc.row(2).mean();
	    
	    RTT::log(RTT::Info) <<"*** Computed mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<RTT::endlog();	
	    RTT::log(RTT::Info) <<"*** Computed gravity: "<<meanacc.norm()<<RTT::endlog();
	    
	    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
	    euler[1] = (double)-atan(meanacc[0]/meanacc[2]); //Pitch
	    euler[2] = 0.00;
	    
	    /** Set the initial attitude when no initial IMU orientation is provided **/
	    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	    
	    /** Set the initial attitude quaternion of the filter **/
	    mysckf.setAttitude (attitude);
	    
	    RTT::log(RTT::Info) << "******** Initial Attitude *******"<<RTT::endlog();
	    RTT::log(RTT::Info) << "Init Roll: "<<euler[0]*R2D<<"Init Pitch: "<<euler[1]*R2D<<"Init Yaw: "<<euler[2]<<RTT::endlog();
	}
    }
    else if (!base::isNaN(mysckf.getAttitude()))
    {
	/** It starts again the sampling **/
	if (counterIMUSamples == 0)
	{
	    outTimeIMU =  calibrated_sensors_sample.time;
	    imuSamples.acc.setZero();
	    imuSamples.gyro.setZero();
	    imuSamples.mag.setZero();
	}
	
	/** Convert the IMU values in the body orientation **/
	imuSamples.time = calibrated_sensors_sample.time;
	imuSamples.acc += tf * calibrated_sensors_sample.acc;
	imuSamples.gyro += tf * calibrated_sensors_sample.gyro;
	imuSamples.mag += tf * calibrated_sensors_sample.mag;
	
	counterIMUSamples++;
	
	if (counterIMUSamples == numberIMUSamples)
	{
	    imuSamples.time = (outTimeIMU + calibrated_sensors_sample.time)/2.0;
	    imuSamples.acc = imuSamples.acc/numberIMUSamples;
	    imuSamples.gyro = imuSamples.gyro/numberIMUSamples;
	    imuSamples.mag = imuSamples.mag/numberIMUSamples;
	    
	    #ifdef DEBUG_PRINTS
	    std::cout<<"** counterIMUSamples ("<<counterIMUSamples<<") at ("<<imuSamples.time.toMicroseconds()<<")**\n";
	    std::cout<<"acc:\n"<<imuSamples.acc<<"\n";
	    std::cout<<"gyro:\n"<<imuSamples.gyro<<"\n";
	    std::cout<<"mag:\n"<<imuSamples.mag<<"\n";
	    #endif
	    
	    counterIMUSamples = 0;
	    outTimeIMU.fromMicroseconds(0.00);
	    
	    imuValues = true;
			
	    if (prevImuSamples.acc[0] == base::NaN<double>() && prevImuSamples.acc[1] == base::NaN<double>() && prevImuSamples.acc[2] == base::NaN<double>())
		prevImuSamples = imuSamples;
	}
    }
}

void Task::hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample)
{
    if (counterHbridgeSamples == 0)
    {
	outTimeHbridge = hbridge_samples_sample.time;
	for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
	{
	    hbridgeStatus.states[i].current = 0.0;
	    hbridgeStatus.states[i].position = 0.0;
	    hbridgeStatus.states[i].positionExtern = 0.0;
	    hbridgeStatus.states[i].pwm = 0.0;
	}
    }
    
    for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
    {
	hbridgeStatus.states[i].current += hbridge_samples_sample.states[i].current;
	hbridgeStatus.states[i].position = hbridge_samples_sample.states[i].position;
	hbridgeStatus.states[i].positionExtern = hbridge_samples_sample.states[i].positionExtern;
	hbridgeStatus.states[i].pwm += hbridge_samples_sample.states[i].pwm;
    }
    
    counterHbridgeSamples++;
    
    if (counterHbridgeSamples == numberHbridgeSamples)
    {
	hbridgeStatus.time = ((outTimeHbridge + hbridge_samples_sample.time)/2.0);
	hbridgeStatus.index = hbridge_samples_sample.index;
	
	for (unsigned int i = 0; i<asguard::NUMBER_OF_WHEELS; i++)
	{
	    hbridgeStatus.states[i].current = hbridgeStatus.states[i].current/numberHbridgeSamples; //the mean current in mA
	    hbridgeStatus.states[i].pwm = hbridgeStatus.states[i].pwm/numberHbridgeSamples; //the mean PWM signal duty cicle [-1,1]
	}
	
	#ifdef DEBUG_PRINTS
 	std::cout<<"** counterHbridgeSamples("<<counterHbridgeSamples<<") at ("<<hbridgeStatus.time.toMicroseconds()<<")**\n";
	#endif
	
	counterHbridgeSamples = 0;
	outTimeHbridge.fromMicroseconds(0.00);
	
	hbridgeValues = true;
	
	if (prevHbridgeStatus.index == base::NaN<unsigned int>())
	    prevHbridgeStatus = hbridgeStatus;
    }
    

}
void Task::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
    if (counterAsguardStatusSamples == 0)
    {
	outTimeStatus = systemstate_samples_sample.time;
	asguardStatus.asguardVoltage = 0.00;
    }
    
    asguardStatus.time = systemstate_samples_sample.time;
    asguardStatus.asguardVoltage += systemstate_samples_sample.asguardVoltage;
    asguardStatus.asguardJointEncoder = systemstate_samples_sample.asguardJointEncoder;
    asguardStatus.systemState = systemstate_samples_sample.systemState;
    asguardStatus.packetsPerSec = systemstate_samples_sample.packetsPerSec;
    asguardStatus.controlPacketsPerSec = systemstate_samples_sample.controlPacketsPerSec;
    
    counterAsguardStatusSamples ++;
    
    if (counterAsguardStatusSamples == numberAsguardStatusSamples)
    {
	asguardStatus.time = (outTimeStatus + systemstate_samples_sample.time)/2.0;
	asguardStatus.asguardVoltage = asguardStatus.asguardVoltage/numberAsguardStatusSamples; //the mean voltage
	
	#ifdef DEBUG_PRINTS
 	std::cout<<"** counterAsguardStatusSamples("<<counterAsguardStatusSamples<<") at ("<<asguardStatus.time.toMicroseconds()<< ")**\n";
	#endif
	
	counterAsguardStatusSamples = 0;
	outTimeStatus.fromMicroseconds(0.00);
	
	if (prevAsguardStatus.asguardJointEncoder == base::NaN<double>())
	    prevAsguardStatus = asguardStatus;
    }

}

void Task::torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for torque_estimated not implemented");
}

void Task::ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for ground_forces_estimated not implemented");
}

void Task::orientation_initTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& orientation_init_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Transformer transformation in quaternion**/
    
    orientation = orientation_init_sample;
    
    if (!_imu2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (Eigen::AngleAxisd(tf.rotation()));
    orientation.orientation = orientation.orientation * qtf;
    
    if (base::isNaN(mysckf.getAttitude()))
    {
	Eigen::Quaterniond attitude = orientation.orientation;
	mysckf.setAttitude(attitude);
    }
    
    orientationValues = true;
     
    if (!prevOrientation.hasValidOrientation())
	prevOrientation = orientation;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    double delta_time;/** Delta (bandwidth) time of inertial sensors **/
    double theoretical_g; /** Ideal gravity value **/
    Eigen::Matrix< double, Eigen::Dynamic,1> x_0; /** Initial vector state **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra; /**< Measurement noise convariance matrix for acc */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rv; /**< Measurement noise convariance matrix for velocity (accelerometers integration) */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /**< Measurement noise convariance matrix for gyros */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /**< Measurement noise convariance matrix for mag */
    Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic> Ren; /** Measurement noise of encoders **/
    Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic> P_0; /**< Initial covariance matrix **/
    Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic> Qec; /** Process noise of slip vector and contact angle **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qbg;
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Qba;
    
    if (! TaskBase::configureHook())
        return false;
    
    if (_sensors_bandwidth.value() != 0.00)
	delta_time = 1.0/_sensors_bandwidth.value();
    
    rbsBC.sourceFrame = "Body Frame";
    rbsBC.targetFrame = "Geographic_Frame (North-West-Up)";
    
    /** Set the initial BC to the Geographic frame **/
    rbsBC.invalidateCovariances(true, true, true, true);
    rbsBC.position.setZero();
    rbsBC.velocity.setZero();
    rbsBC.orientation = Eigen::Quaterniond::Identity();
    rbsBC.angular_velocity.setZero();
    
    /** Assume well known starting position **/
    rbsBC.cov_position = Eigen::Matrix <double, 3 , 3>::Zero();
    rbsBC.cov_orientation = Eigen::Matrix <double, 3 , 3>::Zero();
    
    /** Set the counters to Zero **/
    counterHbridgeSamples = 0;
    counterAsguardStatusSamples = 0;
    counterTorqueSamples = 0;
    counterForceSamples = 0;
    counterIMUSamples = 0;
    
    /** Set the number of samples between each sensor input (if there are not comming at the same sampling rate) */
    numberIMUSamples = (1.0/_calibrated_sensors_period.value())/_filter_frequency.value();
    numberHbridgeSamples = (1.0/_hbridge_samples_period.value())/_filter_frequency.value();
    numberAsguardStatusSamples = (1.0/_systemstate_samples_period.value())/_filter_frequency.value();
    numberForceSamples = (1.0/_ground_forces_estimated_period.value())/_filter_frequency.value();
    numberTorqueSamples = (1.0/_torque_estimated_period.value())/_filter_frequency.value();
    
    /** Filter delta time step in sencods **/
    if (_filter_frequency.value() != 0.00)
	delta_t = 1.0/_filter_frequency.value();
    
    
    /** Fill the filter initial matrices **/
    /** Accelerometers covariance matrix **/
    Ra = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Ra(0,0) = pow(_accrw.get()[0]/sqrt(delta_time),2);
    Ra(1,1) = pow(_accrw.get()[1]/sqrt(delta_time),2);
    Ra(2,2) = pow(_accrw.get()[2]/sqrt(delta_time),2);
    
    /** Gyroscopes covariance matrix with is std_vel = std_acc * sqrt(integration_step) **/
    Rv = Ra * delta_t;
    
    /** Gyroscopes covariance matrix **/
    Rg = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rg(0,0) = pow(_gyrorw.get()[0]/sqrt(delta_time),2);
    Rg(1,1) = pow(_gyrorw.get()[1]/sqrt(delta_time),2);
    Rg(2,2) = pow(_gyrorw.get()[2]/sqrt(delta_time),2);

    /** Magnetometers covariance matrix **/
    Rm = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rm(0,0) = pow(_magrw.get()[0]/sqrt(delta_time),2);
    Rm(1,1) = pow(_magrw.get()[1]/sqrt(delta_time),2);
    Rm(2,2) = pow(_magrw.get()[2]/sqrt(delta_time),2);
    
    /** Encoders errors **/
    Ren.resize(1+sckf::NUMBER_OF_WHEELS, 1+sckf::NUMBER_OF_WHEELS);
    
    /** Initial error covariance **/
    P_0.resize(sckf::X_STATE_VECTOR_SIZE, sckf::X_STATE_VECTOR_SIZE);
    P_0 = Eigen::Matrix <double,sckf::X_STATE_VECTOR_SIZE,sckf::X_STATE_VECTOR_SIZE>::Zero();
    P_0.block <(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS), (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)> (0,0) = 0.001 * Eigen::Matrix <double,(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS),(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS),(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)) = 0.001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+NUMAXIS,(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+NUMAXIS) = 0.00001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+(2*NUMAXIS),(sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+(2*NUMAXIS)) = 0.00001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Process noise matrices **/
    Qec.resize((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS), (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS));
    for (int i = 0; i<sckf::NUMBER_OF_WHEELS; i++)
    {
	Eigen::Matrix<double, NUMAXIS, 1> cov;
	cov[0] = asguard::KinematicModel::STD_FOOT_X;
	cov[1] = asguard::KinematicModel::STD_FOOT_Y;
	cov[2] = asguard::KinematicModel::STD_FOOT_Z;
	Qec.block<NUMAXIS, NUMAXIS>(i*NUMAXIS,i*NUMAXIS) = cov.array().square().matrix().asDiagonal() * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    }
    
    Qec.block<NUMBER_WHEELS, NUMBER_WHEELS>(NUMBER_WHEELS*NUMAXIS,NUMBER_WHEELS*NUMAXIS) = 
	(pow(asguard::KinematicModel::STD_FOOT_X,2) + pow(asguard::KinematicModel::STD_FOOT_Z,2)) * Eigen::Matrix <double,NUMBER_WHEELS,NUMBER_WHEELS>::Identity();
    
    Qbg = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    Qba = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Gravitational value according to the location **/
    theoretical_g = localization::GravityModel (_latitude.value(), _altitude.value());
    
     /** Initialization of the filter **/
     mysckf.Init(P_0, Qec, Qbg, Qba, Rv, Rg, Ren, Ra, Rm, theoretical_g, (double)_dip_angle.value());
     
    /** Initialization set the vector state to zero but it can be changed here **/
    x_0.resize(sckf::X_STATE_VECTOR_SIZE,1);
    x_0 = Eigen::Matrix<double,sckf::X_STATE_VECTOR_SIZE,1>::Zero();
    x_0.block<NUMAXIS, 1> ((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+NUMAXIS,0) = _gbiasof.value();
    x_0.block<NUMAXIS, 1> ((sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)+(2*NUMAXIS),0) = _abiasof.value();
    mysckf.setStatex(x_0);
    
    
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
    
    if (_orientation_init.connected())
    {
	RTT::log(RTT::Warning) << "Initial orientation connected" << RTT::endlog();
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
    RTT::log(RTT::Warning)<<"[Info] Filter running at Frequency[Hertz]: "<<_filter_frequency.value()<<RTT::endlog();
    
    
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
     
    #ifdef DEBUG_PRINTS
    std::cout<<"In UpdateHook\n";
    #endif
 
    /** Start the filter if all the values arrived to the ports in the correct order **/
    if (imuValues && asguardValues && hbridgeValues)
    {
	/** Calculate the position and orientation of all asguard feets with the new encoders information **/
	this->calculateFootPoints();
	
	/** Select the Contact Point among the Foot Points **/
	this->selectContactPoints(contactPoints);
	
	/** Set the Contact Point in the KinematicModel **/
	wheelFL.setContactFootID(contactPoints[3]);
	wheelFR.setContactFootID(contactPoints[2]);
	wheelRR.setContactFootID(contactPoints[1]);
	wheelRL.setContactFootID(contactPoints[0]);
	
	/** Compute the wheel Contact Point **/
	wheelRL.Body2ContactPoint(hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);    
	wheelRR.Body2ContactPoint(hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
	wheelFR.Body2ContactPoint(hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
	wheelFL.Body2ContactPoint(hbridgeStatus.states[3].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
	
	/** Compute the wheel Jacobians for the Selected Foot Point **/
	Eigen::Matrix< double, NUMAXIS , 1> slipvector = Eigen::Matrix< double, 3 , 1>::Zero();
	jacobRL = wheelRL.getWheelJacobian (slipvector);
	jacobRR = wheelRR.getWheelJacobian (slipvector);
	jacobFR = wheelFR.getWheelJacobian (slipvector);
	jacobFL = wheelFL.getWheelJacobian (slipvector);
	
	/** Calculate velocities joints velocities **/
	this->calculateVelocities();
	
	/** Composite Slip-Kinematics and Observation matrix for the SCKF **/
	this->compositeMatrices();
	
	/** Substract Earth rotation from gyros **/
	Eigen::Matrix<double, NUMAXIS, 1> angular_velocity = imuSamples.gyro;
	Eigen::Quaternion <double> currentq = mysckf.getAttitude();
	localization::SubstractEarthRotation(&angular_velocity, &currentq, _latitude.value());
	
	/** Predict the state in the filter **/
	mysckf.predict(angular_velocity, delta_t);
	
	/** Update the state in the filter **/
	Eigen::Matrix<double, NUMAXIS, 1> acc = imuSamples.acc;
	Eigen::Matrix<double, NUMAXIS, 1> mag = imuSamples.mag;
	mysckf.update(H, Be, vjoints, acc, angular_velocity, mag, delta_t, false);
	
	/** Update the estimated values **/
	
	/** Compute rover pose with new calculated values **/
	
	/** Set to false the new values for next filter iteration **/
	imuValues = false;
	orientationValues = false;
	hbridgeValues = false;
	asguardValues = false;
    }
    
   
    
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

void Task::calculateFootPoints()
{
    /** For the Wheel 0 Rear Left **/
    wheelRL.Body2FootPoint (0, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (1, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (2, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (3, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3RL2body = wheelRL.getBody2FootPoint();
    wheelRL.Body2FootPoint (4, hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4RL2body = wheelRL.getBody2FootPoint();
    
//     wheelRL.Body2ContactPoint (hbridgeStatus.states[0].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
//     rbsC2RL2body = wheelRL.getBody2ContactPoint();
    
    /** For the Wheel 1 Rear Right **/
    wheelRR.Body2FootPoint (0, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (1, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (2, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (3, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3RR2body = wheelRR.getBody2FootPoint();
    wheelRR.Body2FootPoint (4, hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4RR2body = wheelRR.getBody2FootPoint();
    
//     wheelRR.Body2ContactPoint (hbridgeStatus.states[1].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
//     rbsC2RR2body = wheelRR.getBody2ContactPoint();
    
    /** For the Wheel 2 Forward Right **/
    wheelFR.Body2FootPoint (0, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC0FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (1, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC1FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (2, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC2FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (3, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC3FR2body = wheelFR.getBody2FootPoint();
    wheelFR.Body2FootPoint (4, hbridgeStatus.states[2].positionExtern, asguardStatus.asguardJointEncoder, 0.00);
    rbsC4FR2body = wheelFR.getBody2FootPoint();
    
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
    footFL[0] = rbsC0FL2body.position(2);
    footFL[1] = rbsC1FL2body.position(2);
    footFL[2] = rbsC2FL2body.position(2);
    footFL[3] = rbsC3FL2body.position(2);
    footFL[4] = rbsC4FL2body.position(2);
    
    contactPoints[3] = std::distance(footFL.begin(), std::min_element(footFL.begin(), footFL.end()));
    /*std::cout<<"FL position: "<< rbsC0FL2body.position(2)<<"\n";
    std::cout<<"footFL: "<<footFL[0]<<" "<<footFL[1]<<" "<<footFL[2]<<" "<<footFL[3]<<" "<<footFL[4]<<"\n";
    std::cout<<"contactPoints[3]: "<< contactPoints[3]<<"\n";*/
    
    /** For the FR wheel **/
    footFR.resize(5);
    
    footFR[0] = rbsC0FR2body.position(2);
    footFR[1] = rbsC1FR2body.position(2);
    footFR[2] = rbsC2FR2body.position(2);
    footFR[3] = rbsC3FR2body.position(2);
    footFR[4] = rbsC4FR2body.position(2);
    
    contactPoints[2] = std::distance(footFR.begin(), std::min_element(footFR.begin(), footFR.end()));
    
    /** For the RR wheel **/
    footRR.resize(5);
    
    footRR[0] = rbsC0RR2body.position(2);
    footRR[1] = rbsC1RR2body.position(2);
    footRR[2] = rbsC2RR2body.position(2);
    footRR[3] = rbsC3RR2body.position(2);
    footRR[4] = rbsC4RR2body.position(2);
    
    contactPoints[1] = std::distance(footRR.begin(), std::min_element(footRR.begin(), footRR.end()));
    
    /** For the RL wheel **/
    footRL.resize(5);
    
    footRL[0] = rbsC0RL2body.position(2);
    footRL[1] = rbsC1RL2body.position(2);
    footRL[2] = rbsC2RL2body.position(2);
    footRL[3] = rbsC3RL2body.position(2);
    footRL[4] = rbsC4RL2body.position(2);
    
    contactPoints[0] = std::distance(footRL.begin(), std::min_element(footRL.begin(), footRL.end()));
    
    return;
    
}

void Task::calculateVelocities()
{
    
    /** Set the correct size for the joint velocities vector **/
    vjoints.resize (1 + NUMBER_WHEELS, 1);
   
     /** Passive joint velocity **/
    vjoints(0) = (asguardStatus.asguardJointEncoder - prevAsguardStatus.asguardJointEncoder)/delta_t; //passive joints
    
    /** Velocities for the vector **/
    for (int i = 1; i< (NUMBER_WHEELS + 1); i++)
    {
	#ifdef DEBUG_PRINTS
	std::cout<<"New: "<< hbridgeStatus.states[i-1].positionExtern <<" Prev: "<<prevHbridgeStatus.states[i-1].positionExtern<<"\n";
	#endif
	vjoints(i) = (hbridgeStatus.states[i-1].positionExtern - prevHbridgeStatus.states[i-1].positionExtern)/delta_t; //wheel rotation
    }
    
    /** The previous values are the current ones for the next iteration **/
    prevAsguardStatus = asguardStatus;
    prevHbridgeStatus = hbridgeStatus;
    prevImuSamples = imuSamples;
    
    return;
}

void Task::compositeMatrices()
{
    /** Set the proper size of the matrices **/
    E.resize(sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS);
    J.resize(sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 1 + sckf::NUMBER_OF_WHEELS + (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS));
    J = Eigen::Matrix <double, sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 1 + sckf::NUMBER_OF_WHEELS + (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)>::Zero();
    H.resize(sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS));
    H = Eigen::Matrix <double, sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)>::Zero();
    Be.resize(sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), sckf::E_MEASUREMENT_VECTOR_SIZE);
    Be = Eigen::Matrix <double, sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), sckf::E_MEASUREMENT_VECTOR_SIZE>::Zero();
    
    /** Compute the composite rover equation matrix E **/
    E.block<2*NUMAXIS, 2*NUMAXIS>(0,0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*NUMAXIS, 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(2*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    E.block<2*NUMAXIS, 2*NUMAXIS>(3*(2*NUMAXIS), 0) = Eigen::Matrix <double, 2*NUMAXIS, 2*NUMAXIS>::Identity();
    
    #ifdef DEBUG_PRINTS
    std::cout<< "Composite matrices\n";
    std::cout<< "E is of size "<<E.rows()<<"x"<<E.cols()<<"\n";
    std::cout << "The E matrix \n" << E << endl;
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
    std::cout << "The J matrix \n" << J << endl;
    #endif
    
    
    /** Form the matrix Be for the measurement vector of the filter **/
    Be.block<sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 2*NUMAXIS>(0,0) = - E;
    Be.block<sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 1 + sckf::NUMBER_OF_WHEELS>(0,2*NUMAXIS) = J.block<sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), 1 + sckf::NUMBER_OF_WHEELS>(0,0);
    
    #ifdef DEBUG_PRINTS
    std::cout<< "Be is of size "<<Be.rows()<<"x"<<Be.cols()<<"\n";
    std::cout << "The Be matrix \n" << Be << endl;
    #endif
    
    /** Form the matrix H for the observation of the filter **/
    H = -J.block<sckf::NUMBER_OF_WHEELS*(2*NUMAXIS), (sckf::E_STATE_VECTOR_SIZE*sckf::NUMBER_OF_WHEELS)>(0,1+sckf::NUMBER_OF_WHEELS);
    
    #ifdef DEBUG_PRINTS
    std::cout<< "H is of size "<<H.rows()<<"x"<<H.cols()<<"\n";
    std::cout << "The H matrix \n" << H << endl;
    #endif
    
    return;
}


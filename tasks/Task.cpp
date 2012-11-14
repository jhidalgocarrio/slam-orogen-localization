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
    
    //     std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
	
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
	mysckf.setAttitude(reinterpret_cast<Eigen::Quaternion<double>&>(orientation.orientation));
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
    double delta_time = 1.0/_sensors_bandwidth.value(); /** Delta (bandwidth) time of inertial sensors **/
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
    delta_t = 1.0/_filter_frequency.value();
    
    
    /** Fill the filter initial matrices **/
    /** Accelerometers covariance matrix **/
    Ra = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Ra(0,0) = pow(_accrw.get()[0]/sqrt(delta_time),2);
    Ra(1,1) = pow(_accrw.get()[1]/sqrt(delta_time),2);
    Ra(2,2) = pow(_accrw.get()[2]/sqrt(delta_time),2);
    
    /** Gyroscopes covariance matrix **/
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
    
    /** Magnetometers covariance matrix **/
    Ren.resize(sckf::EMEASUREMENTVECTORSIZE-(2*NUMAXIS), sckf::EMEASUREMENTVECTORSIZE-(2*NUMAXIS));
    
    /** Initial error covariance **/
    P_0.resize(sckf::XSTATEVECTORSIZE, sckf::XSTATEVECTORSIZE);
    P_0 = Eigen::Matrix <double,sckf::XSTATEVECTORSIZE,sckf::XSTATEVECTORSIZE>::Zero();
    P_0.block <(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS), (sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)> (0, 0) = 0.001 * Eigen::Matrix <double,(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS),(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS),(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)) = 0.001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+NUMAXIS,(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+NUMAXIS) = 0.00001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> ((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+(2*NUMAXIS),(sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+(2*NUMAXIS)) = 0.00001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Process noise matrices **/
    Qec.resize((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS), (sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS));

    
    Qbg = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    Qba = 0.00000000001 * Eigen::Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    /** Gravitational value according to the location **/
    theoretical_g = localization::GravityModel (_latitude.value(), _altitude.value());
    
    /** Initialization of the filter **/
    mysckf.Init(P_0, Qec, Qbg, Qba, Rv, Rg, Ren, Ra, Rm, theoretical_g, (double)_dip_angle.value());
    
    /** Initialization set the vector state to zero but it can be changed here **/
    x_0.resize(sckf::XSTATEVECTORSIZE,1);
    x_0 = Eigen::Matrix<double,sckf::XSTATEVECTORSIZE,1>::Zero();
    x_0.block<NUMAXIS, 1> ((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+NUMAXIS,0) = _gbiasof.value();
    x_0.block<NUMAXIS, 1> ((sckf::ESTATEVECTORSIZE*sckf::NUMBEROFWHEELS)+(2*NUMAXIS),0) = _abiasof.value();
    mysckf.setStatex(x_0);
    
    
    /** Info and Warnings about the Task **/
    if (_calibrated_sensors.connected())
    {
	RTT::log(RTT::Info) << "IMU Samples are connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_hbridge_samples.connected())
    {
	RTT::log(RTT::Info) << "Hbridge Samples are connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Hbridge samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_systemstate_samples.connected())
    {
	RTT::log(RTT::Info) << "System State Samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "System State samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Malfunction on the task!!" << RTT::endlog();
    }
    
    if (_torque_estimated.connected())
    {
	RTT::log(RTT::Info) << "Wheel Torque estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel Torque estimation samples NO connected." << RTT::endlog();
    }
    
    if (_ground_forces_estimated.connected())
    {
	RTT::log(RTT::Info) << "Wheel ground force estimation samples connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Wheel ground force estimation samples NO connected." << RTT::endlog();
    }
    
    if (_orientation_init.connected())
    {
	RTT::log(RTT::Info) << "Initial orientation connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "Initial orientation NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "Pitch and Roll are taken from accelerometers assuming static body at Initial phase of this task." << RTT::endlog();
    }
    
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Frequency of IMU samples[Hertz]: "<<(1.0/_calibrated_sensors_period.value())<<"\n";
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Frequency of Hbridge Status[Hertz]: "<<(1.0/_hbridge_samples_period.value())<<"\n";
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Frequency of Asguard Status[Hertz]: "<<(1.0/_systemstate_samples_period.value())<<"\n";
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Frequency of Torque Estimator[Hertz]: "<<(1.0/_torque_estimated_period.value())<<"\n";
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Frequency of Ground Force Estimator[Hertz]: "<<(1.0/_ground_forces_estimated_period.value())<<"\n";
    RTT::Logger::log()<<RTT::Logger::Info<<"[Info] Filter running at Frequency[Hertz]: "<<_filter_frequency.value()<<"\n";
    
    
    if (! TaskBase::configureHook())
        return false;
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
    std::cout<<"In UpdateHook\n";
 
    /** Start the filter if all the values arrived to the ports in the correct order **/
    if (imuValues && asguardValues && hbridgeValues)
    {
	/** Caluclate the position and orientation of all asguard feets **/
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
	
	/** Compute the wheel Jacobians **/
	Eigen::Matrix< double, NUMAXIS , 1> slipvector = Eigen::Matrix< double, 3 , 1>::Zero();
	jacobRL = wheelRL.getWheelJacobian (slipvector);
	jacobRR = wheelRR.getWheelJacobian (slipvector);
	jacobFR = wheelFR.getWheelJacobian (slipvector);
	jacobFL = wheelFL.getWheelJacobian (slipvector);
	
	
	
	/** Set to false the new values for next filter iteration **/
	imuValues = false;
	orientationValues = false;
	hbridgeValues = false;
	asguardValues = false;
    }
    
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

}

void Task::selectContactPoints(std::vector<int> &contactPoints)
{
    std::vector<double> footFR;
    std::vector<double> footFL;
    std::vector<double> footRR;
    std::vector<double> footRL;

    /** For the FL wheel **/
    footFL.resize(5);
    
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
    
}


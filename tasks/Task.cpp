/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace asguard_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    
    /** Booleans values to false **/
    imuValues = false;
    hbridgeValues = false;
    asguardValues = false;
    orientationValues = false;
    
    /** Set to a NaN index **/
    hbridgeStatus.index = base::NaN<unsigned int>();
    prevHbridgeStatus.index = base::NaN<unsigned int>();
    
    /** Set to NaN passiveJoint **/
    asguardStatus.asguardJointEncoder = base::NaN<double>();
    prevAsguardStatus.asguardJointEncoder = base::NaN<double>();
    
    
    for (int i = 0; i< NUMBER_WHEELS; i++)
    {
	current[i] = 0;
	pwm[i] = 0;
    }
    
    wheelFL = asguard::KinematicModel(3,2);
    wheelFR = asguard::KinematicModel(2,2);
    wheelRR = asguard::KinematicModel(1,2);
    wheelRL = asguard::KinematicModel(0,2);
    
    /** Set also here the default Foot in Contact **/
    contactPoints.resize(4);
    contactPoints[0] = 2; contactPoints[1] = 2;
    contactPoints[2] = 2; contactPoints[3] = 2;
    
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
    
    
    if (!_imu2body.get(ts, tf, false))
	return;
    
//     std::cout<<"Transformer:\n"<<tf.matrix()<<"\n";
    
    /** Convert the IMU values in the body orientation **/
    imuSamples.time = calibrated_sensors_sample.time;
    imuSamples.acc = tf * calibrated_sensors_sample.acc;
    imuSamples.gyro = tf * calibrated_sensors_sample.gyro;
    imuSamples.mag = tf * calibrated_sensors_sample.mag;
    
    imuValues = true;
    
    std::cout<<"acc:\n"<<imuSamples.acc<<"\n";
    std::cout<<"gyro:\n"<<imuSamples.gyro<<"\n";
//     std::cout<<"mag:\n"<<imuSamples.mag<<"\n";
    
}
void Task::ground_forces_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::GroundForces &ground_forces_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for ground_forces_estimated not implemented");
}
void Task::hbridge_samplesTransformerCallback(const base::Time &ts, const ::base::actuators::Status &hbridge_samples_sample)
{
//     throw std::runtime_error("Transformer callback for hbridge_samples not implemented");
}
void Task::systemstate_samplesTransformerCallback(const base::Time &ts, const ::sysmon::SystemStatus &systemstate_samples_sample)
{
//     throw std::runtime_error("Transformer callback for systemstate_samples not implemented");
}
void Task::torque_estimatedTransformerCallback(const base::Time &ts, const ::torque_estimator::WheelTorques &torque_estimated_sample)
{
//     throw std::runtime_error("Transformer callback for torque_estimated not implemented");
}

void Task::orientation_debugTransformerCallback(const base::Time& ts, const base::samples::RigidBodyState& orientation_debug_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    Eigen::Quaternion <double> qtf; /** Transformer transformation in quaternion**/
    
    orientation = orientation_debug_sample;
    
    if (!_imu2body.get(ts, tf, false))
	return;
    
    qtf = Eigen::Quaternion <double> (Eigen::AngleAxisd(tf.rotation()));
    
    orientation.orientation = orientation.orientation * qtf;
    
    orientationValues = true;
     
    if (!prevOrientation.hasValidOrientation())
	prevOrientation = orientation;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
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
    Eigen::Matrix <double,NUMAXIS,1> gtilde; /**< gravitation acceleration */
    
    /** Gravitation acceleration **/
    gtilde << 0, 0, 9.74;
    
    
    std::cout<<"In UpdateHook\n";
    
    if (imuValues && orientationValues)
    {
	Eigen::Matrix <double,NUMAXIS,1> gtilde_body; /**< Gravitation in the body frame */
	
	gtilde_body = orientation.orientation * gtilde;
	
	std::cout<<"gtilde_body:\n"<<gtilde_body<<"\n";
	
	std::cout<<"acceleration:\n"<<imuSamples.acc - gtilde_body<<"\n";
	std::cout<<"velocity:\n"<<(imuSamples.acc - gtilde_body) * 0.008<<"\n";
	
	rbsBC.time = orientation.time;
	rbsBC.orientation = orientation.orientation;
	rbsBC.velocity = (imuSamples.acc - gtilde_body) * 0.008;
	
	_pose_samples_out.write(rbsBC);
	
	/** Set to false the new values **/
	imuValues = false;
	orientationValues = false;
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


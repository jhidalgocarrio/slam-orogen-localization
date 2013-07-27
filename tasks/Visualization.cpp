/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Visualization.hpp"

using namespace rover_localization;

Visualization::Visualization(std::string const& name)
    : VisualizationBase(name)
{
}

Visualization::Visualization(std::string const& name, RTT::ExecutionEngine* engine)
    : VisualizationBase(name, engine)
{
}

Visualization::~Visualization()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Visualization.hpp for more detailed
// documentation about them.

bool Visualization::configureHook()
{
    if (! VisualizationBase::configureHook())
        return false;

    /** Initialize the class field **/
    pose_frontend.invalidate();
    pose_frontend.position.setZero();

    pose_backend.invalidate();
    pose_backend.position.setZero();

    pose_truth.invalidate();
    pose_truth.position.setZero();

    return true;
}

void Visualization::frontend_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &frontend_pose_samples_sample)
{
    this->pose_frontend = frontend_pose_samples_sample;

    /** Front-end pose **/
    RTT::log(RTT::Info) << "[VISUALIZATION] Received new frontend_pose_samples " << RTT::endlog();
    _frontend_pose_samples_out.write(pose_frontend);

}

void Visualization::backend_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &backend_pose_samples_sample)
{
    this->pose_backend = backend_pose_samples_sample;

    /** Back-end pose **/
    RTT::log(RTT::Info) << "[VISUALIZATION] Received new backend_pose_samples " << RTT::endlog();
    _backend_pose_samples_out.write(pose_backend);

}

void Visualization::reference_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &reference_pose_samples_sample)
{
    this->pose_truth = reference_pose_samples_sample;

    /** Ground Truth pose if available **/
    RTT::log(RTT::Info) << "[VISUALIZATION] Received new reference_pose_samples " << RTT::endlog();
    _reference_pose_samples_out.write(pose_truth);

}

void Visualization::fkchains_samplesTransformerCallback(const base::Time &ts, const ::rover_localization::RobotContactPoints &fkchains_samples_sample)
{
    rover_localization::RobotContactPointsRbs robotKineRbs;

    this->robotKineInfo = fkchains_samples_sample;

    /** Asguard Body and Kinematic Chains **/
    RTT::log(RTT::Info) << "[VISUALIZATION]Received new fkchains_samples " << RTT::endlog();

    robotKineRbs.time = robotKineInfo.time;
    robotKineRbs.rbsChain.resize(robotKineInfo.chain.size());

    /** For the AsguardBody type **/
    this->toAsguardBodyState (robotKineInfo);

    /** For the movement of the points with the BC **/
    for (register unsigned int i=0; i<robotKineInfo.chain.size(); ++i)
    {
        Eigen::Affine3d tfChain (robotKineInfo.chain[i]);
        robotKineRbs.rbsChain[i].invalidate();
        robotKineRbs.rbsChain[i].time = robotKineInfo.time;
        robotKineRbs.rbsChain[i].setTransform(pose_frontend.getTransform()*tfChain);
        robotKineRbs.rbsChain[i].cov_position = robotKineInfo.cov[i].topLeftCorner<3,3>();
        robotKineRbs.rbsChain[i].cov_orientation = robotKineInfo.cov[i].bottomRightCorner<3,3>();
    }

    _fkchains_rbs_out.write(robotKineRbs);
}

bool Visualization::startHook()
{
    if (! VisualizationBase::startHook())
        return false;
    return true;
}
void Visualization::updateHook()
{
    VisualizationBase::updateHook();
}
void Visualization::errorHook()
{
    VisualizationBase::errorHook();
}
void Visualization::stopHook()
{
    VisualizationBase::stopHook();
}
void Visualization::cleanupHook()
{
    VisualizationBase::cleanupHook();
}

void Visualization::toAsguardBodyState(rover_localization::RobotContactPoints & robotKineInfo)
{
    /** Asguard BodyState (for visualization) **/
    asguard::BodyState asguardBodyState;

    asguardBodyState.time = robotKineInfo.time;//!Timestamped info
    asguardBodyState.twistAngle = robotKineInfo.modelPositions[0]; //!Passive joint

    /** Information to the dedicated BodyAsguard visualization type **/
    for (register unsigned int i = 0; i < asguard::NUMBER_OF_WHEELS; ++i)
    {
        for (register int j = 0; j < asguard::FEET_PER_WHEEL; ++j)
        {
            asguard::WheelContact mContact;
            mContact.angle = 0.00;

            if(j == robotKineInfo.contactPoints[i])
                mContact.contact = 1.0;
            else
                mContact.contact = 0.00;

            /** AsguardBodyState visualization class has a different numering of the contact points **/
            /** AsguardBodyState is increasing while Asguard moving forward, the AsguardKinematicModel is decreasing while moving forward or **/
            /** increasing in the clockwise angle of the wheel rotation (Y-axis point left and coincident with the wheel axle)) **/
            /** This has an effect on the numering of the point in contact to call the function below **/
            asguardBodyState.setWheelContact(asguard::wheelIdxEnum(static_cast<unsigned long>(i)), (asguard::FEET_PER_WHEEL-j)%asguard::FEET_PER_WHEEL, mContact);
            asguardBodyState.setWheelPos(asguard::wheelIdxEnum(static_cast<unsigned long>(i)), robotKineInfo.modelPositions[i+1]);
        }
    }

    _bodystate_samples_out.write(asguardBodyState);

    return;
}

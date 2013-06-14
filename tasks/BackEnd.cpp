/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BackEnd.hpp"

using namespace rover_localization;

BackEnd::BackEnd(std::string const& name)
    : BackEndBase(name)
{
}

BackEnd::BackEnd(std::string const& name, RTT::ExecutionEngine* engine)
    : BackEndBase(name, engine)
{
}

BackEnd::~BackEnd()
{
}

void BackEnd::exteroceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &exteroceptive_samples_sample)
{
    throw std::runtime_error("Transformer callback for exteroceptive_samples not implemented");
}

void BackEnd::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
    throw std::runtime_error("Transformer callback for pose_samples not implemented");
}

void BackEnd::proprioceptive_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &proprioceptive_samples_sample)
{
    throw std::runtime_error("Transformer callback for proprioceptive_samples not implemented");
}

void BackEnd::update_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &update_samples_sample)
{
    throw std::runtime_error("Transformer callback for update_samples not implemented");
}

void BackEnd::visual_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &visual_samples_sample)
{
    throw std::runtime_error("Transformer callback for visual_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BackEnd.hpp for more detailed
// documentation about them.

bool BackEnd::configureHook()
{
    if (! BackEndBase::configureHook())
        return false;
    return true;
}
bool BackEnd::startHook()
{
    if (! BackEndBase::startHook())
        return false;
    return true;
}
void BackEnd::updateHook()
{
    BackEndBase::updateHook();
}
void BackEnd::errorHook()
{
    BackEndBase::errorHook();
}
void BackEnd::stopHook()
{
    BackEndBase::stopHook();
}
void BackEnd::cleanupHook()
{
    BackEndBase::cleanupHook();
}

/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Visualization.hpp"

using namespace rover_localization;

Visualization::Visualization(std::string const& name, TaskCore::TaskState initial_state)
    : VisualizationBase(name, initial_state)
{
}

Visualization::Visualization(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : VisualizationBase(name, engine, initial_state)
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
    return true;
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

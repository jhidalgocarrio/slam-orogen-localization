/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Dispatcher.hpp"
#include <base/Logging.hpp>



using namespace localization;

Dispatcher::Dispatcher(std::string const& name, TaskCore::TaskState initial_state)
    : DispatcherBase(name, initial_state)
{
}

Dispatcher::Dispatcher(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DispatcherBase(name, engine, initial_state)
{
}

Dispatcher::~Dispatcher()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Dispatcher.hpp for more detailed
// documentation about them.

bool Dispatcher::configureHook()
{
    if (! DispatcherBase::configureHook())
        return false;

    clearPorts(); // make sure all created ports are removed first, in case we aborted one configureHook already
    config = _outputs.get();
    dispatcher.resize(config.size());


    for (size_t i = 0; i < config.size(); ++i)
    {
        /** Create the output port **/
        OutputPortsConfiguration const& conf(config[i]);
        if (getPort(conf.output_name))
        {
            LOG_ERROR("output port %s is listed more than once in the outputs configuration property", conf.output_name.c_str());
            clearPorts();
            return false;
        }
        OutputPort* port = new OutputPort(conf.output_name);
        mOutputPorts.push_back(port);
        addPort(*port);

        /** Create the input ports **/
        {
            /** Create the rigid body state **/
            if (!getPort(conf.delta_pose_name))
            {
                InputPortPose* port = new InputPortPose(conf.delta_pose_name);
                mInputPose.push_back(port);
                addEventPort(*port);
            }

            /** Create the Jacobian **/
            if (!getPort(conf.jacobian_k_name))
            {
                InputPortJacob* port = new InputPortJacob(conf.jacobian_k_name);
                mInputJacobk.push_back(port);
                addEventPort(*port);
            }

            /** Create the Jacobian **/
            if (!getPort(conf.jacobian_k_m_name))
            {
                InputPortJacob* port = new InputPortJacob(conf.jacobian_k_m_name);
                mInputJacobk_m.push_back(port);
                addEventPort(*port);
            }

            /** Create the Covariance **/
            if (!getPort(conf.covariance_name))
            {
                InputPortCov* port = new InputPortCov(conf.covariance_name);
                mInputCov.push_back(port);
                addEventPort(*port);
            }
        }

        /** Create dispatcher named vector **/
        {
            dispatcher.names[i] = conf.output_name;
        }
    }




    return true;
}

bool Dispatcher::startHook()
{
    if (! DispatcherBase::startHook())
        return false;
    return true;
}
void Dispatcher::updateHook()
{
    DispatcherBase::updateHook();
    base::NamedVector<base::samples::RigidBodyState> delta_pose_namedvector;
    base::NamedVector<base::MatrixXd> jacobian_k_namedvector;
    base::NamedVector<base::MatrixXd> jacobian_k_m_namedvector;
    base::NamedVector<base::MatrixXd> covariance_namedvector;

    delta_pose_namedvector.resize(config.size());
    jacobian_k_namedvector.resize(config.size());
    jacobian_k_m_namedvector.resize(config.size());
    covariance_namedvector.resize(config.size());

    /** Check input port samples for delta displacements **/
    for (register size_t i = 0; i < mInputPose.size(); ++i)
    {
        base::samples::RigidBodyState rbs;
        while (mInputPose[i]->read(rbs, false) == RTT::NewData)
        {
            delta_pose_namedvector.names[i] = mInputPose[i]->getName();
            delta_pose_namedvector.elements[i] = rbs;
        }
    }


    /** Check input port samples for Jacobian **/
    for (register size_t i = 0; i < mInputJacobk.size(); ++i)
    {
        base::MatrixXd jacob;
        while (mInputJacobk[i]->read(jacob, false) == RTT::NewData)
        {
            jacobian_k_namedvector.names[i] = mInputJacobk[i]->getName();
            jacobian_k_namedvector.elements[i] = jacob;
        }
    }

    /** Check input port samples for Jacobian **/
    for (register size_t i = 0; i < mInputJacobk_m.size(); ++i)
    {
        base::MatrixXd jacob;
        while (mInputJacobk_m[i]->read(jacob, false) == RTT::NewData)
        {
            jacobian_k_m_namedvector.names[i] = mInputJacobk_m[i]->getName();
            jacobian_k_m_namedvector.elements[i] = jacob;
        }
    }

    /** Check input port samples for Covariance **/

    /** Create Exteroceptive samples **/
    for (register size_t i = 0; i < config.size(); ++i)
    {
        localization::ExteroceptiveSample extero_sample;
        OutputPortsConfiguration const& conf(config[i]);

        extero_sample.delta_pose = delta_pose_namedvector[conf.delta_pose_name];
        extero_sample.jacobian_k = jacobian_k_namedvector[conf.jacobian_k_name];
        extero_sample.jacobian_k_m = jacobian_k_m_namedvector[conf.jacobian_k_m_name];
        extero_sample.covariance = covariance_namedvector[conf.covariance_name];

        dispatcher[conf.output_name] = extero_sample;
    }


    /** Exteroceptive Samples to output ports **/
    for (register size_t i = 0; i < mOutputPorts.size(); ++i)
    {
        localization::ExteroceptiveSample extero_sample = dispatcher[mOutputPorts[i]->getName()];
        mOutputPorts[i]->write(extero_sample);
    }
}
void Dispatcher::errorHook()
{
    DispatcherBase::errorHook();
}
void Dispatcher::stopHook()
{
    DispatcherBase::stopHook();
}
void Dispatcher::cleanupHook()
{
    DispatcherBase::cleanupHook();
}

void Dispatcher::clearPorts()
{

    /** Input ports delta displacements **/
    for (size_t i = 0; i < mInputPose.size(); ++i)
    {
        ports()->removePort(mInputPose[i]->getName());
        delete mInputPose[i];
    }
    mInputPose.clear();

    /** Input ports Jacobian **/
    for (size_t i = 0; i < mInputJacobk.size(); ++i)
    {
        ports()->removePort(mInputJacobk[i]->getName());
        delete mInputJacobk[i];
    }
    mInputJacobk.clear();

    /** Input ports Jacobian **/
    for (size_t i = 0; i < mInputJacobk_m.size(); ++i)
    {
        ports()->removePort(mInputJacobk_m[i]->getName());
        delete mInputJacobk_m[i];
    }
    mInputJacobk_m.clear();

    /** Input ports Covariance **/
    for (size_t i = 0; i < mInputCov.size(); ++i)
    {
        ports()->removePort(mInputCov[i]->getName());
        delete mInputCov[i];
    }
    mInputCov.clear();

    /** Output ports **/
    for (size_t i = 0; i < mOutputPorts.size(); ++i)
    {
        ports()->removePort(mOutputPorts[i]->getName());
        delete mOutputPorts[i];
    }
    mOutputPorts.clear();
}


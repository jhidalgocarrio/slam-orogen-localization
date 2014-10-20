/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Dispatcher.hpp"
#include <base/Logging.hpp>

//#define DEBUG_PRINTS 1

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

    for (size_t i = 0; i < config.size(); ++i)
    {
        /** Create the output port **/
        OutputPortsConfiguration const& conf(config[i]);
        if (getPort(conf.output_name))
        {
            RTT::log(RTT::Error)<<"[LOCALIZATION_DISPATCHER] output port" << conf.output_name <<"is listed more than once in the outputs configuration property\n";
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
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.delta_pose_name<<"\n";
                InputPortPose* port = new InputPortPose(conf.delta_pose_name);
                mInputPose.push_back(port);
                addEventPort(*port);
            }

            /** Create the Point Cloud **/
            if (!getPort(conf.pointcloud_name))
            {
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.pointcloud_name<<"\n";
                InputPortPointcloud* port = new InputPortPointcloud(conf.pointcloud_name);
                mInputPointcloud.push_back(port);
                addEventPort(*port);
            }

            /** Create the Covariance **/
            if (!getPort(conf.covariance_name))
            {
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.covariance_name<<"\n";
                InputPortCov* port = new InputPortCov(conf.covariance_name);
                mInputCov.push_back(port);
                addEventPort(*port);
            }

            /** Create the Jacobian **/
            if (!getPort(conf.jacobian_k_name))
            {
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.jacobian_k_name<<"\n";
                InputPortJacob* port = new InputPortJacob(conf.jacobian_k_name);
                mInputJacobk.push_back(port);
                addEventPort(*port);
            }

            /** Create the Jacobian **/
            if (!getPort(conf.jacobian_k_m_name))
            {
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.jacobian_k_m_name<<"\n";
                InputPortJacob* port = new InputPortJacob(conf.jacobian_k_m_name);
                mInputJacobk_m.push_back(port);
                addEventPort(*port);
            }

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


    /** Check input port samples for delta displacements **/
    for (register size_t i = 0; i < mInputPose.size(); ++i)
    {
        base::samples::RigidBodyState rbs;
        while (mInputPose[i]->read(rbs, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputPose[i]->getName();
            std::cout<<" at time "<< rbs.time.toString()<<"\n";
            #endif
            dispatcher.delta_pose.names.push_back(mInputPose[i]->getName());
            dispatcher.delta_pose.elements.push_back(rbs);
        }
    }

    /** Check input port samples for Point Cloud **/
    for (register size_t i = 0; i < mInputPointcloud.size(); ++i)
    {
        base::samples::Pointcloud point_cloud;
        while (mInputPointcloud[i]->read(point_cloud, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputPointcloud[i]->getName()<<"\n";
            #endif
            dispatcher.point_cloud.names.push_back(mInputPointcloud[i]->getName());
            dispatcher.point_cloud.elements.push_back(point_cloud);
        }
    }

    /** Check input port samples for Covariance **/
    for (register size_t i = 0; i < mInputCov.size(); ++i)
    {
        std::vector<base::Matrix3d> cov;
        while (mInputCov[i]->read(cov, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputCov[i]->getName()<<"\n";
            #endif
            dispatcher.covariance.names.push_back(mInputCov[i]->getName());
            dispatcher.covariance.elements.push_back(cov);
        }
    }


    /** Check input port samples for Jacobian **/
    for (register size_t i = 0; i < mInputJacobk.size(); ++i)
    {
        base::MatrixXd jacob;
        while (mInputJacobk[i]->read(jacob, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputJacobk[i]->getName()<<"\n";
            #endif
            dispatcher.jacobian_k.names.push_back(mInputJacobk[i]->getName());
            dispatcher.jacobian_k.elements.push_back(jacob);
        }
    }

    /** Check input port samples for Jacobian **/
    for (register size_t i = 0; i < mInputJacobk_m.size(); ++i)
    {
        base::MatrixXd jacob;
        while (mInputJacobk_m[i]->read(jacob, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputJacobk_m[i]->getName()<<"\n";
            #endif
            dispatcher.jacobian_k_m.names.push_back(mInputJacobk_m[i]->getName());
            dispatcher.jacobian_k_m.elements.push_back(jacob);
        }
    }


    /** Create Exteroceptive samples **/
    for (register size_t i = 0; i < config.size(); ++i)
    {
        localization::ExteroceptiveSample extero_sample;
        OutputPortsConfiguration const& conf(config[i]);

        #ifdef DEBUG_PRINTS
        std::cout<<"dispatcher.delta_pose: "<<dispatcher.delta_pose.size()<<"\n";
        std::cout<<"dispatcher.point_cloud: "<<dispatcher.point_cloud.size()<<"\n";
        std::cout<<"dispatcher.covariance: "<<dispatcher.covariance.size()<<"\n";
        std::cout<<"dispatcher.jacobian_k: "<<dispatcher.jacobian_k.size()<<"\n";
        std::cout<<"dispatcher.jacobian_k_m: "<<dispatcher.jacobian_k_m.size()<<"\n";
        #endif

        if (dispatcher.delta_pose.hasNames() &&
            dispatcher.point_cloud.hasNames() &&
            dispatcher.covariance.hasNames() &&
            dispatcher.jacobian_k.hasNames() &&
            dispatcher.jacobian_k_m.hasNames())
        {
            try
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"creating output for: "<<conf.output_name<<"\n";
                #endif
                extero_sample.delta_pose = dispatcher.delta_pose[conf.delta_pose_name];
                extero_sample.point_cloud = dispatcher.point_cloud[conf.pointcloud_name];
                extero_sample.covariance = dispatcher.covariance[conf.covariance_name];
                extero_sample.jacobian_k = dispatcher.jacobian_k[conf.jacobian_k_name];
                extero_sample.jacobian_k_m = dispatcher.jacobian_k_m[conf.jacobian_k_m_name];


                /** Exteroceptive Samples to output ports **/
                #ifdef DEBUG_PRINTS
                std::cout<<"sending: "<<mOutputPorts[i]->getName();
                std::cout<<" at time "<< extero_sample.delta_pose.time.toString()<<"\n";
                #endif
                mOutputPorts[i]->write(extero_sample);

                /** Delete the according entries **/
                dispatcher.erase(conf.delta_pose_name, conf.pointcloud_name, conf.covariance_name, conf.jacobian_k_name, conf.jacobian_k_m_name);

            } catch (std::runtime_error)
            {
                RTT::log(RTT::Error)<<"[LOCALIZATION_DISPATCHER] Missing inputs samples\n";
                break;
            }
        }
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
    /** Clean ports **/
    clearPorts();

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

    /** Input ports Point Cloud **/
    for (size_t i = 0; i < mInputPointcloud.size(); ++i)
    {
        ports()->removePort(mInputPointcloud[i]->getName());
        delete mInputPointcloud[i];
    }
    mInputPointcloud.clear();

    /** Input ports Covariance **/
    for (size_t i = 0; i < mInputCov.size(); ++i)
    {
        ports()->removePort(mInputCov[i]->getName());
        delete mInputCov[i];
    }
    mInputCov.clear();

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

    /** Output ports **/
    for (size_t i = 0; i < mOutputPorts.size(); ++i)
    {
        ports()->removePort(mOutputPorts[i]->getName());
        delete mOutputPorts[i];
    }
    mOutputPorts.clear();
}


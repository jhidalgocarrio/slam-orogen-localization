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
            /** Create the Index **/
            if (!getPort(conf.index_name))
            {
                RTT::log(RTT::Warning)<<"[LOCALIZATION_DISPATCHER] Create input port: "<< conf.index_name<<"\n";
                InputPortIdx* port = new InputPortIdx(conf.index_name);
                mInputIdx.push_back(port);
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

    /** Check input port samples for Indexes **/
    for (register size_t i = 0; i < mInputIdx.size(); ++i)
    {
        std::vector<boost::uuids::uuid> idx;
        while (mInputIdx[i]->read(idx, false) == RTT::NewData)
        {
            #ifdef DEBUG_PRINTS
            std::cout<<"received: "<<mInputIdx[i]->getName()<<"\n";
            #endif
            dispatcher.index.names.push_back(mInputIdx[i]->getName());
            dispatcher.index.elements.push_back(idx);
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

    /** Create Exteroceptive samples **/
    for (register size_t i = 0; i < config.size(); ++i)
    {
        ExteroPort extero_samples;
        OutputPortsConfiguration const& conf(config[i]);

        #ifdef DEBUG_PRINTS
        std::cout<<"dispatcher.index: "<<dispatcher.index.size()<<"\n";
        std::cout<<"dispatcher.point_cloud: "<<dispatcher.point_cloud.size()<<"\n";
        std::cout<<"dispatcher.covariance: "<<dispatcher.covariance.size()<<"\n";
        #endif

        if (dispatcher.index.hasNames() &&
            dispatcher.point_cloud.hasNames() &&
            dispatcher.covariance.hasNames())
        {
            try
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"creating output for: "<<conf.output_name<<"\n";
                #endif
                extero_samples.time = dispatcher.point_cloud[conf.pointcloud_name].time;
                extero_samples.features.resize(dispatcher.index[conf.index_name].size());

                /** Store the values **/
                std::vector<boost::uuids::uuid> vector_uuids = dispatcher.index[conf.index_name];
                std::vector<base::Vector3d> vector_points = dispatcher.point_cloud[conf.pointcloud_name].points;
                std::vector<base::Matrix3d> vector_cov = dispatcher.covariance[conf.covariance_name];

                std::vector<ExteroFeature>::iterator it_feature = extero_samples.features.begin();
                std::vector<boost::uuids::uuid>::iterator it_uuid = vector_uuids.begin();
                std::vector<base::Vector3d>::iterator it_point = vector_points.begin();
                std::vector<base::Matrix3d>::iterator it_cov = vector_cov.begin();

                for(; it_feature != extero_samples.features.end();
                        ++it_feature, ++it_uuid, ++it_point, ++it_cov)
                {
                    it_feature->index = *it_uuid;
                    it_feature->point = *it_point;
                    it_feature->cov = *it_cov;
                }

                /** Exteroceptive Samples to output ports **/
                #ifdef DEBUG_PRINTS
                std::cout<<"sending: "<<mOutputPorts[i]->getName();
                std::cout<<" at time "<< extero_samples.time.toString()<<"\n";
                #endif
                mOutputPorts[i]->write(extero_samples);

                /** Delete the according entries **/
                dispatcher.erase(conf.index_name, conf.pointcloud_name, conf.covariance_name);

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

    /** Input ports Index **/
    for (size_t i = 0; i < mInputIdx.size(); ++i)
    {
        ports()->removePort(mInputIdx[i]->getName());
        delete mInputIdx[i];
    }
    mInputIdx.clear();

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

    /** Output ports **/
    for (size_t i = 0; i < mOutputPorts.size(); ++i)
    {
        ports()->removePort(mOutputPorts[i]->getName());
        delete mOutputPorts[i];
    }
    mOutputPorts.clear();
}


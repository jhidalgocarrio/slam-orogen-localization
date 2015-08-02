/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace localization;

/** Process model when accumulating delta poses **/
WSingleState processModel (const WSingleState &state,  const Eigen::Vector3d &delta_position, const localization::SO3 &delta_orientation)
{
    WSingleState s2; /** Propagated state */

    /** Apply Rotation **/
    s2.orient = state.orient * delta_orientation;

    /** Apply Translation **/
    s2.pos = state.pos + (s2.orient * delta_position);

    return s2;
};


MeasurementType measurementModel(const WMultiState &wstate, const std::vector< std::pair<unsigned int, Eigen::Vector3d> > &observation)
{
    MeasurementType z_hat;
    z_hat.resize(2*observation.size(), 1);

    unsigned int measurement_counts = 0;

    std::vector< std::pair<unsigned int, Eigen::Vector3d> >::const_iterator it_observation = observation.begin();
    for(; it_observation != observation.end(); ++it_observation)
    {
        std::pair<unsigned int, Eigen::Vector3d> const &entry(*it_observation);
        localization::SensorState const &st_pose(wstate.sensorsk[entry.first]);

        /** Compute the observation **/
        Eigen::Vector3d feature_pos_in_camera = st_pose.orient.inverse() * (entry.second - st_pose.pos);
        z_hat.block(2*measurement_counts, 0, 2, 1) = Eigen::Vector2d(feature_pos_in_camera.x()/feature_pos_in_camera.z(),feature_pos_in_camera.y()/feature_pos_in_camera.z());
        measurement_counts++;

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"st_pose.pos:\n"<<st_pose.pos<< RTT::endlog();
        RTT::log(RTT::Warning)<<"feature 3d point wrt camera:\n"<<feature_pos_in_camera<< RTT::endlog();
        RTT::log(RTT::Warning)<<"feature 2d point wrt camera:\n"<<Eigen::Vector2d(feature_pos_in_camera.x()/feature_pos_in_camera.z(),feature_pos_in_camera.y()/feature_pos_in_camera.z())<< RTT::endlog();
        #endif

    }

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[MEASUREMENT OBSERVATION MODEL] Number processed measurements: "<<measurement_counts<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT OBSERVATION MODEL] z_hat.size(): "<<z_hat.size()<< RTT::endlog();
    #endif

    return z_hat;
};



Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{

    /******************************/
    /*** Control Flow Variables ***/
    /******************************/
    initFilter = false;

    /**************************/
    /** Input port variables **/
    /**************************/
    delta_pose.invalidate();
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::BodyState &delta_pose_samples_sample)
{

    if(!initFilter)
    {
        /***************************/
        /** Filter Initialization **/
        /***************************/

        Eigen::Affine3d tf; /** Transformer transformation **/

        /** Get the transformation **/
        if (_navigation_frame.value().compare(_world_frame.value()) == 0)
        {
            tf.setIdentity();
        }
        else if (!_navigation2world.get(ts, tf, false))
        {
           RTT::log(RTT::Fatal)<<"[LOCALIZATION FATAL ERROR]  No transformation provided."<<RTT::endlog();
           return;
        }

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[LOCALIZATION POSE_SAMPLES] - Initializing Filter..."<<RTT::endlog();

        #endif

        /** Initialization of the filter **/
        this->initMultiStateFilter (filter, tf);

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[DONE]\n";
        #endif

        initFilter = true;
    }

    #ifdef DEBUG_PRINTS
    base::Time delta_t = delta_pose_samples_sample.time - this->delta_pose.time;
    //base::Time delta_t = base::Time::fromSeconds(_pose_samples_period.get());
    RTT::log(RTT::Warning)<<"[LOCALIZATION POSE_SAMPLES] Received new samples at "<<delta_pose_samples_sample.time.toString()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION POSE_SAMPLES] delta_t: "<<delta_t.toSeconds()<<RTT::endlog();
    #endif

    /** A new sample arrived to the input port **/
    this->delta_pose = delta_pose_samples_sample;

    /********************/
    /** Filter Predict **/
    /********************/

    /** Process Model Uncertainty **/
    typedef MultiStateFilter::SingleStateCovariance SingleStateCovariance;
    SingleStateCovariance cov_process; cov_process.setIdentity();
    //cov_process = 1.e-26 * cov_process;
    MTK::subblock (cov_process, &WSingleState::pos, &WSingleState::pos) = this->delta_pose.cov_position();
    MTK::subblock (cov_process, &WSingleState::orient, &WSingleState::orient) = this->delta_pose.cov_orientation();

    /** Predict the filter state **/
    filter->predict(boost::bind(processModel, _1 ,
                            static_cast<const Eigen::Vector3d>(delta_pose.position()),
                            static_cast<const localization::SO3>(Eigen::Quaterniond(delta_pose.orientation()))),
                            cov_process);

    this->outputPortSamples(delta_pose.time);
}

void Task::visual_features_samplesTransformerCallback(const base::Time &ts, const ::localization::ExteroFeatures &visual_features_samples_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/

    /** Get the transformation (transformation) Tbody_sensor **/
    if (_body_frame.value().compare(_sensor_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_sensor2body.get(ts, tf, false))
    {
        throw std::runtime_error("[THREED_ODOMETRY] Transformation from imu to body is not provided.");
        return;
    }


    /** Perform Measurements Update **/
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] Received sample: "<<visual_features_samples_sample.img_idx<<" at time "<< visual_features_samples_sample.time.toString()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] Number of Measurements: "<< visual_features_samples_sample.features.size()<<RTT::endlog();
    #endif

    if (initFilter)
    {
        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] mu_state\n"<<filter->muState()<<RTT::endlog();
        RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] Pk is of size "<<filter->getPk().rows()<<" x "<<filter->getPk().cols()<<RTT::endlog();
        RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] Pk\n"<<filter->getPk()<<RTT::endlog();
        #endif

        if (filter->muState().sensorsk.size() == _maximum_number_sensor_poses.value())
        {
            /** Observation vector for cameras and features 3d positions **/
            std::vector< std::pair<unsigned int, Eigen::Vector3d> > observation_vector;

            /** Compute the measurement vector and covariance of the current features **/
            Eigen::Matrix<MultiStateFilter::ScalarType, Eigen::Dynamic, Eigen::Dynamic> measurementCov;
            MeasurementType measurement = this->measurementVector(this->envire_tree,
                                                                this->camera_node_labels,
                                                                observation_vector,
                                                                measurementCov);

            /** Function of the measurement **/
            typedef boost::function<MeasurementType (WMultiState &)> MeasurementFunction;
            MeasurementFunction f = boost::bind(measurementModel, _1, boost::cref(observation_vector));

            /** Perform an update when reached maximum number of camera sensor poses **/
            filter->update(static_cast< Eigen::Matrix<MultiStateFilter::ScalarType, Eigen::Dynamic, 1> > (measurement), f, measurementCov);

            //MeasurementType measurement_hat = f(this->filter->muState());

            /** Remove sensor pose from filter **/
            unsigned int it_removed_pose = this->removeSensorPoseFromFilter(filter);

            /** Remove sensor pose and measurements from envire **/
            this->removeSensorPoseFromEnvire(this->envire_tree, it_removed_pose);

        }

        /** New sensor camera pose in the filter**/
        localization::SensorState camera_pose = this->addSensorPoseToFilter(filter, tf);

        /** New measurement to envire **/
        std::string camera_pose_label = "camera_"+boost::lexical_cast<std::string>(visual_features_samples_sample.img_idx);
        this->camera_node_labels.push_back(camera_pose_label);
        this->addMeasurementToEnvire(this->envire_tree, camera_pose_label, camera_pose, visual_features_samples_sample);

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] PRINT FILTER -> ENVIRE"<<RTT::endlog();
        std::vector<std::string>::iterator it = this->camera_node_labels.begin();
        for (; it != this->camera_node_labels.end(); ++it)
        {
            RTT::log(RTT::Warning)<<"\n"<<it-this->camera_node_labels.begin()<<" -> "<<*it;
        }
        RTT::log(RTT::Warning)<<RTT::endlog();
        envire::core::GraphViz gviz;
        gviz.write(envire_tree.graph(), "envire_tree_"+camera_pose_label+".dot");
        #endif

    }

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION VISUAL_FEATURES] **** END ****"<<RTT::endlog();
    #endif
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /***********************/
    /** Configure Values  **/
    /***********************/

    initFilter = false;

    /** Output port **/
    pose_out.invalidate();
    pose_out.sourceFrame = _localization_source_frame.value();

    /** Relative Frame to port out the samples **/
    pose_out.targetFrame = _world_frame.value();

    /** Initialize features covariance **/
    this->feature_cov = _measurement_covariance.value().block(0, 0, 2, 2);

    /***********************/
    /** Info and Warnings **/
    /***********************/
    RTT::log(RTT::Warning)<<"[LOCALIZATION TASK] Desired Target Frame is "<<pose_out.targetFrame<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION TASK] Maximum number of camera poses: "<<_maximum_number_sensor_poses.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION TASK] Features measurement covariance:\n"<<this->feature_cov<<RTT::endlog();

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

    /** Liberate the memory of the shared_ptr **/
    filter.reset();

    /** Remove the content in the envire tree **/
    envire_tree.clear();
}

void Task::initMultiStateFilter(boost::shared_ptr<MultiStateFilter> &filter, Eigen::Affine3d &tf)
{
    /** The filter vector state variables for the navigation quantities **/
    WMultiState statek_0;
    WSingleState single_state;

    /** Set the current pose to initialize the filter structure **/
    single_state.pos = tf.translation(); //!Initial position
    single_state.orient = Eigen::Quaternion<double>(tf.rotation());

    /** Store the single state into the state vector **/
    statek_0.statek = single_state;

    /************************************/
    /** Initialize the Back-End Filter **/
    /************************************/

    /** Initial covariance matrix **/
    MultiStateFilter::SingleStateCovariance P0_single; /** Initial P(0) for the state **/
    MultiStateCovariance P0;
    P0_single.setZero();
    P0.resize(WSingleState::DOF, WSingleState::DOF);
    P0.setIdentity();

    MTK::setDiagonal (P0_single, &WSingleState::pos, 1e-06);
    MTK::setDiagonal (P0_single, &WSingleState::orient, 1e-06);

    P0.block(0, 0, WSingleState::DOF, WSingleState::DOF) = P0_single;

    /** Create the filter **/
    filter.reset (new MultiStateFilter (statek_0, P0));

    #ifdef DEBUG_PRINTS
    WMultiState vstate = filter->muState();
    RTT::log(RTT::Warning)<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] State P0|0 is of size " <<P0_single.rows()<<" x "<<P0_single.cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] State P0|0:\n"<<P0_single<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] Multi State P0|0 is of size " <<filter->getPk().rows()<<" x "<<filter->getPk().cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] Multi State P0|0:\n"<<filter->getPk()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] state:\n"<<vstate.getVectorizedState()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] position:\n"<<vstate.statek.pos<< RTT::endlog();
    Eigen::Vector3d euler; /** In Euler angles **/
    euler[2] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
    euler[1] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
    euler[0] = vstate.statek.orient.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
    RTT::log(RTT::Warning)<<"[LOCALIZATION INIT] orientation Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<< RTT::endlog();
    RTT::log(RTT::Warning)<< RTT::endlog();
    #endif

    return;
}


void Task::outputPortSamples(const base::Time &timestamp)
{
    WSingleState statek_i = filter->muState().statek;

    pose_out.time = timestamp;
    pose_out.position = statek_i.pos;
    pose_out.cov_position = filter->getPkSingleState().block<3,3>(0,0);
    pose_out.orientation = statek_i.orient;
    pose_out.cov_orientation = filter->getPkSingleState().block<3,3>(3,3);
    _pose_samples_out.write(pose_out);

}


localization::SensorState Task::addSensorPoseToFilter(boost::shared_ptr<MultiStateFilter> filter, Eigen::Affine3d &tf)
{
    Eigen::Quaternion <double> qtf = Eigen::Quaternion <double> (tf.rotation());

    /** Create the camera pose **/
    localization::SensorState camera_pose;
    camera_pose.orient = filter->muSingleState().orient * qtf;//Tnav_camera = Tnav_body * Tbody_camera
    Eigen::Vector3d pbody_camera = filter->muSingleState().orient * tf.translation(); //pbody_camera in navigation frame
    camera_pose.pos = filter->muSingleState().pos + pbody_camera; //pnav_camera = pnav_body + Tnav_camera * pbody_camera

    /** Jacobian matrix for new sensor camera pose estimation **/
    MultiStateCovariance J (filter->muState().getDOF() + localization::SensorState::DOF, filter->muState().getDOF()); J.setZero();
    J.block(0, 0, filter->muState().getDOF(), filter->muState().getDOF()) = Eigen::MatrixXd::Identity(filter->muState().getDOF(), filter->muState().getDOF());

    /** Identity in position **/
    J.block(filter->muState().getDOF(), 0, localization::vec3::DOF, localization::vec3::DOF) = Eigen::MatrixXd::Identity(localization::vec3::DOF, localization::vec3::DOF);

    /** Skew symmetric matrix with orientation of the state **/
    J.block(filter->muState().getDOF(), localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF) = Task::makeSkewSymmetric(pbody_camera);

    /* Sensor orientation **/
    J.block(filter->muState().getDOF()+localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF, localization::vec3::DOF) = tf.rotation();

    /** Extend state by new camera pose **/
    filter->muState().sensorsk.push_back(camera_pose);

    /** Extend filter covariance matrix **/
    MultiStateCovariance Pk_i (filter->muState().getDOF(), filter->muState().getDOF());

    /** Multiplication **/
    Pk_i = J * filter->getPk() * J.transpose();

    /** Store the new Pk in the filter **/
    filter->setPk(Pk_i);

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_SENSOR_POSE] J is of size "<<J.rows()<<" x "<<J.cols()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_SENSOR_POSE] J\n"<<J<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_SENSOR_POSE] Pk is of size "<<filter->getPk().rows()<<" x "<<filter->getPk().cols()<< RTT::endlog();
    #endif

    return camera_pose;
}


unsigned int Task::removeSensorPoseFromFilter(boost::shared_ptr<MultiStateFilter> filter)
{
    MultiStateCovariance Pk_i (filter->muState().getDOF(), filter->muState().getDOF());
    Pk_i = filter->getPk();

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_SENSOR_POSE] Sensor size: "<<filter->muState().sensorsk.size()<<" == maximum("<<_maximum_number_sensor_poses.value()<<")\n";
    RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_SENSOR_POSE] Pk_i is of size "<<Pk_i.rows()<<" x "<<Pk_i.cols()<< RTT::endlog();
    #endif

    /**Remove in the state **/
    std::srand( time( NULL ) );  //  using the time seed from srand explanation
    unsigned int it_dice = std::rand() % (filter->muState().sensorsk.size());
    filter->muState().sensorsk.erase(filter->muState().sensorsk.begin() + it_dice);

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_SENSOR_POSE] it_dice: "<<it_dice<< RTT::endlog();
    #endif

    /**Remove in the covariance matrix **/
    unsigned int i_remove = WSingleState::DOF + (localization::SensorState::DOF * it_dice);
    for (register size_t i = 0; i < localization::SensorState::DOF; ++i)
    {
        //RTT::log(RTT::Warning)<<"i -> "<<i<<" i_remove: "<<i_remove<< RTT::endlog();

        /** Remove row and column **/
        Task::removeRow(Pk_i, i_remove);
        Task::removeColumn(Pk_i, i_remove);
        //RTT::log(RTT::Warning)<<"Pk_i is of size "<<Pk_i.rows()<<" x "<<Pk_i.cols()<< RTT::endlog();
    }

    filter->setPk(Pk_i);

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_SENSOR_POSE] After remove Sensor size: "<<filter->muState().sensorsk.size()<<" == maximum("<<_maximum_number_sensor_poses.value()<<")\n";
    RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_SENSOR_POSE] After remove Pk_i is of size "<<Pk_i.rows()<<" x "<<Pk_i.cols()<< RTT::endlog();
    #endif


    return it_dice;
}

void Task::addMeasurementToEnvire(envire::core::LabeledTransformTree &envire_tree,
                            const std::string &camera_pose_label,
                            const ::localization::SensorState &camera_pose,
                            const ::localization::ExteroFeatures &samples)
{

    /** Create the camera node in the envire tree **/
    envire::core::Frame camera_node(camera_pose_label);

    /** Compute the measurement item. Measurement of the features from the camera **/
    std::vector<Feature>::const_iterator it_feature = samples.features.begin();
    for ( ; it_feature != samples.features.end(); ++it_feature)
    {
        Eigen::Matrix<double, 2, 3> projection;
        projection << 1.0/it_feature->point.z(), 0.00, 0.00,
                    0.00, 1.0/it_feature->point.z(), 0.00;

        FeatureMeasurement m(it_feature->index);
        m.point = projection * it_feature->point;

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] UUID:\n"<<boost::uuids::to_string(it_feature->index)<< RTT::endlog();
        RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] 3D Point:\n"<<it_feature->point<< RTT::endlog();
        RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] 2D Point:\n"<<m.point<< RTT::endlog();
        #endif
        boost::intrusive_ptr<MeasurementItem> feature_item(new MeasurementItem());
        feature_item->setData(m);
        camera_node.items.push_back(feature_item);
    }

    /** Include camera on the envire tree **/
    std::pair<envire::core::LabeledTransformTree::vertex_descriptor, bool> camera_pair = envire_tree.insert_vertex(camera_node);
    //if (!camera_pair.second)
    //    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] ERROR!! Camera node("<<camera_pair.second<<")\n";
    //else
    //    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] Camera node with Item vector size: "<<camera_node.items.size()<< RTT::endlog();

    it_feature = samples.features.begin();
    for ( ; it_feature != samples.features.end(); ++it_feature)
    {
        /** Feature Pose w.r.t the global frame **/
        Eigen::Vector3d feature_pos;
        feature_pos = camera_pose.pos + camera_pose.orient * it_feature->point;

        /** Fill the Feature information **/
        envire::core::Frame feature_node(boost::uuids::to_string(it_feature->index));
        feature_node.uuid = it_feature->index;
        boost::intrusive_ptr<FeaturePositionItem> pos_item(new FeaturePositionItem());
        pos_item->setData(feature_pos);
        feature_node.items.push_back(pos_item);

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] Feature("<<feature_node.name<<") Point in navigation:\n"<<feature_pos<< RTT::endlog();
        RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] Stored position:\n"<<pos_item->getData()<< RTT::endlog();
        #endif

        /** Feature to the environment tree **/
        std::pair<envire::core::LabeledTransformTree::vertex_descriptor, bool> feature_pair = envire_tree.insert_vertex(feature_node);
        //if (!feature_pair.second)
        //    RTT::log(RTT::Warning)<<"[LOCALIZATION ADD_MEASUREMENT_ENVIRE] ERROR!! Feature node("<<feature_pair.second<<")\n";

        /** Edge camera node -> it_feature **/
        envire::core::Transform camera_to_feature(samples.time);
        base::TransformWithCovariance tf(Eigen::AngleAxisd::Identity(), static_cast<base::Position>(it_feature->point));
        Eigen::Matrix<double, 6, 6> tf_cov;
        tf_cov << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), it_feature->cov;
        tf.setCovariance(tf_cov);
        camera_to_feature.setTransform(tf);
        envire::core::LabeledTransformTree::edge_descriptor edge; bool b;
        boost::tie(edge, b) = envire_tree.add_edge(camera_node.name, feature_node.name, camera_to_feature);
        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning)<<"ADDED EDGE("<<b<<") "<<camera_node.name<<" -> "<<feature_node.name<< RTT::endlog();
        #endif
    }
}


void Task::removeSensorPoseFromEnvire(envire::core::LabeledTransformTree &envire_tree, const unsigned int it_removed_pose)
{
    /** Get the camera pose node **/
    envire::core::LabeledTransformTree::vertex_descriptor camera_node = envire_tree.vertex(this->camera_node_labels[it_removed_pose]);
    //RTT::log(RTT::Warning)<<"[LOCALIZATION REMOVE_MEASUREMENT_ENVIRE] Got camera_node with Item vector size: "<<envire_tree.getFrame(camera_node).items.size()<< RTT::endlog();

    /** Provides iterators to iterate over the out-going edges of camera node **/
    std::vector<std::string> features_to_remove;
    envire::core::TransformTree::out_edge_iterator ei, ei_end, enext;
    boost::tie(ei, ei_end) = envire_tree.out_edges(camera_node);
    for (enext = ei; ei != ei_end; ei = enext)
    {
        envire::core::LabeledTransformTree::vertex_descriptor source = envire_tree.source(*ei);
        envire::core::LabeledTransformTree::vertex_descriptor target = envire_tree.target(*ei);

        #ifdef DEBUG_PRINTS
        RTT::log(RTT::Warning) << "REMOVE EDGE " << envire_tree.getFrame(source).name <<
                            " -> " << envire_tree.getFrame(target).name << RTT::endlog();
        #endif

        ++enext;
        envire_tree.remove_edge(*ei, false);
        if (envire_tree.degree(target) == 0)
        {
            features_to_remove.push_back(envire_tree.getFrame(target).name);
        }
    }

    /** Erase the it_removed_pose entry in the vector **/
    this->camera_node_labels.erase(this->camera_node_labels.begin() + it_removed_pose);

    /** Clean tree **/
    for (std::vector<std::string>::iterator it_feature_label = features_to_remove.begin();
            it_feature_label != features_to_remove.end(); ++it_feature_label)
    {
        envire_tree.remove_vertex(*it_feature_label);
    }

    envire_tree.remove_vertex(envire_tree.getFrame(camera_node).name);


    return;
}

MeasurementType Task::measurementVector(envire::core::LabeledTransformTree &envire_tree,
        const std::vector<std::string> &camera_node_labels,
        std::vector< std::pair<unsigned int, Eigen::Vector3d> > &observation,
        Eigen::Matrix<MultiStateFilter::ScalarType, Eigen::Dynamic, Eigen::Dynamic> &cov)
{
    MeasurementType z;
    z.resize(2*envire_tree.num_edges(), 1);
    cov.resize(2*envire_tree.num_edges(), 2*envire_tree.num_edges());
    cov.setZero();
    observation.clear();

    unsigned int feature_counts = 0;
    unsigned int measurement_counts = 0;

    /** Iterate trough all vertices **/
    envire::core::TransformTree::vertex_iterator vi, vi_end, vnext;
    boost::tie(vi, vi_end) = envire_tree.vertices();
    for (vnext = vi; vi != vi_end; vi = vnext)
    {
        ++vnext;

        /** Get Frame **/
        envire::core::Frame f = envire_tree.getFrame(*vi);

        /** In case it is a feature node (no camera) **/
        if(f.name.find("camera") == std::string::npos)
        {
            /** Take the feature position **/
            boost::intrusive_ptr<FeaturePositionItem> f_pos_it =  boost::static_pointer_cast<FeaturePositionItem>(f.items[0]);

            /** Loop all the in edges to get the measurement from each camera **/
            std::pair<envire::core::LabeledTransformTree::in_edge_iterator,
                envire::core::LabeledTransformTree::in_edge_iterator> edges_pair = envire_tree.in_edges(*vi);

            /** Process all camera measurements of the feature **/
            while(edges_pair.first != edges_pair.second)
            {
                /** Look at the source of the edge **/
                envire::core::Frame camera_source = envire_tree.getFrame(envire_tree.source(*(edges_pair.first)));

                /** Look for camera index in filter **/
                std::vector<std::string>::const_iterator it_pose = std::find(camera_node_labels.begin(), camera_node_labels.end(), camera_source.name);
                if (it_pose != camera_node_labels.end())
                {
                    /** Fill the observation  **/
                    observation.push_back(std::pair<unsigned int, Eigen::Vector3d> (std::distance(camera_node_labels.begin(), it_pose), f_pos_it->getData()));
                }


                /** Look for the features unique index in all camera measurements **/
                std::vector< boost::intrusive_ptr<envire::core::ItemBase> >::const_iterator it_idx = camera_source.items.begin();
                for(; it_idx != camera_source.items.end(); ++it_idx)
                {
                    boost::intrusive_ptr<MeasurementItem> p_it = boost::static_pointer_cast<MeasurementItem>(*it_idx);
                    if (p_it->getData().index == f.uuid)
                    {
                        /** Fill the measurement **/
                        z.block(2*measurement_counts, 0, 2, 1) = p_it->getData().point;
                        cov.block(2*measurement_counts, 2*measurement_counts, 2, 2) = this->feature_cov;
                        measurement_counts++;

                        #ifdef DEBUG_PRINTS
                        RTT::log(RTT::Warning)<<"2D POINT\n"<<p_it->getData().point<< RTT::endlog();
                        #endif
                    }

                }
                ++edges_pair.first;
            }
            feature_counts++;
        }
    }

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] Number processed features: "<<feature_counts<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] Number processed measurements: "<<measurement_counts<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] envire_tree.num_vertices(): "<<envire_tree.num_vertices()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] envire_tree.num_edges(): "<<envire_tree.num_edges()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] z.size(): "<<z.size()<< RTT::endlog();
    RTT::log(RTT::Warning)<<"[MEASUREMENT VECTOR] observation.size(): "<<observation.size()<< RTT::endlog();
    #endif

    return z;
}


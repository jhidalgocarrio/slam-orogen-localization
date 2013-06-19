#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer/runtime'
require 'vizkit'
require 'utilrb'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs_asguard_localization.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

Orocos.transformer.load_conf('../config/transforms.rb')

# Configuration values
framework = {:proprio => false, :vicon => false, :proprio_vicon => false, :proprio_vicon_head => true, :viz => true, :envire => false}

# Asguard number of chain = #Trees * #ContactPointsPerTree
$number_chains = 20

Orocos.run 'rover_localization::FrontEnd' => 'frontend',  'rover_localization::BackEnd' => 'backend', 'rover_localization::Visualization' => 'visualization' do

    # log all the output ports
    Orocos.log_all_ports
    Orocos.conf.load_dir('../config/')

    # get the FrontEnd
    frontend = Orocos.name_service.get 'frontend'
    Orocos.conf.apply(frontend, ['default'], :override => true )

    # get the BackEnd
    backend = Orocos.name_service.get 'backend'

    if framework[:viz]
        # get the Visualization in Asynchronous mode
        visualization = Orocos::Async.proxy 'visualization'
    end

    # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )
    log_replay.transformer_broadcaster.rename("old_transformer_broadcaster")

    #mapping the logs into the inputs ports in the framework frontend
    if framework[:proprio]
	log_replay.propriocessing.hbridge_samples_out.connect_to(frontend.encoder_samples, :type => :buffer, :size => 200 )
	log_replay.propriocessing.systemstate_samples_out.connect_to(frontend.systemstate_samples, :type => :buffer, :size => 200 )
	log_replay.propriocessing.calibrated_sensors_out.connect_to(frontend.inertial_samples, :type => :buffer, :size => 200 )
    else
	log_replay.hbridge.status_motors.connect_to(frontend.encoder_samples, :type => :buffer, :size => 200 )
	log_replay.sysmon.system_status.connect_to(frontend.systemstate_samples, :type => :buffer, :size => 200 )
	log_replay.stim300.calibrated_sensors.connect_to(frontend.inertial_samples, :type => :buffer, :size => 200 )
    end

    if framework[:vicon]
	log_replay.vicon.pose_samples.connect_to(frontend.pose_init_samples, :type => :buffer, :size => 200 )
    elsif (framework[:proprio_vicon] || framework[:proprio_vicon_head])
	log_replay.propriocessing.pose_samples_out.connect_to(frontend.pose_init_samples, :type => :buffer, :size => 200 )
    end

    #mapping the frontend outports into the backend inports
    frontend.pose_samples_out.connect_to(backend.pose_samples, :type => :buffer, :size => 200)
    frontend.proprioceptive_samples_out.connect_to(backend.proprioceptive_samples, :type => :buffer, :size => 200)

    #Configure the frames names
    frontend.inertial_samples.frame = "stim300"

    if framework[:proprio_vicon_head]
	frontend.pose_init_samples.frame = "vicon_head"
    else
	frontend.pose_init_samples.frame = "vicon_body"
    end

    # Transformer configuration names
    frontend.imu_frame = "stim300"
    frontend.body_frame = "body"

    if framework[:proprio_vicon_head]
	frontend.vicon_frame = "vicon_head"
    else
	frontend.vicon_frame = "vicon_body"
    end

    # Finalize transformer configuration
    Orocos.transformer.setup(frontend)

    # Configure and Run the Front-End
    frontend.configure
    frontend.start

    # Configure and Run the Back-End
    backend.configure
    #backend.start

    #########################
    # VIZKIT VISUALIZATION ##
    #########################

    if framework[:viz]

        #Asguard visualization
        asguardViz = Vizkit.default_loader.AsguardVisualization
        asguardViz.setPluginName("AsguardBodyState")
        asguardViz.setXForward(true)

        #RigidBody of the BodyCenter from frontend
        frontendRBS = Vizkit.default_loader.RigidBodyStateVisualization
        frontendRBS.displayCovariance(true)
        frontendRBS.setPluginName("FrontEndPose")
        frontendRBS.setColor(Eigen::Vector3.new(255, 0, 0))#Red
        frontendRBS.resetModel(0.4)

        #RigidBody of the BodyCenter from backtend
        backendRBS = Vizkit.default_loader.RigidBodyStateVisualization
        backendRBS.displayCovariance(true)
        backendRBS.setPluginName("BackEndPose")
        backendRBS.setColor(Eigen::Vector3.new(0, 0, 0))#Black
        backendRBS.resetModel(0.4)

        #Contact points FL Wheel (RED)
        c0FL = Vizkit.default_loader.RigidBodyStateVisualization
        c0FL.displayCovariance(true)
        c0FL.setPluginName("FLFoot0")
        c0FL.setColor(Eigen::Vector3.new(0, 0, 0))
        c0FL.resetModel(0.1)

        c1FL = Vizkit.default_loader.RigidBodyStateVisualization
        c1FL.displayCovariance(true)
        c1FL.setPluginName("FLFoot1")
        c1FL.setColor(Eigen::Vector3.new(1, 0, 0.2))
        c1FL.resetModel(0.1)

        c2FL = Vizkit.default_loader.RigidBodyStateVisualization
        c2FL.displayCovariance(true)
        c2FL.setPluginName("FLFoot2")
        c2FL.setColor(Eigen::Vector3.new(1, 0, 0.4))
        c2FL.resetModel(0.1)

        c3FL = Vizkit.default_loader.RigidBodyStateVisualization
        c3FL.displayCovariance(true)
        c3FL.setPluginName("FLFoot3")
        c3FL.setColor(Eigen::Vector3.new(1, 0, 0.6))
        c3FL.resetModel(0.1)

        c4FL = Vizkit.default_loader.RigidBodyStateVisualization
        c4FL.displayCovariance(true)
        c4FL.setPluginName("FLFoot4")
        c4FL.setColor(Eigen::Vector3.new(1, 0, 0.8))
        c4FL.resetModel(0.1)

        #Contact points FR Wheel (GREEN)
        c0FR = Vizkit.default_loader.RigidBodyStateVisualization
        c0FR.setColor(Eigen::Vector3.new(0, 0, 0))
        c0FR.setPluginName("FRFoot0")
        c0FR.resetModel(0.1)
        c0FR.displayCovariance(true)

        c1FR = Vizkit.default_loader.RigidBodyStateVisualization
        c1FR.setColor(Eigen::Vector3.new(0, 1, 0.2))
        c1FR.setPluginName("FRFoot1")
        c1FR.resetModel(0.1)
        c1FR.displayCovariance(true)

        c2FR = Vizkit.default_loader.RigidBodyStateVisualization
        c2FR.setColor(Eigen::Vector3.new(0, 1, 0.4))
        c2FR.setPluginName("FRFoot2")
        c2FR.resetModel(0.1)
        c2FR.displayCovariance(true)

        c3FR = Vizkit.default_loader.RigidBodyStateVisualization
        c3FR.setColor(Eigen::Vector3.new(0, 1, 0.6))
        c3FR.setPluginName("FRFoot3")
        c3FR.resetModel(0.1)
        c3FR.displayCovariance(true)

        c4FR = Vizkit.default_loader.RigidBodyStateVisualization
        c4FR.setColor(Eigen::Vector3.new(0, 1, 0.8))
        c4FR.setPluginName("FRFoot4")
        c4FR.resetModel(0.1)
        c4FR.displayCovariance(true)

        #Contact points RL Wheel (BLUE)
        c0RL = Vizkit.default_loader.RigidBodyStateVisualization
        c0RL.setColor(Eigen::Vector3.new(0, 0, 0))
        c0RL.setPluginName("RLFoot0")
        c0RL.resetModel(0.1)
        c0RL.displayCovariance(true)

        c1RL = Vizkit.default_loader.RigidBodyStateVisualization
        c1RL.setColor(Eigen::Vector3.new(0, 0.2, 1))
        c1RL.setPluginName("RLFoot1")
        c1RL.resetModel(0.1)
        c1RL.displayCovariance(true)

        c2RL = Vizkit.default_loader.RigidBodyStateVisualization
        c2RL.setColor(Eigen::Vector3.new(0, 0.4, 1))
        c2RL.setPluginName("RLFoot2")
        c2RL.resetModel(0.1)
        c2RL.displayCovariance(true)

        c3RL = Vizkit.default_loader.RigidBodyStateVisualization
        c3RL.setColor(Eigen::Vector3.new(0, 0.6, 1))
        c3RL.setPluginName("RLFoot3")
        c3RL.resetModel(0.1)
        c3RL.displayCovariance(true)

        c4RL = Vizkit.default_loader.RigidBodyStateVisualization
        c4RL.setColor(Eigen::Vector3.new(0, 0.8, 1))
        c4RL.setPluginName("RLFoot4")
        c4RL.resetModel(0.1)
        c4RL.displayCovariance(true)

        #Contact points RR Wheel (WHITE)
        c0RR = Vizkit.default_loader.RigidBodyStateVisualization
        c0RR.setColor(Eigen::Vector3.new(0, 0, 0))
        c0RR.setPluginName("RRFoot0")
        c0RR.resetModel(0.1)
        c0RR.displayCovariance(true)

        c1RR = Vizkit.default_loader.RigidBodyStateVisualization
        c1RR.setColor(Eigen::Vector3.new(1, 0.2, 1))
        c1RR.setPluginName("RRFoot1")
        c1RR.resetModel(0.1)
        c1RR.displayCovariance(true)

        c2RR = Vizkit.default_loader.RigidBodyStateVisualization
        c2RR.setColor(Eigen::Vector3.new(1, 0.4, 1))
        c2RR.setPluginName("RRFoot2")
        c2RR.resetModel(0.1)
        c2RR.displayCovariance(true)

        c3RR = Vizkit.default_loader.RigidBodyStateVisualization
        c3RR.setColor(Eigen::Vector3.new(1, 0.6, 1))
        c3RR.setPluginName("RRFoot3")
        c3RR.resetModel(0.1)
        c3RR.displayCovariance(true)

        c4RR = Vizkit.default_loader.RigidBodyStateVisualization
        c4RR.setColor(Eigen::Vector3.new(1, 0.8, 1))
        c4RR.setPluginName("RRFoot4")
        c4RR.resetModel(0.1)
        c4RR.displayCovariance(true)

        #FrontEnd Asguard trajectory
        frontendAsguardTrajectory = Vizkit.default_loader.TrajectoryVisualization
        frontendAsguardTrajectory.setColor(Eigen::Vector3.new(255, 0, 0))#Red line
        frontendAsguardTrajectory.setPluginName("FrontEndTrajectory")


        visualization.on_reachable do

            #mapping the inports for the visualization task
            visualization.port('frontend_pose_samples').on_reachable do
                begin
                    t = TaskContext.get 'visualization'
                    frontend.pose_samples_out.connect_to(t.frontend_pose_samples, :type => :buffer, :size => 200)
                rescue Orocos::NotFound
                end
            end

            visualization.port('backend_pose_samples').on_reachable do
                backend.pose_samples_out.connect_to(visualization.port('backend_pose_samples'), :type => :buffer, :size => 200)
            end

            visualization.port('ground_truth_pose_samples').on_reachable do
                frontend.ground_truth_pose_samples_out.connect_to(visualization.port('ground_truth_pose_samples'), :type => :buffer, :size => 200)
            end

            visualization.port('fkchains_samples').on_reachable do
                frontend.fkchains_samples_out.connect_to(visualization.port('fkchains_samples'), :type => :buffer, :size => 500)
            end

            #Access to the chains sub_ports
            vector_rbs = visualization.port('fkchains_rbs_out')

            Vizkit.display vector_rbs.sub_port([:rbsChain, 0]), :widget => c0FL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 1]), :widget => c1FL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 2]), :widget => c2FL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 3]), :widget => c3FL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 4]), :widget => c4FL

            Vizkit.display vector_rbs.sub_port([:rbsChain, 5]), :widget => c0FR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 6]), :widget => c1FR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 7]), :widget => c2FR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 8]), :widget => c3FR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 9]), :widget => c4FR

            Vizkit.display vector_rbs.sub_port([:rbsChain, 10]), :widget => c0RL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 11]), :widget => c1RL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 12]), :widget => c2RL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 13]), :widget => c3RL
            Vizkit.display vector_rbs.sub_port([:rbsChain, 14]), :widget => c4RL

            Vizkit.display vector_rbs.sub_port([:rbsChain, 15]), :widget => c0RR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 16]), :widget => c1RR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 17]), :widget => c2RR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 18]), :widget => c3RR
            Vizkit.display vector_rbs.sub_port([:rbsChain, 19]), :widget => c4RR

           #Vizkit.display vector_rbs.sub_port([:rbsChain, 20]), :widget => Vizkit.default_loader.StructViewer


            if (framework[:vicon] || framework[:proprio_vicon] || framework[:proprio_vicon_head])
                # Trajectory of the ground truth
                truthTrajectory = Vizkit.default_loader.TrajectoryVisualization
                truthTrajectory.setColor(Eigen::Vector3.new(0, 255, 0)) #Green line
                truthTrajectory.setPluginName("GroundTruthTrajectory")

                #RigidBodyState of the ground truth
                rbsTruth = Vizkit.default_loader.RigidBodyStateVisualization
                rbsTruth.setColor(Eigen::Vector3.new(0, 255, 0))#Green rbs
                rbsTruth.setPluginName("GroundTruthPose")
                rbsTruth.resetModel(0.4)

                #Connect to the ground truth output port of the visualization task
                Vizkit.display visualization.port('ground_truth_pose_samples_out'), :widget => rbsTruth

                visualization.port('ground_truth_pose_samples_out').on_data do|ground_truth,_|
                    truthTrajectory.updateTrajectory(ground_truth.position)
                end
            end

            Vizkit.display visualization.port('backend_pose_samples_out'), :widget =>backendRBS
            Vizkit.display visualization.port('frontend_pose_samples_out'), :widget =>frontendRBS


            visualization.port('frontend_pose_samples_out').on_data do |asguard_rbs,_|
                asguardViz.updateRigidBodyState(asguard_rbs)
                frontendAsguardTrajectory.updateTrajectory(asguard_rbs.position)
            end

            visualization.port('bodystate_samples_out').on_data do |asguard_state,_|
                asguardViz.updateBodyState(asguard_state)
            end

            if framework[:envire]
                envire_viz = Vizkit.default_loader.EnvireVisualization
            end

            #Configure and Run the visualization
            visualization.configure
            visualization.start

            #2D plot the instantaneous velocity information
            #     Vizkit.display visualization.body2world_out,:subfield => ["velocity"],:widget=>"Plot2d"
            #     Vizkit.display visualization.imu2world_out,:subfield => ["velocity"],:widget=>"Plot2d"
        end

    end ##END framework[:viz]

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

#     control.auto_replay
    Vizkit.exec

end


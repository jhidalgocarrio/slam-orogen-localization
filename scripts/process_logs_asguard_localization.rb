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
framework = {:proprio => false, :raw_vicon => false, :proprio_vicon => false, :proprio_vicon_head => true, :viz => true, :stim300 => true, :xsens => false, :simu => false}

Orocos.run 'rover_localization::FrontEnd' => 'frontend',  'rover_localization::BackEnd' => 'backend', 'rover_localization::Visualization' => 'visualization' do

    # log all the output ports
    Orocos.log_all_ports
    Orocos.conf.load_dir('../config/')

    # get the FrontEnd
    frontend = Orocos.name_service.get 'frontend'
    Orocos.conf.apply(frontend, ['default', 'asguard', 'bremen', 'stim300', 'bessel50'], :override => true )

    # get the BackEnd
    backend = Orocos.name_service.get 'backend'
    Orocos.conf.apply(backend, ['default', 'stim300'], :override => true )

    if framework[:viz]
        # get the Visualization in Asynchronous mode
        visualization = Orocos.name_service.get 'visualization'
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
        if framework[:simu]
            log_replay.imumodel.imuout.connect_to(frontend.inertial_samples, :type => :buffer, :size => 200 )
        else
            if framework[:stim300]
	        log_replay.stim300.calibrated_sensors.connect_to(frontend.inertial_samples, :type => :buffer, :size => 200 )
            elsif framework[:xsens]
    	        log_replay.xsens_imu.calibrated_sensors.connect_to(frontend.inertial_samples, :type => :buffer, :size => 200 )
            end
        end
    end

    if framework[:raw_vicon]
	log_replay.vicon.pose_samples.connect_to(frontend.reference_pose_samples, :type => :buffer, :size => 200 )
    elsif (framework[:proprio_vicon] || framework[:proprio_vicon_head])
	log_replay.propriocessing.pose_samples_out.connect_to(frontend.reference_pose_samples, :type => :buffer, :size => 200 )
    end

    #mapping the frontend outports into the backend inports
    frontend.pose_samples_out.connect_to(backend.pose_samples, :type => :buffer, :size => 200)
    frontend.inertial_samples_out.connect_to(backend.inertial_samples, :type => :buffer, :size => 200)

    #mapping the backend outports into the frontend inports
    backend.backend_estimation_samples_out.connect_to(frontend.backend_estimation_samples, :type => :buffer, :size => 200)

    #Configure the frames names
    frontend.inertial_samples.frame = "stim300"

    if framework[:proprio_vicon_head]
	frontend.reference_pose_samples.frame = "vicon_head"
    else
	frontend.reference_pose_samples.frame = "vicon_body"
    end

    # Transformer configuration names
    if framework[:stim300]
        frontend.imu_frame = "stim300"
    elsif framework[:xsens]
        frontend.imu_frame = "xsens"
    end

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
    backend.start

    ############################
    # VISUALIZATION COMPONENT ##
    ############################

    if framework[:viz]

        #mapping the inports for the visualization task
        frontend.pose_samples_out.connect_to(visualization.frontend_pose_samples, :type => :buffer, :size => 200)
        backend.pose_samples_out.connect_to(visualization.backend_pose_samples, :type => :buffer, :size => 200)
        frontend.reference_pose_samples_out.connect_to(visualization.reference_pose_samples, :type => :buffer, :size => 200)
        frontend.fkchains_samples_out.connect_to(visualization.fkchains_samples, :type => :buffer, :size => 500)

        #Configure and Run the visualization
        visualization.configure
        visualization.start

    end ##END framework[:viz]

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

    Vizkit.exec

end


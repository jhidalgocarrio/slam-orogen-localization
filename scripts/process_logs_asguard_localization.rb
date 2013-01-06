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

viz = {:proprio => false, :vicon => false, :pvicon => false, :ivicon => true}

Orocos.run('rover_localization_test') do 
  
    # log all the output ports
    Orocos.log_all_ports
    Orocos.conf.load_dir('../config/')
         
    # get the invidual tasks
    asguard_localization_task = TaskContext.get 'rover_localization'
    Orocos.conf.apply(asguard_localization_task, ['default'], :override => true )
    
     # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] ) 
    
    #Mapping the inputs ports in the navigation tasks
    if viz[:proprio]
	log_replay.propriocessing.hbridge_samples_out.connect_to(asguard_localization_task.hbridge_samples, :type => :buffer, :size => 100 )
	log_replay.propriocessing.systemstate_samples_out.connect_to(asguard_localization_task.systemstate_samples, :type => :buffer, :size => 100 )
	log_replay.propriocessing.calibrated_sensors_out.connect_to(asguard_localization_task.calibrated_sensors, :type => :buffer, :size => 100 )
    else
	log_replay.hbridge.status_motors.connect_to(asguard_localization_task.hbridge_samples, :type => :buffer, :size => 100 )
	log_replay.sysmon.system_status.connect_to(asguard_localization_task.systemstate_samples, :type => :buffer, :size => 100 )
	log_replay.stim300.calibrated_sensors.connect_to(asguard_localization_task.calibrated_sensors, :type => :buffer, :size => 100 )
    end
    
    if viz[:vicon]
	log_replay.vicon.pose_samples.connect_to(asguard_localization_task.pose_init, :type => :buffer, :size => 100 )
    elsif (viz[:pvicon] || viz[:ivicon])
	log_replay.propriocessing.pose_samples_out.connect_to(asguard_localization_task.pose_init, :type => :buffer, :size => 100 )
    end

    asguard_localization_task.calibrated_sensors.frame = "stim300"
    
    if viz[:ivicon]
	asguard_localization_task.pose_init.frame = "vicon_head"
    else
	asguard_localization_task.pose_init.frame = "vicon_body"
    end

    # Transformer configuration names
    asguard_localization_task.imu_frame = "stim300"
    asguard_localization_task.body_frame = "body"
    
    if viz[:ivicon]
	asguard_localization_task.vicon_frame = "vicon_head"
    else
	asguard_localization_task.vicon_frame = "vicon_body"
    end
    
    # Finalize transformer configuration (see below for explanations)
    Orocos.transformer.setup(asguard_localization_task)

    asguard_localization_task.configure
    asguard_localization_task.start
    
    #Asguard visualization
    asguardViz = Vizkit.default_loader.AsguardVisualization
    asguardViz.setXForward(true)
    
    #RigidBody of the BodyCenter
    BC = Vizkit.default_loader.RigidBodyStateVisualization
    BC.displayCovariance(true)
    BC.setColor(Eigen::Vector3.new(0, 0, 0))
    BC.resetModel(0.4)
    
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
    
    #Asguard trajectory
    asguardTrajectory = Vizkit.default_loader.TrajectoryVisualization
    asguardTrajectory.setColor(Eigen::Vector3.new(255, 0, 0))
    
    asguard_localization_task.C0FL2body_out.connect_to c0FL
    asguard_localization_task.C1FL2body_out.connect_to c1FL
    asguard_localization_task.C2FL2body_out.connect_to c2FL
    asguard_localization_task.C3FL2body_out.connect_to c3FL
    asguard_localization_task.C4FL2body_out.connect_to c4FL
    
    asguard_localization_task.C0FR2body_out.connect_to c0FR
    asguard_localization_task.C1FR2body_out.connect_to c1FR
    asguard_localization_task.C2FR2body_out.connect_to c2FR
    asguard_localization_task.C3FR2body_out.connect_to c3FR
    asguard_localization_task.C4FR2body_out.connect_to c4FR
    
    asguard_localization_task.C0RL2body_out.connect_to c0RL
    asguard_localization_task.C1RL2body_out.connect_to c1RL
    asguard_localization_task.C2RL2body_out.connect_to c2RL
    asguard_localization_task.C3RL2body_out.connect_to c3RL
    asguard_localization_task.C4RL2body_out.connect_to c4RL
    
    asguard_localization_task.C0RR2body_out.connect_to c0RR
    asguard_localization_task.C1RR2body_out.connect_to c1RR
    asguard_localization_task.C2RR2body_out.connect_to c2RR
    asguard_localization_task.C3RR2body_out.connect_to c3RR
    asguard_localization_task.C4RR2body_out.connect_to c4RR
    
    if (viz[:vicon] || viz[:pvicon] || viz[:ivicon])
	# True Body and true trajectory
	truthTrajectory = Vizkit.default_loader.TrajectoryVisualization
	truthTrajectory.setColor(Eigen::Vector3.new(0, 255, 0)) #Green
	rbsTruth = Vizkit.default_loader.RigidBodyStateVisualization
	rbsTruth.setColor(Eigen::Vector3.new(0, 255, 0))
	rbsTruth.resetModel(0.4)
	
	#Connect to the ground truth output port of the navigation kinematics task
	asguard_localization_task.rbsVicon.connect_to rbsTruth

	asguard_localization_task.rbsVicon.connect_to do|vicon,_|    
	    truthTrajectory.updateTrajectory(vicon.position)
	end
    end
    
    asguard_localization_task.pose_samples_out.connect_to BC
    asguard_localization_task.pose_samples_out.connect_to do |asguard_rbs,_|
	asguardViz.updateRigidBodyState(asguard_rbs)
    end
    
    
    asguard_localization_task.pose_samples_out.connect_to do |data,_|
	asguardTrajectory.updateTrajectory(data.position)
    end
    
    asguard_localization_task.bodystate_samples.connect_to do |asguard_state,_|
	asguardViz.updateBodyState(asguard_state)
    end
    
    #2D plor the instantaneous velocity information
#     Vizkit.display asguard_localization_task.body2world_out,:subfield => ["velocity"],:widget=>"Plot2d"
#     Vizkit.display asguard_localization_task.imu2world_out,:subfield => ["velocity"],:widget=>"Plot2d"

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4
#     control.auto_replay
    Vizkit.exec
        
end


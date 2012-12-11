#! /usr/bin/env ruby
require 'orocos'
require 'orocos/log'
require 'transformer/runtime'
require 'vizkit'
require 'utilrb'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs_ikf_navigation_odometry.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

Orocos.transformer.load_conf('../config/transforms.rb')

Orocos.run('rover_localization_test', 'ikf_orientation_estimator', 'lowlevel') do 
  
    # log all the output ports
    Orocos.log_all_ports 
    
    Orocos.conf.load_dir('../../../../slam/orogen/orientation_estimator/config/')
	
    # get the invidual tasks
    localization_task = TaskContext.get 'rover_localization'
    ikf_attitude_task = TaskContext.get 'ikf_orientation_estimator'
    Orocos.conf.apply(ikf_attitude_task, ['stim300'], :override => true )
    ikf_attitude_task.delta_time = (1.0/16.0)
    nav_odometry_task = TaskContext.get 'nav_odometry'

     # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )
    
    localization_task.calibrated_sensors.frame = "stim300"
    localization_task.orientation_init.frame = "stim300"
    
    # Transformer configuration names
    localization_task.imu_frame = "stim300"
    localization_task.body_frame = "body"
    
    #Mapping the inputs ports in the localization tasks
    log_replay.hbridge.status_motors.connect_to(localization_task.hbridge_samples, :type => :buffer, :size => 10 )
    log_replay.sysmon.system_status.connect_to(localization_task.systemstate_samples, :type => :buffer, :size => 10 )
    log_replay.stim300.calibrated_sensors.connect_to(localization_task.calibrated_sensors, :type => :buffer, :size => 10 )
#     log_replay.torque_estimator.torque_estimated.connect_to(localization_task.torque_estimated, :type => :buffer, :size => 10 )
#     log_replay.torque_estimator.ground_forces_estimated.connect_to(localization_task.ground_forces_estimated, :type => :buffer, :size => 10 )
    
    
    #Mapping the inputs ports in the ikf orientation task
    log_replay.stim300.calibrated_sensors.connect_to( ikf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    
    # Connect the output of the ikf to the input port of the localization_task
    ikf_attitude_task.attitude_b_g.connect_to (localization_task.orientation_init, :type => :buffer, :size => 10 )
    
    #Mapping the inputs ports in the navigation tasks
    log_replay.hbridge.status_motors.connect_to(nav_odometry_task.hbridge_samples, :type => :buffer, :size => 10 )
    ikf_attitude_task.attitude_b_g.connect_to(nav_odometry_task.orientation_samples, :type => :buffer, :size => 10 )
    log_replay.sysmon.system_status.connect_to(nav_odometry_task.systemstate_samples, :type => :buffer, :size => 10 )

     # Finalize transformer configuration (see below for explanations)
    Orocos.transformer.setup(localization_task)
    
    ikf_attitude_task.configure
    ikf_attitude_task.start

    localization_task.configure
    localization_task.start
    
    nav_odometry_task.configure
    nav_odometry_task.start
        
    Vizkit.display nav_odometry_task.body2world_out,:subfield => ["velocity"],:widget=>"Plot2d"
    Vizkit.display localization_task.pose_samples_out,:subfield => ["velocity"],:widget=>"Plot2d"
    Vizkit.display log_replay.Vicon.pose_samples, :subfield => ["velocity"],:widget=>"Plot2d"
        
    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4
#     control.auto_replay
    Vizkit.exec
        
end

#! /usr/bin/env ruby
require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb <data_log_directory>"
    exit
end

Rock::Bundles.transformer_config = 'sim_vicon_transforms.rb'

Rock::Bundles.initialize

Rock::Bundles.run('rover_localization_test') do 
  
    # get the invidual tasks
    localization_task = Rock::Bundles.get 'rover_localization'
    
     # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )
    
    localization_task.calibrated_sensors.frame = "stim300"
    
    # Transformer configuration names
    localization_task.imu_frame = "stim300"
    localization_task.body_frame = "body"
    
    #Mapping the inputs ports in the navigation tasks
    log_replay.hbridge.status_motors.connect_to(localization_task.hbridge_samples, :type => :buffer, :size => 10 )
    log_replay.stim300.calibrated_sensors.connect_to(localization_task.calibrated_sensors, :type => :buffer, :size => 10 )
    log_replay.sysmon.system_status.connect_to(localization_task.systemstate_samples, :type => :buffer, :size => 10 )
#     log_replay.torque_estimator.torque_estimated.connect_to(localization_task.torque_estimated, :type => :buffer, :size => 10 )
#     log_replay.torque_estimator.ground_forces_estimated.connect_to(localization_task.ground_forces_estimated, :type => :buffer, :size => 10 )
    
    # Finalize transformer configuration (see below for explanations)
    Orocos.transformer.setup(localization_task)
    
    localization_task.configure
    localization_task.start
        
#     Vizkit.display log_replay.hbridge.status_motors,:subfield => ["states.[0]"],:widget=>"Plot2d"
#     Vizkit.display localization_task.hbridge_samples_out,:subfield => ["states.[0]"],:widget=>"Plot2d"
    
    
    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4
#     control.auto_replay
    Vizkit.exec
        
end

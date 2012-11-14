#! /usr/bin/env ruby
require 'orocos'
require 'orocos/log'
require 'transformer/runtime'
require 'vizkit'
require 'utilrb'

include Orocos


#Initializes the CORBA communication layer
Orocos.initialize

Orocos.transformer.load_conf('../config/transforms.rb')

Orocos.run('asguard_localization_test', 'ikf_orientation_estimator') do 
  
    # log all the output ports
    Orocos.log_all_ports 
    
    Orocos.conf.load_dir('../../../../slam/orogen/orientation_estimator/config/')
	
    # get the invidual tasks
    localization_task = TaskContext.get 'asguard_localization'
    ikf_attitude_task = TaskContext.get 'ikf_orientation_estimator'
    nav_odometry_task = TaskContext.get 'nav_odometry'
    
    #get the lowlevel tasks (already running)
    hbridge_task = TaskContext.get 'hbridge'
    sysmon_task = TaskContext.get 'sysmon'
    stim300_task = TaskContext.get 'stim300'
    
    
    Orocos.conf.apply(ikf_attitude_task, ['stim300'], :override => true )
    ikf_attitude_task.delta_time = (1.0/16.0)
    
    localization_task.calibrated_sensors.frame = "stim300"
    localization_task.orientation_debug.frame = "stim300"
    
    # Transformer configuration names
    localization_task.imu_frame = "stim300"
    localization_task.body_frame = "body"
    
    #Mapping the inputs ports in the localization tasks
    hbridge_task.status_motors.connect_to (localization_task.hbridge_samples, :type => :buffer, :size => 10 )
    sysmon_task.system_status.connect_to(localization_task.systemstate_samples, :type => :buffer, :size => 10 )
    stim300_task.calibrated_sensors.connect_to(localization_task.calibrated_sensors, :type => :buffer, :size => 10 )
    
    #Mapping the inputs ports in the ikf orientation task
    stim300_task.calibrated_sensors.connect_to( ikf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    
    # Connect the output of the ikf to the input port of the localization_task
    ikf_attitude_task.attitude_b_g.connect_to (localization_task.orientation_debug, :type => :buffer, :size => 10 )
    
    #Mapping the inputs ports in the navigation tasks
    hbridge_task.status_motors.connect_to(nav_odometry_task.hbridge_samples, :type => :buffer, :size => 10 )
    sysmon_task.system_status.connect_to(nav_odometry_task.orientation_samples, :type => :buffer, :size => 10 )
    stim300_task.calibrated_sensors.connect_to(nav_odometry_task.systemstate_samples, :type => :buffer, :size => 10 )

     # Finalize transformer configuration
    Orocos.transformer.setup(localization_task)
    
    ikf_attitude_task.configure
    ikf_attitude_task.start

    localization_task.configure
    localization_task.start
    
    nav_odometry_task.configure
    nav_odometry_task.start
        
    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4
#     control.auto_replay
    Vizkit.exec
        
end
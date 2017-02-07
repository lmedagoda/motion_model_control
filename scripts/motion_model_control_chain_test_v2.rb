#***********************************************************************/
#                                                                      */
#                                                                      */
# FILE --- motion_model_control_chain.rb                               */
#                                                                      */
# PURPOSE --- Tests for the motion model component using Rock          */
# control chain                                                        */
#                                                                      */
#  Bilal Wehbe                                                         */
#  bilal.wehbe@dfki.de                                                 */
#  DFKI - BREMEN 2015                                                  */
#***********************************************************************/


require 'orocos'
require 'Qt'
require 'vizkit'
include Orocos

Orocos.initialize

Orocos.run 'mission_base','cmd_producer','sensors_leng', :valgrind => false do |p|
    Orocos.conf.load_dir('../config')
    #Bundles.transformer.load_conf("#{ENV['AUTOPROJ_CURRENT_ROOT']}/leng_scripts/config/transformer.rb")

    #Bundles.log_all
	motionModel            = TaskContext.get 'perception_model_only'
    cmd_producer_speed = TaskContext.get 'control_cmd_producer'
    cmd_producer = TaskContext.get 'control_cmd_producer2'
    constantCommand        = TaskContext.get 'control_cmd_producer3'

    world_to_aligned =        TaskContext.get 'control_chain0_world_to_aligned'
    aligned_position_controller = TaskContext.get 'control_chain1_aligned_position_controller'
    aligned_velocity_controller = TaskContext.get 'control_chain2_aligned_velocity_controller'
    velocity_limiter = TaskContext.get 'control_chain31_velocity_limiter'
    acceleration_controller = TaskContext.get 'control_chain4_acceleration_controller'
#    joystick =                TaskContext.get 'mc20'

    acceleration_injection =  TaskContext.get 'control_chain3_acceleration_injection'

    #velocity_injection =      TaskContext.get 'velocity_injection'
    #world_injection =         TaskContext.get 'world_injection'

    Orocos.conf.apply(acceleration_injection, ['default'])

##########################WORLD_TO_ALIGNED
    STDERR.print "setting up world_to_aligned ..."
    Orocos.conf.apply(world_to_aligned,['default'], true) #the config "only_z" disables all axes but z axis linear.
    world_to_aligned.configure
    STDERR.puts "done"

##########################ALIGNED_POSITION_CONTROLLER
    STDERR.print "setting up aligned_position_controller ..."
    Orocos.conf.apply(aligned_position_controller,['default', 'position_parallel'], true)
    aligned_position_controller.configure
    STDERR.puts "done"

##########################ALIGNED_VELOCITY_CONTROLLER
    STDERR.print "setting up aligned_velocity_controller ..."
    Orocos.conf.apply(aligned_velocity_controller,['default', 'velocity_parallel', 'debug'], true)
    aligned_velocity_controller.configure
    STDERR.puts "done"
    
    ##########################ALIGNED_VELOCITY_CONTROLLER
    STDERR.print "setting up velocity_limiter ..."
    Orocos.conf.apply(velocity_limiter,['default'], true)
    velocity_limiter.configure
    STDERR.puts "done"

#########################CONSTANT_COMMAND for depth
    STDERR.print "setting up cmd_producer for depth..."
    cmd = cmd_producer.cmd
    cmd.linear[0] = 0
    cmd.linear[1] = 0
    cmd.linear[2] = 0
    cmd.angular[0] = 0
    cmd.angular[1] = 0
    cmd.angular[2] = 0
    cmd_producer.cmd = cmd
    STDERR.puts "done"

#########################CONSTANT_COMMAND for speed
    STDERR.print "setting up cmd_producer for speed..."
    cmd = cmd_producer_speed.cmd
    cmd.linear[0] = 0
    cmd.linear[1] = 0
    cmd.linear[2] = 0
    cmd.angular[0] = 0
    cmd.angular[1] = 0
    cmd.angular[2] = NaN
    cmd_producer_speed.cmd = cmd
    STDERR.puts "done"

  init = constantCommand.cmd
  init.linear[0] = 0
  init.linear[1] = 0
  init.linear[2] = 0
  init.angular[0] = 0
  init.angular[1] = 0
  init.angular[2] = 0
  constantCommand.cmd = init
 
##########################ACCELERATION_CONTROLLER
    STDERR.print "setting up acceleration_controller ..."
    Orocos.conf.apply(acceleration_controller,["default","no_vector"], true)
    #acceleration_controller.apply_conf_file("auv_control::AccelerationController.yml",["default","no_vector"], true)
    acceleration_controller.configure
    STDERR.puts "done"

##########################MotionModel
    STDERR.print "setting up MotionModel ..."
    Orocos.conf.apply(motionModel,['default'])
    #motionModel.apply_conf_file("uwv_motion_model::Task.yml",["default"])
    motionModel.configure
    STDERR.puts "done" 
  
  #########################Command Injection 
    STDERR.print "setting up command injection ..."
    acceleration_injection.safe_mode = false
    acceleration_injection.timeout_cascade = 0
    acceleration_injection.timeout_in = 0
    acceleration_injection.configure
    STDERR.puts "done"

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################


	#constantCommand.cmd_out.connect_to aligned_velocity_controller.cmd_cascade
    motionModel.cmd_out.connect_to world_to_aligned.pose_samples    
    motionModel.cmd_out.connect_to aligned_position_controller.pose_samples
    motionModel.cmd_out.connect_to aligned_velocity_controller.pose_samples


    cmd_producer.cmd_out.connect_to(world_to_aligned.cmd_in)
    world_to_aligned.cmd_out.connect_to aligned_position_controller.cmd_cascade
    aligned_position_controller.cmd_out.connect_to aligned_velocity_controller.cmd_cascade
  #  aligned_velocity_controller.cmd_out.connect_to acceleration_controller.cmd_cascade
    aligned_velocity_controller.cmd_out.connect_to acceleration_injection.cmd_cascade #velocity_limiter.cmd_cascade
#    velocity_limiter.cmd_out.connect_to acceleration_injection.cmd_cascade
    acceleration_injection.cmd_out.connect_to acceleration_controller.cmd_cascade
    

#    aligned_velocity_controller.cmd_out.connect_to acceleration_controller.cmd_cascade
#    cmd_producer_speed.cmd_out.connect_to aligned_velocity_controller.cmd_in
    
#	constantCommand.cmd_out.connect_to acceleration_controller.cmd_in
	#aligned_velocity_controller.cmd_out.connect_to accelerationController.cmd_cascade
	acceleration_controller.cmd_out.connect_to motionModel.cmd_in
	
	
# Configuring and starting the component
  constantCommand.configure
  constantCommand.start

cmd_producer.configure
cmd_producer.start
cmd_producer_speed.configure
cmd_producer_speed.start
  #aligned_velocity_controller.configure
 # aligned_velocity_controller.start
#  accelerationController.configure
  acceleration_controller.start
#  motionModel.configure
  motionModel.start
  world_to_aligned.start
  aligned_velocity_controller.start
  aligned_position_controller.start
  acceleration_injection.start
  velocity_limiter.start

  
  motionmodelproxy = Orocos::Async.proxy("perception_model_only")

  statesPort = motionmodelproxy.port("cmd_out")    

  rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization

  # Connecting the vehicle model output to the 3D Visualization plugin  
  statesPort.connect_to do |sample, |
    rbs_3DVisualization.updateRigidBodyState(sample)
  end
  

  Vizkit.exec

end

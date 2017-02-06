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

Orocos.run 'uwv_control','uwv_remote','leng_gliding::Task'=>'leng_gliding' do


Orocos.conf.load_dir('../config')     

	## Get the specific task context ##
	acceleration_controller = TaskContext.get 'acceleration_controller'
	motionModel            = TaskContext.get 'motion_model'
	cmd_producer_speed = TaskContext.get 'cmd_producer_speed'
        cmd_producer = TaskContext.get 'cmd_producer'
        constantCommand        = TaskContext.get 'ConstantCommand'
	world_to_aligned = TaskContext.get 'world_to_aligned'
	aligned_position_controller = TaskContext.get 'aligned_position_controller'
	aligned_velocity_controller = TaskContext.get 'aligned_velocity_controller'
	leng_gliding = TaskContext.get 'leng_gliding'


##########################LENG_GLIDING
    STDERR.print "setting up leng_gliding ..."
    Orocos.conf.apply(leng_gliding,['default'], true) 
    leng_gliding.configure
    STDERR.puts "done"

##########################WORLD_TO_ALIGNED
    STDERR.print "setting up world_to_aligned ..."
    Orocos.conf.apply(world_to_aligned,['only_z_pitch'], true) #the config "only_z" disables all axes but z axis linear.
    world_to_aligned.configure
    STDERR.puts "done"

##########################ALIGNED_POSITION_CONTROLLER
    STDERR.print "setting up aligned_position_controller ..."
    Orocos.conf.apply(aligned_position_controller,['only_z_pitch', 'position_parallel'], true)
    aligned_position_controller.configure
    STDERR.puts "done"

##########################ALIGNED_VELOCITY_CONTROLLER
    STDERR.print "setting up aligned_velocity_controller ..."
    Orocos.conf.apply(aligned_velocity_controller,['only_z_pitch', 'velocity_parallel', 'debug'], true)
    aligned_velocity_controller.configure
    STDERR.puts "done"

#########################CONSTANT_COMMAND for depth
    STDERR.print "setting up cmd_producer for depth..."
    cmd = cmd_producer.cmd
    cmd.linear[0] = NaN
    cmd.linear[1] = NaN
    cmd.linear[2] = 0
    cmd.angular[0] = NaN
    cmd.angular[1] = 0
    cmd.angular[2] = NaN
    cmd_producer.cmd = cmd
    STDERR.puts "done"

#########################CONSTANT_COMMAND for speed
    STDERR.print "setting up cmd_producer for speed..."
    cmd = cmd_producer_speed.cmd
    cmd.linear[0] = 0
    cmd.linear[1] = 0
    cmd.linear[2] = NaN
    cmd.angular[0] = 0
    cmd.angular[1] = NaN
    cmd.angular[2] = 0
    cmd_producer_speed.cmd = cmd
    STDERR.puts "done"

  init = constantCommand.cmd
  init.linear[0] = 0
  init.linear[1] = 0
  init.linear[2] = NaN
  init.angular[0] = 0
  init.angular[1] = NaN
  init.angular[2] = 0
  constantCommand.cmd = init
 
##########################ACCELERATION_CONTROLLER
    STDERR.print "setting up acceleration_controller ..."
    Orocos.conf.apply(acceleration_controller,['leng_simulator'])
    acceleration_controller.configure
    STDERR.puts "done"

##########################MotionModel
    STDERR.print "setting up MotionModel ..."
    Orocos.conf.apply(motionModel,['leng_simulator'])
    motionModel.configure
    STDERR.puts "done" 
  

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################


	#constantCommand.cmd_out.connect_to aligned_velocity_controller.cmd_cascade
    motionModel.cmd_out.connect_to world_to_aligned.pose_samples    
    motionModel.cmd_out.connect_to aligned_position_controller.pose_samples
    motionModel.cmd_out.connect_to aligned_velocity_controller.pose_samples
    motionModel.cmd_out.connect_to leng_gliding.pose_samples

#    leng_gliding.world_command.connect_to(world_to_aligned.cmd_in)
    cmd_producer.cmd_out.connect_to(world_to_aligned.cmd_in)
    world_to_aligned.cmd_out.connect_to aligned_position_controller.cmd_cascade
    aligned_position_controller.cmd_out.connect_to aligned_velocity_controller.cmd_cascade
    aligned_velocity_controller.cmd_out.connect_to acceleration_controller.cmd_cascade

#    aligned_velocity_controller.cmd_out.connect_to acceleration_controller.cmd_cascade
#    cmd_producer_speed.cmd_out.connect_to aligned_velocity_controller.cmd_in
    
	constantCommand.cmd_out.connect_to acceleration_controller.cmd_in
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
  leng_gliding.start

  
  motionmodelproxy = Orocos::Async.proxy("motion_model")

  statesPort = motionmodelproxy.port("cmd_out")    

  rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization

  # Connecting the vehicle model output to the 3D Visualization plugin  
  statesPort.connect_to do |sample, |
    rbs_3DVisualization.updateRigidBodyState(sample)
  end
  

  Vizkit.exec

end

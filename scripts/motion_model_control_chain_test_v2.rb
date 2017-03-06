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
require 'vizkit'
include Orocos

Orocos.initialize

Orocos.run 'control_chain', 'uwv_motion_model::Task' => 'motion_model' do
    Orocos.conf.load_dir('../config')
    
    tasks = Array.new

    #Orocos.log_all
    
	motion_model            = TaskContext.get 'motion_model'
    cmd_producer_pose = TaskContext.get 'pose_cmd'
    cmd_producer_velocity = TaskContext.get 'velocity_cmd'
    world_to_aligned =        TaskContext.get 'control_chain_world_to_aligned'
    aligned_position_controller = TaskContext.get 'control_chain_aligned_position_controller'
    aligned_velocity_controller = TaskContext.get 'control_chain_aligned_velocity_controller'
    acceleration_controller = TaskContext.get 'control_chain_acceleration_controller'


##########################WORLD_TO_ALIGNED
    STDERR.print "setting up world_to_aligned ..."
    Orocos.conf.apply(world_to_aligned,['default'], true)
    #Orocos.conf.apply(world_to_aligned,['default', 'no_xy'], true)
    tasks.push world_to_aligned
    STDERR.puts "done"

##########################ALIGNED_POSITION_CONTROLLER
    STDERR.print "setting up aligned_position_controller ..."
    Orocos.conf.apply(aligned_position_controller,['default', 'position_parallel'], true)
    #Orocos.conf.apply(aligned_position_controller,['default', 'position_parallel', 'no_xy'], true)
    tasks.push aligned_position_controller
    STDERR.puts "done"

##########################ALIGNED_VELOCITY_CONTROLLER
    STDERR.print "setting up aligned_velocity_controller ..."
    Orocos.conf.apply(aligned_velocity_controller,['default', 'velocity_parallel'], true)
    tasks.push aligned_velocity_controller
    STDERR.puts "done"

#########################CONSTANT_COMMAND for depth
    STDERR.print "setting up cmd_producer for depth..."
    cmd = cmd_producer_pose.cmd
    cmd.linear[0] = 0
    cmd.linear[1] = 0
    cmd.linear[2] = 0
    cmd.angular[0] = 0
    cmd.angular[1] = 0
    cmd.angular[2] = 0
    cmd_producer_pose.cmd = cmd
    tasks.push cmd_producer_pose
    STDERR.puts "done"

#########################CONSTANT_COMMAND for speed
    STDERR.print "setting up cmd_producer for speed..."
    cmd = cmd_producer_velocity.cmd
    cmd.linear[0] = 0
    cmd.linear[1] = 0
    cmd.linear[2] = NaN
    cmd.angular[0] = NaN
    cmd.angular[1] = NaN
    cmd.angular[2] = NaN
    cmd_producer_velocity.cmd = cmd
    tasks.push cmd_producer_velocity
    STDERR.puts "done"
 
##########################ACCELERATION_CONTROLLER
    STDERR.print "setting up acceleration_controller ..."
    Orocos.conf.apply(acceleration_controller,["default","no_vector"], true)
    tasks.push acceleration_controller
    STDERR.puts "done"

##########################MotionModel
    STDERR.print "setting up MotionModel ..."
    Orocos.conf.apply(motion_model,['default'])
    tasks.push motion_model
    STDERR.puts "done" 

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################



    motion_model.cmd_out.connect_to world_to_aligned.pose_samples    
    motion_model.cmd_out.connect_to aligned_position_controller.pose_samples
    motion_model.cmd_out.connect_to aligned_velocity_controller.pose_samples


    cmd_producer_pose.cmd_out.connect_to world_to_aligned.cmd_in
    #cmd_producer_velocity.cmd_out.connect_to aligned_velocity_controller.cmd_in
        
    world_to_aligned.cmd_out.connect_to aligned_position_controller.cmd_cascade
    aligned_position_controller.cmd_out.connect_to aligned_velocity_controller.cmd_cascade
    aligned_velocity_controller.cmd_out.connect_to acceleration_controller.cmd_cascade
	acceleration_controller.cmd_out.connect_to motion_model.cmd_in

	
	
	tasks.each do |task|
	    task.configure
	    task.start
	end

  
    Vizkit.display motion_model.cmd_out, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    
    task_inspector = Vizkit.default_loader.TaskInspector
    tasks.each do |task|
	    Vizkit.display task, :widget => task_inspector
	end
    
    Vizkit.exec
end

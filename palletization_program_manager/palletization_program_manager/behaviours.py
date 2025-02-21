import py_trees
import rcl_interfaces.srv as rcl_srvs
import rcl_interfaces.msg as rcl_msgs
import time
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import *
from  palletization_vision_interface.srv import GetPoses

class URPower(py_trees.behaviour.Behaviour):
    """
    Power ON the UR Robot
    Input name: Node name
    """
    def __init__(self, name):
        super(URPower, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name="URPower")
        self.blackboard.register_key(key="power", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_power", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="mode", access=py_trees.common.Access.WRITE)
        self.blackboard.current_power = None
        self.future = None
        self.request = Trigger.Request()
        self.state_request = GetRobotMode.Request()
    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
        self.poweron_client = self.node.create_client(Trigger, '/dashboard_client/brake_release')
        self.poweroff_client = self.node.create_client(Trigger, '/dashboard_client/power_off')
        self.state_client = self.node.create_client(GetRobotMode, '/dashboard_client/get_robot_mode')
        self.play_client = self.node.create_client(Trigger, '/dashboard_client/play')
        self.program_state_client = self.node.create_client(GetProgramState, '/dashboard_client/program_state')
        self.mode_client  = self.node.create_client(rcl_srvs.GetParameters, '/palletization_program_manager/get_parameters')

        # Wait for the service to be available
        while not (self.poweron_client.wait_for_service(timeout_sec=1.0) and self.poweroff_client.wait_for_service(timeout_sec=1.0) and self.state_client.wait_for_service(timeout_sec=1.0)):
            self.node.get_logger().info('Service not available, waiting...')

    def update(self):
        # When Robot first started
        if self.blackboard.current_power == None:
            self.node.get_logger().info("Initializing UR Robot...")
            if self.future == None:
                self.future = self.state_client.call_async(self.state_request)
            elif self.future.done():
                response = self.future.result()
                if response.robot_mode.mode == 7:
                    self.node.get_logger().info("Robot current status Power ON")
                    setattr(self.blackboard, "current_power", True)
                else:
                    self.node.get_logger().info("Robot current status Power OFF")
                    setattr(self.blackboard, "current_power", False)
                self.future = None
            return py_trees.common.Status.FAILURE
        # Robot initial state is ON set robot mode
        elif not self.blackboard.exists("mode"):
            self.node.get_logger().info("Fetching robot mode")
            if self.future == None:
                request = rcl_srvs.GetParameters.Request()
                request.names = ['mode']
                self.future = self.mode_client.call_async(request) 
            elif self.future.done():
                response = self.future.result()
                for param, value in zip(['mode'], response.values):
                    setattr(self.blackboard, "mode", value.string_value)
                self.future = None
            return py_trees.common.Status.FAILURE
        # State change
        elif self.blackboard.current_power != self.blackboard.power:
            # Robot ON
            if self.blackboard.power:
                self.poweron_client.call_async(self.request)
                if self.future == None:
                    self.future = self.state_client.call_async(self.state_request)
                    return py_trees.common.Status.FAILURE
                elif self.future.done():
                    response = self.future.result()
                    if response.robot_mode.mode == 7:
                        self.play_client.call_async(self.request)
                        self.future = None
                        setattr(self.blackboard, "current_power", True)
                        self.node.get_logger().info("UR Robot has turned ON")
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.future = self.state_client.call_async(self.state_request)
                        return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.FAILURE

            # Robot OFF
            else:
                self.poweroff_client.call_async(self.request)
                setattr(self.blackboard, "current_power", False)
                self.node.get_logger().info("UR Robot has turned OFF")
                return py_trees.common.Status.SUCCESS
        # No state change
        else:
            return py_trees.common.Status.SUCCESS
  
class CheckParam(py_trees.behaviour.Behaviour):
    """
    Check condition whether it matches the key
    Input:
        name: Node name
        keyname: Keyname from blackboard (string)
        condition: Value wanted to check
    """
    def __init__(self, name, keyname, condition):
        super(CheckParam, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=keyname, access=py_trees.common.Access.READ)
        self.keyname = keyname
        self.condition = condition
    def update(self):
        value = getattr(self.blackboard, self.keyname, None)
        if value == self.condition:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
class ParamToBlackboard(py_trees.behaviour.Behaviour):
    """
    Update ROS2 Param and store on Blackboard
    Input:
        name: Node name
        param_name: Parameter name (string)
        param_type: Parameter type (bool / int / float / string) //Might not contain all of the param type
    """
    def __init__(self, name, param_name, param_type):
        super(ParamToBlackboard, self).__init__(name=name)
        self.future = None  # To hold the service call future
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.param_name = param_name
        self.param_type = param_type

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
        # Create service client
        self.client = self.node.create_client(rcl_srvs.GetParameters, '/palletization_program_manager/get_parameters')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

    def update(self):
        # If the service call is not already in progress, make the request
        if self.future is None:
            self.node.get_logger().info(f"Requesting parameter {self.param_name}")
            request = rcl_srvs.GetParameters.Request()
            request.names = [self.param_name]
            self.future = self.client.call_async(request)  # Make the service call asynchronously

        # Check if the future has completed
        if self.future.done():
            # If service call succeeded
            try:
                response = self.future.result()
                # Extract the bool_value for the 'power' parameter
                for param, value in zip([self.param_name], response.values):
                    if self.param_type == 'bool':
                        param_value = value.bool_value
                    elif self.param_type == 'int':
                        param_value = value.integer_value
                    elif self.param_type == 'float':
                        param_value = value.double_value
                    elif self.param_type == 'string':
                        param_value = value.string_value
                    else:
                        raise ValueError(f"Unsupported parameter type: {self.param_type}")
                    
                    # Log the retrieved value
                    self.node.get_logger().info(f"Retrieved parameter {param}: {param_value}")
                    self.blackboard.register_key(key=self.param_name, access=py_trees.common.Access.WRITE)
                    setattr(self.blackboard, self.param_name, param_value)
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {str(e)}")
                return py_trees.common.Status.FAILURE

            # Reset the future to None to allow future requests
            self.future = None

            # Return SUCCESS once the service has been successfully called
            return py_trees.common.Status.SUCCESS

        # If the service call is still in progress, return RUNNING to keep the behavior active
        return py_trees.common.Status.RUNNING

class CheckPosition(py_trees.behaviour.Behaviour):
    """
    Check Robot position
    Input:
        name: Node name
    """
    def __init__(self, name):
        super(CheckPosition, self).__init__(name=name)

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
    def update(self):
        self.node.get_logger().info("Check Robot Position")

        return py_trees.common.Status.SUCCESS
    
class MovePosition(py_trees.behaviour.Behaviour):
    """
    Move Robot position
    Input:
        name: Node name
    """
    def __init__(self, name):
        super(MovePosition, self).__init__(name=name)

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
    def update(self):
        self.node.get_logger().info("Move Robot Position")
        return py_trees.common.Status.SUCCESS

class GetPose(py_trees.behaviour.Behaviour):
    """
    Get box position and store it in blackboard
    Input:
        name: Node name
    """
    def __init__(self, name):
        super(GetPose, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="box_pose", access=py_trees.common.Access.WRITE)
        self.blackboard.box_pose = [0, 0, 0, 0, 0, 0]
        self.box = None

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        self.box_client = self.node.create_client(GetPoses, '/get_pose')

        # Wait for the service to be available
        while not self.box_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Palletization vision server not available, waiting...')

    def update(self):

        if self.box is None:

            request = GetPoses.Request()
            self.box = self.box_client.call_async(request)
            return py_trees.common.Status.FAILURE

        elif self.box.done():
            response = self.box.result()
            if response.success:
                # Get pose successful
                self.blackboard.box_pose = [response.optimal_target.x, 
                                            response.optimal_target.y, 
                                            response.optimal_target.z,
                                            response.optimal_target.rx,
                                            response.optimal_target.ry,
                                            response.optimal_target.rz]
                self.node.get_logger().info(f"Box detected at: {self.blackboard.box_pose}")
                self.box = None
                return py_trees.common.Status.SUCCESS
            else:
                # No box pose or failed to transform coordinates
                self.node.get_logger().info("Box not detected or transformed.")
                return py_trees.common.Status.FAILURE
        # Get pose in progress
        else:
            return py_trees.common.Status.FAILURE

class PickNPlace(py_trees.behaviour.Behaviour):
    """
    Pick and place algorithm
    Input:
        name: Node name
    """
    def __init__(self, name):
        super(PickNPlace, self).__init__(name=name)

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            # Assuming the node is passed as a keyword argument
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        
    def update(self):
        self.node.get_logger().info("Pick and place in progress")
        time.sleep(5)
        self.node.get_logger().info("Pick and place completed")
        return py_trees.common.Status.SUCCESS
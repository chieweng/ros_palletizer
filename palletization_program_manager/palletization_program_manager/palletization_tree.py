import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
from . import behaviours
import rclpy.executors

def create_palletization_root() -> py_trees.behaviour.Behaviour:
    
    # Root of the tree
    root = py_trees.composites.Sequence(name="Update Power", memory=False)

    # Update power behaviours
    update_power = behaviours.ParamToBlackboard(name="Update Power", param_name='power', param_type='bool')

    # Selector to run whether ON or OFF sequence
    check_power = py_trees.composites.Selector(name="Check Power", memory=False)
    
    # Run sequence depending robot is ON or OFF
    on_sequence = py_trees.composites.Sequence(name="ON Sequence", memory=False)
    off_sequence = py_trees.composites.Sequence(name="OFF Sequence", memory=False)

    # ON sequence behaviours
    power_on = behaviours.CheckParam(name="Power ON ?", keyname="power", condition=True)
    robot_on = behaviours.URPower(name="RobotON")
    mode_selector = py_trees.composites.Selector(name="Robot Mode", memory=False)
    check_palletizing = behaviours.CheckParam(name="Palletizer Mode", keyname="mode", condition="palletizing")
    check_depalletizing = behaviours.CheckParam(name="Depalletizing Mode", keyname="mode", condition="depalletizing")

    # Robot Mode Sequence
    palletizing_sequence = py_trees.composites.Sequence(name="Palletizing Sequence", memory=False)
    depalletizing_sequence = py_trees.composites.Sequence(name="Depalletizing Sequence", memory=False)

    # OFF sequence behaviours
    power_off = behaviours.CheckParam(name="Power OFF ?", keyname="power", condition=False)
    robot_off = behaviours.URPower(name="RobotOFF")
    update_mode = behaviours.ParamToBlackboard(name="Update Mode", param_name='mode', param_type='string')

    # Depalletizing sequence behaviours
    position = py_trees.composites.Selector(name="Position ?", memory=False)
    pallet_position = behaviours.CheckPosition(name="Pallet position ?")

    is_home = py_trees.composites.Sequence(name="Is home position ?", memory=False)
    home_position = behaviours.CheckPosition(name="Home position ?")
    move_pallet_1 = behaviours.MovePosition(name="Move pallet position")

    not_home = py_trees.composites.Sequence(name="Not home position", memory=False)
    move_home_2 = behaviours.MovePosition(name="Move home position")
    move_pallet_2 = behaviours.MovePosition(name="Move pallet position")

    get_pose = behaviours.GetPose(name="Get Pose")

    pick_and_place = behaviours.PickNPlace(name="Pick And Place")

    root.add_children([update_power, check_power])
    check_power.add_children([on_sequence, off_sequence])
    on_sequence.add_children([power_on, robot_on, mode_selector])
    off_sequence.add_children([power_off, robot_off, update_mode])
    mode_selector.add_children([palletizing_sequence, depalletizing_sequence])
    palletizing_sequence.add_children([check_palletizing])
    depalletizing_sequence.add_children([check_depalletizing, position, get_pose, pick_and_place])
    position.add_children([pallet_position, is_home, not_home])
    is_home.add_children([home_position, move_pallet_1])
    not_home.add_children([move_home_2, move_pallet_2])

    return root

def main():
    """
    Entry point for palletization tree
    """
    rclpy.init(args=None)
    root = create_palletization_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "Failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

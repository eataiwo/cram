"""
The Quadraplex T-3000 Computer (also simply known as the Computer or Computress)
is Dexter's computer that oversees the running of the lab and has a personality
of its own.
"""

import rospy

# Based on https://github.com/jward0/farscope_group_project/blob/main/catkin_ws/src/ur10_picking/src/scripts/pipeline.py


class State:
    """
    Define statemachine class
    Each state will be initiated using this class.
    The class includes state name, on_event, next_state
    :param: pipeline_core: pipeline_core class containing all pipeline data needed for states
    :param: event: event causing change to the state
    :param: state_status: binary status number to used to update state machine status
    """

    def __init__(self):
        print("Initiating state:", str(self))
        self.state_status = 0b00000000000000

    def run(self, pipeline_core):
        assert 0, "run not implemented"

    def on_event(self, event):
        assert 0, "on_event not implemented"

    def next_state(self, state_complete):
        assert 0, "next state not implemented"
# TODO: Figure out the binary status word scheme for this state machine/supervisor

# TODO: Update the init state
class Initialise(State):
    """
    Class definition for the initialisation state
        Check nodes are on and topics are publishing
        Prioritisation - list of lists output
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Initialising the system")

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Initialisation complete")
            return 0b10010000000000  # Move to Calibration
        else:
            return 0b00000000000000  # Stay in Initialise

# TODO: Update calibration state
class Calibrate(State):
    """
    Class definition for the calibration state
        Output - binary status of calibration = 1
        Output - Home position - hard coded - move to home
        Output - Bin position - xyz
        Output - Centroids of each shelf - list or dict of coordinates / Pose for shelf home
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """
        print("Beginning calibration process")

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Calibration complete. State complete.")
            return 0b10011000000000  # Move to FindShelf
        else:
            return 0b10010000000000  # Stay in Calibration


class StateTemplate(State):
    """
    Class definition for the FindShelf state
        Read item from the prioritised list
        Identify shelf reference
        Identify retrieval mechanism
        Move UR10 to shelf centre and conform position has been reached
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """

        state_complete = True
        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Shelf found. State complete.")
            return 0b01011000000000  # Move to AssessShelf
        else:
            return 0b10011000000000  # Stay in FindShelf


class WorkOrderManagement(State):
    """
    Update work order
    """

    def run(self, pipeline_core):
        """
        :param pipeline_core: PipelineCore object
        :return: integer ID of next state
        """

        return self.next_state(state_complete)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Work order updated. State complete.")
            return 0b10011000000000  # Back to FindShelf
        else:
            return 5  # Complete


class SpinState(State):

    def run(self, pipeline_core):

        return self.next_state(False)

    def on_event(self, event):
        print("No on-event function for this state")

    def next_state(self, state_complete):

        if state_complete:
            print("Finished spinning")
            return 5
        else:
            return 999  # Stay in SpinState


class StateSupervisor:
    """
    Governs transitions between states and acts as scope for all state objects
    """

    def __init__(self):

        self.status = 0b00000000000000
        self.state_initialise = Initialise()
        self.state_calibrate = Calibrate()
        self.state_find_shelf = FindShelf()
        self.state_assess_shelf = AssessShelf()
        self.state_do_grip = DoGrip()
        self.state_work_order_management = WorkOrderManagement()
        self.state_spin = SpinState()

        print("State machine started. Current status:", self.status)

    def run(self, pipeline_core):
        """
        Monitors status and runs relevant states
        :param pipeline_core: PipelineCore object
        :return: None
        """

        if self.status == 0b00000000000000:
            self.status = self.state_initialise.run(pipeline_core)
        if self.status in (0b10010000000000, 0b01010000000000):
            self.status = self.state_calibrate.run(pipeline_core)
        if self.status in (0b10011000000000, 0b10011000000000):
            self.status = self.state_find_shelf.run(pipeline_core)
        if self.status in (0b01011000000000, 0b01111110110000, 0b01011111010000):
            self.status = self.state_assess_shelf.run(pipeline_core)
        if self.status == 0b01011110000000:
            self.status = self.state_do_grip.run(pipeline_core)
        if self.status in (0b01011110101010, 0b01011100000000):
            self.status = self.state_work_order_management.run(pipeline_core)
        else:
            print("End of pipeline")

    def report_status(self):
        print("Current state: ", self.status)

class PipelineCore:
    """
    Handles all persistent data and ROS interfaces
    """

    def __init__(self):
        # Initiate Pipeline Node:
        rospy.init_node("pipeline", anonymous=False)
        self.rate = rospy.Rate(10)
        self.data = None

        # Imported from prioritise.py
        self.all_items = all_items
        self.item_profile = item_profiles
        self.bin_profiles = bin_profiles
        self.bin_contents = None  # Might not need to be shared to all states
        self.work_order = None  # Might not need to be shared to all states.
        self.work_order_prioritised = None
        self.target_shelf = None
        self.skipped_items = []
        self.picked_items = []
        self.dropped_items = []

        # Initiate topics and services:
        # UR10 control:
        self.current_pose = None
        self.pose_publisher = TopicWriter('/pipeline/next_cartesian_pose', PoseMessage)
        self.trajectory_publisher = TopicWriter('/pipeline/cartesian_trajectory', PoseArray)
        self.pose_feedback_subscriber = TopicReader('/moveit_interface/cartesian_pose_feedback', PoseMessage)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Vision topics and services:
        self.get_object_centroid = ServiceCaller("detect_object", detect_object)

        # Vacuum control (commented out for UR10 testing purposes):
        self.vacuumonoff = ServiceCaller("vacuum_switch", vacuum_switch)
        self.vacuumcalibration = ServiceCaller("vacuum_calibration", vacuum_calibration)
        self.vacuumsucking = TopicReader("vacuum_status", Bool)

        # Scratch space for pose logging between states:
        self.stored_pose = Pose()

    def callbacktest(data):
        self.data = data.data


def run_pipeline():
    """
    Instantiates pipeline_core and state_machine objects, and then just spins the state machine
    :return: None
    """
    pipeline_core = PipelineCore()
    print("Waiting for moveit interface to start...")
    while not pipeline_core.pose_feedback_subscriber.var:
        rospy.sleep(0.1)
    print("Moveit interface started. Starting state machine...")
    state_machine = StateSupervisor()
    while True:
        state_machine.run(pipeline_core)
        state_machine.report_status()
        rospy.sleep(5.0)


if __name__ == "__main__":
    run_pipeline()

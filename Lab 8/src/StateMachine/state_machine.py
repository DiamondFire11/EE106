import rospy

from pid_controller import PIDController


class StateMachine:
    def __init__(self):
        self.state = 'FORWARD'
        self.has_turned = False
        self.turn_count = 0
        self.forward_count = 0
        self.pid = PIDController(0.7, 0, 0)
        self.pid.update_set_point(0.15)

    def action(self, min_dist):
        if self.state == 'MOVE':
            return 0.1, self.pid.get_control_signal(min_dist)

        if self.state == 'RIGHT_HARD':
            return 0, -0.28

    def transition(self, min_dist, front_dist):
        # If too close to front, turn right hard
        if front_dist < 0.50:
            self.state = 'RIGHT_HARD'
            return
        else:
            self.state = 'MOVE'
            return

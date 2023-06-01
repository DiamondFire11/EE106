class StateMachine:
    def __init__(self):
        self.state = 'FORWARD'

    def action(self):
        if self.state == 'FORWARD':
            return 0.35, 0

        if self.state == 'TURN_RIGHT':
            return 0.35, -0.08

        if self.state == 'TURN_LEFT':
            return 0.35, 0.08

    def transition(self, min_dist):
        # If too close to the left wall but not at end
        if min_dist < 0.10:
            self.state = 'TURN_RIGHT'
            return

        # If too close to the right wall but not at end
        if min_dist > 0.20:
            self.state = 'TURN_LEFT'
            return

        # If not too close or too far, and not close to end
        if 0.10 <= min_dist <= 0.20:
            self.state = 'FORWARD'
            return

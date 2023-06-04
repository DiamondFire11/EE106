class StateMachine:
    def __init__(self):
        self.state = "FORWARD"
        self.left_side = "FREE"
        self.front_side = "FREE"

    def action(self):
        if self.state == "FORWARD":
            return 0.20, 0

        if self.state == "RIGHT":
            return 0.20, -0.0525

        if self.state == "LEFT":
            return 0.20, 0.0525

        if self.state == "LEFT_HARD":
            return 0.155, 0.28

        if self.state == "RIGHT_HARD":
            return 0.02, -0.28

    def set_front_state(self, front_dist):
        # Set front_dist flags
        if front_dist < 0.35:
            self.front_side = "CRITICAL"
            return

        if 0.35 <= front_dist <= 0.50:
            self.front_side = "OCCUPIED"
            return

        if front_dist > 0.50:
            self.front_side = "FREE"
            return

    def set_left_state(self, left_dist):
        # Set left_dist flags
        if left_dist < 0.15:
            self.left_side = "CRITICAL"
            return

        if 0.15 <= left_dist <= 0.23:
            self.left_side = "OCCUPIED"
            return

        if left_dist > 0.23:
            if left_dist > 0.60:
                self.left_side = "LEFT_HARD"
                return

            self.left_side = "FREE"
            return

    def transition(self, min_dist, front_dist):
        self.set_front_state(front_dist)
        self.set_left_state(min_dist)

        if self.state == "LEFT":
            if self.front_side == "CRITICAL":
                self.state = "RIGHT_HARD"
                return

            if self.left_side == "LEFT_HARD":
                self.state = "LEFT_HARD"
                return

            if self.left_side == "OCCUPIED":
                self.state = "FORWARD"
                return

            if self.front_side == "OCCUPIED":
                self.state = "RIGHT"
                return

            if self.left_side == "FREE" and self.front_side == "FREE":
                self.state = "LEFT"
                return

        if self.state == "FORWARD":
            if self.front_side == "CRITICAL":
                self.state = "RIGHT_HARD"
                return

            if self.left_side == "LEFT_HARD":
                self.state = "LEFT_HARD"
                return

            if self.left_side == "FREE":
                self.state = "LEFT"
                return

            if self.left_side == "CRITICAL" or self.front_side == "OCCUPIED" or self.front_side == "CRITICAL":
                self.state = "RIGHT"
                return

            if self.left_side == "OCCUPIED" and self.front_side == "FREE":
                self.state = "FORWARD"
                return

        if self.state == "RIGHT":
            if self.front_side == "CRITICAL":
                self.state = "RIGHT_HARD"
                return

            if self.left_side == "LEFT_HARD" and front_dist > 1:
                self.state = "LEFT_HARD"
                return

            if self.left_side == "CRITICAL":
                self.state = "RIGHT"
                return

            if self.left_side == "LEFT_HARD" and front_dist < 1:
                self.state = "RIGHT"
                return

            if self.left_side == "FREE" and self.front_side == "OCCUPIED":
                self.state = "RIGHT"
                return

            if self.front_side == "FREE":
                self.state = "FORWARD"
                return

        if self.state == "LEFT_HARD":
            if self.front_side == "CRITICAL":
                self.state = "RIGHT_HARD"
                return

            if self.left_side == "LEFT_HARD" and front_dist > 1:
                self.state = "LEFT_HARD"

            if self.left_side == "LEFT_HARD" and front_dist < 1:
                self.state = "RIGHT"
                return

            if self.left_side != "LEFT_HARD":
                self.state = "LEFT"
                return

        if self.state == "RIGHT_HARD":
            if self.front_side == "FREE":
                self.state = "FORWARD"

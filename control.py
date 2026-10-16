class PDController:
    """PD depth controller for the UUV."""
    
    def __init__(self, Kp: float = 0.15, Kd: float = 0.6):
        """
        Initialise the controller with gains and state memory. 

        Parameters:
            Kp (float): Proportional gain (default: 0.15)
            Kd (float): Derivative gain (default: 0.6)
        """
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0.0 # e(-1) = 0

    def control(self, reference: float, current_depth: float) -> float:
        """
        Calculate the control action based on the reference depth and current depth.

        Parameters:
            reference (float): Desired depth
            current_depth (float): Current depth

        Returns:
            float: Control action
        """
        # Calculate the error and derivative
        error = reference - current_depth
        derivative = error - self.prev_error
        # Update the error for the next discrete step
        self.prev_error = error
        # Calculate the action using the PD control prescribed in document
        return (self.Kp * error) + (self.Kd * derivative)
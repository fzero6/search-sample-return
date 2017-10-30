import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # offset angle for steering the rover.
    theta = 13
    # navigable pixel threshold for stopping.
    stop_thresh = 200

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            # if the rover has navigable terrain, but it is not moving forward reverse the rover

            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle

                # the rover is stuck then reverse
                Rover.debug = 'speeding up'

                # the velocity is below the maximum keep accelerating
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                    # Rover.steer = 15
                Rover.brake = 0

                # calculate the mean angle of the navigable pixels
                mean_angle = np.mean(Rover.nav_angles * 180 / np.pi)
                nav_angle = mean_angle
                # steer the rover to the theta degree counter clockwise from the mean angle.
                # clip the values between the -15 and 15 steering range.
                # this forces the rover to corrective steer to the left enabling left wall follow.
                Rover.steer = np.clip(nav_angle + theta, -15, 15)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            if len(Rover.nav_dists) < stop_thresh:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.debug = 'stopping'

        # if the rover mode is stop, then execute the following code
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.debug = 'braking'
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # if the rover has navigation but is not moving and trying to throttle

                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    Rover.mode = 'stuck'
                    Rover.debug = 'rover turning'

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:

                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0

                    # calculate the mean angle of the navigable pixels in the rovers camera image
                    mean_angle = np.mean(Rover.nav_angles * 180 / np.pi)
                    nav_angle = mean_angle
                    # steer the rover to theta degrees counter clodwise of the mean angle
                    # this forces the rover to corrective steer to the left, forcing left wall follow.
                    Rover.steer = np.clip(nav_angle + theta, -15, 15)

                    Rover.mode = 'forward'
                    Rover.debug = 'moving forward'

        elif Rover.mode == 'stuck':
            Rover.steer = -15
            Rover.debug = 'stuck & turning'
            if len(Rover.nav_angles) >= Rover.go_forward:
                Rover.steer = 0
                Rover.mode = 'forward'
                Rover.debug = 'unstuck'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    elif Rover.nav_angles == 0:
        Rover.brake = Rover.brake_set
        Rover.brake = 0
        Rover.throttle = -1 * Rover.throttle_set
        Rover.steer = -15

    # If in a state where want to pickup a rock send pickup command


    return Rover





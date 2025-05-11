Core Rules:

    Freezing Criteria (When robots stop moving):

        A robot freezes when it reaches a distance ≥ lower_threshold (7 units) from at least one other robot

        Exception: The last active robot won't freeze if it would leave no active robots

    Movement Rules for Active Robots:

        Regular movement (when ≥2 robots are active):

            Move forward at constant speed

            If movement would either:

                Increase average RSSI (signal strength) with other robots OR

                Exceed upper_threshold (8 units) with any robot

            Then: Undo movement and turn 45° instead

        Last active robot behavior (when only 1 robot remains active):

            Calculates desired movement based on distances to frozen robots:

                If too close (<7 units) to a frozen robot: moves away from it

                If too far (>8 units) from a frozen robot: moves toward it

                If in ideal zone (7-8 units): stays put

            Turns toward calculated ideal direction before moving

    Termination Condition:

        All robots freeze when:

            The last active robot reaches distances between 7-8 units from all frozen robots

Visual Thresholds:

    Red/Blue/Green dashed circles (7 units): Show freeze zones

    Black dotted circles (8 units): Show maximum allowed distances

Key Concepts:

    RSSI (Radio Signal Strength Indicator):

        Used as a proxy for distance (higher RSSI = closer)

        Robots try to maintain medium signal strength

    Emergent Behavior:

        The system self-organizes until all robots are:

            At least 7 units apart (preventing crowding)

            No more than 8 units apart (maintaining connectivity)

    Three-Phase Progression:

        Phase 1: Random walking until first pair freezes

        Phase 2: Remaining active robots adjust to frozen ones

        Phase 3: Final robot navigates to ideal positions

This creates a distributed algorithm where robots only need to know:

    Their own state

    Current distances to others

    No global coordination required



Turning Criteria:

    Regular Movement (When Multiple Robots Are Active):

        Turn Condition:

            If moving forward would either:

                Increase average RSSI (signal strength) with other robots (meaning they're getting too close), or

                Exceed the upper_threshold (8 units) from any other robot (meaning they're getting too far).

            Action Taken:

                The robot undoes its forward movement (reverts to previous position).

                It then turns by turn_angle (45°) in a random direction (left or right).

    Last Active Robot (When Only One Robot Remains Active):

        Turn Condition:

            If the robot is not facing the calculated "ideal direction" needed to adjust distances to frozen robots.

            The "ideal direction" is computed based on:

                Moving away from robots that are too close (<7 units).

                Moving toward robots that are too far (>8 units).

        Action Taken:

            The robot turns toward the computed direction (in 45° increments) until it’s aligned.

            If already aligned, it moves forward instead of turning.

    Randomness in Turning (When No Clear Better Option):

        If a robot is stuck (e.g., moving forward doesn't improve RSSI but no clear direction is better), it turns randomly by 45° to explore new paths.

Key Mechanics Behind Turning:

    RSSI-Based Decision:

        Robots prefer moving in directions that maintain moderate signal strength (neither too weak nor too strong).

    Threshold Checks:

        The lower_threshold (7) and upper_threshold (8) define a "Goldilocks zone" where robots try to stay.

    Obstacle Avoidance:

        If moving forward worsens positioning, the robot turns to find a better path.

Example Scenarios Where a Robot Turns:

    Approaching Too Close to Another Robot:

        Robot A moves toward Robot B, reducing distance below 7.

        Robot A detects rising RSSI (too strong) → turns away.

    Drifting Too Far from Another Robot:

        Robot A moves away from Robot B, exceeding 8 units.

        Robot A detects falling RSSI (too weak) → turns back toward Robot B.

    Last Robot Adjusting Position:

        The final active robot must position itself between 7–8 units from all frozen robots.

        It calculates a weighted direction and turns until aligned, then moves.

Summary Table:
Scenario	                                Condition for Turn	                        Turn Action

Multiple active robots	                    Movement worsens RSSI or exceeds max dist	Undo move + turn 45° randomly

Last active robot	                        Not facing ideal direction	                Turn 45° toward calculated heading

No improvement in movement	                Stuck in a loop	                            Random 45° turn
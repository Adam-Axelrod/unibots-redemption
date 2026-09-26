## What does this change?

<!-- One or two sentences. What can the robot do now that it could not before? -->


## How did you test it?

<!-- Delete what does not apply. -->

- [ ] Ran it in the simulator (`docker compose up`, watched http://localhost:8080)
- [ ] Ran it on the real robot
- [ ] Not tested yet — this is a draft and I would like a look before I go further

**World(s) tested in:** <!-- empty_room / furnished_room / l_shaped_room -->


## Checklist

- [ ] New nodes are registered in the package's `setup.py` under `console_scripts`
- [ ] The node publishes `/cmd_vel` on a **timer**, not once (see CONTRIBUTING.md)
- [ ] The node publishes a zero `Twist` on shutdown
- [ ] No `ws/build/`, `ws/install/` or `ws/log/` files in the diff
- [ ] Tunable numbers are `declare_parameter`, not constants in the code


## Anything you want a second opinion on?

<!-- Optional. "Is this the right way to handle X?" is a perfectly good PR comment. -->

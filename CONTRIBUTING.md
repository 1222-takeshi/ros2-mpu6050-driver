# Contributing

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

## Validation

Use a ROS 2 Humble workspace and run all registered tests, including lint:

```sh
colcon build --packages-select imu_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select imu_driver --event-handlers console_cohesion+
colcon test-result --verbose
git diff --check
```

Hardware timing and robot behavior require separate validation on the target system.

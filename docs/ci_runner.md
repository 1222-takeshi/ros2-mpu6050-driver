# GitHub-hosted CI

This public repository uses the standard GitHub-hosted `ubuntu-22.04-arm`
runner. Standard hosted runners are free for public repositories, including
ARM64; see the [GitHub runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners).
DGX Spark remains available for the separate inverted-pendulum repository.

Each job uses a fresh GitHub-hosted VM and a digest-pinned Jammy container with
ROS 2 Humble. Pushes to `main` and pull requests, including fork pull requests,
run all registered driver tests and lints. Fork workflow execution remains
subject to GitHub's repository approval policy. The job has read-only contents
permission and does not use `pull_request_target`.

The colcon workspace is created outside the checkout in a unique `RUNNER_TEMP`
directory. Test results are uploaded even on failure, then that job's temporary
workspace is removed. Check PR and post-merge `main` runs for the ARM64 runner,
all test/lint results, artifact upload, and workspace cleanup.

After successful migration checks, retire the driver-specific DGX runner
`mpu6050-driver-dgx-spark` and its user service. The inverted-pendulum runner is
independent and remains enabled.

Mock tests, production-backend file-descriptor tests, and ARM64 builds do not
measure Raspberry Pi I2C timing or qualify balancing. Hardware timing remains
tracked in #41; integration requirements and results remain in #61–#63.

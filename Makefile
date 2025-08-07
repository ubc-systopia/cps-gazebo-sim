.PHONY: build demo test clean

# Build the workspace
build:
	colcon build --symlink-install
	@echo "Don't forget to run: source install/setup.bash"

# Launch the demo
demo:
	@source /opt/ros/humble/setup.bash && \
	source install/setup.bash && \
	ros2 launch two_arm_moveit_config demo.launch.py

# Run integration tests
test:
	@source /opt/ros/humble/setup.bash && \
	source install/setup.bash && \
	pytest test/integration

# Clean build artifacts
clean:
	rm -rf build install log

# Help
help:
	@echo "Available commands:"
	@echo "  make build  - Build the ROS workspace"
	@echo "  make demo   - Launch the multi-arm demo"
	@echo "  make test   - Run integration tests"
	@echo "  make clean  - Clean build artifacts"
all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build  - build images"
	# @echo "   1. make pull   - pull images"
	# @echo "   1. make clean  - remove images"
	@echo ""

build:
	@docker build --tag=ruffsl/ros_armhf:kinetic kinetic-ros-core/
	@docker build --tag=ruffsl/ros_armhf:kinetic_cartographer_ros cartographer_ros/

pull:
	# @docker pull ruffsl/ros_armhf:kinetic

clean:
	# @docker rmi -f ruffsl/ros_armhf:kinetic

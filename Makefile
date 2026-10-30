# Warehouse AMR simulation stack.
#
# Two classes of target, and the distinction matters when reading the README:
#
#   OFFLINE  need only Python, numpy and scipy. Run anywhere, in seconds, and
#            produce every number the README quotes about segmentation and
#            scan-matching accuracy.
#              make test  make segmentation  make scan-odometry  make world
#
#   SIMULATED  need ROS 2 Jazzy and gz-sim Harmonic, so in practice the Docker image.
#              Produce the end-to-end localisation and recovery numbers.
#                make demo  make map  make eval  make faults
#
# `make check` runs everything in the first class and is what CI does.

SHELL := /bin/bash
PYTHON ?= python3
RESULTS ?= results
SCANS ?= 25

.DEFAULT_GOAL := help

# ---------------------------------------------------------------------------
# Offline
# ---------------------------------------------------------------------------

.PHONY: help
help:  ## Show this help
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'

.PHONY: test
test:  ## Run the unit tests (no ROS required)
	$(PYTHON) -m pytest tests/ -q

.PHONY: lint
lint:  ## Check formatting and obvious errors
	$(PYTHON) -m flake8 src/ analysis/ tests/ simulation/

.PHONY: world
world:  ## Regenerate the Gazebo world, zone definitions and reference map
	$(PYTHON) simulation/generate_world.py

.PHONY: segmentation
segmentation:  ## Measure segmentation accuracy and latency (no ROS required)
	$(PYTHON) analysis/segmentation_metrics.py --scans $(SCANS) \
		--output $(RESULTS)/segmentation.json

.PHONY: scan-odometry
scan-odometry:  ## Measure LiDAR odometry against ground truth (no ROS required)
	$(PYTHON) analysis/scan_odometry_benchmark.py \
		--output $(RESULTS)/scan_odometry.json --csv-dir $(RESULTS)/scan_odometry

.PHONY: check
check: test segmentation scan-odometry  ## Everything that runs without ROS
	@echo
	@echo "All offline checks passed. Results in $(RESULTS)/."

# ---------------------------------------------------------------------------
# Simulated. These require a sourced ROS 2 Jazzy workspace.
# ---------------------------------------------------------------------------

.PHONY: build
build:  ## colcon build the workspace
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: demo
demo:  ## Launch gz-sim with the full stack
	ros2 launch amr_bringup simulation.launch.py headless:=false use_rviz:=true

.PHONY: headless
headless:  ## Launch the stack with no GUI
	ros2 launch amr_bringup simulation.launch.py headless:=true

.PHONY: map
map:  ## Run Cartographer to build the warehouse map
	./scripts/run_mapping.sh

.PHONY: eval
eval:  ## Run the mission with logging, then report localisation accuracy
	./scripts/run_evaluation.sh $(RESULTS)

.PHONY: openloop
openloop:  ## Measure localisation with Nav2 out of the loop (the harness that isolates it)
	./scripts/run_openloop.sh $(RESULTS)/openloop

.PHONY: audit
audit:  ## Path length per pose source: which link in the chain lost the motion
	$(PYTHON) analysis/odometry_audit.py --results $(RESULTS)

.PHONY: integration
integration:  ## The two headless integration tests CI runs (launch_testing)
	./scripts/run_integration_tests.sh $(RESULTS)/integration

.PHONY: faults
faults:  ## Run the mission with induced failures, then report the recovery matrix
	./scripts/run_faults.sh $(RESULTS)

.PHONY: report
report:  ## Recompute every metric from existing logs in $(RESULTS)
	$(PYTHON) analysis/localisation_error.py \
		--estimate $(RESULTS)/amcl_pose.csv \
		--reference $(RESULTS)/ground_truth.csv \
		--output $(RESULTS)/localisation_error.json \
		--title "AMCL against Gazebo ground truth"
	$(PYTHON) analysis/localisation_error.py \
		--estimate $(RESULTS)/wheel_odom.csv \
		--reference $(RESULTS)/ground_truth.csv \
		--output $(RESULTS)/wheel_odom_error.json \
		--align \
		--title "Wheel odometry against Gazebo ground truth"
	$(PYTHON) analysis/recovery_matrix.py \
		--faults $(RESULTS)/faults.json \
		--mission $(RESULTS)/mission.json \
		--output $(RESULTS)/recovery_matrix.json

# ---------------------------------------------------------------------------
# Docker
# ---------------------------------------------------------------------------

.PHONY: docker-build
docker-build:  ## Build the container image
	docker compose -f docker/docker-compose.yml build

.PHONY: docker-demo
docker-demo:  ## Run the demo in Docker
	docker compose -f docker/docker-compose.yml up demo

.PHONY: docker-check
docker-check:  ## Run the offline checks in Docker
	docker compose -f docker/docker-compose.yml run --rm check

.PHONY: clean
clean:  ## Remove build and result artefacts
	rm -rf build install log $(RESULTS) .pytest_cache
	find . -name __pycache__ -type d -prune -exec rm -rf {} +

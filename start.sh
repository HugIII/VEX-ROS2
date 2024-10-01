#!/bin/bash

env UID=$(id -u $USER) GID=$(id -g $USER) docker compose run --rm ros2_galactic
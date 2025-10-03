#!/bin/bash
git pull
docker build -t mazerun:latest .
docker run --it --privileged mazerun:latest
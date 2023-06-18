#!/bin/bash
now=$(date +"%Y%m%d_%H%M%S")
now_date=$(date +"%Y%m%d_%H%M%S")

git add *
git commit -m "$now"
git remote add "$now_date" git@github.com:MinchangSung0223/RTIndy7_sim.git
git push "$now_date" main


#!/bin/bash
#
# Script to
# - lint files with pep8 linter
# - run tests
# - view coverage results in Google Chrome
# - upload coverage results to Coveralls
roscd pr2_pbd_gui; pep8 src/pr2_pbd_gui/*.py
roscd pr2_pbd_interaction; pep8 src/*.py nodes/*.py test/*.py
roscd pr2_pbd_speech_recognition; pep8 nodes/*.py scripts/*.py
roscd pr2_social_gaze; pep8 nodes/*.py
rostest pr2_pbd_interaction test_endtoend.test
google-chrome ~/.ros/htmlcov/index.html
coveralls --data_file ~/.ros/.coverage

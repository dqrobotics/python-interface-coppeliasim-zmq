#!/bin/bash
set -e

echo "Installing dqrobotics ..."
pip install dqrobotics --pre
echo "Installing this package ..."
pip install ../../

echo "Basic evaluation ..."
python basic_eval.py





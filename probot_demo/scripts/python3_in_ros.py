#!~/anaconda3/bin/env	python3
# coding=utf-8
import sys
path = sys.path
python2_path = [s for s in path if 'python2' in s]
python3_path = [s for s in path if s not in python2_path]
sys.path = python3_path + python2_path

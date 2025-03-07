#!/usr/bin/env python3
"""
Test runner script for the robot code.
"""
import os
import sys
import pytest

# Add the current directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

if __name__ == "__main__":
    sys.exit(pytest.main(["tests/"])) 
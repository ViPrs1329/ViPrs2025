# test_imports.py
# Simple test script to verify module imports

import rev
import wpimath

def main():
    print(dir(wpimath))
    print(hasattr(rev, "SparkMax"))  # Should return False if it doesn't exist
    print(hasattr(rev, "CANSparkMax"))  # Should return True if it exists

    print(hasattr(rev, "SparkBase"))  # Should return False
    print(hasattr(rev, "CANSparkBase"))  # Should return True

if __name__ == "__main__":
    main()
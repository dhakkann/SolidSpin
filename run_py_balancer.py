"""
Convenience launcher for the standalone Python CAD Balancer.
Usage:
    python run_py_balancer.py            # Instant single balance
    python run_py_balancer.py --pareto   # Explore Pareto trade-off curve
"""

import sys
from python_balancer.main import main

if __name__ == "__main__":
    main()

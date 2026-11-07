"""Direct entry point: python sample/riceswarm_ai/run_demo.py"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from riceswarm_ai.cli import main  # noqa: E402

if __name__ == '__main__':
    main()

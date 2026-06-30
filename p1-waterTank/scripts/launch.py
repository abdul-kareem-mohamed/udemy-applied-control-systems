import argparse
import logging
import os
import subprocess
import sys
from pathlib import Path

from logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


def main():
  parser = argparse.ArgumentParser(description="Water Tank Launcher")
  parser.add_argument('--backend', choices=['cpp', 'python'], default='python', help="Which controller backend to run")
  parser.add_argument('--cpp-exec', type=str, help="Absolute path to the C++ executable")
  args = parser.parse_args()
  
  base_dir = Path(__file__).resolve().parent
  
  if args.backend == 'cpp':
    if not args.cpp_exec or not Path(args.cpp_exec).exists():
      raise FileNotFoundError(f"Error: C++ executable not found at {args.cpp_exec}")
    controller_cmd = [args.cpp_exec]
  else:
    controller_cmd = [sys.executable, str(base_dir / "controller.py")]

  logger.debug(f"Base directory: {base_dir}")
  logger.debug(f"Exec directory: {args.cpp_exec}")
  
  processes = []  
  
  try:
    logger.info(f"Starting System with {args.backend.upper()} backend...")
    
    processes.append(subprocess.Popen(controller_cmd))
    processes.append(subprocess.Popen([sys.executable, os.path.join(base_dir, "tank_sim.py")]))
    processes.append(subprocess.Popen([sys.executable, os.path.join(base_dir, "visualizer.py")])) 

    for p in processes:
      p.wait()
    
  except KeyboardInterrupt:
    logger.info("\n🛑 Ctrl+C detected. Shutting down all sockets...")
  
  finally:
    logger.info("Cleaning up all processes...")
    for p in processes:
      p.terminate()
      p.wait()
    logger.info("All processes terminated sucessfully.")


if __name__ == "__main__":
    sys.exit(main())